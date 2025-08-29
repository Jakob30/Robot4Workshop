/*
 * motion.motor_control.c
 *  Created on: Jul 14, 2025
 *      Author: jakob
 */
#include "motor/motor_control.h"
#include "motor/motor_init.h"

//Globals
extern motor_t *motors[]; //To gain access to motor variables in interrupt service routine
extern TIM_HandleTypeDef htim9;


#define NUMBER_OF_MOTOR 5
#define ALPHA 0.25f

//uint16_t stallguard_result_g;
//float smoothed_result_g;
//float v_g;
//
//uint16_t negative_diff_counter_g;
//int consecutive_low_counter_g;

/*
 * Function Declaration
 */
int32_t toSteps(float degrees, motor_t* motor);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
void initMovementVars(motor_t * motor, motion_mode_t motion_mode);
void startMovement(motor_t * motor);
void startStatusChecks(motor_t * motor);
void checkOverheating(tmc2209_status_t status);
void checkStall(uint16_t stallguard_result, motor_t* motor);
void toggle_inverse_motor_direction(tmc2209_stepper_driver_t* stepper_driver);
//void initializeDefaults(motor_t * motor);

/*
 * Calculates the steps needed to rotate the amount stated in the variable degrees.
 */
int32_t toSteps(float degrees, motor_t* motor)
{
	int32_t steps;
	steps = ((200.0 * (float)(motor->microsteps)/360.0)*degrees) * motor->gear_ratio;
	return steps;
}

static inline void stopMotorMovement(motor_t * motor)
{
	HAL_TIM_OC_Stop_IT(&motor->motion.motor_control_timer, TIM_CHANNEL_1);
	motor->active_movement_flag = 0;
	initializeDefaults(motor);
}

static inline void trapezMove(motion_t* mt)
{
	if (mt->step >= 0 && mt->step < mt->acc_steps)
	{
		mt->v = sqrtf(2 * mt->ACC_MAX * (mt->step + 1));
		// motion.v = acc_ramp[motion.step]
	}
	else if (mt->const_steps != 0 && mt->step >= mt->acc_steps && mt->step < (mt->total_steps - mt->dec_steps))
		mt->v = mt->V_MAX;
	else if (mt->step >= (mt->total_steps - mt->dec_steps) && mt->step < mt->total_steps)
	{
		mt->v = sqrtf(2 * mt->DEC_MAX * (mt->total_steps - mt->step));
		// motion.v = acc_ramp[motion.total_steps - motion.step]
	}
}
/*
 *
 * Interrupt service routine for timer in output compare mode.
 * Timer counts until compare value is reached.
 * When value is reached, the GPIO toggles.
 * Every other motion.cycle, as motion.step only triggers on rising edge,
 * the velocity and the compare value is changed depending on current state of velocity ramp.
 *
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	int8_t index;
	motor_t* motor;

	//To know which timer and thus which motor caused the interrupt
	if (htim->Instance == motors[0]->motion.motor_control_timer.Instance){ index = 0; }
	else if (htim->Instance == motors[1]->motion.motor_control_timer.Instance){ index = 1; }
	else if (htim->Instance == motors[2]->motion.motor_control_timer.Instance){ index = 2; }
	else if (htim->Instance == motors[3]->motion.motor_control_timer.Instance){ index = 3; }
	else if (htim->Instance == motors[4]->motion.motor_control_timer.Instance){ index = 4; }

	motor = motors[index];
	motion_t* mt = &motor->motion;

	//Stop timer and movement if the robot reaches its destination
	if (mt->motion_mode == MOTION_TRAPEZ && mt->step >= mt->total_steps)
	{
		stopMotorMovement(motor);
		return;
	}

	if (mt->cycle % 2 == 0) //Change velocity only every other cycle because step only triggers on rising edge
	{
		switch(mt->motion_mode)
		{
		case MOTION_TRAPEZ:
			trapezMove(mt);
			break;
		case MOTION_HOME:
		case MOTION_GRIP:
			if (mt->step >= 0 && mt->step < mt->acc_steps)
				mt->v = sqrtf(2 * mt->ACC_MAX * (mt->step + 1));
			else
				mt->v = mt->V_MAX;

			break;
		}
		mt->step++;
	}

	mt->cycle++;

	HAL_GPIO_TogglePin(motor->gpio_ports.step, motor->gpio_pins.step);

	/*
	 * 	To reach the desired speed, we need to calculate the delay between two toggles.
	 *	v is in µsteps/s or just 1/s because one µstep is one period.
	 *	For one period, the duration is 1/v.
	 *	Between two toggles, it is 1/(2*v).
	 *	But this is not the answer since the time has to be converted into timer ticks.
	 *	The timer runs at 2 MHz so we need to divide our current period duration by 1 / 2000000 s or 0.5 µs.
	 *	-> delay in ticks = 1/(2*v)/0.0000005 = 2000000/(2*v)
	 */

	int32_t delay = 2000000 / (2 * motor->motion.v);
	//Add delay to current compare value in register
	int32_t total_delay = __HAL_TIM_GET_COMPARE(&mt->motor_control_timer, TIM_CHANNEL_1) + delay;
	__HAL_TIM_SET_COMPARE(&mt->motor_control_timer, TIM_CHANNEL_1, total_delay);
}

/*
 * Interrupt service routine for timer 9, which periodically invokes status checks.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	for(int i = 0; i < NUMBER_OF_MOTOR; i++)
	{
		if (htim->Instance == motors[i]->status_check_timer.Instance)
		{
			if (motors[i]->active_movement_flag)
				motors[i]->status_flag = 1;
			else
				HAL_TIM_Base_Stop_IT(&motors[i]->status_check_timer);

			break;
		}
	}
//	writeDisplay("HAHA");
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B1_Pin)  // Prüfen ob User Button
	{
		toggle_inverse_motor_direction(motors[4]->driver);
	}
}

/*
 * Initiates motor movement by starting the timer and calculating the steps
 */
void moveDegrees(float degrees, motor_t* motor)
{
	if (HAL_GPIO_ReadPin(motor->gpio_ports.mot_en, motor->gpio_pins.mot_en) == GPIO_PIN_SET)
		tmc2209_enable(motor->driver);

	motor->motion.total_steps = toSteps(degrees, motor); //Convert degrees to steps
	motor->motion.acc_steps = (motor->motion.V_MAX * motor->motion.V_MAX) / (2 * motor->motion.ACC_MAX); //Calculate total acceleration and deceleration steps
	motor->motion.dec_steps = (motor->motion.V_MAX * motor->motion.V_MAX) / (2 * motor->motion.DEC_MAX);
	motor->motion.const_steps = motor->motion.total_steps - (motor->motion.acc_steps + motor->motion.dec_steps);

	motion_mode_t motion_mode = MOTION_TRAPEZ;

	initMovementVars(motor, motion_mode);

	if (motor->motion.const_steps < 0)	//If acceleration steps + deceleration steps are bigger than total steps
	{
		motor->motion.acc_steps = motor->motion.total_steps / 2;
		motor->motion.dec_steps = motor->motion.total_steps / 2;
		motor->motion.const_steps = 0;
	}

	//Start timer in output compare with interrupt
	startMovement(motor);

	startStatusChecks(motor);
}

void grip()
{
	motor_t * motor5 = motors[4];

	disable_inverse_motor_direction(motor5->driver);
	moveDegrees(10000, motor5);
	while(motor5->active_movement_flag);

	enable_inverse_motor_direction(motor5->driver);

	if (HAL_GPIO_ReadPin(motor5->gpio_ports.mot_en, motor5->gpio_pins.mot_en) == GPIO_PIN_SET)
		tmc2209_enable(motor5->driver);

	motor5->motion.acc_steps = (motor5->motion.V_MAX * motor5->motion.V_MAX) / (2 * motor5->motion.ACC_MAX); //Calculate total acceleration and deceleration steps

	motion_mode_t motion_mode = MOTION_GRIP;
	initMovementVars(motor5, motion_mode);

	startMovement(motor5);
	startStatusChecks(motor5);
	while (motor5->stallguard.stall_flag == 0)
	{
		checkDriverStatus(motor5);
	}
}

void goHome()
{
	for (int i = 0; i < NUMBER_OF_MOTOR; i++)
	{
		motor_t * motor = motors[i];
//		if (i == 3)
//			continue;
		if (i == 4)
		{
			enable_inverse_motor_direction(motor->driver);
		}

		motion_mode_t motion_mode = MOTION_HOME;

		if (HAL_GPIO_ReadPin(motor->gpio_ports.mot_en, motor->gpio_pins.mot_en) == GPIO_PIN_SET)
			tmc2209_enable(motor->driver);

		motor->motion.acc_steps = (motor->motion.V_MAX * motor->motion.V_MAX) / (2 * motor->motion.ACC_MAX); //Calculate total acceleration and deceleration steps

		initMovementVars(motor, motion_mode);

		startMovement(motor);
		startStatusChecks(motor);
	}

	while(motors[0]->stallguard.stall_flag == 0
			|| motors[1]->stallguard.stall_flag == 0
			|| motors[2]->stallguard.stall_flag == 0
			|| motors[3]->stallguard.stall_flag == 0
			|| motors[4]->stallguard.stall_flag == 0)
	{
		checkDriverStatus(motors[0]);
		checkDriverStatus(motors[1]);
		checkDriverStatus(motors[2]);
		checkDriverStatus(motors[3]);
		checkDriverStatus(motors[4]);
	}

	motors[0]->stallguard.stall_flag = 0;
	motors[1]->stallguard.stall_flag = 0;
	motors[2]->stallguard.stall_flag = 0;
	motors[3]->stallguard.stall_flag = 0;
	motors[4]->stallguard.stall_flag = 0;
}

void initMovementVars(motor_t * motor, motion_mode_t motion_mode)
{
	motor->motion.v = 0;
	motor->motion.step = 0;
	motor->motion.cycle = 0;
	motor->motion.motion_mode = motion_mode;
}

void startMovement(motor_t * motor)
{
	HAL_GPIO_WritePin(motor->gpio_ports.step, motor->gpio_pins.step, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&motor->motion.motor_control_timer, TIM_CHANNEL_1, 1);
	HAL_TIM_OC_Start_IT(&motor->motion.motor_control_timer, TIM_CHANNEL_1);

	motor->active_movement_flag = 1;
}

void startStatusChecks(motor_t * motor)
{
	HAL_TIM_Base_Start_IT(&motor->status_check_timer);  //Timer for periodical status checks

	motor->status_flag = 0;
	motor->stallguard.previous_smoothed_result = 0;

//	while(motor->active_movement_flag)		//While motor is moving, periodically check driver status
//	{
//		checkDriverStatus(motor);
//	}
}

/*
 * Work in progress, simple prototype function.
 */
void checkOverheating(tmc2209_status_t status)
{
	if (status.over_temperature_157c || status.over_temperature_150c || status.over_temperature_143c)
	{
		writeDisplay("Critical Overheating!");
	}
	else if (status.over_temperature_120c)
	{
		writeDisplay("Warning, temperature above 120c");
	}
}

/*
 * Also work in progress, now simply outputs stallguard result to monitor.
 */
void checkStall(uint16_t stallguard_result, motor_t* motor)
{
	stallguard_t* sg = &motor->stallguard;
	uint16_t result = stallguard_result;
//	float diff;

	sg->smoothed_result = ALPHA * stallguard_result + (1-ALPHA) * sg->previous_smoothed_result;

//	smoothed_result_g = sg->smoothed_result;
//	stallguard_result_g = stallguard_result;
//	v_g = motor->motion.v;

//	diff = sg->smoothed_result - sg->previous_smoothed_result;
	float k = sg->MAX_STALLGUARD_VALUE / (float) motor->motion.V_MAX;

	float dynamic_stall_threshold = k * motor->motion.v - sg->STALL_BUFFER;

	if (motor->ID == '5')
	{
		result = sg->smoothed_result;
	}

	if (result < dynamic_stall_threshold)
	{
		sg->consecutive_low_counter++;
		if (sg->consecutive_low_counter >= sg->MAX_CONSECUTIVE_LOW)
		{
			stopMotorMovement(motor);
			sg->stall_flag = 1;
			sg->consecutive_low_counter = 0;
		}
	}
	else
	{
		sg->consecutive_low_counter = 0;
	}

	sg->previous_smoothed_result = sg->smoothed_result;

}

/*
 * This function is continuously called while a motor is active.
 * It only does something when status_flag has been set to 1.
 * Then it calls the checkOverheat and Load functions.
 */
void checkDriverStatus(motor_t* motor)
{
	if (motor->status_flag)
	{
//		tmc2209_status_t status;
		uint16_t stallguard_result;

		motor->status_flag = 0;
//		status = get_status(motor->driver);
//
//		checkOverheating(status);

		stallguard_result = get_stall_guard_result(motor->driver);
		checkStall(stallguard_result, motor);
	}
}

void toggle_inverse_motor_direction(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->global_config_.shaft = 1 - stepper_driver->global_config_.shaft;
  write_stored_global_config(stepper_driver);
}

