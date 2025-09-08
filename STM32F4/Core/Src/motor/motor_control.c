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

//Point to Point Globals
#define LENGTH_SEGMENT_1 210
#define LENGTH_SEGMENT_2 160
#define LENGTH_SEGMENT_3 160
#define LENGTH_SEGMENT_4 112

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
void toPolar(float x, float y, float * theta_p, float * r_p);
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

/*
 * Trapezoidal motion profile (TrapezMove) – physical derivation
 *
 * The motion is divided into 3 phases:
 *   1) Acceleration: from v=0 to v_max with constant acceleration a_acc
 *   2) Constant velocity: v = v_max (if total distance is large enough)
 *   3) Deceleration: from v_max back to 0 with constant deceleration a_dec
 *
 * Fundamental kinematic relations:
 *   v = a * t
 *   s = 0.5 * a * t^2
 *   v^2 = 2 * a * s    (key equation for ramps)
 *
 * Acceleration phase:
 *   t_acc = v_max / a_acc
 *   s_acc = v_max^2 / (2 * a_acc)
 *
 * Deceleration phase:
 *   t_dec = v_max / a_dec
 *   s_dec = v_max^2 / (2 * a_dec)
 *
 * Constant velocity phase (if present. It won't be present if total distance isn't large enough):
 *   s_const = s_total - (s_acc + s_dec)
 *
 * If s_const >= 0 → trapezoidal profile (true trapez)
 * If s_const < 0  → triangular profile (no constant section)
 *
 * For the triangular profile:
 *   s_total = s_acc + s_dec
 *   s_acc = v_peak^2 / (2 * a_acc)
 *   s_dec = v_peak^2 / (2 * a_dec)
 *
 * Solve for v_peak:
 *   v_peak = sqrt( (2 * a_acc * a_dec) / (a_acc + a_dec) * s_total )
 *
 * Summary of implementation:
 *   - Precompute acc_steps, dec_steps, const_steps based on desired v_max, a_acc, a_dec.
 *   - If const_steps < 0, recalculate acc_steps, dec_steps using v_peak and set const_steps=0.
 *   - Online velocity update:
 *       Accel: v = sqrt(2 * a_acc * s)
 *       Const: v = v_max
 *       Decel: v = sqrt(2 * a_dec * s_remaining)
 */
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
		case MOTION_HOME:	//Since we don't know the exact distance to move in these 2 following cases, there's no deceleration
		case MOTION_GRIP:
			if (mt->step >= 0 && mt->step < mt->acc_steps)
				mt->v = sqrtf(2 * mt->ACC_MAX * (mt->step + 1));
			else
				mt->v = mt->V_MAX;

			break;
		}
		mt->step++;
		mt->position++;
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
	if (GPIO_Pin == B1_Pin)  // Check if User Button
	{
		toggle_inverse_motor_direction(motors[4]->driver);
	}
}

void moveAbsolute(float degrees, motor_t* motor)
{
	if (degrees < 0)
		return;

	if (HAL_GPIO_ReadPin(motor->gpio_ports.mot_en, motor->gpio_pins.mot_en) == GPIO_PIN_SET)
		tmc2209_enable(motor->driver);

	int actualSteps = toSteps(degrees, motor); //Convert degrees to steps

	if (actualSteps > motor->motion.position)
	{
		enable_inverse_motor_direction(motor->driver);
		motor->motion.total_steps = actualSteps - motor->motion.position;
	}
	else
	{
		disable_inverse_motor_direction(motor->driver);
		motor->motion.total_steps = motor->motion.position - actualSteps;
	}

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

/*
 * Initiates motor movement by:
 *   - enabling the driver if needed
 *   - converting angle to total steps
 *   - calculating acc_steps, dec_steps, const_steps
 *   - adjusting to triangle profile if total steps are too short
 *   - starting the timer for output compare with interrupt
 */
void moveDegrees(float degrees, motor_t* motor)
{
	if (HAL_GPIO_ReadPin(motor->gpio_ports.mot_en, motor->gpio_pins.mot_en) == GPIO_PIN_SET)
		tmc2209_enable(motor->driver);

	if (degrees < 0)
	{
		enable_inverse_motor_direction(motor->driver);
		degrees = degrees * (-1);
	}
	else
		disable_inverse_motor_direction(motor->driver);

	motor->motion.total_steps = toSteps(degrees, motor); //Convert degrees to steps
	motor->motion.acc_steps = (motor->motion.V_MAX * motor->motion.V_MAX) / (2 * motor->motion.ACC_MAX); //Calculate total acceleration and deceleration steps
	motor->motion.dec_steps = (motor->motion.V_MAX * motor->motion.V_MAX) / (2 * motor->motion.DEC_MAX);
	motor->motion.const_steps = motor->motion.total_steps - (motor->motion.acc_steps + motor->motion.dec_steps);

	motion_mode_t motion_mode = MOTION_TRAPEZ;

	initMovementVars(motor, motion_mode);

	if (motor->motion.const_steps < 0)  // If acceleration steps + deceleration steps exceed total steps -> trapezoid not possible -> triangle profile
	{
	    // Compute peak velocity based on available distance (triangle case)
	    float v_peak = sqrtf((2.0f * motor->motion.ACC_MAX * motor->motion.DEC_MAX) /
	                         (motor->motion.ACC_MAX + motor->motion.DEC_MAX) *
	                         (float)motor->motion.total_steps);

	    // Recalculate acceleration and deceleration steps for triangle profile
	    motor->motion.acc_steps = (uint32_t)(v_peak * v_peak / (2.0f * motor->motion.ACC_MAX));
	    motor->motion.dec_steps = motor->motion.total_steps - motor->motion.acc_steps;
	    motor->motion.const_steps = 0;

	    // Adjust maximum velocity to the achievable peak
	    motor->motion.V_MAX = v_peak;
	}

	//Start timer in output compare with interrupt
	startMovement(motor);

	startStatusChecks(motor);
}

void toPolar(float x, float y, float * theta_p, float * r_p)
{
	if (x < 0 && y > 0)
		*theta_p = atan(y/x);
	else if (x > 0 && y > 0)
		*theta_p = atan (x/y) + 90;
	else if (x > 0 && y > 0)
		*theta_p = atan (y/x) + 180;
	else if (x > 0 && y > 0)
		*theta_p = atan (x/y) + 270;

	*r_p = sqrtf(x*x + y*y);
}

void movePolar(float theta, float r, float z, gripper_direction_t gripper_direction)
{
	uint32_t length_segment_3_adjusted;
	switch(gripper_direction)
	{
	case VERTICAL_UP:
		z += LENGTH_SEGMENT_4;
		length_segment_3_adjusted = LENGTH_SEGMENT_3;
		break;
	case HORIZONTAL:
		r += LENGTH_SEGMENT_4;
		length_segment_3_adjusted = LENGTH_SEGMENT_3 + LENGTH_SEGMENT_4;
		break;
	case VERTICAL_DOWN:
		z -= LENGTH_SEGMENT_4;
		length_segment_3_adjusted = LENGTH_SEGMENT_3;
		break;
	}
	float phi1, phi2, phi3, phi4;
	phi1 = theta;

	phi3 = acos(((r*r) - LENGTH_SEGMENT_1 + LENGTH_SEGMENT_2 - (z*z)) / (2 * LENGTH_SEGMENT_1 * LENGTH_SEGMENT_2));
	float d = sqrtf(r*r + abs(z-LENGTH_SEGMENT_1) * abs(z-LENGTH_SEGMENT_1));

	float gamma = acos(d / sqrtf((r*r) + (z*z)));
	float beta = atan(length_segment_3_adjusted * sin(phi3) / (LENGTH_SEGMENT_2 + cos(phi3)*length_segment_3_adjusted));

	if (z >= LENGTH_SEGMENT_1)
		phi2 = 90 - gamma - beta;
	else
		phi2 = 90 + gamma -beta;

	switch(gripper_direction)
	{
	case VERTICAL_UP:
		phi4 = 180 - phi3 - phi2;
	case HORIZONTAL:
		phi4 = 0;
	case VERTICAL_DOWN:
		phi4 = 270 - phi3 - phi2;
	}

	moveAbsolute(phi1, motors[0]);
	moveAbsolute(phi2, motors[1]);
	moveAbsolute(phi3, motors[2]);
	moveAbsolute(phi4, motors[3]);
}

void moveToCoordinates(float x, float y, float z, gripper_direction_t gripper_direction)
{
	float theta;
	float r;

	toPolar(x, y, &theta, &r);
	movePolar(theta, r, z, gripper_direction);
}

void grip()
{
	motor_t * motor5 = motors[4];

	disable_inverse_motor_direction(motor5->driver);
	moveDegrees(10000, motor5);
	while(motor5->active_movement_flag); //wait until movement finished

	enable_inverse_motor_direction(motor5->driver);

	if (HAL_GPIO_ReadPin(motor5->gpio_ports.mot_en, motor5->gpio_pins.mot_en) == GPIO_PIN_SET)
		tmc2209_enable(motor5->driver);

	motor5->motion.acc_steps = (motor5->motion.V_MAX * motor5->motion.V_MAX) / (2 * motor5->motion.ACC_MAX); //Calculate total acceleration steps

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
	writeDisplay("Homing...");
	for (int i = 0; i < NUMBER_OF_MOTOR; i++)
	{
		motor_t * motor = motors[i];
		if (i == 3)
			continue;
		if (i == 4)
		{
			enable_inverse_motor_direction(motor->driver);
		}

		motion_mode_t motion_mode = MOTION_HOME;

		if (HAL_GPIO_ReadPin(motor->gpio_ports.mot_en, motor->gpio_pins.mot_en) == GPIO_PIN_SET)
			tmc2209_enable(motor->driver);

		motor->motion.acc_steps = (motor->motion.V_MAX * motor->motion.V_MAX) / (2 * motor->motion.ACC_MAX); //Calculate total acceleration steps

		initMovementVars(motor, motion_mode);

		startMovement(motor);
		startStatusChecks(motor);
	}

	while(motors[0]->stallguard.stall_flag == 0
			|| motors[1]->stallguard.stall_flag == 0
			|| motors[2]->stallguard.stall_flag == 0
//			|| motors[3]->stallguard.stall_flag == 0
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

	motors[0]->motion.position = 1245;
	motors[1]->motion.position = 0;
	motors[2]->motion.position = 0;
	motors[3]->motion.position = 0;
	motors[4]->motion.position = 0;

	writeDisplay("Homing finished");
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

	sg->smoothed_result = ALPHA * stallguard_result + (1-ALPHA) * sg->previous_smoothed_result; //Exponential smoothing/exponential moving average (EMA) filter

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

