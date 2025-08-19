/*
 * motor_control.c
 *
 *  Created on: Jul 14, 2025
 *      Author: jakob
 */
#include "motor/motor_control.h"

//Globals

extern motor_t *motors[]; //To gain access to motor variables in interrupt service routine
extern TIM_HandleTypeDef htim9;

#define MOTOR_COUNT 5
#define VELOCITY_THRESHOLD 200
/*
 * Function Declaration
 */

int32_t toSteps(float degrees, motor_t* motor);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
void checkOverheating(tmc2209_status_t status);
void checkStall(uint16_t stallguard_result, motor_t* motor);
void checkDriverStatus(motor_t* motor);

/*
 * Calculates the steps needed to rotate the amount stated in the variable degrees.
 */
int32_t toSteps(float degrees, motor_t* motor)
{
	int32_t steps;
	steps = ((200.0 * (float)(motor->microsteps)/360.0)*degrees) * motor->gear_ratio;
	return steps;
}

/*
 *
 * Interrupt service routine for timer in output compare mode.
 * Timer counts until compare value is reached.
 * When value is reached, the GPIO toggles.
 * Every other cycle, as step only triggers on rising edge,
 * the velocity and the compare value is changed depending on current state of velocity ramp.
 *
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	int8_t index;
	motor_t* motor;

	//To know which timer and thus which motor caused the interrupt
	if (htim->Instance == motors[0]->motor_control_timer.Instance){ index = 0; }
	else if (htim->Instance == motors[1]->motor_control_timer.Instance){ index = 1; }
	else if (htim->Instance == motors[2]->motor_control_timer.Instance){ index = 2; }
	else if (htim->Instance == motors[3]->motor_control_timer.Instance){ index = 3; }
	else if (htim->Instance == motors[4]->motor_control_timer.Instance){ index = 4; }

	motor = motors[index];

	//Stop timer and movement if the robot reaches its destination
	if (motor->step >= motor->total_steps || motor->stall_flag)
	{
		HAL_TIM_OC_Stop_IT(&motor->motor_control_timer, TIM_CHANNEL_1);
		motor->active_movement_flag = 0;
		motor->stall_flag = 0;
		return;
	}

	if (motor->cycle % 2 == 0) //Change velocity only every other cycle because step only triggers on rising edge
	{
//		if (motor->stall_flag && motor->step < (motor->step_at_stall + motor->dec_steps_after_stall))
//		{
//			motor->v = sqrtf(2 * motor->dec_max * (2 * motor->step_at_stall - motor->step));
//		}
		if (motor->step >= 0 && motor->step < motor->acc_steps)
		{
			motor->v = sqrtf(2 * motor->acc_max * (motor->step + 1));
			// v = acc_ramp[step]
		}
		else if (motor->const_steps != 0 && motor->step >= motor->acc_steps && motor->step < (motor->total_steps - motor->dec_steps))
			motor->v = motor->v_max;
		else if (motor->step >= (motor->total_steps - motor->dec_steps) && motor->step < motor->total_steps)
		{
			motor->v = sqrtf(2 * motor->dec_max * (motor->total_steps - motor->step));
			// v = acc_ramp[total_steps - step]
		}

		motor->step++;
	}

	motor->cycle++;

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

	int32_t delay = 2000000 / (2 * motor->v);
	//Add delay to current compare value in register
	int32_t total_delay = __HAL_TIM_GET_COMPARE(&motor->motor_control_timer, TIM_CHANNEL_1) + delay;
	__HAL_TIM_SET_COMPARE(&motor->motor_control_timer, TIM_CHANNEL_1, total_delay);
}

/*
 * Interrupt service routine for timer 9, which periodically invokes status checks.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	for(int i = 0; i < MOTOR_COUNT; i++)
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

/*
 * Initiates motor movement by starting the timer and calculating the steps
 */
void moveDegrees(float degrees, motor_t* motor)
{
	tmc2209_enable(motor->driver);
	motor->total_steps = toSteps(degrees, motor); //Convert degrees to steps
	motor->acc_steps = (motor->v_max * motor->v_max) / (2 * motor->acc_max); //Calculate total acceleration and deceleration steps
	motor->dec_steps = (motor->v_max * motor->v_max) / (2 * motor->dec_max);
	motor->const_steps = motor->total_steps - (motor->acc_steps + motor->dec_steps);

	motor->v = 0;
	motor->step = 0;
	motor->cycle = 0;
	motor->stall_flag = 0;

	if (motor->const_steps < 0)	//If acceleration steps + deceleration steps are bigger than total steps
	{
		motor->acc_steps = motor->total_steps / 2;
		motor->dec_steps = motor->total_steps / 2;
		motor->const_steps = 0;
	}

	//Start timer in output compare with interrupt
	HAL_GPIO_WritePin(motor->gpio_ports.step, motor->gpio_pins.step, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&motor->motor_control_timer, TIM_CHANNEL_1, 1);
	HAL_TIM_OC_Start_IT(&motor->motor_control_timer, TIM_CHANNEL_1);

	motor->active_movement_flag = 1;

	HAL_TIM_Base_Start_IT(&motor->status_check_timer);  //Timer for periodical status checks

	motor->status_flag = 0;

	while(motor->active_movement_flag)		//While motor is moving, periodically check driver status
	{
		checkDriverStatus(motor);
	}
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

	if (stallguard_result <= motor->stallGuard_threshold && motor->v > VELOCITY_THRESHOLD)
	{
		motor->stall_flag = 1;
	}

	char str[10];
	snprintf(str, sizeof(str), "%c:%u", motor->ID, stallguard_result);
	HAL_GPIO_TogglePin(LED_red_GPIO_Port, LED_red_Pin);
	writeDisplay(str);
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

