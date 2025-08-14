/*
 * motor_control.c
 *
 *  Created on: Jul 14, 2025
 *      Author: jakob
 */
#include "motor/motor_control.h"
//#define MAX_ACC_STEPS 10000

//Globals

//float acc_ramp[MAX_ACC_STEPS]; //To store the acceleration ramp

extern motor_t *motors[]; //To gain access to motor variables in interrupt service routine
extern uint8_t status_flag; //Indicates if
extern TIM_HandleTypeDef htim9;	//

/*
 * Function Declaration
 */

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

	//To know which timer and thus which motor caused the interrupt
	if (htim->Instance == motors[0]->tim.Instance){ index = 0; }
	else if (htim->Instance == motors[1]->tim.Instance){ index = 1; }
	else if (htim->Instance == motors[2]->tim.Instance){ index = 2; }
	else if (htim->Instance == motors[3]->tim.Instance){ index = 3; }
	else if (htim->Instance == motors[4]->tim.Instance){ index = 4; }

	//Stop timer and movement if the roboter reaches its destination
	if (motors[index]->step >= motors[index]->total_steps)
	{
		HAL_TIM_OC_Stop_IT(&motors[index]->tim, TIM_CHANNEL_1);
		motors[index]->active_movement_flag = 0;
		return;
	}

	if (motors[index]->cycle % 2 == 0) //Change velocity only every other cycle because step only triggers on rising edge
	{
		if (motors[index]->step >= 0 && motors[index]->step < motors[index]->acc_steps)
		{
			motors[index]->v = sqrtf(2 * motors[index]->acc_max * (motors[index]->step + 1));
			// v = acc_ramp[step]
		}
		else if (motors[index]->const_steps != 0 && motors[index]->step >= motors[index]->acc_steps && motors[index]->step < (motors[index]->total_steps - motors[index]->dec_steps))
			motors[index]->v = motors[index]->v_max;
		else if (motors[index]->step >= (motors[index]->total_steps - motors[index]->dec_steps) && motors[index]->step < motors[index]->total_steps)
		{
			motors[index]->v = sqrtf(2 * motors[index]->dec_max * (motors[index]->total_steps - motors[index]->step));
			// v = acc_ramp[total_steps - step]
		}

		motors[index]->step++;
	}

	motors[index]->cycle++;

	HAL_GPIO_TogglePin(motors[index]->gpio_ports.step, motors[index]->gpio_pins.step);

	/*
	 * 	To reach the desired speed, we need to calculate the delay between two toggles.
	 *	v is in µsteps/s or just 1/s because one µstep is one period.
	 *	For one period, the duration is 1/v.
	 *	Between two toggles, it is 1/(2*v).
	 *	But this is not the answer since the time has to be converted into timer ticks.
	 *	The timer runs at 2 MHz so we need to divide our current period duration by 1 / 2000000 s or 0.5 µs.
	 *	-> delay in ticks = 1/(2*v)/0.0000005 = 2000000/(2*v)
	 */

	int32_t delay = 2000000 / (2 * motors[index]->v);
	//Add delay to current compare value in register
	int32_t total_delay = __HAL_TIM_GET_COMPARE(&motors[index]->tim, TIM_CHANNEL_1) + delay;
	__HAL_TIM_SET_COMPARE(&motors[index]->tim, TIM_CHANNEL_1, total_delay);
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


	if (motor->const_steps < 0)	//If acceleration steps + deceleration steps are bigger than total steps
	{
		motor->acc_steps = motor->total_steps / 2;
		motor->dec_steps = motor->total_steps / 2;
		motor->const_steps = 0;
	}

/*
*	for (int i = 0; i < acc_steps; i++)
*	{
*		acc_ramp[i] = sqrtf(2 * motor->acc_max * step); //Calculate acc and dec ramp beforehand
*	}
*/

	//Start timer in output compare with interrupt
	HAL_GPIO_WritePin(motor->gpio_ports.step, motor->gpio_pins.step, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&motor->tim, TIM_CHANNEL_1, 1);
	HAL_TIM_OC_Start_IT(&motor->tim, TIM_CHANNEL_1);

	motor->active_movement_flag = 1;

//	HAL_TIM_Base_Start_IT(&htim9);  //Timer for periodical status checks
//
//	status_flag = 0;
//
//	while(active_movement_flag)		//While motor is moving, periodically check driver status
//	{
//		checkDriverStatus(motor->driver);
//	}
}

