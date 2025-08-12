/*
 * motor_control.c
 *
 *  Created on: Jul 14, 2025
 *      Author: jakob
 */
#include "motor_control.h"
//#define MAX_ACC_STEPS 10000

//Globals

float v; //Monitors the current velocity of stepper motor in µsteps/s
int32_t total_steps;
int32_t const_steps;
int32_t acc_steps;
int32_t dec_steps;
int32_t step; //Monitors the current step
uint32_t cycle; //Monitors current cycle, which is double step size
//float acc_ramp[MAX_ACC_STEPS]; //To store the acceleration ramp

extern uint8_t active_movement_flag; //Indicates if a motor is active
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
	if (step >= total_steps)
	{
		HAL_TIM_OC_Stop_IT(&motors[index]->tim, TIM_CHANNEL_1);
		active_movement_flag = 0; //Reset active_movement_flag
		return;
	}

	if (cycle % 2 == 0) //Change velocity only every other cycle because step only triggers on rising edge
	{
		if (step >= 0 && step < acc_steps)
		{
			v = sqrtf(2 * motors[index]->acc_max * (step + 1));
			// v = acc_ramp[step]
		}
		else if (const_steps != 0 && step >= acc_steps && step < (total_steps - dec_steps))
			v = motors[index]->v_max;
		else if (step >= (total_steps - dec_steps) && step < total_steps)
		{
			v = sqrtf(2 * motors[index]->dec_max * (total_steps - step));
			// v = acc_ramp[total_steps - step]
		}

		step++;
	}

	cycle++;

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

	int32_t delay = 2000000 / (2 * v);
	//Add delay to current compare value in register
	int32_t total_delay = __HAL_TIM_GET_COMPARE(&motors[index]->tim, TIM_CHANNEL_1) + delay;
	__HAL_TIM_SET_COMPARE(&motors[index]->tim, TIM_CHANNEL_1, total_delay);
}

/*
 * Initiates motor movement by starting the timer and calculating the acceleration and deceleration steps.
 */
void moveDegrees(float degrees, motor_t* motor)
{
	tmc2209_enable(motor->driver);
	total_steps = toSteps(degrees, motor); //Convert degrees to steps
	acc_steps = (motor->v_max * motor->v_max) / (2 * motor->acc_max); //Calculate total acceleration and deceleration steps
	dec_steps = (motor->v_max * motor->v_max) / (2 * motor->dec_max);
	const_steps = total_steps - (acc_steps + dec_steps);

	v = 0;
	step = 0;
	cycle = 0;


	if (const_steps < 0)	//If acceleration steps + deceleration steps are bigger than total steps
	{
		acc_steps = total_steps / 2;
		dec_steps = total_steps / 2;
		const_steps = 0;
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

	active_movement_flag = 1;

	HAL_TIM_Base_Start_IT(&htim9);  //Timer for periodical status checks

	status_flag = 0;

	while(active_movement_flag)		//While motor is moving, periodically check driver status
		checkDriverStatus(motor->driver);
}

