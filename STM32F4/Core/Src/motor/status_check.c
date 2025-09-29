/*
 * status_check.c
 *
 *  Created on: Sep 16, 2025
 *      Author: jakob
 */

#include "motor/status_check.h"
#include "motor/motor_control.h"
#include "display/helper.h"
#include "motor/motor_defines.h"

#define ALPHA 0.25f
extern motor_t * motors[];

float stallguard_value, dynamic_stallguard_value;

motor_error_t startStatusChecks(motor_t * motor)
{
	HAL_StatusTypeDef status;
	motor_error_t error = NO_ERROR;

	status = HAL_TIM_Base_Start_IT(&motor->status_check_timer);  //Timer for periodical status checks

	if (status != HAL_OK)
		error = MOTOR_ERROR;

	motor->status_flag = 0;
	motor->stallguard.previous_smoothed_result = 0;

	return error;
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

	float k = sg->MAX_STALLGUARD_VALUE / (float) motor->motion.V_MAX;

	if (motor->ID == '5')
	{
		sg->smoothed_result = ALPHA * stallguard_result + (1-ALPHA) * sg->previous_smoothed_result; //Exponential smoothing/exponential moving average (EMA) filter
		result = sg->smoothed_result;
		if (motor->motion.motion_mode == MOTION_GRIP)
		{
			sg->STALL_BUFFER = STALL_GRIP_BUFFER_M_5;
		}
	}

	float dynamic_stall_threshold = k * motor->motion.v - sg->STALL_BUFFER;

	if (motor->ID == '2')
	{
		stallguard_value = stallguard_result;
		dynamic_stallguard_value = dynamic_stall_threshold;
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

void checkAllDrivers()
{
	if (motors[0]->active_movement_flag)
		checkDriverStatus(motors[0]);

	if (motors[1]->active_movement_flag)
		checkDriverStatus(motors[1]);

	if (motors[2]->active_movement_flag)
		checkDriverStatus(motors[2]);

	if (motors[3]->active_movement_flag)
		checkDriverStatus(motors[3]);

	if (motors[4]->active_movement_flag)
		checkDriverStatus(motors[4]);
}


