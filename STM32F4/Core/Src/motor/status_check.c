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

float stallguard_value, dynamic_stallguard_value; //global variables so they can be read in STM32CubeMonitor easily.

void checkOverheating(tmc2209_status_t status);
void checkStall(motor_t* motor);

/*
 * Starts the status_check_timer of the motor given by the parameter, which causes an interrupt every 10ms.
 */
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
 * Checks if the driver is overheating.
 * The driver can tell 4 states apart at which different messages and warnings can be send.
 * If wanted, the motor could also be shutted down.
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
 * A dynamic threshold is calculated and if the stallguard_result
 * read from the driver is below this threshold for more than MAX_CONSECUTIVE_LOW,
 * a Stall will be detected.
 */
void checkStall(motor_t* motor)
{
	stallguard_t* sg = &motor->stallguard;
	uint16_t stallguard_result = get_stall_guard_result(motor->driver); //Reads the stallguard_result via UART from driver.

	//As motor 5 stallguard values have a lot of noise, a smoothed version of the stallguard result is used.
	if (motor->ID == '5')
	{
		sg->smoothed_result = ALPHA * stallguard_result + (1-ALPHA) * sg->previous_smoothed_result; //Exponential smoothing/exponential moving average (EMA) filter
		stallguard_result = sg->smoothed_result;
		if (motor->motion.motion_mode == MOTION_GRIP) //
		{
			sg->STALL_BUFFER = STALL_GRIP_BUFFER_M_5;
		}
	}

	//calculating  the dynamic threshold
	float k = sg->MAX_STALLGUARD_VALUE / (float) motor->motion.V_MAX;
	float dynamic_stall_threshold = k * motor->motion.v - sg->STALL_BUFFER;

//	If wanted, this code can be uncommented to read stallguard values from the motor specified by ID
//	if (motor->ID == '2')
//	{
//		stallguard_value = stallguard_result;
//		dynamic_stallguard_value = dynamic_stall_threshold;
//	}


	if (stallguard_result < dynamic_stall_threshold)
	{
		sg->consecutive_low_counter++;
		//Set MAX_CONSECUTIVE_LOW to 1 to stop motor immediately after first stallguard value drop
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
	//for smoothing, the current stallguard value needs to be stored for one cycle.
	sg->previous_smoothed_result = sg->smoothed_result;

}

/*
 * This function should be continuously called while a motor is active.
 * It only does something when status_flag has been set to 1.
 * Then it calls the checkOverheat and checkStall functions.
 */
void checkDriverStatus(motor_t* motor)
{
	if (motor->status_flag)
	{
		motor->status_flag = 0;

//		tmc2209_status_t status;
//		status = get_status(motor->driver);
//		checkOverheating(status);

		checkStall(motor);
	}
}

/*
 * Use this function in a while-Loop while the motors are moving to check StallGuard values and overheating of every moving motor.
 */
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


