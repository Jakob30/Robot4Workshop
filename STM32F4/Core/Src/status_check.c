/*
 * status_check.c
 *
 *  Created on: Aug 7, 2025
 *      Author: jakob
 */

#include "status_check.h"

uint8_t status_flag;
extern uint8_t active_movement_flag;

/*
 * Interrupt service routine for timer 9, which periodically invokes status checks.
 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM9)
    {
    	if (active_movement_flag)
    	{
    		status_flag = 1;	//Set flag, to indicate an interrupt has happened
    	}
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
void checkLoad(uint16_t stallguard_result)
{
	char str[4];
	snprintf(str, sizeof(str), "%u", stallguard_result);
	writeDisplay(str);
}

/*
 * This function is continuously called while a motor is active.
 * It only does something when status_flag has been set to 1.
 * Then it calls the checkOverheat and Load functions.
 */

void checkDriverStatus(tmc2209_stepper_driver_t* driver)
{
	if (status_flag)
	{
		tmc2209_status_t status;
		uint16_t stallguard_result;

		status_flag = 0;
		status = get_status(driver);

		checkOverheating(status);

		stallguard_result = get_stall_guard_result(driver);
		checkLoad(stallguard_result);
	}

}



