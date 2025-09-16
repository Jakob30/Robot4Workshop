/*
 * status_check.h
 *
 *  Created on: Sep 16, 2025
 *      Author: jakob
 */

#ifndef INC_MOTOR_STATUS_CHECK_H_
#define INC_MOTOR_STATUS_CHECK_H_

#include "motor_types.h"

motor_error_t startStatusChecks(motor_t * motor);
void checkOverheating(tmc2209_status_t status);
void checkStall(uint16_t stallguard_result, motor_t* motor);
void checkDriverStatus(motor_t* motor);
void checkAllDrivers();

#endif /* INC_MOTOR_STATUS_CHECK_H_ */
