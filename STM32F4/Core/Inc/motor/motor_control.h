/*
 * motor_control.h
 *
 *  Created on: Jul 14, 2025
 *      Author: jakob
 */
#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "motor_types.h"

/*
 * Function Declaration
 */

motor_error_t moveDegrees(float degrees, motor_t* motor);
motor_error_t moveAbsolute(float degrees, motor_t* motor);
motor_error_t movePolar(float theta, float r, float z, float gripper_direction);
motor_error_t moveToCoordinates(float x, float y, float z, float gripper_direction);
motor_error_t goHome();
motor_error_t openGripper();
motor_error_t grip();

void stopMotorMovement(motor_t * motor);

#endif /* INC_MOTOR_CONTROL_H_ */
