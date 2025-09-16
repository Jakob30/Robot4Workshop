/*
 * kinematics.h
 *
 *  Created on: Sep 16, 2025
 *      Author: jakob
 */

#ifndef INC_MOTOR_KINEMATICS_H_
#define INC_MOTOR_KINEMATICS_H_

#include "motor_types.h"

void toPolar(float x, float y, float * theta_p, float * r_p);
motor_error_t checkPositionLimits(float phi, float LOW_LIMIT, float HIGH_LIMIT, uint8_t id);
motor_error_t calculateAngles(float phi[], float theta, float r, float z, float gripper_direction);

#endif /* INC_MOTOR_KINEMATICS_H_ */
