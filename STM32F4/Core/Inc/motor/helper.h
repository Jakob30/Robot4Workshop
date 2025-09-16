/*
 * helper.h
 *
 *  Created on: Sep 16, 2025
 *      Author: jakob
 */

#ifndef INC_MOTOR_HELPER_H_
#define INC_MOTOR_HELPER_H_

#include "motor/motor_types.h"

float toGrad(float degrees_rad);
float toRad(float degrees_grad);
int32_t toSteps(float degrees, motor_t* motor);

#endif /* INC_MOTOR_HELPER_H_ */
