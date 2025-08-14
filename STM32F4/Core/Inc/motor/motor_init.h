/*
 * motor_init.h
 *
 *  Created on: Aug 11, 2025
 *      Author: jakob
 */

#ifndef INC_MOTOR_INIT_H_
#define INC_MOTOR_INIT_H_

#include "motor_defines.h"
#include "motor_control.h"

void init_motor_1(motor_t* motor, tmc2209_stepper_driver_t* driver);
void init_motor_2(motor_t* motor, tmc2209_stepper_driver_t* driver);
void init_motor_3(motor_t* motor, tmc2209_stepper_driver_t* driver);
void init_motor_4(motor_t* motor, tmc2209_stepper_driver_t* driver);
void init_motor_5(motor_t* motor, tmc2209_stepper_driver_t* driver);


#endif /* INC_MOTOR_INIT_H_ */
