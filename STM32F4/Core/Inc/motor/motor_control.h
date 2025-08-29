/*
 * motor_control.h
 *
 *  Created on: Jul 14, 2025
 *      Author: jakob
 */
#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include <display/helper.h>
#include "tmc2209.h"
#include "main.h"
#include "math.h"

typedef struct
{
	GPIO_TypeDef *step;
	GPIO_TypeDef *dir;
	GPIO_TypeDef *mot_en;
	GPIO_TypeDef *diag;
}GPIO_Ports_t;

typedef struct
{
	uint16_t step;
	uint16_t dir;
	uint16_t mot_en;
	uint16_t diag;
}GPIO_Pins_t;

typedef enum {
    MOTION_TRAPEZ,
    MOTION_HOME,
	MOTION_GRIP
} motion_mode_t;

typedef struct
{
	uint32_t V_MAX;
	uint32_t ACC_MAX;
	uint32_t DEC_MAX;

	float v; //Monitors the current velocity of stepper motor in µsteps/s
	int32_t total_steps;
	int32_t const_steps;
	int32_t acc_steps;
	int32_t dec_steps;
	int32_t step; //Monitors the current step
	uint32_t cycle; //Monitors current cycle, which is double step size

	motion_mode_t motion_mode;
	TIM_HandleTypeDef motor_control_timer;
}motion_t;

typedef struct
{
	uint8_t HOLD_CURRENT_PERCENT;
	uint8_t RUN_CURRENT_PERCENT;
}current_settings_t;

typedef struct
{
	uint32_t POSITION_LIMIT;
	uint16_t MAX_CONSECUTIVE_LOW;
	float MAX_STALLGUARD_VALUE;
	uint16_t STALL_BUFFER;

	uint8_t stall_flag;
	float smoothed_result;
	float previous_smoothed_result;
	uint16_t consecutive_low_counter;
}stallguard_t;

typedef struct
{
	char ID;

	float gear_ratio;
	uint32_t microsteps;

	uint8_t active_movement_flag;
	motion_t motion;

	current_settings_t current_settings;

	uint8_t status_flag;
	TIM_HandleTypeDef status_check_timer;

	stallguard_t stallguard;

	GPIO_Pins_t gpio_pins;
	GPIO_Ports_t gpio_ports;
	UART_HandleTypeDef uart;

	tmc2209_stepper_driver_t* driver;
}motor_t;


/*
 * Function Declaration
 */

void moveDegrees(float degrees, motor_t* motor);
void checkDriverStatus(motor_t * motor);
void goHome();
void grip();

#endif /* INC_MOTOR_CONTROL_H_ */
