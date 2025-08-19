/*
 * motor_control.h
 *
 *  Created on: Jul 14, 2025
 *      Author: jakob
 */
#include <display/helper.h>
#include "tmc2209.h"
#include "main.h"
#include "math.h"

typedef struct
{
	GPIO_TypeDef *step;
	GPIO_TypeDef *dir;
	GPIO_TypeDef *mot_en;
}GPIO_Ports_t;

typedef struct
{
	uint16_t step;
	uint16_t dir;
	uint16_t mot_en;
}GPIO_Pins_t;

typedef struct
{
	char ID;

	float gear_ratio;
	uint32_t microsteps;
	uint32_t v_max;
	uint32_t acc_max;
	uint32_t dec_max;

	float v; //Monitors the current velocity of stepper motor in µsteps/s
	int32_t total_steps;
	int32_t const_steps;
	int32_t acc_steps;
	int32_t dec_steps;
	int32_t step; //Monitors the current step
	uint32_t cycle; //Monitors current cycle, which is double step size

	uint8_t active_movement_flag;

	uint8_t possible_stall_flag;
	int32_t step_at_stall;
	int32_t dec_steps_after_stall;
	uint8_t stall_flag;
	uint8_t status_flag;

	uint8_t hold_current_percent;
	uint8_t run_current_percent;

	GPIO_Pins_t gpio_pins;
	GPIO_Ports_t gpio_ports;

	TIM_HandleTypeDef motor_control_timer;
	TIM_HandleTypeDef status_check_timer;

	UART_HandleTypeDef uart;

	uint32_t position_limit;
	uint32_t stallGuard_threshold;

	tmc2209_stepper_driver_t* driver;
}motor_t;


/*
 * Function Declaration
 */

void moveDegrees(float degrees, motor_t* motor);


