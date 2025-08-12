/*
 * motor_control.h
 *
 *  Created on: Jul 14, 2025
 *      Author: jakob
 */
#include "status_check.h"
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
	float gear_ratio;
	uint32_t microsteps;
	uint32_t v_max;
	uint32_t acc_max;
	uint32_t dec_max;

	uint8_t hold_current_percent;
	uint8_t run_current_percent;

	GPIO_Pins_t gpio_pins;
	GPIO_Ports_t gpio_ports;

	TIM_HandleTypeDef tim;

	UART_HandleTypeDef uart;

	uint32_t position_limit;
	uint32_t stallGuard_threshold;

	tmc2209_stepper_driver_t* driver;
}motor_t;


/*
 * Function Declaration
 */

void moveDegrees(float degrees, motor_t* motor);


