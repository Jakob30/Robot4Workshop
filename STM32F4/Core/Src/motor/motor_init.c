/*
 * motor_init.c
 *
 *  Created on: Aug 11, 2025
 *      Author: jakob
 */

#include "motor/motor_init.h"
#include "motor/motor_defines.h"
#include "main.h"

//status check timer
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;

//motor control timer
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

extern motor_t * motors[];
void initMotor1(uart_mode_t UART_MODE);
void initMotor2(uart_mode_t UART_MODE);
void initMotor3(uart_mode_t UART_MODE);
void initMotor4(uart_mode_t UART_MODE);
void initMotor5(uart_mode_t UART_MODE);

void initAllMotors(uart_mode_t UART_MODE)
{
	initMotor1(UART_MODE);
	initMotor2(UART_MODE);
	initMotor3(UART_MODE);
	initMotor4(UART_MODE);
	initMotor5(UART_MODE);
}

void initMotor1(uart_mode_t UART_MODE)
{
	motor_t * motor1 = motors[0];
	tmc2209_stepper_driver_t * driver1 = motor1->driver;

	motor1->gpio_pins.mot_en 	= MOT_EN_1_Pin;
	motor1->gpio_ports.mot_en 	= MOT_EN_1_GPIO_Port;

	tmc2209_set_hardware_enable_pin(driver1, motor1->gpio_pins.mot_en, motor1->gpio_ports.mot_en);
	tmc2209_disable(driver1);

	initializeDefaults(motor1);
	motor1->ID = '1';

	motor1->gear_ratio = GEAR_RATIO_M_1;
	motor1->microsteps = MICROSTEPS_M_1;
	motor1->motion.V_MAX 	= V_MAX_M_1;
	motor1->motion.ACC_MAX = ACC_MAX_M_1;
	motor1->motion.DEC_MAX = DEC_MAX_M_1;

	motor1->current_settings.HOLD_CURRENT_PERCENT = HOLD_CURRENT_PERCENT_M_1;
	motor1->current_settings.RUN_CURRENT_PERCENT = RUN_CURRENT_PERCENT_M_1;

	motor1->gpio_pins.step 		= STEP_1_Pin;
	motor1->gpio_pins.dir 		= DIR_1_Pin;
//	motor1->gpio_pins.diag 		= DIAG_1_Pin;
	motor1->gpio_ports.step 	= STEP_1_GPIO_Port;
	motor1->gpio_ports.dir 		= DIR_1_GPIO_Port;
//	motor1->gpio_ports.diag		= DIAG_1_GPIO_Port;

	motor1->motion.motor_control_timer = htim12;
	motor1->status_check_timer = htim1;
	if (UART_MODE == MODE_5_UART)
	{
		motor1->uart = huart1;
		tmc2209_setup(driver1, 115200, SERIAL_ADDRESS_0);
	}
	else if (UART_MODE == MODE_2_UART)
	{
		motor1->uart = huart1;
		tmc2209_setup(driver1, 115200, SERIAL_ADDRESS_0);
	}

	motor1->stallguard.POSITION_LIMIT = POSITION_LIMIT_M_1;
	motor1->motion.HIGH_LIMIT = HIGH_LIMIT_M_1;
	motor1->motion.LOW_LIMIT = LOW_LIMIT_M_1;

	motor1->stallguard.MAX_CONSECUTIVE_LOW = LOW_COUNTER_THRESHOLD_M_1;
	motor1->stallguard.MAX_STALLGUARD_VALUE = STALL_MAX_M_1;
	motor1->stallguard.STALL_BUFFER = STALL_BUFFER_M_1;

	driver1->uart_ = motor1->uart;

	set_micro_steps_per_step(driver1, motor1->microsteps);
	set_all_current_percent_values(driver1, motor1->current_settings.RUN_CURRENT_PERCENT, motor1->current_settings.HOLD_CURRENT_PERCENT, 0);

}

void initMotor2(uart_mode_t UART_MODE)
{
	motor_t * motor2 = motors[1];
	tmc2209_stepper_driver_t * driver2 = motor2->driver;

	motor2->gpio_pins.mot_en 	= MOT_EN_2_Pin;
	motor2->gpio_ports.mot_en 	= MOT_EN_2_GPIO_Port;

	tmc2209_set_hardware_enable_pin(driver2, motor2->gpio_pins.mot_en, motor2->gpio_ports.mot_en);
	tmc2209_disable(driver2);

	initializeDefaults(motor2);
	motor2->ID = '2';

	motor2->gear_ratio = GEAR_RATIO_M_2;
	motor2->microsteps = MICROSTEPS_M_2;
	motor2->motion.V_MAX 	= V_MAX_M_2;
	motor2->motion.ACC_MAX = ACC_MAX_M_2;
	motor2->motion.DEC_MAX = DEC_MAX_M_2;


	motor2->current_settings.HOLD_CURRENT_PERCENT = HOLD_CURRENT_PERCENT_M_2;
	motor2->current_settings.RUN_CURRENT_PERCENT = RUN_CURRENT_PERCENT_M_2;

	motor2->gpio_pins.step 		= STEP_2_Pin;
	motor2->gpio_pins.dir 		= DIR_2_Pin;
//	motor2->gpio_pins.diag 		= DIAG_2_Pin;
	motor2->gpio_ports.step 	= STEP_2_GPIO_Port;
	motor2->gpio_ports.dir 		= DIR_2_GPIO_Port;
//	motor2->gpio_ports.diag 	= DIAG_2_GPIO_Port;

	motor2->motion.motor_control_timer = htim13;
	motor2->status_check_timer = htim6;

	if (UART_MODE == MODE_5_UART)
	{
		motor2->uart = huart6;
		tmc2209_setup(driver2, 115200, SERIAL_ADDRESS_0);
	}
	else if (UART_MODE == MODE_2_UART)
	{
		motor2->uart = huart1;
		tmc2209_setup(driver2, 115200, SERIAL_ADDRESS_1);
	}

	motor2->stallguard.POSITION_LIMIT = POSITION_LIMIT_M_2;
	motor2->motion.HIGH_LIMIT = HIGH_LIMIT_M_2;
	motor2->motion.LOW_LIMIT = LOW_LIMIT_M_2;

	motor2->stallguard.MAX_CONSECUTIVE_LOW = LOW_COUNTER_THRESHOLD_M_2;
	motor2->stallguard.MAX_STALLGUARD_VALUE = STALL_MAX_M_2;
	motor2->stallguard.STALL_BUFFER = STALL_BUFFER_M_2;

	driver2->uart_ = motor2->uart;


	set_micro_steps_per_step(driver2, motor2->microsteps);
	set_all_current_percent_values(driver2, motor2->current_settings.RUN_CURRENT_PERCENT, motor2->current_settings.HOLD_CURRENT_PERCENT, 0);

}

void initMotor3(uart_mode_t UART_MODE)
{
	motor_t * motor3 = motors[2];
	tmc2209_stepper_driver_t * driver3 = motor3->driver;

	motor3->gpio_pins.mot_en 	= MOT_EN_3_Pin;
	motor3->gpio_ports.mot_en 	= MOT_EN_3_GPIO_Port;

	tmc2209_set_hardware_enable_pin(driver3, motor3->gpio_pins.mot_en, motor3->gpio_ports.mot_en);
	tmc2209_disable(driver3);

	initializeDefaults(motor3);

	motor3->ID = '3';

	motor3->gear_ratio = GEAR_RATIO_M_3;
	motor3->microsteps = MICROSTEPS_M_3;
	motor3->motion.V_MAX 	= V_MAX_M_3;
	motor3->motion.ACC_MAX = ACC_MAX_M_3;
	motor3->motion.DEC_MAX = DEC_MAX_M_3;

	motor3->current_settings.HOLD_CURRENT_PERCENT = HOLD_CURRENT_PERCENT_M_3;
	motor3->current_settings.RUN_CURRENT_PERCENT = RUN_CURRENT_PERCENT_M_3;

	motor3->gpio_pins.step 		= STEP_3_Pin;
	motor3->gpio_pins.dir 		= DIR_3_Pin;
	motor3->gpio_pins.diag 		= DIAG_3_Pin;
	motor3->gpio_ports.step 	= STEP_3_GPIO_Port;
	motor3->gpio_ports.dir 		= DIR_3_GPIO_Port;
	motor3->gpio_ports.diag 	= DIAG_3_GPIO_Port;

	motor3->motion.motor_control_timer = htim3;
	motor3->status_check_timer = htim7;

	if (UART_MODE == MODE_5_UART)
	{
		motor3->uart = huart3;
		tmc2209_setup(driver3, 115200, SERIAL_ADDRESS_0);
	}
	else if (UART_MODE == MODE_2_UART)
	{
		motor3->uart = huart1;
		tmc2209_setup(driver3, 115200, SERIAL_ADDRESS_2);
	}

	motor3->stallguard.POSITION_LIMIT = POSITION_LIMIT_M_3;
	motor3->motion.HIGH_LIMIT = HIGH_LIMIT_M_3;
	motor3->motion.LOW_LIMIT = LOW_LIMIT_M_3;

	motor3->stallguard.MAX_CONSECUTIVE_LOW = LOW_COUNTER_THRESHOLD_M_3;
	motor3->stallguard.MAX_STALLGUARD_VALUE = STALL_MAX_M_3;
	motor3->stallguard.STALL_BUFFER = STALL_BUFFER_M_3;

	driver3->uart_ = motor3->uart;

	set_micro_steps_per_step(driver3, motor3->microsteps);
	set_all_current_percent_values(driver3, motor3->current_settings.RUN_CURRENT_PERCENT, motor3->current_settings.HOLD_CURRENT_PERCENT, 0);

}

void initMotor4(uart_mode_t UART_MODE)
{
	motor_t * motor4 = motors[3];
	tmc2209_stepper_driver_t * driver4 = motor4->driver;

	motor4->gpio_pins.mot_en 	= MOT_EN_4_Pin;
	motor4->gpio_ports.mot_en 	= MOT_EN_4_GPIO_Port;

	tmc2209_set_hardware_enable_pin(driver4, motor4->gpio_pins.mot_en, motor4->gpio_ports.mot_en);
	tmc2209_disable(driver4);

	initializeDefaults(motor4);

	motor4->ID = '4';

	motor4->gear_ratio = GEAR_RATIO_M_4;
	motor4->microsteps = MICROSTEPS_M_4;
	motor4->motion.V_MAX 	= V_MAX_M_4;
	motor4->motion.ACC_MAX = ACC_MAX_M_4;
	motor4->motion.DEC_MAX = DEC_MAX_M_4;

	motor4->current_settings.HOLD_CURRENT_PERCENT = HOLD_CURRENT_PERCENT_M_4;
	motor4->current_settings.RUN_CURRENT_PERCENT = RUN_CURRENT_PERCENT_M_4;

	motor4->gpio_pins.step 		= STEP_4_Pin;
	motor4->gpio_pins.dir 		= DIR_4_Pin;
//	motor4->gpio_pins.diag 		= DIAG_4_Pin;
	motor4->gpio_ports.step 	= STEP_4_GPIO_Port;
	motor4->gpio_ports.dir 		= DIR_4_GPIO_Port;
//	motor4->gpio_ports.diag 		= DIAG_4_GPIO_Port;

	motor4->motion.motor_control_timer = htim4;
	motor4->status_check_timer = htim9;

	if (UART_MODE == MODE_5_UART)
	{
		motor4->uart = huart4;
		tmc2209_setup(driver4, 115200, SERIAL_ADDRESS_0);
	}
	else if (UART_MODE == MODE_2_UART)
	{
		motor4->uart = huart4;
		tmc2209_setup(driver4, 115200, SERIAL_ADDRESS_0);
	}

	motor4->stallguard.POSITION_LIMIT = POSITION_LIMIT_M_4;
	motor4->motion.HIGH_LIMIT = HIGH_LIMIT_M_4;
	motor4->motion.LOW_LIMIT = LOW_LIMIT_M_4;

	motor4->stallguard.MAX_CONSECUTIVE_LOW = LOW_COUNTER_THRESHOLD_M_4;
	motor4->stallguard.MAX_STALLGUARD_VALUE = STALL_MAX_M_4;
	motor4->stallguard.STALL_BUFFER = STALL_BUFFER_M_4;

	driver4->uart_ = motor4->uart;

	set_micro_steps_per_step(driver4, motor4->microsteps);
	set_all_current_percent_values(driver4, motor4->current_settings.RUN_CURRENT_PERCENT, motor4->current_settings.HOLD_CURRENT_PERCENT, 0);

}

void initMotor5(uart_mode_t UART_MODE)
{
	motor_t * motor5 = motors[4];
	tmc2209_stepper_driver_t * driver5 = motor5->driver;

	motor5->gpio_pins.mot_en 	= MOT_EN_5_Pin;
	motor5->gpio_ports.mot_en 	= MOT_EN_5_GPIO_Port;

	tmc2209_set_hardware_enable_pin(driver5, motor5->gpio_pins.mot_en, motor5->gpio_ports.mot_en);
	tmc2209_disable(driver5);

	initializeDefaults(motor5);

	motor5->ID = '5';

	motor5->gear_ratio = GEAR_RATIO_M_5;
	motor5->microsteps = MICROSTEPS_M_5;
	motor5->motion.V_MAX 	= V_MAX_M_5;
	motor5->motion.ACC_MAX = ACC_MAX_M_5;
	motor5->motion.DEC_MAX = DEC_MAX_M_5;

	motor5->current_settings.HOLD_CURRENT_PERCENT = HOLD_CURRENT_PERCENT_M_5;
	motor5->current_settings.RUN_CURRENT_PERCENT = RUN_CURRENT_PERCENT_M_5;

	motor5->gpio_pins.step 		= STEP_5_Pin;
	motor5->gpio_pins.dir 		= DIR_5_Pin;
	motor5->gpio_ports.step 	= STEP_5_GPIO_Port;
//	motor5->gpio_pins.diag 		= DIAG_5_Pin;
	motor5->gpio_ports.dir 		= DIR_5_GPIO_Port;
//	motor5->gpio_ports.diag 		= DIAG_5_GPIO_Port;

	motor5->motion.motor_control_timer = htim8;
	motor5->status_check_timer = htim10;

	if (UART_MODE == MODE_5_UART)
	{
		motor5->uart = huart5;
		tmc2209_setup(driver5, 115200, SERIAL_ADDRESS_0);
	}
	else if (UART_MODE == MODE_2_UART)
	{
		motor5->uart = huart4;
		tmc2209_setup(driver5, 115200, SERIAL_ADDRESS_1);
	}

	motor5->stallguard.POSITION_LIMIT = POSITION_LIMIT_M_5;
	motor5->stallguard.MAX_CONSECUTIVE_LOW = LOW_COUNTER_THRESHOLD_M_5;
	motor5->stallguard.MAX_STALLGUARD_VALUE = STALL_MAX_M_5;
	motor5->stallguard.STALL_BUFFER = STALL_BUFFER_M_5;

	driver5->uart_ = motor5->uart;

	set_micro_steps_per_step(driver5, motor5->microsteps);
	set_all_current_percent_values(driver5, motor5->current_settings.RUN_CURRENT_PERCENT, motor5->current_settings.HOLD_CURRENT_PERCENT, 0);

}

void initializeDefaults(motor_t * motor)
{
	motor->motion.v = 0;
	motor->motion.total_steps = 0;
	motor->motion.const_steps = 0;
	motor->motion.acc_steps = 0;
	motor->motion.dec_steps = 0;
	motor->motion.step = 0;
	motor->motion.cycle = 0;
	motor->motion.inverse_motor_direction = 0;

	motor->active_movement_flag = 0;

	motor->status_flag = 0;

	motor->stallguard.smoothed_result = 0;
	motor->stallguard.previous_smoothed_result = 0;
	motor->stallguard.consecutive_low_counter = 0;
	motor->stallguard.stall_flag = 0;
}
