/*
 * motor_init.c
 *
 *  Created on: Aug 11, 2025
 *      Author: jakob
 */

#include "motor/motor_init.h"

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


void init_motor_1(motor_t *motor1, tmc2209_stepper_driver_t *driver1)
{
	motor1->ID = '1';

	motor1->gear_ratio = GEAR_RATIO_M_1;
	motor1->microsteps = MICROSTEPS_M_1;
	motor1->v_max 	= V_MAX_M_1;
	motor1->acc_max = ACC_MAX_M_1;
	motor1->dec_max = DEC_MAX_M_1;

	motor1->v = 0;
	motor1->total_steps = 0;
	motor1->const_steps = 0;
	motor1->acc_steps = 0;
	motor1->dec_steps = 0;
	motor1->step = 0;
	motor1->cycle = 0;

	motor1->active_movement_flag = 0;

	motor1->step_at_stall = 0;
	motor1->dec_steps_after_stall = 0;
	motor1->stall_flag = 0;
	motor1->status_flag = 0;

	motor1->hold_current_percent = HOLD_CURRENT_PERCENT_M_1;
	motor1->run_current_percent = RUN_CURRENT_PERCENT_M_1;

	motor1->gpio_pins.step 		= STEP_1_Pin;
	motor1->gpio_pins.dir 		= DIR_1_Pin;
	motor1->gpio_pins.mot_en 	= MOT_EN_1_Pin;
	motor1->gpio_ports.step 	= STEP_1_GPIO_Port;
	motor1->gpio_ports.dir 		= DIR_1_GPIO_Port;
	motor1->gpio_ports.mot_en 	= MOT_EN_1_GPIO_Port;

	motor1->motor_control_timer = htim12;
	motor1->status_check_timer = htim1;
	motor1->uart = huart1;

	motor1->position_limit = POSITION_LIMIT_M_1;
	motor1->stallGuard_threshold = STALLGUARD_THRESHOLD_M_1;

	tmc2209_set_uart(driver1, huart1);
	tmc2209_set_hardware_enable_pin(driver1, motor1->gpio_pins.mot_en, motor1->gpio_ports.mot_en);
	tmc2209_setup(driver1, 115200, SERIAL_ADDRESS_0);
	set_micro_steps_per_step(driver1, motor1->microsteps);
	set_all_current_percent_values(driver1, motor1->run_current_percent, motor1->hold_current_percent, 0);

	motor1->driver = driver1;
}

void init_motor_2(motor_t *motor2, tmc2209_stepper_driver_t *driver2)
{
	motor2->ID = '2';

	motor2->gear_ratio = GEAR_RATIO_M_2;
	motor2->microsteps = MICROSTEPS_M_2;
	motor2->v_max 	= V_MAX_M_2;
	motor2->acc_max = ACC_MAX_M_2;
	motor2->dec_max = DEC_MAX_M_2;

	motor2->v = 0;
	motor2->total_steps = 0;
	motor2->const_steps = 0;
	motor2->acc_steps = 0;
	motor2->dec_steps = 0;
	motor2->step = 0;
	motor2->cycle = 0;

	motor2->active_movement_flag = 0;

	motor2->step_at_stall = 0;
	motor2->dec_steps_after_stall = 0;
	motor2->stall_flag = 0;
	motor2->status_flag = 0;

	motor2->hold_current_percent = HOLD_CURRENT_PERCENT_M_2;
	motor2->run_current_percent = RUN_CURRENT_PERCENT_M_2;

	motor2->gpio_pins.step 		= STEP_2_Pin;
	motor2->gpio_pins.dir 		= DIR_2_Pin;
	motor2->gpio_pins.mot_en 	= MOT_EN_2_Pin;
	motor2->gpio_ports.step 	= STEP_2_GPIO_Port;
	motor2->gpio_ports.dir 		= DIR_2_GPIO_Port;
	motor2->gpio_ports.mot_en 	= MOT_EN_2_GPIO_Port;

	motor2->motor_control_timer = htim13;
	motor2->status_check_timer = htim6;

	motor2->uart = huart6;

	motor2->position_limit = POSITION_LIMIT_M_2;
	motor2->stallGuard_threshold = STALLGUARD_THRESHOLD_M_2;

	tmc2209_set_uart(driver2, motor2->uart);
	tmc2209_set_hardware_enable_pin(driver2, motor2->gpio_pins.mot_en, motor2->gpio_ports.mot_en);
	tmc2209_setup(driver2, 115200, SERIAL_ADDRESS_0);
//	disable_stealth_chop(motor2->driver);
	set_micro_steps_per_step(driver2, motor2->microsteps);
	set_all_current_percent_values(driver2, motor2->run_current_percent, motor2->hold_current_percent, 0);

	motor2->driver = driver2;
}

void init_motor_3(motor_t *motor3, tmc2209_stepper_driver_t *driver3)
{
	motor3->ID = '3';

	motor3->gear_ratio = GEAR_RATIO_M_3;
	motor3->microsteps = MICROSTEPS_M_3;
	motor3->v_max 	= V_MAX_M_3;
	motor3->acc_max = ACC_MAX_M_3;
	motor3->dec_max = DEC_MAX_M_3;

	motor3->v = 0;
	motor3->total_steps = 0;
	motor3->const_steps = 0;
	motor3->acc_steps = 0;
	motor3->dec_steps = 0;
	motor3->step = 0;
	motor3->cycle = 0;

	motor3->active_movement_flag = 0;

	motor3->step_at_stall = 0;
	motor3->dec_steps_after_stall = 0;
	motor3->stall_flag = 0;
	motor3->status_flag = 0;

	motor3->hold_current_percent = HOLD_CURRENT_PERCENT_M_3;
	motor3->run_current_percent = RUN_CURRENT_PERCENT_M_3;

	motor3->gpio_pins.step 		= STEP_3_Pin;
	motor3->gpio_pins.dir 		= DIR_3_Pin;
	motor3->gpio_pins.mot_en 	= MOT_EN_3_Pin;
	motor3->gpio_ports.step 	= STEP_3_GPIO_Port;
	motor3->gpio_ports.dir 		= DIR_3_GPIO_Port;
	motor3->gpio_ports.mot_en 	= MOT_EN_3_GPIO_Port;

	motor3->motor_control_timer = htim3;
	motor3->status_check_timer = htim7;

	motor3->uart = huart3;

	motor3->position_limit = POSITION_LIMIT_M_3;
	motor3->stallGuard_threshold = STALLGUARD_THRESHOLD_M_3;

	tmc2209_set_uart(driver3, huart3);
	tmc2209_set_hardware_enable_pin(driver3, motor3->gpio_pins.mot_en, motor3->gpio_ports.mot_en);
	tmc2209_setup(driver3, 115200, SERIAL_ADDRESS_0);
	set_micro_steps_per_step(driver3, motor3->microsteps);
	set_all_current_percent_values(driver3, motor3->run_current_percent, motor3->hold_current_percent, 0);

	motor3->driver = driver3;
}

void init_motor_4(motor_t *motor4, tmc2209_stepper_driver_t *driver4)
{
	motor4->ID = '4';

	motor4->gear_ratio = GEAR_RATIO_M_4;
	motor4->microsteps = MICROSTEPS_M_4;
	motor4->v_max 	= V_MAX_M_4;
	motor4->acc_max = ACC_MAX_M_4;
	motor4->dec_max = DEC_MAX_M_4;

	motor4->v = 0;
	motor4->total_steps = 0;
	motor4->const_steps = 0;
	motor4->acc_steps = 0;
	motor4->dec_steps = 0;
	motor4->step = 0;
	motor4->cycle = 0;

	motor4->active_movement_flag = 0;

	motor4->step_at_stall = 0;
	motor4->dec_steps_after_stall = 0;
	motor4->stall_flag = 0;
	motor4->status_flag = 0;

	motor4->hold_current_percent = HOLD_CURRENT_PERCENT_M_4;
	motor4->run_current_percent = RUN_CURRENT_PERCENT_M_4;

	motor4->gpio_pins.step 		= STEP_4_Pin;
	motor4->gpio_pins.dir 		= DIR_4_Pin;
	motor4->gpio_pins.mot_en 	= MOT_EN_4_Pin;
	motor4->gpio_ports.step 	= STEP_4_GPIO_Port;
	motor4->gpio_ports.dir 		= DIR_4_GPIO_Port;
	motor4->gpio_ports.mot_en 	= MOT_EN_4_GPIO_Port;

	motor4->motor_control_timer = htim4;
	motor4->status_check_timer = htim9;

	motor4->uart = huart4;

	motor4->position_limit = POSITION_LIMIT_M_4;
	motor4->stallGuard_threshold = STALLGUARD_THRESHOLD_M_4;

	tmc2209_set_uart(driver4, huart4);
	tmc2209_set_hardware_enable_pin(driver4, motor4->gpio_pins.mot_en, motor4->gpio_ports.mot_en);
	tmc2209_setup(driver4, 115200, SERIAL_ADDRESS_0);
	enable_cool_step(driver4, 0, 1);
	set_micro_steps_per_step(driver4, motor4->microsteps);
	set_all_current_percent_values(driver4, motor4->run_current_percent, motor4->hold_current_percent, 0);

	motor4->driver = driver4;
}

void init_motor_5(motor_t *motor5, tmc2209_stepper_driver_t *driver5)
{
	motor5->ID = '5';

	motor5->gear_ratio = GEAR_RATIO_M_5;
	motor5->microsteps = MICROSTEPS_M_5;
	motor5->v_max 	= V_MAX_M_5;
	motor5->acc_max = ACC_MAX_M_5;
	motor5->dec_max = DEC_MAX_M_5;

	motor5->v = 0;
	motor5->total_steps = 0;
	motor5->const_steps = 0;
	motor5->acc_steps = 0;
	motor5->dec_steps = 0;
	motor5->step = 0;
	motor5->cycle = 0;

	motor5->active_movement_flag = 0;

	motor5->step_at_stall = 0;
	motor5->dec_steps_after_stall = 0;
	motor5->stall_flag = 0;
	motor5->status_flag = 0;

	motor5->hold_current_percent = HOLD_CURRENT_PERCENT_M_5;
	motor5->run_current_percent = RUN_CURRENT_PERCENT_M_5;

	motor5->gpio_pins.step 		= STEP_5_Pin;
	motor5->gpio_pins.dir 		= DIR_5_Pin;
	motor5->gpio_pins.mot_en 	= MOT_EN_5_Pin;
	motor5->gpio_ports.step 	= STEP_5_GPIO_Port;
	motor5->gpio_ports.dir 		= DIR_5_GPIO_Port;
	motor5->gpio_ports.mot_en 	= MOT_EN_5_GPIO_Port;

	motor5->motor_control_timer = htim8;
	motor5->status_check_timer = htim10;

	motor5->uart = huart5;

	motor5->position_limit = POSITION_LIMIT_M_5;
	motor5->stallGuard_threshold = STALLGUARD_THRESHOLD_M_5;

	tmc2209_set_uart(driver5, huart5);
	tmc2209_set_hardware_enable_pin(driver5, motor5->gpio_pins.mot_en, motor5->gpio_ports.mot_en);
	tmc2209_setup(driver5, 115200, SERIAL_ADDRESS_0);
	set_micro_steps_per_step(driver5, motor5->microsteps);
	set_all_current_percent_values(driver5, motor5->run_current_percent, motor5->hold_current_percent, 0);

	motor5->driver = driver5;
}

