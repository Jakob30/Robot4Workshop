/*
 * motor_init.h
 *
 *  Created on: Aug 11, 2025
 *      Author: jakob
 */

#ifndef INC_MOTOR_INIT_H_
#define INC_MOTOR_INIT_H_

#include "motor_types.h"

typedef enum
{
	MODE_2_UART,
	MODE_5_UART
}uart_mode_t;

void initializeDefaults(motor_t * motor);
void initAllMotors(uart_mode_t UART_MODE);


#endif /* INC_MOTOR_INIT_H_ */
