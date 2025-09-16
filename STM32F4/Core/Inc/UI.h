/*
 * UI.h
 *
 *  Created on: Sep 10, 2025
 *      Author: jakob
 */

#ifndef INC_UI_H_
#define INC_UI_H_

#include "motor/motor_types.h"

typedef enum
{
	MOVE_DEGREES,
	MOVE_ABSOLUTE,
	MOVE_POLAR,
	MOVE_TO_COORDINATES,
	GO_HOME,
	OPEN_GRIPPER,
	GRIP
}call_t;

typedef struct
{
	float param1;
	float param2;
	float param3;
	uint8_t param4;
}params_t;

typedef struct
{
	uint8_t sync;
	call_t call;
	params_t data;
	uint8_t crc;
}call_msg_t;



#endif /* INC_UI_H_ */
