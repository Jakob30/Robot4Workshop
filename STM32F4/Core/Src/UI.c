/*
 * UI.c
 *
 *  Created on: Sep 10, 2025
 *      Author: jakob
 */

#include "UI.h"
#include "display/helper.h"

#define FRAME_SIZE 16
#define START_BYTE 0xAA

extern motor_t * motors[];

uint8_t datagram[FRAME_SIZE];

uint8_t crc8_xor(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];   // einfach alle Bytes XORen
    }
    return crc;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		writeDisplay("Hello World");
//		char feedback_msg[64];
//
//		if (datagram[0] != START_BYTE)
//		{
//			snprintf(feedback_msg, sizeof(feedback_msg), "wrong start byte");
//			writeDisplay(feedback_msg);
//			HAL_UART_Receive_IT(huart, datagram, 10);
//			return;
//		}
//
//		uint8_t crc = crc8_xor(&datagram[1], FRAME_SIZE - 2);
//		if (crc != datagram[FRAME_SIZE - 1])
//		{
//			snprintf(feedback_msg, sizeof(feedback_msg), "wrong crc");
//			writeDisplay(feedback_msg);
//			HAL_UART_Receive_IT(huart, datagram, 10);
//			return;
//		}
//
//		call_msg_t msg;
//		msg.call = (call_t) datagram[1];
//		memcpy(&msg.data.param1, &datagram[2], 4);
//		memcpy(&msg.data.param2, &datagram[6], 4);
//		memcpy(&msg.data.param3, &datagram[10], 4);
//		msg.data.param4 = datagram[14];
//
//		uint8_t motor_id;
//		float z;
//		gripper_direction_t gripper_direction;
//
//		switch (msg.call)
//		{
//		case MOVE_DEGREES:
//			float degrees = msg.data.param1;
//			motor_id = msg.data.param4;
//			snprintf(feedback_msg, sizeof(feedback_msg), "moving %d degrees", (int)degrees);
//			moveDegrees(degrees, motors[motor_id]);
//			break;
//		case MOVE_ABSOLUTE:
//			float absolute_degrees = msg.data.param1;
//			motor_id = msg.data.param4;
//			snprintf(feedback_msg, sizeof(feedback_msg), "moving %d degrees absolute", (int)absolute_degrees);
//			moveAbsolute(absolute_degrees, motors[motor_id]);
//			break;
//		case MOVE_POLAR:
//			float theta = msg.data.param1;
//			float r = msg.data.param2;
//			z = msg.data.param3;
//			gripper_direction = (gripper_direction_t) msg.data.param4;
//			snprintf(feedback_msg, sizeof(feedback_msg), "moving to %d theta %d r %d z", (int)theta, (int)r, (int)z);
//			movePolar(theta, r, z, gripper_direction);
//			break;
//		case MOVE_TO_COORDINATES:
//			float x = msg.data.param1;
//			float y = msg.data.param2;
//			z = msg.data.param3;
//			gripper_direction = (gripper_direction_t) msg.data.param4;
//			snprintf(feedback_msg, sizeof(feedback_msg), "moving to %d x %d y %d z", (int)x, (int)y, (int)z);
//			moveToCoordinates(x, y, z, gripper_direction);
//			break;
//		case GO_HOME:
//			snprintf(feedback_msg, sizeof(feedback_msg), "going home");
//			goHome();
//			break;
//		case OPEN_GRIPPER:
//			snprintf(feedback_msg, sizeof(feedback_msg), "opening gripper");
//			openGripper();
//			break;
//		case GRIP:
//			snprintf(feedback_msg, sizeof(feedback_msg), "gripping");
//			grip();
//			break;
//		default:
//			snprintf(feedback_msg, sizeof(feedback_msg), "wrong call id");
//		}
//
//		writeDisplay(feedback_msg);
//		HAL_UART_Receive_IT(huart, datagram, 10);
	}


}

