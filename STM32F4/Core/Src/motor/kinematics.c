/*
 * kinematics.c
 *
 *  Created on: Sep 16, 2025
 *      Author: jakob
 */
#include "motor/kinematics.h"
#include "motor/helper.h"
#include "math.h"
#include "display/helper.h"

#define LENGTH_SEGMENT_1 223
#define LENGTH_SEGMENT_2 160
#define LENGTH_SEGMENT_3 160
#define LENGTH_SEGMENT_4 112

#define NUMBER_JOINTS 4

extern motor_t * motors[];

void toPolar(float x, float y, float * theta_p, float * r_p)
{
	if (x < 0 && y >= 0)
		*theta_p = -atan(y/x);
	else if (x >= 0 && y > 0)
		*theta_p = atan (x/y) + M_PI/2;
	else if (x > 0 && y <= 0)
		*theta_p = -atan (y/x) + M_PI;
	else if (x <= 0 && y < 0)
		*theta_p = atan (x/y) + 3*M_PI/2;

	*r_p = sqrtf(x*x + y*y);
}

motor_error_t checkPositionLimits(float phi, float LOW_LIMIT, float HIGH_LIMIT, uint8_t id)
{
	motor_error_t error = NO_ERROR;
	if (phi < LOW_LIMIT)
	{
		HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_SET);
		char asdf[64];
		snprintf(asdf, sizeof(asdf), "phi %d is too low", id);
		writeDisplay(asdf);
		error = MOTOR_ERROR;
	}
	else if (phi > HIGH_LIMIT)
	{
		HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_SET);
		char asdf[64];
		snprintf(asdf, sizeof(asdf), "phi %d is too high", id);
		writeDisplay(asdf);
		error = MOTOR_ERROR;
	}
	return error;
}

motor_error_t calculateAngles(float phi[], float theta, float r, float z, float gripper_direction)
{
	motor_error_t error = NO_ERROR;
	gripper_direction = toRad(gripper_direction);

	z += LENGTH_SEGMENT_4 * sin(gripper_direction);
	r -= LENGTH_SEGMENT_4 * cos(gripper_direction);

	if (motors[0]->active_movement_flag ||
			motors[1]->active_movement_flag ||
			motors[2]->active_movement_flag ||
			motors[3]->active_movement_flag)
	{
		error = MOTOR_MOVING_ERROR;
		return error;
	}

	float h = abs(z-LENGTH_SEGMENT_1);
	float d = sqrtf(r*r + h*h);
	phi[0] = theta;

	phi[2] = acos(((r*r) + (h*h) - LENGTH_SEGMENT_3 * LENGTH_SEGMENT_3 - LENGTH_SEGMENT_2 * LENGTH_SEGMENT_2) / (2 * LENGTH_SEGMENT_3 * LENGTH_SEGMENT_2));

	float gamma = acos(r/d);
	float beta = atan(LENGTH_SEGMENT_3 * sin(phi[2]) / (LENGTH_SEGMENT_2 + cos(phi[2])*LENGTH_SEGMENT_3));

	if (z >= LENGTH_SEGMENT_1)
		phi[1] = M_PI/2 - gamma - beta;
	else
		phi[1] = M_PI/2 + gamma -beta;

	phi[3] = gripper_direction - (phi[1] + phi[2]) + M_PI;

	phi[0] = toGrad(phi[0]);
	phi[1] = toGrad(phi[1]);
	phi[2] = toGrad(phi[2]);
	phi[3] = toGrad(phi[3]);

	for (int i= 0; i < NUMBER_JOINTS; i++)
	{
		if (checkPositionLimits(phi[i], motors[i]->motion.LOW_LIMIT, motors[i]->motion.HIGH_LIMIT, i+1) == MOTOR_ERROR)
		{
			error = MOTOR_ERROR;
			return error;
		}
	}

	return error;
}


































