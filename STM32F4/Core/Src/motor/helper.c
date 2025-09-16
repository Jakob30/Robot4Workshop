/*
 * helper.c
 *
 *  Created on: Sep 16, 2025
 *      Author: jakob
 */
#include "motor/helper.h"
#include "math.h"

float toGrad(float degrees_rad)
{
	return degrees_rad * 180/M_PI;
}

float toRad(float degrees_grad)
{
	return degrees_grad * M_PI/180;
}

/*
 * Calculates the steps needed to rotate the amount stated in the variable degrees.
 */
int32_t toSteps(float degrees, motor_t* motor)
{
	int32_t steps;
	steps = ((200.0 * (float)(motor->microsteps)/360.0)*degrees) * motor->gear_ratio;
	return steps;
}


