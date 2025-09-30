/*
 * motion.motor_control.c
 *  Created on: Jul 14, 2025
 *      Author: jakob
 */
#include "motor/motor_control.h"
#include "motor/status_check.h"
#include "math.h"
#include "display/helper.h"
#include "motor/motor_init.h"
#include "motor/helper.h"
#include "motor/kinematics.h"

//Globals
extern motor_t *motors[]; //To gain access to motor variables in interrupt service routine

#define NUMBER_OF_MOTOR 5


void initMovementVars(motor_t * motor, motion_mode_t motion_mode);
motor_error_t startMovement(motor_t * motor);
void stopMotorMovement(motor_t * motor);
static inline void trapezMove(motion_t * mt);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
motor_error_t moveGripper(gripper_close_open_t direction);
void toggle_inverse_motor_direction(tmc2209_stepper_driver_t* stepper_driver);


void initMovementVars(motor_t * motor, motion_mode_t motion_mode)
{
	motor->motion.v = 0;
	motor->motion.step = 0;
	motor->motion.cycle = 0;
	motor->motion.motion_mode = motion_mode;
	motor->stallguard.stall_flag = 0;
}

/*
 * Starts the timer to start interrupts which are necessary for the motor movement.
 */
motor_error_t startMovement(motor_t * motor)
{
	HAL_StatusTypeDef status;
	motor_error_t error = NO_ERROR;

	HAL_GPIO_WritePin(motor->gpio_ports.step, motor->gpio_pins.step, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&motor->motion.motor_control_timer, TIM_CHANNEL_1, 1);
	status = HAL_TIM_OC_Start_IT(&motor->motion.motor_control_timer, TIM_CHANNEL_1);

	if (status != HAL_OK)
		error = MOTOR_ERROR;

	motor->active_movement_flag = 1;

	return error;
}

/*
 * Stops the motor movement by stopping the timer.
 */
void stopMotorMovement(motor_t * motor)
{
	HAL_TIM_OC_Stop_IT(&motor->motion.motor_control_timer, TIM_CHANNEL_1);
	motor->active_movement_flag = 0;
	initializeDefaults(motor);
}

/*
 * Trapezoidal motion profile (TrapezMove) – physical derivation
 *
 * The motion is divided into 3 phases:
 *   1) Acceleration: from v=0 to v_max with constant acceleration a_acc
 *   2) Constant velocity: v = v_max (if total distance is large enough)
 *   3) Deceleration: from v_max back to 0 with constant deceleration a_dec
 *
 * Fundamental kinematic relations:
 *   v = a * t
 *   s = 0.5 * a * t^2
 *   v^2 = 2 * a * s    (key equation for ramps)
 *
 * Acceleration phase:
 *   t_acc = v_max / a_acc
 *   s_acc = v_max^2 / (2 * a_acc)
 *
 * Deceleration phase:
 *   t_dec = v_max / a_dec
 *   s_dec = v_max^2 / (2 * a_dec)
 *
 * Constant velocity phase (if present. It won't be present if total distance isn't large enough):
 *   s_const = s_total - (s_acc + s_dec)
 *
 * If s_const >= 0 → trapezoidal profile (true trapez)
 * If s_const < 0  → triangular profile (no constant section)
 *
 * For the triangular profile:
 *   s_total = s_acc + s_dec
 *   s_acc = v_peak^2 / (2 * a_acc)
 *   s_dec = v_peak^2 / (2 * a_dec)
 *
 * Solve for v_peak:
 *   v_peak = sqrt( (2 * a_acc * a_dec) / (a_acc + a_dec) * s_total )
 *
 * Summary of implementation:
 *   - Precompute acc_steps, dec_steps, const_steps based on desired v_max, a_acc, a_dec.
 *   - If const_steps < 0, recalculate acc_steps, dec_steps using v_peak and set const_steps=0.
 *   - Online velocity update:
 *       Accel: v = sqrt(2 * a_acc * s)
 *       Const: v = v_max
 *       Decel: v = sqrt(2 * a_dec * s_remaining)
 */
static inline void trapezMove(motion_t* mt)
{
	if (mt->step >= 0 && mt->step < mt->acc_steps)
	{
		mt->v = sqrtf(2 * mt->ACC_MAX * (mt->step + 1));
		// motion.v = acc_ramp[motion.step]
	}
	else if (mt->const_steps != 0 && mt->step >= mt->acc_steps && mt->step < (mt->total_steps - mt->dec_steps))
		mt->v = mt->V_MAX;
	else if (mt->step >= (mt->total_steps - mt->dec_steps) && mt->step < mt->total_steps)
	{
		mt->v = sqrtf(2 * mt->DEC_MAX * (mt->total_steps - mt->step));
		// motion.v = acc_ramp[motion.total_steps - motion.step]
	}
}
/*
 *
 * Interrupt service routine for timer in output compare mode.
 * Timer counts until compare value is reached.
 * When value is reached, the GPIO toggles.
 * Every other motion.cycle, as motion.step only triggers on rising edge,
 * the velocity and the compare value is changed depending on current state of velocity ramp.
 *
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	int8_t index;
	motor_t* motor;

	//To know which timer and thus which motor caused the interrupt
	if (htim->Instance == motors[0]->motion.motor_control_timer.Instance){ index = 0; }
	else if (htim->Instance == motors[1]->motion.motor_control_timer.Instance){ index = 1; }
	else if (htim->Instance == motors[2]->motion.motor_control_timer.Instance){ index = 2; }
	else if (htim->Instance == motors[3]->motion.motor_control_timer.Instance){ index = 3; }
	else if (htim->Instance == motors[4]->motion.motor_control_timer.Instance){ index = 4; }

	motor = motors[index];
	motion_t* mt = &motor->motion;

	//Stop timer and movement if the robot reaches its destination
	if (mt->motion_mode == MOTION_TRAPEZ && mt->step >= mt->total_steps)
	{
		stopMotorMovement(motor);
		return;
	}

	if (mt->cycle % 2 == 0) //Change velocity only every other cycle because step only triggers on rising edge
	{
		switch(mt->motion_mode)
		{
		case MOTION_TRAPEZ:
			trapezMove(mt);
			break;
		case MOTION_HOME:	//Since we don't know the exact distance to move in these 2 following cases, there's no deceleration
		case MOTION_GRIP:
			if (mt->step >= 0 && mt->step < mt->acc_steps)
				mt->v = sqrtf(2 * mt->ACC_MAX * (mt->step + 1));
			else
				mt->v = mt->V_MAX;
			break;
		}
		mt->step++;
		//Track the position of motors in steps.
		if (mt->inverse_motor_direction)
			mt->position--;
		else
			mt->position++;

	}

	mt->cycle++;

	HAL_GPIO_TogglePin(motor->gpio_ports.step, motor->gpio_pins.step);

	/*
	 * 	To reach the desired speed, we need to calculate the delay between two toggles.
	 *	v is in µsteps/s or just 1/s because one µstep is one period.
	 *	For one period, the duration is 1/v.
	 *	Between two toggles, it is 1/(2*v).
	 *	But this is not the answer since the time has to be converted into timer ticks.
	 *	The timer runs at 2 MHz so we need to divide our current period duration by 1 / 2000000 s or 0.5 µs.
	 *	-> delay in ticks = 1/(2*v)/0.0000005 = 2000000/(2*v)
	 */

	int32_t delay = 2000000 / (2 * mt->v);
	//Add delay to current compare value in register
	int32_t total_delay = __HAL_TIM_GET_COMPARE(&mt->motor_control_timer, TIM_CHANNEL_1) + delay;
	__HAL_TIM_SET_COMPARE(&mt->motor_control_timer, TIM_CHANNEL_1, total_delay);
}

/*
 * Interrupt service routine for status_check_timers, which periodically invokes status checks.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	for(int i = 0; i < NUMBER_OF_MOTOR; i++)
	{
		if (htim->Instance == motors[i]->status_check_timer.Instance)
		{
			if (motors[i]->active_movement_flag)
				motors[i]->status_flag = 1;
			else
				HAL_TIM_Base_Stop_IT(&motors[i]->status_check_timer);

			break;
		}
	}
}

/*
 * Moves a motor by degrees but in absolute matters.
 * When a motor already moved 40 degrees from its starting position,
 * calling this function again with 40 degrees will not move the motor since the motor is already at 40 degrees.
 */
motor_error_t moveAbsolute(float degrees, motor_t* motor)
{
	motor_error_t error = NO_ERROR;
	if (motor->active_movement_flag)
	{
		error = MOTOR_MOVING_ERROR;
		return error;
	}
	if (degrees <= 0)
	{
		error = MOTOR_ERROR;
		return error;
	}

	//Enable driver if not already enabled.
	if (HAL_GPIO_ReadPin(motor->gpio_ports.mot_en, motor->gpio_pins.mot_en) == GPIO_PIN_SET)
		tmc2209_enable(motor->driver);

	int actualSteps = toSteps(degrees, motor);

	//Based on current position, the motor is moved in different directions.
	if (actualSteps > motor->motion.position)
	{
		motor->motion.inverse_motor_direction = 0;
		enable_inverse_motor_direction(motor->driver);
		//Calculate the steps needed to go to target angle.
		motor->motion.total_steps = actualSteps - motor->motion.position;
	}
	else
	{
		motor->motion.inverse_motor_direction = 1;
		disable_inverse_motor_direction(motor->driver);
		motor->motion.total_steps = motor->motion.position - actualSteps;
	}

	//Calculating steps for trapez motion profile.
	motor->motion.acc_steps = (motor->motion.V_MAX * motor->motion.V_MAX) / (2 * motor->motion.ACC_MAX); //Calculate total acceleration and deceleration steps
	motor->motion.dec_steps = (motor->motion.V_MAX * motor->motion.V_MAX) / (2 * motor->motion.DEC_MAX);
	motor->motion.const_steps = motor->motion.total_steps - (motor->motion.acc_steps + motor->motion.dec_steps);

	motion_mode_t motion_mode = MOTION_TRAPEZ;

	initMovementVars(motor, motion_mode);
	//If acceleration steps + deceleration steps are bigger than total steps, do a triangle like movement.
	if (motor->motion.const_steps < 0)
	{
		motor->motion.acc_steps = motor->motion.total_steps / 2;
		motor->motion.dec_steps = motor->motion.total_steps / 2;
		motor->motion.const_steps = 0;
	}

	//Start timer in output compare with interrupt
	error = startMovement(motor);

	error = startStatusChecks(motor);

	return error;
}

/*
 * Initiates motor movement by:
 *   - enabling the driver if needed
 *   - converting angle to total steps
 *   - calculating acc_steps, dec_steps, const_steps
 *   - adjusting to triangle profile if total steps are too short
 *   - starting the timer for output compare with interrupt
 */
motor_error_t moveDegrees(float degrees, motor_t* motor)
{
	motor_error_t error = NO_ERROR;
	if (motor->active_movement_flag)
	{
		error = MOTOR_MOVING_ERROR;
		return error;
	}

	if (HAL_GPIO_ReadPin(motor->gpio_ports.mot_en, motor->gpio_pins.mot_en) == GPIO_PIN_SET)
		tmc2209_enable(motor->driver);

	if (fabs(degrees) < 1e-5)
	{
		error = MOTOR_ERROR;
	    return error;
	}
	else if (degrees < -1e-5)
	{
		enable_inverse_motor_direction(motor->driver);
		motor->motion.inverse_motor_direction = 0;
		degrees = degrees * (-1);
	}
	else if (degrees > 1e-5)
	{
		disable_inverse_motor_direction(motor->driver);
		motor->motion.inverse_motor_direction = 1;
	}


	motor->motion.total_steps = toSteps(degrees, motor); //Convert degrees to steps
	motor->motion.acc_steps = (motor->motion.V_MAX * motor->motion.V_MAX) / (2 * motor->motion.ACC_MAX); //Calculate total acceleration and deceleration steps
	motor->motion.dec_steps = (motor->motion.V_MAX * motor->motion.V_MAX) / (2 * motor->motion.DEC_MAX);
	motor->motion.const_steps = motor->motion.total_steps - (motor->motion.acc_steps + motor->motion.dec_steps);

	motion_mode_t motion_mode = MOTION_TRAPEZ;

	initMovementVars(motor, motion_mode);

	if (motor->motion.const_steps < 0)  // If acceleration steps + deceleration steps exceed total steps -> trapezoid not possible -> triangle profile
	{
	    // Compute peak velocity based on available distance (triangle case)
	    float v_peak = sqrtf((2.0f * motor->motion.ACC_MAX * motor->motion.DEC_MAX) /
	                         (motor->motion.ACC_MAX + motor->motion.DEC_MAX) *
	                         (float)motor->motion.total_steps);

	    // Recalculate acceleration and deceleration steps for triangle profile
	    motor->motion.acc_steps = (uint32_t)(v_peak * v_peak / (2.0f * motor->motion.ACC_MAX));
	    motor->motion.dec_steps = motor->motion.total_steps - motor->motion.acc_steps;
	    motor->motion.const_steps = 0;

	}

	//Start timer in output compare with interrupt
	error = startMovement(motor);

	error = startStatusChecks(motor);

	return error;
}

motor_error_t movePolar(float theta, float r, float z, float gripper_direction)
{
	float phi[4] = {0, 0, 0, 0};
	motor_error_t error;

	error = calculateAngles(phi, theta, r, z, gripper_direction);
	if (error == MOTOR_ERROR)
		while(1);


	moveAbsolute(phi[0], motors[0]);
	moveAbsolute(phi[1], motors[1]);
	moveAbsolute(phi[2], motors[2]);
	moveAbsolute(phi[3], motors[3]);
	return error;
}
//
//void moveToCoordinates_(float x, float y, float z, float gripper_direction)
//{
//	error = moveToCoordinates(x, y, z, gripper_direction);
//	if (error == MOTOR_ERROR || error == MOTOR_MOVING_ERROR)
//		while(1);
//}

motor_error_t moveToCoordinates(float x, float y, float z, float gripper_direction)
{
	float theta;
	float r;

	motor_error_t error;
	toPolar(x, y, &theta, &r);
	error = movePolar(theta, r, z, gripper_direction);
	if (error == NO_ERROR)
	{
		char movingMsg[64];
		snprintf(movingMsg, sizeof(movingMsg), "moving to %d x  %d y %d z", (int)x, (int)y, (int)z);
		writeDisplay(movingMsg);
	}
 	while(motors[0] -> active_movement_flag ||
						motors[1] -> active_movement_flag ||
						motors[2] -> active_movement_flag ||
						motors[3] -> active_movement_flag )
  	{
  		checkDriverStatus(motors[0]);
  		checkDriverStatus(motors[1]);
  		checkDriverStatus(motors[2]);
  		checkDriverStatus(motors[3]);
  	}
	return error;
}

motor_error_t openGripper()
{
	motor_error_t error = NO_ERROR;
	error = moveGripper(OPEN);
	return error;
}

motor_error_t grip()
{
	motor_error_t error = NO_ERROR;
	error = moveGripper(CLOSE);
	return error;
}

motor_error_t moveGripper(gripper_close_open_t direction)
{
	motor_error_t error = NO_ERROR;
	motor_t * motor5 = motors[4];
	if (motor5->active_movement_flag)
	{
		error = MOTOR_MOVING_ERROR;
		return error;
	}

	if (direction == CLOSE)
	{
		enable_inverse_motor_direction(motor5->driver);
		motor5->motion.inverse_motor_direction = 0;
	}
	else if (direction == OPEN)
	{
		disable_inverse_motor_direction(motor5->driver);
		motor5->motion.inverse_motor_direction = 1;
	}
	else
	{
		error = MOTOR_ERROR;
		return error;
	}

	if (HAL_GPIO_ReadPin(motor5->gpio_ports.mot_en, motor5->gpio_pins.mot_en) == GPIO_PIN_SET)
		tmc2209_enable(motor5->driver);

	motor5->motion.acc_steps = (motor5->motion.V_MAX * motor5->motion.V_MAX) / (2 * motor5->motion.ACC_MAX); //Calculate total acceleration steps

	motion_mode_t motion_mode = MOTION_GRIP;
	initMovementVars(motor5, motion_mode);

	error = startMovement(motor5);
	error = startStatusChecks(motor5);
	while (motor5->active_movement_flag)
	{
		checkDriverStatus(motor5);
	}
	return error;
}

motor_error_t goHome()
{
	motor_error_t error = NO_ERROR;

	if (motors[0]->active_movement_flag ||
			motors[1]->active_movement_flag ||
			motors[2]->active_movement_flag ||
			motors[3]->active_movement_flag ||
			motors[4]->active_movement_flag)
	{
		error = MOTOR_MOVING_ERROR;
		return error;
	};

	writeDisplay("Homing...");

	for (int i = 0; i < NUMBER_OF_MOTOR; i++)
	{
		motor_t * motor = motors[i];
//		if (i == 3)
//			continue;

		motion_mode_t motion_mode = MOTION_HOME;

		if (HAL_GPIO_ReadPin(motor->gpio_ports.mot_en, motor->gpio_pins.mot_en) == GPIO_PIN_SET)
			tmc2209_enable(motor->driver);

		motor->motion.acc_steps = (motor->motion.V_MAX * motor->motion.V_MAX) / (2 * motor->motion.ACC_MAX); //Calculate total acceleration steps

		initMovementVars(motor, motion_mode);

		error = startMovement(motor);
		error = startStatusChecks(motor);
	}

	while(motors[0]->stallguard.stall_flag == 0
			|| motors[1]->stallguard.stall_flag == 0
			|| motors[2]->stallguard.stall_flag == 0
			|| motors[3]->stallguard.stall_flag == 0
			|| motors[4]->stallguard.stall_flag == 0) //If there is no flag and motors[4]->stallguard.stall_flag == 1 (and every other motors already stalled),
										// while loop breaks -> so openGripper() isn't called. It make sure even if motor 5 is the last one stalled, openGripper() always called.
	{
		checkAllDrivers();
	}

	writeDisplay("Homing finished");

	motors[0]->stallguard.stall_flag = 0;
	motors[1]->stallguard.stall_flag = 0;
	motors[2]->stallguard.stall_flag = 0;
	motors[3]->stallguard.stall_flag = 0;
	motors[4]->stallguard.stall_flag = 0;

	motors[0]->motion.position = 1245;
	motors[1]->motion.position = 0;
	motors[2]->motion.position = 0;
	motors[3]->motion.position = 0;
	motors[4]->motion.position = 0;

	writeDisplay("Homing finished");

	return error;
}

void toggle_inverse_motor_direction(tmc2209_stepper_driver_t *stepper_driver)
{
  stepper_driver->global_config_.shaft = 1 - stepper_driver->global_config_.shaft;
  write_stored_global_config(stepper_driver);
}

