/*
 * status_check.h
 *
 *  Created on: Aug 7, 2025
 *      Author: jakob
 */

#ifndef INC_STATUS_CHECK_H_
#define INC_STATUS_CHECK_H_

#include "tmc2209.h"
#include "helper.h"
#include "main.h"

//continuously check status of current acting driver
//every 100 ms or so
//setup timer for this matter with interrupt
//maybe look into priorities so motor movement is steady and only interrupted if serious problems exist
//check load and temperature
//think about what to do if a certain temperature level or load level is reached


void checkDriverStatus(tmc2209_stepper_driver_t* driver);

#endif /* INC_STATUS_CHECK_H_ */
