/*
 * project.h
 *
 *  Created on: Nov 18, 2017
 *      Author: Lasitha Wijayarathne
 */

#ifndef PROJECT_H_
#define PROJECT_H_

#include "msp.h"
#include "driverlib.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


//FUNCTION PROTOTYPES


void delayMs(int);
void InitCS();
void NVICconfig();
void InitTIMER();
void InitUART();
void calVelocity();


//STATIC VARIABLES
extern uint8_t oneBit;
extern uint16_t oneBit16;
extern int CSfrq;
extern char update[300];
extern int counter;
extern float actual_joint_pos[3];
extern float actual_joint_vel[3];


#endif /* PROJECT_H_ */
