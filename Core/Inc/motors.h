

#ifndef __MOTORS_H
#define __MOTORS_H


#include <stdio.h>
#include "stm32l4xx_hal.h"


#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR4 4


// PA10 motor1 front right
// PA11 motor2 rear right
// PB14 motor3 rear left
// PB15 motor4 front left

void DriveMotor(int motor, uint32_t value);
void DriveMotors(uint32_t motor1,uint32_t motor2,uint32_t motor3,uint32_t motor4);


#endif
