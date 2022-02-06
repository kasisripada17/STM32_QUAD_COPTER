#include "motors.h"

void DriveMotor(int motor, uint32_t value) {
	uint32_t motorValue = map(value, 1000, 2000, 800, 1600);
	if (motor == MOTOR1) {
		TIM1->CCR3 = motorValue;
	} else if (motor == MOTOR2) {
		TIM1->CCR4 = motorValue;
	} else if (motor == MOTOR3) {
		TIM15->CCR1 = motorValue;
	} else if (motor == MOTOR4) {
		TIM15->CCR2 = motorValue;
	}
}

void DriveMotors(uint32_t motor1,uint32_t motor2,uint32_t motor3,uint32_t motor4) {

	uint32_t motorValue1 = map(motor1, 1000, 2000, 800, 1600);
	uint32_t motorValue2 = map(motor2, 1000, 2000, 800, 1600);
	uint32_t motorValue3 = map(motor3, 1000, 2000, 800, 1600);
	uint32_t motorValue4 = map(motor4, 1000, 2000, 800, 1600);

	TIM1->CCR3  =  motorValue1;
	TIM1->CCR4  =  motorValue2;
	TIM15->CCR1 =  motorValue3;
	TIM8->CCR4 =  motorValue4;
}
