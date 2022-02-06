
#include "pid.h"




extern float pid_roll_output ;
extern float pid_pitch_output ;
extern float pid_yaw_output ;

extern float iErrorRoll;
extern float rollPreviousInput;

extern float iErrorPitch ;
extern float pitchPreviousInput ;

extern float iErrorYaw;
extern float yawPreviousInput;

extern float rollSetpoint;
extern float pitchSetpoint ;
extern float yawSetpoint;

extern float rollInput ;
extern float pitchInput ;
extern float yawInput ;




extern float pidRollError ;
extern float pidPitchError;
extern float pidYawError ;




/*

 void calculate_pid_output(void) {


	pidRollError =   rollInput - rollSetpoint;

	iErrorRoll += pidRollError;

	if (iErrorRoll > pid_max) {
		iErrorRoll = pid_max;
	} else if (iErrorRoll < (pid_max * -1) ) {
		iErrorRoll = (pid_max * -1);
	}
//	if(fabs(rollInput)> 500.0f)iErrorRoll = 0.0f;

	float proportional_roll = rollpGain * pidRollError ;
	float intigral_roll     = rolliGain * iErrorRoll;
	float derivative_roll   = rolldGain * (rollInput - rollPreviousInput);

	pid_roll_output = proportional_roll + intigral_roll + derivative_roll;

	if (pid_roll_output > pid_max) {
		pid_roll_output = pid_max;
	} else if (pid_roll_output < (pid_max * -1) ) {
		pid_roll_output = (pid_max * -1);
	}

	rollPreviousInput = rollInput;





	pidPitchError =   pitchInput - pitchSetpoint;
	iErrorPitch += pidPitchError;

	if (iErrorPitch > pid_max) {
		iErrorPitch = pid_max;
	} else if (iErrorPitch < (pid_max * -1)) {
		iErrorPitch = (pid_max * -1);
	}
	//if(fabs(pitchInput)> 500.0f)iErrorPitch = 0.0f;

	float proportional_pitch = pitchpGain * pidPitchError ;
	float intigral_pitch     = pitchiGain * iErrorPitch;
	float derivative_pitch   = pitchdGain * (pitchInput - pitchPreviousInput);

	pid_pitch_output = proportional_pitch + intigral_pitch +  derivative_pitch;

	if (pid_pitch_output > pid_max) {
		pid_pitch_output = pid_max;
	} else if (pid_pitch_output < pid_max * -1) {
		pid_pitch_output = pid_max * -1;
	}
	pitchPreviousInput = pitchInput;








	pidYawError =   yawInput - yawSetpoint ;
	iErrorYaw +=  pidYawError ;
	if (iErrorYaw > pid_max) {
		iErrorYaw = pid_max;
	} else if (iErrorYaw < pid_max * -1) {
		iErrorYaw = pid_max * -1;
	}

	float proportional_yaw = yawpGain * pidYawError ;
	float intigral_yaw     = 	yawiGain * iErrorYaw;

	pid_yaw_output = proportional_yaw + intigral_yaw ;


	if (pid_yaw_output > pid_max) {
		pid_yaw_output = pid_max;
	} else if (pid_yaw_output < pid_max * -1) {
		pid_yaw_output = pid_max * -1;
	}

}
*/

void calculate_pid_output(PID_Data *pidData) {
	float iError = 0.0f;
	float pidError = 0.0f;
	float dError = 0.0f;

	float pTerm = 0.0f;
	float iTerm = 0.0f;
	float dTerm = 0.0f;

	pidError =   pidData->setPoint - pidData->input;    //calculate the error

	iError = pidData->previError + pidError;   //calculate intigral of error

	if (iError > pid_max) {          //limit intigral output
		iError = pid_max;
	} else if (iError < pid_max * -1.0f) {
		iError = pid_max * -1.0f;
	}

	dError = (pidError - pidData->prevInput);    //calculate derivative output

	pTerm = pidData->pGain * pidError;
	iTerm = pidData->iGain * iError;
	dTerm = pidData->dGain * dError;

	pidData->output = pTerm + iTerm + dTerm; // calculate the PID output by adding  p,i,d terms

	if (pidData->output > pid_max) {    //limit the pid output
		pidData->output = pid_max;
	} else if (pidData->output < pid_max * -1.0f) {
		pidData->output = pid_max * -1.0f;
	}

	pidData->previError = iError;
	pidData->prevInput = pidError;

}

