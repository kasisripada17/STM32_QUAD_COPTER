#ifndef __PID_H
#define __PID_H

#define ROLL_P_GAIN (float)1.3f
#define ROLL_I_GAIN (float)0.01f
#define ROLL_D_GAIN (float)5.0f

#define PITCH_P_GAIN  (float)1.3f
#define PITCH_I_GAIN  (float)0.01f
#define PITCH_D_GAIN  (float)5.0f

#define YAW_P_GAIN  (float)4.0f
#define YAW_I_GAIN  (float)0.02f
#define YAW_D_GAIN  (float)0.0f





//void calculate_pid_output(void);
#define pid_max (float) 400.0f    //max limit on pid output



typedef struct
{
float prevInput;
float pGain;
float iGain;
float dGain;
float previError;
float input;
float setPoint;
float output;
}PID_Data;

void calculate_pid_output(PID_Data *pidData);

#endif
