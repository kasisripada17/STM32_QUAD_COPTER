#include "main.h"




#define LSM6DS3_I2C_ADD_L 0xD5U

#define LSM6DS3_CTRL1_XL 0x10
#define LSM6DS3_CTRL2_G  0x11
#define LSM6DS3_CTRL3_C  0x12





typedef struct
{

int16_t X;
int16_t Y;
int16_t Z;
}Sensor_Data_raw;




typedef struct
{

int32_t X;
int32_t Y;
int32_t Z;
}Sensor_Data;



typedef struct
{

float X;
float Y;
float Z;
}Sensor_Data_Readable;




void ReadAccelerometerData(I2C_HandleTypeDef *hi2c,Sensor_Data_Readable *Acc_Data);

void ReadGyroScopeData(I2C_HandleTypeDef *hi2c, Sensor_Data_Readable *Gyro_Data);

