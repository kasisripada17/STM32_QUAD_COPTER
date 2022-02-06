#include "sensors.h"
#include "string.h"

void ReadAccelerometerData(I2C_HandleTypeDef *hi2c,
		Sensor_Data_Readable *Acc_Data) {

	uint8_t data[6];
	memset(data, 0x00, 6);
	int ret = 0;
	Sensor_Data_raw ACCData_RAW = { 0 };

	ret = HAL_I2C_Mem_Read(hi2c, LSM6DS3_I2C_ADD_L, 0x28,
	I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

	memcpy((uint8_t*)&ACCData_RAW, data, 6);

	Acc_Data->X = ACCData_RAW.X * 0.001f * 0.244f;
	Acc_Data->Y = ACCData_RAW.Y * 0.001f * 0.244f;
	Acc_Data->Z = ACCData_RAW.Z * 0.001f * 0.244f;


}
void ReadGyroScopeData(I2C_HandleTypeDef *hi2c, Sensor_Data_Readable *Gyro_Data) {
	uint8_t data[6];
	memset(data, 0x00, 6);
	int ret = 0;
	Sensor_Data_raw GyroData_RAW = { 0 };

	ret = HAL_I2C_Mem_Read(hi2c, LSM6DS3_I2C_ADD_L, 0x22,
	I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

	memcpy((uint8_t *)&GyroData_RAW, data, 6);


	Gyro_Data->X = GyroData_RAW.X * 17.50f * 0.001f;
	Gyro_Data->Y = GyroData_RAW.Y * 17.50f * 0.001f;
	Gyro_Data->Z = GyroData_RAW.Z * 17.50f * 0.001f;

}
