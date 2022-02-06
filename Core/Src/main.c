/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "motors.h"
#include <string.h>
#include "sensors.h"
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void reset_variables(void);
void CalculateOutput(float thr);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

RC_Data RadioData = { 0 };
RC_CAL RadioMeasureChannel1 = { 0 };
RC_CAL RadioMeasureChannel2 = { 0 };
RC_CAL RadioMeasureChannel3 = { 0 };
RC_CAL RadioMeasureChannel4 = { 0 };
RC_CAL RadioMeasureChannel5 = { 0 };

Sensor_Data GyroData_RAW = { 0 };
Sensor_Data_Readable GyroData = { 0 };
Sensor_Data_Readable Current_GyroData = { 0 };
Sensor_Data_Readable Acc_Data = { 0 };

Sensor_Data_Readable GyroDataBias = { 0 };
float biasX = 0.0f;
float biasY = 0.0f;
float biasZ = 0.0f;

int bias_count = 0;
uint8_t buff[256];
uint8_t size = 0;
uint32_t old_time;

float pid_roll_output = 0.0;
float pid_pitch_output = 0.0;
float pid_yaw_output = 0.0;

float iErrorRoll = 0.0f;
float rollPreviousInput = 0.0f;

float iErrorPitch = 0.0f;
float pitchPreviousInput = 0.0f;

float iErrorYaw = 0.0f;
float yawPreviousInput = 0.0f;

float rollInput = 0.0f;
float pitchInput = 0.0f;
float yawInput = 0.0f;

float pidRollError = 0.0f;
float pidPitchError = 0.0f;
float pidYawError = 0.0f;

uint16_t motor1 = 0;
uint16_t motor2 = 0;
uint16_t motor3 = 0;
uint16_t motor4 = 0;

uint16_t throttle = 0;
uint16_t pitch = 0;
uint16_t roll = 0;
uint16_t yaw = 0;

PID_Data rollPID = { 0 };
PID_Data pitchPID = { 0 };
PID_Data yawPID = { 0 };

uint32_t arm_counter = 0;
uint8_t armed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int button_count = 0;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM15_Init();
	MX_TIM8_Init();
	MX_UART4_Init();
	/* USER CODE BEGIN 2 */
	//HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
	//HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
	/** Device Identification (Who am I) **/
//#define LSM6DS3_
	//receiver setup
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM3->DIER |= 0x18;
	TIM3->CCER |= 0x1100;

	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->DIER |= 0x18;
	TIM2->CCER |= 0x1100;

	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn);

	HAL_StatusTypeDef ret = 0;

	// SENSORS SETUP
	//set GYRO fullscale and ODR

	static uint8_t val;
	HAL_Delay(100);
	uint8_t reg = 0x00;

	reg = 0x80;
	ret = HAL_I2C_Mem_Write(&hi2c1, 0xD5, 0x12, 1, &reg, 1, HAL_MAX_DELAY);
	HAL_Delay(100);

	reg = 0x74;
	ret = HAL_I2C_Mem_Write(&hi2c1, 0xD5, 0x11, 1, &reg, 1, HAL_MAX_DELAY);
//
	reg = 0x7C;
	ret = HAL_I2C_Mem_Write(&hi2c1, 0xD5, 0x10, 1, &reg, 1, HAL_MAX_DELAY);

	reg = 0x44;
	ret = HAL_I2C_Mem_Write(&hi2c1, 0xD5, 0x12, 1, &reg, 1, HAL_MAX_DELAY);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

	//set motor values to 1000 to stop spinning accidentally
	DriveMotors(1000, 1000, 1000, 1000);

	rollPID.pGain = ROLL_P_GAIN;
	rollPID.iGain = ROLL_I_GAIN;
	rollPID.dGain = ROLL_D_GAIN;

	pitchPID.pGain = PITCH_P_GAIN;
	pitchPID.iGain = PITCH_I_GAIN;
	pitchPID.dGain = PITCH_D_GAIN;

	yawPID.pGain = YAW_P_GAIN;
	yawPID.iGain = YAW_I_GAIN;
	yawPID.dGain = YAW_D_GAIN;

	HAL_Delay(2000);
	for (int i = 0; i < 200; i++) {
		ReadGyroScopeData(&hi2c1, &Current_GyroData);
		GyroDataBias.X += Current_GyroData.X;
		GyroDataBias.Y += Current_GyroData.Y;
		GyroDataBias.Z += Current_GyroData.Z;
		HAL_Delay(10);
		HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);

	}

	GyroDataBias.X /= 200.0f;
	GyroDataBias.Y /= 200.0f;
	GyroDataBias.Z /= 200.0f;

	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
	/* USER CODE END 2 */
	float roll_copter = 0.0f;
	float pitch_copter = 0.0f;

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		if (HAL_GetTick() - old_time >= 4) {

			ReadGyroScopeData(&hi2c1, &Current_GyroData);
			ReadAccelerometerData(&hi2c1, &Acc_Data);

			float roll_acc  = 0.0f;
			float pitch_acc = 0.0f;
			float resultant_acceleration = sqrt(
					(Acc_Data.X * Acc_Data.X) + (Acc_Data.Y * Acc_Data.Y)
							+ (Acc_Data.Z * Acc_Data.Z)); //Calculate the total accelerometer vector.

			if (fabs(Acc_Data.Y) < resultant_acceleration) { //Prevent the asin function to produce a NaN
				pitch_acc = asin((float) Acc_Data.Y / resultant_acceleration)
						* 57.296;          //Calculate the pitch angle.
			}
			if (abs(Acc_Data.X) < resultant_acceleration) { //Prevent the asin function to produce a NaN
				roll_acc = asin((float) Acc_Data.X / resultant_acceleration)
						* -57.296;          //Calculate the roll angle.
			}


			GyroData.X = GyroData.X * 0.7f + (Current_GyroData.X - GyroDataBias.X) * 0.3f;
			GyroData.Y = GyroData.Y * 0.7f + (Current_GyroData.Y - GyroDataBias.Y) * 0.3f;
			GyroData.Z = GyroData.Z * 0.7f + (Current_GyroData.Z - GyroDataBias.Z) * 0.3f;

			pitch_copter += (-1.0*GyroData.X) * 0.004f;
			roll_copter  += (-1.0*GyroData.Y) * 0.004f;



			roll_copter =  0.99f * roll_copter  + 0.01f  * roll_acc;
			pitch_copter = 0.99f * pitch_copter + 0.01f  * pitch_acc;



			float pitch_level_adjust = roll_copter  * 15.0f; //Calculate the pitch angle correction
			float roll_level_adjust =  pitch_copter * 15.0f;


			if (throttle <= 1100 && yaw > 1800 && armed == 0) {
				arm_counter += 1;
			}

			if (arm_counter == 500) {
				arm_counter = 499;
				armed = 1;
				HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin,
						GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
				DriveMotors(MOTOR_MIN_SPEED, MOTOR_MIN_SPEED, MOTOR_MIN_SPEED,MOTOR_MIN_SPEED);          //SPIN MOTORS AT LOW SPEED

			}

			if (throttle <= 1100 && yaw <= 1200 && armed == 1) {
				arm_counter -= 1;

			}
			if (armed == 1 && arm_counter == 0) {

				DriveMotors(1000, 1000, 1000, 1000);
				armed = 0;
				HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin,
						GPIO_PIN_SET);
				HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin,
						GPIO_PIN_RESET);
				reset_variables();
			}

			float thr = (float) condition_values(1100, 1800, throttle);

			//calculate roll, pitch and yaw pid outputs

			rollPID.input = GyroData.X;
			pitchPID.input = -1* GyroData.Y;
			yawPID.input = GyroData.Z;



			if(rollPID.input > 500.0f)
			{
				rollPID.input = 500.0f	;
			}
			if(rollPID.input < -500.0f)
			{
				rollPID.input = -500.0f	;
			}
			if(pitchPID.input > 500.0f)
			{
				pitchPID.input = 500.0f	;
			}
			if(pitchPID.input < -500.0f)
			{
				pitchPID.input = -500.0f	;
			}
			if(yawPID.input > 500.0f)
			{
				yawPID.input = 500.0f	;
			}
			if(yawPID.input < -500.0f)
			{
				yawPID.input = -500.0f	;
			}


//
//			size = sprintf(buff, "\r\n%f, %f, %f", rollPID.input,pitchPID.input,yawPID.input );
//
//			HAL_UART_Transmit(&huart4, buff, size, 1000);

			uint16_t roll_in = roll;
			uint16_t pitch_in = pitch;
			uint16_t yaw_in  = yaw;

#ifdef ROLL_REVERSE
			 roll_in = map(roll, 1000, 2000, 2000, 1000);
#endif

#ifdef PITCH_REVERSE
			 pitch_in = map(pitch, 1000, 2000, 2000, 1000);
#endif

#ifdef YAW_REVERSE
			 yaw_in = map(yaw, 1000, 2000, 2000, 1000);
#endif


			rollPID.setPoint = 0.0f;
			pitchPID.setPoint = 0.0f;
			yawPID.setPoint = 0.0f;




			if (roll_in > 1508) {
				rollPID.setPoint = roll_in - 1508;

			} else if (roll_in < 1492) {
				rollPID.setPoint = roll_in - 1492;

			}

			if (pitch_in > 1508) {
				pitchPID.setPoint = pitch_in - 1508;

			} else if (pitch_in < 1492) {
				pitchPID.setPoint = pitch_in - 1492;
			}

			if (yaw_in > 1508) {
				yawPID.setPoint = yaw_in - 1508;

			} else if (yaw_in < 1492) {
				yawPID.setPoint = yaw_in - 1492;
			}

				rollPID.setPoint += roll_level_adjust;
			rollPID.setPoint = rollPID.setPoint / 3.0f;

				pitchPID.setPoint -= pitch_level_adjust;
			pitchPID.setPoint = pitchPID.setPoint / 3.0f;

			if (throttle > 1200) {
				yawPID.setPoint = yawPID.setPoint / 5.0f;
			}
			else
			{
				yawPID.setPoint  = 0.0f;
			}




//			size = sprintf(buff, "\r\n%f, %f, %f", rollPID.output,pitchPID.output,yawPID.output );
//
//			HAL_UART_Transmit(&huart4, buff, size, 1000);
			if (armed) {
				calculate_pid_output(&rollPID);
				calculate_pid_output(&pitchPID);
				calculate_pid_output(&yawPID);

				//calculate motor speed and drive motors
				CalculateOutput(thr);
			}
			old_time = HAL_GetTick();
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00702991;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 100 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 2000 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 79;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */
	__HAL_RCC_TIM3_CLK_ENABLE();
	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 79;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 100 - 1;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 2000 - 1;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void) {

	/* USER CODE BEGIN TIM15_Init 0 */

	/* USER CODE END TIM15_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM15_Init 1 */

	/* USER CODE END TIM15_Init 1 */
	htim15.Instance = TIM15;
	htim15.Init.Prescaler = 100 - 1;
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Period = 2000 - 1;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim15.Init.RepetitionCounter = 0;
	htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim15) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim15) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM15_Init 2 */

	/* USER CODE END TIM15_Init 2 */
	HAL_TIM_MspPostInit(&htim15);

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : GREEN_LED_Pin */
	GPIO_InitStruct.Pin = GREEN_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RED_LED_Pin */
	GPIO_InitStruct.Pin = RED_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void LED_ON(uint8_t LED) {
	if (LED == 1) {
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
	}
	if (LED == 2) {
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
	}
}
void LED_OFF(uint8_t LED) {
	if (LED == 1) {
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
	}
	if (LED == 2) {
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
	}
}

void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_CC3IF) {
		if (RadioMeasureChannel3.edge == 0) {
			RadioMeasureChannel3.ticks = TIM2->CCR3;
			RadioMeasureChannel3.edge = 1;
			TIM2->CCER |= TIM_CCER_CC3P;
		} else if (RadioMeasureChannel3.edge == 1) {
			uint32_t gu32_Ticks = (TIM2->CCR3 - RadioMeasureChannel3.ticks);
			if (gu32_Ticks < 0) {
				gu32_Ticks += 0xFFFF;
			}
			RadioData.channel3 = gu32_Ticks;
			throttle = condition_values(1000, 2000, RadioData.channel3);
			RadioMeasureChannel3.edge = 0;
			TIM2->CCER &= ~TIM_CCER_CC3P;
		}

		TIM2->SR &= ~TIM_SR_CC3IF;
	} else if (TIM2->SR & TIM_SR_CC4IF) {
		if (RadioMeasureChannel4.edge == 0) {
			RadioMeasureChannel4.ticks = TIM2->CCR4;
			RadioMeasureChannel4.edge = 1;
			TIM2->CCER |= TIM_CCER_CC4P;
		} else if (RadioMeasureChannel4.edge == 1) {
			uint32_t gu32_Ticks = (TIM2->CCR4 - RadioMeasureChannel4.ticks);
			if (gu32_Ticks < 0) {
				gu32_Ticks += 0xFFFF;
			}
			RadioData.channel4 = gu32_Ticks;
			yaw = condition_values(1000, 2000, RadioData.channel4);
			RadioMeasureChannel4.edge = 0;
			TIM2->CCER &= ~TIM_CCER_CC4P;
		}
		TIM2->SR &= ~TIM_SR_CC4IF;
	}
}
void TIM3_IRQHandler(void) {
	if (TIM3->SR & TIM_SR_CC3IF) {
		if (RadioMeasureChannel1.edge == 0) {
			RadioMeasureChannel1.ticks = TIM3->CCR3;
			RadioMeasureChannel1.edge = 1;
			TIM3->CCER |= TIM_CCER_CC3P;
		} else if (RadioMeasureChannel1.edge == 1) {
			uint32_t gu32_Ticks = (TIM3->CCR3 - RadioMeasureChannel1.ticks);
			if (gu32_Ticks < 0) {
				gu32_Ticks += 0xFFFF;
			}
			RadioData.channel1 = gu32_Ticks;
			roll = condition_values(1000, 2000, RadioData.channel1);
			RadioMeasureChannel1.edge = 0;
			TIM3->CCER &= ~TIM_CCER_CC3P;
		}

		TIM3->SR &= ~TIM_SR_CC3IF;
	} else if (TIM3->SR & TIM_SR_CC4IF) {
		if (RadioMeasureChannel2.edge == 0) {
			RadioMeasureChannel2.ticks = TIM3->CCR4;
			RadioMeasureChannel2.edge = 1;
			TIM3->CCER |= TIM_CCER_CC4P;
		} else if (RadioMeasureChannel2.edge == 1) {
			uint32_t gu32_Ticks = (TIM3->CCR4 - RadioMeasureChannel2.ticks);
			if (gu32_Ticks < 0) {
				gu32_Ticks += 0xFFFF;
			}
			RadioData.channel2 = gu32_Ticks;
			pitch = condition_values(1000, 2000, RadioData.channel2);
			RadioMeasureChannel2.edge = 0;
			TIM3->CCER &= ~TIM_CCER_CC4P;
		}
		TIM3->SR &= ~TIM_SR_CC4IF;
	}
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (float) ((x - in_min) * (out_max - out_min) / (in_max - in_min)
			+ out_min);
}
/*map function*/

/*reset variables*/

void reset_variables(void) {
	memset(&rollPID, 0x00, sizeof(rollPID));
	memset(&pitchPID, 0x00, sizeof(pitchPID));
	memset(&yawPID, 0x00, sizeof(yawPID));

}

uint32_t condition_values(uint32_t lowerLimit, uint32_t upperLimit,
		uint32_t value) {

	if (value < lowerLimit) {
		value = lowerLimit;
	} else if (value > upperLimit) {
		value = upperLimit;
	}
	return value;
}






void CalculateOutput(float thr)
{

	motor1 = (uint16_t) (thr + pitchPID.output - rollPID.output - yawPID.output);
	motor2 = (uint16_t) (thr - pitchPID.output - rollPID.output + yawPID.output);
	motor3 = (uint16_t) (thr - pitchPID.output + rollPID.output - yawPID.output);
	motor4 = (uint16_t) (thr + pitchPID.output + rollPID.output + yawPID.output);

	//condition_motor_speed_values

	motor1 = condition_values(MOTOR_MIN_SPEED, MOTOR_MAX_SPEED,
			motor1);
	motor2 = condition_values(MOTOR_MIN_SPEED, MOTOR_MAX_SPEED,
			motor2);
	motor3 = condition_values(MOTOR_MIN_SPEED, MOTOR_MAX_SPEED,
			motor3);
	motor4 = condition_values(MOTOR_MIN_SPEED, MOTOR_MAX_SPEED,
			motor4);

	DriveMotors(motor1,motor2,motor3,motor4);
}

//	    HAL_UART_Receive(&huart4, buff, 7, 1000);
//
//	    if(buff[5]  == '\r' || buff[5] == '\n' )
//	    {
//
//
//			if (buff[0] == 'a' && buff[1] == 'p') {
//
//				char in_char[3];
//				memcpy(in_char,&buff[2],3);
//
//				rollPID.pGain = atof(in_char);
//				pitchPID.pGain = atof(in_char);
//
//			} else if (buff[0] == 'a' && buff[1] == 'i') {
//				char in_char[3];
//				memcpy(in_char,&buff[2],3);
//
//				rollPID.iGain = atof(in_char);
//				pitchPID.iGain = atof(in_char);
//			} else if (buff[0] == 'a' && buff[1] == 'd') {
//				char in_char[3];
//				memcpy(in_char,&buff[2],3);
//
//				rollPID.dGain = atof(in_char);
//				pitchPID.dGain = atof(in_char);
//			}
//			if (buff[0] == 'y' && buff[1] == 'p') {
//				char in_char[3];
//				memcpy(in_char,&buff[2],3);
//
//				yawPID.pGain = atof(in_char);;
//			} else if (buff[0] == 'y' && buff[1] == 'i') {
//				char in_char[3];
//				memcpy(in_char,&buff[2],3);
//				yawPID.iGain = atof(in_char);
//			}
//
//			memset(buff,0x00,sizeof(buff));
//
//	    }

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
