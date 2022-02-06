/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GREEN_LED_Pin GPIO_PIN_5
#define GREEN_LED_GPIO_Port GPIOA
#define RECEIVER4_TIM3_CH4_Pin GPIO_PIN_1
#define RECEIVER4_TIM3_CH4_GPIO_Port GPIOB
#define MOTOR3_TIM15_CH1_Pin GPIO_PIN_14
#define MOTOR3_TIM15_CH1_GPIO_Port GPIOB
#define MOTOR4_TIM15_CH2_Pin GPIO_PIN_15
#define MOTOR4_TIM15_CH2_GPIO_Port GPIOB
#define RED_LED_Pin GPIO_PIN_6
#define RED_LED_GPIO_Port GPIOC
#define RECEIVER3_TIM3_CH3_Pin GPIO_PIN_8
#define RECEIVER3_TIM3_CH3_GPIO_Port GPIOC
#define MOTOR1_TIM1_CH3_Pin GPIO_PIN_10
#define MOTOR1_TIM1_CH3_GPIO_Port GPIOA
#define MOTOR2_TIM1_CH4_Pin GPIO_PIN_11
#define MOTOR2_TIM1_CH4_GPIO_Port GPIOA
#define SENSORS_I2C1_SCL_Pin GPIO_PIN_8
#define SENSORS_I2C1_SCL_GPIO_Port GPIOB
#define SENSORS_I2C1_SDA_Pin GPIO_PIN_9
#define SENSORS_I2C1_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


typedef struct
{
	volatile uint16_t channel1;
	volatile uint16_t channel2;
	volatile uint16_t channel3;
	volatile uint16_t channel4;
	volatile uint16_t channel5;
}RC_Data;
typedef struct
{
volatile uint8_t edge;
volatile uint32_t ticks;

}RC_CAL;

void LED_ON(uint8_t LED);
void LED_OFF(uint8_t LED);
float map(float x, float in_min, float in_max, float out_min, float out_max) ;
uint32_t condition_values(uint32_t lowerLimit, uint32_t upperLimit, uint32_t value);
#define MOTOR_MIN_SPEED 1150
#define MOTOR_MAX_SPEED 2000

#define ROLL_REVERSE
#define PITCH_REVERSE
//#define YAW_REVERSE

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
