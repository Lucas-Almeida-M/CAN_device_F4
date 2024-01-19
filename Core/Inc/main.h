/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Parse.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

//#include "queue.h"
//#include "lfs.h"
//#include "lfs_app.h"
#include "module_cfg.h"
#include <stm32f4xx_hal_flash.h>
#include <stm32f4xx_hal_flash_ex.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define MAX_ADC 4096
#define MAX_ANALOG 3.3

#define VA 0
#define VB 1
#define VC 2


typedef struct _signalQ
{
	float rms  [60];
	float phase[60];
	float freq [60];

}SignalQ;

typedef struct _sensorData
{
	uint16_t sensorData_buff[3][64];
}SensorData;

typedef struct _Data
{
	float sensorData_values[3][64];
}Data;

typedef struct MeanValues
{
	float meanRMS;
	float meanPhase;
	float meanFreq;
}MeanValues;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void calculate_analog(Data *data, SensorData *sensorData);
void calculate_RMS(SignalQ *RMS, Data *data, uint8_t i);
void calculate_Phase(SensorData *data);
void send_data_to_queue(MeanValues values[]);
void calculate_mean(MeanValues sumValues[] , SignalQ signalQ[]);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
