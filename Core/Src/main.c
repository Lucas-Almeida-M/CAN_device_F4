/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int count = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BOARD_ID 1
#define SAMPLES_PER_CYCLE 64
#define OFFSET 1
#define CORRECTION_FACTOR 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uartRxBuffer[256] = {0};
uint8_t uartTxBuffer[256] = {0};


uint16_t adcint[8] = {0};
extern module_cfg configs;
bool aux = 0;
uint16_t adc_count = 0;

//extern uint16_t bufferTensao[64];
//extern uint16_t bufferTensaoVAD[64];
//extern uint16_t bufferTensaoVBD[64];
//extern uint16_t bufferTensaoVCD[64];
extern osMessageQId queue_can_sendHandle;
extern  osMessageQId queue_process_dataHandle;
SignalQ sgnalQual[3] = {0};
SensorData sensorData;
uint16_t sensorValues[8] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart1, uartRxBuffer, 256);
  module_cfg_init();
//  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, adcint, 8);


  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
osDelay(3000);
__HAL_TIM_CLEAR_IT(&htim3 ,TIM_IT_UPDATE);
 HAL_TIM_Base_Start_IT(&htim3);
  /* Infinite loop */
  for(;;)
  {
	  osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}


void Process_data_task(void *argument)
{
  /* USER CODE BEGIN Process_data_task */
	uint16_t count = 0;
	SignalQ signalQ[3] = {0};
	SensorData sensorData = {0};

	Data data;
  /* Infinite loop */
  for(;;)
  {
	BaseType_t xStatus = xQueueReceive(queue_process_dataHandle, &sensorData, 0);
	if (xStatus == pdPASS)
	{
			// conseguiu tirar da fila
		calculate_analog(&data, &sensorData);
		calculate_RMS(signalQ,&data, count);
//		calculate_Phase(signalQ,&data);
//		calculate_freq(signalQ,&data);
		count++;
		if (count == 60)
		{
			MeanValues meanValues[3] = {0};
			calculate_mean(meanValues, signalQ);
//			send_data_to_queue(meanValues);

			memset(signalQ, 0, sizeof(signalQ));
			count = 0;
		}
		memset(&sensorData, 0, sizeof(sensorData));
	}
    osDelay(1);
  }
  /* USER CODE END Process_data_task */
}


void calculate_analog(Data *data, SensorData *sensorData)
{
	for (int i = 0; i < SAMPLES_PER_CYCLE; i++)
	{
		data->sensorData_values[VA][i] = (( (float)sensorData->sensorData_buff[VA][i] / MAX_ADC ) * MAX_ANALOG - OFFSET) * CORRECTION_FACTOR;
		data->sensorData_values[VB][i] = (( (float)sensorData->sensorData_buff[VB][i] / MAX_ADC ) * MAX_ANALOG - OFFSET) * CORRECTION_FACTOR;
		data->sensorData_values[VC][i] = (( (float)sensorData->sensorData_buff[VC][i] / MAX_ADC ) * MAX_ANALOG - OFFSET) * CORRECTION_FACTOR;
	}
}

void calculate_RMS(SignalQ *RMS, Data *data, uint8_t i)
{
	float sum_square[3] = {0};

	for (int i = 0; i < SAMPLES_PER_CYCLE; i++)
	{
		sum_square[VA] += pow(data->sensorData_values[VA][i], 2);
		sum_square[VB] += pow(data->sensorData_values[VB][i], 2);
	    sum_square[VC] += pow(data->sensorData_values[VC][i], 2);
	}
	RMS[VA].rms[i] = (float) (sqrt(sum_square[VA] / SAMPLES_PER_CYCLE));
	RMS[VB].rms[i] = (float) (sqrt(sum_square[VB] / SAMPLES_PER_CYCLE));
	RMS[VC].rms[i] = (float) (sqrt(sum_square[VC] / SAMPLES_PER_CYCLE));
}

void calculate_Phase(SensorData *data)
{

}

void fill_data(CanPacket *message, MeanValues values)
{
	message->canDataFields.data[0] = (uint8_t)( ( (uint16_t)values.meanRMS >> 8) & 0xFF);
	message->canDataFields.data[1] = (uint8_t)(   (uint16_t)values.meanRMS & 0xFF );

	message->canDataFields.data[2] = (uint8_t)( ( (uint16_t)values.meanPhase >> 8) & 0xFF);
	message->canDataFields.data[3] = (uint8_t)(   (uint16_t)values.meanPhase & 0xFF );

	message->canDataFields.data[4] = (uint8_t)( ( (uint16_t)values.meanFreq >> 8) & 0xFF);
	message->canDataFields.data[5] = (uint8_t)(   (uint16_t)values.meanFreq & 0xFF );

}

void send_data_to_queue(MeanValues values[])
{
	CanPacket message = {0};

	for (int i = 0; i < 3; i++)
	{
		message.canID = DEVICE_ID;
		message.canDataFields.ctrl0 = 0; //revisar
		message.canDataFields.ctrl1 = i; //numero do sensor

		fill_data(&message, values[i]); // a cada tensao (a b c) envia uma mensagem can com rms, fase e freq

		BaseType_t xStatus = xQueueSendToBack(queue_can_sendHandle, &message, 0);
		if (xStatus != pdPASS)
		{
	//			fila estourou
		}
		HAL_Delay(1);
	}
}

void calculate_mean(MeanValues *meanValues , SignalQ *signalQ)
{
	float sumValues[3][3] = {0};
	for (int i = 0; i < 60; i++)
	{
		sumValues[VA][0]  += signalQ[VA].rms[i];
		sumValues[VA][1]  += signalQ[VA].phase[i];
		sumValues[VA][2]  += signalQ[VA].freq[i];

		sumValues[VB][0]  += signalQ[VB].rms[i];
		sumValues[VB][1]  += signalQ[VB].phase[i];
		sumValues[VB][2]  += signalQ[VB].freq[i];

		sumValues[VC][0]  += signalQ[VC].rms[i];
		sumValues[VC][1]  += signalQ[VC].phase[i];
		sumValues[VC][2]  += signalQ[VC].freq[i];
	}
	meanValues[VA].meanRMS   = sumValues[VA][0] / 60;
	meanValues[VA].meanPhase = sumValues[VA][1] / 60;
	meanValues[VA].meanFreq  = sumValues[VA][2] / 60;

	meanValues[VB].meanRMS   = sumValues[VB][0] / 60;
	meanValues[VB].meanPhase = sumValues[VB][1] / 60;
	meanValues[VB].meanFreq  = sumValues[VB][2] / 60;

	meanValues[VC].meanRMS   = sumValues[VC][0] / 60;
	meanValues[VC].meanPhase = sumValues[VC][1] / 60;
	meanValues[VC].meanFreq  = sumValues[VC][2] / 60;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // interrupação RX da serial
{
	HAL_UART_Transmit(&huart1, uartRxBuffer , 256, 10);


	HAL_UART_Receive_IT(&huart1, uartRxBuffer, 256);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if(htim->Instance==TIM2)
  {
	  if (adc_count < SAMPLES_PER_CYCLE)
	  {
		sensorData.sensorData_buff[VA][adc_count] = bufferTensaoVAD[adc_count];
		sensorData.sensorData_buff[VB][adc_count] = bufferTensaoVBD[adc_count];
		sensorData.sensorData_buff[VC][adc_count] = bufferTensaoVCD[adc_count];
		adc_count++;
		if (adc_count == 64)
		{
			BaseType_t xStatus = xQueueSendToBackFromISR(queue_process_dataHandle, &sensorData.sensorData_buff, 0);
			if (xStatus != pdPASS)
			{
				uint8_t a = 6;
			}
			aux = 1;
			adc_count = 0;
		}
	  }
  }

  if (htim->Instance == TIM3)
  {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      CanPacket canMSG = {0};
      canMSG.canID = DEVICE_ID;
      canMSG.canDataFields.ctrl0 = DATA;

      int countMSG = 0;
      int dataIdx = 0;

      for (int sensorIdx = 0; sensorIdx < SENSOR_NUMBERS; sensorIdx++)
      {
          sensorValues[sensorIdx] = adcint[sensorIdx];

          if (configs.sensors[sensorIdx].enable)
          {
              canMSG.canDataFields.ctrl1 |= (1 << sensorIdx);

              canMSG.canDataFields.data[dataIdx] = (uint8_t)((sensorValues[sensorIdx] >> 8) & 0xFF);
              canMSG.canDataFields.data[dataIdx + 1] = (uint8_t)(sensorValues[sensorIdx] & 0xFF);

              countMSG++;
              dataIdx += 2;
          }

          if (countMSG == 3 || (sensorIdx == 7 && countMSG != 0))
          {
              xQueueSendToBackFromISR(queue_can_sendHandle, &canMSG, &xHigherPriorityTaskWoken);
              canMSG.canDataFields.ctrl1 = 0;
              countMSG = 0;
              dataIdx = 0;
          }
      }
  }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
