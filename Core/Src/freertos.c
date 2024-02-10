/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern count;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ReceiveCAN_MSG_Handle;
osThreadId SendCAN_MSG_Handle;
osThreadId processDatataskHandle;
osMessageQId queue_can_receiveHandle;
osMessageQId queue_can_sendHandle;
osMessageQId queue_process_dataHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void ReceiveCAN_MSG(void const * argument);
void SendCAN_MSG(void const * argument);
void Process_data_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of queue_can_receive */
  osMessageQDef(queue_can_receive, 10, CanPacket);
  queue_can_receiveHandle = osMessageCreate(osMessageQ(queue_can_receive), NULL);

  /* definition and creation of queue_can_send */
  osMessageQDef(queue_can_send, 10, CanPacket);
  queue_can_sendHandle = osMessageCreate(osMessageQ(queue_can_send), NULL);

  /* definition and creation of queue_process_data */
  osMessageQDef(queue_process_data, 20, SensorData);
  queue_process_dataHandle = osMessageCreate(osMessageQ(queue_process_data), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ReceiveCAN_MSG_ */
  osThreadDef(ReceiveCAN_MSG_, ReceiveCAN_MSG, osPriorityNormal, 0, 1024);
  ReceiveCAN_MSG_Handle = osThreadCreate(osThread(ReceiveCAN_MSG_), NULL);

  /* definition and creation of SendCAN_MSG_ */
  osThreadDef(SendCAN_MSG_, SendCAN_MSG, osPriorityNormal, 0, 512);
  SendCAN_MSG_Handle = osThreadCreate(osThread(SendCAN_MSG_), NULL);

  /* definition and creation of processDatatask */
  osThreadDef(processDatatask, Process_data_task, osPriorityHigh, 0, 1024);
  processDatataskHandle = osThreadCreate(osThread(processDatatask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
osDelay(1000);
  /* Infinite loop */
  for(;;)
  {
	count++;
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ReceiveCAN_MSG */
/**
* @brief Function implementing the ReceiveCAN_MSG_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReceiveCAN_MSG */
__weak void ReceiveCAN_MSG(void const * argument)
{
  /* USER CODE BEGIN ReceiveCAN_MSG */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ReceiveCAN_MSG */
}

/* USER CODE BEGIN Header_SendCAN_MSG */
/**
* @brief Function implementing the SendCAN_MSG_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SendCAN_MSG */
__weak void SendCAN_MSG(void const * argument)
{
  /* USER CODE BEGIN SendCAN_MSG */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SendCAN_MSG */
}

/* USER CODE BEGIN Header_Process_data_task */
/**
* @brief Function implementing the processDatatask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Process_data_task */
__weak void Process_data_task(void const * argument)
{
  /* USER CODE BEGIN Process_data_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Process_data_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
