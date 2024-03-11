/*
 * can_app.c
 *
 *  Created on: Jan 13, 2024
 *      Author: lucas
 */


#include "can_app.h"
#include "cmsis_os.h"
#include "queue.h"
#include "debug_level.h"

uint8_t canRX[CAN_SIZE] = {};
uint8_t canTX[CAN_SIZE] = {};
extern uint32_t TxMailbox;
extern osMessageQId queue_can_sendHandle;
extern osMessageQId queue_can_receiveHandle;
extern module_cfg configs;
char CanMsgDebug [64] = {0};


/***
 * @fn void ReceiveCAN_MSG(void*)
 * @brief Task to process received CAN messages
 *
 * @param argument
 */
void ReceiveCAN_MSG(void *argument)
{
  /* USER CODE BEGIN ProcessCAN_MSG */
	CanPacket canMSG = {0};
  /* Infinite loop */
  for(;;)
  {
	BaseType_t xStatus = xQueueReceive(queue_can_receiveHandle, &canMSG, 0);
	if (xStatus == pdPASS)
	{
		CanPacket canPacket = {0};
		if (DEBUG_LEVEL > NONE)
		{
			sprintf(CanMsgDebug, "CAN message received [ID: %d] [DeviceNum: %d] \r\n", canMSG.canID, canMSG.canDataFields.deviceNum);
			print_debug (CanMsgDebug);
			memset(CanMsgDebug, 0, sizeof(CanMsgDebug));
		}
		// conseguiu tirar da fila

			switch (canMSG.canID)
			{
				case CONFIG:
					module_cfg cfg = {0};
					for (int i = 0; i < 8; i++)
					{
						cfg.sensors[i].enable = (bool)(canMSG.canDataFields.data[0] & (1 << i));
					}

					canPacket.canID = CONFIG;
					canPacket.canDataFields.deviceNum = DEVICE_NUM;
					if (apply_config(cfg))
					{
						canPacket.canDataFields.control = 1;
					}
					else
					{
						canPacket.canDataFields.control = 0;
					}
					xQueueSendToBack(queue_can_sendHandle, &canPacket , 0);
					break;
				case DATA:

					break;
				case SYNC:
					canPacket.canID = SYNC;
					canPacket.canDataFields.deviceNum = DEVICE_NUM;
					for (int i = 0; i < 8; i++)
					{
						canPacket.canDataFields.control = canPacket.canDataFields.control | (configs.sensors[i].enable << (i));
					}
					osDelay(5);
					xQueueSendToBack(queue_can_sendHandle, &canPacket , 0);

					break;
				case REBOOT:
					HAL_Delay(100);
					NVIC_SystemReset();

				default:
					break;
			}
	}
    osDelay(1);
  }
  /* USER CODE END ProcessCAN_MSG */
}

/***
 * @fn void SendCAN_MSG(void*)
 * @brief Task to send CAN messages
 *
 * @param argument
 */
void SendCAN_MSG(void *argument)
{
  /* USER CODE BEGIN SendCAN_MSG */
	CanPacket canMSG = {0};
	uint8_t buffer[8];
  /* Infinite loop */
  for(;;)
  {
	BaseType_t xStatus = xQueueReceive(queue_can_sendHandle, &canMSG, 0);
	if (xStatus == pdPASS)
	{
		// conseguiu tirar da fila

		TxHeader.StdId             = canMSG.canID;
		TxHeader.RTR               = CAN_RTR_DATA;
		TxHeader.IDE               = CAN_ID_STD;
		TxHeader.DLC               = CAN_SIZE;
		TxHeader.TransmitGlobalTime = DISABLE;

		memcpy(buffer , &canMSG.canDataFields , sizeof(buffer));
		int status = HAL_CAN_AddTxMessage (&hcan1, &TxHeader, buffer, &TxMailbox);
		if (DEBUG_LEVEL > NONE)
		{
			sprintf(CanMsgDebug, "CAN message Sent error = %d \r\n", status);
			print_debug (CanMsgDebug);
			memset(CanMsgDebug, 0, sizeof(CanMsgDebug));
		}
		if(status)
		{
			Error_Handler();
		}
		if (DEBUG_LEVEL > NONE)
		{
			sprintf(CanMsgDebug, "CAN message Sent [ID: %d] [DeviceNum: %d] \r\n", canMSG.canID, canMSG.canDataFields.deviceNum);
			print_debug (CanMsgDebug);
			memset(CanMsgDebug, 0, sizeof(CanMsgDebug));
		}
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}

    osDelay(1);
  }
  /* USER CODE END SendCAN_MSG */
}


/***
 * @fn void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*)
 * @brief driver function that receives can messages from CAN FIFO
 *
 * @param hcan
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	CanPacket canPacket = { 0 } ;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, canRX);
	canPacket.canID = RxHeader.StdId;
	memcpy(&canPacket.canDataFields, canRX, sizeof(canRX));

	if ((canPacket.canDataFields.deviceNum == DEVICE_NUM) || (canPacket.canDataFields.deviceNum == BroadCast))
	{
		xQueueSendToBackFromISR(queue_can_receiveHandle, &canPacket, &xHigherPriorityTaskWoken);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}


}

void sendCanMsg_test(int delay)
{
	  uint8_t tx[8] = {0,1,2,3,4,5,6,7};
	  TxHeader.StdId             = SYNC;    	 // ID do dispositivo
	  TxHeader.RTR               = CAN_RTR_DATA;     //(Remote Transmission Request) especifica Remote Frame ou Data Frame.
	  TxHeader.IDE               = CAN_ID_STD;    	 //define o tipo de id (standard ou extended)
	  TxHeader.DLC               = 8;     			 //Tamanho do pacote 0 - 8 bytes
	  TxHeader.TransmitGlobalTime = DISABLE;

	  int status = HAL_CAN_AddTxMessage(&hcan, &TxHeader, tx, &TxMailbox);
	  if(status)
	  {
		 Error_Handler();
	  }
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(delay);
}
