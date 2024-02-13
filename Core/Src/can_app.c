/*
 * can_app.c
 *
 *  Created on: Jan 13, 2024
 *      Author: lucas
 */


#include "can_app.h"
//#include "cmsis_os2.h"
#include "cmsis_os.h"
#include "queue.h"


uint8_t canRX[CAN_SIZE] = {};
uint8_t canTX[CAN_SIZE] = {};
extern uint32_t TxMailbox;
extern osMessageQId queue_can_sendHandle;
extern osMessageQId queue_can_receiveHandle;
extern module_cfg configs;
bool en = 1;



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
		// conseguiu tirar da fila

			switch (canMSG.canDataFields.ctrl0)
			{
				case CONFIG:
					module_cfg cfg = {0};
					for (int i = 0; i < 8; i++)
					{
						cfg.sensors[i].enable = (bool)(canMSG.canDataFields.data[0] & (1 << i));
					}

					canPacket.canID = DEVICE_ID;
					canPacket.canDataFields.ctrl0 = CONFIG;
					if (apply_config(cfg))
					{
						canPacket.canDataFields.ctrl1 = 1;
					}
					else
					{
						canPacket.canDataFields.ctrl1 = 0;
					}
					xQueueSendToBack(queue_can_sendHandle, &canPacket , 0);
					break;
				case DATA:

					break;
				case SYNC:
					canPacket.canID = DEVICE_ID;
					canPacket.canDataFields.ctrl0 = SYNC;
					for (int i = 0; i < 8; i++)
					{
						canPacket.canDataFields.ctrl1 = canPacket.canDataFields.ctrl1 | (configs.sensors[i].enable << (i));
					}

					xQueueSendToBack(queue_can_sendHandle, &canPacket , 0);
					break;
				case REBOOT:
					HAL_Delay(100);
					NVIC_SystemReset();

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
	CanPacket canMsg = {0};
	uint8_t buffer[8];
  /* Infinite loop */
  for(;;)
  {
	BaseType_t xStatus = xQueueReceive(queue_can_sendHandle, &canMsg, 0);
	if (xStatus == pdPASS)
	{
		// conseguiu tirar da fila

		TxHeader.StdId             = canMsg.canID;
		TxHeader.RTR               = CAN_RTR_DATA;
		TxHeader.IDE               = CAN_ID_STD;
		TxHeader.DLC               = CAN_SIZE;
		TxHeader.TransmitGlobalTime = DISABLE;

		memcpy(buffer , &canMsg.canDataFields , sizeof(buffer));
		int status = HAL_CAN_AddTxMessage (&hcan1, &TxHeader, buffer, &TxMailbox);
		if(status)
		{
			Error_Handler();
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
	xQueueSendToBackFromISR(queue_can_receiveHandle, &canPacket, &xHigherPriorityTaskWoken);

	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

}





void sendCanMsg_test(int delay)
{
	  uint8_t tx[7] = {1,2,3,4,5,6,7};
	  TxHeader.StdId             = 0x0;     // ID do dispositivo
	  TxHeader.RTR               = CAN_RTR_DATA;       //(Remote Transmission Request) especifica Remote Frame ou Data Frame.
	  TxHeader.IDE               = CAN_ID_STD;    //define o tipo de id (standard ou extended)
	  TxHeader.DLC               = 7;      //Tamanho do pacote 0 - 8 bytes
	  TxHeader.TransmitGlobalTime = DISABLE;

	  int status = HAL_CAN_AddTxMessage(&hcan, &TxHeader, tx, &TxMailbox);
	  if(status)
	  {
		 Error_Handler();
	  }
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(delay);
}
