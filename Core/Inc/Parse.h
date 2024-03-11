
#ifndef SRC_PARSER_H_
#define SRC_PARSER_H_


#include "can.h"
#include <stdio.h>
#include <stdbool.h>

#define ID 0x02

#define STD_ID  0
#define EXT_ID  1
#define FILTER_TYPE_16 0
#define FILTER_TYPE_32 1
#define FILTER_MAX_NUM 14


#define CAN_HEADER 2
#define CAN_SIZE 8


extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;


typedef union
{

	enum FilterIds

	{
		BROADCAST  = 0x00,
		BOARD_F7   = 0x01,
		DEVICE_ID   = ID, // HABILITAR DE ACORDO COM DEVICE

	} FilterId;

	uint32_t FilterIdList[3];

} CanFilterList;

enum MessageType
{
	CONFIG = 0x0,
	DATA   = 0x1,
	SYNC   = 0x2,
	REBOOT = 0x3
};


//typedef struct control
//{
//	bool bit0 : 1;
//	bool bit1 : 1;
//	bool bit2 : 1;
//	bool bit3 : 1;
//	bool bit4 : 1;
//	bool bit5 : 1;
//	bool bit6 : 1;
//	bool bit7 : 1;
//}controlBit;

typedef struct candata
{
	uint8_t ctrl0;
	uint8_t ctrl1;
	uint8_t data [CAN_SIZE - CAN_HEADER];
}CanData;


typedef struct
{
	uint8_t canID;
	CanData canDataFields;
} CanPacket;


typedef union UARTPACKET
{
	uint8_t buffer[CAN_SIZE + 1];
	struct
	{
		uint8_t canID;
		uint8_t seq;
		uint8_t crtl;
		uint8_t data [(CAN_SIZE + 1) - CAN_HEADER];
	}packet;

}UartPacket;



bool ValidatePacket(uint8_t canID);


void DecodeCanPacket(uint32_t canID, UartPacket *uartPacket, uint8_t *buffer);
bool CanWritePacket(uint32_t id, uint8_t *buffer, uint8_t can_rtr, uint16_t tamanho);
void ConfigFilterList (uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4, uint32_t filterBank, uint8_t idType, uint8_t filterScale);





void LoadFilterList(CanFilterList *filterIdList);
void InitFilterList(uint32_t *idList, uint8_t numFilters, uint8_t filterScale);
void CAN_ConfigFilter(void);


#endif /* SRC_PARSER_H_ */
