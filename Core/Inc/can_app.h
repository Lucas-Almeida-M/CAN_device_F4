/*
 * can_app.h
 *
 *  Created on: Jan 13, 2024
 *      Author: lucas
 */

#ifndef INC_CAN_APP_H_
#define INC_CAN_APP_H_

#include "can.h"
#include "Parse.h"
#include "module_cfg.h"


typedef struct _StatusSlave
{
	uint8_t sensorsHab;
	uint8_t internalTemp;
	uint16_t transmissionErrors;
	uint16_t runtime;
}StatusSlave;


#endif /* INC_CAN_APP_H_ */
