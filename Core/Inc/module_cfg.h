/*
 * module_cfg.h
 *
 *  Created on: Jan 6, 2024
 *      Author: lucas
 */

#ifndef INC_MODULE_CFG_H_
#define INC_MODULE_CFG_H_

#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "flash_sector.h"
#include "can_app.h"


#define SENSOR_NUMBERS 8
#define INPUT_NUMBERS  4
#define OUTPUT_NUMBERS 4



typedef struct _sensorsCfg
{
	bool enable;

}sensorsCfg;

typedef struct _inputsCfg
{
	bool enable;
	bool inverted;
	uint16_t debouncingTime;


}inputsCfg;

typedef struct _outputsCfg
{

	bool enable;

}outputsCfg;

typedef struct _module_cfg
{
	uint16_t boardID;
	bool enable;
	sensorsCfg sensors[SENSOR_NUMBERS];
	outputsCfg outputs[INPUT_NUMBERS];
	inputsCfg  inputs[OUTPUT_NUMBERS];

}module_cfg;


#define CFG_SECTOR_ADDRESS 0x08060000
#define HEADER_LENGTH 4 // 4 bites header

enum SavedCFG
{
	HEADER_0		= 0,
	HEADER_1 		= 1,
	HEADER_2 		= 2,
	HEADER_3 		= 3,
	IDENTIFIER      = 4,
	SENSOR_0_ENABLE = 5,
	SENSOR_1_ENABLE = 6,
	SENSOR_2_ENABLE = 7,
	SENSOR_3_ENABLE = 8,
	SENSOR_4_ENABLE = 9,
	SENSOR_5_ENABLE = 10,
	SENSOR_6_ENABLE = 11,
	SENSOR_7_ENABLE = 12,
	TOTAL_CFG_BYTES

};


void module_cfg_init(void);
int apply_config(module_cfg newConfig);

#endif /* INC_MODULE_CFG_H_ */
