/*
 * debug_level.c
 *
 *  Created on: Mar 9, 2024
 *      Author: lucas
 */
#include "debug_level.h"
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
#define NUCLEO
//#define BLACK_PILL
int print_debug (char *msg)
{
	#ifdef NUCLEO
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
	#endif //NUCLEO

	#ifdef BLACK_PILL
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
	#endif //BLACK_PILL

	return 0;
}
