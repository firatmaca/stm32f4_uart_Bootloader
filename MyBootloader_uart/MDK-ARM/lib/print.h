#ifndef _PRINTT_
#define _PRINTT_

#include "stm32f4xx_hal.h"
#include "stdarg.h"
#include "string.h"
#include "stdint.h"

extern UART_HandleTypeDef huart2;


void printmsg(char *format,...);





#endif

