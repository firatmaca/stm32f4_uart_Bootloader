#include "print.h"

#define BL_DEBUG_MSG_EN

void printmsg(char *format,...){

#ifdef BL_DEBUG_MSG_EN
  char str[80];
	va_list args;
	va_start(args,format);
  vsprintf(str,format,args);
	HAL_UART_Transmit(&huart2,(uint8_t*) str,strlen(str),HAL_MAX_DELAY);
	va_end(args);
#endif
}

