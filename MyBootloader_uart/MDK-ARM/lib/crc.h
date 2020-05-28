#ifndef _CRC_
#define _CRC_

#include "stm32f4xx_hal.h"

extern CRC_HandleTypeDef hcrc;
#define VERIFY_CRC_FAIL    1
#define VERIFY_CRC_SUCCESS 0


uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len,uint32_t crc_host);




#endif
