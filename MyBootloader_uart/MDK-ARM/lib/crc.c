#include "crc.h"



uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len,uint32_t crc_host){
	uint32_t CRCValue=0xff;
	for(int i=0;i< len;i++){
		uint32_t i_data=pData[i];
		CRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}  
	/* Reset CRC Calculation Unit */
  __HAL_CRC_DR_RESET(&hcrc);

	if( CRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}

	return VERIFY_CRC_FAIL;
	
}
