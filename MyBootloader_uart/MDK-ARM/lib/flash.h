#ifndef FLASH_H_
#define FLASH_H_

#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdint.h"
#include "crc.h"
#include "print.h"
extern UART_HandleTypeDef huart3;

#define FlASH_SECTOR2_BASE_ADDRESS 0x08008000U  // user app address
#define bl_flash_rx_buffer_size 200  

#define BL_ACK   0XA5
#define BL_NACK  0X7F

void uart_bootloader(void);

void flash_bootloader_erase(uint8_t *buff);
void flash_bootloader_write(uint8_t *buff);
void flash_bootloader_uart_read_data(void);  // coiming data from host
void flash_bootloader_jump_to_user_app(void);
void flash_bootloader_send_ack(uint8_t command_code, uint8_t follow_len);
void flash_bootloader_send_nack(void);

uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector);
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len);
uint8_t execute_mem_write(uint8_t *buff, uint32_t mem_address, uint32_t len);


#define BL_FLASH_ERASE        0x56 //mass erase or sector erase of the user flash
#define BL_MEM_WRITE          0x57 //write data in to different memories of the MCU

#define INVALID_SECTOR 0x04
#define ADDR_VALID 0x00
#define ADDR_INVALID 0x01






#endif

