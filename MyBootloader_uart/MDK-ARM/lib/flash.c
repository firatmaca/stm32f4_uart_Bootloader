#include "flash.h"
#include "print.h"

uint8_t bl_flash_rx_buffer[bl_flash_rx_buffer_size];

void uart_bootloader(void){
	 /* Lets check whether button is pressed or not, if not pressed jump to user application */ 
 if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
	printmsg("Button is pressed(bootloader mode)\n\r");
	flash_bootloader_uart_read_data();
	}
	else 
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
		printmsg("Button is not presssed(user app mode)");
		flash_bootloader_jump_to_user_app();
	
	 }
}

void flash_bootloader_jump_to_user_app(){
	void (*app_reset_handler)(void); // hold the address of the reset handler of the user app.
	printmsg("Bootloader jump to user_app \n");
	uint32_t msp_value= *(volatile uint32_t*)FlASH_SECTOR2_BASE_ADDRESS ;//MSP(base address of the sector 2) 
	printmsg("msp_value: %#x \n",msp_value);
	__set_MSP(msp_value);
	uint32_t resethandler_address=*(volatile uint32_t*)(FlASH_SECTOR2_BASE_ADDRESS+4);
	app_reset_handler=(void*) resethandler_address;
	printmsg("resethandler_address : %#x \n",app_reset_handler);
	app_reset_handler(); //jump to reset handler of the user application
}
void flash_bootloader_uart_read_data(){
	  uint8_t rcv_len=0;
	while(1){

		memset(bl_flash_rx_buffer,0,200);
    HAL_UART_Receive(&huart3,bl_flash_rx_buffer,1,HAL_MAX_DELAY);  //first bl command is "following length"
		rcv_len=bl_flash_rx_buffer[0];
		HAL_UART_Receive(&huart3,&bl_flash_rx_buffer[1],rcv_len,HAL_MAX_DELAY); 	//second  bl command is our cmd
	
		switch(bl_flash_rx_buffer[1]){		 // we call the function for coming cmd
			case BL_FLASH_ERASE:
				flash_bootloader_erase(bl_flash_rx_buffer);
			break;
			
			case BL_MEM_WRITE:
				flash_bootloader_write(bl_flash_rx_buffer);
			break;
			
			default:
				printmsg("Invalid command from recieved from host \n");
			break;
		}
		
	}
	
}


void flash_bootloader_erase(uint8_t *buff){
	uint8_t erase_status=0x00;
	printmsg("flash erase commend");
	uint32_t command_packet_length=bl_flash_rx_buffer[0]+1; //buffer[0] = following length   --   +1 is buf[0]
	uint32_t host_crc = *((uint32_t * ) (bl_flash_rx_buffer+command_packet_length - 4) ) ; 
	/* host_crc refers addres of bufer[command_packet_length - 4]
	 Think of  bl_rx_buffer+command_packet_length - 4 as the memory address based on the original 
	 bl_rx_buffer pointer plus the number of bytes for command_packet_length - 4 uint32_t objects. */
	
	if(!bootloader_verify_crc(&bl_flash_rx_buffer[0],command_packet_length-4,host_crc)){
		printmsg("checksum is correct \n");
		flash_bootloader_send_ack(buff[0],1);
		printmsg("inital_sector :%d  nofsector: %d \n",buff[2],buff[3]);
		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
		erase_status=execute_flash_erase(buff[2] , buff[3]);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
		
		printmsg("flash erase status: %#x\n",erase_status);
		  bootloader_uart_write_data(&erase_status,1);

	}else
	{
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");
        flash_bootloader_send_nack();
	}
		
}

void flash_bootloader_write(uint8_t *buff){
	uint8_t addr_valid = ADDR_VALID;
	uint8_t write_status = 0x00;
	uint8_t chksum =0, len=0;
	len = buff[0];
	uint8_t payload_len = buff[6];

	uint32_t mem_address = *((uint32_t *) ( &buff[2]) );

	chksum = buff[len];

    printmsg("BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_flash_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_flash_rx_buffer+command_packet_len - 4) ) ;


	if (! bootloader_verify_crc(&bl_flash_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg("BL_DEBUG_MSG:checksum success !!\n");

        flash_bootloader_send_ack(buff[0],1);

        printmsg("BL_DEBUG_MSG: mem write address : %#x\n",mem_address);
		
    if ( mem_address >= FLASH_BASE && mem_address <= FLASH_END)
		{
            printmsg("BL_DEBUG_MSG: valid mem write address\n");

            //glow the led to indicate bootloader is currently writing to memory
            HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_12, GPIO_PIN_SET);

            //execute mem write
            write_status = execute_mem_write(&buff[7],mem_address, payload_len);

            //turn off the led to indicate memory write is over
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

            //inform host about the status
            bootloader_uart_write_data(&write_status,1);

		}else
		{
            printmsg("BL_DEBUG_MSG: invalid mem write address\n");
            write_status = ADDR_INVALID;
            //inform host that address is invalid
            bootloader_uart_write_data(&write_status,1);
		}


	}else
	{
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");
        flash_bootloader_send_nack();
	}
	
}
/*This function writes the contents of pBuffer to  "mem_address" byte by byte */
//Note1 : Currently this function supports writing to Flash only .
//Note2 : This functions does not  check whether "mem_address" is a valid address of the flash range.
uint8_t execute_mem_write(uint8_t *buff, uint32_t mem_address, uint32_t len)
{
    uint8_t status=HAL_OK;

    //We have to unlock flash module to get control of registers
    HAL_FLASH_Unlock();

    for(uint32_t i = 0 ; i <len ; i++)
    {
        //Here we program the flash byte by byte
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,mem_address+i,buff[i] );
    } 

    HAL_FLASH_Lock();

    return 1;
}


void flash_bootloader_send_ack(uint8_t command_code, uint8_t follow_len){
	uint8_t ack_buff[2];
	ack_buff[0]=command_code;
	ack_buff[1]=follow_len;
	HAL_UART_Transmit(&huart3,(uint8_t*)ack_buff,sizeof(ack_buff),HAL_MAX_DELAY);
}

void flash_bootloader_send_nack(void){
	uint8_t nack=BL_NACK; 
	HAL_UART_Transmit(&huart3,&nack,sizeof(nack),HAL_MAX_DELAY);
	 
}



uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector){
	// if sector_number = 0xff , that means mass eras, otherwise sector eras
	//we have totally 8 sectors in STM32F446RE mcu .. sector[0 to 7]
	//number_of_sector has to be in the range of 0 to 7
	// if sector_number = 0xff , that means mass erase !
	//Code needs to modified if your MCU supports more flash sectors
	FLASH_EraseInitTypeDef flashErase_handle;
	uint32_t sectorError;
	HAL_StatusTypeDef status;


	if( number_of_sector > 8 )
		return INVALID_SECTOR;

	if( (sector_number == 0xff ) || (sector_number <= 7) )
	{
		if(sector_number == (uint8_t) 0xff)
		{
			flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}else
		{
		    /*Here we are just calculating how many sectors needs to erased */
			uint8_t remanining_sector = 8 - sector_number;
            if( number_of_sector > remanining_sector)
            {
            	number_of_sector = remanining_sector;
            }
			flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
			flashErase_handle.Sector = sector_number; // this is the initial sector
			flashErase_handle.NbSectors = number_of_sector;
		}
		flashErase_handle.Banks = FLASH_BANK_1;

		/*Get access to touch the flash registers */
		HAL_FLASH_Unlock();
		flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // our mcu will work on this voltage range
		status =  HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
		HAL_FLASH_Lock();

		return status;
	}


	return INVALID_SECTOR;
}

void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len){ // write data tu uart
	HAL_UART_Transmit(&huart3,pBuffer,len,HAL_MAX_DELAY);
}


