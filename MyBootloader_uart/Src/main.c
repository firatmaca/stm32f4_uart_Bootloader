
#include "main.h"
#include <stdarg.h>
#include <string.h>
#include <stdint.h>

#define BL_DEBUG_MSG_EN

CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2; // tx pin pa2    rx pin pa3
UART_HandleTypeDef huart3; // tx pin pb10   rx pin pb11      
 
#define D_UART &huart3   // for debug
#define C_UART &huart2  // for bootloader
#define FlASH_SECTOR2_BASE_ADDRESS 0x08008000U  // user app address
#define bl_rx_buffer_size 200  
uint8_t bl_rx_buffer[bl_rx_buffer_size];
#define BL_ACK   0XA5
#define BL_NACK  0X7F


void bootloader_handle_flash_erase_cmd(uint8_t *buff);
void bootloader_handle_mem_write_cmd(uint8_t *buff);
void printmsg(char *format,...); //  debuging from uart3
void bootloader_uart_read_data(void);  // coiming data from host
void bootloader_jump_to_user_app(void);
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len);
void bootloader_send_nack(void);
uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len,uint32_t crc_host);
uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector);
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len);
uint8_t execute_mem_write(uint8_t *buff, uint32_t mem_address, uint32_t len);
uint8_t verify_address(uint32_t go_address);

#define BL_FLASH_ERASE        0x56 //mass erase or sector erase of the user flash
#define BL_MEM_WRITE          0x57 //write data in to different memories of the MCU
#define VERIFY_CRC_FAIL    1
#define VERIFY_CRC_SUCCESS 0
#define INVALID_SECTOR 0x04
#define ADDR_VALID 0x00
#define ADDR_INVALID 0x01

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);


char hello[] = "Hello from Bootloader \r\n";


int main(void)
{
  HAL_Init();
  SystemClock_Config(); 
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
	char abc[] ="abc \n\r";
	HAL_UART_Transmit(&huart2,(uint8_t*) abc,strlen(abc),HAL_MAX_DELAY);
  /* Lets check whether button is pressed or not, if not pressed jump to user application */ 
  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
	printmsg("Button is pressed(bootloader mode)\n\r");
	bootloader_uart_read_data();
	}
	else 
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
		printmsg("Button is not presssed(user app mode)");
		bootloader_jump_to_user_app();
	
	 }
}
void bootloader_jump_to_user_app(){
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
void bootloader_uart_read_data(){
	  uint8_t rcv_len=0;
	while(1){

		memset(bl_rx_buffer,0,200);
    HAL_UART_Receive(&huart3,bl_rx_buffer,1,HAL_MAX_DELAY);  //first bl command is "following length"
		rcv_len=bl_rx_buffer[0];
		HAL_UART_Receive(&huart3,&bl_rx_buffer[1],rcv_len,HAL_MAX_DELAY); 	//second  bl command is our cmd
	
		switch(bl_rx_buffer[1]){		 // we call the function for coming cmd
			case BL_FLASH_ERASE:
				bootloader_handle_flash_erase_cmd(bl_rx_buffer);
			break;
			
			case BL_MEM_WRITE:
				bootloader_handle_mem_write_cmd(bl_rx_buffer);
			break;
			
			default:
				printmsg("Invalid command from recieved from host \n");
			break;
		}
		
	}
	
}
//ptint uart
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


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

 
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
 
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_CRC_Init(void)
{


  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
 

}


static void MX_USART2_UART_Init(void)
{

 
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
 
}


static void MX_USART3_UART_Init(void)
{

 
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
 
}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}


void Error_Handler(void)
{
 
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{ 
 
}
#endif /* USE_FULL_ASSERT */

// Implimentation of  Bootloader Command Hanle function


void bootloader_handle_flash_erase_cmd(uint8_t *buff){
	uint8_t erase_status=0x00;
	printmsg("flash erase commend");
	uint32_t command_packet_length=bl_rx_buffer[0]+1; //buffer[0] = following length   --   +1 is buf[0]
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_length - 4) ) ; 
	/* host_crc refers addres of bufer[command_packet_length - 4]
	 Think of  bl_rx_buffer+command_packet_length - 4 as the memory address based on the original 
	 bl_rx_buffer pointer plus the number of bytes for command_packet_length - 4 uint32_t objects. */
	
	if(!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_length-4,host_crc)){
		printmsg("checksum is correct \n");
		bootloader_send_ack(buff[0],1);
		printmsg("inital_sector :%d  nofsector: %d \n",buff[2],buff[3]);
		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
		erase_status=execute_flash_erase(buff[2] , buff[3]);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
		
		printmsg("flash erase status: %#x\n",erase_status);
		  bootloader_uart_write_data(&erase_status,1);

	}else
	{
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");
        bootloader_send_nack();
	}
		
}

void bootloader_handle_mem_write_cmd(uint8_t *buff){
	uint8_t addr_valid = ADDR_VALID;
	uint8_t write_status = 0x00;
	uint8_t chksum =0, len=0;
	len = buff[0];
	uint8_t payload_len = buff[6];

	uint32_t mem_address = *((uint32_t *) ( &buff[2]) );

	chksum = buff[len];

    printmsg("BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;


	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg("BL_DEBUG_MSG:checksum success !!\n");

        bootloader_send_ack(buff[0],1);

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
        bootloader_send_nack();
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


void bootloader_send_ack(uint8_t command_code, uint8_t follow_len){
	uint8_t ack_buff[2];
	ack_buff[0]=command_code;
	ack_buff[1]=follow_len;
	HAL_UART_Transmit(&huart3,(uint8_t*)ack_buff,sizeof(ack_buff),HAL_MAX_DELAY);
}

void bootloader_send_nack(void){
	uint8_t nack=BL_NACK; 
	HAL_UART_Transmit(&huart3,&nack,sizeof(nack),HAL_MAX_DELAY);
	 
}

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


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
