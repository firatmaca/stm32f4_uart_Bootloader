

/* This file implements the logic to read and process the replies from the Bootloader .
 * This file is common across win/linux/mac
 */

#include "main.h"



//Reads and processes the reply sent from the MCU bootloader
int read_bootloader_reply(uint8_t command_code)
{
    uint8_t ack[2]={0}; //MCU sends ack + len field
    uint32_t len_to_follow=0;
    int ret = -2;

    //The MCU bootloader always sends ack/nack first . read that !!
    read_serial_port(ack,2);
    if(ack[0] == 0xA5)
    {
        //CRC of last command was good .. received ACK and "len to follow"
        len_to_follow=ack[1];
        printf("\n\n   CRC : SUCCESS Len : %d\n",len_to_follow);

        switch(0x50 | command_code)
        {

        case COMMAND_BL_FLASH_ERASE:
            process_COMMAND_BL_FLASH_ERASE(len_to_follow);
            break;
        case COMMAND_BL_MEM_WRITE:
            process_COMMAND_BL_MEM_WRITE(len_to_follow);
            break;

        default:
            printf("\n  Invalid command code\n");

        }

          ret = 0;
    }
    else if( ack[0] == 0x7F)
    {
        //CRC of last command was bad .. received NACK
        printf("\n   CRC: FAIL \n");
        ret= -1;
    }

    return ret;
}

void process_COMMAND_BL_FLASH_ERASE(uint32_t len)
{
    uint8_t erase_status=0;
    read_serial_port(&erase_status,len);
    if(erase_status == Flash_HAL_OK)
    {
        printf("\n  Erase Status: Success  Code: Flash_HAL_OK\n");
    }
    else if(erase_status == Flash_HAL_ERROR)
    {
        printf("\n  Erase Status: Fail  Code: Flash_HAL_ERROR\n");

    }
    else if(erase_status == Flash_HAL_BUSY)
    {
        printf("\n  Erase Status: Fail  Code: Flash_HAL_BUSY\n");
    }
    else if(erase_status == Flash_HAL_TIMEOUT)
    {
        printf("\n  Erase Status: Fail  Code: Flash_HAL_TIMEOUT\n");
    }
     else if(erase_status == Flash_HAL_INV_ADDR)
    {
        printf("\n  Erase Status: Fail  Code: Flash_HAL_INV_SECTOR\n");
    }
    else
    {
        printf("\n  Erase Status: Fail  Code: UNKNOWN_ERROR_CODE\n");
    }
   // printf("     Erase Status : 0x%x\n",erase_status);
}

void process_COMMAND_BL_MEM_WRITE(uint32_t len)
{
    uint8_t write_status=0;
    read_serial_port(&write_status,len);
    if(write_status == Flash_HAL_OK)
    {
        printf("\n   Write_status: Flash_HAL_OK\n");
    }
    else if(write_status == Flash_HAL_ERROR)
    {
        printf("\n   Write_status: Flash_HAL_ERROR\n");
    }
    else if(write_status == Flash_HAL_BUSY)
    {
        printf("\n   Write_status: Flash_HAL_BUSY\n");
    }
    else if(write_status == Flash_HAL_TIMEOUT)
    {
        printf("\n   Write_status: Flash_HAL_TIMEOUT\n");
    }
     else if(write_status == Flash_HAL_INV_ADDR)
    {
        printf("\n   Write_status: Flash_HAL_INV_ADDR\n");
    }
    else
    {
        printf("\n   Write_status: UNKNOWN_ERROR\n");
    }
   // printf("     Erase Status : 0x%x\n",erase_status);
}


int check_flash_status(void)
{
    uint8_t ack[2]={0};
    //uint32_t i =0;
    int ret = -1;

    uint8_t ch = 0;

    //The MCU bootloader always sends ack/nack first . read that !!
    read_serial_port(ack,2);
    if(ack[0] == 0xA5)
    {
        //CRC of last command was good .. received ACK and "len to follow"
        printf("ACK received : %d\n",ack[1]);
        //ReadFile(hComm, &ch, 1, &read, NULL);
        read_serial_port(&ch,1);
        printf("flash status : %d\n",ch);
        ret = 0;
    }
    else if ( ack[0] == 0x7F)
    {
        //CRC of last command was bad .. received NACK
        printf("\nNACK received \n");
        ret= -1;
    }

    return ret;
}
