/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : hal.c
* Author             : AMS - HEA&RF BU
* Version            : V1.0.0
* Date               : 04-Oct-2013
* Description        : Implementation of some APIs to access BlueNRG interface.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <hal.h> 
#include "platform_config.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_tim.h"
#include "hw_config.h"
#include "low_power.h"
#include "misc.h"
#include <hci.h>
#include "gp_timer.h"

#include "SDK_EVAL_Spi_Driver.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Constants
 *****************************************************************************/

/******************************************************************************
 * Types
 *****************************************************************************/

/******************************************************************************
 * Local Function Prototypes
 *****************************************************************************/

/******************************************************************************
 * Global Functions
 *****************************************************************************/
     
#define SPI_BUFFER_SIZE 256
volatile unsigned int spi_ptr_in = 0, spi_buff_len = 0;
unsigned spi_ptr_out = 0;
uint8_t spi_buffer[SPI_BUFFER_SIZE];

/*
* Returns 1 if the byte is successfully inserted in the buffer, 0 otherwise.
*/
int spi_input(uint8_t byte)
{
    if(spi_buff_len == SPI_BUFFER_SIZE)
        return 0;
    
    spi_buffer[spi_ptr_in] = byte;
    spi_ptr_in++;
    spi_buff_len++;
    if(spi_ptr_in == SPI_BUFFER_SIZE){
        spi_ptr_in = 0;
    }
    
    return 1;
}

int spi_fifor(void)
{
    return spi_buff_len;
}

void spi_read_ms(int count, char *data)
{
    int i;
    
    for(i = 0; spi_ptr_out != spi_ptr_in && i < count; i++){
        
        data[i] = spi_buffer[spi_ptr_out++];
        if(spi_ptr_out == SPI_BUFFER_SIZE)
            spi_ptr_out = 0;
        
    }
    spi_buff_len -= i;
}

void Hal_Write_Serial(const void* data1, const void* data2, tHalInt32 n_bytes1, tHalInt32 n_bytes2)
{
    struct timer t;
            
	Timer_Set(&t, CLOCK_SECOND/10);
    
    while(1){
      if(BlueNRG_SPI_Write((uint8_t *)data1,(uint8_t *)data2, n_bytes1, n_bytes2)==0) break;
      if(Timer_Expired(&t)){
          break;
      }
    }
}



/****************************************************************************/


