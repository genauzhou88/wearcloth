/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : hw_config.h
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <usb_type.h>
#include "stm32l1xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

#define USART_RX_DATA_SIZE   2048 

#define USB_OUT_BUFFER_IS_FULL() (USART_Rx_ptr_in == USART_Rx_ptr_out - 1 || (USART_Rx_ptr_in == USART_RX_DATA_SIZE && USART_Rx_ptr_out == 0))
#define USB_OUT_BUFFER_IS_EMPTY() (USART_Rx_ptr_in == USART_Rx_ptr_out)

#define UNPACK_2_BYTE_PARAMETER(ptr)  \
                (uint16_t)((uint16_t)(*((uint8_t *)ptr))) |   \
                (uint16_t)((((uint16_t)(*((uint8_t *)ptr + 1))) << 8))


#define UNPACK_3_BYTE_PARAMETER(ptr)  \
                (uint32_t)(((uint32)(*((uint8_t *)ptr))) << 16)    |   \
                (uint32_t)(((uint32)(*((uint8_t *)ptr + 1))) << 8) |   \
                (uint32_t)(((uint32)(*((uint8_t *)ptr + 2))))


#define UNPACK_4_BYTE_PARAMETER(ptr)  \
                (uint32_t)(((uint32)(*((uint8_t *)ptr))) << 24)     |   \
                (uint32_t)(((uint32)(*((uint8_t *)ptr + 1))) << 16) |   \
                (uint32_t)(((uint32)(*((uint8_t *)ptr + 2))) << 8)  |   \
                (uint32_t)(((uint32)(*((uint8_t *)ptr + 3))))


#define PACK_2_BYTE_PARAMETER(ptr, param)  do{\
                *((uint8_t *)ptr) = (uint8_t)(param);   \
                *((uint8_t *)ptr+1) = (uint8_t)(param)>>8; \
                }while(0)

#define PACK_3_BYTE_PARAMETER(ptr, param)  do{\
                *((uint8_t *)ptr) = (uint8_t)(param);   \
                *((uint8_t *)ptr+1) = (uint8_t)(param)>>8; \
                *((uint8_t *)ptr+2) = (uint8_t)(param)>>16; \
                }while(0)

#define PACK_4_BYTE_PARAMETER(ptr, param)  do{\
                *((uint8_t *)ptr) = (uint8_t)(param);   \
                *((uint8_t *)ptr+1) = (uint8_t)(param)>>8; \
                *((uint8_t *)ptr+2) = (uint8_t)(param)>>16; \
                *((uint8_t *)ptr+3) = (uint8_t)(param)>>24; \
                }while(0)


/* Exported functions ------------------------------------------------------- */
void Set_System(void);
void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config (FunctionalState NewState);

void Handle_USBAsynchXfer (void);
void Get_SerialNum(void);
void msDelay (uint32_t msec);
void usDelay(uint32_t usec);
void USB_Send_Data(uint8_t byte);
void _100nsDelay(void);
void USB_Init(void);

/* External variables --------------------------------------------------------*/

#endif  /*__HW_CONFIG_H*/
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
