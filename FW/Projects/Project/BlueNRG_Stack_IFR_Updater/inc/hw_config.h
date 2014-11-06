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
#include "usb_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

#define USART_RX_DATA_SIZE   2000

#define USB_OUT_BUFFER_IS_FULL() (USART_Rx_ptr_in == USART_Rx_ptr_out - 1 || (USART_Rx_ptr_in == USART_RX_DATA_SIZE && USART_Rx_ptr_out == 0))
#define USB_OUT_BUFFER_IS_EMPTY() (USART_Rx_ptr_in == USART_Rx_ptr_out)

/* Exported functions ------------------------------------------------------- */
void Set_System(void);

void Init_GPIOs(void);
void RCC_Configuration(void);
void GPIO_LowPower_Config(void);
void Restore_GPIO_Config(void);
void Enter_LP_Sleep_Mode(void);
void Exit_LP_Sleep_Mode(void);
void BlueNRG_RST(void);

void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config (FunctionalState NewState);
void SPI_Config(void);
void SPI_Disable(void);
void Handle_USBAsynchXfer (void);
void Get_SerialNum(void);
int USB_free_buffer_size();
void USB_Send_Data(uint8_t byte);
void us150Delay();

/* External variables --------------------------------------------------------*/

#endif  /*__HW_CONFIG_H*/
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
