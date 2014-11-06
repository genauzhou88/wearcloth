/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : BlueNRG_VCOM_main.c
* Author             : MCD Application Team
* Version            : V1.3.0
* Date               : 03-June-2014
* Description        : BlueNRG Virtual Com Port main file
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/**
 * @file  BlueNRG_VCOM_main.c
 * @brief This is the FW application for running the BlueNRG GUI on BlueNRG development Kits 
 *
 * <!-- Copyright 2014 by STMicroelectronics.  All rights reserved.       *80*-->

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM.
  -# From the File->Open->Workspace menu, open the IAR workspace
     <tt> ...\\Projects\\Project\\Virtual_COM_Port\\EWARM\\VirtualCOMPort.eww </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect STLink to JTAG connector  in your board (if available).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG GUI, put the board in DFU mode and download the built binary image.

* \subsection IAR_project_configurations IAR project configurations

  - \c  Release - It builds the BlueNRG_VCOM_1_3.hex binary image needed for using the BlueNRG GUI
 
* \section Prebuilt_images Prebuilt images
  - BlueNRG_VCOM_1_3.hex
 
* \section Jumper_settings Jumper settings
@table
------------------------------------------------------
| Jumper name       |  Description                   | 
------------------------------------------------------
| JP1, if available | USB or Battery supply position | 

@endtable 


* \section Board_supported Board supported
@table
| Board name      | Board revision | NOTE        |
--------------------------------------------------
| STEVAL-IDB002V1 | 3.0            | Eval Kit    |
| STEVAL-IDB003V1 | 1.0            | USB Dongle  |
@endtable

* \section Serial_IO Serial I/O
- NOTE: No serial port is used

* \section LEDs_description LEDs description
- NOTE: No led is used

* \section Buttons_description Buttons description
- NOTE: No button is used

* \section DFU_Activation  DFU activation
BlueNRG boards are preprogrammed with a DFU application which allows to upload the 
STM32L micro with a selected binary image through USB. Follow list of actions 
for activating DFU on each supported platforms
@table
| Board  name          | Event                                             | Note               |
------------------------------------------------------------------------------------------------|
| STEVAL-IDB002V1      | Press RESET and Push Button. Release RESET button | LED D2 is toggling |  
| STEVAL-IDB003V1      | Press SW1 button and plug USB dongle on a PC port | LED D3 is toggling |  
@endtable

* \section Usage Usage

  - BlueNRG VCOM is the application to be loaded  in order to use the BlueNRG GUI on BlueNRG development platforms
    (STEVAL-IDB002V1, STEVAL-IDB003V1).
    - User  can use this project in order to port the BlueNRG VCOM application to his specific BlueNRG PCB 
     (assuming that the customer PCB has a USB or RS232 I/O port available for PC connection).
    - This application provides an interface compliant with the Bluetooth Low Energy DTM test commands.
    - This application is not a reference application to be used for BlueNRG application development and evaluation. 

    - For information  about how to use the  BlueNRG GUI, refer to the BlueNRG Development Kit User Manual UM1755
      available on BlueNRG web site: www.st.com/bluenrg.
**/
/** @addtogroup VirtualCOM_test_application
 * BlueNRG Virtual Com Port main file \see BlueNRG_VCOM_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#if defined  (STM32L1XX_MD) || (STM32L1XX_XL) 
 #include "stm32l1xx.h"
#else
 #include "stm32f10x.h"
#endif /* STM32L1XX_MD */
 
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "clock.h"

#include "SDK_EVAL_Config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Default BlueNRG SDK platforms uses DFU at address 0x0000 */
#ifndef VECTOR_TABLE_BASE_ADDRESS
#define VECTOR_TABLE_BASE_ADDRESS  (0x3000)
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void hci_input(uint8_t *buff, uint16_t len);
/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/

unsigned char rx_buffer[255];
uint16_t rx_bytes;  
/* buffer for the debug */
unsigned char debug_buffer[240];
uint16_t debug_bytes;

int main(void)
{
  /* Set the Vector Table base location */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, VECTOR_TABLE_BASE_ADDRESS);  
  
  /* Identify BlueNRG platform */
  SdkEvalIdentification();
  
  Set_System();
  
  Clock_Init();
  
  /* Delay needed only to be able to acces the JTAG interface after reset
    if it will be disabled later. */
  Clock_Wait(500);
  
  /* Configure I/O communication channel:
     It requires the IO_Receive_Data(uint8_t * rx_data, uint16_t data_size) function
     where user received data should be processed */
  SdkEval_IO_Config(hci_input);
  
  /* Delay for debug purpose, in order to see printed data at startup. */
  Clock_Wait(2000);
  
  /* Init the BlueNRG SPI interface in polling mode */
  SdkEvalSpiInit(SPI_MODE_GPIO); 
  /* Init the SPI eeprom CS (same SPI interface as BlueNRG module) */
  EepromCsPinInitialization();
  
  /* Reset BlueNRG SPI interface */
  BlueNRG_RST();

  while (1)
  {
    /* Check if BluenRG IRQ line is triggered */
    if (SdkEvalSPI_Irq_Pin() == Bit_SET){
      /* Data are available from BlueNRG: read them through SPI */
      rx_bytes = BlueNRG_SPI_Read_All(rx_buffer, sizeof(rx_buffer)); 
      /* Check if there is data is so, send it to VCOM */
      if (rx_bytes > 0) {
        for(int i = 0; i < rx_bytes; i++) 
          SdkEval_IO_Send_Data(rx_buffer[i]);
        rx_bytes = 0;
      }      
    }
  }
}
#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
/** \endcond
 */