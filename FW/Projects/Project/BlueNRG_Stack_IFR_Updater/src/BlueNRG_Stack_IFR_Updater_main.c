/******************** (C) COPYRIGHT 2014 STMicroelectronics ********************
* File Name          : BlueNRG_Stack_IFR_Updater_main.c
* Author             : AMS - AAS Division
* Version            : V1.0.0
* Date               : 20-May-2014
* Description        : BlueNRG demo example for IFR updater and stack updater
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/**
 * @file  BlueNRG_Stack_IFR_Updater_main.c
 * @brief This is a  demo that shows how to update BlueNRG IFR and BlueNRG stack image
 *
 * <!-- Copyright 2014 by STMicroelectronics.  All rights reserved.       *80*-->

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM.
  -# From the File->Open->Workspace menu, open the IAR workspace
     <tt> ...\\Projects\\Project\\BlueNRG_Updater_IFR\\EWARM\\BlueNRG_Stack_IFR_Updater.eww </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect STLink to JTAG connector  in your board (if available).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG GUI, put the board in DFU mode and download the built binary image.

* \subsection IAR_project_configurations IAR project configurations

  - \c IFR_Updater - Verify and Update BlueNRG IFR (it defines APPLY_BLUENRG_IFR_UPDATER  on preprocessor options).
  - \c Stack_Updater - Update BlueNRG stack image and if necessary the bootloader (it defines APPLY_BLUENRG_STACK_UPDATER on preprocessor options).
 
* \section Prebuilt_images Prebuilt images
  -  NA 
 
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
@table
|                  | STEVAL-IDB002V1                         | STEVAL-IDB003V1                         |                    
| LED name         | IFR_Updater/ Stack Updater              | IFR_Updater/ Stack Updater              | 
--------------------------------------------------------------------------------------------------------
| D1               | Blinking  if update operation is OK     | NA                                      | 
| D2               | Blinking  if update operation is not OK | Blinking  if update operation is OK     |          
| D3               | Not used                                | Blinking  if update operation is not OK |  
| D4               | Not used                                | NA                                      |      
| D5               | Not used                                | NA                                      |          
| D6               | Not used                                | NA                                      |       
@endtable
- NA:   Not Applicable;

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

 This demo shows how to performs the following two operations:

* \subsection BlueNRG_IFR_Updater BlueNRG IFR update: it verifies if BlueNRG IFR configuration is already the one selected by the user and, if not, it programs BlueNRG IFR.
 
   - IMPORTANT: Before using this utility, make sure the BlueNRG bootloader has been already updated using the BlueNRG Stack Updater workspace.

   - Required Files: 
    - \c bluenrg_IFR.c: it contains the \c IFR_config() data to be programmed on BlueNRG IFR.
    - \c bluenrg_utils.c: it provides the IFR utility APIs for upgrading BlueNRG IFR with selected \c IFR_config() data. 
    - \c bluenrg_utils.h: header file for BlueNRG IFR updater & BlueNRG stack updater utility APIs.

   - Required APIs: 
    - \c verify_IFR(): it checks if the IFR has been already correctly programmed with the new selected configuration.
    - \c program_IFR(): it programs IFR with the new selected configuration.

   - BlueNRG IFR update steps: 
    -# User must add one of the following define values  on IAR preprocessor options,  based on the selected IFR configuration:
        - \c BLUENRG_CONFIG=BLUENRG_32_MHZ    (32MHz high speed crystal and external 32 kHz low speed crystal)
        - \c BLUENRG_CONFIG=BLUENRG_32_MHZ_RO (32MHz high speed crystal and internal low speed ring oscillator)
        - \c BLUENRG_CONFIG=BLUENRG_16_MHZ    (16MHz high speed crystal and external 32 kHz low speed crystal)
        - \c BLUENRG_CONFIG=BLUENRG_16_MHZ_RO (16MHz high speed crystal and internal low speed ring oscillator)
    -# On user \c main.c application, just after \c BlueNRG_RST(); line, add these instructions:
        - \c if (\c bootloader_version \c != \c latest_bootloader_version) \c while(1);
        - \c ret=verify_IFR(\c &IFR_config);
        - \c if(ret) \c {\c ret=program_IFR(\c &IFR_config); \c if(ret) \c while(1);}
 
* \subsection BlueNRG_Updater BlueNRG stack image update: it programs BlueNRG with the selected stack image provided through a C array, also if the bootloader is an old version it programs the latest bootloader version in the first sector. 

   - Required Files: 
    - \c bluenrg_utils.c: it provides the utility API for upgrading BlueNRG stack and bootloader with selected stack image. 
    - \c bluenrg_utils.h: header file for BlueNRG IFR updater & BlueNRG stack updater utility APIs.

   - Required APIs: 
    - \c program_device(): it upgrades the BlueNRG stack with the selected binary image.
   
- BlueNRG stack image update steps (use \c bluenrg_6_3_Mode_2-16MHz.img as reference example):
    -# Select the BlueNRG stack image file \c bluenrg_6_3_Mode_2-16MHz.img (all available stack images are provided on Firmware folder)
    -# Apply the \c bluenrg_fw_to_C_array.exe utility in order to convert the selected \c *.img to a \c *.c file (it is provided inside the Utilities folder)
    -# On user \c main.c application, include the file \c bluenrg_6_3_Mode_2-16MHz.c, and, after \c BlueNRG_RST(); line, add these instructions: 
      - \c if (\c bootloader_version \c != \c latest_bootloader_version) \c { \c ret=program_device(\c bootloader_image, \c sizeof(\c bootloader_image),\c TRUE); \c}
      - \c Clock_Wait(500); 
      - \c ret=program_device(\c fw_image, \c sizeof(\c fw_image),\c FALSE); 
      - \c if(ret) \c while(1);

    - NOTEs: 
     - \c bluenrg_6_3_Mode_2-16MHz.c file is generated from \c bluenrg_6_3_Mode_2-16MHz.img using the \c bluenrg_fw_to_C_array.exe utility. 
     - For information about how using the \c bluenrg_fw_to_C_array.exe refer to the related doxygen documentation.
**/
/** @addtogroup BlueNRG_Stack_IFR_Updater_Utilities
 * BlueNRG IFR updater & stack image updater examples \see BlueNRG_Stack_IFR_Updater_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "hw_config.h"
#include "hal_types.h"
#include "hci.h"
#include "bluenrg_hci.h"
#include "gp_timer.h"
#include "hal.h"
#include "osal.h"
#include "gatt_server.h"
#include "hci_internal.h"
#include "bluenrg_hci_internal.h"
#include "gap.h"
#include "sm.h"
#include <stdio.h>
#include "SDK_EVAL_Config.h"
/* APIs supporting BlueNRG IFR updater & stack updater functionalities */
#include <bluenrg_utils.h>

/* Update BlueNRG Stack image: include the c arrays generated from the selected 
   BlueNRG firmware stack image and the Bootloader image.

   NOTE: The c array image must be previusly generated from the BlueNRG  
   stack *.img file available on Firmware folder, using the 
   bluenrg_fw_to_C_array.exe utility (on Utility folder).
*/
#if defined (APPLY_BLUENRG_STACK_UPDATER) 
#include "bluenrg_6_3_Mode_2-16MHz.c"  
#include "bluenrg_bootloader_update_6_xto6_3_v2.c"
#endif /* APPLY_BLUENRG_STACK_UPDATER */

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define HW_CUT_30             0x30
#define BLUENRG_FW_60         0x0600
#define BLUENRG_FW_TO_UPDATE  0x0630
#define RIGHT_UPDATER_VERSION 5

/* Private macros ------------------------------------------------------------*/
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifndef VECTOR_TABLE_BASE_ADDRESS 
/* default configuration: DFU upgrade is supported */
#define VECTOR_TABLE_BASE_ADDRESS            (0x3000)
#endif

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void statusProcedureOk(uint8_t procedureOk)
{
  while (1) {
      /* Just toggling LED1: update is OK */
    if (procedureOk)
      SdkEvalLedToggle(LED1);
    else
      SdkEvalLedToggle(LED2);
    Clock_Wait(500);
  }
}

/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
    int ret;
    uint8_t hwVersion, updaterVersion;
    uint16_t fwVersion;

    NVIC_SetVectorTable(NVIC_VectTab_FLASH, VECTOR_TABLE_BASE_ADDRESS);
    
    /* Identify BlueNRG platform */
    SdkEvalIdentification();

    RCC_Configuration();
    
    /* Init I/O ports */
    Init_GPIOs ();
    
    PWR_PVDCmd(DISABLE);
    
    /* Disable FLASH during Sleep  */
    FLASH_SLEEPPowerDownCmd(ENABLE);
    
    /* Enable Ultra low power mode */
    PWR_UltraLowPowerCmd(ENABLE);
    
    PWR_FastWakeUpCmd(DISABLE);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    
    Clock_Init();

    HCI_Init();
    
    /* Init SPI interface */
    SdkEvalSpiInit(SPI_MODE_EXTI);
    
    SdkEvalLedInit(LED1);
    SdkEvalLedInit(LED2);
    SdkEvalLedOff(LED1);
    SdkEvalLedOff(LED2);

    BlueNRG_RST();

    fwVersion = 0;
    hwVersion = 0;
    updaterVersion = 0;

    /* GET Updater Version */
    if (getBlueNRGUpdaterVersion(&updaterVersion) != BLE_STATUS_SUCCESS)
      statusProcedureOk(FALSE);

    /* WARNING: Get BlueNRG stack version.  */
    getBlueNRGVersion(&hwVersion, &fwVersion);

    /* WARNING: The only HW BlueNRG cut version supported
       is greater or equal than 3.0 */
    if ((hwVersion < HW_CUT_30) && (hwVersion != 0))  {
      statusProcedureOk(FALSE);
    }

#if defined (APPLY_BLUENRG_STACK_UPDATER)  

    /* Check if the bootloader version is the latest. 
       If the version is 6.0 OR the updater version is not 5.0
       we need to update the bootloader */
    if ((fwVersion == BLUENRG_FW_60) || (updaterVersion != RIGHT_UPDATER_VERSION)) {
      /* Update the BlueNRG bootloader image using the bootloader_image */
      ret = program_device(bootloader_image, sizeof(bootloader_image),TRUE);
      if (ret) {
	statusProcedureOk(FALSE);
      }
    }
    Clock_Wait(500);
    if (fwVersion != BLUENRG_FW_TO_UPDATE) {
      /* Update the BlueNRG stack image using the fw_image */
      ret = program_device(fw_image, sizeof(fw_image),FALSE);
      if (ret) {
	statusProcedureOk(FALSE);
      }
    }

#endif /* APPLY_BLUENRG_STACK_UPDATER */

#if defined (APPLY_BLUENRG_IFR_UPDATER)
    /* Check the bootloader is the latest version */
    if ((updaterVersion != RIGHT_UPDATER_VERSION) || 
	(fwVersion != BLUENRG_FW_TO_UPDATE))  {
      statusProcedureOk(FALSE);
    }
    
    /* Verify if IFR configuration is already the one  
       selected by the user (refer to define values) */
    ret = verify_IFR(&IFR_config);
    if (ret) {
      /* IFR configuration is not the selected one: program IFR */
      ret = program_IFR(&IFR_config);
      if (ret) {
	statusProcedureOk(FALSE);
      }
    }
#endif  /* APPLY_BLUENRG_IFR_UPDATER */  
  
    while(1)
    {
      /* Just toggling LED1: update is OK */
      statusProcedureOk(TRUE);
    }
}


/**
  * @brief  This function is called whenever there is an ACI event to be processed.
  * @note   Inside this function each event must be identified and correctly
  *         parsed.
  * @param  pckt  Pointer to the ACI packet
  * @retval None
  */
void HCI_Event_CB(void *pckt)
{
    hci_uart_pckt *hci_pckt = pckt;
    hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
    
    if(hci_pckt->type != HCI_EVENT_PKT)
        return;
    
    switch(event_pckt->evt){
        
    case EVT_DISCONN_COMPLETE:
        {
            //GAP_DisconnectionComplete_CB();
        }
        break;
        
    case EVT_LE_META_EVENT:
        {
            evt_le_meta_event *evt = (void *)event_pckt->data;
            
            switch(evt->subevent){
            case EVT_LE_CONN_COMPLETE:
                {
                    //evt_le_connection_complete *cc = (void *)evt->data;
                }
                break;
            }
        }
        break;
        
    case EVT_VENDOR:
        {
            evt_blue_aci *blue_evt = (void*)event_pckt->data;
            switch(blue_evt->ecode){
                
            case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
                {
                    //evt_gatt_attr_modified *evt = (evt_gatt_attr_modified*)blue_evt->data; 
                }
                break;
            case EVT_BLUE_GATT_NOTIFICATION:
                {
                    //evt_gatt_attr_notification *evt = (evt_gatt_attr_notification*)blue_evt->data;
                }
                break;
            }
        }
        break;
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

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
