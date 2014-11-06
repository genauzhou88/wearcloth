/******************** (C) COPYRIGHT 2014 STMicroelectronics ********************
* File Name          : OTA_ServiceManager_main.c
* Author             : AMS - AAS division
* Version            : V1.0.0
* Date               : 14-February-2014
* Description        : BlueNRG OTA Service Manager Application.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/**
 * @file  OTA_ServiceManager_main.c
 * @brief This application implements a basic standalone BLE OTA bootloader. 
 *        It provides the BLE Over-The-Air Service for handling the OTA upgrade
 *        of a BLE application always  from a fixed base address on user Flash (0x8006200).
 *        It assumes that the OTA_ResetManager application has been already downloaded.
 * 
 * NOTE: OTA service support is enabled through ST_OTA_BTL (preprocessor & linker) options and btl.[ch] file.
 * <!-- Copyright 2014 by STMicroelectronics.  All rights reserved.       *80*-->

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM.
  -# From the File->Open->Workspace menu, open the IAR workspace
     <tt> ...\\Projects\\Project\\OTA_ServiceManager\\EWARM\\OTA_ServiceManager.eww </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect STLink to JTAG connector  in your board (if available).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG GUI, put the board in DFU mode and download the built binary image.
  -# Connect the application board to a PC USB port. 

* \subsection IAR_project_configurations IAR project configurations

  - \c App  - OTA Service Manager Application
  
* \section Prebuilt_images Prebuilt images
  - OTA_ServiceManager_App.hex
 
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
@table
| Parameter name  | Value          | Unit        |
--------------------------------------------------
| Baudrate        | 115200         | bit/sec     |
| Data bits       | 8              | bit         |
| Parity          | None           | bit         |
| Stop bits       | 1              | bit         |
@endtable

* \section LEDs_description LEDs description
@table
|                              
| LED name         |STEVAL-IDB002V1  | STEVAL-IDB003V1  |  
---------------------------------------------------------
| D1               | Blinking        |  NA              |
| D2               | ON during OTA   |  Blinking        |       
| D3               | Not used        |  ON during OTA   |  
| D4               | Not used        |  NA              |     
| D5               | Not used        |  NA              |            
| D6               | Not used        |  NA              |       
@endtable
- NA: Not Applicable

* \section Buttons_description Buttons description
@table
|                      
| Button name      |STEVAL-IDB002V1 | STEVAL-IDB003V1 |
-------------------------------------------------------
| RESET            | X              | NA              |  
| Push Button      | Not used       | NA              |   
| Jostick Sel      | (1)            | NA              |  
| SW1              | NA             | Not Used        |  
| SW2              | NA             |(1)              |   
@endtable
- NA: Not Applicable;
- (1): On Power up or Reset, pressing this key the OTA EEPROM application base address information are cleaned
     (Flash address where last valid BLE application has been downloaded through OTA service).

* \section DFU_Activation  DFU activation
BlueNRG boards are preprogrammed with a DFU application which allows to upload the 
STM32L micro with a selected binary image through USB. Follow list of action 
for activating DFU on each supported board
@table
| Board  name          | Event                                             | Note               |
------------------------------------------------------------------------------------------------|
| STEVAL-IDB002V1      | Press RESET and Push Button. Release RESET button | LED D2 is toggling |  
| STEVAL-IDB003V1      | Press SW1 button and plug USB dongle on a PC port | LED D3 is toggling |  
@endtable

* \section Usage Usage

 - The OTA Service Manager is a basic application which only supports the OTA BTL service.
 - It is stored after the OTA_ResetManager and it provides the BLE OTA bootloader service to
   any BLE application stored at fixed base address on user Flash (0x8006200).
 - User is only requested to load the OTA_Reset manager and then the OTA_ServiceManager application 
   using the DFU and to enable the ST_OTA_BASIC_APPLICATION as define and linker option on
   its EWARM workspace. 
 - Further, for jumping to the OTA Service Manager application, the 
   Switch_To_OTA_Service_Manager_Application() can be called (just using a platform button to 
   activate such call). 

  - NOTE: Refer to BLE_Chat, Server_OTA_Basic and SensorDemo, OTA_Basic EWARM workspaces for related examples.

**/
/** @addtogroup BlueNRG_demonstrations_applications
 * BlueNRG OTA Service manager \see OTA_ServiceManager_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
//#include "usb_lib.h"
#include "hw_config.h"
#include "hal_types.h"
#include "hci.h"
#include "bluenrg_hci.h"
#include "gp_timer.h"
#include "hal.h"
#include "osal.h"
//#include "gatt_db.h"
#include "gatt_server.h"
#include "hci_internal.h"
#include "bluenrg_hci_internal.h"
#include "gap.h"
#include "sm.h"
//#include <stdio.h>
#include <platform_config.h>
#include <string.h>
#include "SDK_EVAL_Config.h"

/* include OTA bootloader header file */
#include "btl.h"

#ifndef DEBUG
#define DEBUG 0
#endif

#define APPLICATION_JUMP_ADDRESS (0x08000000 + VECTOR_TABLE_BASE_DFU_OFFSET + VECTOR_TABLE_BASE_RESET_MANAGER_OFFSET + VECTOR_TABLE_BASE_ADDRESS_OTA_BASIC_APP)
static GPIO_TypeDef* ButtonPort;
static uint16_t ButtonPin;

volatile uint8_t set_connectable = 1;
tHalUint16 connection_handle = 0;

#ifdef ST_OTA_BTL
static uint16_t led_blinking_rate = 500; 
#endif

void Configure_Button(void)
{
  
#ifdef USER_DEFINED_PLATFORM    
    /* Select BUTTON_SEL  push-button */
    ButtonPort = USER_BUTTON1_GPIO_PORT;
    ButtonPin = USER_BUTTON1_GPIO_PIN;  
#else /* Runtime configuration for BlueNRG Development board and USB Dongle platform */
    /* Select button & led resource */
    if(SdkEvalGetVersion() == SDK_EVAL_VERSION_3) 
    {
      /* Select User push-button */
      ButtonPort = SCM_PS_BUTTON_GPIO_PORT;
      ButtonPin = SCM_PS_BUTTON_PIN; 
    }
    else if (SdkEvalGetVersion() == SDK_EVAL_VERSION_D1)
    {
      /* Select  button SW1 for BlueNRG USB Dongle */
      ButtonPort = SCM_PS_BUTTON_VD1_GPIO_PORT;
      ButtonPin = SCM_PS_BUTTON_VD1_PIN;
    }
#endif /* USER_DEFINED_PLATFORM */
}

/**
  * @brief  Configures peripherals' clock.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
  
  /* Enable the GPIOs Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC| RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOE| RCC_AHBPeriph_GPIOH , ENABLE);     
    
  /* Enable SYSCFG */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG , ENABLE);
  
  /* Set low power configuration */
  //RCC->AHBLPENR = RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN|RCC_AHBENR_GPIOCEN|RCC_AHBENR_GPIODEN;       
  //RCC->APB1LPENR = RCC_APB1ENR_PWREN|RCC_APB1ENR_TIM2EN;
  //RCC->APB2LPENR = 0;  
}

/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void setConnectable(void);

int connected = FALSE;

int main(void)
{
    //int ret;
    
    NVIC_SetVectorTable(NVIC_VectTab_FLASH,VECTOR_TABLE_BASE_ADDRESS);
    
    /* Identify the BlueNRG platform */
    SdkEvalIdentification();

    RCC_Configuration();
    /* Basic button init function for handling application jumping */
    Configure_Button();
 
#if 0 /* TBR */
    PWR_PVDCmd(DISABLE);
    
    /* Disable FLASH during Sleep  */
    FLASH_SLEEPPowerDownCmd(ENABLE);
    
    /* Enable Ultra low power mode */
    PWR_UltraLowPowerCmd(ENABLE);
    
    PWR_FastWakeUpCmd(DISABLE);
#endif 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    
    Clock_Init();
    
    HCI_Init();
 
    /* Init SPI interface */
    SdkEvalSpiInit(SPI_MODE_EXTI);
    /* Reset BlueNRG SPI interface */
    BlueNRG_RST();
    
    /* Init leds */
    SdkEvalLedInit(LED1);
    SdkEvalLedInit(LED2);

    {
        tHalUint8 bdaddr[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02};

        aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN,
                                        bdaddr);
    }
    
    aci_gatt_init();    
    
    {
        uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
        aci_gap_init(1, &service_handle, &dev_name_char_handle, &appearance_char_handle);        
    }
    
#if 0/* TBR */
    aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                       OOB_AUTH_DATA_ABSENT,
                                       NULL,
                                       7,
                                       16,
                                       USE_FIXED_PIN_FOR_PAIRING,
                                       123456,
                                       BONDING);
#endif 
    
    //PRINTF("BLE Stack Initialized.\n");
    
#ifdef ST_OTA_BTL
    /* Add OTA bootloader service */
    Add_Btl_Service();
#endif
    
    /* -2 dBm output power */
    aci_hal_set_tx_power_level(1,4);
    
    while(1)
    {
#ifdef ST_OTA_BTL
      static tClockTime startTime = 0;

      if (Clock_Time() - startTime >led_blinking_rate)
      {    
        /* LED D1 is toggling on OTA_Service Manager */
        SdkEvalLedToggle(LED1);     
        startTime = Clock_Time();
      }
#endif /* end ST_OTA_BTL */

        HCI_Process();
        
        if(set_connectable){
            setConnectable();
            set_connectable = 0;
        }
        
      /* Use button to switch to the basic Reset Manager */
      if (GPIO_ReadInputDataBit(ButtonPort,ButtonPin) == RESET)
      {
        /* Add delay to avoid conlict with DFU activation */
        Clock_Wait(2000);
        
        /* Check if an application has been loaded previously through OTA service
           manager */
        if (*((uint32_t*) NEW_APP_MEM_INFO)!= 0) 
          /* Service Manager will jump to the Application previously loaded at
           address  APPLICATION_JUMP_ADDRESS */
          Switch_To_OTA_Service_Manager_Application(APPLICATION_JUMP_ADDRESS);
      }
    }
}

void setConnectable(void)
{  
    /* Set Name as OTAServiceMgr */
    const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'O','T','A','S','e','r','v','i','c','e','M','g','r'};

#ifdef ST_OTA_BTL
    /* Add OTA service UUID to scan response */
    hci_le_set_scan_resp_data(18,BTLServiceUUID4Scan); 
#endif 
    //PRINTF("General Discoverable Mode.\n");
    
    aci_gap_set_discoverable(ADV_IND, 0, 0, RANDOM_ADDR, NO_WHITE_LIST_USE,
                             14, local_name, 0, NULL, 0, 0);    

    //PRINTF("%d\n",ret);
}

void GAP_ConnectionComplete_CB(tHalUint8 addr[6], tHalUint16 handle){
    
    connected = TRUE;
    connection_handle = handle;
 
#if 0 /* TBR */
    //PRINTF("Connected to device:");
    for(int i = 5; i > 0; i--){
        PRINTF("%02X-", addr[i]);
    }
    PRINTF("%02X\r\n", addr[0]);
#endif 
    
#ifdef ST_OTA_BTL
    led_blinking_rate = 125;
#endif
}

void GAP_DisconnectionComplete_CB(void){
    connected = FALSE;

    //PRINTF("Disconnected\n");
    
    /* Make the device connectable again. */
    set_connectable = TRUE;
#ifdef ST_OTA_BTL
    led_blinking_rate = 500;
#endif
}


void HCI_Event_CB(void *pckt)
{
    hci_uart_pckt *hci_pckt = pckt;
    hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
    
    if(hci_pckt->type != HCI_EVENT_PKT)
        return;
    
    switch(event_pckt->evt){
        
    case EVT_DISCONN_COMPLETE:
        {
            GAP_DisconnectionComplete_CB();
        }
        break;
        
    case EVT_LE_META_EVENT:
        {
            evt_le_meta_event *evt = (void *)event_pckt->data;
            
            switch(evt->subevent){
            case EVT_LE_CONN_COMPLETE:
                {
                    evt_le_connection_complete *cc = (void *)evt->data;
                    GAP_ConnectionComplete_CB(cc->peer_bdaddr,cc->handle);
                }
                break;
            }
        }
        break;
        
    case EVT_VENDOR:
        {
            evt_blue_aci *blue_evt = (void*)event_pckt->data;
            switch(blue_evt->ecode){
                
#ifdef ST_OTA_BTL
            /* We need to indicate to the stack that the response can be sent to the client */
            case EVT_BLUE_GATT_READ_PERMIT_REQ:
                {
                    //evt_gatt_read_permit_req *pr = (void*)blue_evt->data;                    
                    if(connection_handle !=0)
                        aci_gatt_allow_read(connection_handle);                 
                }
                break;      
#endif
            case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
                {  
#ifdef ST_OTA_BTL
                  evt_gatt_attr_modified *am = (void*)blue_evt->data;
                  OTA_Write_Request_CB(am);  
#endif
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
