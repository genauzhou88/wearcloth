/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : OTA_ResetManager_main.c
* Author             : AMS - AAS BU
* Version            : V1.0.1
* Date               : 10-March-2014
* Description        : BlueNRG OTA Reset manager file
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/**
 * @file  OTA_ResetManager_main.c
 * @brief This application implements the OTA Reset Manager which, at reset,  
 *        passes control to the latest valid BLE application updated through the 
 *        BLE Over-The-Air (OTA) Service.
 * 
 * <!-- Copyright 2014 by STMicroelectronics.  All rights reserved.       *80*-->

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM.
  -# From the File->Open->Workspace menu, open the IAR workspace
     <tt> ...\\Projects\\Project\\OTA_ResetManager\\EWARM\\ResetManager.eww </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect STLink to JTAG connector  in your board (if available).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG GUI, put the board in DFU mode and download the built binary image.
  -# Connect the application board to a PC USB port. 

* \subsection IAR_project_configurations IAR project configurations

  - \c App  - OTA Reset Manager Application
  
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
| LED name         |STEVAL-IDB002V1         | STEVAL-IDB003V1         |  
-----------------------------------------------------------------------
| D1               | ON during EEPROM Erase |  NA                     |
| D2               | Not used               |  ON during EEPROM Erase |    
| D3               | Not used               |  Not used               |  
| D4               | Not used               |  NA                     |     
| D5               | Not used               |  NA                     |            
| D6               | Not used               |  NA                     |       
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

  - The OTA Reset Manager is a basic application which is is stored at FLASH base + DFU size (0x3000) and 
    it allows to transfer of control towards the new updated application every time we reset.
  - At device reset, the reset manager will take care of jumping to the location of the last image 
    that was successfully loaded by the OTA bootloader, as indicated in the higher 8 bytes
    within the EEPROM memory range of the device (starting at address 0x08080FF8).

**/
/** @addtogroup BlueNRG_demonstrations_applications
 * BlueNRG OTA Reset manager \see OTA_ResetManager_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "SDK_EVAL_Config.h"

typedef  void (*pFunction)(void);
#define NEW_APP_MEM_INFO (uint32_t)0x08080FF8
#define GPIO_HIGH(a,b)          a->BSRRL = b
#define GPIO_LOW(a,b)           a->BSRRH = b
#define GPIO_HIGH_DONGLE(a,b)   a->BSRRH = b
#define GPIO_LOW_DONGLE(a,b)    a->BSRRL = b

#ifndef VECTOR_TABLE_BASE_ADDRESS 
/* default configuration: DFU upgrade is supported */
#define VECTOR_TABLE_BASE_ADDRESS            (0x3000)
#endif

static GPIO_TypeDef* ButtonPort;
static uint16_t ButtonPin;

static GPIO_TypeDef* LedPort;
static uint16_t LedPin;

/**
  * @brief  Set_System
  * @param  None
  * @retval None
  */

void Set_System(void)
{

}

/**
  * @brief  Configures peripherals' clock.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
  
  /* Enable the GPIOs Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC| RCC_AHBPeriph_GPIOD| RCC_AHBPeriph_GPIOE| RCC_AHBPeriph_GPIOH, ENABLE);     
    
  /* Enable SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG , ENABLE);
  
  /* Set low power configuration */
  RCC->AHBLPENR = RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN;   
  RCC->APB1LPENR = RCC_APB1ENR_PWREN|RCC_APB1ENR_TIM2EN;
  RCC->APB2LPENR = 0;  
}


int main(void) {
    pFunction Jump_To_Application;
    uint32_t JumpAddress;
    uint32_t * p = (uint32_t*) NEW_APP_MEM_INFO;
    uint32_t appAddress;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, VECTOR_TABLE_BASE_ADDRESS);

    RCC_Configuration();
    
    /* Identify the BluenRG platform */
    SdkEvalIdentification();
      
#ifdef USER_DEFINED_PLATFORM
    /* Select BUTTON_SEL  push-button */
    ButtonPort = USER_BUTTON1_GPIO_PORT;
    ButtonPin = USER_BUTTON1_GPIO_PIN; 
    LedPort = USER_LED1_GPIO_PORT;
    LedPin = USER_LED1_GPIO_PIN;
#else /* Runtime configuration for BlueNRG Development board and USB Dongle platform */
    /* Select button & led resource */
    if(SdkEvalGetVersion() == SDK_EVAL_VERSION_3) 
    {
      /* Select BUTTON_SEL  push-button */
      ButtonPort = SEL_BUTTON_V3_GPIO_PORT;
      ButtonPin = SEL_BUTTON_V3_PIN; 
      LedPort = SDK_EVAL_V3_LED1_GPIO_PORT;
      LedPin = SDK_EVAL_V3_LED1_PIN;
    }
    else if (SdkEvalGetVersion() == SDK_EVAL_VERSION_D1)
    {
      /* Select  button SW2 for BlueNRG USB Dongle */
      ButtonPort = KEY_BUTTON_VD1_GPIO_PORT;
      ButtonPin = KEY_BUTTON_VD1_PIN;
      LedPort = SDK_DONGLE_V1_LED1_GPIO_PORT;
      LedPin = SDK_DONGLE_V1_LED1_PIN;
    }
#endif /* USER_DEFINED_PLATFORM */
    
    //Joystick button GPIO init
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin =  ButtonPin;
    GPIO_Init(ButtonPort, &GPIO_InitStructure);
    
    //Red led GPIO init
    GPIOD->MODER   = 0xFFFFFFFF;
    GPIO_InitStructure.GPIO_Pin =  LedPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(LedPort, &GPIO_InitStructure);
    if (SdkEvalGetVersion() == SDK_EVAL_VERSION_3) 
      GPIO_HIGH(LedPort,LedPin);
    else if (SdkEvalGetVersion() == SDK_EVAL_VERSION_D1)
      GPIO_HIGH_DONGLE(LedPort,LedPin);
    else
      GPIO_HIGH(LedPort,LedPin);
    /* Check if selected  button was pressed in order to erase OTA-BTL EEPROM locations */
    if (GPIO_ReadInputDataBit(ButtonPort,ButtonPin) == RESET)
    {
      FLASH_Status flashStatus;
   
      DATA_EEPROM_Unlock(); 
      flashStatus = (DATA_EEPROM_EraseWord(NEW_APP_MEM_INFO) | DATA_EEPROM_EraseWord(NEW_APP_MEM_INFO + 4));
      DATA_EEPROM_Lock();
   
      if (flashStatus == FLASH_COMPLETE)
      {
        volatile uint32_t z;
        if (SdkEvalGetVersion() == SDK_EVAL_VERSION_3) 
          GPIO_LOW(LedPort,LedPin);
        else if (SdkEvalGetVersion() == SDK_EVAL_VERSION_D1)
          GPIO_LOW_DONGLE(LedPort,LedPin);
        else
          GPIO_HIGH(LedPort,LedPin);
        //passive delay (for loop to avoid implementing/importing flash consuming functions - 
        //- drawback is that it is clock frequency dependent therefore might need retuning on new platforms)
        for(z=0;z<0xAFFFFF;z++); 
        if (SdkEvalGetVersion() == SDK_EVAL_VERSION_3) 
          GPIO_HIGH(LedPort,LedPin);
        else if (SdkEvalGetVersion() == SDK_EVAL_VERSION_D1)
          GPIO_HIGH_DONGLE(LedPort,LedPin);
        else
          GPIO_HIGH(LedPort,LedPin);
      }
    }
    
    appAddress = *p;
    if (appAddress == 0)
      appAddress = 0x8003800;
    
    /* Jump to user application */
    JumpAddress = *(__IO uint32_t*) (appAddress + 4);
    Jump_To_Application = (pFunction) JumpAddress;
    /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t*) appAddress);
    Jump_To_Application();
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
