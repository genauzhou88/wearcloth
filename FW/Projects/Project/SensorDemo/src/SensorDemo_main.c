/******************** (C) COPYRIGHT 2014 STMicroelectronics ********************
* File Name          : SensorDemo_main.c
* Author             : AMS - AAS division
* Version            : V1.0.1
* Date               : 10-February-2014
* Description        : BlueNRG Sensor Demo main file with no OTA
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/**
 * @file  SensorDemo_main.c
 * @brief This application contains an example which shows how implementing a proprietary
 * Bluetooth Low Energy profile: the sensor profile.
 *
 * <!-- Copyright 2014 by STMicroelectronics.  All rights reserved.       *80*-->

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM.
  -# From the File->Open->Workspace menu, open the IAR workspace
     <tt> ...\\Projects\\Project\\SensorDemo\\EWARM\\SensorDemo.eww </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect STLink to JTAG connector  in your board (if available).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG GUI, put the board in DFU mode and download the built binary image.
  -# Connect the application board to a PC USB port. Open a hyperterminal on the
     corresponding USB virtual COMx port with the configuration as described in \ref Serial_IO.

* \subsection IAR_project_configurations IAR project configurations

  - \c USB - USB Virtual COM configuration x debug messages
  - \c LowPower - Low power configuration

* \section Prebuilt_images Prebuilt images
  - SensorDemo.hex 
 
* \section Jumper_settings Jumper settings
@table
------------------------------------------------------
| Jumper name       |  Description                   | 
------------------------------------------------------
| JP1, if available | USB or Battery supply position | 

@endtable 


* \section Board_supported Board supported
The SensorDemo application is not supported on the STEVAL-IDB003V1 (USB Dongle).
If loaded on the USB Dongle, the LED D2 is blinking to indicate that this application 
is not supported on such platform.
@table
| Board name      | Board revision | NOTE        |
--------------------------------------------------
| STEVAL-IDB002V1 | 3.0            | Eval Kit    |
@endtable

* \section Serial_IO Serial I/O
  Serial I/O is only supported with USB IAR project configurations (only for debugging purposes)
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
| LED name         | LowPower          | USB               |
------------------------------------------------------------
| D1               | Not used          |  Not used         |  
| D2               | Blinking if error | Blinking if error |   
| D3               | Not used          |  Not used         |  
| D4               | Not used          |  Not used         |   
| D5               | Not used          |  Not used         |           
| D6               | Not used          |  Not used         |      
@endtable

* \section Buttons_description Buttons description
@table
| Button name          | Event                                                  | 
---------------------------------------------------------------------------------
| RESET                | Reset board                                            |       
| Push Button          | On RESET it allows to activate DFU                     |                   
@endtable


* \section DFU_Activation  DFU activation
BlueNRG boards are preprogrammed with a DFU application which allows to upload the 
STM32L micro with a selected binary image through USB. Follow list of actions 
for activating DFU on STEVAL-IDB002V1 board
@table
| Board  name          | Event                                             | Note               |
------------------------------------------------------------------------------------------------
| STEVAL-IDB002V1      | Press RESET and Push Button. Release RESET button | LED D2 is toggling |  
@endtable

* \section Usage Usage

This profile exposes two services: 
 - acceleration service 
 - environmental service.

Acceleration service exposes these characteristics: 
 - free-fall characteristic (read & notify properties ). 
    The application will send a notification on this characteristic if a free-fall condition has been
detected by the LIS3DH MEMS sensor (the condition is detected if the acceleration on the 3
axes is near zero for a certain amount of time).
 - acceleration characteristic  measured by the accelerometer (read & notify properties). The value is made up of six bytes. Each couple of
bytes contains the acceleration on one of the 3 axes. The values are given in mg.

Environmental service exposes these characteristics: 
-  temperature, pressure and humidity chracteristics (read property). For each
characteristic, a characteristic format descriptor is present to describe the type of data
contained inside the characteristic. 

Note: An expansion board with LPS25H pressure sensor and HTS221 humidity sensor can be connected to the
motherboard through the expansion connector  If the expansion board is not detected, only
temperature from STLM75 (on BlueNRG motherboard) will be used.

**/
/** @addtogroup BlueNRG_demonstrations_applications
 * BlueNRG SensorDemo \see SensorDemo_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "usb_lib.h"
#include "hw_config.h"
#include "hal_types.h"
#include "hci.h"
#include "bluenrg_hci.h"
#include "gp_timer.h"
#include "hal.h"
#include "osal.h"
#include "gatt_db.h"
#include "gatt_server.h"
#include "hci_internal.h"
#include "bluenrg_hci_internal.h"
#include "gap.h"
#include "sm.h"
#include <stdio.h>
#include <lis3dh_driver.h>
#include <stlm75.h>
#include <HTS221.h>
#include <LPS25H.h>
#include <platform_config.h>
#include <string.h>
#include "low_power.h"
#include "SDK_EVAL_Config.h"
#include <bluenrg_utils.h>

/* Include the BlueNRG firmware image in case of update. */
//#include "bluenrg_6_3_Mode_2-16MHz.c"

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifndef VECTOR_TABLE_BASE_ADDRESS /* Default configuration: no OTA bootloader */
/* default configuration: DFU upgrade is supported */
#define VECTOR_TABLE_BASE_ADDRESS            (0x3000)
#endif


volatile uint8_t set_connectable = 1;
tHalUint16 connection_handle = 0;

bool sensor_board = FALSE; // It is True if sensor boad has been detected

#define UPDATE_CONN_PARAM 0

/*******************************************************************************
* Function Name  : Init_Accelerometer.
* Description    : Init LIS331DLH accelerometer.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Init_Accelerometer(void)
{
    uint8_t reg;
      
    LIS3DH_SpiInit();
    
    LIS3DH_GetWHO_AM_I(&reg);        
    PRINTF("LIS_WHO_AM_I_ADDR: %02X\n", reg);    
    //Initialize MEMS Sensor
    //set ODR (turn ON device)
     LIS3DH_SetODR(LIS3DH_ODR_100Hz);
    //set PowerMode 
     LIS3DH_SetMode(LIS3DH_NORMAL);
    //set Fullscale
    LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2);
    //set axis Enable
    LIS3DH_SetAxis(LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE |  LIS3DH_Z_ENABLE);    
    // Enable Free Fall detection
    //LIS3DH_SetInt1Pin(LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE);
    LIS3DH_SetInt2Pin(LIS3DH_I2_INT1_ON_PIN_INT2_ENABLE);
    LIS3DH_Int1LatchEnable(MEMS_ENABLE);
    LIS3DH_SetInt1Threshold(16);    
    LIS3DH_SetInt1Duration(5);    
    LIS3DH_SetIntConfiguration(LIS3DH_INT1_AND|LIS3DH_INT1_ZLIE_ENABLE|
                               LIS3DH_INT1_YLIE_ENABLE|LIS3DH_INT1_XLIE_ENABLE);    
    LIS3DH_Interrupts_Config();    
    LIS3DH_ResetInt1Latch();
}

/*******************************************************************************
* Function Name  : Init_Temperature_Sensor.
* Description    : Init STLM75 temperature sensor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Init_Temperature_Sensor(void)
{
    //uint8_t reg;
    //int ret;
    signed short data;
      
    STLM75_I2C_Init();
    
    STLM75_Read_Temperature_Signed(&data);
}

/*******************************************************************************
* Function Name  : Init_Humidity_Sensor.
* Description    : Init HTS221 temperature sensor.
* Input          : None.
* Output         : None.
* Return         : 0 if success, 1 if error.
*******************************************************************************/
int Init_Humidity_Sensor(void)
{
    uint8_t reg = 0;
    //int ret;
      
    HTS221_I2C_Init();
    
    HTS221_Power_On();
    
    HTS221_Read(&reg, HTS221_WHO_AM_I, 1);
    
    PRINTF("HTS221_WHO_AM_I: %02X\n", reg);
    
    if(reg != 0xBC)
        return 1;
    
    return 0;
}

/*******************************************************************************
* Function Name  : Init_Pressure_Sensor.
* Description    : Init LPS25H pressure sensor.
* Input          : None.
* Output         : None.
* Return         : 0 if success, 1 if error.
*******************************************************************************/
int Init_Pressure_Sensor(void)
{
    uint8_t reg = 0;
    //int ret;
    LPS25HInit pxLPS25HInitStruct;
    
    pxLPS25HInitStruct.xOutputDataRate = ODR_P_25HZ_T_25HZ;
    pxLPS25HInitStruct.xPresRes = LPS_PRESS_AVG_512;
    pxLPS25HInitStruct.xTempRes = LPS_TEMP_AVG_64;
    pxLPS25HInitStruct.xPressureAutoZero = LPS_DISABLE;
    pxLPS25HInitStruct.fPressureRef = 0;
    pxLPS25HInitStruct.xBDU = LPS_DISABLE;
    
      
    LPS25H_I2C_Init();
    
    LPS25H_Read(&reg, LPS_WHO_AM_I_ADDR, 1);
    
    PRINTF("LPS_WHO_AM_I_ADDR: %02X\n", reg);
    
    if(reg != 0xBD)
        return 1;
    
    Lps25hConfig(&pxLPS25HInitStruct);
    
    reg = 0x40;
    Lps25hByteWrite(&reg, LPS_CTRL_REG2_ADDR);
    
    reg = 0xDF;  // FIFO_MEAN MODE, 32 samples moving average
    Lps25hByteWrite(&reg, LPS_CTRL_FIFO_ADDR);    
    
    return 0;
}

/*  User Function where serial received data should be processed */
void processInputData(uint8_t * rx_data, uint16_t data_size)
{
  /* No data processing is required from this application */
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

#if UPDATE_CONN_PARAM
int l2cap_request_sent = FALSE;
struct timer l2cap_req_timer;
#endif

int main(void)
{
    int ret;
    
    NVIC_SetVectorTable(NVIC_VectTab_FLASH,VECTOR_TABLE_BASE_ADDRESS);
    
    /* Identify the BlueNRG platform */
    SdkEvalIdentification();
    
    RCC_Configuration();
    
    /* Init I/O ports */
    Init_GPIOs ();
    
    Clock_Init();
    
    /* This application is supported only on BlueNRG Development Kit : STEVAL-IDB002V1.
       If a loaded on a different platform, just toggle led to notify this error */
    if (SdkEvalGetVersion() != SDK_EVAL_VERSION_3)
    {
      /* LED1 initialization */
      SdkEvalLedInit(LED1);
      
      /* Main loop */
      while (1)
      {
        /* LED1 toggling */
        SdkEvalLedToggle(LED1);
    
        /* pause */
        Clock_Wait(500);
      }
    }
    
    PWR_PVDCmd(DISABLE);
    
    /* Disable FLASH during Sleep  */
    FLASH_SLEEPPowerDownCmd(ENABLE);
    
    /* Enable Ultra low power mode */
    PWR_UltraLowPowerCmd(ENABLE);
    
    PWR_FastWakeUpCmd(DISABLE);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    
    //PowerOff(); //eventually disable USB cable before re-starting USB
    
    //Clock_Init();
    
    /* Delay needed only to be able to acces the JTAG interface after reset
    if it will be disabled later. */
    Clock_Wait(500);
   
    /* Configure I/O communication channel:
       It requires the void IO_Receive_Data(uint8_t * rx_data, uint16_t data_size) function
       where user received data should be processed */
    SdkEval_IO_Config(processInputData);
    
    /* Delay for debug purpose, in order to see printed data at startup. */
    Clock_Wait(2000);
    
    HCI_Init();
 
    /* Init SPI interface */
    SdkEvalSpiInit(SPI_MODE_EXTI);
    /* Reset BlueNRG SPI interface */
    BlueNRG_RST();
#if 0 //TBR    
    //program_device(fw_image, sizeof(fw_image),FALSE);
    ret = verify_IFR(&IFR_config);
    if (ret) {
      ret = program_IFR(&IFR_config);
      if (ret)
	while(1);
    }
#endif    
    
    /* The 2 LED1, LED2 leds are used for debugging purposes */
    SdkEvalLedInit(LED1);
    SdkEvalLedInit(LED2);
    SdkEvalLedInit(LED3);
    SdkEvalLedInit(LED4);
    SdkEvalLedInit(LED5);
    {
        tHalUint8 bdaddr[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02};

        ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN,
                                        bdaddr);
        if(ret){
            PRINTF("Setting BD_ADDR failed.\n");
        }
    }
    
    ret = aci_gatt_init();    
    if(ret){
        PRINTF("GATT_Init failed.\n");
    }
    
    {
        uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
        ret = aci_gap_init(1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
        if(ret){
            PRINTF("GAP_Init failed.\n");
        }
        
        const char *name = "BlueNRG";
        
        ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, strlen(name), (tHalUint8 *)name);
        
        if(ret){
            PRINTF("aci_gatt_update_char_value failed.\n");            
            SdkEvalLedOn(LED2);
            while(1);
        }
    }
    
    ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                       OOB_AUTH_DATA_ABSENT,
                                       NULL,
                                       7,
                                       16,
                                       USE_FIXED_PIN_FOR_PAIRING,
                                       123456,
                                       BONDING);
    
    if(ret){
            SdkEvalLedOn(LED2);
        while(1);
    }
    
    PRINTF("BLE Stack Initialized.\n");
    
    Init_Accelerometer();
    
#ifndef SENSOR_EMULATION
    if(Init_Pressure_Sensor() != 0 || Init_Humidity_Sensor()!=0){
        // Error reading from pressure sensor. Sensor missing?
        sensor_board = FALSE;
        HTS221_I2C_DeInit_GPIO();
        Init_Temperature_Sensor();
    }
    else {
        sensor_board = TRUE;
    }
        
#endif
    
    PRINTF("Sensors initialized.\n");
    
    ret = Add_Acc_Service();
    
    if(ret == BLE_STATUS_SUCCESS)
        PRINTF("Acc service added successfully.\n");
    else
        PRINTF("Error while adding Acc service.\n");

    ret = Add_Environmental_Sensor_Service();
    
    
    Init_User_Timer();
    Start_User_Timer();
    
    /* -2 dBm output power */
    ret = aci_hal_set_tx_power_level(1,4);
    
    if(ret != 0)
      SdkEvalLedOn(LED2);
        
    while(1)
    {
      HCI_Process();
      
      if(set_connectable){
          setConnectable();
          set_connectable = 0;
      }
#if UPDATE_CONN_PARAM      
      if(connected && !l2cap_request_sent && Timer_Expired(&l2cap_req_timer)){
        ret = aci_l2cap_connection_parameter_update_request(connection_handle, 9, 20, 0, 600);
        l2cap_request_sent = TRUE;
      }
#endif
      
      if(user_timer_expired){
        user_timer_expired = FALSE;
        if(connected){
           AxesRaw_t data;
           status_t response;

          //get Acceleration data
          response = LIS3DH_GetAccAxesRaw(&data);
          if(response){
              LIS3DH_ConvAccValue(&data);
              //PRINTF("ACC: X=%6d Y=%6d Z=%6d\r\n", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
              Acc_Update(&data);
          }
        }
      }
      
      if(request_free_fall_notify == TRUE){
        request_free_fall_notify = FALSE;
        
        Free_Fall_Notify();
        
      }
#if ENABLE_MICRO_SLEEP
      Enter_LP_Sleep_Mode();
#endif
    }/* while (1) */
}

/*
 * setConnectable() function puts the device in connectable mode.
 * If you want to specify a UUID list in the advertising data, those data can
 * be specified as a parameter in aci_gap_set_discoverable().
 * For manufacture data, aci_gap_update_adv_data must be called.
 * Ex.:
 *
 *  tBleStatus ret;    
 *  const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G'};    
 *  const uint8_t serviceUUIDList[] = {AD_TYPE_16_BIT_SERV_UUID,0x34,0x12};    
 *  const uint8_t manuf_data[] = {4, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0x05, 0x02, 0x01};
 *  
 *  ret = aci_gap_set_discoverable(ADV_IND, 0, 0, RANDOM_ADDR, NO_WHITE_LIST_USE,
 *                                 8, local_name, 3, serviceUUIDList, 0, 0);    
 *  ret = aci_gap_update_adv_data(5, manuf_data);
 *
 */


void setConnectable(void)
{  
    tBleStatus ret;

    const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G'};

    /* disable scan response */
    hci_le_set_scan_resp_data(0,NULL);
    PRINTF("General Discoverable Mode.\n");
    
    ret = aci_gap_set_discoverable(ADV_IND, 0, 0, RANDOM_ADDR, NO_WHITE_LIST_USE,
                                   8, local_name, 0, NULL, 0, 0);    
    if(ret != 0)
       SdkEvalLedOn(LED2);
    PRINTF("%d\n",ret);
}

void GAP_ConnectionComplete_CB(tHalUint8 addr[6], tHalUint16 handle){
    
    connected = TRUE;
    connection_handle = handle;
    
    PRINTF("Connected to device:");
    for(int i = 5; i > 0; i--){
        PRINTF("%02X-", addr[i]);
    }
    PRINTF("%02X\r\n", addr[0]);
    

#if UPDATE_CONN_PARAM    
    l2cap_request_sent = FALSE;
    Timer_Set(&l2cap_req_timer, CLOCK_SECOND*2);
#endif
}

void GAP_DisconnectionComplete_CB(void){
    connected = FALSE;
    PRINTF("Disconnected\n");
    /* Make the device connectable again. */
    set_connectable = TRUE;
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
            //evt_disconn_complete *evt = (void *)event_pckt->data;
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
                
            case EVT_BLUE_GATT_READ_PERMIT_REQ:
                {
                    evt_gatt_read_permit_req *pr = (void*)blue_evt->data;                    
                    Read_Request_CB(pr->attr_handle);                    
                }
                break;

            case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
                {  
                }
                break;
#if UPDATE_CONN_PARAM                
            case EVT_BLUE_L2CAP_CONN_UPD_RESP:
                {
                    evt_l2cap_conn_upd_resp *resp = (void*)blue_evt->data;
                    //uint16_t a = resp->result;
                }
                break;
#endif
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
