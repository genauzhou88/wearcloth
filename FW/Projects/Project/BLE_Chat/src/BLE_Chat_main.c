/******************** (C) COPYRIGHT 2014 STMicroelectronics ********************
* File Name          : BLE_Chat_main.c
* Author             : AMS - AAS Division
* Version            : V1.0.1
* Date               : 10-February-2014
* Description        : BlueNRG main file for Chat demo
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/**
 * @file  BLE_Chat_main.c
 * @brief This is a Chat demo that shows how to implement a simple 2-way communication between two BlueNRG devices.
 *
 * <!-- Copyright 2014 by STMicroelectronics.  All rights reserved.       *80*-->

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM.
  -# From the File->Open->Workspace menu, open the IAR workspace
     <tt> ...\\Projects\\Project\\BLE_Chat\\EWARM\\BLE_Chat.eww </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect STLink to JTAG connector  in your board (if available).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG GUI, put the board in DFU mode and download the built binary image.
  -# Connect the application board to a PC USB port. Open a hyperterminal on the
     corresponding USB virtual COMx port with the configuration as described in \ref Serial_IO.

* \subsection IAR_project_configurations IAR project configurations

  - \c Client - Client role configuration 
  - \c Server - Server role configuration

* \section Prebuilt_images Prebuilt images
  - BLE_Chat_Client.hex, BLE_Chat_Server.hex 
 
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
  The application will listen for keys typed in one node and, on return press, it will send them to the remote node.
  The remote node will listen for RF messages and it will output them in the serial port.
  In other words everything typed in one node will be visible to the other node and viceversa.
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
|                  | STEVAL-IDB002V1 | STEVAL-IDB003V1 |                 
| LED name         | Client/Server   | Client/Server   | 
--------------------------------------------------------
| D1               | Not used        | NA              |        
| D2               | Not used        | Not used        |     
| D3               | Not used        | Not used        |     
| D4               | Not used        | NA              |      
| D5               | Not used        | NA              |       
| D6               | Not used        | NA              |       
@endtable
 - NA : Not Applicable;

* \section Buttons_description Buttons description
@table
|                  | STEVAL-IDB002V1 | STEVAL-IDB003V1 |                 
| Button name      | Client/Server   | Client/Server   | 
--------------------------------------------------------
| RESET            | X               | NA              |  
| Push Button      | Not used        | NA              |   
| Jostick Sel      | Not used        | NA              |    
| SW1              | NA              | Not used        |     
| SW2              | NA              | Not used        |       
@endtable
 - NA : Not Applicable;

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

This Chat demo has  are 2 roles:
 - The server that expose the Chat service. It is the slave.
 - The client that uses the Chat service. It is the master.

The Chat Service contains 2 Characteristics:
 -# The TX Characteristic: the client can enable notifications on this characteristic. When the server has data to be sent, it will send notifications which will contains the value of the TX Characteristic
 -# The RX Characteristic: it is a writable caracteristic. When the client has data to be sent to the server, it will write a value into this characteristic.

The maximum length of the characteristic value is 20 bytes.

NOTES:
 - The Client and Server workspaces can be configured in order to target a throughput test. Jut add the THROUGHPUT_TEST define on the preprocessor options on both workspaces.
 - Program the client on one BlueNRG platform and reset it. The platform will be seen on the PC as a virtual COM port. Open the port in a serial terminal emulator. Client will start 4 seconds after reset.
 - Program the server on a second BlueNRG platform and reset it. The two platforms will try to establish a connection. As soon as they get connected, the slave will 
   send notification to the client of 20 bytes (200 notifications). 
 - After all the notifications are received, the measured application throughput will be displayed.

**/
/** @addtogroup BlueNRG_demonstrations_applications
 * BlueNRG Chat demo \see BLE_Chat_main.c for documentation.
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
#include "app_state.h"
#include <stdio.h>

#include <lis3dh_driver.h>

#include "rtctime.h"

#include "SDK_EVAL_Config.h"
#include <bluenrg_utils.h>
/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** 
  * @brief  Enable febug printf's
  */ 
#ifndef DEBUG
#define DEBUG 1
#endif
      
#define REQUEST_CONN_PARAM_UPDATE 0

/* Private macros ------------------------------------------------------------*/
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define NUM_PACKETS 200 // Only used for throughput test (define THROUGHPUT_TEST)

#ifndef VECTOR_TABLE_BASE_ADDRESS 
/* default configuration: DFU upgrade is supported */
#define VECTOR_TABLE_BASE_ADDRESS            (0x3000)
#endif

/* Private variables ---------------------------------------------------------*/
volatile int app_flags = SET_CONNECTABLE;
volatile int app2_flags = SET_CONNECTABLE; //maintain two flags for each server
volatile uint16_t connection_handle = 0;
volatile uint16_t connection2_handle = 0; //maintain two handle for different server
extern tHalUint16 chatServHandle, TXCharHandle, RXCharHandle;
extern tHalUint16 chatServHandle2, TXCharHandle2, RXCharHandle2; //maintain two handle
struct timer l2cap_req_timer;

int connection_num = 0;
int bServerConnected = 0;

/* Private variables ---------------------------------------------------------*/
extern RTC_TimeTypeDef RTC_TimeStructure;
extern RTC_InitTypeDef RTC_InitStructure;

extern __IO uint32_t AsynchPrediv, SynchPrediv;

/** 
  * @brief  Handle of TX,RX  Characteristics.
  */ 
#ifdef CLIENT
uint16_t tx_handle;
uint16_t rx_handle;

uint16_t tx_handle2; //handle for SERVER_2
uint16_t rx_handle2; //handle for SERVER_2
#endif 


/* Private function prototypes -----------------------------------------------*/
void Make_Connection(void);
void User_Process(void);
void User_Process2(void);
/* Private functions ---------------------------------------------------------*/

/*  User Function where serial received data should be processed */
void processInputData(uint8_t * rx_data, uint16_t data_size);


/* Extra function for Accel */
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
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
    int ret;
    
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
    
    /* Delay needed only to be able to acces the JTAG interface after reset
    if it will be disabled later. */
    Clock_Wait(500);    
    
    /* Configure I/O communication channel:
       It requires the void IO_Receive_Data(uint8_t * rx_data, uint16_t data_size) function
       where user received data should be processed */
    SdkEval_IO_Config(processInputData);
    
    /* Delay for debug purpose, in order to see printed data at startup. */
    for(int i = 0; i < 100 ; i++){
        printf(".");
        Clock_Wait(40);
    }

    HCI_Init();
    
    /* Check RTC function */
    //Add by Fred 
    if (RTC_ReadBackupRegister(RTC_BKP_DR0) != 0x32F2)
    {  
      /* RTC configuration  */
      printf("No backup register found, re-config RTC!\n");
      RTC_Config();

      /* Configure the RTC data register and RTC prescaler */
      RTC_InitStructure.RTC_AsynchPrediv = AsynchPrediv;
      RTC_InitStructure.RTC_SynchPrediv = SynchPrediv;
      RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
     
      /* Check on RTC init */
      if (RTC_Init(&RTC_InitStructure) == ERROR)
      {
        printf("\n\r        /!\\***** RTC Prescaler Config failed ********/!\\ \n\r");
      }

      /* Configure the time register */
      RTC_TimeRegulate(); 
    }
    else
    {
      /* Check if the Power On Reset flag is set */
      if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
      {
        printf("\r\n Power On Reset occurred....\n\r");
      }
      /* Check if the Pin Reset flag is set */
      else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
      {
        printf("\r\n External Reset occurred....\n\r");
      }

      printf("\n\r No need to configure RTC....\n\r");
      
      /* Enable the PWR clock */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

      /* Allow access to RTC */
      PWR_RTCAccessCmd(ENABLE);

      /* Wait for RTC APB registers synchronisation */
      RTC_WaitForSynchro();

      /* Clear the RTC Alarm Flag */
      RTC_ClearFlag(RTC_FLAG_ALRAF);

      /* Clear the EXTI Line 17 Pending bit (Connected internally to RTC Alarm) */
      EXTI_ClearITPendingBit(EXTI_Line17);

      /* Display the RTC Time and Alarm */
      RTC_TimeShow();
    }
    
    /* Init SPI interface */
    SdkEvalSpiInit(SPI_MODE_EXTI);
    BlueNRG_RST(); 
    
    //check para
#if CLIENT
    //Write Mode 3 to make sure we can connect multiple clients
    tHalUint8 newmode = 0x03;
    ret = aci_hal_write_config_data(0x2D,0x01,&newmode); //Directly from ST support
    
    if(ret){
      PRINTF("Setting Mode 3 failed! \n");
    }
#endif
    
    {
#if CLIENT
        tHalUint8 bdaddr[] = {0xbb, 0x00, 0x00, 0xE1, 0x80, 0x02}; //client bd address
#elif SERVER 
        tHalUint8 bdaddr[] = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02}; //server1 bd address
#else
        tHalUint8 bdaddr[] = {0xcc, 0x00, 0x00, 0xE1, 0x80, 0x02}; //server2 bd address
#endif
        
        
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
#if CLIENT //centrol role for client
        ret = aci_gap_init(GAP_CENTRAL_ROLE, &service_handle, &dev_name_char_handle, &appearance_char_handle);
#else //peripheral role for server and server_2
        ret = aci_gap_init(GAP_PERIPHERAL_ROLE, &service_handle, &dev_name_char_handle, &appearance_char_handle);        
#endif
        if(ret){
            PRINTF("GAP_Init failed.\n");
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
    PRINTF("BLE Stack Initialized.\n");
    
    Init_Accelerometer();
    
    PRINTF("Accelerometer Initialized.\n");
    
#if CLIENT
    PRINTF("CLIENT: BLE Stack Initialized  (platform: %d)\n", SdkEvalGetVersion());
#elif SERVER
    PRINTF("SERVER: BLE Stack Initialized (platform: %d)\n", SdkEvalGetVersion());
    ret = Add_Chat_Service();
#else
    PRINTF("SERVER2: BLE Stack Initialized  (platform: %d)\n", SdkEvalGetVersion());
    ret = Add_Chat_Service();
    
    if(ret == BLE_STATUS_SUCCESS)
        PRINTF("Service added successfully.\n");
    else
        PRINTF("Error while adding service.\n");
#endif 
    

    
    /* -2 dBm output power */
    ret = aci_hal_set_tx_power_level(1,4);
        
    while(1)
    { 
        HCI_Process();
        User_Process();
#if TWO_NODES //only try to connect 2nd nodes when user want to
        if (bServerConnected) //only make connection when Server1 is connectted
        {
          //PRINTF("Start to connect with Server_2\n");
          User_Process2();
        }
#endif
    }
    }

/**
  * @brief  Make the device connectable
  * @param  None 
  * @retval None
  */
void Make_anotherConnection(void)
{  
    tBleStatus ret;
    
    
#if CLIENT
    //wait for another 5 second before connection
    //Clock_Wait(5000);
    tBDAddr mbdaddr = {0xcc, 0x00, 0x00, 0xE1, 0x80, 0x02};
    //connect with SERVER_2
     ret = aci_gap_create_connection(0x4000, 0x4000, PUBLIC_ADDR, mbdaddr, PUBLIC_ADDR, 16, 16, 0, 0x03E8, 0 , 0x3E80); 
    //ret = aci_gap_create_connection(0x1000, 0x1000, PUBLIC_ADDR, bdaddr, PUBLIC_ADDR, 8, 8, 0, 60, 2000 , 2000);
    
    if (ret != 0){
        PRINTF("Error while starting connection with multi-node. return code is %d\n",ret);
        Clock_Wait(100);        
	}
#else
    //set a slightly different local name for SERVER_2
    const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G','_','C','h','a','t'};
    
    /* disable scan response */
    hci_le_set_scan_resp_data(0,NULL);
    
    PRINTF("General Discoverable Mode ");
    ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                   13, local_name, 0, NULL, 0, 0);

    PRINTF("%d\n",ret);
#endif
}
void Make_Connection(void)
{  
    tBleStatus ret;
    
    
#if CLIENT
    
    tBDAddr bdaddr = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02};
    //connect with SERVER_1
    ret = aci_gap_create_connection(0x4000, 0x4000, PUBLIC_ADDR, bdaddr, PUBLIC_ADDR, 16, 16, 0, 0x03E8, 0 , 0x3E80); 
    //ret = aci_gap_create_connection(0x1000, 0x1000, PUBLIC_ADDR, bdaddr, PUBLIC_ADDR, 8, 8, 0, 60, 2000 , 2000);
    
    if (ret != 0){
        PRINTF("Error while starting connection.\n");
        Clock_Wait(100);        
	}
    
#else
    
    const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G','_','C','h','a','t'};
    
    /* disable scan response */
    hci_le_set_scan_resp_data(0,NULL);
    
    PRINTF("General Discoverable Mode ");
    ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                   13, local_name, 0, NULL, 0, 0);

    PRINTF("%d\n",ret);
#endif
}

/**
  * @brief  This function is called when there is a LE Connection Complete event.
  * @param  addr Address of peer device
  * @param  handle Connection handle
  * @retval None
  */
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{    
    if (addr[0] == 0xaa) //server is connectted
    {
      connection_handle = handle;
      connection_num++;
      APP_FLAG_SET(CONNECTED); 
      bServerConnected = 1;
    }
    else if (addr[0] == 0xcc) //server_2 is connectted
    {
      connection2_handle = handle;
      connection_num++;
      APP_FLAG_SET2(CONNECTED); 
    }
    
    
    PRINTF("Connected to device:");
    for(int i = 5; i > 0; i--){
        PRINTF("%02X-", addr[i]);
    }
    PRINTF("\ntotoal connection %d, addr: %02X\n", connection_num, addr[0]);
    
      
#if REQUEST_CONN_PARAM_UPDATE
    APP_FLAG_CLEAR(L2CAP_PARAM_UPD_SENT);
    Timer_Set(&l2cap_req_timer, CLOCK_SECOND*2);
#endif
    
}

/**
  * @brief  This function is called when the peer device get disconnected.
  * @param  None 
  * @retval None
  */
void GAP_DisconnectionComplete_CB(void)
{
    connection_num--;
    bServerConnected = 0;
    APP_FLAG_CLEAR(CONNECTED);
    APP_FLAG_CLEAR2(CONNECTED);
    PRINTF("Disconnected\n");
    /* Make the device connectable again. */
    APP_FLAG_SET(SET_CONNECTABLE);
    APP_FLAG_SET2(SET_CONNECTABLE);
    APP_FLAG_CLEAR(NOTIFICATIONS_ENABLED);
    APP_FLAG_CLEAR2(NOTIFICATIONS_ENABLED);
    
    APP_FLAG_CLEAR(START_READ_TX_CHAR_HANDLE);
    APP_FLAG_CLEAR(END_READ_TX_CHAR_HANDLE);
    APP_FLAG_CLEAR(START_READ_RX_CHAR_HANDLE); 
    APP_FLAG_CLEAR(END_READ_RX_CHAR_HANDLE); 
    
    APP_FLAG_CLEAR2(START_READ_TX_CHAR_HANDLE);
    APP_FLAG_CLEAR2(END_READ_TX_CHAR_HANDLE);
    APP_FLAG_CLEAR2(START_READ_RX_CHAR_HANDLE); 
    APP_FLAG_CLEAR2(END_READ_RX_CHAR_HANDLE); 
}

/**
  * @brief  This function is called when there is a notification from the sever.
  * @param  attr_handle Handle of the attribute
  * @param  attr_len    Length of attribute value in the notification
  * @param  attr_value  Attribute value in the notification
  * @retval None
  */
void GATT_Notification_CB(uint16_t attr_handle, uint8_t attr_len, uint8_t *attr_value)
{
#if THROUGHPUT_TEST && CLIENT
    static tClockTime time, time2;
    static int packets=0;     
    
    if(attr_handle == tx_handle+1){ 
        if(packets==0){
            printf("Test start\n");
            time = Clock_Time();
        }
        
        for(int i = 0; i < attr_len; i++)
            printf("%c", attr_value[i]);
        
        packets++;
        
        if(packets == NUM_PACKETS){
            time2 = Clock_Time();
            tClockTime diff = time2-time;
            printf("\n%d packets. Elapsed time: %d ms. App throughput: %.2f kbps.\n", NUM_PACKETS, diff, (float)NUM_PACKETS*20*8/diff);
        }        
        
    }
#elif CLIENT
    //PRINTF("Receiev sth from Server, display it on Client!!\n");
    RTC_TimeShow();
    if(attr_handle == tx_handle+1 || attr_handle == tx_handle2+1){ //display content from both server
      for(int i = 0; i < attr_len; i++)
          printf("%c", attr_value[i]);
    }
        
#endif
}

void User_Process2(void)
{
    if(APP_FLAG2(SET_CONNECTABLE)){
        Make_anotherConnection();
        APP_FLAG_CLEAR2(SET_CONNECTABLE);
    }

#if REQUEST_CONN_PARAM_UPDATE    
    if(APP_FLAG2(CONNECTED) && !APP_FLAG2(L2CAP_PARAM_UPD_SENT) && Timer_Expired2(&l2cap_req_timer)){
        aci_l2cap_connection_parameter_update_request(connection2_handle, 8, 16, 0, 600);
        APP_FLAG_SET2(L2CAP_PARAM_UPD_SENT);
    }
#endif
    
#if CLIENT
    
    /* Start TX handle Characteristic discovery if not yet done */
    if (APP_FLAG2(CONNECTED) && !APP_FLAG2(END_READ_TX_CHAR_HANDLE))
    {
      //PRINTF("connection2 is ok, wait for TX\n");
      if (!APP_FLAG2(START_READ_TX_CHAR_HANDLE))
      {
        /* Discovery TX characteristic handle by UUID 128 bits */
         PRINTF("Ready to expose TX2!\n");
         const tHalUint8 charUuid128_TX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
         
         aci_gatt_discovery_characteristic_by_uuid(connection2_handle, 0x0001, 0xFFFF,UUID_TYPE_128,
                                                   charUuid128_TX);
         APP_FLAG_SET2(START_READ_TX_CHAR_HANDLE);
      }
    }
    /* Start RX handle Characteristic discovery if not yet done */
    else if (APP_FLAG2(CONNECTED) && !APP_FLAG2(END_READ_RX_CHAR_HANDLE))
    {
      /* Discovery RX characteristic handle by UUID 128 bits */
      if (!APP_FLAG2(START_READ_RX_CHAR_HANDLE))
      {
        /* Discovery TX characteristic handle by UUID 128 bits */
        
         const tHalUint8 charUuid128_RX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};
         aci_gatt_discovery_characteristic_by_uuid(connection2_handle, 0x0001, 0xFFFF,UUID_TYPE_128,
                                                   charUuid128_RX);
         APP_FLAG_SET2(START_READ_RX_CHAR_HANDLE);
       }
    }
    
    if(APP_FLAG2(CONNECTED) && APP_FLAG2(END_READ_TX_CHAR_HANDLE) && APP_FLAG2(END_READ_RX_CHAR_HANDLE) && !APP_FLAG2(NOTIFICATIONS_ENABLED)){
        uint8_t client_char_conf_data[] = {0x01, 0x00}; // Enable notifications
        struct timer t;
        Timer_Set(&t, CLOCK_SECOND*10);
        
        while(aci_gatt_write_charac_descriptor(connection2_handle, tx_handle2+2, 2, client_char_conf_data)==BLE_STATUS_NOT_ALLOWED){ //TX_HANDLE2;
            // Radio is busy.
            if(Timer_Expired(&t)) break;
        }
        APP_FLAG_SET2(NOTIFICATIONS_ENABLED);
      }
     

#endif  
    
#if ACCEL_TEST
    //if(APP_FLAG2(CONNECTED) && APP_FLAG2(NOTIFICATIONS_ENABLED))
    {
      AxesRaw_t data;
      status_t response;
      tHalUint8 accdata[20] = {0};
      //get Acceleration data
      response = LIS3DH_GetAccAxesRaw(&data);
      if(response){
          LIS3DH_ConvAccValue(&data);
          sprintf(accdata,"2 %5d %5d %5d\n",data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
          //PRINTF(accdata);
          struct timer t;
          Timer_Set(&t, CLOCK_SECOND*10);
                              
          while(aci_gatt_update_char_value(chatServHandle2, TXCharHandle2, 0, 20, accdata)==BLE_STATUS_INSUFFICIENT_RESOURCES)
          {
            // Radio is busy (buffer full).
            if(Timer_Expired(&t))
                break;
          }
          Clock_Wait(200);
      }
    }
#endif
    
   
}

void User_Process(void)
{
    if(APP_FLAG(SET_CONNECTABLE)){
        Make_Connection();
        APP_FLAG_CLEAR(SET_CONNECTABLE);
    }

#if REQUEST_CONN_PARAM_UPDATE    
    if(APP_FLAG(CONNECTED) && !APP_FLAG(L2CAP_PARAM_UPD_SENT) && Timer_Expired(&l2cap_req_timer)){
        aci_l2cap_connection_parameter_update_request(connection_handle, 8, 16, 0, 600);
        APP_FLAG_SET(L2CAP_PARAM_UPD_SENT);
    }
#endif
    
#if CLIENT
    
    /* Start TX handle Characteristic discovery if not yet done */
    if (APP_FLAG(CONNECTED) && !APP_FLAG(END_READ_TX_CHAR_HANDLE))
    {
      if (!APP_FLAG(START_READ_TX_CHAR_HANDLE))
      {
        /* Discovery TX characteristic handle by UUID 128 bits */
        
         const tHalUint8 charUuid128_TX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
         
         aci_gatt_discovery_characteristic_by_uuid(connection_handle, 0x0001, 0xFFFF,UUID_TYPE_128,
                                                   charUuid128_TX);
         APP_FLAG_SET(START_READ_TX_CHAR_HANDLE);
      }
    }
    /* Start RX handle Characteristic discovery if not yet done */
    else if (APP_FLAG(CONNECTED) && !APP_FLAG(END_READ_RX_CHAR_HANDLE))
    {
      /* Discovery RX characteristic handle by UUID 128 bits */
      if (!APP_FLAG(START_READ_RX_CHAR_HANDLE))
      {
        /* Discovery TX characteristic handle by UUID 128 bits */
        
         const tHalUint8 charUuid128_RX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};
         aci_gatt_discovery_characteristic_by_uuid(connection_handle, 0x0001, 0xFFFF,UUID_TYPE_128,
                                                   charUuid128_RX);
         APP_FLAG_SET(START_READ_RX_CHAR_HANDLE);
       }
    }
    
    if(APP_FLAG(CONNECTED) && APP_FLAG(END_READ_TX_CHAR_HANDLE) && APP_FLAG(END_READ_RX_CHAR_HANDLE) && !APP_FLAG(NOTIFICATIONS_ENABLED)){
        uint8_t client_char_conf_data[] = {0x01, 0x00}; // Enable notifications
        struct timer t;
        Timer_Set(&t, CLOCK_SECOND*10);
        
        while(aci_gatt_write_charac_descriptor(connection_handle, tx_handle+2, 2, client_char_conf_data)==BLE_STATUS_NOT_ALLOWED){ //TX_HANDLE;
            // Radio is busy.
            if(Timer_Expired(&t)) break;
        }
        APP_FLAG_SET(NOTIFICATIONS_ENABLED);
      }    
   
#endif
    
#if ACCEL_TEST
    //if(APP_FLAG(CONNECTED) && APP_FLAG(NOTIFICATIONS_ENABLED))
    {
      AxesRaw_t data;
      status_t response;
      tHalUint8 accdata[20] = {0};
      //get Acceleration data
      response = LIS3DH_GetAccAxesRaw(&data);
      if(response){
          LIS3DH_ConvAccValue(&data);
          sprintf(accdata,"1 %5d %5d %5d\n",data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
          //PRINTF(accdata);
          struct timer t;
          Timer_Set(&t, CLOCK_SECOND*10);
                              
          while(aci_gatt_update_char_value(chatServHandle, TXCharHandle, 0, 20, accdata)==BLE_STATUS_INSUFFICIENT_RESOURCES)
          {
            // Radio is busy (buffer full).
            if(Timer_Expired(&t))
                break;
          }
          Clock_Wait(200);
      }
    }
#endif 
    

#if THROUGHPUT_TEST && SERVER
    
    static uint8_t test_done = FALSE;
    
    if(APP_FLAG(CONNECTED) && !test_done && APP_FLAG(NOTIFICATIONS_ENABLED)){
    
        tHalUint8 data[20] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','G','H','I','J'};
        
        static tClockTime time, time2;
        time = Clock_Time();
        
        for(int i = 0; i < NUM_PACKETS; i++){
            
            struct timer t;
            Timer_Set(&t, CLOCK_SECOND*10);
                            
            while(aci_gatt_update_char_value(chatServHandle, TXCharHandle, 0, 20, data)==BLE_STATUS_INSUFFICIENT_RESOURCES)
            {
              // Radio is busy (buffer full).
              if(Timer_Expired(&t))
                  break;
            }
        
        }
        
        time2 = Clock_Time();
        tClockTime diff = time2-time;
        printf("\n%d packets. Elapsed time: %d ms. App throughput: %.2f kbps.\n", NUM_PACKETS, diff, (float)NUM_PACKETS*20*8/diff);
        
        test_done = TRUE;
    }
#endif
    
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
                    GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
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
                    evt_gatt_attr_modified *evt = (evt_gatt_attr_modified*)blue_evt->data;
                    Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);                    
                }
                break;
            case EVT_BLUE_GATT_NOTIFICATION:
                {
                    evt_gatt_attr_notification *evt = (evt_gatt_attr_notification*)blue_evt->data;
                    GATT_Notification_CB(evt->attr_handle, evt->data_length - 2, evt->attr_value);
                }
                break;
             case EVT_BLUE_L2CAP_CONN_UPD_RESP:
                {
                    evt_l2cap_conn_upd_resp *resp = (void*)blue_evt->data;
                    if(resp->result){
                        PRINTF("> Connection parameters rejected.\n");
                    }
                    else{
                        PRINTF("> Connection parameters accepted.\n");
                    }
                }
                break;
#ifdef CLIENT            
              case EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP:
                {
                    evt_gatt_disc_read_char_by_uuid_resp *resp = (void*)blue_evt->data;
                    
                    if (APP_FLAG(START_READ_TX_CHAR_HANDLE) && !APP_FLAG(END_READ_TX_CHAR_HANDLE))
                    {
                      tx_handle = resp->attr_handle;
                      PRINTF("TX Char Handle %04X\n", tx_handle);
                    }
                    else if (APP_FLAG(START_READ_RX_CHAR_HANDLE) && !APP_FLAG(END_READ_RX_CHAR_HANDLE))
                    {
                      rx_handle = resp->attr_handle;
                      PRINTF("RX Char Handle %04X\n", rx_handle);
                    }
                    PRINTF("Receive sth is %04X\n",resp->attr_handle);
                    //server_2 process
                    if (APP_FLAG2(START_READ_TX_CHAR_HANDLE) && !APP_FLAG2(END_READ_TX_CHAR_HANDLE))
                    {
                      tx_handle2 = resp->attr_handle;
                      PRINTF("TX2 Char2 Handle2 %04X\n", tx_handle2);
                    }
                    else if (APP_FLAG2(START_READ_RX_CHAR_HANDLE) && !APP_FLAG2(END_READ_RX_CHAR_HANDLE))
                    {
                      rx_handle2 = resp->attr_handle;
                      PRINTF("RX2 Char2 Handle2 %04X\n", rx_handle2);
                    }
                }
                break;  
                
                case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
                {
                  /* Wait for gatt procedure complete event trigger related to Discovery Charac by UUID */
                  //evt_gatt_procedure_complete *pr = (void*)blue_evt->data;
                  
                  if (APP_FLAG(START_READ_TX_CHAR_HANDLE) && !APP_FLAG(END_READ_TX_CHAR_HANDLE))
                  {
                    APP_FLAG_SET(END_READ_TX_CHAR_HANDLE);
                  }
                  else if (APP_FLAG(START_READ_RX_CHAR_HANDLE) && !APP_FLAG(END_READ_RX_CHAR_HANDLE))
                  {
                    APP_FLAG_SET(END_READ_RX_CHAR_HANDLE);
                  }
                  
                  if (APP_FLAG2(START_READ_TX_CHAR_HANDLE) && !APP_FLAG2(END_READ_TX_CHAR_HANDLE))
                  {
                    APP_FLAG_SET2(END_READ_TX_CHAR_HANDLE);
                  }
                  else if (APP_FLAG2(START_READ_RX_CHAR_HANDLE) && !APP_FLAG2(END_READ_RX_CHAR_HANDLE))
                  {
                    APP_FLAG_SET2(END_READ_RX_CHAR_HANDLE);
                  }
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
