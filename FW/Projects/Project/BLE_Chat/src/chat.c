#include <stdio.h>
#include <string.h>

#include "stm32l1xx.h"
#include "hal_types.h"
#include "gap.h"
#include "gatt_server.h"
#include "hci.h"
#include "osal.h"
#include "clock.h"
#include "gp_timer.h"
#include "bluenrg_hci.h"

#include "rtctime.h"
/* Private macros ------------------------------------------------------------*/
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define MIN(a,b)            ((a) < (b) )? (a) : (b) 

#define CMD_BUFF_SIZE 512

static char cmd[CMD_BUFF_SIZE];

extern tHalUint16 chatServHandle, TXCharHandle, RXCharHandle;
extern tHalUint16 chatServHandle2, TXCharHandle2, RXCharHandle2;
extern volatile uint16_t connection_handle;
extern volatile uint16_t connection2_handle;


#ifdef CLIENT
extern uint16_t rx_handle;
extern uint16_t rx_handle2;
#endif 

void mysetsettime(char *s1,uint8_t type)
{
    char seps[] = " ";
    char *token = NULL;
    char *next_token = NULL;
    
    token = strtok_r(s1, seps, &next_token);
    int i = 0;
    int hr, min, sec;  
    while (token != NULL)
    {
      
      if (i == 1)
        {
          hr = atoi(token);
          //printf("hour/year is %d\n", hr);
        }
        if (i == 2)
        {
          min = atoi(token);
          //printf("min is %s\n", min);
        }
        if (i == 3 )
        {
          sec = atoi(token);
          //printf ("sec/date is %s\n", sec);
          //ToDO: Need data check here;
          if (type == 1) //set time
            RTC_SetMyTime(hr,min,sec);
          if (type == 2) //set date
            RTC_SetMyDate(hr,min,sec);
        }
        token = strtok_r(NULL,seps,&next_token);
        i++;
    }
}

int mystrcmp(const char *s1, const char *s2, uint8_t length)
{       /* compare unsigned char s1[], s2[] */
    int i;
    for (i=0; *s1 == *s2; ++s1, ++s2,i++)
    {
      //printf("s1 is %02X, s2 is %02X\n",*s1,*s2);
      
    }
  //printf("after compare, i is %d, lens of s2 is %d, s1 is %02X\n",i,length,*s1);
      if (i == length)
        return (0);
        
      if (*s1 < 32 || *s2 > 126)
        return (0);
    return (*(unsigned char *)s1 < *(unsigned char *)s2
                ? -1 : +1);
}
/*******************************************************************************
* Function Name  : processInputData.
* Description    : Process a command. It should be called when data are received.
* Input          : data_buffer: data address.
*	           Nb_bytes: number of received bytes.
* Return         : none.
*******************************************************************************/

void processInputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
    static uint16_t end = 0;
    uint8_t i;
    //PRINTF("DATA is received in processInputData!!\n");
    //PRINTF("DATA is %s", data_buffer);
    char cmd_gettime[] = "showtime";
    char cmd_settime[] = "settime";
    char cmd_setdate[] = "setdate";
    //Process command from user
    if (mystrcmp(data_buffer, cmd_gettime,strlen(cmd_gettime)) == 0)
    {
      printf("let's show time!!\n");
      RTC_TimeShow();
    }
    
    if (mystrcmp(data_buffer,cmd_settime,strlen(cmd_settime)) == 0)
    {
      printf("let's set time!!\n");
      mysetsettime(data_buffer,1);
    }
    
    if (mystrcmp(data_buffer,cmd_setdate,strlen(cmd_setdate)) == 0)
    {
      printf("let's set date!!\n");
      mysetsettime(data_buffer,2);
    }
    for (i = 0; i < Nb_bytes; i++)
    {
        if(end >= CMD_BUFF_SIZE-1){
            end = 0;
        }
        
        cmd[end] = data_buffer[i];
        end++;
        
        if(cmd[end-1] == '\n'){
            if(end != 1){
                int j = 0;
                cmd[end] = '\0';
                    
                while(j < end){
                    uint32_t len = MIN(20, end - j);
                    struct timer t;
                    Timer_Set(&t, CLOCK_SECOND*10);
                    
#if SERVER                      
                    //PRINTF("running on SERVER \n");
                    while(aci_gatt_update_char_value(chatServHandle,TXCharHandle,0,len,(tHalUint8 *)cmd+j)==BLE_STATUS_INSUFFICIENT_RESOURCES){
#elif SERVER_2                      
                    
                    //PRINTF("running on SERVER_2! \n");
                    while(aci_gatt_update_char_value(chatServHandle2,TXCharHandle2,0,len,(tHalUint8 *)cmd+j)==BLE_STATUS_INSUFFICIENT_RESOURCES){
#elif CLIENT
                    //PRINTF("running on CLENT, waiting for SERVER!\n");
                    while(aci_gatt_write_without_response(connection_handle, rx_handle+1, len, (tHalUint8 *)cmd+j)==BLE_STATUS_NOT_ALLOWED){
#else
#error "Define SERVER or CLIENT"
#endif
                        // Radio is busy (buffer full).
                        if(Timer_Expired(&t))
                        {
                            PRINTF("time out for waititng from client for server 1 \n");
                            break;
                        }
                    }
#if CLIENT
                    //handle another connection
                    PRINTF("running on CLENT, waiting for SERVER_2!\n");
                    while(aci_gatt_write_without_response(connection2_handle, rx_handle2+1, len, (tHalUint8 *)cmd+j)==BLE_STATUS_NOT_ALLOWED){
                        // Radio is busy (buffer full).
                        if(Timer_Expired(&t))
                        {
                            PRINTF("time out for waititng from client for server 2 \n");
                            break;
                        }
                    }
#endif
                    j += len;            
                }
            }
            end = 0;
        }
    }
}