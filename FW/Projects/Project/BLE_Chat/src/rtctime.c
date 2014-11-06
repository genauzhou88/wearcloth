/**
 * @file  rtctime.c
 * @brief This is RTC time interface which will keep time in FW.
 *
**/
#include "string.h"
#include <stdio.h>
#include "stm32l1xx.h"

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* RTC Related ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Uncomment the corresponding line to select the RTC Clock source */
//#define RTC_CLOCK_SOURCE_LSE   /* LSE used as RTC source clock */
#define RTC_CLOCK_SOURCE_LSI */ /* LSI used as RTC source clock. The RTC Clock
                                      may varies due to LSI frequency dispersion. */ 

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RTC_TimeTypeDef RTC_TimeStructure;
RTC_DateTypeDef RTC_DateStructure;
RTC_InitTypeDef RTC_InitStructure;

__IO uint32_t AsynchPrediv = 0, SynchPrediv = 0;

/**
  * @brief  Configure the RTC peripheral by selecting the clock source.
  * @param  None
  * @retval None
  */
void RTC_Config(void)
{
    /* Enable the PWR clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    /* Allow access to RTC */
    PWR_RTCAccessCmd(ENABLE);
      
  #if defined (RTC_CLOCK_SOURCE_LSI)  /* LSI used as RTC source clock*/
  /* The RTC Clock may varies due to LSI frequency dispersion. */   
    /* Enable the LSI OSC */ 
    printf("LSI OSC is selected!\n");
    RCC_LSICmd(ENABLE);

    /* Wait till LSI is ready */  
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    {
    }

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
    //change the prediv based on current clock
    //refer to AN3371 pg9
    SynchPrediv =  0x127;//0xFF;
    AsynchPrediv = 0x7C;//0x7F;

  #elif defined (RTC_CLOCK_SOURCE_LSE) /* LSE used as RTC source clock */
    /* Enable the LSE OSC */
    RCC_LSEConfig(RCC_LSE_ON);

    /* Wait till LSE is ready */  
    while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {
    }

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    
    SynchPrediv =  0x127;//0xFF;
    AsynchPrediv = 0x7C;//0x7F;

  #else
    #error Please select the RTC Clock source inside the main.c file
  #endif /* RTC_CLOCK_SOURCE_LSI */
    
    /* Enable the RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC APB registers synchronisation */
    RTC_WaitForSynchro();
}

/**
  * @brief  Display the current time on the Hyperterminal.
  * @param  None
  * @retval None
  */
void RTC_TimeShow(void)
{
    /* Get the current Time */
    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
    printf("\n\r  The current time is :  %0.2d:%0.2d:%0.2d \n\r", RTC_TimeStructure.RTC_Hours, RTC_TimeStructure.RTC_Minutes, RTC_TimeStructure.RTC_Seconds);
    
    /* Get the current Date */
    RTC_GetDate(RTC_Format_BIN,&RTC_DateStructure);
    printf("\n\r  The current date is :  %0.2d:%0.2d:%0.2d \n\r", RTC_DateStructure.RTC_Year, RTC_DateStructure.RTC_Month, RTC_DateStructure.RTC_Date);
    

}
void RTC_SetMyDate(uint32_t yy, uint32_t mm, uint32_t dd)
{
    RTC_DateStructure.RTC_Year =  yy;
    RTC_DateStructure.RTC_Month = mm;
    RTC_DateStructure.RTC_Date = dd;
    
    /* Configure the RTC time register */
    if(RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure) == ERROR)
    {
      printf("\n\r>> !! RTC Set Date failed. !! <<\n\r");
    }
    else
    {
      printf("\n\r>> !! RTC Set Date success. !! <<\n\r");
      RTC_TimeShow();
     
    }
}
void RTC_SetMyTime(uint32_t hh, uint32_t mm, uint32_t ss)
{
    uint32_t tmp_hh = hh, tmp_mm = mm, tmp_ss = ss;
    RTC_TimeStructure.RTC_Hours = tmp_hh;
    RTC_TimeStructure.RTC_Minutes = tmp_mm;
    RTC_TimeStructure.RTC_Seconds = tmp_ss;
    
    /* Configure the RTC time register */
    if(RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure) == ERROR)
    {
      printf("\n\r>> !! RTC Set Time failed. !! <<\n\r");
    } 
    else
    {
      printf("\n\r>> !! RTC Set Time success. !! <<\n\r");
      RTC_TimeShow();
    }
}
void RTC_DateRegulate()
{
  uint32_t tmp_yy = 0xFF, tmp_mm = 0xFF, tmp_dd = 0xFF;
  RTC_DateStructure.RTC_Year =  tmp_yy;
  RTC_DateStructure.RTC_Month = tmp_mm;
  RTC_DateStructure.RTC_Date = tmp_dd;
  /* Configure the RTC date register */
  if(RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure) == ERROR)
  {
    printf("\n\r>> !! RTC Set Date failed. !! <<\n\r");
  }
  else
  {
    printf("\n\r>> !! RTC Set Date success. !! <<\n\r");
    RTC_TimeShow();
    /* Indicator for the RTC configuration */
    RTC_WriteBackupRegister(RTC_BKP_DR0, 0x32F2);
  }
}
/**
  * @brief  Returns the time entered by user, using Hyperterminal.
  * @param  None
  * @retval None
  */
void RTC_TimeRegulate(void)
{
    uint32_t tmp_hh = 0xFF, tmp_mm = 0xFF, tmp_ss = 0xFF;
    RTC_TimeStructure.RTC_Hours = tmp_hh;
    RTC_TimeStructure.RTC_Minutes = tmp_mm;
    RTC_TimeStructure.RTC_Seconds = tmp_ss;
    
    /* Configure the RTC time register */
    if(RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure) == ERROR)
    {
      printf("\n\r>> !! RTC Set Time failed. !! <<\n\r");
    } 
    else
    {
      printf("\n\r>> !! RTC Set Time success. !! <<\n\r");
      RTC_DateRegulate();
      RTC_TimeShow();
      /* Indicator for the RTC configuration */
      RTC_WriteBackupRegister(RTC_BKP_DR0, 0x32F2);
    }

    
}



