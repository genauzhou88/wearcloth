/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : low_power.c
* Author             : AMS - HEA&RF BU
* Version            : V1.0.0
* Date               : 04-Oct-2013
* Description        : Functions for low power management
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifdef SYSCLK_MSI

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "gp_timer.h"
#include "hw_config.h"
#include "platform_config.h"

#include "SDK_EVAL_Config.h"

/* Variables used for saving GPIO configuration */
uint32_t GPIOA_MODER, GPIOB_MODER, GPIOC_MODER,GPIOD_MODER,GPIOE_MODER ,GPIOE_MODER,GPIOH_MODER;
uint32_t GPIOA_PUPDR, GPIOB_PUPDR , GPIOC_PUPDR, GPIOD_PUPDR,GPIOE_PUPDR,GPIOH_PUPDR;

#define RUN     0
#define SLEEP   1
#define STOP    2

volatile uint8_t LP_mode = RUN;

/*******************************************************************************
* Function Name  : GPIO_LowPower_Config
* Description    : Configures GPIO to reach the lowest power consumption.
* Input          : None.
* Return         : None.
*******************************************************************************/
void GPIO_LowPower_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIOA_MODER = GPIOA->MODER;
    GPIOB_MODER = GPIOB->MODER;
    GPIOC_MODER = GPIOC->MODER;
    GPIOD_MODER = GPIOD->MODER;
    GPIOE_MODER = GPIOE->MODER;
    GPIOH_MODER = GPIOH->MODER;
    
    GPIOA_PUPDR = GPIOA->PUPDR;
    GPIOB_PUPDR = GPIOB->PUPDR;
    GPIOC_PUPDR = GPIOC->PUPDR;
    GPIOD_PUPDR = GPIOD->PUPDR;
    GPIOE_PUPDR = GPIOE->PUPDR;
    GPIOH_PUPDR = GPIOH->PUPDR;
    
    /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    
    GPIOD->MODER   = 0xFFFFFFFF;
    GPIOE->MODER   = 0xFFFFFFFF;
    GPIOH->MODER   = 0xFFFFFFFF;
    
    /* Configure unused GPIO port pins in Analog Input mode (floating input trigger OFF) */
    
    /*  PA4 is CS for LIS3DH, PA5,6,7 for  LIS3DH SPI */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | \
         GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14| GPIO_Pin_15; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
     /* PB11 is for INT2 pin of LIS3DH. PB6 and PB7 are for STLM75. PB12,13,14,15 are for BlueNRG SPI. */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4| GPIO_Pin_5 | GPIO_Pin_8 | \
        GPIO_Pin_9 | GPIO_Pin_10 ;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* Only STEVAL-IDB002V1 is used for SensorDemo */
    /* PC0, PC1, PC4, PC5 are for LEDs. PC10 is for BlueNRG IRQ. PC13 is BlueNRG RESET */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_8 \
        | GPIO_Pin_9 | GPIO_Pin_11 |  GPIO_Pin_12 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    /* PD2 is for LEDs. */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_4|  GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_8\
        | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    /* Turn off LEDs. TBR*/
    //GPIO_HIGH(LD_PORT_1,LD_GREEN);
    //GPIO_HIGH(LD_PORT_1,LD_ORANGE);
    //GPIO_HIGH(LD_PORT_1,LD_BLUE);
    //GPIO_HIGH(LD_PORT_1,LD_YELLOW);
    //GPIO_HIGH(LD_PORT_2,LD_RED);
    SdkEvalLedOff(LED1);
    SdkEvalLedOff(LED2);
    SdkEvalLedOff(LED3);
    SdkEvalLedOff(LED4);
    SdkEvalLedOff(LED5);
}

/*******************************************************************************
* Function Name  : Restore_GPIO_Config
* Description    : Restore GPIO settings as before GPIO_LowPower_Config.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Restore_GPIO_Config(void)
{
    GPIOA->MODER = GPIOA_MODER;
    GPIOB->MODER = GPIOB_MODER;
    GPIOC->MODER = GPIOC_MODER;
    GPIOD->MODER = GPIOD_MODER;
    GPIOE->MODER = GPIOE_MODER;
    GPIOH->MODER = GPIOH_MODER;
    
    GPIOA->PUPDR = GPIOA_PUPDR;
    GPIOB->PUPDR = GPIOB_PUPDR;
    GPIOC->PUPDR = GPIOC_PUPDR;
    GPIOD->PUPDR = GPIOD_PUPDR;
    GPIOE->PUPDR = GPIOE_PUPDR;
    GPIOH->PUPDR = GPIOH_PUPDR;
}

/*******************************************************************************
* Function Name  : ChangeMSIClock
* Description    : Change MSI frequency
* Input          : freq: frequency range,
*                  div2: if FALSE, HCLK will be SYSCLK, otherwise SYSCLK/2.
* Return         : None.
*******************************************************************************/
void ChangeMSIClock(uint32_t freq, bool div2)
{ 
    /* Enable the PWR APB1 Clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    
    /* To configure the MSI frequency */
    RCC_MSIRangeConfig(freq);
    
    if (div2)
    {
        RCC_HCLKConfig(RCC_SYSCLK_Div2);
    }
    else {
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
    }
}

/*******************************************************************************
* Function Name  : User_Timer_Enter_Sleep
* Description    : Adjust timer prescaler when entering sleep mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void User_Timer_Enter_Sleep(void)
{
    uint16_t cnt;
    
    if(TIM3->CR1 & TIM_CR1_CEN){
        TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
        TIM3->PSC = USER_TIMER_PRESCALER_SLEEP;
        cnt = TIM3->CNT;
        TIM3->EGR = TIM_PSCReloadMode_Immediate;
        TIM3->CNT = cnt;
        TIM_ClearFlag(TIM3, TIM_FLAG_Update);
        TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    }
}

/*******************************************************************************
* Function Name  : User_Timer_Exit_Sleep
* Description    : Adjust timer prescaler when exiting sleep mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void User_Timer_Exit_Sleep(void)
{  
    uint16_t cnt;
    
    if(TIM3->CR1 & TIM_CR1_CEN){
        TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
        TIM3->PSC = USER_TIMER_PRESCALER;
        cnt = TIM3->CNT;
        TIM3->EGR = TIM_PSCReloadMode_Immediate;
        TIM3->CNT = cnt;
        TIM_ClearFlag(TIM3, TIM_FLAG_Update);
        TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    }    
}

/**
  * @brief  Configures the RTC periodic auto-wakeup interrupts. The wakeup timer
            clock input is LSI divided by 16 (about 38 kHz/16 = 2.37 kHz).
  *         LSI 
  * @param  RTC_WakeUpCounter: specifies the WakeUp counter.
  * @retval None
  */
void RTC_AutoWakeup_Config(void)
{
    
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    
    /* Allow access to the RTC */
    PWR_RTCAccessCmd(ENABLE);
    
    /* Reset Backup Domain */
    RCC_RTCResetCmd(ENABLE);
    RCC_RTCResetCmd(DISABLE);    
    
    RCC_LSICmd(ENABLE);
    
    /* Wait till LSI is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    {}
    
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
    
    
    RCC_RTCCLKCmd(ENABLE);
    
    /* EXTI configuration *******************************************************/
    EXTI_ClearITPendingBit(EXTI_Line20);
    EXTI_InitStructure.EXTI_Line = EXTI_Line20;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    /* Enable the RTC Wakeup Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);  
    
    /* Configure the RTC WakeUp Clock source */
    RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);
    
    RTC_ITConfig(RTC_IT_WUT, ENABLE);
    
    RCC_RTCCLKCmd(DISABLE);
    
}

/*******************************************************************************
* Function Name  : Enter_LP_Sleep_Mode
* Description    : Put the micro in low power sleep mode. This function assumes
*                   that the system clock is already MSI.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LP_Sleep_Mode()
{
    //__disable_interrupt();
    
    LP_mode = SLEEP;
    
    GPIO_LowPower_Config();
    
    /* Disable PVD */
    PWR_PVDCmd(DISABLE);
    
    /* Stop the sys tick to avoid interrupts */
    SysTick->CTRL = 0;
    
    /* Switch in MSI 65KHz, HCLK 32kHz */
    ChangeMSIClock(RCC_MSIRange_0,TRUE);
    
    User_Timer_Enter_Sleep();
    
    //__enable_interrupt();
    /* Request Wait For Interrupt */    
    PWR_EnterSleepMode(PWR_Regulator_LowPower,PWR_SLEEPEntry_WFI);
    
}

/**
  * @brief  Put the micro in stop mode.
  * @param  RTC_WakeUpCounter: specifies the WakeUp counter.
  * @retval None
  */
void Enter_Stop_Mode(uint32_t RTC_WakeUpCounter)
{    
    GPIO_LowPower_Config(); 
    
    //__disable_interrupt();
    
    LP_mode = STOP;
    
    /* Stop the sys tick to avoid interrupts */
    SysTick->CTRL = 0;
    
    if(RTC_WakeUpCounter > 0){
    
      RCC_RTCCLKCmd(ENABLE); 
      RTC_SetWakeUpCounter(RTC_WakeUpCounter);
      RTC_WakeUpCmd(ENABLE);
    }
    
    //__enable_interrupt();
    /* Request Wait For Interrupt */    
    PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI); 
    
    /* Clear Wake Up flag */
    PWR_ClearFlag(PWR_FLAG_WU);    
}

/*******************************************************************************
* Function Name  : Exit_LP_Sleep_Mode
* Description    : Put the micro in run mode, with MSI @ 4MHz as SYSCLK.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Exit_LP_Mode(void)
{    
    if(LP_mode == SLEEP){
        
        /* Switch in MSI 4MHz, HCLK 4MHz */
        ChangeMSIClock(RCC_MSIRange_6,FALSE);
        
        User_Timer_Exit_Sleep();
        
        Restore_GPIO_Config();
        
        Clock_Init();
        
        LP_mode = RUN;
    }
    else if(LP_mode == STOP){
     
        Restore_GPIO_Config();
        
        RTC_WakeUpCmd(DISABLE);
        RCC_RTCCLKCmd(DISABLE);
        
        Clock_Init();
        
        LP_mode = RUN;
    }
}

#endif /* SYSCLK_MSI */