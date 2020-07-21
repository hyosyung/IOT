/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/

/* Keyseek Lew*/

#include "device_driver.h"

void Invalid_ISR(void)
{
  Uart1_Printf("Invalid_Exception: %d!\n", Macro_Extract_Area(SCB->ICSR, 0x1ff, 0));
  Uart1_Printf("Invalid_ISR: %d!\n", Macro_Extract_Area(SCB->ICSR, 0x1ff, 0)-16);
  for(;;);
}


#define STACK_DUMP  16

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/*******************************************************************************
 * Function Name  : NMIException
 * Description    : This function handles NMI exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void NMI_Handler(void)
{
  Peri_Set_Bit(RCC->CIR, 23);
  Uart1_Printf("HSE fail!\n");
  
  /* Write rescue code */
}

/*******************************************************************************
 * Function Name  : HardFaultException
 * Description    : This function handles Hard Fault exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
//void HardFault_Handler(void)
//{
//  int i;
//  unsigned int * sp = (unsigned int *)__get_MSP();
//  
//  Uart1_Printf("Hard Fault!\n");
//  
//  for(i=0; i<STACK_DUMP; i++)
//  {
//    Uart1_Printf("SP[%d]=0x%.8X\n", i, sp[i]);
//  }    
//  
//  Uart1_Printf("HFSR => %#.8X\n", SCB->HFSR);
//  Uart1_Printf("SHCSR => %#.8X\n", SCB->SHCSR);
//  Uart1_Printf("Fault Reason => %#.8X\n", SCB->CFSR);
//  Uart1_Printf("MMFAR => %#.8X\n", SCB->MMFAR);        
//  Uart1_Printf("BFAR => %#.8X\n", SCB->BFAR);
//  SCB->HFSR = (1u << 31)|(30 << 1)|(1 << 1);
//  SCB->CFSR = 0xffff << 0;
//  
//  for(;;);
//}

/*******************************************************************************
 * Function Name  : MemManageException
 * Description    : This function handles Memory Manage exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void MemManage_Handler(void)
{
  int i;
  unsigned int * sp = (unsigned int *)__get_MSP();

  Uart1_Printf("Memory Management Fault!\n");  
  
  for(i=0; i<STACK_DUMP; i++)
  {
    Uart1_Printf("SP[%d]=0x%.8X\n", i, sp[i]);
  }  
  
  Uart1_Printf("SHCSR => %#.8X\n", SCB->SHCSR);
  Uart1_Printf("Fault Reason => %#.8X\n", SCB->CFSR);
  Uart1_Printf("MMFAR Valid => %d\n", Macro_Check_Bit_Set(SCB->CFSR, 7));
  Uart1_Printf("MMFAR => %#.8X\n", SCB->MMFAR);   
  SCB->CFSR = 0xff << 0;
  
  for(;;);
}

/*******************************************************************************
 * Function Name  : BusFaultException
 * Description    : This function handles Bus Fault exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void BusFault_Handler(void)
{
  int i;
  unsigned int * sp = (unsigned int *)__get_MSP();
  
  Uart1_Printf("BUS Fault!\n");
  
  for(i=0; i<STACK_DUMP; i++)
  {
    Uart1_Printf("SP[%d]=0x%.8X\n", i, sp[i]);
  }  
  
  Uart1_Printf("SHCSR => %#.8X\n", SCB->SHCSR);
  Uart1_Printf("Fault Reason => %#.8X\n", SCB->CFSR);
  Uart1_Printf("BFAR Valid => %d\n", Macro_Check_Bit_Set(SCB->CFSR, 15));
  Uart1_Printf("BFAR => %#.8X\n", SCB->BFAR);
  SCB->CFSR = 0xff << 8;
  
  for(;;);
}

/*******************************************************************************
 * Function Name  : UsageFaultException
 * Description    : This function handles Usage Fault exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void UsageFault_Handler(void)
{
  int i;
  unsigned int * sp = (unsigned int *)__get_MSP();
  
  Uart1_Printf("Usage Fault!\n");
  
  for(i=0; i<STACK_DUMP; i++)
  {
    Uart1_Printf("SP[%d]=0x%.8X\n", i, sp[i]);
  }  
  
  Uart1_Printf("Saved CPSR(IPSR)=0x%.2X\n", sp[7] & 0xFF);
  Uart1_Printf("SHCSR => %#.8X\n", SCB->SHCSR);
  Uart1_Printf("Fault Reason => %#.8X\n", SCB->CFSR);
  SCB->CFSR = 0xffffu << 16;
  
  for(;;);
}

/*******************************************************************************
 * Function Name  : SVCHandler
 * Description    : This function handles SVCall exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void SVC_Handler(void)
{
  Uart1_Printf("SVC Call\n");
}

/*******************************************************************************
 * Function Name  : DebugMonitor
 * Description    : This function handles Debug Monitor exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void DebugMon_Handler(void)
{
  Uart1_Printf("DebugMon Call\n");
}

/*******************************************************************************
 * Function Name  : PendSVC
 * Description    : This function handles PendSVC exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void PendSV_Handler(void)
{
  Uart1_Printf("PendSV Call\n");
  SCB->ICSR = 1<<27;
}

/*******************************************************************************
 * Function Name  : SysTickHandler
 * Description    : This function handles SysTick Handler.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
volatile int SysTick_Flag = 0;

void SysTick_Handler(void)
{
  SysTick_Flag = 1;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */

/*******************************************************************************
 * Function Name  : WWDG_IRQHandler
 * Description    : This function handles WWDG interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
volatile int WWDG_Time = 0;

void WWDG_IRQHandler(void)
{
  WWDG->CR = WWDG_Time;
  WWDG->SR = 0;
  NVIC_ClearPendingIRQ((IRQn_Type)0);
  
  /* 시험용 추가 코드 */
  Uart1_Printf("@\n");
}

/*******************************************************************************
 * Function Name  : PVD_IRQHandler
 * Description    : This function handles PVD interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void PVD_IRQHandler(void)
{
  Invalid_ISR();
}

/*******************************************************************************
 * Function Name  : TAMPER_IRQHandler
 * Description    : This function handles Tamper interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
volatile int Tamper_Occur = 0;

void TAMPER_IRQHandler(void)
{
  Tamper_Occur = 1;
  
  Peri_Set_Bit(PWR->CR, 8);
  Peri_Set_Bit(BKP->CSR, 0);
  Peri_Set_Bit(BKP->CSR, 1);
  Peri_Clear_Bit(PWR->CR, 8);
  
  NVIC_ClearPendingIRQ((IRQn_Type)2);
}


/*******************************************************************************
 * Function Name  : RTC_IRQHandler
 * Description    : This function handles RTC global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
volatile int RTC_alarm_flag = 0;
volatile int RTC_overflow_flag = 0;
volatile int RTC_second_flag = 0;

void RTC_IRQHandler(void)
{
  if(Peri_Check_Bit_Set(RTC->CRL, 0))
  {
    Peri_Clear_Bit(RTC->CRL,0);
    RTC_second_flag = 1;
  }
  
  if(Peri_Check_Bit_Set(RTC->CRL, 1))
  {
    Peri_Clear_Bit(RTC->CRL,1);
    RTC_alarm_flag = 1;
  }
  
  if(Peri_Check_Bit_Set(RTC->CRL, 2))
  {
    Peri_Clear_Bit(RTC->CRL,2);
    RTC_overflow_flag = 1;
  }
  
  NVIC_ClearPendingIRQ((IRQn_Type)3);
}


/*******************************************************************************
 * Function Name  : FLASH_IRQHandler
 * Description    : This function handles Flash interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void FLASH_IRQHandler(void)
{
  Invalid_ISR();
}

/*******************************************************************************
 * Function Name  : RCC_IRQHandler
 * Description    : This function handles RCC interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void RCC_IRQHandler(void)
{
  Invalid_ISR();
}



void EXTI0_IRQHandler(void)
{

}
void EXTI1_IRQHandler(void)
{

}
void EXTI2_IRQHandler(void){}
void EXTI3_IRQHandler(void){}
void EXTI4_IRQHandler(void){}
void DMA1_Channel1_IRQHandler(void){}
void DMA1_Channel2_IRQHandler(void){}
void DMA1_Channel3_IRQHandler(void){}
void DMA1_Channel4_IRQHandler(void){}
void DMA1_Channel5_IRQHandler(void){}
void DMA1_Channel6_IRQHandler(void){}
void DMA1_Channel7_IRQHandler(void){}
/*******************************************************************************
 * Function Name  : ADC1_2_IRQHandler
 * Description    : This function handles ADC1 and ADC2 global interrupts requests.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
volatile int      ADC1_value = 0;
volatile int      ADC1_flag  = 0;
void ADC1_2_IRQHandler(void)
{
    NVIC_ClearPendingIRQ((IRQn_Type)18);
    Macro_Clear_Bit(ADC1->SR,1);
    ADC1_flag = 1;
    ADC1_value = ADC1->DR;
    NVIC_ClearPendingIRQ((IRQn_Type)18);
}

void USB_HP_CAN1_TX_IRQHandler(void){}
void USB_LP_CAN1_RX0_IRQHandler(void){}
void CAN1_RX1_IRQHandler(void){}
void CAN1_SCE_IRQHandler(void){}
void EXTI9_5_IRQHandler(void){}
void TIM1_BRK_IRQHandler(void){}
void TIM1_UP_IRQHandler(void){}
void TIM1_TRG_COM_IRQHandler(void){}
void TIM1_CC_IRQHandler(void){}

extern void Timer2_ISR(void);
void TIM2_IRQHandler(void)
{

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
		Timer2_ISR();
	}
}
void TIM3_IRQHandler(void){}



/*******************************************************************************
 * Function Name  : TIM4_IRQHandler
 * Description    : This function handles TIM4 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
volatile int TIM4_Expired = 0;

void TIM4_IRQHandler(void)
{
  Peri_Clear_Bit(TIM4->SR, 0);
  NVIC_ClearPendingIRQ((IRQn_Type)30);
  TIM4_Expired = 1;
}

void I2C1_EV_IRQHandler(void){}
void I2C1_ER_IRQHandler(void){}
void I2C2_EV_IRQHandler(void){}
void I2C2_ER_IRQHandler(void){}
void SPI1_IRQHandler(void){}
void SPI2_IRQHandler(void){}

/*******************************************************************************
 * Function Name  : USART1_IRQHandler
 * Description    : This function handles USART1 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
volatile int Uart1_Rx_In = 0;
volatile int Uart1_Rx_Data = 0;

void USART1_IRQHandler(void)
{
  Uart1_Rx_Data = (char)USART1->DR;
  NVIC_ClearPendingIRQ((IRQn_Type)37);
  Uart1_Rx_In = 1;
}

/*******************************************************************************
 * Function Name  : USART2_IRQHandler
 * Description    : This function handles USART2 global interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void USART2_IRQHandler(void)
{
  Invalid_ISR();
}

void USART3_IRQHandler(void){}

/*******************************************************************************
 * Function Name  : EXTI15_10_IRQHandler
 * Description    : This function handles External lines 15 to 10 interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
volatile int Key_Value = 0;

void EXTI15_10_IRQHandler(void)
{
  Key_Value = Macro_Extract_Area(EXTI->PR, 0x7, 13);
  
  EXTI->PR = 0x7<<13;
  NVIC_ClearPendingIRQ((IRQn_Type)40);
}

/*******************************************************************************
 * Function Name  : RTCAlarm_IRQHandler
 * Description    : This function handles RTC Alarm interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void RTCAlarm_IRQHandler(void)
{
  /* For STOP Mode Test Only */
  Peri_Set_Bit(PWR->CR,0);
  Set_Clock_72MHz_HSE();

  Uart1_Printf("EXTI Alarm Wakeup!\n");
  Peri_Set_Bit(EXTI->PR, 17);
  NVIC_ClearPendingIRQ((IRQn_Type)41);
}

void USBWakeUp_IRQHandler(void){}






/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
