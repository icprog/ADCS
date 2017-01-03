/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011
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
#include "stm32f4xx_it.h"
#include "main.h"
#include "tcp_echoclient.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint8_t EthLinkStatus;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  /* Update the LocalTime by adding SYSTEMTICK_PERIOD_MS each SysTick interrupt */
  Time_Update();
}

/**
  * @brief  This function handles EXTI15_10
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void) 
{
  
}
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab,u32 Offset)	 
{ 	   	  
	SCB->VTOR=NVIC_VectTab|(Offset&(u32)0xFFFFFE00);//����NVIC��������ƫ�ƼĴ���,VTOR��9λ����,��[8:0]������
}
//����NVIC����
//NVIC_Group:NVIC���� 0~4 �ܹ�5�� 		   
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)	 
{ 
	u32 temp,temp1;	  
	temp1=(~NVIC_Group)&0x07;//ȡ����λ
	temp1<<=8;
	temp=SCB->AIRCR;  //��ȡ��ǰ������
	temp&=0X0000F8FF; //�����ǰ����
	temp|=0X05FA0000; //д��Կ��
	temp|=temp1;	   
	SCB->AIRCR=temp;  //���÷���	    	  				   
}
//����NVIC 
//NVIC_PreemptionPriority:��ռ���ȼ�
//NVIC_SubPriority       :��Ӧ���ȼ�
//NVIC_Channel           :�жϱ��
//NVIC_Group             :�жϷ��� 0~4
//ע�����ȼ����ܳ����趨����ķ�Χ!����������벻���Ĵ���
//�黮��:
//��0:0λ��ռ���ȼ�,4λ��Ӧ���ȼ�
//��1:1λ��ռ���ȼ�,3λ��Ӧ���ȼ�
//��2:2λ��ռ���ȼ�,2λ��Ӧ���ȼ�
//��3:3λ��ռ���ȼ�,1λ��Ӧ���ȼ�
//��4:4λ��ռ���ȼ�,0λ��Ӧ���ȼ�
//NVIC_SubPriority��NVIC_PreemptionPriority��ԭ����,��ֵԽС,Խ����	   
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group)	 
{ 
	u32 temp;	  
	MY_NVIC_PriorityGroupConfig(NVIC_Group);//���÷���
	temp=NVIC_PreemptionPriority<<(4-NVIC_Group);	  
	temp|=NVIC_SubPriority&(0x0f>>NVIC_Group);
	temp&=0xf;								//ȡ����λ
	NVIC->ISER[NVIC_Channel/32]|=1<<NVIC_Channel%32;//ʹ���ж�λ(Ҫ����Ļ�,����ICER��ӦλΪ1����)
	NVIC->IP[NVIC_Channel]|=temp<<4;				//������Ӧ���ȼ����������ȼ�   	    	  				   
} 
//�ⲿ�ж����ú���
//ֻ���GPIOA~I;������PVD,RTC,USB_OTG,USB_HS,��̫�����ѵ�
//����:
//GPIOx:0~8,����GPIOA~I
//BITx:��Ҫʹ�ܵ�λ;
//TRIM:����ģʽ,1,������;2,�Ͻ���;3�������ƽ����
//�ú���һ��ֻ������1��IO��,���IO��,���ε���
//�ú������Զ�������Ӧ�ж�,�Լ�������   	    
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM) 
{ 
	u8 EXTOFFSET=(BITx%4)*4;  
	RCC->APB2ENR|=1<<14;  						//ʹ��SYSCFGʱ��  
	SYSCFG->EXTICR[BITx/4]&=~(0x000F<<EXTOFFSET);//���ԭ�����ã�����
	SYSCFG->EXTICR[BITx/4]|=GPIOx<<EXTOFFSET;	//EXTI.BITxӳ�䵽GPIOx.BITx 
	//�Զ�����
	EXTI->IMR|=1<<BITx;					//����line BITx�ϵ��ж�(���Ҫ��ֹ�жϣ��򷴲�������)
	if(TRIM&0x01)EXTI->FTSR|=1<<BITx;	//line BITx���¼��½��ش���
	if(TRIM&0x02)EXTI->RTSR|=1<<BITx;	//line BITx���¼��������ش���
} 	
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT)//Clears the TIMx's interrupt pending bits.
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));

  /* Clear the IT pending Bit */
  TIMx->SR = (uint16_t)~TIM_IT;
}
/**
  * @brief  Enables or disables the specified TIM interrupts.
  * @param  TIMx: where x can be 1 to 14 to select the TIMx peripheral.
  * @param  TIM_IT: specifies the TIM interrupts sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg TIM_IT_Update: TIM update Interrupt source
  *            @arg TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
  *            @arg TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
  *            @arg TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
  *            @arg TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
  *            @arg TIM_IT_COM: TIM Commutation Interrupt source
  *            @arg TIM_IT_Trigger: TIM Trigger Interrupt source
  *            @arg TIM_IT_Break: TIM Break Interrupt source
  *  
  * @note   For TIM6 and TIM7 only the parameter TIM_IT_Update can be used
  * @note   For TIM9 and TIM12 only one of the following parameters can be used: TIM_IT_Update,
  *          TIM_IT_CC1, TIM_IT_CC2 or TIM_IT_Trigger. 
  * @note   For TIM10, TIM11, TIM13 and TIM14 only one of the following parameters can
  *          be used: TIM_IT_Update or TIM_IT_CC1   
  * @note   TIM_IT_COM and TIM_IT_Break can be used only with TIM1 and TIM8 
  *        
  * @param  NewState: new state of the TIM interrupts.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState)
{  
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_TIM_IT(TIM_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the Interrupt sources */
    TIMx->DIER |= TIM_IT;
  }
  else
  {
    /* Disable the Interrupt sources */
    TIMx->DIER &= (uint16_t)~TIM_IT;
  }
}
/**
  * @brief  Enables or disables the specified TIM peripheral.
  * @param  TIMx: where x can be 1 to 14 to select the TIMx peripheral.
  * @param  NewState: new state of the TIMx peripheral.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx)); 
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the TIM Counter */
    TIMx->CR1 |= TIM_CR1_CEN;
  }
  else
  {
    /* Disable the TIM Counter */
    TIMx->CR1 &= (uint16_t)~TIM_CR1_CEN;
  }
}
/**
  * @brief  Initializes the TIMx Time Base Unit peripheral according to 
  *         the specified parameters in the TIM_TimeBaseInitStruct.
  * @param  TIMx: where x can be  1 to 14 to select the TIM peripheral.
  * @param  TIM_TimeBaseInitStruct: pointer to a TIM_TimeBaseInitTypeDef structure
  *         that contains the configuration information for the specified TIM peripheral.
  * @retval None
  */
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct)
{
  uint16_t tmpcr1 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx)); 
  assert_param(IS_TIM_COUNTER_MODE(TIM_TimeBaseInitStruct->TIM_CounterMode));
  assert_param(IS_TIM_CKD_DIV(TIM_TimeBaseInitStruct->TIM_ClockDivision));

  tmpcr1 = TIMx->CR1;  

  if((TIMx == TIM1) || (TIMx == TIM8)||
     (TIMx == TIM2) || (TIMx == TIM3)||
     (TIMx == TIM4) || (TIMx == TIM5)) 
  {
    /* Select the Counter Mode */
    tmpcr1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS));
    tmpcr1 |= (uint32_t)TIM_TimeBaseInitStruct->TIM_CounterMode;
  }
 
  if((TIMx != TIM6) && (TIMx != TIM7))
  {
    /* Set the clock division */
    tmpcr1 &=  (uint16_t)(~TIM_CR1_CKD);
    tmpcr1 |= (uint32_t)TIM_TimeBaseInitStruct->TIM_ClockDivision;
  }

  TIMx->CR1 = tmpcr1;

  /* Set the Autoreload value */
  TIMx->ARR = TIM_TimeBaseInitStruct->TIM_Period ;
 
  /* Set the Prescaler value */
  TIMx->PSC = TIM_TimeBaseInitStruct->TIM_Prescaler;
    
  if ((TIMx == TIM1) || (TIMx == TIM8))  
  {
    /* Set the Repetition Counter value */
    TIMx->RCR = TIM_TimeBaseInitStruct->TIM_RepetitionCounter;
  }

  /* Generate an update event to reload the Prescaler 
     and the repetition counter(only for TIM1 and TIM8) value immediatly */
  TIMx->EGR = TIM_PSCReloadMode_Immediate;          
}
/**
  * @brief  Gets the TIMx Counter value.
  * @param  TIMx: where x can be 1 to 14 to select the TIM peripheral.
  * @retval Counter Register value
  */
uint32_t TIM_GetCounter(TIM_TypeDef* TIMx)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));

  /* Get the Counter Register value */
  return TIMx->CNT;
}
/**
  * @brief  Checks whether the TIM interrupt has occurred or not.
  * @param  TIMx: where x can be 1 to 14 to select the TIM peripheral.
  * @param  TIM_IT: specifies the TIM interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg TIM_IT_Update: TIM update Interrupt source
  *            @arg TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
  *            @arg TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
  *            @arg TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
  *            @arg TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
  *            @arg TIM_IT_COM: TIM Commutation Interrupt source
  *            @arg TIM_IT_Trigger: TIM Trigger Interrupt source
  *            @arg TIM_IT_Break: TIM Break Interrupt source
  *
  * @note   TIM6 and TIM7 can generate only an update interrupt.
  * @note   TIM_IT_COM and TIM_IT_Break are used only with TIM1 and TIM8.
  *     
  * @retval The new state of the TIM_IT(SET or RESET).
  */
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT)
{
  ITStatus bitstatus = RESET;  
  uint16_t itstatus = 0x0, itenable = 0x0;
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_TIM_GET_IT(TIM_IT));
   
  itstatus = TIMx->SR & TIM_IT;
  
  itenable = TIMx->DIER & TIM_IT;
  if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
