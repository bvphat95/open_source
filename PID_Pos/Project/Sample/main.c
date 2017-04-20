#include "stm32f10x.h"
#include <math.h>


#define Ts           0.005//5ms
#define Ne           5440
#define PWM_Period   2800 
#define PWM_PORT     GPIOA 
#define PWM_PIN      GPIO_Pin_6
#define DIR_PORT     GPIOB
#define DIR_PIN_0    GPIO_Pin_0
#define DIR_PIN_1    GPIO_Pin_1

GPIO_InitTypeDef            GPIO_InitStructure;
TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
NVIC_InitTypeDef            NVIC_InitStructure;
TIM_OCInitTypeDef           TIM_OCInitStructure;


void encoder_Config(void);
void Delay(__IO uint32_t nCount);
void TIM2_Config(void);
void Motor_Config(void);
void PWM_Config(void);
 
volatile int16_t pulse;
float err,pre_err,des_pulse,real_pos;
float Kp=5,Ki=5,Kd=0;//K=50,T=0.6
float pPart=0,iPart=0,dPart=0;
float ctrl_pos=360;
volatile int32_t out=0;



int main(void)
{
  encoder_Config();
	PWM_Config();
	Motor_Config();
	TIM2_Config();
  while (1)
  {

  }
}

void encoder_Config(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  TIM_EncoderInterfaceConfig(TIM1,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising );
	TIM_Cmd(TIM1,ENABLE);  
}


void TIM2_Config(void)
{
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
   
 /* Time base configuration */
 TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock)/1000000)-1;     // frequency = 1000000,T=10ms
 TIM_TimeBaseStructure.TIM_Period = 5000 - 1;
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
 TIM_Cmd(TIM2, ENABLE);
 
 NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init(&NVIC_InitStructure);   
}

void Motor_Config(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin=PWM_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(PWM_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=DIR_PIN_0|DIR_PIN_1;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(DIR_PORT,&GPIO_InitStructure);	
}

void PWM_Config(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_Period = PWM_Period;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
 
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3,ENABLE);	
	TIM_Cmd(TIM3,ENABLE);
}
void Motor_Control(float des_pos)
{
	pulse=TIM_GetCounter(TIM1)/4;
	//if (pulse>32
	des_pulse=des_pos*Ne/360;
	real_pos=(float)pulse*360/Ne;
	err=des_pulse-pulse;	
	pPart=Kp*err;
	dPart=Kd*(err-pre_err)/Ts;
	iPart+=Ki*Ts*err;
	out=pPart+iPart+dPart;	
	if(out>=0)
		{
			GPIO_SetBits(DIR_PORT, DIR_PIN_0);
	    GPIO_ResetBits(DIR_PORT,DIR_PIN_1);
//			TIM3->CCR1=out;
		}
	else if(out<0) //out<0
		{
			GPIO_ResetBits(DIR_PORT, DIR_PIN_0);
	    GPIO_SetBits(DIR_PORT,DIR_PIN_1);
//			TIM3->CCR1=-out;
		}	
	if(out>=PWM_Period) out=PWM_Period;
	if(out<=-PWM_Period) out=-PWM_Period;
//TIM3->CCR1=out;
	pre_err=err;	
	TIM3->CCR1=out;
}
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
	{
   	Motor_Control(ctrl_pos);
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}

void Delay(__IO uint32_t nCount)
{
 while(nCount--)
 {
 }
}
/*******************************************************************************
Function name: USE_FULL_ASSERT
Decription: None
Input: None
Output: None
*******************************************************************************/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/