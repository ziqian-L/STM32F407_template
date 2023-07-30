#include "timer.h"

volatile uint8_t timeout = 0;

void TIM6_init(void)//������ʱ��
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};//��ʱ
	NVIC_InitTypeDef NVIC_InitStructure = {0};
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);//APB1:42Mhz ��ʱ��6��84Mhz/8400 = 10Khz  0.0001s/count  6s
	
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 60000-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 8400-1;
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM6, DISABLE);
}


void set_time(uint16_t time)
{
	timeout = 0;
	if(time > 65535){
		time = 65535;
	}
	TIM_SetAutoreload(TIM6, time -1);
	TIM_Cmd(TIM6, ENABLE);
}

void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)==SET) //����ж�
	{
		timeout = 1;
		TIM_Cmd(TIM6, DISABLE);
		TIM_SetCounter(TIM6, 0);
	}
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update); //����жϱ�־λ
}

