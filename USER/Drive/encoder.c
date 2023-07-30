#include "encoder.h"

/********************************************************************
 * ʹ�ñ�������Ҫ�õ�������ʱ���ı�����ģʽ
 * ������ѡ��16λ��ʱ��TIM3��TIM4
 * һ·������ʹ��TIM3_CH1(PB4 �� PA6)��TIM3_CH2(PB5 �� PA7)������ѡ��PB4��PB5
 * һ·������ʹ��TIM4_CH1(PB6)��TIM4_CH2(PB7)
********************************************************************/

void TIM3_Encoder_Init(void)
{
    //GPIOBʱ�ӡ�TIM3ʱ��
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    //GPIO��ʼ��
    //���á����ơ�100MHz������
    GPIOB->MODER    |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);
    GPIOB->OTYPER   |= 0x00;
    GPIOB->OSPEEDR  |= (GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5);
    GPIOB->PUPDR    |= 0x00;

    //GPIO����
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);

    //ʱ����Ԫ��ʼ��
    //����������ʱ�Ӳ���Ƶ��Ԥ��Ƶϵ��Ϊ1���Զ���װ��ֵΪ65536
    TIM3->CR1 |= 0x00;
    TIM3->PSC  = 0;
    TIM3->ARR  = 65535;

    //���ñ�����ģʽ
    //������ģʽ3(0x03)�����벶��ͨ��CH3��CH4���Ƿ���/�����ش���
    TIM3->SMCR  |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);
    TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM3->CCER  |= 0x00;
    
    //TIM3_CH1���벶���ʼ��
    //IC1ӳ�䵽TI1������Ƶ���˲�(fSAMPLING=fCK_INT��N=8)
    TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0 | 0x00 | (TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_0));
    //ʹ 8  5 �����벶�񡢷Ƿ���/�����ش���( TIM_CCER_CC1P = 0x00,TIM_CCER_CC1NP = 0x00 )
    TIM3->CCER  |= (TIM_CCER_CC1E | 0x00 | 0x00);
    //TIM3_CH2���벶���ʼ��
    //IC2ӳ�䵽TI2������Ƶ���˲�(fSAMPLING=fCK_INT��N=8)
    TIM3->CCMR1 |= (TIM_CCMR1_CC2S_0 | 0x00 | (TIM_CCMR1_IC2F_1 | TIM_CCMR1_IC2F_0));
    //ʹ�����벶�񡢷Ƿ���/�����ش���( TIM_CCER_CC2P = 0x00,TIM_CCER_CC2NP = 0x00 )
    TIM3->CCER  |= (TIM_CCER_CC2E | 0x00 | 0x00);

    //��������жϱ�־λ
    TIM3->SR = (uint16_t)~TIM_SR_UIF;
    //ʹ�ܸ����ж�
    TIM3->DIER |= TIM_DIER_UIE;
    //������ֵ����
    TIM3->CNT = 0;
    //ʹ�ܶ�ʱ��
    TIM_Cmd(TIM3,ENABLE);
}

void TIM4_Encoder_Init(void)
{
    //GPIOBʱ�ӡ�TIM4ʱ��
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    //GPIO��ʼ��
    //���á����ơ�100MHz������
    GPIOB->MODER    |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
    GPIOB->OTYPER   |= 0x00;
    GPIOB->OSPEEDR  |= (GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7);
    GPIOB->PUPDR    |= 0x00;
    //GPIO����
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);

    //ʱ����Ԫ��ʼ��
    //����������ʱ�Ӳ���Ƶ��Ԥ��Ƶϵ��Ϊ1���Զ���װ��ֵΪ65535
    TIM4->CR1 |= 0x00;
    TIM4->PSC  = 0;
    TIM4->ARR  = 65535;

    //���ñ�����ģʽ
    //������ģʽ3(0x03)�����벶��ͨ��CH1��CH2���Ƿ���/�����ش���
    TIM4->SMCR  |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);
    TIM4->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM4->CCER  |= 0x00;
    
    //TIM4_CH1���벶���ʼ��
    //IC1ӳ�䵽TI1������Ƶ���˲�(fSAMPLING=fCK_INT��N=8)
    TIM4->CCMR1 |= (TIM_CCMR1_CC1S_0 | 0x00 | (TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_0));
    //ʹ�����벶�񡢷Ƿ���/�����ش���( TIM_CCER_CC1P = 0x00,TIM_CCER_CC1NP = 0x00 )
    TIM4->CCER  |= (TIM_CCER_CC1E | 0x00 | 0x00);
    //TIM4_CH2���벶���ʼ��
    //IC2ӳ�䵽TI2������Ƶ���˲�(fSAMPLING=fCK_INT��N=8)
    TIM4->CCMR1 |= (TIM_CCMR1_CC2S_0 | 0x00 | (TIM_CCMR1_IC2F_1 | TIM_CCMR1_IC2F_0));
    //ʹ�����벶�񡢷Ƿ���/�����ش���( TIM_CCER_CC2P = 0x00,TIM_CCER_CC2NP = 0x00 )
    TIM4->CCER  |= (TIM_CCER_CC2E | 0x00 | 0x00);

    //��������жϱ�־λ
    TIM4->SR = (uint16_t)~TIM_SR_UIF;
    //ʹ�ܸ����ж�
    TIM4->DIER |= TIM_DIER_UIE;
    //������ֵ����
    TIM4->CNT = 0;
    //ʹ�ܶ�ʱ��
    TIM_Cmd(TIM4,ENABLE);
}

/**********************
������
�ٶȶ�ȡ����
��ڲ�������ʱ��
**********************/
int Read_Speed(int TIMx)
{
	int value_1;
	switch(TIMx)
	{
        //1.�ɼ��������ļ���ֵ�����档2.����ʱ���ļ���ֵ���㡣
		case 3:value_1=(short)TIM_GetCounter(TIM3);TIM_SetCounter(TIM3,0);break;
		case 4:value_1=(short)TIM_GetCounter(TIM4);TIM_SetCounter(TIM4,0);break;
		default:value_1=0;
	}
	return value_1;
}


void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3,TIM_IT_Update)!=0)
    {
        TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
    }
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4,TIM_IT_Update)!=0)
    {
        TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
    }
}
