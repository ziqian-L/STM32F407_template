#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//ϵͳʱ�ӳ�ʼ��	
//����ʱ������/�жϹ���/GPIO���õ�
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//********************************************************************************
//�޸�˵��
//��
//////////////////////////////////////////////////////////////////////////////////  


//THUMBָ�֧�ֻ������
//�������·���ʵ��ִ�л��ָ��WFI  
__asm void WFI_SET(void)
{
	WFI;		  
}
//�ر������ж�(���ǲ�����fault��NMI�ж�)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR	  
}
//���������ж�
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR  
}
//����ջ����ַ
//addr:ջ����ַ
__asm void MSR_MSP(u32 addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}


/******************************************����ĳһ�жϵ��ж����ȼ�******************************************/
/******
 * �жϳ�ʼ��
 * NVIC_IRQChannel�������ж����ȼ����ж�
 * NVIC_IRQPreemptivePriority����ռ���ȼ�
 * NVIC_IRQResponsePriority����Ӧ���ȼ�
******/
void MY_NVIC_Init(uint8_t NVIC_IRQChannel,uint8_t NVIC_IRQPreemptivePriority,uint8_t NVIC_IRQResponsePriority)
{
    uint8_t Preemptive = 0x00,Response = 0x00,Priority = 0x0F,NVICsetting = 0x00;
    //�õ���ռ���ȼ���λ��
    Preemptive = (0x07 - ((uint32_t)(SCB->AIRCR)>>0x08));
    //�õ���Ӧ���ȼ���λ��
    Response = 0x04 - Preemptive;
    //�õ��ж����ȼ�����
    Priority = Priority >> Preemptive;

    //������жϵ��ж����ȼ�
    NVICsetting = NVIC_IRQPreemptivePriority << Response;
    NVICsetting |= (uint8_t)(NVIC_IRQResponsePriority & Priority);
    NVICsetting = NVICsetting << 0x04;

    //��ֵ
    NVIC->IP[NVIC_IRQChannel] = NVICsetting;

    //ʹ��
    NVIC->ISER[NVIC_IRQChannel >> 0x05] = (uint32_t)0x01 << (NVIC_IRQChannel & (uint8_t)0x1F);
}


/******************************************�޸�Ԥװ��ֵ******************************************/

/**********
 * �����ڣ�
 *      4 ͨ����TIM1��TIM8
 *      4 ͨ����TIM2��TIM3��TIM4��TIM5
 *      2 ͨ����TIM9��TIM12
 *      1 ͨ����TIM10��TIM11��TIM13��TIM14
 * ͨ������4
 * Ԥװ��ֵ��Χ��0~65535��TIM2,TIM5�ɴ� 0 ~ 2^32
**********/
void TIM_SetComparex(TIM_TypeDef* TIMx, uint32_t Compare,uint8_t CH)
{
    if (CH == 1)
    {
        TIMx->CCR1 = Compare;
    }
    else if (CH == 2)
    {
        TIMx->CCR2 = Compare;
    }
    else if (CH == 3)
    {
        TIMx->CCR3 = Compare;
    }
    else if (CH == 4)
    {
        TIMx->CCR4 = Compare;
    }
}









