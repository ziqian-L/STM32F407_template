#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//系统时钟初始化	
//包括时钟设置/中断管理/GPIO设置等
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//********************************************************************************
//修改说明
//无
//////////////////////////////////////////////////////////////////////////////////  


//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
__asm void WFI_SET(void)
{
	WFI;		  
}
//关闭所有中断(但是不包括fault和NMI中断)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR	  
}
//开启所有中断
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR  
}
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}


/******************************************设置某一中断的中断优先级******************************************/
/******
 * 中断初始化
 * NVIC_IRQChannel：设置中断优先级的中断
 * NVIC_IRQPreemptivePriority：抢占优先级
 * NVIC_IRQResponsePriority：响应优先级
******/
void MY_NVIC_Init(uint8_t NVIC_IRQChannel,uint8_t NVIC_IRQPreemptivePriority,uint8_t NVIC_IRQResponsePriority)
{
    uint8_t Preemptive = 0x00,Response = 0x00,Priority = 0x0F,NVICsetting = 0x00;
    //得到抢占优先级的位数
    Preemptive = (0x07 - ((uint32_t)(SCB->AIRCR)>>0x08));
    //得到响应优先级的位数
    Response = 0x04 - Preemptive;
    //得到中断优先级分组
    Priority = Priority >> Preemptive;

    //计算该中断的中断优先级
    NVICsetting = NVIC_IRQPreemptivePriority << Response;
    NVICsetting |= (uint8_t)(NVIC_IRQResponsePriority & Priority);
    NVICsetting = NVICsetting << 0x04;

    //赋值
    NVIC->IP[NVIC_IRQChannel] = NVICsetting;

    //使能
    NVIC->ISER[NVIC_IRQChannel >> 0x05] = (uint32_t)0x01 << (NVIC_IRQChannel & (uint8_t)0x1F);
}


/******************************************修改预装载值******************************************/

/**********
 * 适用于：
 *      4 通道的TIM1、TIM8
 *      4 通道的TIM2、TIM3、TIM4、TIM5
 *      2 通道的TIM9、TIM12
 *      1 通道的TIM10、TIM11、TIM13、TIM14
 * 通道数：4
 * 预装载值范围：0~65535、TIM2,TIM5可达 0 ~ 2^32
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









