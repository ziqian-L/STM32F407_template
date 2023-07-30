#include "matrixkey.h"

uint8_t KEY;

/******
 * 矩阵键盘初始化(CH451L)
 * 串行数据输出线DOUT : PE13，键盘中断、数据输出。
 * 串行数据加载线LOAD : PE11，上升沿加载12位命令。
 * 串行数据输入线DIN  : PE9，加载命令给CH451L。在复位后，在DCLK之前输出低电平脉冲(高->低->高)以使能4线串行接口。
 * 串行数据时钟线DCLK : PE7，加载命令时，在每个上升沿加载数据给DIN。输出数据时，在每个下降沿输出数据。
******/
void Matrix_Key_Init(void)
{
    //GPIOE时钟初始化
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    //PE13:普通输入模式、挽推、100MHz、上拉
    GPIOE->MODER    |= 0x00;
    GPIOE->OTYPER   |= 0x00;
    GPIOE->OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR13;
    GPIOE->PUPDR    |= GPIO_PUPDR_PUPDR13_0;
    //PE11、PE9、PE7:通用输出模式、挽推、100MHz、上拉
    GPIOE->MODER    |= GPIO_OSPEEDER_OSPEEDR11_0|GPIO_OSPEEDER_OSPEEDR9_0|GPIO_OSPEEDER_OSPEEDR7_0;
    GPIOE->OTYPER   |= 0x00;
    GPIOE->OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR11|GPIO_OSPEEDER_OSPEEDR9|GPIO_OSPEEDER_OSPEEDR7;
    GPIOE->PUPDR    |= GPIO_PUPDR_PUPDR11_0|GPIO_PUPDR_PUPDR9_0|GPIO_PUPDR_PUPDR7_0;
    //外部中断
    //GPIOE13 连接到中断线 13
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PE;
    //中断线 13 下降沿触发
    EXTI->IMR   |= (1<<13);
    EXTI->FTSR  |= (1<<13);
	//配置中断优先级
	MY_NVIC_Init(EXTI15_10_IRQn,1,1);
    //CH451L初始化,DIN输出低电平脉冲,拉高DCLK、LOAD
    DIN  = 0;
    DIN  = 1;
    DCLK = 1;
    LOAD = 1;
    Matrix_Key_Writer(0x0402);
}

/******
 * 矩阵键盘写命令(CH451L)
******/
void Matrix_Key_Writer(uint16_t command)
{
	uint8_t i;
    LOAD = 0;
    for(i=0;i<12;i++)
    {
        DIN  = (command&0x0001);
        DCLK = 0;
        DCLK = 1;
        command = command >> 1;
    }
    LOAD = 1;
}

/******
 * 矩阵键盘键值判断(CH451L)
******/
void Matrix_Key_Data(uint8_t *key)
{
	switch(*key)
	{
        case 88:*key = 1;   break;
        case 89:*key = 2;   break;
        case 91:*key = 3;   break;
        case 90:*key = 4;   break;
        case 80:*key = 5;   break;
        case 81:*key = 6;   break;
        case 83:*key = 7;   break;
        case 82:*key = 8;   break;
        case 72:*key = 9;   break;
        case 73:*key = 10;  break;
        case 75:*key = 11;  break;
        case 74:*key = 12;  break;
        case 64:*key = 13;  break;
        case 65:*key = 14;  break;
        case 67:*key = 15;  break;
        case 66:*key = 16;  break;
		default:*key = 0;   break;
	}
}

/******
 * 矩阵键盘中断(CH451L)
******/
void EXTI15_10_IRQHandler(void)
{
    uint8_t i;
	KEY = 0;
    if (EXTI->PR & (EXTI->IMR))
    {
        Matrix_Key_Writer(0x700);
        for(i=0;i<7;i++)
        {
            KEY = KEY << 1;
            KEY |= DOUT_Read;
            DCLK = 0;
            DCLK = 1;
        }
        EXTI->PR |= 0<<7;
        Matrix_Key_Data(&KEY);
    }
}


