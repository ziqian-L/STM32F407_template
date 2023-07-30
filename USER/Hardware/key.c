#include "key.h"
#include "delay.h"
// 按键初始化函数
void KEY_Init(void)
{
    // 使能GPIOA、GPIOE时钟
    RCC->AHB1ENR |= 1<<0 | 1<<4;
    // 初始化PE3、PE4
    // 普通输入模式、挽推输出、100MHz、上拉
    GPIOE->MODER    |= (0<<(2*3) | 0<<(2*4));
    GPIOE->OTYPER   |= (0<<3 | 0<<4);
    GPIOE->OSPEEDR  |= (3<<(2*3) | 3<<(2*4));
    GPIOE->PUPDR    |= (1<<(2*3) | 1<<(2*4));
    // 初始化PA0
    // 普通输入模式、挽推输出、100MHz、下拉
    GPIOA->MODER    |= 0<<(2*0);
    GPIOA->OTYPER   |= 0<<0;
    GPIOA->OSPEEDR  |= 3<<(2*0);
    GPIOA->PUPDR    |= 2<<(2*0);
}

// 按键处理函数
// 返回按键值
// mode:0,不支持连续按;1,支持连续按;
// 0，没有任何按键按下
// 1，KEY0按下
// 2，KEY1按下
// 3，WKUP按下 WK_UP
// 注意此函数有响应优先级,KEY0>KEY1>KEY2>WK_UP!!
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(KEY0==0||KEY1==0||WK_UP==1))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY0==0)return 1;
		else if(KEY1==0)return 2;
		else if(WK_UP==1)return 3;
	}else if(KEY0==1&&KEY1==1&&WK_UP==0)key_up=1; 	    
 	return 0;// 无按键按下
}

