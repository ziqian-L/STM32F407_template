#include "led.h"
// 初始化 PF9 和 PF10 为输出口.并使能这两个口的时钟
// LED IO 初始化
void LED_Init(void)
{
    // 使能 PORTF 时钟
    RCC->AHB1ENR |= 1 << 5;
//    GPIOF->MODER &= ~(0x03<<(2*9));
//    GPIOF->OTYPER &= ~(1<<9);
//    GPIOF->OSPEEDR &= ~(0x03<<(2*9));
//    GPIOF->PUPDR &= ~(0x03<<(2*9));
    // PF9、PF10模式设置为普通输出模式
    GPIOF->MODER    |= (1<<(2*9) | 1<<(2*10));
    // PF9、PF10输出设置为挽推输出
    GPIOF->OTYPER   |= (0<<9 | 0<<10);
    // PF9、PF10速度设置为100MHz
    GPIOF->OSPEEDR  |= (3<<(2*9) | 3<<(2*10));
    // PF9、PF10设置为上拉
    GPIOF->PUPDR    |= (1<<(2*9) | 1<<(2*10));
    LED0 = 1;
    LED1 = 1;
}
