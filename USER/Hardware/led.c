#include "led.h"
// ��ʼ�� PF9 �� PF10 Ϊ�����.��ʹ���������ڵ�ʱ��
// LED IO ��ʼ��
void LED_Init(void)
{
    // ʹ�� PORTF ʱ��
    RCC->AHB1ENR |= 1 << 5;
//    GPIOF->MODER &= ~(0x03<<(2*9));
//    GPIOF->OTYPER &= ~(1<<9);
//    GPIOF->OSPEEDR &= ~(0x03<<(2*9));
//    GPIOF->PUPDR &= ~(0x03<<(2*9));
    // PF9��PF10ģʽ����Ϊ��ͨ���ģʽ
    GPIOF->MODER    |= (1<<(2*9) | 1<<(2*10));
    // PF9��PF10�������Ϊ�������
    GPIOF->OTYPER   |= (0<<9 | 0<<10);
    // PF9��PF10�ٶ�����Ϊ100MHz
    GPIOF->OSPEEDR  |= (3<<(2*9) | 3<<(2*10));
    // PF9��PF10����Ϊ����
    GPIOF->PUPDR    |= (1<<(2*9) | 1<<(2*10));
    LED0 = 1;
    LED1 = 1;
}
