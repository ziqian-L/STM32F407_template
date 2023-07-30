#include "buzz.h"

/**
 * ��������PE7
 * Ĭ������
**/
void Buzz_Init(void)
{
    //GPIOʱ��
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    //GPIO��ʼ��
    //ͨ����������ơ�100MHz������
    GPIOG->MODER    |= GPIO_MODER_MODER1_0;
    GPIOG->OTYPER   |= 0x00;
    GPIOG->OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR1;
    GPIOG->PUPDR    |= GPIO_PUPDR_PUPDR1_1;
}


void Buzz(uint8_t flag)
{
    if (flag == 0)
        Buzz_PG1 = 0;
    else if (flag == 1)
        Buzz_PG1 = 1;
    else
        Buzz_PG1 = 0;
}
