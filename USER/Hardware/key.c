#include "key.h"
#include "delay.h"
// ������ʼ������
void KEY_Init(void)
{
    // ʹ��GPIOA��GPIOEʱ��
    RCC->AHB1ENR |= 1<<0 | 1<<4;
    // ��ʼ��PE3��PE4
    // ��ͨ����ģʽ�����������100MHz������
    GPIOE->MODER    |= (0<<(2*3) | 0<<(2*4));
    GPIOE->OTYPER   |= (0<<3 | 0<<4);
    GPIOE->OSPEEDR  |= (3<<(2*3) | 3<<(2*4));
    GPIOE->PUPDR    |= (1<<(2*3) | 1<<(2*4));
    // ��ʼ��PA0
    // ��ͨ����ģʽ�����������100MHz������
    GPIOA->MODER    |= 0<<(2*0);
    GPIOA->OTYPER   |= 0<<0;
    GPIOA->OSPEEDR  |= 3<<(2*0);
    GPIOA->PUPDR    |= 2<<(2*0);
}

// ����������
// ���ذ���ֵ
// mode:0,��֧��������;1,֧��������;
// 0��û���κΰ�������
// 1��KEY0����
// 2��KEY1����
// 3��WKUP���� WK_UP
// ע��˺�������Ӧ���ȼ�,KEY0>KEY1>KEY2>WK_UP!!
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//�������ɿ���־
	if(mode)key_up=1;  //֧������		  
	if(key_up&&(KEY0==0||KEY1==0||WK_UP==1))
	{
		delay_ms(10);//ȥ���� 
		key_up=0;
		if(KEY0==0)return 1;
		else if(KEY1==0)return 2;
		else if(WK_UP==1)return 3;
	}else if(KEY0==1&&KEY1==1&&WK_UP==0)key_up=1; 	    
 	return 0;// �ް�������
}

