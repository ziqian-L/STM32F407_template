#include "motor.h"

#define PWM_MIN -7000
#define PWM_MAX 7000

void Motor_drive_Init(void)
{
    //GPIOAʱ��
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    
	/******************��ʼ������תGPIO���******************/
	//PG5��PG6��PG7��PG8��������ơ�100MHz������
    GPIOG->MODER    |= (GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0);
    GPIOG->OTYPER   |= 0x00;
    GPIOG->OSPEEDR  |= (GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7 | GPIO_OSPEEDER_OSPEEDR8);
    GPIOG->PUPDR    |= 0x00;
}

/*****
 * ��ڲ�����
 *  PSC��Ԥ��Ƶϵ��
 *  ARR���Զ���װ��ֵ
*****/
void TIM8_PWM_Init(uint16_t PSC,uint16_t ARR)
{
    //GPIOCʱ�ӡ�TIM8ʱ��
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

    //GPIO��ʼ��
    //������������ơ�100MHz������
    GPIOC->MODER    |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
    GPIOC->OTYPER   |= 0x00;
    GPIOC->OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
    GPIOC->PUPDR    |= GPIO_PUPDR_PUPDR6_1 | GPIO_PUPDR_PUPDR6_1;

    //GPIO����
    //PC6��PC7����ΪTIM8
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);

    //ʱ����Ԫ��ʼ��
	//���ϼ��������ָ�ʱ�ӡ�Ԥ��Ƶϵ�����Զ���װ��ֵ
    TIM8->CR1 |= 0<<4;
    TIM8->CR1 |= 0<<8;
    TIM8->PSC  = PSC;
    TIM8->ARR  = ARR;
    
    //CH1��CH2����Ƚϳ�ʼ��
    //CH1��CH2����ΪPWM1ģʽ������������ߵ�ƽ��Ч
    TIM8->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
    TIM8->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);

    TIM8->CCER  |= (TIM_CCER_CC1E | 0x00 | TIM_CCER_CC2E | 0x00);

    //ʹ��Ԥװ��ֵ�Ĵ���
    TIM8->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM8->CCMR1 |= TIM_CCMR1_OC2PE;
    //ʹ���Զ���װ�ص�Ԥװ�ؼĴ�������λ
    TIM8->CR1 |= 1<<7;
	//�߼���ʱ�������ʹ��
	TIM_CtrlPWMOutputs(TIM8,ENABLE);
    //ʹ�ܶ�ʱ��
    TIM_Cmd(TIM8,ENABLE);
}

/**********
 * �޷�����
**********/
void PWM_Limit(int32_t *motorA,int32_t *motorB)
{
	if(*motorA>PWM_MAX)*motorA=PWM_MAX;
	if(*motorA<PWM_MIN)*motorA=PWM_MIN;
	
	if(*motorB>PWM_MAX)*motorB=PWM_MAX;
	if(*motorB<PWM_MIN)*motorB=PWM_MIN;
}

/**********
 * ����ֵ����
**********/
int32_t my_abs(int32_t p)
{
	int q;
	q=p>0?p:(-p);
	return q;
}

/**********
 * ��ֵ����
**********/
void Load_PWM(int32_t motorA,int32_t motorB)
{
    //����ת
    if (motorA > 0) MotorA_IN1 = 0, MotorA_IN2 = 1;
    else            MotorA_IN1 = 1, MotorA_IN2 = 0;
    //��ֵ��PWMA���
    TIM_SetCompare1(TIM8,my_abs(motorA));

    //����ת
    if (motorB > 0) MotorB_IN3 = 1, MotorB_IN4 = 0;
    else            MotorB_IN3 = 0, MotorB_IN4 = 1;
    //��ֵ��PWMB���
    TIM_SetCompare2(TIM8,my_abs(motorB));
}

void Stop_PWM(void)
{
    MotorA_IN1 = 0, MotorA_IN2 = 0;
    MotorB_IN3 = 0, MotorB_IN4 = 0;
    TIM_SetCompare1(TIM8,0);
    TIM_SetCompare2(TIM8,0);
}


