#include "servos.h"

/****************���������ʼ��****************/
/*******��Բ�ͬ�Ķ�����ò�ͬ��PWM���*******/
/**********
 * ���1�Ƕȣ�		-90     -45      0       45      90       
 * �Զ���װ��ֵ��	530   1013.75  1497.5  1981.25  2465
**********/
Server_Typedef Server1 =
{
	.TIM_CH=1,			//��ʱ��ͨ��(TIM12_CH1(PB14))
	.PWM_Limit_Min=350,	//�����СPWMֵ
	.PWM_Limit_Max=2679,//������PWMֵ
	.PWM_Angle_Min=590,	//���0��PWMֵ
	.PWM_Angle_Max=2530,//���180/270��ʱPWMֵ
	.Angle=180,			//�����׼�Ƕȷ�Χ,180/270
	.Angle_MIN=0,		//�޷���С�Ƕ�ֵ
	.Angle_MAX=180		//�޷����Ƕ�ֵ
};
/**********
 * ���2�Ƕȣ�		-90     -45      0       45      90       
 * �Զ���װ��ֵ��	530   1013.75  1497.5  1981.25  2465
**********/
Server_Typedef Server2 =
{
	.TIM_CH=2,			//��ʱ��ͨ��(TIM12_CH2(PB15))
	.PWM_Limit_Min=341,	//�����СPWMֵ
	.PWM_Limit_Max=2678,//������PWMֵ
	.PWM_Angle_Min=580,	//���0��PWMֵ
	.PWM_Angle_Max=2500,//���180/270��ʱPWMֵ
	.Angle=180,			//�����׼�Ƕȷ�Χ,180/270
	.Angle_MIN=0,		//�޷���С�Ƕ�ֵ
	.Angle_MAX=180		//�޷����Ƕ�ֵ
};
/******************************************������******************************************/

/**********
 * �����ʼ��
 * TIM12_CH1(PB14)��TIM12_CH2(PB15)
**********/
void Servos_Init(void)
{
/***************���PWM�����ʼ��***************/
    //����ʱ��
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
    //���á����ơ�100MHz������
    GPIOB->MODER    |= GPIO_MODER_MODER14_1|GPIO_MODER_MODER15_1;
    GPIOB->OTYPER   |= 0x00;
    GPIOB->OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR14|GPIO_OSPEEDER_OSPEEDR15;
    GPIOB->PUPDR    |= GPIO_PUPDR_PUPDR14_1|GPIO_PUPDR_PUPDR15_1;
    //GPIO����
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM12);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_TIM12);
    //��ʼ��ʱ����Ԫ(20ms)
    TIM12->CR1 |= 0<<4;
    TIM12->CR1 |= 0<<8;
    TIM12->PSC = 84-1;
    TIM12->ARR = 20000-1;
    //PWM1ģʽ,ʹ������Ƚϡ�����Ƚϼ���Ϊ��,����Ƚϵ�Ԥװ��ֵ,ʹ��Ԥװ�ؼĴ���,ʹ���Զ���װ�ص�Ԥװ�ؼĴ�������λ
    TIM12->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
    TIM12->CCER  |= (TIM_CCER_CC1E | 0x00 | TIM_CCER_CC2E | 0x00);
    TIM12->CCR1  |= 0x00;
	TIM12->CCR2  |= 0x00;
    TIM12->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM12->CR1 |= TIM_CR1_ARPE;

    //ʹ�ܶ�ʱ��
    TIM12->CR1 |= 1<<0;
}

/**********
 * ����Ƕ��޷�
**********/
void servos_angle_limit(Server_Typedef* servos,float *angle)
{
    if(*angle > servos->Angle_MAX) *angle=servos->Angle_MAX;
    if(*angle < servos->Angle_MIN) *angle=servos->Angle_MIN;
}

/**********
 * ����Ƕ��޸�
**********/
void servos_angle(Server_Typedef* servos,TIM_TypeDef* TIMx,float angle)
{
    float angle_compare = 0;
	//�޷�
	servos_angle_limit(servos,&angle);
	//����PWM
    angle_compare = (float)(servos->PWM_Angle_Max - servos->PWM_Angle_Min)*(float)(angle/servos->Angle);
    angle_compare += servos->PWM_Angle_Min;
	//���PWM
    TIM_SetComparex(TIMx,angle_compare,servos->TIM_CH);
}

/**********
 * ���1�Ƕ��޸�
**********/
void Servos1_Angle(float angle)
{
	servos_angle(&Server1,TIM12,angle);
}

/**********
 * ���2�Ƕ��޸�
**********/
void Servos2_Angle(float angle)
{
	servos_angle(&Server2,TIM12,angle);
}

/**********
 * ��ά��̨�Ƕ��޸�
**********/
void gimbal_angle(float Server1_angle,float Server2_angle)
{
	Servos1_Angle(Server1_angle);
	Servos2_Angle(Server2_angle);
}

