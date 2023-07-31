#include "servos.h"

/****************舵机参数初始化****************/
/*******针对不同的舵机配置不同的PWM输出*******/
/**********
 * 顶舵机角度：		-90     -45      0       45      90       
 * 自动重装载值：
**********/
Server_Typedef Server_Top =
{
	.TIM_CH=1,			//定时器通道(TIM12_CH2(PB15))
	.PWM_Limit_Min=341,	//舵机最小PWM值
	.PWM_Limit_Max=2678,//舵机最大PWM值
	.PWM_Angle_Min=580,	//舵机0度PWM值
	.PWM_Angle_Max=2500,//舵机180/270度时PWM值
	.Angle=180,			//舵机标准角度范围,180/270
	.Angle_MIN=50,		//限幅最小角度值
	.Angle_MAX=130		//限幅最大角度值
};
/**********
 * 底舵机角度：		-90     -45      0       45      90       
 * 自动重装载值：
**********/
Server_Typedef Server_End =
{
	.TIM_CH=2,			//定时器通道(TIM12_CH1(PB14))
	.PWM_Limit_Min=350,	//舵机最小PWM值
	.PWM_Limit_Max=2679,//舵机最大PWM值
	.PWM_Angle_Min=590,	//舵机0度PWM值
	.PWM_Angle_Max=2530,//舵机180/270度时PWM值
	.Angle=180,			//舵机标准角度范围,180/270
	.Angle_MIN=0,		//限幅最小角度值
	.Angle_MAX=180		//限幅最大角度值
};
/******************************************舵机相关******************************************/

/**********
 * 舵机初始化
 * TIM12_CH1(PB14)、TIM12_CH2(PB15)
**********/
void Servos_Init(void)
{
/***************舵机PWM输出初始化***************/
    //开启时钟
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
    //复用、挽推、100MHz、下拉
    GPIOB->MODER    |= GPIO_MODER_MODER14_1|GPIO_MODER_MODER15_1;
    GPIOB->OTYPER   |= 0x00;
    GPIOB->OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR14|GPIO_OSPEEDER_OSPEEDR15;
    GPIOB->PUPDR    |= GPIO_PUPDR_PUPDR14_1|GPIO_PUPDR_PUPDR15_1;
    //GPIO复用
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM12);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_TIM12);
    //初始化时基单元(20ms)
    TIM12->CR1 |= 0<<4;
    TIM12->CR1 |= 0<<8;
    TIM12->PSC = 84-1;
    TIM12->ARR = 20000-1;
    //PWM1模式,使能输出比较、输出比较极性为低,输出比较的预装载值,使能预装载寄存器,使能自动重装载的预装载寄存器允许位
    TIM12->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
    TIM12->CCER  |= (TIM_CCER_CC1E | 0x00 | TIM_CCER_CC2E | 0x00);
    TIM12->CCR1  |= 0x00;
	TIM12->CCR2  |= 0x00;
    TIM12->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM12->CR1 |= TIM_CR1_ARPE;

    //使能定时器
    TIM12->CR1 |= 1<<0;
}

/**********
 * 舵机角度限幅
**********/
void servos_angle_limit(Server_Typedef* servos,float *angle)
{
    if(*angle > servos->Angle_MAX) *angle=servos->Angle_MAX;
    if(*angle < servos->Angle_MIN) *angle=servos->Angle_MIN;
}

/**********
 * 舵机角度修改
**********/
void servos_angle(Server_Typedef* servos,TIM_TypeDef* TIMx,float angle)
{
    float angle_compare = 0;
	//限幅
	servos_angle_limit(servos,&angle);
	//计算PWM
    angle_compare = (float)(servos->PWM_Angle_Max - servos->PWM_Angle_Min)*(float)(angle/servos->Angle);
    angle_compare += servos->PWM_Angle_Min;
	//输出PWM
    TIM_SetComparex(TIMx,angle_compare,servos->TIM_CH);
}

/**********
 * 舵机角度修改
**********/
void Server_Top_Angle(float angle)
{
	servos_angle(&Server_Top,TIM12,angle);
}

/**********
 * 舵机2角度修改
**********/
void Server_End_Angle(float angle)
{
	servos_angle(&Server_End,TIM12,angle);
}

/**********
 * 二维云台角度修改
**********/
void gimbal_angle(float Top_Angle,float End_Angle)
{
	Server_Top_Angle(Top_Angle);
	Server_End_Angle(End_Angle);
}

