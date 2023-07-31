#include "motor.h"

#define PWM_MIN -7000
#define PWM_MAX 7000

void Motor_drive_Init(void)
{
    //GPIOA时钟
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    
	/******************初始化正反转GPIO输出******************/
	//PG5、PG6、PG7、PG8输出、挽推、100MHz、浮空
    GPIOG->MODER    |= (GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0);
    GPIOG->OTYPER   |= 0x00;
    GPIOG->OSPEEDR  |= (GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7 | GPIO_OSPEEDER_OSPEEDR8);
    GPIOG->PUPDR    |= 0x00;
}

/*****
 * 入口参数：
 *  PSC：预分频系数
 *  ARR：自动重装载值
*****/
void TIM8_PWM_Init(uint16_t PSC,uint16_t ARR)
{
    //GPIOC时钟、TIM8时钟
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

    //GPIO初始化
    //复用输出、挽推、100MHz、下拉
    GPIOC->MODER    |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
    GPIOC->OTYPER   |= 0x00;
    GPIOC->OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
    GPIOC->PUPDR    |= GPIO_PUPDR_PUPDR6_1 | GPIO_PUPDR_PUPDR6_1;

    //GPIO复用
    //PC6、PC7复用为TIM8
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);

    //时基单元初始化
	//向上计数、不分割时钟、预分频系数、自动重装载值
    TIM8->CR1 |= 0<<4;
    TIM8->CR1 |= 0<<8;
    TIM8->PSC  = PSC;
    TIM8->ARR  = ARR;
    
    //CH1、CH2输出比较初始化
    //CH1、CH2配置为PWM1模式，开启输出、高电平有效
    TIM8->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
    TIM8->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);

    TIM8->CCER  |= (TIM_CCER_CC1E | 0x00 | TIM_CCER_CC2E | 0x00);

    //使能预装载值寄存器
    TIM8->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM8->CCMR1 |= TIM_CCMR1_OC2PE;
    //使能自动重装载的预装载寄存器允许位
    TIM8->CR1 |= 1<<7;
	//高级定时器主输出使能
	TIM_CtrlPWMOutputs(TIM8,ENABLE);
    //使能定时器
    TIM_Cmd(TIM8,ENABLE);
}

/**********
 * 限幅函数
**********/
void PWM_Limit(int32_t *motorA,int32_t *motorB)
{
	if(*motorA>PWM_MAX)*motorA=PWM_MAX;
	if(*motorA<PWM_MIN)*motorA=PWM_MIN;
	
	if(*motorB>PWM_MAX)*motorB=PWM_MAX;
	if(*motorB<PWM_MIN)*motorB=PWM_MIN;
}

/**********
 * 绝对值函数
**********/
int32_t my_abs(int32_t p)
{
	int q;
	q=p>0?p:(-p);
	return q;
}

/**********
 * 赋值函数
**********/
void Load_PWM(int32_t motorA,int32_t motorB)
{
    //正反转
    if (motorA > 0) MotorA_IN1 = 0, MotorA_IN2 = 1;
    else            MotorA_IN1 = 1, MotorA_IN2 = 0;
    //赋值，PWMA输出
    TIM_SetCompare1(TIM8,my_abs(motorA));

    //正反转
    if (motorB > 0) MotorB_IN3 = 1, MotorB_IN4 = 0;
    else            MotorB_IN3 = 0, MotorB_IN4 = 1;
    //赋值，PWMB输出
    TIM_SetCompare2(TIM8,my_abs(motorB));
}

void Stop_PWM(void)
{
    MotorA_IN1 = 0, MotorA_IN2 = 0;
    MotorB_IN3 = 0, MotorB_IN4 = 0;
    TIM_SetCompare1(TIM8,0);
    TIM_SetCompare2(TIM8,0);
}


