/***
 * 本库支持HY-SRF0、HY-SRF05、DFRobot-URM09 V1.0超声波模块
 * 返回的距离单位均为 厘米
***/

#include "ultrasound.h"
#include "delay.h"

/********
 * HY_SRF05        对应HY-SRF04/HY-SRF05/US-100的触发测距模式
 * DFRobot_URM09   对应DFRobot-URM09/HY-SRF05三线触发测距模式(OUT接地)
 * DFRobot_URM09资料：
 * https://wiki.dfrobot.com.cn/SKU_SEN0388_URM09_Ultrasonic_Sensor_Gravity_Trig#target_4
 * 这两种模式的引脚为：PF4、PF6(TIM10_CH1)
 * PF4：发送脉冲
 * PF6：输入捕获
 * **********************************************************************************
 * US_100          对应US-100
 * 引脚为：PC12(UART5_TX)、PD2(UART5_RX)
********/

/*
触发测距思路：

超声波模块工作原理：
单片机发出 10 ~ 20us 的触发信号后，超声波发送高频脉冲测距，单片等待接收模块返回的高电平。高电平时间即为声波经过的距离。
使用公式 【 距离 = 高电平时间 * 声速（340m/s）/ 2 】 计算即可

代码编写思路：使用输入捕获获取高电平时间，或者使用外部中断+定时器获取高电平时间

本代码使用的是输入捕获获取高电平时间

由参考手册可知，在输入捕获模式下，，当相应的 ICx 信号检测到跳变沿后，将使用捕获/比较寄存器(TIMx_CCRx)来锁存计数器的值
所以，我们使用输入捕获触发中断可以获取到高电平的持续时间。

首先，初始化输入捕获为上升沿触发
当读取开始标志为1时，发送触发信号，开启定时器
当捕获到上升沿信号时，读取CCR1的值，然后设置为下降沿触发
之后再捕获到下降沿信号时，再次读取CCR1的值，然后设置为上降沿触发，关闭定时器，开始标志置1，计算距离

如果在高电平时间时间内，触发了更新中断，计算结果依然准确

*/

#ifdef HY_SRF05
struct
{
	uint32_t count1;	//上升沿捕获时的计数值
	uint32_t count2;	//下降沿捕获时的计数值
	uint8_t cycle;		//定时器计数溢出次数
	uint8_t start;		//读取开始标志
	uint8_t times;
	float Distance;     //距离
}Ultrasound;

void Ultrasound_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
	//GPIOF时钟初始化
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    //TIM10时钟初始化
    RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
	
	//PF4输出、挽推、100MHz、下拉
	//PF6复用、挽推、100MHz、下拉
	GPIOF->MODER    |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER6_1);
	GPIOF->OTYPER   |= 0x00;
	GPIOF->OSPEEDR  |= (GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR6);
	GPIOF->PUPDR    |= (GPIO_PUPDR_PUPDR4_1 | GPIO_PUPDR_PUPDR6_1);

	//GPIOF6复用为TIM10
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource6,GPIO_AF_TIM10);

	//TIM10时基单元初始化
	TIM10->PSC = 167;
	TIM10->ARR = 65534;
	//TIM10_CH1输入捕获初始化
	//IC1映射到TI1、不分频、不滤波
	TIM10->CCMR1 |= (TIM_CCMR1_CC1S_0 | 0x00 | 0x00);
	//使能输入捕获、上升沿触发
	TIM10->CCER  |= (TIM_CCER_CC1E | 0x00);

	//配置定时器中断
	//更新中断、捕获中断
	TIM10->DIER |= (TIM_DIER_UIE | TIM_DIER_CC1IE);

	//配置中断优先级
	NVIC_InitStruct.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	//开始标志置1
	Ultrasound.start = 1;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    // 更新中断
    if (((TIM10->SR & TIM_CR1_CEN) != 0) && ((TIM10->DIER & TIM_CR1_CEN) != 0))
    {
        Ultrasound.cycle++;
		//清除更新中断的中断标志位
		TIM10->SR &= ~TIM_SR_UIF;		
    }
    // 捕获中断
    if (((TIM10->SR & TIM_SR_CC1IF) != 0) && ((TIM10->DIER & TIM_SR_CC1IF) != 0))
    {
		
        //设置为上升沿触发时
        if (((TIM10->CCER & TIM_CCER_CC1E) != 0) && ((TIM10->CCER & TIM_CCER_CC1P) == 0))
        {
            // 读取此时TIM10_CCR1中的值
            Ultrasound.count1 = TIM10->CCR1;
			//定时器计数溢出次数清零
            Ultrasound.cycle = 0;
            // 设置为下降沿触发
            TIM10->CCER |= TIM_CCER_CC1P;
			//清除捕获中断的中断标志位
			TIM10->SR &= ~(TIM_SR_CC1IF);
        }
		
	}
    if (((TIM10->SR & TIM_SR_CC1IF) != 0) && ((TIM10->DIER & TIM_SR_CC1IF) != 0))
    {
        //设置为下降沿触发时
        if (((TIM10->CCER & TIM_CCER_CC1E) != 0) && ((TIM10->CCER & TIM_CCER_CC1P) != 0))
        {
            //读取此时TIM10_CCR1中的值
			Ultrasound.count2 = TIM10->CCR1;
			//关闭定时器
			TIM10->CR1 &= ~TIM_CR1_CEN;
			//设置为上升沿触发
			TIM10->CCER &= ~TIM_CCER_CC1P;
			//开始标志置1
			Ultrasound.start = 1;
			//计数器清零
			TIM10->CNT = 0x00;
			//计算
			//公式为(高电平时间 * 声速(340M/s))/2
			Ultrasound.Distance = (Ultrasound.count2 - Ultrasound.count1 + 
				65535 * Ultrasound.cycle)*0.017;
			//清除捕获中断的中断标志位
			TIM10->SR &= ~(TIM_SR_CC1IF);
        }
    }
}


int16_t Ultrasound_Read(void)
{
	static float old_distance;
	PFout(4) = 0;
	if (Ultrasound.start == 1)
	{
		//发送20us的高电平，表示开启超声波模块
		PFout(4) = 1;
		delay_us(20);
		PFout(4) = 0;
		//开启定时器，等待超声波模块返回高电平
		TIM10->CR1 |= TIM_CR1_CEN;
		//标志位清零
		Ultrasound.start = 0;
	}
	//防止跑飞
	if(old_distance == Ultrasound.Distance) Ultrasound.times++;	
	if(Ultrasound.times == 1)
	{
		Ultrasound.start = 1;
		Ultrasound.times = 0;
	}
	old_distance = Ultrasound.Distance;
	if(Ultrasound.Distance > 450 ) Ultrasound.Distance = 450;
	//返回值
    return Ultrasound.Distance;
}
#endif

#ifdef DFRobot_URM09
struct
{
	uint32_t count1;	//上升沿捕获时的计数值
	uint32_t count2;	//下降沿捕获时的计数值
	uint8_t cycle;		//定时器计数溢出次数
	uint8_t start;		//读取开始标志
	uint8_t times;
	float Distance;     //距离
}Ultrasound;

#define temp 20

void Ultrasound_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
	//GPIOF时钟初始化
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    //TIM10时钟初始化
    RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;

	//PF6输出、挽推、100MHz、浮空
	GPIOF->MODER    |= GPIO_MODER_MODER6_0;
	GPIOF->OTYPER   |= 0x00;
	GPIOF->OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR6;
	GPIOF->PUPDR    |= 0x00;
	
	//GPIOF6复用为TIM10
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource6,GPIO_AF_TIM10);
	
	//TIM10时基单元初始化
	TIM10->PSC = 167;
	TIM10->ARR = 65534;
	//TIM10_CH1输入捕获初始化
	//IC1映射到TI1、不分频、不滤波
	TIM10->CCMR1 |= (TIM_CCMR1_CC1S_0 | 0x00 | 0x00);
	//使能输入捕获、上升沿触发
	TIM10->CCER  |= (TIM_CCER_CC1E | 0x00);

	//配置定时器中断
	//更新中断、捕获中断
	TIM10->DIER |= (TIM_DIER_UIE | TIM_DIER_CC1IE);

	//配置中断优先级
	NVIC_InitStruct.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	//开始标志置1
	Ultrasound.start = 1;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    // 更新中断
    if (((TIM10->SR & TIM_CR1_CEN) != 0) && ((TIM10->DIER & TIM_CR1_CEN) != 0))
    {
        Ultrasound.cycle++;
		//清除更新中断的中断标志位
		TIM10->SR &= ~TIM_SR_UIF;		
    }
    // 捕获中断
    if (((TIM10->SR & TIM_SR_CC1IF) != 0) && ((TIM10->DIER & TIM_SR_CC1IF) != 0))
    {
		
        //设置为上升沿触发时
        if (((TIM10->CCER & TIM_CCER_CC1E) != 0) && ((TIM10->CCER & TIM_CCER_CC1P) == 0))
        {
            // 读取此时TIM10_CCR1中的值
            Ultrasound.count1 = TIM10->CCR1;
			//定时器计数溢出次数清零
            Ultrasound.cycle = 0;
            // 设置为下降沿触发
            TIM10->CCER |= TIM_CCER_CC1P;
			//清除捕获中断的中断标志位
			TIM10->SR &= ~(TIM_SR_CC1IF);
        }
		
	}
    if (((TIM10->SR & TIM_SR_CC1IF) != 0) && ((TIM10->DIER & TIM_SR_CC1IF) != 0))
    {
        //设置为下降沿触发时
        if (((TIM10->CCER & TIM_CCER_CC1E) != 0) && ((TIM10->CCER & TIM_CCER_CC1P) != 0))
        {
            //读取此时TIM10_CCR1中的值
			Ultrasound.count2 = TIM10->CCR1;
			//关闭定时器
			TIM10->CR1 &= ~TIM_CR1_CEN;
			//设置为上升沿触发
			TIM10->CCER &= ~TIM_CCER_CC1P;
			//开始标志置1
			Ultrasound.start = 1;
			//计数器清零
			TIM10->CNT = 0x00;
			//计算
			//HY-SRF05公式为(高电平时间 * 声速(340M/s))/2
			//DFRobot_URM09公式为(高电平时间 * 温度补偿))/2，temp为环境温度
			Ultrasound.Distance = (Ultrasound.count2 - Ultrasound.count1 + 
				65535 * Ultrasound.cycle)*((331.5 + 0.6 * (double)(temp))*100/1000000) / 2;
			//清除捕获中断的中断标志位
			TIM10->SR &= ~(TIM_SR_CC1IF);
        }
    }
}


int16_t Ultrasound_Read(void)
{
	static float old_distance;
	if (Ultrasound.start == 1)
	{
		//GPIOF6配置为输出模式
		GPIOF->MODER &= ~GPIO_MODER_MODER6;
		GPIOF->MODER |= GPIO_MODER_MODER6_0;
		PFout(6) = 0;
		delay_us(10);
		//发送20us的高电平，表示开启超声波模块
		PFout(6) = 1;
		delay_us(20);
		PFout(6) = 0;
		//开启定时器，等待超声波模块返回高电平
		TIM10->CR1 |= TIM_CR1_CEN;
		//标志位清零
		Ultrasound.start = 0;
	}
	//GPIOF6配置为复用模式
	GPIOF->MODER &= ~GPIO_MODER_MODER6;
	GPIOF->MODER |= GPIO_MODER_MODER6_1;
	//防止跑飞
	if(old_distance == Ultrasound.Distance) Ultrasound.times++;	
	if(Ultrasound.times == 1)
	{
		Ultrasound.start = 1;
		Ultrasound.times = 0;
	}
	old_distance = Ultrasound.Distance;
	if(Ultrasound.Distance > 450 ) Ultrasound.Distance = 450;
	//返回值
    return Ultrasound.Distance;
}
#endif


//PC12(UART5_TX)、PD2(UART5_RX)
#ifdef US_100

struct
{
    uint8_t Hflag;      //接收距离高位数标志位
    uint8_t Lflag;      //接收距离低位数据标志位
    uint8_t start;   	//接收完成标志位
	uint32_t Hdata;	    //距离数据高位
	uint32_t Ldata;	    //距离数据低位
	uint8_t times;
	float Distance;     //距离
}Ultrasound;

void Ultrasound_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
	float temp;
	u16 mantissa;
	u16 fraction;	   

    //引脚为：PC12(UART5_TX)、PD2(UART5_RX)
    //GPIOC、GPIOD时钟初始化
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN);
    //UART5时钟初始化
    RCC->APB1ENR |= RCC_APB1ENR_UART5EN;

    //PC12复用、挽推、100MHz、上拉
    GPIOC->MODER    |= GPIO_MODER_MODER12_1;
    GPIOC->OTYPER   |= 0x00;
    GPIOC->OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR12;
    GPIOC->PUPDR    |= GPIO_PUPDR_PUPDR12_0;
    //PD2复用、挽推、100MHz、上拉
    GPIOD->MODER    |= GPIO_MODER_MODER2_1;
    GPIOD->OTYPER   |= 0x00;
    GPIOD->OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR2;
    GPIOD->PUPDR    |= GPIO_PUPDR_PUPDR2_0;

    //GPIO复用
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5);

    //如果要修改该处的配置，要注意UART5不支持硬件流控制、0.5个停止位、1.5个停止位
    //发送接收均使能、无奇偶校验、8位字长、16倍过采样
    UART5->CR1 |= ((USART_CR1_RE | USART_CR1_TE) | 0x00 | 0x00 | 0x00);
    //1个停止位
    UART5->CR2 |= 0x00;
    //不使能硬件流控制
    UART5->CR3 |= 0x00;
    //时钟为84MHz
    //波特率9600，USARTDIV = 84 000 000 / 16 * 9600 
    //计算波特率
	temp=(float)(42*1000000)/(9600*16);//得到USARTDIV@OVER8=0
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction;
    UART5->BRR = mantissa;
    //接收缓冲区非空中断使能
    UART5->CR1 |= USART_CR1_RXNEIE;
    //配置中断优先级
	NVIC_InitStruct.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    //开启串口
    UART5->CR1 |= USART_CR1_UE;
	
	Ultrasound.start = 1;
}
void UART5_IRQHandler(void)
{
    if(UART5->SR & USART_SR_RXNE)//接收到数据
	{
        if (Ultrasound.Hflag == 1)
        {
            Ultrasound.Hdata = UART5 ->DR;
            Ultrasound.Hflag = 0;
            Ultrasound.Lflag = 1;
        }
        else if (Ultrasound.Lflag == 1)
        {
            Ultrasound.Ldata = UART5 ->DR;
            Ultrasound.Lflag = 0;
            Ultrasound.start = 1;
        }
    }
}

void UART5_Send(uint8_t data)
{
    UART5->DR = (data & (uint16_t)0x01FF);
}

int16_t Ultrasound_Read(void)
{
	static float old_distance;
    if (Ultrasound.start == 1)
    {
        Ultrasound.start = 0;
        UART5_Send(0x55);
        Ultrasound.Hflag = 1;
        Ultrasound.Lflag = 0;		
        Ultrasound.Distance = ((Ultrasound.Hdata << 8) | Ultrasound.Ldata)/10;
    }
	//防止跑飞
	if(old_distance == Ultrasound.Distance) Ultrasound.times++;	
	if(Ultrasound.times == 1)
	{
		Ultrasound.start = 1;
		Ultrasound.times = 0;
	}
	old_distance = Ultrasound.Distance;
	if(Ultrasound.Distance > 450 ) Ultrasound.Distance = 450;
	//返回值
    return Ultrasound.Distance;
}

#endif
