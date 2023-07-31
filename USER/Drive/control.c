#include "delay.h"
#include "usart.h"
#include "motor.h"
#include "encoder.h"
#include "control.h"
#include "pid.h"
#include "timer.h"
#include "led.h"
#include "key.h"
#include "buzz.h"
#include "oled.h"
#include "servos.h"
#include "ultrasound.h"
#include "graysensor.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "matrixkey.h"
#include "mlx90614.h"

/********************************************************************
 * 控制均在中断里完成，使用TIM9作为定时中断
********************************************************************/
//PID控制，左电机PID、右电机PID、巡线PID、顶舵机PID、底舵机PID
PID_TypeDef PID_Left,PID_Right,PID_LinePatrol,PID_Servos_Top,PID_Servos_End;

/*****************数据采集*****************/
//通过编码器获取的速度
int16_t Left_Wheel_speed,Right_Wheel_speed;
//OpenMV传来的数据
uint16_t Object_Center_X,Object_Center_Y;	//使用OpenMV控制云台接收的数据
uint16_t Angle_Left,Angle_Right;			//使用OpenMV巡线接收的数据
//MPU6050获取的值
float Pitch,Roll,Yaw;		//角度
short gyrox,gyroy,gyroz;	//角速度
short aacx,aacy,aacz;		//加速度
/*****************控制量*****************/
//控速
int32_t Left_Wheel_PWM,Right_Wheel_PWM;
//巡线(OpenMV巡线或灰度巡线PID计算结果)
float LinePatrol_Control;
//物体追踪得到的舵机角度变化量
float Top_Track_Angle,End_Track_Angle;
//上一次的角度
float Last_Top_Track_Angle=90,Last_End_Track_Angle=90;
/*****
 * 入口参数：
 *  PSC：预分频系数
 *  ARR：自动重装载值
*****/
void TIM9_Timed_Interrupt(uint32_t PSC,uint32_t ARR)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    //初始化TIM9时钟
    RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;

    //时基单元初始化
    //向上计数模式
    TIM9->CR1 |= 0x00;
    TIM9->PSC = PSC;
    TIM9->ARR = ARR;

    //中断优先级配置
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //使能定时器中断
    TIM_ITConfig(TIM9,TIM_IT_Update,ENABLE);
	
	//开启TIM9
	TIM_Cmd(TIM9,ENABLE);
}



/*****
 * TIM9定时中断服务函数
*****/
void TIM1_BRK_TIM9_IRQHandler(void)
{
	static uint8_t Time_Interval = 0;
    if (TIM_GetITStatus(TIM9,TIM_IT_Update) != RESET)
    {	
		/**********************循迹控制*********************/
		/*灰度循迹控制*/
		LinePatrol_Control = GraySensor_LinePatrol();		
		/*OpenMV循迹控制*/
		
		/*********************编码器控速*********************/
		/*1.定时读取编码器、MPU6050的值*/
        Left_Wheel_speed = Read_Speed(3);
        Right_Wheel_speed = Read_Speed(4);
        mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
        /*2.PID控制*/
		Left_Wheel_PWM = Positional_PID_Contorl(&PID_Left,Left_Wheel_speed);
        Right_Wheel_PWM = Positional_PID_Contorl(&PID_Right,Right_Wheel_speed);
        /*3.PWM输出*/
        PWM_Limit(&Left_Wheel_PWM,&Right_Wheel_PWM);
        Load_PWM(Left_Wheel_PWM,Right_Wheel_PWM);
		/**********************云台追踪*********************/
		/*******************50ms控制一次******************/
		Time_Interval++;
		if(Time_Interval==10)
		{
			Time_Interval = 0;
			/*1.将获取到的物体的X、Y坐标进行PID解算*/
			//Y坐标用于控制顶部的舵机
			Top_Track_Angle = Incremental_PID_Contorl(&PID_Servos_Top,Object_Center_Y);
			//X坐标用于控制底部的舵机
			End_Track_Angle = Incremental_PID_Contorl(&PID_Servos_End,Object_Center_X);
			/*2.角度输出*/
			gimbal_angle(Last_Top_Track_Angle + Top_Track_Angle,Last_End_Track_Angle+End_Track_Angle);
			Last_Top_Track_Angle = Last_Top_Track_Angle + Top_Track_Angle;
			Last_End_Track_Angle = Last_End_Track_Angle + End_Track_Angle;
			/*3.处理数据*/
			Top_Track_Angle=0;
			End_Track_Angle=0;
		}
    }
    TIM_ClearFlag(TIM9,TIM_IT_Update);
//	OLED_ShowFloat(0,16,Ultrasound_Read(),6,16,1);
}


/*****
 * 串口1中断服务函数
*****/
void USART1_IRQHandler(void)
{
    /* 局部静态变量：接收缓存 */
    static u8 RxBuffer[10];
    /* 数据长度 *//* 数据数组下标 */
    static u8  data_cnt = 0;
    /* 接收状态 */
    static u8 state = 0;
	uint8_t data;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		data = USART_ReceiveData(USART1);
		if(state==0&&data==0xAA&&data_cnt==0)state=1;
		else if(state==1)
		{
			RxBuffer[data_cnt++]=data;
			if(data_cnt>4)
			{
				state = 2;
			}
		}
		else if(state==2&&data==0xFF)
		{
			state = 1;
			data_cnt=0;
			Object_Center_X = RxBuffer[1];
			Object_Center_Y = RxBuffer[2];
		}
	}
}

/*****
 * 串口2中断服务函数
*****/
void USART2_IRQHandler(void)
{
	uint8_t data;
	//读取SR寄存器和DR寄存器后，中断标志位清零
    if (USART_GetITStatus(USART2,USART_IT_IDLE) != RESET)
    {
        data = USART_ReceiveData(USART2);
    }
    else if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        data = USART_ReceiveData(USART2);
        if(Write_RingBuff(&Uart2_RingBuff, data) == RINGBUFF_ERR){//缓冲区满灯亮
            LED1=0;
        }else{
            LED1=1;
        }
    }

}



