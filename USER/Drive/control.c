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
#include "ultrasound.h"
#include "graysensor.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "matrixkey.h"
#include "mlx90614.h"

/********************************************************************
 * 控制均在中断里完成，使用TIM9作为定时中断
********************************************************************/
//PID控制
PID_TypeDef PID_Left,PID_Right;

//通过编码器获取的速度
int16_t Left_Wheel_speed,Right_Wheel_speed;
int32_t Left_Wheel_PWM,Right_Wheel_PWM;

//MPU6050获取的值
float Pitch,Roll,Yaw;		//角度

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
    if (TIM_GetITStatus(TIM9,TIM_IT_Update) != RESET)
    {	
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
    }
    TIM_ClearFlag(TIM9,TIM_IT_Update);
	GraySensor_LinePatrol();
	OLED_ShowFloat(0,16,Ultrasound_Read(),6,16,1);
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



