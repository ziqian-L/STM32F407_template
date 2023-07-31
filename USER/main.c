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
#include "matrixkey.h"
#include "mlx90614.h"
/*****
 * 左电机：
 *      编码器：TIM3_CH1(PB4)、TIM3_CH2(PB5)
 *      PWM输出：TIM8_CH1(PC6)
 *      正反转：PG7、PG5
 * 右电机：
 *      编码器：TIM4_CH1(PB6)、TIM4_CH2(PB7)
 *      PWM输出：TIM8_CH2(PC7)
 *      正反转：PG8、PG6
 * 蜂鸣器：PG1
 * OLED：Back接法
 * 舵机/二维云台：
 * 		顶舵机：TIM12_CH1(PB14)
 * 		底舵机：TIM12_CH2(PB15)
 * 超声波：PC12(UART5_TX)、PD2(UART5_RX)
 * 感为8路灰度：带5V容忍的PF0(SDA)、PF1(SCL)
 * 蓝牙/2.4G：PA2(USART2_TX)、PA3(USART2_RX)
 * MPU6050：PC1(SCL)、PC2(SDA)
 * CH451L：PE13(DOUT)、PE11(LOAD)、PE9(DIN)、PE7(DCLK)
 * MLX90614：PE14(SDA)、PE15(SCL)*不可用,和OLED的IO口冲突
*****/

int main(void)
{
	static uint8_t status=0;
	/*系统初始化*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断优先级分组
    delay_init(168);//延时初始化
	USART1_Init(115200);//串口1
	USART2_Init(115200);//串口2
	RingBuff_Init(&Uart2_RingBuff);//串口2环形缓冲区
	/*板载外设初始化*/
	LED_Init();
	KEY_Init();
	/*模块初始化*/
	Buzz_Init();//蜂鸣器
	Buzz(1);//关闭蜂鸣器
	OLED_Init();//OLED初始化
	Servos_Init();//舵机
	Ultrasound_Init();//超声波
	GraySensor_Init();//灰度传感器
//	MPU_Init();//MPU6050初始化
//	DMP_Init();//DMP初始化
	Matrix_Key_Init();//CH451L矩阵键盘
	Temperature_measure_Init();//MLX90614
	/*控制初始化*/
	PID_Init();
	Motor_drive_Init();//正反转
	TIM8_PWM_Init(168-1,7200-1);//PWM输出
	TIM3_Encoder_Init();//左电机编码器
	TIM4_Encoder_Init();//右电机编码器
	TIM9_Timed_Interrupt(168-1,5000);//定时中断
	TIM6_init();//定时器延时初始化
    PID_Encoders_SetPoint(0,0);
	gimbal_angle(90,90);
	delay_ms(300);
	while(1)
	{
//		OLED_Drawsqrt(25,55,33,63,gray_sensor[7]);
//		OLED_Drawsqrt(35,55,43,63,gray_sensor[6]);
//		OLED_Drawsqrt(45,55,53,63,gray_sensor[5]);
//		OLED_Drawsqrt(55,55,63,63,gray_sensor[4]);
//		OLED_Drawsqrt(65,55,73,63,gray_sensor[3]);
//		OLED_Drawsqrt(75,55,83,63,gray_sensor[2]);
//		OLED_Drawsqrt(85,55,93,63,gray_sensor[1]);
//		OLED_Drawsqrt(95,55,103,63,gray_sensor[0]);
//		OLED_ShowNum(64,16,KEY,2,16,1);
//		OLED_ShowFNum(0,0,Left_Wheel_speed,5,16,1);
//		OLED_ShowFNum(64,0,Right_Wheel_speed,5,16,1);
		OLED_ShowSNum(0,32,Object_Center_X,4,16,1);
		OLED_ShowSNum(64,32,Object_Center_Y,4,16,1);
		OLED_ShowFloat(0,48,Last_Top_Track_Angle,7,16,1);
		OLED_ShowFloat(64,48,Last_End_Track_Angle,7,16,1);
		OLED_Refresh();
//		//LED交替闪烁
//		if(status==0)
//		{
//			LED0=0;
//			set_time(200);
//			status=1;		
//		}
//		else if(timeout==1&&status==1)
//		{
//			set_time(200);
//			status=2;
//		}
//		else if(timeout==0&&status==2)
//		{
//			LED0=1;
//		}
//		else if(timeout==1&&status==2)
//		{
//			status=0;
//		}
	}
}







