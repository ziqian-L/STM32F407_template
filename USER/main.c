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
 * ������
 *      ��������TIM3_CH1(PB4)��TIM3_CH2(PB5)
 *      PWM�����TIM8_CH1(PC6)
 *      ����ת��PG7��PG5
 * �ҵ����
 *      ��������TIM4_CH1(PB6)��TIM4_CH2(PB7)
 *      PWM�����TIM8_CH2(PC7)
 *      ����ת��PG8��PG6
 * ��������PE10
 * OLED��Back�ӷ�
 * ���/��ά��̨��
 * 		���1��TIM12_CH1(PB14)
 * 		���2��TIM12_CH2(PB15)
 * ��������PC12(UART5_TX)��PD2(UART5_RX)
 * ��Ϊ8·�Ҷȣ���5V���̵�PF0(SDA)��PF1(SCL)
 * ����/2.4G��PA2(USART2_TX)��PA3(USART2_RX)
 * MPU6050��PC1(SCL)��PC2(SDA)
 * CH451L��PE13(DOUT)��PE11(LOAD)��PE9(DIN)��PE7(DCLK)
 * MLX90614��PE14(SDA)��PE15(SCL)*������
*****/

int main(void)
{
	static uint8_t status=0;
	uint8_t i,j;
	/*ϵͳ��ʼ��*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�ж����ȼ�����
    delay_init(168);//��ʱ��ʼ��
	USART1_Init(115200);//����1
	USART2_Init(115200);//����2
	RingBuff_Init(&Uart2_RingBuff);//����2���λ�����
	/*���������ʼ��*/
	LED_Init();
	KEY_Init();
	/*ģ���ʼ��*/
	Buzz_Init();//������
	Buzz(1);//�رշ�����
	OLED_Init();//OLED��ʼ��
	Servos_Init();//���
	Ultrasound_Init();//������
	GraySensor_Init();//�Ҷȴ�����
//	MPU_Init();//MPU6050��ʼ��
//	DMP_Init();//DMP��ʼ��
	Matrix_Key_Init();//CH451L�������
	Temperature_measure_Init();//MLX90614
	/*���Ƴ�ʼ��*/
	PID_Init();
	Motor_drive_Init();//����ת
	TIM8_PWM_Init(168-1,7200-1);//PWM���
	TIM3_Encoder_Init();//����������
	TIM4_Encoder_Init();//�ҵ��������
//	TIM9_Timed_Interrupt(168-1,5000);//��ʱ�ж�
	TIM6_init();//��ʱ����ʱ��ʼ��
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
//		OLED_Refresh();
//		//LED������˸
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







