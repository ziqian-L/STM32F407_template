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
 * ���ƾ����ж�����ɣ�ʹ��TIM9��Ϊ��ʱ�ж�
********************************************************************/
//PID����
PID_TypeDef PID_Left,PID_Right;

//ͨ����������ȡ���ٶ�
int16_t Left_Wheel_speed,Right_Wheel_speed;
int32_t Left_Wheel_PWM,Right_Wheel_PWM;

//MPU6050��ȡ��ֵ
float Pitch,Roll,Yaw;		//�Ƕ�

/*****
 * ��ڲ�����
 *  PSC��Ԥ��Ƶϵ��
 *  ARR���Զ���װ��ֵ
*****/
void TIM9_Timed_Interrupt(uint32_t PSC,uint32_t ARR)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    //��ʼ��TIM9ʱ��
    RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;

    //ʱ����Ԫ��ʼ��
    //���ϼ���ģʽ
    TIM9->CR1 |= 0x00;
    TIM9->PSC = PSC;
    TIM9->ARR = ARR;

    //�ж����ȼ�����
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //ʹ�ܶ�ʱ���ж�
    TIM_ITConfig(TIM9,TIM_IT_Update,ENABLE);
	
	//����TIM9
	TIM_Cmd(TIM9,ENABLE);
}



/*****
 * TIM9��ʱ�жϷ�����
*****/
void TIM1_BRK_TIM9_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM9,TIM_IT_Update) != RESET)
    {	
		/*********************����������*********************/

		/*1.��ʱ��ȡ��������MPU6050��ֵ*/
        Left_Wheel_speed = Read_Speed(3);
        Right_Wheel_speed = Read_Speed(4);
        mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
        /*2.PID����*/
		Left_Wheel_PWM = Positional_PID_Contorl(&PID_Left,Left_Wheel_speed);
        Right_Wheel_PWM = Positional_PID_Contorl(&PID_Right,Right_Wheel_speed);
        /*3.PWM���*/
        PWM_Limit(&Left_Wheel_PWM,&Right_Wheel_PWM);
        Load_PWM(Left_Wheel_PWM,Right_Wheel_PWM);
    }
    TIM_ClearFlag(TIM9,TIM_IT_Update);
	GraySensor_LinePatrol();
	OLED_ShowFloat(0,16,Ultrasound_Read(),6,16,1);
}


/*****
 * ����1�жϷ�����
*****/
void USART1_IRQHandler(void)
{
    /* �ֲ���̬���������ջ��� */
    static u8 RxBuffer[10];
    /* ���ݳ��� *//* ���������±� */
    static u8  data_cnt = 0;
    /* ����״̬ */
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
 * ����2�жϷ�����
*****/
void USART2_IRQHandler(void)
{
	uint8_t data;
	//��ȡSR�Ĵ�����DR�Ĵ������жϱ�־λ����
    if (USART_GetITStatus(USART2,USART_IT_IDLE) != RESET)
    {
        data = USART_ReceiveData(USART2);
    }
    else if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        data = USART_ReceiveData(USART2);
        if(Write_RingBuff(&Uart2_RingBuff, data) == RINGBUFF_ERR){//������������
            LED1=0;
        }else{
            LED1=1;
        }
    }

}



