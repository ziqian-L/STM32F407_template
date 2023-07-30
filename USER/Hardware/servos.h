#ifndef __SERVOS_H
#define __SERVOS_H

#include "sys.h"

//����ṹ��
typedef struct{
	uint8_t TIM_CH;			//��ʱ��ͨ��
	uint16_t PWM_Limit_Min;	//�����СPWMֵ
	uint16_t PWM_Limit_Max;	//������PWMֵ
	uint16_t PWM_Angle_Min;	//���0��PWMֵ
	uint16_t PWM_Angle_Max;	//���180/270��ʱPWMֵ
	uint16_t Angle;			//�����׼�Ƕȷ�Χ180/270
	uint16_t Angle_MIN;		//�޷���С�Ƕ�ֵ
	uint16_t Angle_MAX;		//�޷����Ƕ�ֵ
}Server_Typedef;

void Servos_Init(void);
void servos_angle_limit(Server_Typedef* servos,float *angle);
void servos_angle(Server_Typedef* servos,TIM_TypeDef* TIMx,float angle);
void Servos1_Angle(float angle);
void Servos2_Angle(float angle);
void gimbal_angle(float Server1_angle,float Server2_angle);
#endif
