#ifndef __SERVOS_H
#define __SERVOS_H

#include "sys.h"

//舵机结构体
typedef struct{
	uint8_t TIM_CH;			//定时器通道
	uint16_t PWM_Limit_Min;	//舵机最小PWM值
	uint16_t PWM_Limit_Max;	//舵机最大PWM值
	uint16_t PWM_Angle_Min;	//舵机0度PWM值
	uint16_t PWM_Angle_Max;	//舵机180/270度时PWM值
	uint16_t Angle;			//舵机标准角度范围180/270
	uint16_t Angle_MIN;		//限幅最小角度值
	uint16_t Angle_MAX;		//限幅最大角度值
}Server_Typedef;

void Servos_Init(void);
void servos_angle_limit(Server_Typedef* servos,float *angle);
void servos_angle(Server_Typedef* servos,TIM_TypeDef* TIMx,float angle);
void Servos1_Angle(float angle);
void Servos2_Angle(float angle);
void gimbal_angle(float Server1_angle,float Server2_angle);
#endif
