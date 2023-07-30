#ifndef __CONTROL_H
#define __CONTROL_H

#include "sys.h"

extern float Pitch,Roll,Yaw;    //角度
extern short gyrox,gyroy,gyroz;	//角速度
extern short aacx,aacy,aacz;    //加速度
extern int16_t Left_Wheel_speed,Right_Wheel_speed;
extern int32_t Left_Wheel_PWM,Right_Wheel_PWM;

void TIM9_Timed_Interrupt(uint32_t PSC,uint32_t ARR);
void Mode_Select(uint8_t mode,double speed);

void mode1_Control(double speed);
void mode2_Control(double speed);
void mode3_Control(double speed);
void mode4_Control(double speed);

#endif
