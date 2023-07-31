#ifndef __CONTROL_H
#define __CONTROL_H

#include "sys.h"

/*****************数据采集*****************/
//通过编码器获取的速度
extern int16_t Left_Wheel_speed,Right_Wheel_speed;
//OpenMV传来的数据
extern uint16_t Object_Center_X,Object_Center_Y;	//使用OpenMV控制云台接收的数据
extern uint16_t Angle_Left,Angle_Right;				//使用OpenMV巡线接收的数据
//MPU6050获取的值
extern float Pitch,Roll,Yaw;    //角度
extern short gyrox,gyroy,gyroz;	//角速度
extern short aacx,aacy,aacz;    //加速度
/*****************控制量*****************/
//控速
extern int32_t Left_Wheel_PWM,Right_Wheel_PWM;
//巡线(OpenMV巡线或灰度巡线PID计算结果)
extern float LinePatrol_Control;
//物体追踪得到的舵机角度
extern float Top_Track_Angle,End_Track_Angle;

void TIM9_Timed_Interrupt(uint32_t PSC,uint32_t ARR);
void Mode_Select(uint8_t mode,double speed);

void mode1_Control(double speed);
void mode2_Control(double speed);
void mode3_Control(double speed);
void mode4_Control(double speed);

#endif
