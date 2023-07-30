#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"

#define MotorA_IN1 PGout(5)
#define MotorA_IN2 PGout(7)

#define MotorB_IN3 PGout(6)
#define MotorB_IN4 PGout(8)

void Motor_drive_Init(void);
void TIM8_PWM_Init(uint16_t PSC,uint16_t ARR);

void PWM_Limit(int32_t *motorA,int32_t *motorB);
int32_t my_abs(int32_t p);
void Load_PWM(int32_t motorA,int32_t motorB);
void Stop_PWM(void);

#endif
