#ifndef __PID_H
#define __PID_H

#include "sys.h"

typedef struct 
{
    float Kp;
    float Ki;
    float Kd;
    float PID_Out;
    float SetPoint;  // 设定点
    float PrevError; // 上一次的偏差
    float SumError;  // 误差的积分
    float LsatError; // 最后的偏差
} PID_TypeDef;

extern PID_TypeDef PID_Left,PID_Right;

float Positional_PID_Contorl(PID_TypeDef *PID,float current_value);
float Incremental_PID_Contorl(PID_TypeDef *PID,float current_value);
void PID_Encoders_SetPoint(float enA_Speed,float enB_Speed);
void PID_Init(void);
void PID_Release(void);

void PID_modify(PID_TypeDef *PID, float Kp, float Ki, float Kd);
void SetPoint_modify(PID_TypeDef *PID,float SetPoint);

void Limit_float_Value(float *num, float Limit_Min, float Limit_Max);
void Limit_double_Value(double *num, double Limit_Min, double Limit_Max);
void Limit_int32_t_Value(int32_t *num, int32_t Limit_Min, int32_t Limit_Max);

#endif
