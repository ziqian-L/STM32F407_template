#include "pid.h"

/////////////////////////////////////////////位置式PID/////////////////////////////////////////////
/****
 * 位置式PID控制
 * 入口参数：当前测量值
****/
float Positional_PID_Contorl(PID_TypeDef *PID,float current_value)
{
    float Error;    //偏差
    Error = current_value - PID->SetPoint;  //偏差 = 当前值 - 设定值
    PID->SumError += Error;                 //偏差的积分
	Limit_float_Value(&(PID->SumError),-3000,3000);	//积分限幅，根据需要调整
    PID->PID_Out =	//计算PID
            PID->Kp * Error +                   //比例项计算
            PID->Ki * PID->SumError +           //积分项计算
            PID->Kd * (Error - PID->PrevError); //微分项计算
    PID->PrevError = Error;                 //储存偏差值            
    return PID->PID_Out;
}

/////////////////////////////////////////////增量式PID/////////////////////////////////////////////
/****
 * 增量式PID控制
 * 入口参数：当前测量值
****/
float Incremental_PID_Contorl(PID_TypeDef *PID,float current_value)
{
    float Error;//偏差
    Error = current_value - PID->SetPoint;  //偏差 = 当前值 - 设定值
    PID->PID_Out =  //计算PID
        PID->Kp * (Error - PID->PrevError) +                    //比例项计算
        PID->Ki * Error +                                       //积分项计算
        PID->Kd * (Error - 2*PID->PrevError + PID->LsatError)   //微分项计算
    + PID->PID_Out;
    PID->LsatError = PID->PrevError;    //存储上上次的偏差
    PID->PrevError = Error;             //存储上次的偏差
    return PID->PID_Out;
}

/****
 * 修改编码器的目标速度
 * 入口参数：编码器A的目标速度，编码器B的目标速度
****/
void PID_Encoders_SetPoint(float Left_Wheel_speed, float Right_Wheel_speed)
{
    PID_Left.SetPoint = Left_Wheel_speed;
    PID_Right.SetPoint = Right_Wheel_speed;
}

/*******************************************PID参数初始化*******************************************/
void PID_Init(void)
{

    //电机A、B编码器控速PID
    PID_Left.Kp = PID_Right.Kp = -600;
    PID_Left.Ki = PID_Right.Ki = -3;
    PID_Left.Kd = PID_Right.Kd = -30;
    //编码器A、B的目标速度为0
    PID_Encoders_SetPoint(0,0);
}
void PID_Release(void)
{
	//循迹PID
}
/*******************************************PID参数*******************************************/
/****
 * 修改某个PID的比例系数，积分系数，微分系数
 * 入口参数：PID，比例系数，积分系数，微分系数
****/
void PID_modify(PID_TypeDef *PID, float Kp, float Ki, float Kd)
{
    PID->Kp = Kp;
    PID->Ki = Ki;
    PID->Kd = Kd;
}
void SetPoint_modify(PID_TypeDef *PID,float SetPoint)
{
    PID->SetPoint = SetPoint;
}

/*******************************************其他*******************************************/
/******
 * 限幅，幅度由自己确定
 * *num：指向变量的指针
 * Limit_Min：最小值
 * Limit_Max：最大值
*****/
void Limit_float_Value(float *num, float Limit_Min, float Limit_Max)
{
    *num = (*num < Limit_Min) ? Limit_Min : *num;
    *num = (*num > Limit_Max) ? Limit_Max : *num;
}
void Limit_double_Value(double *num, double Limit_Min, double Limit_Max)
{
    *num = (*num < Limit_Min) ? Limit_Min : *num;
    *num = (*num > Limit_Max) ? Limit_Max : *num;
}
void Limit_int32_t_Value(int32_t *num, int32_t Limit_Min, int32_t Limit_Max)
{
    *num = (*num < Limit_Min) ? Limit_Min : *num;
    *num = (*num > Limit_Max) ? Limit_Max : *num;
}




