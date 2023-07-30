#include "pid.h"

/////////////////////////////////////////////λ��ʽPID/////////////////////////////////////////////
/****
 * λ��ʽPID����
 * ��ڲ�������ǰ����ֵ
****/
float Positional_PID_Contorl(PID_TypeDef *PID,float current_value)
{
    float Error;    //ƫ��
    Error = current_value - PID->SetPoint;  //ƫ�� = ��ǰֵ - �趨ֵ
    PID->SumError += Error;                 //ƫ��Ļ���
	Limit_float_Value(&(PID->SumError),-3000,3000);	//�����޷���������Ҫ����
    PID->PID_Out =	//����PID
            PID->Kp * Error +                   //���������
            PID->Ki * PID->SumError +           //���������
            PID->Kd * (Error - PID->PrevError); //΢�������
    PID->PrevError = Error;                 //����ƫ��ֵ            
    return PID->PID_Out;
}

/////////////////////////////////////////////����ʽPID/////////////////////////////////////////////
/****
 * ����ʽPID����
 * ��ڲ�������ǰ����ֵ
****/
float Incremental_PID_Contorl(PID_TypeDef *PID,float current_value)
{
    float Error;//ƫ��
    Error = current_value - PID->SetPoint;  //ƫ�� = ��ǰֵ - �趨ֵ
    PID->PID_Out =  //����PID
        PID->Kp * (Error - PID->PrevError) +                    //���������
        PID->Ki * Error +                                       //���������
        PID->Kd * (Error - 2*PID->PrevError + PID->LsatError)   //΢�������
    + PID->PID_Out;
    PID->LsatError = PID->PrevError;    //�洢���ϴε�ƫ��
    PID->PrevError = Error;             //�洢�ϴε�ƫ��
    return PID->PID_Out;
}

/****
 * �޸ı�������Ŀ���ٶ�
 * ��ڲ�����������A��Ŀ���ٶȣ�������B��Ŀ���ٶ�
****/
void PID_Encoders_SetPoint(float Left_Wheel_speed, float Right_Wheel_speed)
{
    PID_Left.SetPoint = Left_Wheel_speed;
    PID_Right.SetPoint = Right_Wheel_speed;
}

/*******************************************PID������ʼ��*******************************************/
void PID_Init(void)
{

    //���A��B����������PID
    PID_Left.Kp = PID_Right.Kp = -600;
    PID_Left.Ki = PID_Right.Ki = -3;
    PID_Left.Kd = PID_Right.Kd = -30;
    //������A��B��Ŀ���ٶ�Ϊ0
    PID_Encoders_SetPoint(0,0);
}
void PID_Release(void)
{
	//ѭ��PID
}
/*******************************************PID����*******************************************/
/****
 * �޸�ĳ��PID�ı���ϵ��������ϵ����΢��ϵ��
 * ��ڲ�����PID������ϵ��������ϵ����΢��ϵ��
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

/*******************************************����*******************************************/
/******
 * �޷����������Լ�ȷ��
 * *num��ָ�������ָ��
 * Limit_Min����Сֵ
 * Limit_Max�����ֵ
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




