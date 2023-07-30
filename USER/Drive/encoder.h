#ifndef __ENCODER_H
#define __ENCODER_H

#include "sys.h"

void TIM3_Encoder_Init(void);
void TIM4_Encoder_Init(void);

int Read_Speed(int TIMx);

#endif
