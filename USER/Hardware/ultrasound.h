#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "sys.h"

/*使用哪个就取消哪个的注释,并将另外两个注释掉*/
//#define HY_SRF05
//#define DFRobot_URM09
#define US_100


void Ultrasound_Init(void);
int16_t Ultrasound_Read(void);
#ifdef US_100
void UART5_Send(uint8_t data);
#endif

#endif
