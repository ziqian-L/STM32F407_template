#ifndef __LED_H
#define __LED_H

#include "sys.h"

//LED 端口定义
#define LED0 PFout(9)
#define LED1 PFout(10)

void LED_Init(void); //初始化
#endif
