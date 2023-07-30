#ifndef __KEY_H
#define __KEY_H
#include "sys.h"

/*下面方式是通过位带操作方式读取IO*/
#define KEY0    PEin(4) // PE4
#define KEY1    PEin(3) // PE3
#define WK_UP   PAin(0) // PA0

#define KEY0_PRES 1 // KEY0按下
#define KEY1_PRES 2 // KEY1按下
#define WKUP_PRES 3 // KEY_UP按下(即WK_UP)

void KEY_Init(void);  // 按键IO初始化
u8 KEY_Scan(u8 mode); // 按键扫描函数
uint8_t KEY_Scan_(void);
#endif
