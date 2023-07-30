#ifndef __KEY_H
#define __KEY_H
#include "sys.h"

/*���淽ʽ��ͨ��λ��������ʽ��ȡIO*/
#define KEY0    PEin(4) // PE4
#define KEY1    PEin(3) // PE3
#define WK_UP   PAin(0) // PA0

#define KEY0_PRES 1 // KEY0����
#define KEY1_PRES 2 // KEY1����
#define WKUP_PRES 3 // KEY_UP����(��WK_UP)

void KEY_Init(void);  // ����IO��ʼ��
u8 KEY_Scan(u8 mode); // ����ɨ�躯��
uint8_t KEY_Scan_(void);
#endif
