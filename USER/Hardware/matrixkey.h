#ifndef __MATRIXKEY_H
#define __MATRIXKEY_H

#include "sys.h"

extern uint8_t KEY;

#define DOUT_Read   PEin(13)
#define LOAD    PEout(11)
#define DIN     PEout(9)
#define DCLK    PEout(7)

void Matrix_Key_Init(void);
void Matrix_Key_Writer(uint16_t command);
void Matrix_Key_Data(uint8_t *key);


#endif
