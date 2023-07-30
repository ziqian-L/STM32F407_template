#ifndef __BUZZ_H
#define __BUZZ_H

#include "sys.h"

#define Buzz_PG1 PGout(1)

void Buzz_Init(void);
void Buzz(uint8_t flag);

#endif
