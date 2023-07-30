#ifndef __TIMER_H
#define __TIMER_H

#include "sys.h"

extern volatile uint8_t timeout;

void TIM6_init(void);
void set_time(uint16_t time);

#endif
