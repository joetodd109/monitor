#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

void SysTick_Configuration(void);

void mdelay(uint32_t ms);
extern void mydelay(uint32_t ms);

#endif
