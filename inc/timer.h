/**
  ******************************************************************************
  * @file    timer.h 
  * @author  Joe Todd
  * @version 
  * @date    
  * @brief   Header for timer.c
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TIMER_H
#define TIMER_H

#include "stm32f4xx.h"
#include <stdint.h>
#include "utl.h"

extern void timer_delay(__IO uint32_t time);
extern uint32_t timer_get(void);
extern void timer_init(void);
extern void timer_start(void);

#endif