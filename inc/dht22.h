/**
  ******************************************************************************
  * @file    dht22.h 
  * @author  Joe Todd
  * @version 
  * @date    
  * @brief   Header for dht22.c
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DHT22_H__
#define __DHT22_H__

#include <inttypes.h>

extern uint16_t dht22_get_humidity(void);
extern int16_t dht22_get_temp(void);
extern uint8_t dht22_read(void);
extern uint8_t dht22_check_checksum(void);
extern uint8_t dht22_get_checksum(void);

#endif
