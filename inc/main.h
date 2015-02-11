/**
  ******************************************************************************
  * @file    main.h 
  * @author  Joe Todd
  * @version 
  * @date    
  * @brief   Header for main.c
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

/* Define to enable different elements ---------------------------------------*/
/*
 * Enable debugging info via uart to usb dongle.
 */
// #define DEBUG

/* 
 * Enable temp/humidity sensor.
*/
#define TH_SENSOR

/* 
 * Enable WiFi 
 */
#define WIFI_EN

/* Global Definitions -------------------------------------------------------*/
#ifdef WIFI_EN
extern void handleRxSNIC(uint8_t *buf, uint16_t len);
extern void handleRxWiFi(uint8_t *buf, uint16_t len);
#endif

#endif