/**
  ******************************************************************************
  * @file    uart.h 
  * @author  Joe Todd
  * @version 
  * @date    
  * @brief   Header for uart.c
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UART_H
#define UART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "iox.h"

#define SOM_CHAR 0x02
#define EOM_CHAR 0x04
#define ESC_CHAR 0x10

#define ACK_REQUIRED 1
#define ACK_NOT_REQUIRED 0

/* Definitions ---------------------------------------------------------------*/
#define RX_BUFFER_SIZE 4096


extern void uart_init(uint32_t baudrate);
extern void uart_send_data(unsigned char *buf, uint32_t len);
extern bool uart_buffer_empty(void);
extern uint8_t uart_read_byte(void);

#endif