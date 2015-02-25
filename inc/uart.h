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
#include "utl.h"

#define USART1_TX_PIN   6u   /* PB6 */
#define USART1_RX_PIN   7u   /* PB7 */

#define USART2_TX_PIN   2u   /* PA2 */
#define USART2_RX_PIN   3u   /* PA3 */

#define USART1_AF       7u   /* AF7 */

#define PCLK2       16000000u

/* Function Declarations ----------------------------------------------------*/
#define RX_BUFFER_SIZE 4096u

typedef void (*uart_recv_callback_fn) (uint8_t bytes);

extern void uart_init(uint32_t baudrate);
extern void dbg_uart_init(uint32_t baudrate);
extern void dbg_uart_puts(const char *s);
extern void uart_send_data(char *buf, uint32_t len);
extern bool uart_buffer_empty(void);
extern char uart_read_byte(void);

#endif