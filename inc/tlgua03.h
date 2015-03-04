/**
  ******************************************************************************
  * @file    wifi.h 
  * @author  Joe Todd
  * @version 
  * @date    
  * @brief   Header for wifi.c
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef WIFI_H
#define WIFI_H

/* Includes ------------------------------------------------------------------*/
#include "iox.h"
#include "utl.h"
#include "uart.h"
#include "timer.h"

#define CMD_FORMAT_ERR      -1
#define NOT_SUPPORT_ERR     -2
#define OP_SYMBOL_ERR       -3
#define PARAM_ERR           -4
#define NO_PERMIT_ERR       -5
#define MEM_LACK_ERR        -6
#define FLASH_ERR           -7
#define JOIN_ERR            -10
#define NO_SOCK_ERR         -11
#define SOCK_ERR            -12
#define SOCK_CONN_ERR       -13
#define UNDEFINED_ERR       -100

#define TX_BUF_SIZE         4096
#define RX_BUF_SIZE         4096

/* Public typedefs -----------------------------------------------------------*/

typedef enum {
    encrypt_open,
    encrypt_wep64,
    encrypt_wep128,
    encrypt_wpa_tkip,
    encrypt_wpa_aes,
    encrypt_wpa2_tkip,
    encrypt_wpa2_aes,
} encrypt_mode_t;

typedef enum {
    format_hex,
    format_ascii,
} key_format_t;

typedef enum {
    uart_data_8bits,
    uart_data_7bits,
} uart_data_t;

typedef enum {
    uart_stop_1bit,
    uart_stop_none,
    uart_stop_2bits,
} uart_stop_t;

typedef enum {
    uart_parity_none,
    uart_parity_odd,
    uart_parity_even,
} uart_parity_t;

/* Function Declarations -----------------------------------------------------*/

extern bool wifi_test(void);
extern bool wifi_reset(void);

#if 0 
extern bool wifi_join(char *rsp);
extern bool wifi_disconnect(void);
extern bool wifi_scan(char *rsp);
extern bool wifi_status(char *rsp);
extern bool wifi_sleep(void);
extern bool wifi_factory_reset(void);
extern bool wifi_save_flash(void);
extern bool wifi_set_ssid(char *ssid);
extern bool wifi_get_ssid(void);
extern bool wifi_set_encrypt(encrypt_mode_t mode);
extern bool wifi_set_password(key_format_t format, uint8_t index, char *passwd);
extern bool wifi_uart_init(uint32_t baudrate, uart_data_t data,
                uart_stop_t stop, uart_parity_t parity);
#endif

#endif
