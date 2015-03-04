/**
 ******************************************************************************
 * @file    wifi.c
 * @author  Joe Todd
 * @version
 * @date    January 2015
 * @brief   WiFi Automation
 *          For TLG10UA3 WiFi devices.
 *
  ******************************************************************************/


/* Includes -------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "wifi.h"

/* Private variables ----------------------------------------------------------*/

static char txdata[TX_BUF_SIZE];
static char rxdata[RX_BUF_SIZE];

static uint16_t concat(char *d, char *s);
static bool parse(char *dst, char *src, char *f);
static void reset(char *buf);
static uint16_t encode(char *dst, char *cmd, char *op, char *params);

/* Function Declarations ------------------------------------------------------*/

/*
 * Concatenate s2 onto s1.
 */
static uint16_t
concat(char *d, char *s)
{
    uint16_t l = 0;

    if ((d == NULL) || (s == NULL)) {
        return 0;
    }

    while (*d != '\0') {
        l++, d++;
    }

    while (*s != '\0') {
        *d++ = *s++;
        l++;
    }

    *d = '\0';

    return l;
}

/*
 * Scan src for characters in f, if f
 * is found, then copy subsequent chars
 * into dst.
 */
static bool
parse(char *dst, char *src, char *f)
{
    int i = 0;

    if ((dst == NULL) || (src == NULL) || (f == NULL)) {
        return false;
    }

    while (*src != '\0') {
        while (*src == *f) {
            src++, f++, i++;
            if (*f == '\0') {
                /* found f so copy the rest
                 * into destination. */
                while (*src != '\0') {
                    *dst++ = *src++;
                }
                *dst = '\0';
                return true;
            }
        }
        if (i > 0) {
            f -= i;
            i = 0;
        }
        src++;
    }
    return false;
}

static void
reset(char *buf)
{
    while (*buf) {
        *buf++ = 0;
    }
}

static uint16_t
encode(char *dst, char *cmd, char *op, char *params)
{
    uint16_t len;

    len = concat(dst, "AT+");
    len += concat(&dst[len], cmd);
    len += concat(&dst[len], op);
    len += concat(&dst[len], params);

    dst[len++] = '\r';
    dst[len] = '\0';

    return len;
}

/*
 * WiFi Commands.
 */
extern bool
wifi_test(void)
{
    uint16_t txlen;
    uint16_t rxlen;
    char tmp[RX_BUFFER_SIZE];
    reset(txdata);
    reset(rxdata);
    rxlen = 0;

    txlen = encode(txdata, "E", NULL, NULL);
    uart_send_data(txdata, txlen);
    timer_delay(1000000UL);

    if (!uart_buffer_empty()) {
        while (!uart_buffer_empty()) {
            rxdata[rxlen++] = uart_read_byte();
        }
        dbg_uart_puts(rxdata);

        if (parse(tmp, rxdata, "+OK")) {
            return true;
        }
    }
    return false;
}

extern bool
wifi_reset(void)
{
    uint16_t rxlen;
    uint16_t txlen;
    char tmp[RX_BUFFER_SIZE];
    reset(txdata);
    reset(rxdata);
    rxlen = 0;

    txlen = encode(txdata, "Z", NULL, NULL);
    uart_send_data(txdata, txlen);
    timer_delay(1000000UL);

    if (!uart_buffer_empty()) {
        while (!uart_buffer_empty()) {
            rxdata[rxlen++] = uart_read_byte();
        }
        dbg_uart_puts(rxdata);

        if (parse(tmp, rxdata, "+OK")) {
            return true;
        }
    }
    return false;
}

#if 0

extern bool
wifi_join(char *rsp)
{
    reset(txdata);
    encode(txdata, "WJOIN", NULL, NULL);
    printf("%s", txdata);
}

extern bool
wifi_disconnect(void)
{
    reset(txdata);
    encode(txdata, "WLEAV", NULL, NULL);
    printf("%s", txdata);
}

extern bool
wifi_scan(char *rsp)
{
    reset(txdata);
    encode(txdata, "WSCAN", NULL, NULL);
    printf("%s", txdata);
}

extern bool
wifi_status(char *rsp)
{
    reset(txdata);
    encode(txdata, "LKSTT", NULL, NULL);
    printf("%s", txdata);
}

extern bool
wifi_sleep(void)
{
    reset(txdata);
    encode(txdata, "ENTS", NULL, NULL);
    printf("%s", txdata);
}

extern bool
wifi_factory_reset(void)
{
    reset(txdata);
    encode(txdata, "RSTF", NULL, NULL);
    printf("%s", txdata);
}

extern bool
wifi_save_flash(void)
{
    reset(txdata);
    encode(txdata, "PMTF", NULL, NULL);
    printf("%s", txdata);
}

extern bool
wifi_set_ssid(char *ssid)
{
    reset(txdata);
    encode(txdata, "SSID", "=!", ssid);
    printf("%s", txdata);

    char rsp[16] = "+OK=[ssid]\r\n\r\n";

    if (parse(rxdata, rsp, "+OK=")) {
        printf("resp = %s", rxdata);
        return true;
    }
    return false;
}

extern bool
wifi_get_ssid(void)
{
    reset(txdata);
    encode(txdata, "SSID", "=?", NULL);
}

extern bool
wifi_set_encrypt(encrypt_mode_t mode)
{
    char modestr[2];
    sprintf(modestr, "%d", mode);

    reset(txdata);
    encode(txdata, "ENCRY", "=!", modestr);
    printf("%s", txdata);
}

extern bool
wifi_set_password(key_format_t format, uint8_t index, char *passwd)
{
    char params[68];
    sprintf(params, "%d,%d,", format, index);
    concat(params, passwd);

    reset(txdata);
    encode(txdata, "KEY", "=!", params);
    printf("%s", txdata);

    char rsp[32] = "+OK=[format][key][passwd]\r\n\r\n";
    if (parse(rxdata, rsp, "+OK=")) {
        printf("resp = %s", rxdata);
        return true;
    }
    return false;
}

extern bool
wifi_uart_init(uint32_t baudrate, uart_data_t data,
                uart_stop_t stop, uart_parity_t parity)
{
    char params[10];
    sprintf(params, "%d,%d,%d,%d", baudrate, data, stop, parity);

    reset(txdata);
    encode(txdata, "UART", "=!", params);
}

#endif

