/**
 ******************************************************************************
 * @file    main.c
 * @author  Joe Todd
 * @version
 * @date    April 2014
 * @brief   WiFi Automation
 *
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdbool.h>

#include "uart.h"
#include "iox.h"
#include "timer.h"
#include "nrf24l01.h"
#include "dht22.h"

/* Private defines -----------------------------------------------------------*/
#define DATA_LEN       8u

#define TEMP_ID        200
#define HUMID_ID       201

/*
 * Enable debugging info via uart to usb dongle.
 */
#define DEBUG


/* Private variables ----------------------------------------------------------*/
static uint8_t txdata[DATA_LEN];

static uint16_t data_humid = 56;
static int16_t data_temp = 22;

/* Private functions ----------------------------------------------------------*/
static void assemble_frame(int16_t temp, uint16_t humid);

/* Function Definitions -------------------------------------------------------*/

/*              Frame Structure
 * -------- * -------- * -------- * -------- *
 *    ID    |        Value        |   Denom  *
 * -------- * -------- * -------- * -------- *
 *    ID    |        Value        |   Denom  *
 * -------- * -------- * -------- * -------- *
 */ 
static void
assemble_frame(int16_t temp, uint16_t humid) 
{
    txdata[0] = TEMP_ID;
    txdata[1] = temp >> 8u;
    txdata[2] = temp & 0xFF;
    txdata[3] = 10;
    txdata[4] = HUMID_ID;
    txdata[5] = humid >> 8u;
    txdata[6] = humid & 0xFF;
    txdata[7] = 10;
}

int 
main(void)
{
    SystemCoreClockUpdate();
    iox_led_init();
    bool success = false;

#ifdef DEBUG
    dbg_uart_init(115200);
#endif
    timer_init();
    nrf24l01_init();

    iox_led_on(true, false, false, false);
    nrf24l01_tx_mode();

#ifdef DEBUG
    dbg_uart_puts("\r\nnRF24L01 Registers\r\n\r\n"); 
    print_regs();
#endif

    while (1) {
        iox_led_on(false, false, false, true);
        timer_delay(1000000UL);    /* wait 1sec */

        if (dht22_read()) {
            data_humid = dht22_get_humidity();
            data_temp = dht22_get_temp();
            assemble_frame(data_temp, data_humid);
            success = nrf24l01_transmit(txdata, DATA_LEN);

#ifdef DEBUG
            success ? dbg_uart_puts("tx success\r\n") : dbg_uart_puts("tx failure\r\n");
#endif
        }

        iox_led_on(false, false, false, false);
        timer_delay(4000000UL);       /* wait 4sec */
    }

    return 0;
}

