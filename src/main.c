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
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "main.h"
#include "uart.h"
#include "iox.h"
#include "timer.h"
#ifdef TH_SENSOR
#include "dht22.h"
#endif
#include "nrf24l01.h"

/* Private defines -----------------------------------------------------------*/
#define DATA_LEN    256        /* Length of temp/humid data array */


/* Private typedefs -----------------------------------------------------------*/
typedef enum {
    disconnected,
    connected,
    joined,
    transmitting,
} conn_status_t;

/* Private variables ----------------------------------------------------------*/
#ifdef WIFI_EN
static uint8_t txdata[10] = {1,2,3,4,5,6,7,8,9,10};
static uint32_t data_recv;
#endif

static uint8_t config_reg = 0;

static conn_status_t conn_status;

#ifdef TH_SENSOR
static uint16_t data_humid[DATA_LEN];
static uint16_t data_temp[DATA_LEN];
static uint32_t sensor = 0;
/* 
 * We store a pointer to "now" in the data arrays so that we do not have to
 * move them in memory.
 */
static uint16_t current_index = 0;
#endif

/* Private functions ----------------------------------------------------------*/

#ifdef TH_SENSOR
static void get_sensor(void);
#endif

/* Function Definitions -------------------------------------------------------*/
int 
main(void)
{
    SystemCoreClockUpdate();
    iox_led_init();

#ifdef TH_SENSOR
    memset(data_humid, 0, sizeof(data_humid));
    memset(data_temp, 0, sizeof(data_temp));
#endif

#ifdef DEBUG
    dbg_uart_init(115200);
#endif
    timer_init();
    nrf24l01_init();
    conn_status = disconnected;

#ifdef WIFI_EN
    //uart_init(115200); 
    data_recv = 0;
#endif

    nrf24l01_tx_mode();
    iox_led_on(true, false, false, false);
#ifdef DEBUG
    /* 
     * Send a message to indicate that it works
     */
    dbg_uart_puts("Hello Embedded World!..\r\n"); 
    config_reg = nrf24l01_read_reg(0x20);
    dbg_uart_puts("CONFIG = ");
    dbg_uart_puts((const char*)config_reg);
    dbg_uart_puts("\r\n");
#endif
    iox_led_on(true, true, false, false);

#ifdef WIFI_EN
    timer_delay(1000000UL); // wait 1sec
    iox_led_on(true, true, true, false);
#endif

    while (1) {
        timer_delay(1000000);    //wait 1sec
        iox_led_on(true, true, true, true);
        nrf24l01_transmit(txdata);

#ifdef TH_SENSOR
        if (sensor == 5) {
            get_sensor();
            sensor = 0;
        }
#endif

#ifdef WIFI_EN
        timer_delay(4000000);
        iox_led_on(true, true, true, false);
#endif
#ifdef TH_SENSOR
        sensor++;
#endif
    }

    return 0;
}

#ifdef TH_SENSOR
static void
get_sensor(void) 
{
    uint16_t temp_humid;
    uint16_t temp_temp;
    uint8_t crc;

    crc = dht22_read();
    if (crc == 1) {
        temp_humid = dht22_get_humidity() / 10u - 30u; // calibration
        temp_temp = dht22_get_temp() / 10u + 2u;

        if ((temp_humid < 100) && (temp_temp < 100)) {
            data_humid[current_index] = temp_humid; 
            data_temp[current_index] = temp_temp; 
            current_index = (current_index + 1) % DATA_LEN;
            GPIOD->ODR ^= GPIO_Pin_14; // red led toggle             
        }
    }
}
#endif

