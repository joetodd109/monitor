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
#include "wifi.h"

/* Private defines -----------------------------------------------------------*/
#define DBGU_RX_BUFFER_SIZE 256

#define ACK_REQUIRED        1
#define ACK_NOT_REQUIRED    0

/* Length of temp/humid data array */
#define DATA_LEN 256


/* Private typedefs -----------------------------------------------------------*/
typedef enum {
    disconnected,
    connected,
    joined,
    transmitting,
} conn_status_t;

/* Private variables ----------------------------------------------------------*/
#ifdef WIFI_EN
static uint32_t timeout = 0;

static const char *SSID;
static const char *sec_key;
static char http_rsp[4096];

static char rxdata[4096];
static uint32_t data_recv;
#endif

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

<<<<<<< HEAD
#ifdef WIFI_EN
static void wifi_on(void);
static void wifi_join(void);
static void wifi_disconnect(void);
static void wifi_join_network(void);
static void snic_init(void);
static void snic_ip_config(void);
static void http_get(const char *domain);
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
    conn_status = disconnected;

#ifdef WIFI_EN
    uart_init(115200); 
    data_recv = 0;
#endif

    iox_led_on(false, false, true, false);
#ifdef DEBUG
    /* 
     * Send a message to indicate that it works
     */
    dbg_uart_puts("Hello Embedded World!..\r\n"); 
#endif
    iox_led_on(false, false, true, true);

#ifdef WIFI_EN
    
    bool success = wifi_test();
    timer_delay(1000000UL); // wait 1sec

    iox_led_on(false, false, true, success);
#endif

    while (1) {
        timer_delay(1000000);    //wait 1sec

#ifdef TH_SENSOR
        if (sensor == 5) {
            get_sensor();
            sensor = 0;
        }
#endif

#ifndef WIFI_EN
        timer_delay(4000000UL);
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

