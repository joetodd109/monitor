/**
 ******************************************************************************
 * @file    dht22.c
 * @author  Joe Todd
 * @version
 * @date    June 2014
 * @brief   WiFi Automation
 *
  ******************************************************************************/


/* Includes -------------------------------------------------------------------*/
#include "inttypes.h"
#include <string.h>

#include "stm32f4xx.h"
#include "dht22.h"
#include "timer.h"
#include "iox.h"

/* Defines --------------------------------------------------------------------*/
#define DHT22_BITS  40u

/* Globals --------------------------------------------------------------------*/
static uint8_t dht_bytes[6];

/* Function Definitions -------------------------------------------------------*/
static void dht22_start(void);

/* Function Declarations ------------------------------------------------------*/
/* 
 * Read values from DHT22
 * Returns success or failure
 */
extern uint8_t 
dht22_read(void) 
{
    uint8_t num = 0u;
    uint8_t last_state;
    uint8_t cur_state;
    int32_t cnt;
    uint32_t start;
    uint32_t last_timer;
    uint32_t cur_timer;

    memset(&dht_bytes, 0, 6);

    /* 
     * Send startup sequence.
     */
    dht22_start();

    timer_start();
    while (iox_get_pin_state(iox_port_e, 5u) != 0);
    last_state = iox_get_pin_state(iox_port_e, 5u);
    cur_state = last_state;
    cnt = -1;
    start = timer_get();
    last_timer = start;
    cur_timer = start;

    while (cnt <= 40 && (cur_timer - start) < 10000UL) {
        cur_state = iox_get_pin_state(iox_port_e, 5u);
        /* 
         * On a rising edge, store timer value.
         */
        if ((last_state == 0) && (cur_state == 1)) {
            last_timer = cur_timer;
        }
        /* 
         * On a falling edge, check time for binary data.
        */
        if ((last_state == 1u) && (cur_state == 0u)) {
            if (cnt >= 0) {
                num = cnt / 8u;
                if ((cur_timer - last_timer) > 50u) {
                    dht_bytes[num] |= 1u;
                }
                dht_bytes[num] <<= 1u;
            }
            cnt++;
        }
        last_state = cur_state;
        cur_timer = timer_get();
    } 

    return dht22_check_checksum();
}

static void 
dht22_start(void) 
{
    /*
     * Configure pin
     */
    iox_configure_pin(iox_port_e, 5u, iox_mode_out, iox_type_pp, iox_speed_fast, iox_pupd_none);

    /* 
     * Drive low and wait at least 1ms
     */
    iox_set_pin_state(iox_port_e, 5u, false);
    timer_delay(1500UL);

    /* 
     * Drive high and wait at least 40us 
     */
    iox_set_pin_state(iox_port_e, 5u, true);
    timer_delay(50UL); 

    /*
     * Switch to input
     */
    iox_configure_pin(iox_port_e, 5u, iox_mode_in, iox_type_pp, iox_speed_fast, iox_pupd_up);
}

extern uint16_t 
dht22_get_humidity(void) 
{
    return (dht_bytes[0] << 8 | dht_bytes[1]);
}

extern int16_t 
dht22_get_temp(void) 
{
    return (dht_bytes[2] << 8 | dht_bytes[3]);
}

extern uint8_t 
dht22_check_checksum(void) 
{
    uint8_t chksum = (dht_bytes[0] + dht_bytes[1] + dht_bytes[2] + dht_bytes[3]);
    return chksum == dht_bytes[4];
}

extern uint8_t 
dht22_get_checksum(void) 
{
    uint8_t chksum = (dht_bytes[0] + dht_bytes[1] + dht_bytes[2] + dht_bytes[3]);
    return chksum;
}
