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
static uint8_t txdata[32] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,
                                20,21,22,23,24,25,26,27,28,29,30,31,32};
static uint8_t rxdata[32];
static uint32_t data_recv;
#endif

static uint8_t status = 0;
static uint8_t flag = 0;

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

#ifdef DEBUG
static void print_byte(uint8_t byte);
static void print_regs(void);
static void print_status(void);
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

    iox_led_on(true, false, false, false);
    nrf24l01_tx_mode();
    timer_delay(150000UL); /* wait 150ms */

#ifdef DEBUG
    dbg_uart_puts("\r\nnRF24L01 Registers\r\n\r\n"); 
    print_regs();
#endif

    while (1) {
        timer_delay(1000000);    /* wait 1sec */
        iox_led_on(false, false, false, true);
        nrf24l01_transmit(txdata);

#ifdef TH_SENSOR
        if (sensor == 5) {
            get_sensor();
            sensor = 0;
        }
        sensor++;
#endif

#ifdef WIFI_EN
        timer_delay(4000000);       /* wait 4sec */
        iox_led_on(false, false, false, false);
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

#ifdef DEBUG
static void
print_byte(uint8_t byte)
{
    int32_t i;
    uint8_t mask, bit;

    dbg_uart_puts("0");
    dbg_uart_puts("b");
    for (i = 7; i >= 0; i--) {
        mask = 1u << i;
        bit = mask & byte;
        if (bit == 0) {
            dbg_uart_puts("0");
        }
        else {
            dbg_uart_puts("1");
        }
    }
    dbg_uart_puts("\r\n");
}

static void
print_regs(void)
{
    uint8_t i, reg;
    uint8_t addr[5];

    reg = nrf24l01_read_reg(CONFIG);
    dbg_uart_puts("CONFIG = ");
    print_byte(reg);
    reg = nrf24l01_read_reg(EN_AA);
    dbg_uart_puts("EN_AA = ");
    print_byte(reg);
    reg = nrf24l01_read_reg(EN_RXADDR);
    dbg_uart_puts("EN_RXADDR = ");
    print_byte(reg);    
    reg = nrf24l01_read_reg(SETUP_AW);
    dbg_uart_puts("SETUP_AW = ");
    print_byte(reg);
    reg = nrf24l01_read_reg(SETUP_RETR);
    dbg_uart_puts("SETUP_RETR = ");
    print_byte(reg);
    reg = nrf24l01_read_reg(RF_CH);
    dbg_uart_puts("RF_CH = ");
    print_byte(reg);
    reg = nrf24l01_read_reg(RF_SETUP);
    dbg_uart_puts("RF_SETUP = ");
    print_byte(reg);
    reg = nrf24l01_read_reg(RX_PW_P0);
    dbg_uart_puts("RX_PW_P0 = ");
    print_byte(reg);
    reg = nrf24l01_read_reg(RX_PW_P1);
    dbg_uart_puts("RX_PW_P1 = ");
    print_byte(reg);
    reg = nrf24l01_read_reg(FEATURE);
    dbg_uart_puts("FEATURE = ");
    print_byte(reg);
    reg = nrf24l01_read_reg(DYNPD);
    dbg_uart_puts("DYNPD = ");
    print_byte(reg);
    nrf24l01_spi_read(TX_ADDR, addr, 5);
    dbg_uart_puts("TXADDR = ");
    for (i = 0; i < 5; i++) {
        print_byte(addr[i]);
    }
}

static void
print_status(void)
{
    uint8_t reg;

    reg = nrf24l01_read_reg(STATUS);
    dbg_uart_puts("STATUS = ");
    print_byte(reg);
}
#endif
