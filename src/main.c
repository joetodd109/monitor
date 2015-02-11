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
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "main.h"
#include "uart.h"
#include "iox.h"
#include "timer.h"
#ifdef WIFI_EN
#include "sn8200.h"
#endif
#ifdef TH_SENSOR
#include "dht22.h"
#endif

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
static uint8_t seqNo = 0;
static uint32_t timeout = 0;

static const char *SSID;
static const char *sec_key;
static char http_rsp[4096];

static char rxdata[4096];
static uint32_t data_recv;

static char red_led;
static char green_led;
static char amber_led;
static char blue_led;
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
#ifdef DEBUG
static void dbg_uart_init(void);
static void dbg_uart_puts(USART_TypeDef* USARTx, const char *s);
#endif

#ifdef TH_SENSOR
static void get_sensor(void);
#endif

#ifdef WIFI_EN
static void wifi_on(void);
static void wifi_join(void);
static void wifi_disconnect(void);
static void wifi_join_network(void);
static void snic_init(void);
static void snic_ip_config(void);
static void http_get(const char *domain);
#endif

/* Function Declarations ------------------------------------------------------*/
int 
main(void)
{
    SystemCoreClockUpdate();

    iox_led_init();

#ifdef TH_SENSOR
    memset(data_humid, 0, sizeof(data_humid));
    memset(data_temp, 0, sizeof(data_temp));

    //GPIOD->ODR |= GPIO_Pin_14; 
#endif

#ifdef DEBUG
    dbg_uart_init();
#endif
    timer_init();
    conn_status = disconnected;

#ifdef WIFI_EN
    uart_init(921600); 

    data_recv = 0;
    seqNo++;
#endif

#ifdef DEBUG
    /* 
     * Send a message to indicate that it works
     */
    dbg_uart_puts(USART3, "Hello Embedded World!..\r\n"); 
#endif

#ifdef WIFI_EN
    wifi_on();
    wifi_join_network();

    const char *domain = "lukejacksonn.com";
    timer_delay(1000000UL); // wait 1sec
    seqNo++;

    if (!uart_buffer_empty()) {
        rxdata[data_recv] = uart_read_byte();
        data_recv++;
    }
#endif

    while (1) {
        timer_delay(1000000);    //wait 1sec

#ifdef TH_SENSOR
        if (sensor == 5) {
            get_sensor();
            sensor = 0;
        }
#endif

#ifdef WIFI_EN
        red_led = http_rsp[36];
        green_led = http_rsp[41];
        amber_led = http_rsp[46];
        blue_led = http_rsp[51];

        if (red_led == '1') {
            GPIOD->ODR |= GPIO_ODR_ODR_14;
        }
        else {
            GPIOD->ODR &= ~GPIO_ODR_ODR_14;
        }
        if (green_led == '1') {
            GPIOD->ODR |= GPIO_ODR_ODR_12;
        }
        else {
            GPIOD->ODR &= ~GPIO_ODR_ODR_12;
        }
        if (amber_led == '1') {
            GPIOD->ODR |= GPIO_ODR_ODR_13;
        }
        else {
            GPIOD->ODR &= ~GPIO_ODR_ODR_13;
        }
        if (blue_led == '1') {
            GPIOD->ODR |= GPIO_ODR_ODR_15;
        }
        else {
            GPIOD->ODR &= ~GPIO_ODR_ODR_15;
        }

        http_get(domain);

        if (seqNo >= 110) {
            seqNo = 0;
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

#ifdef WIFI_EN
static void 
wifi_join_network(void)
{
    wifi_disconnect();
    wifi_join();
    snic_init();
    snic_ip_config();

#ifdef DEBUG
    dbg_uart_puts(USART3, "Successfully joined network\r\n");
#endif
}

static void 
wifi_on(void)
{
    unsigned char buf[4];

    buf[0] = WIFI_ON_REQ;
    buf[1] = seqNo;
    buf[2] = (char)'G';
    buf[3] = (char)'B';

    sn8200_send_frame(CMD_ID_WIFI, 4u, buf, ACK_NOT_REQUIRED);
#ifdef DEBUG
    dbg_uart_puts(USART3, "Wifi On\r\n");
#endif
    seqNo++;
}

static void 
wifi_disconnect(void)
{
    uint8_t buf[2];

    buf[0] = WIFI_DISCONNECT_REQ;
    buf[1] = seqNo;
    sn8200_send_frame(CMD_ID_WIFI, 2, buf, ACK_NOT_REQUIRED);
    seqNo++;
}

static void 
wifi_join(void)
{
    uint8_t buf[128];
    uint8_t *p = buf;
    uint8_t sec_mode;
    uint8_t keylen = 0;
 
    *p++ = WIFI_JOIN_REQ;
    *p++ = seqNo;

    SSID = "Marlow Net";
    //SSID = "SKY8775C";
    //SSID = "PlusnetWirelessD1B3CB";
    memcpy(p, SSID, strlen(SSID));

    p += strlen(SSID);
    *p++ = 0x00;

    sec_mode = 2;

    sec_key = "frankiethedog";
    //sec_key = "RSVVVDUD";
    //sec_key = "8DD532F8DE";
    keylen = (unsigned char)strlen(sec_key);

    *p++ = sec_mode;
    *p++ = keylen;

    if (keylen) {
        memcpy(p, sec_key, keylen);
        p += keylen;
    }

#ifdef DEBUG
    dbg_uart_puts(USART3, "Joining Marlow Net...\r\n");
#endif
    sn8200_send_frame(CMD_ID_WIFI, (uint32_t)(p - buf), buf, ACK_NOT_REQUIRED);
    seqNo++;

    /* 
     * Process Received Data ......
     */
    timeout = 1000;
    while (timeout--) {
        if (!uart_buffer_empty()) {
            sn8200_handle_rsp();
        }
        timer_delay(1000);
    }
}

static void 
snic_init(void)
{
    uint8_t buf[4];
    uint8_t tmp;
    tmp = 0x00;         //The Default receive buffer size

    buf[0] = SNIC_INIT_REQ;
    buf[1] = seqNo;
    memcpy(buf + 2, (uint8_t*)&tmp, 2);
    sn8200_send_frame(CMD_ID_SNIC, 4, buf, ACK_NOT_REQUIRED);
    seqNo++;
}

static void 
snic_ip_config(void)
{
    uint8_t buf[16];

    buf[0] = SNIC_IP_CONFIG_REQ;
    buf[1] = seqNo;
    buf[2] = 0; //STA
    buf[3] = 1; //DHCP

    sn8200_send_frame(CMD_ID_SNIC, 4, buf, ACK_NOT_REQUIRED);
    seqNo++;

    /* 
     * Process Received Data ......
     */
    timeout = 1000;
    while (timeout--) {
        if (!uart_buffer_empty()) {
            sn8200_handle_rsp();
        }
        timer_delay(1000);
    } 
}

static void
http_get(const char *domain)
{
    char method = 0; //GET
    unsigned char timeout_byte = 60;

    const char *contentType = "text/html";
    const char *otherHeader = "";
    const char *uri = "/config";

    uint8_t buf[1024];
    uint8_t *p = buf;
    memset(buf, 0, sizeof(buf));

    *p++ = SNIC_HTTP_REQ;
    *p++ = seqNo;
    *p++ = 0x00; 
    *p++ = 0x50; // port 80
    *p++ = method;
    *p++ = timeout_byte;

    memcpy(p, domain, strlen(domain));
    p += strlen(domain);
    *p++ = 0x00;

    memcpy(p, uri, strlen(uri));
    p += strlen(uri);
    *p++ = 0x00;

    memcpy(p, contentType, strlen(contentType));
    p += strlen(contentType);
    *p++ = 0x00;

    memcpy(p, otherHeader, strlen(otherHeader));
    p += strlen(otherHeader);
    *p++ = 0x00;

    //*p++ = encodedLen;
    //ptr += 2; // post content length
    //*ptr = *postContent;
    //ptr += 1; // post content

    sn8200_send_frame(CMD_ID_SNIC, (uint32_t)(p - buf), buf, ACK_NOT_REQUIRED);
    seqNo++;

    /* 
     * Process Received Data ......
     */
    timeout = 1000UL;
    while (timeout--) {
        if (!uart_buffer_empty()) {
            sn8200_handle_rsp();
        }
        timer_delay(1000);
    } 
}

extern void 
handleRxWiFi(uint8_t *buf, uint16_t len)
{
    uint8_t subCmdId = buf[0];

    switch (subCmdId) {

        case WIFI_JOIN_RSP: 
            if (WIFI_SUCCESS == buf[2]) {
#ifdef DEBUG
                dbg_uart_puts(USART3, "Join success\n\r");
#endif
                conn_status = connected;
            }
            else {
#ifdef DEBUG
                dbg_uart_puts(USART3, "Join fail\n\r");
#endif
            }
        break;

        default:
            break;
    }
}

extern void 
handleRxSNIC(uint8_t *buf, uint16_t len)
{
    uint8_t subCmdId = buf[0];
    uint32_t i;

    switch (subCmdId) {

        case SNIC_IP_CONFIG_RSP:
            if (SNIC_SUCCESS == buf[2]) {
#ifdef DEBUG
                dbg_uart_puts("IPConfig OK\n\r");
#endif
                conn_status = joined;
            } 
            else {
#ifdef DEBUG
                dbg_uart_puts("IPConfig fail\n\r");
#endif
            }
        break;

        case SNIC_HTTP_RSP:
            if (SNIC_SUCCESS == buf[2]) {
                for (i = 0; i < (len - 6); i++) {
                    http_rsp[i] = buf[i + 6];
                }
            }

        default:
            break;
    }
}
#endif

#ifdef DEBUG

static void 
dbg_uart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* Connect PXx to USARTx_Tx*/
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);

    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

    /* Configure USART Tx as alternate function  */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure USART Rx as alternate function  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Enable the USART3 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* USART configuration */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART3, &USART_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    /* Enable USART */
    USART_Cmd(USART3, ENABLE);
}

static void 
dbg_uart_puts(USART_TypeDef* USARTx, const char *s) 
{
    while (*s) {
        // wait until data register is empty
        while (!(USARTx->SR & 0x00000040));
        USART3->DR = (uint16_t) *s;
        s++;
    }
}

#endif
