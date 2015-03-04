/**
 ******************************************************************************
 * @file    uart.c
 * @author  Joe Todd
 * @version
 * @date    January 2014
 * @brief   WiFi Automation
 *
  ******************************************************************************/


/* Includes -------------------------------------------------------------------*/
#include "uart.h"

/* Private typedefs -----------------------------------------------------------*/
typedef struct {
    char data[RX_BUFFER_SIZE];
    uint32_t tail;
    uint32_t head;
} uart_buf_t;

/* Private variables ----------------------------------------------------------*/
static uart_buf_t uart_buf;
static uint32_t uart_recv_cnt;

/* Function Definitions -------------------------------------------------------*/
extern void 
uart_init(uint32_t baudrate)
{
    uart_buf.tail = 0;
    uart_buf.head = 0;
    uart_recv_cnt = 0;

    uint32_t reg;
    uint32_t fraction;
    uint32_t mantissa;
    //callback = callback_fn;

    uart_recv_cnt = 0;

    /* Enable GPIO clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* Enable UART clock */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* Connect PXx to USARTx_Tx/Rx*/
    iox_alternate_func(iox_port_a, USART1_TX_PIN, USART1_AF);
    iox_alternate_func(iox_port_a, USART1_RX_PIN, USART1_AF);

    /* Configure USART Tx/Rx as alternate function  */
    iox_configure_pin(iox_port_b, USART1_TX_PIN, iox_mode_af, iox_type_pp, 
                        iox_speed_fast, iox_pupd_up);
    iox_configure_pin(iox_port_b, USART1_RX_PIN, iox_mode_af, iox_type_pp,
                        iox_speed_fast, iox_pupd_up);

    /* 1 Stop bit, asynchronous mode */
    USART1->CR2 = 0x00;
    /* Rx interrupt enabled, Tx/Rx enabled */
    USART1->CR1 = (USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE);
    /* No hardware flow control, DMA enabled */
    USART1->CR3 = 0x00;
    /* Configure baudrate */
    mantissa = ((25 * PCLK2) / (4 * baudrate));
    reg = (mantissa / 100) << 4;
    fraction = mantissa - (100 * (reg >> 4));
    reg |= (((fraction * 16) + 50) / 100) & 0x0F;
    USART1->BRR = reg;

    utl_enable_irq(USART1_IRQn);

    USART1->CR1 |= USART_CR1_UE;
}


extern void 
dbg_uart_init(uint32_t baudrate)
{
    uint32_t reg;
    uint32_t fraction;
    uint32_t mantissa;
    //callback = callback_fn;

    uart_recv_cnt = 0;

    /* Enable GPIO clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    /* Enable UART clock */
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

    /* Connect PXx to USARTx_Tx/Rx*/
    iox_alternate_func(iox_port_c, USART6_TX_PIN, USART6_AF);
    iox_alternate_func(iox_port_c, USART6_RX_PIN, USART6_AF);

    /* Configure USART Tx/Rx as alternate function  */
    iox_configure_pin(iox_port_c, USART6_TX_PIN, iox_mode_af, iox_type_pp, 
                        iox_speed_fast, iox_pupd_up);
    iox_configure_pin(iox_port_c, USART6_RX_PIN, iox_mode_af, iox_type_pp,
                        iox_speed_fast, iox_pupd_up);

    /* 1 Stop bit, asynchronous mode */
    USART6->CR2 = 0x00;
    /* Tx/Rx enabled */
    USART6->CR1 = USART_CR1_TE;
    /* No hardware flow control, DMA enabled */
    USART6->CR3 = 0x00;
    /* Configure baudrate */
    mantissa = ((25 * PCLK2) / (4 * baudrate));
    reg = (mantissa / 100) << 4;
    fraction = mantissa - (100 * (reg >> 4));
    reg |= (((fraction * 16) + 50) / 100) & 0x0F;
    USART6->BRR = reg;

    utl_enable_irq(USART6_IRQn);

    USART6->CR1 |= USART_CR1_UE;
}

extern void 
dbg_uart_puts(const char *s) 
{
    while (*s) {
        while ((USART6->SR & USART_SR_TC) == 0);
        USART6->DR = (uint16_t) *s++;
    }
}

extern void
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

extern void 
uart_send_data(char *buf, uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++) {
        USART1->DR = buf[i];
        while ((USART1->SR & USART_SR_TC) == 0);
    }
}

extern bool 
uart_buffer_empty(void)
{
    return (uart_buf.head == uart_buf.tail);
}

extern char 
uart_read_byte(void)
{
    char data = ' ';

    if (uart_buf.head != uart_buf.tail) {
        data = uart_buf.data[uart_buf.tail];
        uart_buf.tail = (uart_buf.tail + 1) % RX_BUFFER_SIZE;
    }

    return data;
}

void USART1_IRQHandler(void)
{
    uint32_t sr;

    sr = USART1->SR;

    if (sr & USART_SR_RXNE) {
        //callback(USART1->DR);
        uart_buf.data[uart_buf.head] = USART1->DR;

        USART1->SR &= ~USART_SR_RXNE;
        uart_buf.head = (uart_buf.head + 1) % RX_BUFFER_SIZE;
        uart_recv_cnt++;
    }
}