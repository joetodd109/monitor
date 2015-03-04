/********************************************************************************
 * @file    nrf24l01.c
 * @author  Joe Todd
 * @version
 * @date    February 2015
 * @brief   Wireless Temp/Humidity Monitor
 *          Use nRF24L01 to transmit data to another nRF24L01 RPi gateway.
 *          This is the best solution so far as mongolab requires secure 
 *          https communication.
 *
  ******************************************************************************/

/* Includes -------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdint.h>
#include "nrf24l01.h"
#include "uart.h"

/* Private defines -----------------------------------------------------------*/

#define RX_PLOAD_WIDTH  32u
#define TX_PLOAD_WIDTH  32u

#define BUFSIZE         256u

/*
 * SPI GPIO's.
 */
#define GPIO_CE_PIN     3u
#define GPIO_CS_PIN     4u
#define SPI_SCK_PIN     5u
#define SPI_MISO_PIN   	6u
#define SPI_MOSI_PIN    7u

#define SPI1_AF5        5u

static uint8_t rx_addr1[ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};
static uint8_t tx_addr[ADR_WIDTH]= {0xC2,0xC2,0xC2,0xC2,0xC2};
static uint8_t rx_addr[ADR_WIDTH]= {0xC2,0xC2,0xC2,0xC2,0xC2};

/* Private functions ----------------------------------------------------------*/
static uint8_t nrf24l01_spi_write(uint8_t reg, uint8_t *buf, uint8_t len);

/* Function Definitions -------------------------------------------------------*/
extern void
nrf24l01_init(void)
{
    /*
     * Enable SPI clock.
     */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /*
     * Configure SPI GPIO's.
     */
    iox_configure_pin(iox_port_a, SPI_SCK_PIN, iox_mode_af,
                        iox_type_pp, iox_speed_fast, iox_pupd_down);
    iox_configure_pin(iox_port_a, SPI_MISO_PIN, iox_mode_af,
                        iox_type_od, iox_speed_fast, iox_pupd_down);
    iox_configure_pin(iox_port_a, SPI_MOSI_PIN, iox_mode_af,
                        iox_type_pp, iox_speed_fast, iox_pupd_down);

    /*
     * Prepare output ports for alternate function.
     */
    iox_alternate_func(iox_port_a, SPI_SCK_PIN, SPI1_AF5);
    iox_alternate_func(iox_port_a, SPI_MISO_PIN, SPI1_AF5);
    iox_alternate_func(iox_port_a, SPI_MOSI_PIN, SPI1_AF5);

    /*
     * Configure CE and CS GPIO's.
     */
    iox_configure_pin(iox_port_a, GPIO_CE_PIN, iox_mode_out,
                        iox_type_pp, iox_speed_fast, iox_pupd_down);
    iox_configure_pin(iox_port_a, GPIO_CS_PIN, iox_mode_out,
                        iox_type_pp, iox_speed_fast, iox_pupd_up);

    /*
     * SPI Configuration.
     */
    SPI1->CR1 = (SPI_CR1_MSTR   /* Master mode */
        | SPI_CR1_SSM           /* Software slave management enabled */
        | SPI_CR1_SSI);

    /*
     * Enable SPI.
     */
    SPI1->CR1 |= SPI_CR1_SPE;
}

extern void
nrf24l01_tx_mode(void)
{
    iox_set_pin_state(iox_port_a, GPIO_CE_PIN, false);

	nrf24l01_write_reg(WRITE_nRF_REG + CONFIG, 0x0E); /* Enable 2-byte crc, ptx and power up */
    nrf24l01_write_reg(WRITE_nRF_REG + EN_AA, 0x00);     /* Disable Auto ACKs */
    nrf24l01_write_reg(WRITE_nRF_REG + EN_RXADDR, 0x03); /* Enable data pipe 0 and 1 */
    nrf24l01_write_reg(WRITE_nRF_REG + SETUP_AW, 0x03);  /* 5 bytes address width */
	nrf24l01_write_reg(WRITE_nRF_REG + SETUP_RETR, 0xFF); /* Auto Retransmit Delay: 4ms, Up to 15 Retransmissions */
	nrf24l01_write_reg(WRITE_nRF_REG + RF_CH, 0x60); /* Set channel */
	nrf24l01_write_reg(WRITE_nRF_REG + RF_SETUP, 0x24); /* Setup power -6dbm, rate 250kbps */
    nrf24l01_write_reg(WRITE_nRF_REG + RX_PW_P0, 0x08); /* Set Rx pipe 0 to 8 bytes */
    nrf24l01_write_reg(WRITE_nRF_REG + RX_PW_P1, 0x08); /* Set Rx pipe 1 to 8 bytes */
    nrf24l01_spi_write(WRITE_nRF_REG + TX_ADDR, tx_addr, ADR_WIDTH); /* write address into tx_addr */
	nrf24l01_spi_write(WRITE_nRF_REG + RX_ADDR_P0, rx_addr, ADR_WIDTH); /* write address into rx_addr_p0 */
    nrf24l01_spi_write(WRITE_nRF_REG + RX_ADDR_P1, rx_addr1, ADR_WIDTH); /* write address into rx_addr_p1 */
    nrf24l01_write_reg(WRITE_nRF_REG + FEATURE, 0x06); /* Enable payload with/without ACK, and dynamic payload len */
    nrf24l01_write_reg(WRITE_nRF_REG + DYNPD, 0x3F); /* Enable dynamic payload length  */
    
    timer_delay(150000UL); /* wait 150ms */
}

/* 
 * Transmit data, max 32 bytes.
 */
extern bool
nrf24l01_transmit(uint8_t *tx_buf, uint8_t len)
{
    uint8_t status;

    iox_set_pin_state(iox_port_a, GPIO_CE_PIN, false);

	nrf24l01_write_reg(WRITE_nRF_REG + STATUS, 0x7E); /* Write 1 to clear status register */
	nrf24l01_write_reg(WRITE_nRF_REG + CONFIG, 0x0E); /* Enable power up and ptx */
	nrf24l01_write_reg(FLUSH_TX, 0x00);               /* Flush TX FIFO */

    /* Write data to FIFO */
	status = nrf24l01_spi_write(WR_TX_PLOAD, tx_buf, len);

	nrf24l01_write_reg(WRITE_nRF_REG + STATUS, 0x20); /* Clear TX FIFO data sent status bit */
	timer_delay(200);      /* Wait 130us settling time */

    iox_set_pin_state(iox_port_a, GPIO_CE_PIN, true);
	timer_delay(20);       /* Pulse CE for at least 10us */
	iox_set_pin_state(iox_port_a, GPIO_CE_PIN, false);
    timer_delay(5000);     /* Wait until transmission complete */

    if ((nrf24l01_read_reg(STATUS) & 0x20)) {
        return true;
    }
    return false;
}

/* 
 * Receive data, max 32 bytes.
 */
extern bool
nrf24l01_receive(uint8_t *rx_buf)
{
    uint8_t status;

	status = nrf24l01_read_reg(STATUS);
    /* Write 1 to clear bit */
    nrf24l01_write_reg(WRITE_nRF_REG + STATUS, status);

	if (status & 0x40) /* Data Ready RX FIFO interrupt */
	{
        nrf24l01_spi_read(RD_RX_PLOAD, rx_buf, RX_PLOAD_WIDTH);
        return true;
	}
    return false;
}

/*
 * Read a register value via SPI.
 */
extern uint8_t
nrf24l01_read_reg(uint8_t reg)
{
	uint8_t value;

	iox_set_pin_state(iox_port_a, GPIO_CS_PIN, false);
	timer_delay(20);
	nrf24l01_send_byte(reg);
    value = nrf24l01_send_byte(NOP);
	iox_set_pin_state(iox_port_a, GPIO_CS_PIN, true);
    timer_delay(20);

	return value;
}

/*
 * Write a value to a register via SPI.
 */
extern uint8_t
nrf24l01_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

	iox_set_pin_state(iox_port_a, GPIO_CS_PIN, false);
    timer_delay(20);
	status = nrf24l01_send_byte(reg);
	nrf24l01_send_byte(value);
	iox_set_pin_state(iox_port_a, GPIO_CS_PIN, true);
    timer_delay(20);

	return status;
}

/*
 * Return value from a register via SPI.
 */
extern uint8_t
nrf24l01_spi_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t status, i;

	iox_set_pin_state(iox_port_a, GPIO_CS_PIN, false);
	timer_delay(20);
	status = nrf24l01_send_byte(reg);

    for (i = 0; i < len; i++) {
        buf[i] = nrf24l01_send_byte(NOP);
    }

	iox_set_pin_state(iox_port_a, GPIO_CS_PIN, true);
    timer_delay(20);

	return status;
}

/*
 * Write a sequence of bytes to nRF24L01 via SPI.
 */
static uint8_t
nrf24l01_spi_write(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t status, i;

	iox_set_pin_state(iox_port_a, GPIO_CS_PIN, false);
	timer_delay(20);
	status = nrf24l01_send_byte(reg);
	for (i = 0; i < len; i++) {
		nrf24l01_send_byte(*buf++);
	}
	iox_set_pin_state(iox_port_a, GPIO_CS_PIN, true);
    timer_delay(20);

	return status;
}

/*
 * Send byte to nRF24L01 via SPI
 * Returns the status register.
 */
extern uint8_t
nrf24l01_send_byte(uint8_t data)
{
    /* Wait until Tx buffer is empty */
    while ((SPI1->SR & SPI_SR_TXE) == 0);
    /* Send byte through SPI */
    SPI1->DR = data;
    /* Wait to receive a byte */
    while ((SPI1->SR & SPI_SR_RXNE) == 0);
    /* Return byte from SPI */
    return SPI1->DR;
}

extern void
print_status(uint8_t reg)
{
    dbg_uart_puts("STATUS = ");
    print_byte(reg);
}

extern void
print_regs(void)
{
    uint8_t reg;

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
}