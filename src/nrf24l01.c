/********************************************************************************
 * @file    nrf24l01.c
 * @author  Joe Todd
 * @version
 * @date    February 2015
 * @brief   Wireless Temp/Humidity Monitor
 *
  ******************************************************************************/

/* Includes -------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdint.h>
#include "nrf24l01.h"

/* Private defines -----------------------------------------------------------*/

/*
 * nRF24L01 Commands.
 */
#define READ_nRF_REG    0x00  /* Command for read register */
#define WRITE_nRF_REG   0x20  /* Command for write register - access in power down standby modes only */
#define RD_RX_PLOAD     0x61  /* Command to read Rx payload */
#define WR_TX_PLOAD     0xA0  /* Command to write Tx payload */
#define FLUSH_TX        0xE1  /* Command to flush Tx FIFO */
#define FLUSH_RX        0xE2  /* Command to flush Rx FIFO */
#define REUSE_TX_PL     0xE3  /* Command to reuse Tx payload */
#define RX_PAYLD_LEN    0x60  /* Command to read Rx payload width */
#define NOP             0xFF  /* Reserve */

/*
 * nRF24L01 Registers.
 */
#define CONFIG          0x00  /*  Configuration Register - interrupts/crc */
#define EN_AA           0x01  /*  Enable the Auto-Acknowledgement Function */
#define EN_RXADDR       0x02  /*  Enable Rx Addresses */
#define SETUP_AW        0x03  /*  Setup address widths */
#define SETUP_RETR      0x04  /*  Setup automatic retransmission */
#define RF_CH           0x05  /*  RF Channel */
#define RF_SETUP        0x06  /*  Setup the rate of data, and transmit power */
#define STATUS          0x07  /*  Status Register */
#define OBSERVE_TX      0x08  /*  Transmit observe register */
#define CD              0x09  /*  Carrier detect */
#define RX_ADDR_P0      0x0A  /*  Receive address of channel 0 */
#define RX_ADDR_P1      0x0B  /*  Receive address of channel 1 */
#define RX_ADDR_P2      0x0C  /*  Receive address of channel 2 */
#define RX_ADDR_P3      0x0D  /*  Receive address of channel 3 */
#define RX_ADDR_P4      0x0E  /*  Receive address of channel 4 */
#define RX_ADDR_P5      0x0F  /*  Receive address of channel 5 */
#define TX_ADDR         0x10  /*  Transmit address */
#define RX_PW_P0        0x11  /*  Size of receive data in channel 0 */
#define RX_PW_P1        0x12  /*  Size of receive data in channel 1 */
#define RX_PW_P2        0x13  /*  Size of receive data in channel 2 */
#define RX_PW_P3        0x14  /*  Size of receive data in channel 3 */
#define RX_PW_P4        0x15  /*  Size of receive data in channel 4 */
#define RX_PW_P5        0x16  /*  Size of receive data in channel 5 */
#define FIFO_STATUS     0x17  /*  FIFO Status */
#define DYNPD           0x1C  /*  Enable dynamic payload length */
#define FEATURE         0x1D  /*  Feature register */

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

#define ADR_WIDTH       5u

static uint8_t txbuf[BUFSIZE] = {0};
static uint8_t rxbuf[BUFSIZE] = {0};

//static uint8_t tx_addr[ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};
static uint8_t rx_addr1[ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};
static uint8_t tx_addr[ADR_WIDTH]= {0xC2,0xC2,0xC2,0xC2,0xC2};
static uint8_t rx_addr[ADR_WIDTH]= {0xC2,0xC2,0xC2,0xC2,0xC2};
//static uint8_t tx_addr[ADR_WIDTH]= {0xB3,0xB4,0xB5,0xB6,0xF1};
//static uint8_t rx_addr[ADR_WIDTH]= {0xB3,0xB4,0xB5,0xB6,0xF1};
//static uint8_t tx_addr[ADR_WIDTH]= {0x01,0x23,0x45,0x67,0x89};
//static uint8_t rx_addr[ADR_WIDTH]= {0x01,0x23,0x45,0x67,0x89};

/* Private functions ----------------------------------------------------------*/
static uint8_t nrf24l01_write_reg(uint8_t reg, uint8_t value);
static uint8_t nrf24l01_spi_read(uint8_t reg, uint8_t *buf, uint8_t len);
static uint8_t nrf24l01_spi_write(uint8_t reg, uint8_t *buf, uint8_t len);
static uint8_t nrf24l01_send_byte(uint8_t data);
static void nrf24l01_delay(unsigned long n);

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
                        iox_type_pp, iox_speed_fast, iox_pupd_down);
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
                        iox_type_pp, iox_speed_fast, iox_pupd_up);
    iox_configure_pin(iox_port_a, GPIO_CS_PIN, iox_mode_out,
                        iox_type_pp, iox_speed_fast, iox_pupd_up);

    /*
     * SPI Configuration.
     */
    SPI1->CR1 = (SPI_CR1_MSTR   /* Master mode */
        | SPI_CR1_BR            /* BaudRate / 256 */
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
    nrf24l01_delay(20);
    nrf24l01_write_reg(WRITE_nRF_REG + EN_AA, 0x3F);     /* Enable Auto ACKs */
    nrf24l01_delay(20);
    nrf24l01_write_reg(WRITE_nRF_REG + EN_RXADDR, 0x03); /* Enable data pipe 0 and 1 */
	nrf24l01_delay(20);
    nrf24l01_write_reg(WRITE_nRF_REG + SETUP_AW, 0x03);  /* 5 bytes address width */
	nrf24l01_delay(20);
	nrf24l01_write_reg(WRITE_nRF_REG + SETUP_RETR, 0xFF); /* Auto Retransmit Delay: 4ms, Up to 15 Retransmissions */
	nrf24l01_delay(20);
	nrf24l01_write_reg(WRITE_nRF_REG + RF_CH, 0x60); /* Set channel */
	nrf24l01_delay(20);
	nrf24l01_write_reg(WRITE_nRF_REG + RF_SETUP, 0x08); /* Setup power -18dbm, rate 2Mbps */
	nrf24l01_delay(20);
    nrf24l01_write_reg(WRITE_nRF_REG + RX_PW_P0, 0x20); /* Set Rx pipe 0 to 32 bytes */
    nrf24l01_delay(20);
    nrf24l01_write_reg(WRITE_nRF_REG + RX_PW_P1, 0x20); /* Set Rx pipe 1 to 32 bytes */
	nrf24l01_delay(20);
    nrf24l01_spi_write(WRITE_nRF_REG + TX_ADDR, tx_addr, ADR_WIDTH); /* write address into tx_addr */
	nrf24l01_delay(20);
	nrf24l01_spi_write(WRITE_nRF_REG + RX_ADDR_P0, rx_addr, ADR_WIDTH); /* write address into rx_addr_p0 */
    nrf24l01_delay(20);
    nrf24l01_spi_write(WRITE_nRF_REG + RX_ADDR_P1, rx_addr1, ADR_WIDTH); /* write address into rx_addr_p1 */
    nrf24l01_delay(20);
    nrf24l01_write_reg(WRITE_nRF_REG + FEATURE, 0x06); /* Enable payload with ACK, and dynamic payload len */
    nrf24l01_delay(20);
    nrf24l01_write_reg(WRITE_nRF_REG + DYNPD, 0x3F); /* Enable dynamic payload length  */
	nrf24l01_delay(200);
}

extern void
nrf24l01_transmit(uint8_t *tx_buf)
{
    iox_set_pin_state(iox_port_a, GPIO_CE_PIN, false);

	nrf24l01_write_reg(WRITE_nRF_REG + STATUS, 0x70); /* Write 1 to clear status register */
	nrf24l01_delay(20);
	//nrf24l01_write_reg(WRITE_nRF_REG + CONFIG, 0x0E); /* Enable power up and ptx */
	//nrf24l01_delay(20);
	nrf24l01_write_reg(FLUSH_TX, 0x00);               /* Flush TX FIFO */
	nrf24l01_delay(20);

    /* Write data to FIFO */
	nrf24l01_spi_write(WR_TX_PLOAD, tx_buf, 10);

	nrf24l01_write_reg(WRITE_nRF_REG + STATUS, 0x20); /* Clear TX FIFO data sent status bit */
	nrf24l01_delay(200);

    iox_set_pin_state(iox_port_a, GPIO_CE_PIN, true);
	nrf24l01_delay(300000);
	iox_set_pin_state(iox_port_a, GPIO_CE_PIN, false);

}

extern uint8_t
nrf24l01_receive(uint8_t *rx_buf)
{
	uint8_t flag = 0;
    uint8_t status;

	status = nrf24l01_read_reg(STATUS);

	if (status & 0x40) /* Data Ready RX FIFO interrupt */
	{
        nrf24l01_spi_read(RD_RX_PLOAD, rx_buf, RX_PLOAD_WIDTH);
        flag = 1;
	}
    /* Write 1 to clear bit */
	nrf24l01_write_reg(WRITE_nRF_REG + STATUS, status);
	return flag;
}

/*
 * Read a register value via SPI.
 */
extern uint8_t
nrf24l01_read_reg(uint8_t reg)
{
	uint8_t value;

	iox_set_pin_state(iox_port_a, GPIO_CS_PIN, false);
	nrf24l01_delay(20);
	nrf24l01_send_byte(reg);
	value = nrf24l01_send_byte(0); /* send dummy byte */
	iox_set_pin_state(iox_port_a, GPIO_CS_PIN, true);

	return value;
}

/*
 * Write a value to a register via SPI.
 */
static uint8_t
nrf24l01_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

	iox_set_pin_state(iox_port_a, GPIO_CS_PIN, false);
    nrf24l01_delay(20);
	status = nrf24l01_send_byte(reg);
	nrf24l01_send_byte(value);
	iox_set_pin_state(iox_port_a, GPIO_CS_PIN, true);

	return status;
}

/*
 * Return value from a register via SPI.
 */
static uint8_t
nrf24l01_spi_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t status, i;

	iox_set_pin_state(iox_port_a, GPIO_CS_PIN, false);
	nrf24l01_delay(20);
	status = nrf24l01_send_byte(reg);

    for (i = 0; i < len; i++) {
        buf[i] = nrf24l01_send_byte(0);
    }

	iox_set_pin_state(iox_port_a, GPIO_CS_PIN, true);

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
	nrf24l01_delay(20);
	status = nrf24l01_send_byte(reg);
	for (i = 0; i < len; i++) {
		nrf24l01_send_byte(*buf);
		buf++;
	}
	iox_set_pin_state(iox_port_a, GPIO_CS_PIN, true);

	return status;
}

/*
 * Send byte to nRF24L01 via SPI
 * Returns the acknowledgement.
 */
static uint8_t
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

static void
nrf24l01_delay(unsigned long n)
{
	unsigned long i;

	while (n--)  // delay n us
	{
        i = 100;
        while (i--); // delay 1 us
    }
}
