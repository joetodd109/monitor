/**
  ******************************************************************************
  * @file    nrf24l01.h 
  * @author  Joe Todd
  * @version 
  * @date    
  * @brief   Header for nrf24l01.c
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef NRF24L01_H
#define NRF24L01_H

/* Includes ------------------------------------------------------------------*/
#include "iox.h"
#include "utl.h"
#include "timer.h"

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
#define WR_TX_NO_ACK	0xB0  /* Command to write Tx payload with no ACK */
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

#define ADR_WIDTH       5u

/* Function Declarations -----------------------------------------------------*/
extern void nrf24l01_init(void);
extern void nrf24l01_tx_mode(void);
extern bool nrf24l01_transmit(uint8_t *tx_buf);
extern bool nrf24l01_receive(uint8_t *rx_buf);
extern uint8_t nrf24l01_send_byte(uint8_t data);
extern uint8_t nrf24l01_read_reg(uint8_t reg);
extern uint8_t nrf24l01_write_reg(uint8_t reg, uint8_t value);
extern uint8_t nrf24l01_spi_read(uint8_t reg, uint8_t *buf, uint8_t len);

/* for debugging */
extern void print_status(uint8_t reg);
extern void print_byte(uint8_t byte);
extern void print_regs(void);
#endif