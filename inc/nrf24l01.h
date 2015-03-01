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


/* Function Declarations -----------------------------------------------------*/
extern void nrf24l01_init(void);
extern void nrf24l01_tx_mode(void);
extern void nrf24l01_transmit(uint8_t *tx_buf);
extern uint8_t nrf24l01_receive(uint8_t *rx_buf);

extern uint8_t nrf24l01_read_reg(uint8_t reg);

#endif