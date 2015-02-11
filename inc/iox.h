/**
  ******************************************************************************
  * @file    iox.h 
  * @author  Joe Todd
  * @version 
  * @date    
  * @brief   Header for iox.c
  *
  ******************************************************************************
*/
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IOX_H
#define IOX_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"


/* Global Defines ----------------------------------------------------------- */

/* 
 * Pin definitions 
 */
#define PIN0    0u
#define PIN1    1u
#define PIN2    2u
#define PIN3    3u
#define PIN4    4u
#define PIN5    5u
#define PIN6    6u
#define PIN7    7u
#define PIN8    8u
#define PIN9    9u
#define PIN10   10u
#define PIN11   11u
#define PIN12   12u
#define PIN13   13u
#define PIN14   14u
#define PIN15   15u

/* Alternate function defintions */
#define AF0     0u
#define AF1     1u
#define AF2     2u
#define AF3     3u
#define AF4     4u
#define AF5     5u
#define AF6     6u
#define AF7     7u
#define AF8     8u
#define AF9     9u
#define AF10    10u
#define AF11    11u
#define AF12    12u
#define AF13    13u
#define AF14    14u
#define AF15    15u


/**
 * Macro to get the pin state.
 *
 * The compiler does a good job on this, so it is worth using a macro.
 */
#define iox_get_pin(port, pin) \
    ((iox_gpios[port]->IDR & (1u << (pin))) ? true : false)

/**
 * Macro to set the pin state.
 *
 * The compiler does a good job on this, so it is worth using a macro.
 * @notes:  upgrade to atomic write.
 */
#define iox_set_pin(port, pin) \
    (iox_gpios[port]->BSRRL = (1u << (pin)))

#define iox_reset_pin(port, pin) \
    (iox_gpios[port]->BSRRH = (1u << (pin)))

typedef enum {
    iox_port_a,
    iox_port_b,
    iox_port_c,
    iox_port_d,
    iox_port_e,
} iox_port_t;

typedef struct iox_port_pin_t iox_port_pin_t;

struct iox_port_pin_t {
    iox_port_t port;
    uint8_t pin;
};

typedef enum {
    iox_mode_in,
    iox_mode_out,
    iox_mode_af,
    iox_mode_analog,
} iox_mode_t;

typedef enum {
    iox_type_pp,
    iox_type_od,
} iox_type_t;

typedef enum {
    iox_speed_low,
    iox_speed_med,
    iox_speed_fast,
    iox_speed_high,
} iox_speed_t;

typedef enum {
    iox_pupd_none,
    iox_pupd_up,
    iox_pupd_down,
} iox_pupd_t;


/*
 * Used by the macros above.  
 */
extern GPIO_TypeDef *const iox_gpios[];

/**
 * Configure a port pin.
 */
extern void iox_configure_pin(iox_port_t port, uint32_t pin,
                            iox_mode_t mode, iox_type_t type, 
                            iox_speed_t speed, iox_pupd_t pupd);

/*
 * Configure a pin for an alternate function
 */
extern void iox_alternate_func(iox_port_t port, uint32_t pin,
                                uint32_t af);

/**
 * Change an output pin state.
 */
extern void (iox_set_pin_state) (iox_port_t port, uint32_t pin,
                                 bool state);

/**
 * Read the state of an input pin.
 */
extern bool(iox_get_pin_state) (iox_port_t port, uint32_t pin);

/* 
 * Sets up GPIO's for LED's
 */
extern void iox_led_init(void);

/**
 * Turn on an LED
 */
extern void iox_led_on(bool green, bool amber, bool red, bool blue);

/**
 * Turn off all LEDs
 */
extern void iox_leds_off(void);


#endif