/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UTL_H
#define UTL_H

/* Includes ------------------------------------------------------------------*/
#include "misc.h"
#include <stdint.h>


/**
 * Enable an interrupt.
 */
extern void utl_enable_irq(uint8_t irq);

/**
 * Disable an interrupt.
 */
extern void utl_disable_irq(uint8_t irq);

#endif /* __UTL_H */