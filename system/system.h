#ifndef SYSTEM_H
#define SYSTEM_H

/* ===== file header ===== */

/* ===== includes ===== */
#include <msp430.h>

/* ===== public datatypes ===== */

/* ===== public symbols ===== */
#define SYSTEM_DCOCLK   16000000
#define SYSTEM_DIVM     1
#define SYSTEM_DIVS     16
#define SYSTEM_MCLK     (SYSTEM_DCOCLK/SYSTEM_DIVM)
#define SYSTEM_SMCLK    (SYSTEM_DCOCLK/SYSTEM_DIVS)

/* ===== public constants ===== */

/* ===== public variables ===== */

/* ===== public functions ===== */
#define system_interrupts_enable()  __bis_status_register(GIE)
#define system_interrupts_disable() __bic_status_register(GIE)

void system_init (void);

#endif

