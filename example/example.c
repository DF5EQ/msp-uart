/* ===== file header ===== */
/*************************************************************************

Example program, based on Andy Gock's example.
Example program, based on Peter Fluery's example.

*************************************************************************/

/*************************************************************************

Title:    Example program for the Interrupt controlled UART library
Author:   Peter Fleury <pfleury@gmx.ch>   http://tinyurl.com/peterfleury
Software: msp430-gcc
Hardware: MSP403FR5969 may be usable for other MSP430

DESCRIPTION: This example shows how to use the UART module uart.c and uart.h

*************************************************************************/

/* ===== includes ===== */
#include <stdlib.h>
#include <msp430.h>
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>

#include "system.h"
#include "uart.h"

/* ===== private datatypes ===== */

/* ===== private symbols ===== */
/* Define CPU frequency in Hz in Makefile or toolchain compiler configuration */
//#ifndef F_CPU
//#error "F_CPU undefined, please define CPU frequency in Hz in Makefile or compiler configuration"
//#endif

/* ===== private constants ===== */

/* ===== public constants ===== */

/* ===== private variables ===== */

/* ===== public variables ===== */

/* ===== private functions ===== */

/* ===== public functions ===== */

int main(void)
{
    uint16_t c;
    char buffer[7];
    int8_t num = 42;

    system_init();
    uart_init();

    /* enable interrupt, since UART module is interrupt controlled */
    system_interrupts_enable();

    /* Transmit string to UART                                                         */
    /* The string is buffered by the uart module in a circular buffer and              */
    /* one character at a time is transmitted to the UART using interrupts.            */
    /* uart_puts() blocks if it can not write the whole string to the circular buffer. */
    uart_puts("\r\nHello World!\r\n");

    /* Use standard functions to convert numbers into string before transmitting */
    itoa(num, buffer, 10); // convert integer into string (decimal format)
    uart_puts(buffer);     // and transmit string to UART

    /* Transmit single character to UART */
    uart_putc('\r');
    uart_putc('\n');

    while (1)
    {
        /* Get received character from ringbuffer.                      */
        /* uart_getc() returns in the lower byte the received character */
        /* and in the higher byte (bitmask) the last receive error.     */
        /* UART_NO_DATA is returned when no data is available.          */
        c = uart_getc();
        if (c & UART_NO_DATA)
        {
            /* No data available from UART */
        }
        else
        {
            /* New data available from UART */
            /* Check for Frame or Overrun error */
            if (c & UART_FRAME_ERROR)
            {
                /* Framing Error detected, i.e no stop bit detected */
                uart_puts("UART Frame Error: ");
            }
            if (c & UART_OVERRUN_ERROR)
            {
                /* Overrun, a character already present in the UART register was        */
                /* not read by the interrupt handler before the next character arrived, */
                /* one or more received characters have been dropped                    */
                uart_puts("UART Overrun Error: ");
            }
            if (c & UART_BUFFER_OVERFLOW)
            {
                /* We are not reading the receive buffer fast enough, */
                /* one or more received character have been dropped   */
                uart_puts("Buffer overflow error: ");
            }

            /* Send received character back */
            uart_putc(c);
        }
    }
}
