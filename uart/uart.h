/* ===== file header ===== */
/*************************************************************************
Title   : Interrupt UART module with receive/transmit circular buffers
Author  : Peter Bägel
Software: msp430-gcc
Hardware: MSP430, tested on EXP430FR5969
License : GNU General Public License
Usage   : see README.md

Based on original library by
    Andy Gock
    Peter Fluery
    Tim Sharpe
    Nicholas Zambetti

https://github.com/andygock/avr-uart
http://beaststwo.org/avr-uart/index.shtml
http://www.peterfleury.epizy.com/avr-software.html

LICENSE:
    Copyright (C) 2022 Peter Bägel (DF5EQ)
	Copyright (C) 2012 Andy Gock
	Copyright (C) 2006 Peter Fleury

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
*************************************************************************/

/*************************************************************************
uart_available and uart_flush functions were adapted from the Arduino
HardwareSerial.h library by Tim Sharpe on 11 Jan 2009.
The license info for HardwareSerial.h is as follows:

  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*************************************************************************/

/*************************************************************************
This module can be used to transmit and receive data through the built in UART.

An interrupt is generated when the UART has finished transmitting or
receiving a byte. The interrupt handling routines use circular buffers
for buffering received and transmitted data.

UART_RX_BUFFER_SIZE and UART_TX_BUFFER_SIZE define the size of the
circular buffers in bytes. Note that these constants must be a power of 2.

You need to define these buffer sizes as a symbol in your compiler
settings or in uart.h

See README.md for more detailed information.
*************************************************************************/

#ifndef UART_H
#define UART_H

/* ===== includes ===== */
#include <msp430.h>
#include <stdint.h>

/* ===== public datatypes ===== */

/* ===== public symbols ===== */

/* Set baudrate */
#define UART_BRCLK 1000000 /* baudrate generator clock in hz */
#define UART_BAUDRATE 9600 /* baudrate in bit per second */

/* Set size of receive and transmit buffers */
#define UART_RX_BUFFER_SIZE 128 /* Size of the circular receive buffer, must be power of 2 */
#define UART_TX_BUFFER_SIZE 128 /* Size of the circular transmit buffer, must be power of 2 */

/* high byte error return code of uart_getc() */
#define UART_FRAME_ERROR      0x0800 /* Framing Error by UART       */
#define UART_OVERRUN_ERROR    0x0400 /* Overrun condition by UART   */
#define UART_BUFFER_OVERFLOW  0x0200 /* receive ringbuffer overflow */
#define UART_NO_DATA          0x0100 /* no receive data available   */

/* ===== public constants ===== */

/* ===== public variables ===== */

/* ===== public functions ===== */
#define putchar uart_putc
#define getchar uart_getc
#define puts    uart_puts

void     uart_init      (void);
uint16_t uart_getc      (void);
uint16_t uart_peek      (void);
void     uart_putc      (uint8_t data);
void     uart_puts      (const char *s);
uint16_t uart_available (void);
void     uart_flush     (void);

#endif

