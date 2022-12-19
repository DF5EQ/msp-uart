/* ===== file header ===== */
/************************************************************************
Title:    Interrupt UART module with receive/transmit circular buffers
Author:   Peter Bägel
Software: msp430-gcc
Hardware: MSP430, tested on EXP430FR5969
License:  GNU General Public License
Usage:    see README.md

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
************************************************************************/

/************************************************************************
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
************************************************************************/

/**
 *  This library can be used to transmit and receive data through the built in UART.
 *
 *  An interrupt is generated when the UART has finished transmitting or
 *  receiving a byte. The interrupt handling routines use circular buffers
 *  for buffering received and transmitted data.
 *
 *  The UART_RXn_BUFFER_SIZE and UART_TXn_BUFFER_SIZE constants define
 *  the size of the circular buffers in bytes. Note that these constants must be a power of 2.
 *
 *  You need to define these buffer sizes as a symbol in your compiler settings or in uart.h
 *
 *  See README.md for more detailed information. Especially that relating to symbols: USARTn_ENABLED and USARTn_LARGE_BUFFER
 */

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
#define UART_RX_BUFFER_SIZE 128 /**< Size of the circular receive buffer, must be power of 2 */
#define UART_TX_BUFFER_SIZE 128 /**< Size of the circular transmit buffer, must be power of 2 */

/* high byte error return code of uart_getc() */
#define UART_FRAME_ERROR      0x0800 /**< Framing Error by UART       */
#define UART_OVERRUN_ERROR    0x0400 /**< Overrun condition by UART   */
#define UART_BUFFER_OVERFLOW  0x0200 /**< receive ringbuffer overflow */
#define UART_NO_DATA          0x0100 /**< no receive data available   */

/* ===== public constants ===== */

/* ===== public variables ===== */

/* ===== public functions ===== */
/**
    Initialize UART and set baudrate
    parameters: UART_BRCLK and UART_BAUDRATE as #define
    return    : none
*/
extern void uart_init(void);

/**
 * Get received byte from ringbuffer
 *
 * Returns in the lower byte the received character and in the
 * higher byte the last receive error.
 * UART_NO_DATA is returned when no data is available.
 *
 *  return: lower byte:  received byte from ringbuffer
 *          higher byte: last receive status
 *                       0 - successfully received data from UART
 *            UART_NO_DATA - no receive data available
 *    UART_BUFFER_OVERFLOW - receive ringbuffer overflow
 *                           We are not reading the receive buffer fast enough,
 *                           one or more received character have been dropped
 *      UART_OVERRUN_ERROR - Overrun condition by UART
 *                           A character already present in the UART register was
 *                           not read by the interrupt handler before the next character arrived,
 *                           one or more received characters have been dropped.
 *        UART_FRAME_ERROR - Framing Error by UART
 */
extern uint16_t uart_getc(void);

/**
 * Peek at next byte in ringbuffer
 *
 * Returns the next byte (character) of incoming UART data without removing it from the
 * internal ring buffer. That is, successive calls to uart_peek() will return the same
 * character, as will the next call to uart_getc().
 *
 * return: lower byte:  next byte in ringbuffer
 *         higher byte: last receive status
 *                      0 - successfully received data from UART
 *           UART_NO_DATA - no receive data available
 *   UART_BUFFER_OVERFLOW - Receive ringbuffer overflow
 *                          We are not reading the receive buffer fast enough,
 *                          one or more received character have been dropped
 *     UART_OVERRUN_ERROR - Overrun condition by UART
 *                          A character already present in the UART UDR register was
 *                          not read by the interrupt handler before the next character arrived,
 *                          one or more received characters have been dropped.
 *       UART_FRAME_ERROR - Framing Error by UART
 */
extern uint16_t uart_peek(void);

/**
 *  @brief   Put byte to ringbuffer for transmitting via UART
 *  @param   data byte to be transmitted
 *  @return  none
 */
extern void uart_putc(uint8_t data);

/**
 * Put string to ringbuffer for transmitting via UART
 *
 * The string is buffered by the uart library in a circular buffer
 * and one character at a time is transmitted to the UART using interrupts.
 * Blocks if it can not write the whole string into the circular buffer.
 *
 * parameters: s - string to be transmitted
 * return    : none
 */
extern void uart_puts(const char *s);

/**
 * Put string from program memory to ringbuffer for transmitting via UART.
 *
 * The string is buffered by the uart library in a circular buffer
 * and one character at a time is transmitted to the UART using interrupts.
 * Blocks if it can not write the whole string into the circular buffer.
 *
 * parameters: s - program memory string to be transmitted
 * return    : none 
 */
extern void uart_puts_p(const char *s);

/**
 * Return number of bytes waiting in the receive buffer
 * parameters: none
 * return    : bytes waiting in the receive buffer
 */
extern uint16_t uart_available(void);

/**
 * Flush bytes waiting in receive buffer
 */
extern void uart_flush(void);

#endif /* UART_H */

