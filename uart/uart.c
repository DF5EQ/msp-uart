/* ===== file header ===== */
/*************************************************************************
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

*************************************************************************/

/*************************************************************************
uart_available and uart_flush functions were adapted from the Arduino
HardwareSerial.h library by Tim Sharpe on 11 Jan 2009.
The license info for HardwareSerial.h is as follows:

  HardwareSerial.cpp - Hardware serial library for Wiring
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

  Modified 23 November 2006 by David A. Mellis
*************************************************************************/

/* ===== includes ===== */
#include "uart.h"

/* ===== private datatypes ===== */

/* ===== private symbols ===== */

/* see SLAU367P chapter 30.3.10 Setting a Baud Rate */
#define NN (UART_BRCLK/UART_BAUDRATE)
#if NN > 17
    #define OS16 1
    #define UCBRx NN/16
    #define UCBRFx NN%16
#else
    #define OS16 0
    #define UCBRx NN
    #define UCBRFx 0
#endif
#define FRACT (10000ULL * UART_BRCLK / UART_BAUDRATE - UART_BRCLK / UART_BAUDRATE * 10000ULL)
#if FRACT < 529
    #define UCBRSx 0x00
#elif FRACT < 715
    #define UCBRSx 0x01
#elif FRACT < 835
    #define UCBRSx 0x02
#elif FRACT < 1001
    #define UCBRSx 0x04
#elif FRACT < 1252
    #define UCBRSx 0x08
#elif FRACT < 1430
    #define UCBRSx 0x10
#elif FRACT < 1670
    #define UCBRSx 0x20
#elif FRACT < 2147
    #define UCBRSx 0x11
#elif FRACT < 2224
    #define UCBRSx 0x21
#elif FRACT < 2503
    #define UCBRSx 0x22
#elif FRACT < 3000
    #define UCBRSx 0x44
#elif FRACT < 3335
    #define UCBRSx 0x25
#elif FRACT < 3575
    #define UCBRSx 0x49
#elif FRACT < 3753
    #define UCBRSx 0x4a
#elif FRACT < 4003
    #define UCBRSx 0x52
#elif FRACT < 4286
    #define UCBRSx 0x92
#elif FRACT < 4378
    #define UCBRSx 0x53
#elif FRACT < 5002
    #define UCBRSx 0x55
#elif FRACT < 5715
    #define UCBRSx 0xaa
#elif FRACT < 6003
    #define UCBRSx 0x6b
#elif FRACT < 6254
    #define UCBRSx 0xad
#elif FRACT < 6432
    #define UCBRSx 0xb5
#elif FRACT < 6667
    #define UCBRSx 0xb6
#elif FRACT < 7001
    #define UCBRSx 0xd6
#elif FRACT < 7147
    #define UCBRSx 0xb7
#elif FRACT < 7503
    #define UCBRSx 0xbb
#elif FRACT < 7861
    #define UCBRSx 0xdd
#elif FRACT < 8004
    #define UCBRSx 0xed
#elif FRACT < 8333
    #define UCBRSx 0xee
#elif FRACT < 8464
    #define UCBRSx 0xbf
#elif FRACT < 8572
    #define UCBRSx 0xdf
#elif FRACT < 8751
    #define UCBRSx 0xef
#elif FRACT < 9004
    #define UCBRSx 0xf7
#elif FRACT < 9170
    #define UCBRSx 0xfb
#elif FRACT < 9288
    #define UCBRSx 0xfd
#elif FRACT < 10000
    #define UCBRSx 0xfe
#else
    #error no lookup value for UCBRSx
#endif

/* baudrate settings */
#define UCBRS(x)  ((x)<<8)
#define UCBRF(x)  ((x)<<4)
#undef UCOS16 /* defined in msp430.h -> msp430fr5969.h */
#define UCOS16(x) ((x)<<0)

#if UART_BAUDRATE == 9600
    #define UCAxBRW     6
    #define UCAxMCTLW   ( UCBRS(0x20) | UCBRF(8) | UCOS16(1) )
#elif UART_BAUDRATE == 19200
    #define UCAxBRW    3
    #define UCAxMCTLW  ( UCBRS(0x02) | UCBRF(4) | UCOS16(1) )
#elif UART_BAUDRATE == 38400
    #define UCAxBRW    1
    #define UCAxMCTLW  ( UCBRS(0x00) | UCBRF(10) | UCOS16(1) )
#elif UART_BAUDRATE == 57600
    #define UCAxBRW    17
    #define UCAxMCTLW  ( UCBRS(0x4a) | UCBRF(0) | UCOS16(0) )
#elif UART_BAUDRATE == 115200
    #define UCAxBRW   8
    #define UCAxMCTLW ( UCBRS(0xd6) | UCBRF(0) | UCOS16(0) )
#else
    #error Baudrate not supported
#endif

/* size of RX/TX buffers */
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)

#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK)
	#error RX buffer size is not a power of 2
#endif

#if (UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK)
	#error TX buffer size is not a power of 2
#endif

/* ===== private constants ===== */

/* ===== public constants ===== */

/* ===== private variables ===== */
static volatile uint8_t UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile uint8_t UART_RxBuf[UART_RX_BUFFER_SIZE];
static volatile uint8_t UART_TxHead;
static volatile uint8_t UART_TxTail;
static volatile uint8_t UART_RxHead;
static volatile uint8_t UART_RxTail;
static volatile uint8_t UART_LastRxError;

/* ===== public variables ===== */

/* ===== private functions ===== */

static void uart_rx (void)
/*************************************************************************
Function: UART Receive Complete interrupt
Purpose:  called when the UART has received a character
**************************************************************************/
{
    uint16_t tmphead;
    uint8_t data;
    uint8_t usr;
    uint8_t lastRxError;

    /* read UART status register and UART data register */
    data        = UCA0RXBUF;
    usr         = 0; /* TODO which register is here equivalent to avr's USART0_RXDATAH resp. UART0_STATUS ? */
    lastRxError = 0; /* TODO what is here equivalent to avr ? */

    /* calculate buffer index */
    tmphead = (UART_RxHead + 1) & UART_RX_BUFFER_MASK;

    if (tmphead == UART_RxTail)
    {
        /* error: receive buffer overflow */
        lastRxError = UART_BUFFER_OVERFLOW >> 8;
    }
    else
    {
        /* store new index */
        UART_RxHead = tmphead;

        /* store received data in buffer */
        UART_RxBuf[tmphead] = data;
    }
    UART_LastRxError = lastRxError;
}

static void uart_tx (void)
/*************************************************************************
Function: UART Data Register Empty interrupt
Purpose:  called when the UART is ready to transmit the next byte
**************************************************************************/
{
    uint16_t tmptail;

    if (UART_TxHead != UART_TxTail)
    {
        /* calculate and store new buffer index */
        tmptail = (UART_TxTail + 1) & UART_TX_BUFFER_MASK;
        UART_TxTail = tmptail;

        /* get one byte from buffer and write it to UART */
        UCA0TXBUF = UART_TxBuf[tmptail];  /* start transmission */
    }
    else
    {
        /* tx buffer empty, disable interrupt */
        UCA0IE &= ~UCTXIE;
    }
}

/* ===== interrupt functions ===== */

#pragma vector = USCI_A0_VECTOR
__interrupt void uart_interrupt (void)
{
    switch(UCA0IV)
    {
        case 0x00:  // Vector 0: No interrupts
            break;
        case 0x02:  // Vector 2: UCRXIFG
            uart_rx();
            break;
        case 0x04:  // Vector 4: UCTXIFG
            uart_tx();
            break;
        case 0x06:  // Vector 6: UCSTTIFG
            break;
        case 0x08:  // Vector 8: UCTXCPTIFG
            break;
        default:
            break;
    }
}

/* ===== public functions ===== */
void uart_debug (void)
{
    char buffer[30];

     itoa(OS16,   buffer, 10); uart_puts("OS16   "); uart_puts(buffer); uart_puts("\r\n");
    ultoa(UCBRx,  buffer, 10); uart_puts("UCBRx  "); uart_puts(buffer); uart_puts("\r\n");
    ultoa(UCBRFx, buffer, 10); uart_puts("UCBRFx "); uart_puts(buffer); uart_puts("\r\n");
     itoa(UCBRSx, buffer, 16); uart_puts("UCBRSx "); uart_puts(buffer); uart_puts("\r\n");
}
/*************************************************************************
Function: uart_init()
Purpose:  initialize UART and set baudrate
Input:    baudrate using macro UART_BAUD_SELECT()
Returns:  none
**************************************************************************/
void uart_init(uint16_t baudrate)
{
	/* set heads and tails to initial positions */
    UART_TxHead = 0;
    UART_TxTail = 0;
    UART_RxHead = 0;
    UART_RxTail = 0;

    /* attach port pins to UART eUSCI_A0 */
    P2SEL1 |=   BIT0 | BIT1;
    P2SEL0 &= ~(BIT0 | BIT1);

    /* configure eUSCI_A0 for UART mode 9600 8N1 */
    UCA0CTLW0  = UCSWRST;       /* put eUSCI in reset */
    UCA0CTLW0 |= UCSSEL__SMCLK; /* BRCLK = SMCLK */
    UCA0BRW    = UCAxBRW;       /* see SLAU367P table 30-5 */
    UCA0MCTLW  = UCAxMCTLW;     /* see SLAU367P table 30-5 */
    UCA0CTLW0 &= ~UCSWRST;      /* release eUSCI from reset */

    /* enable receive interrupt */
    UCA0IE |= UCRXIE;
}

/*************************************************************************
Function: uart_getc()
Purpose:  return byte from ringbuffer
Returns:  lower byte:  received byte from ringbuffer
          higher byte: last receive error
**************************************************************************/
uint16_t uart_getc(void)
{
	uint16_t tmptail;
	uint8_t data;

    /* TODO uart rx interrupt should be disabled during this function             */
    /*      so that UART_RxHead and UART_RxTail are consistent throughout the run */

    if (UART_RxHead == UART_RxTail)
    {
        /* no data available */
        return UART_NO_DATA;
	}

	/* calculate / store buffer index */
	tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;
	UART_RxTail = tmptail;

	/* get data from receive buffer */
	data = UART_RxBuf[tmptail];

	return (UART_LastRxError << 8) + data;
}

/*************************************************************************
Function: uart_peek()
Purpose:  Returns the next byte (character) of incoming UART data without
          removing it from the ring buffer. That is, successive calls to
		  uartN_peek() will return the same character, as will the next
		  call to uartN_getc()
Returns:  lower byte:  next byte in ring buffer
          higher byte: last receive error
**************************************************************************/
uint16_t uart_peek(void)
{
	uint16_t tmptail;
	uint8_t data;

    /* TODO uart rx interrupt should be disabled during this function             */
    /*      so that UART_RxHead and UART_RxTail are consistent throughout the run */

    if (UART_RxHead == UART_RxTail)
    {
        /* no data available */
        return UART_NO_DATA;
	}

	/* calculate buffer index */
	tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;

	/* get data from receive buffer */
	data = UART_RxBuf[tmptail];

	return (UART_LastRxError << 8) + data;
}

/*************************************************************************
Function: uart_putc()
Purpose:  write byte to ringbuffer for transmitting via UART
Input:    byte to be transmitted
Returns:  none
**************************************************************************/
void uart_putc(uint8_t data)
{
	uint16_t tmphead;

	/* calculate buffer index */
	tmphead = (UART_TxHead + 1) & UART_TX_BUFFER_MASK;

    /* wait for free space in buffer */
	while (tmphead == UART_TxTail);

	/* put data to transmit buffer */
	UART_TxBuf[tmphead] = data;
	UART_TxHead = tmphead;

    /* if transmit interrupt not enabled */
    if( (UCA0IE & UCTXIE) == 0 )
    {
        /* enable interrupt */
        UCA0IFG |= UCTXIFG; /* set transmit interrupt flag */
        UCA0IE  |= UCTXIE;  /* enable transmit interrupt */
    }
}

/*************************************************************************
Function: uart_puts()
Purpose:  transmit string to UART
Input:    string to be transmitted
Returns:  none
**************************************************************************/
void uart_puts(const char *s)
{
	while (*s)
    {
        uart_putc(*s++);
	}
}

/*************************************************************************
Function: uart_available()
Purpose:  Determine the number of bytes waiting in the receive buffer
Input:    None
Returns:  Integer number of bytes in the receive buffer
**************************************************************************/
uint16_t uart_available(void)
{
	uint16_t ret;

    /* TODO uart rx interrupt should be disabled during this function             */
    /*      so that UART_RxHead and UART_RxTail are consistent throughout the run */

    ret = (UART_RX_BUFFER_SIZE + UART_RxHead - UART_RxTail) & UART_RX_BUFFER_MASK;

	return ret;
}

/*************************************************************************
Function: uart_flush()
Purpose:  Flush bytes waiting the receive buffer. Actually ignores them.
Input:    None
Returns:  None
**************************************************************************/
void uart_flush(void)
{
    /* TODO uart rx interrupt should be disabled during this function             */
    /*      so that UART_RxHead and UART_RxTail are consistent throughout the run */
    UART_RxHead = UART_RxTail;
}

