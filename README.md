# msp-uart

A port of the original avr-uart from Andy Gock to MSP430 controller.

## avr-uart

An interrupt driven UART Library for 8-bit AVR microcontrollers.

Maintained by Andy Gock.

<https://github.com/andygock/avr-uart>

Derived from original library by [Peter Fleury](http://www.peterfleury.epizy.com/avr-software.html).

Interrupt driven UART library using the built-in UART with circular transmit and receive buffers.

An interrupt is generated when the UART has finished transmitting or
receiving a byte. The interrupt handling routines use circular buffers
for buffering received and transmitted data.

## Setting up

### Define UARTs enabled and buffer sizes

The `UART_RX_BUFFER_SIZE` and `UART_TX_BUFFER_SIZE` symbols define
the size of the circular buffers in bytes.
These values **must be a power of 2**.
You may need to adapt this symbols to your target and your application by adding into your compiler options:

    -DUART_RX_BUFFER_SIZE=nn -DUART_TX_BUFFER_SIZE=nn

This module supports MSP430 devices with build in UART.
Tested only for MSP430FR5969 but should work for other MSP430.

### Define UART baudrate generator frequency

Define `UART_BRCLK` in your Makefile or compiler examples.
Example, if you're running a 1 MHz baudrate generator clock, then use:

    -DUART_BRCLK=1000000UL

### Compiler flags

msp430-gcc compiler requires no special flags concerning this module.

## Documentation

see comments in uart.h and uart.c

## Notes

### Buffer overflow behaviour

When the RX circular buffer is full, and it receives further data from the UART, a buffer overflow condition occurs. Any new data is dropped. The RX buffer must be read before any more incoming data from the UART is placed into the RX buffer.

If the TX buffer is full, and new data is sent to it using one of the `uart_put*()` functions, this function will loop and wait until the buffer is not full any more. It is important to make sure you have not disabled your UART transmit interrupts elsewhere in your application before calling the `uart_put*()` functions, as the application will lock up. The UART interrupts are automatically enabled when you use the `uart_init()` function. This is probably not the ideal behaviour, maybe changed some time.

For now, make sure interrupts are enabled when calling `uart_put*()` functions. This should not be an issue unless you have code elsewhere purposely turning it off.
