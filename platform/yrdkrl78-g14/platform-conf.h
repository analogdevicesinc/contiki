/*
 * Copyright (c) 2014, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \author Ian Martin <martini@redwirellc.com>
 */

#ifndef NEWLINE
#define NEWLINE "\r\n"
#endif

#ifndef BIT
#define BIT(n) (1 << (n))
#endif

#define BAUD2UBR(x) (x)

#define PLATFORM_HAS_LEDS   0
#define PLATFORM_HAS_BUTTON 0

#define LED1  P1bits.bit1
#define LED2  P1bits.bit0
#define LED3  P6bits.bit2
#define LED4  P4bits.bit2
#define LED5  P6bits.bit3
#define LED6  P4bits.bit3
#define LED7  P6bits.bit4
#define LED8  P4bits.bit4
#define LED9  P6bits.bit5
#define LED10 P4bits.bit5
#define LED11 P6bits.bit6
#define LED12 P15bits.bit2
#define LED13 P6bits.bit7
#define LED14 P10bits.bit1
#define LED15 P4bits.bit1

#define PMOD2_P9 P11bits.bit0
#define SWITCH1 P7bits.bit6
#define SWITCH2 P7bits.bit4
#define SWITCH3 P7bits.bit5

#define HEARTBEAT_LED1 LED6
#define HEARTBEAT_LED2 LED15
#define RADIO_TX_LED   LED13
#define RADIO_RX_LED   LED14
#define UART_RX_LED    LED2

#define RL78_NB_PINS 100

#define MAX_DEVICES_PER_SPI 2
#define PLATFORM_USE_UART1 1
/* #define PLATFROM_HAS_I2C */
