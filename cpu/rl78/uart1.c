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
 * \author Maxim Salov <max.salov@gmail.com>, Ian Martin <martini@redwirellc.com>
 */

#include "rl78.h"     /* for f_CLK */
#include "sfrs.h"
#include "sfrs-ext.h"

#include "uart1.h"

#define DESIRED_BAUDRATE 38400

/* Note that only 9600, 38400, and 115200 bps were tested. */
#define PRESCALE_THRESH  ((38400 + 115200) / 2)
#define PRS_VALUE        ((DESIRED_BAUDRATE < PRESCALE_THRESH) ? 4 : 0)
#define f_MCK            (f_CLK / (1 << PRS_VALUE))
#define SDR_VALUE        (f_MCK / DESIRED_BAUDRATE / 2 - 1)

#define BIT(x) (1 << (x))

void
uart1_stop(void)
{
  ST0 = 0x0cU; /* Stop operation of channel 2 and 3 */

  SOE0 &= ~4; /* Disable UART1 output */

  /* Setup interrupts (disable) */
  STMK1 = 1;   /* Disable INTST0 interrupt */
  STIF1 = 0;   /* Clear INTST0 interrupt request flag */
#if 0
  STPR11 = 1;  /* Set INTST0 priority: lowest  */
  STPR01 = 1;
#endif

  SRMK1 = 1;   /* Disable INTSR0 interrupt */
  SRIF1 = 0;   /* Clear INTSR0 interrupt request flag */
#if 0
  SRPR11 = 1;  /* Set INTSR0 priority: lowest */
  SRPR01 = 1;
#endif

  SREMK1 = 1;  /* Disable INTSRE0 interrupt */
  SREIF1 = 0;  /* Clear INTSRE0 interrupt request flag */
#if 0
  SREPR10 = 1; /* Set INTSRE0 priority: lowest */
  SREPR00 = 1;
#endif
}

void
uart1_init(void)
{
  /* Reference R01AN0459EJ0100 or hardware manual for details */

  /* Disable IO redirection */
  PIOR1 = 0U;

  /* Set RxD1, TxD1 pins */
  PM0 |= 0x0cU;

  /* Enable clock to serial array 0 */
  SAU0EN = 1;

  /* Set input clock (CK00 and CK01) to fclk/16 = 2MHz */
  SPS0 = (PRS_VALUE << 4) | PRS_VALUE;

  uart1_stop();

  /* ------------------------------------------------ */
  /* Setup operation mode for transmitter (channel 2) */
  /* ------------------------------------------------ */

  /* Operation clock: CK00,
     Transfer clock: division of CK00
     Start trigger: software
     Detect falling edge as start bit
     Operation mode: UART
     Interrupt source : buffer empty */
  SMR02 = 0x0023U;

  /* Transmission only
     Reception error interrupt masked
     Phase clock : type 1
     No parity
     LSB first
     1 stop bit
     8-bit data length */
  SCR02 = 0x8097U;
  SDR02 = SDR_VALUE << 9;

  /* --------------------------------------------- */
  /* Setup operation mode for receiver (channel 3) */
  /* --------------------------------------------- */

  /* Enable noise filter on RxD1 pin */
  NFEN0 |= 0x04U;

  /* Clear error flags */
  SIR03 = 0x0007U;

  /* Operation clock : CK00
     Transfer clock : division of CK00
     Start trigger : valid edge on RxD pin
     Detect falling edge as start bit
     Operation mode : UART
     Interrupt source : transfer end */
  SMR03 = 0x0122U;

  /* Reception only
     Reception error interrupt masked
     Phase clock : type 1
     No parity
     LSB first
     1 stop bit
     8-bit data length */
  SCR03 = 0x4097U;
  SDR03 = SDR_VALUE << 9;

  /* Enable TX and RX channels */
  SO0 |= 0x4;
  SOE0 |= 0x4;

  /* Set TxD1 high */
  P0 |= BIT(2);

  /* Set TxD1 pin as output */
  PM0 &= ~BIT(2);

  /* Set RxD1 as input */
  PM0 |= BIT(3);

  /* Enable UART1 receive and transmit */
  SS0 |= 0x0cU;

  /* Set buffer empty interrupt request flag */
  STIF1 = 1;
}
void
uart1_putchar(int c)
{
  while(0 == STIF1) ;
  STIF1 = 0;
  SDR02 = c;
}
char
uart1_getchar(void)
{
  char c;
  while(!uart1_can_getchar()) ;
  c = SDR03;
  SRIF0 = 0;
  return c;
}
