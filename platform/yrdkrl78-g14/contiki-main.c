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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "adf7242.h"
#include "contiki.h"

#include "dev/button-sensor.h"
#include "dev/serial-line.h"
#include "net/ip/uip.h"
#include "net/netstack.h"
#include "sys/procinit.h"


#include "Communication.h"

#include "net/ipv6/uip-ds6.h"

#include "net/rime/rime.h"
#include "uart0.h"
#include "uart1.h"
#include "contiki-uart.h"
#include "watchdog.h"
#include "slip-arch.h"

#include "contiki-net.h"

SENSORS(&button_sensor);

uint16_t node_id = 0x0103;

uip_ipaddr_t uip_hostaddr;

uip_802154_longaddr ieee_802154_extended_addr = {
	.addr = {0xFF, 0x02, 0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xDB, },
};

const unsigned short cs_pinouts[][MAX_DEVICES_PER_SPI] = {
	[CSI00] = { 0,  },
	[CSI01] = { 0,  },
	[CSI10] = { 0,  },
	[CSI11] = { 0,  },
	[CSI20] = { 82, },
	[CSI21] = { 83, },
	[CSI30] = { 0,  },
	[CSI31] = { 0,  },
};

const struct adf7242_platform_data adf7242_platform_data = {
	.mode = ADF_IEEE802154_HW_AACK | ADF_IEEE802154_AUTO_CSMA_CA,
	.max_frame_retries = 4,
	.max_cca_retries = 4,
	.max_csma_be = 6,
	.min_csma_be = 1,
	.bus = CSI21,
	.device = 0,
};

int contiki_argc = 0;
char **contiki_argv;

static void BUG(void)
{
	/* Blink loop of the death! */
	while (1) {
		clock_wait(CLOCK_SECOND / 10);
		LED15 = 1;
		clock_wait(CLOCK_SECOND / 10);
		LED15 = 0;
	}
}

static void set_ipv6_addr(void)
{
	uip_ipaddr_t ipaddr;

#if 1
	uip_ip6addr(&ipaddr, 0xfe80, 0, 0, 0, 0xa200, 0, 0, 0);
#if UIP_CONF_ROUTER
	uip_ds6_prefix_add(&ipaddr, UIP_DEFAULT_PREFIX_LEN, 0, 0, 0, 0);
#else
	uip_ds6_prefix_add(&ipaddr, UIP_DEFAULT_PREFIX_LEN, 0);
#endif

	uip_ip6addr(&ipaddr, 0xfe80, 0, 0, 0, 0xa200, 0, 0, 0x3);
#if 0
	uip_ds6_set_addr_iid(&ipaddr,
			(uip_lladdr_t *) &ieee_802154_extended_addr);
#endif
	uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);
#endif

	/* Add a global IPv6 address */
	uip_ip6addr(&ipaddr, 0x2001, 0, 0, 0, 0, 0, 0, 0x3);
	uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);

	/* And the corresponding prefixes */
	uip_ip6addr(&ipaddr, 0x2001, 0, 0, 0, 0, 0, 0, 0);
#if UIP_CONF_ROUTER
	uip_ds6_prefix_add(&ipaddr, UIP_DEFAULT_PREFIX_LEN, 0, 0, 0, 0);
#else
	uip_ds6_prefix_add(&ipaddr, UIP_DEFAULT_PREFIX_LEN, 0);
#endif
}

int
main(int argc, char **argv)
{
  bool flip_flop = false;
  unsigned int i;
  int ret;

  asm ("di");

  /* Disable I/O redirection */
  PIOR0 = 0x00U;
  PIOR1 = 0x00U;

  /* Set all GPIOs as input by default */
  for (i = 0; i <= 15; i++)
	  *((volatile unsigned char *) 0xfff20 + i) = 0xff;

  /* Disable the LCD backlight by default */
  PM0 &= ~BIT(0);
  P0 &= ~BIT(0);

  /* Initialize LED outputs */
  PM1  &= ~BIT(1); /* LED1  */
  PM1  &= ~BIT(0); /* LED2  */
  PM6  &= ~BIT(2); /* LED3  */
  PM4  &= ~BIT(2); /* LED4  */
  PM6  &= ~BIT(3); /* LED5  */
  PM4  &= ~BIT(3); /* LED6  */
  PM6  &= ~BIT(4); /* LED7  */
  PM4  &= ~BIT(4); /* LED8  */
  PM6  &= ~BIT(5); /* LED9  */
  PM4  &= ~BIT(5); /* LED10 */
  PM6  &= ~BIT(6); /* LED11 */
  PM15 &= ~BIT(2); /* LED12 */
  PM6  &= ~BIT(7); /* LED13 */
  PM10 &= ~BIT(1); /* LED14 */
  PM4  &= ~BIT(1); /* LED15 */

  /* Switch off all the LEDs by default */
  LED1 = 0;
  LED2 = 0;
  LED3 = 1;
  LED4 = 1;
  LED5 = 1;
  LED6 = 1;
  LED7 = 1;
  LED8 = 1;
  LED9 = 1;
  LED10 = 1;
  LED11 = 1;
  LED12 = 1;
  LED13 = 1;
  LED14 = 1;
  LED15 = 1;

  /* Enable interrupts on rising edge on INTP2 (PMOD1) */
  EGP0 |= 0x4;
  EGN0 &= ~0x4;
  PMK2 = 0;

  /* Setup clocks */
  CMC = 0x11U;                                        /* Enable XT1, disable X1 */
  CSC = 0x80U;                                        /* Start XT1 and HOCO, stop X1 */
  CKC = 0x00U;

  clock_init();
  clock_wait(CLOCK_SECOND / 1000);

//  OSMC = 0x00;                                       /* Supply fsub to peripherals, including Interval Timer */
  uart0_init();
  uart1_init();

#ifdef __GNUC__
  /* Force linking of custom write() function: */
  write(1, NULL, 0);
#endif

  /* Setup 12-bit interval timer */
  RTCEN = 1;                                              /* Enable 12-bit interval timer and RTC */
//  ITMK = 1;                                               /* Disable IT interrupt */
  ITPR0 = 0;                                              /* Set interrupt priority - highest */
  ITPR1 = 0;
  ITMC = 0x8FFFU;                                    /* Set maximum period 4096/32768Hz = 1/8 s, and start timer */
  ITIF = 0;                                               /* Clear interrupt request flag */
//  ITMK = 0;                                               /* Enable IT interrupt */

  /* Disable analog inputs because they can conflict with the SPI buses: */
  ADPC = 0x01;  /* Configure all analog pins as digital I/O. */
  PMC0 &= 0xF0; /* Disable analog inputs. */

  memcpy(&uip_lladdr, &ieee_802154_extended_addr, sizeof (uip_lladdr));
  linkaddr_set_node_addr((linkaddr_t *) &ieee_802154_extended_addr);

  process_init();
  process_start(&etimer_process, NULL);
  ctimer_init();

  /* XXX: Remove that line and the board won't start after a cold boot.
   * It is not timing-related, as a delay_ms(1000) here does not fix the
   * problem. So why is a fwrite() of some characters fixing it?
   * Not even write() works... */
  fwrite("\r\n\r\n", 1, sizeof("\r\n\r\n"), stdout);

#if UIP_CONF_IPV6
#if UIP_CONF_IPV6_RPL
  printf(CONTIKI_VERSION_STRING " started with IPV6, RPL" NEWLINE);
#else
  printf(CONTIKI_VERSION_STRING " started with IPV6" NEWLINE);
#endif
#else
  printf(CONTIKI_VERSION_STRING " started" NEWLINE);
#endif

  /* crappy way of remembering and accessing argc/v */
  contiki_argc = argc;
  contiki_argv = argv;

//  set_rime_addr();

  clock_wait(CLOCK_SECOND / 100);
  queuebuf_init();

  ret =  NETSTACK_RADIO.init();
  if (ret < 0)
	  BUG();

  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();

#ifdef NETSTACK_ENCRYPTION_INIT
  NETSTACK_ENCRYPTION_INIT();
#endif /* NETSTACK_ENCRYPTION_INIT */

  NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, RF_CHANNEL);
  NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, IEEE802154_CONF_PANID);
  NETSTACK_RADIO.set_value(RADIO_PARAM_16BIT_ADDR, 0xCAFE);
  NETSTACK_RADIO.set_object(RADIO_PARAM_64BIT_ADDR,
		  ieee_802154_extended_addr.addr,
		  sizeof(ieee_802154_extended_addr.addr));

  printf("MAC %s RDC %s NETWORK %s" NEWLINE, NETSTACK_MAC.name,
		  NETSTACK_RDC.name, NETSTACK_NETWORK.name);

  /* The radio chip started, we can enable interrupts now */
  asm ("ei");

  /* It is required to wait for a bit here. Otherwise weird bugs
   * appear in the IPv6 stack. */
  clock_wait(CLOCK_SECOND);

  process_start(&tcpip_process, NULL);
  set_ipv6_addr();

  autostart_start(autostart_processes);

  while(1) {
    watchdog_periodic();

    if(NETSTACK_RADIO.pending_packet()) {
      int len;
      packetbuf_clear();
      len = NETSTACK_RADIO.read(packetbuf_dataptr(), PACKETBUF_SIZE);
      if(len > 0) {
        packetbuf_set_datalen(len);
        NETSTACK_RDC.input();
      }
    }

#if 0
    while(uart0_can_getchar()) {
      char c;
      UART_RX_LED = 1;
      c = uart0_getchar();
      if(uart0_input_handler) {
        uart0_input_handler(c);
      }
    }
    UART_RX_LED = 0;
#endif

    process_run();

    etimer_request_poll();

    HEARTBEAT_LED1 = flip_flop;
    flip_flop = !flip_flop;
    HEARTBEAT_LED2 = flip_flop;
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
void
log_message(char *m1, char *m2)
{
  printf("%s%s" NEWLINE, m1, m2);
}
/*---------------------------------------------------------------------------*/
void
uip_log(char *m)
{
  printf("%s" NEWLINE, m);
}
/*---------------------------------------------------------------------------*/

void leds_arch_set(void) { }
