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

#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#include <stdint.h>

#include "rl78.h"
#include "platform-conf.h"

/* Clock ticks per second */
#define CLOCK_CONF_SECOND (f_CLK >> CLOCK_SCALER)

#define CCIF
#define CLIF

/* A trick to resolve a compilation error with IAR. */
#ifdef __IAR_SYSTEMS_ICC__
#define UIP_CONF_DS6_AADDR_NBU      1
#endif

#define USE_FORMATTED_STDIO         1

#define RDC_CONF_HARDWARE_ACK  1
#define RDC_CONF_HARDWARE_CSMA 1

#define NULLRDC_CONF_802154_AUTOACK_HW 1

/* Network setup for IPv6 */
#define NETSTACK_CONF_NETWORK sicslowpan_driver
#define NETSTACK_CONF_MAC     nullmac_driver
/* #define NETSTACK_CONF_RDC     sicslowmac_driver */
#define NETSTACK_CONF_RDC     nullrdc_driver
#define NETSTACK_CONF_RADIO   adf7242_driver
#define NETSTACK_CONF_FRAMER  framer_802154

#define RF_CHANNEL 11
#define IEEE802154_CONF_PANID 0x0777

#define LINKADDR_CONF_SIZE                      8

#define UIP_CONF_ROUTER                         0
#define UIP_CONF_IPV6_RPL                       0

/* IPv6 configuration options */
#define UIP_CONF_IPV6                           1
#define NBR_TABLE_CONF_MAX_NEIGHBORS            20 /* number of neighbors */
#define UIP_CONF_DS6_ROUTE_NBU                  20 /* number of routes */
#define UIP_CONF_ND6_SEND_RA                    0
#define UIP_CONF_ND6_REACHABLE_TIME             600000
#define UIP_CONF_ND6_RETRANS_TIMER              10000

#define UIP_CONF_BUFFER_SIZE                    1300

#define QUEUEBUF_CONF_NUM                       4
#define QUEUEBUF_CONF_REF_NUM     16

/* UDP configuration options */
#define UIP_CONF_UDP                            1
#define UIP_CONF_UDP_CHECKSUMS                  1
#define UIP_CONF_UDP_CONNS                      10

/* 6lowpan options (for ipv6) */
#define SICSLOWPAN_CONF_COMPRESSION             SICSLOWPAN_COMPRESSION_HC06
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS       2
#define SICSLOWPAN_CONF_COMPRESSION_THRESHOLD   63
#define SICSLOWPAN_CONF_FRAG                    1
#define SICSLOWPAN_CONF_MAXAGE                  8

/* General configuration options */
#define UIP_CONF_STATISTICS                     0
#define UIP_CONF_LOGGING                        0
#define UIP_CONF_BROADCAST                      1
#define UIP_CONF_LLH_LEN                        0
#define UIP_CONF_LL_802154                      1

/* include the project config */
/* PROJECT_CONF_H might be defined in the project Makefile */
#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif /* PROJECT_CONF_H */

#endif /* __CONTIKI_CONF_H__ */
