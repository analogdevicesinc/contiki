/*
 * Copyright (c) 2013, Institute for Pervasive Computing, ETH Zurich
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

#include <stdlib.h>
#include <string.h>
#include "rest-engine.h"

static void
res_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	const char *mode;
	uint8_t len = REST.get_post_variable(request, "mode", &mode);
	printf("Got mode %s length=%u\n", mode, len);

    if(strncmp(mode, "on", len) == 0) {
	    LED1 = 1;
    } else if(strncmp(mode, "off", len) == 0) {
	    LED1 = 0;
    } else {
	    REST.set_response_status(response, REST.status.BAD_REQUEST);
    }
}

static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	uint8_t length;
	const char *msg;

	if (LED1)
		msg = "LAMP is ON";
	else
		msg = "LAMP is OFF";

	length = strlen(msg);
	memcpy(buffer, msg, length);

	REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
	REST.set_header_etag(response, &length, 1);
	REST.set_response_payload(response, buffer, length);
}

RESOURCE(res_led,
          "title=\"LEDs: ?POST/PUT mode=on|off\";rt=\"Control\"",
         res_get_handler,
	 res_put_handler,
         res_put_handler,
         NULL);
