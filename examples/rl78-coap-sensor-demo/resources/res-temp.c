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

extern int pt100_temp;
extern int pt100_thresh_off;
extern int pt100_thresh_on;
extern unsigned char adf7242_lqi;

static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_response_payload(response, buffer, snprintf((char *)buffer, preferred_size, "Temperature %d °C", pt100_temp));
}

static void
res_get_lqi_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_response_payload(response, buffer, snprintf((char *)buffer, preferred_size, "LQI %u", (unsigned) adf7242_lqi));
}

static void
res_get_thresh_on_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_response_payload(response, buffer, snprintf((char *)buffer, preferred_size, "Threshold ON %d °C", pt100_thresh_on));
}

static void
res_get_thresh_off_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_response_payload(response, buffer, snprintf((char *)buffer, preferred_size, "Threshold OFF %d °C", pt100_thresh_off));
}

static void
res_put_thresh_on_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	const char *thresh;
	uint8_t len = REST.get_post_variable(request, "t", &thresh);

	if (len)
		pt100_thresh_on = atoi(thresh);
	else
		REST.set_response_status(response, REST.status.BAD_REQUEST);

}

static void
res_put_thresh_off_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	const char *thresh;
	uint8_t len = REST.get_post_variable(request, "t", &thresh);

	if (len)
		pt100_thresh_off = atoi(thresh);
	else
		REST.set_response_status(response, REST.status.BAD_REQUEST);

}





RESOURCE(res_adf7242_lgi,
          "title=\"ADF7242 Link Quality Indicator\";rt=\"Control\"",
         res_get_lqi_handler,
	 NULL,
         NULL,
         NULL);

RESOURCE(res_temp,
          "title=\"Temperature: \";rt=\"Control\"",
         res_get_handler,
	 NULL,
         NULL,
         NULL);

RESOURCE(res_temp_thresh_on,
          "title=\"Threshold: ?POST/PUT t=temp\";rt=\"Control\"",
         res_get_thresh_on_handler,
	 res_put_thresh_on_handler,
         res_put_thresh_on_handler,
         NULL);

RESOURCE(res_temp_thresh_off,
          "title=\"Threshold: ?POST/PUT t=temp\";rt=\"Control\"",
         res_get_thresh_off_handler,
	 res_put_thresh_off_handler,
         res_put_thresh_off_handler,
         NULL);