/***************************************************************************//**
 *   @file   AD7091R.c
 *   @brief  Implementation of AD7091R Driver.
 *   @author Dan Nechita
 ********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************
 *   SVN Revision: 788
 *******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "AD7091R.h"
#include "Communication.h"
#include "contiki.h"

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Initializes the SPI communication peripheral.
 *
 * @return Result of the initialization procedure.
 *					Example: 0x0 - SPI peripheral was not initialized.
 *				         0x1 - SPI peripheral is initialized.
 *******************************************************************************/
unsigned char
AD7091R_Init(enum CSI_Bus bus, char slaveDeviceId)
{
  unsigned char status = 1;
  unsigned char tmpVal;

  status = SPI_Init(bus, slaveDeviceId, 1000000, 0, 1);

  /* RESET - do s short read */
  SPI_Read(bus, slaveDeviceId, &tmpVal, 1);
  PMOD2_P9 = 1;

  return status;
}
/***************************************************************************//**
 * @brief Initiates one conversion and reads back the result. During this
 *        process the device runs in normal mode and operates without the busy
 *        indicator.
 *
 * @return conversionResult - 12bit conversion result.
 *******************************************************************************/
unsigned short
AD7091R_ReadSample(enum CSI_Bus bus, char slaveDeviceId)
{
  unsigned short conversionResult = 0;
  unsigned char buffer[2];

  PMOD2_P9 = 0;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  PMOD2_P9 = 1;

  clock_wait(CLOCK_SECOND / 1000);
  /* Read conversion data. */
  SPI_Read(bus, slaveDeviceId, buffer, 2);
  conversionResult = (buffer[0] << 8) + buffer[1];
  conversionResult >>= 4;

  return conversionResult;
}
