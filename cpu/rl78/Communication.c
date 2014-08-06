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
 * \author Dragos Bogdan <Dragos.Bogdan@Analog.com>, Ian Martin <martini@redwirellc.com>
 */

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "rl78.h"

#include "Communication.h"  /* Communication definitions */

#undef BIT
#define BIT(n) (1 << (n))

#define CLK_SCALER (0x4)
#define SCALED_CLK (f_CLK / (1 << CLK_SCALER))
#define BITBANG_SPI 0

extern const unsigned short cs_pinouts[][MAX_DEVICES_PER_SPI];

char IICA0_Flag;

struct rl78_serial_pinouts {
	unsigned short clk, miso, mosi;
	volatile unsigned short *scr, *sdr, *smr, *ssr;
	volatile unsigned char *sio;
};

static struct rl78_serial_pinouts serial_pinouts[] = {
	[CSI00] = { 30,  50,  51,  &SCR00, &SDR00, &SMR00, &SSR00, &SIO00 },
	[CSI10] = { 4,   3,   2,   &SCR02, &SDR02, &SMR02, &SSR02, &SIO10 },
	[CSI20] = { 15,  14,  13,  &SCR10, &SDR10, &SMR10, &SSR10, &SIO20 },
	[CSI21] = { 70,  71,  72,  &SCR11, &SDR11, &SMR11, &SSR11, &SIO21 },
#if !defined(RL78_NB_PINS) || RL78_NB_PINS < 80
	[CSI01] = { 43,  44,  45,  &SCR01, &SDR01, &SMR01, &SSR01, &SIO01 },
	[CSI11] = { 30,  50,  51,  &SCR03, &SDR03, &SMR03, &SSR03, &SIO11 },
#else
	[CSI01] = { 75,  74,  73,  &SCR01, &SDR01, &SMR01, &SSR01, &SIO01 },
	[CSI11] = { 10,  11,  12,  &SCR03, &SDR03, &SMR03, &SSR03, &SIO11 },
	[CSI30] = { 142, 143, 144, &SCR12, &SDR12, &SMR12, &SSR12, &SIO30 },
	[CSI31] = { 54,  53,  52,  &SCR13, &SDR13, &SMR13, &SSR13, &SIO31 },
#endif
};

static void set_pin(unsigned short pin, bool value)
{
	if (value)
		*(((volatile uint8_t *) 0xFFF00) + pin / 10) |= BIT(pin % 10);
	else
		*(((volatile uint8_t *) 0xFFF00) + pin / 10) &= ~BIT(pin % 10);
}

static bool read_pin(unsigned short pin)
{
	return *(((volatile uint8_t *) 0xFFF00) + pin / 10) & BIT(pin % 10);
}

static void set_pin_mode(unsigned short pin, bool output)
{
	if (output)
		*(((volatile uint8_t *) 0xFFF20) + pin / 10) &= ~BIT(pin % 10);
	else
		*(((volatile uint8_t *) 0xFFF20) + pin / 10) |= BIT(pin % 10);
}

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief I2C interrupt service routine.
 *
 * @return None.
 *******************************************************************************/
/*__interrupt */ static void
IICA0_Interrupt(void)
{
  IICA0_Flag = 1;
}
/***************************************************************************//**
 * @brief Initializes the SPI communication peripheral.
 *
 * @param lsbFirst  - Transfer format (0 or 1).
 *                    Example: 0x0 - MSB first.
 *                             0x1 - LSB first.
 * @param clockFreq - SPI clock frequency (Hz).
 *                    Example: 1000 - SPI clock frequency is 1 kHz.
 * @param clockPol  - SPI clock polarity (0 or 1).
 *                    Example: 0x0 - Idle state for clock is a low level; active
 *                                   state is a high level;
 *                             0x1 - Idle state for clock is a high level; active
 *                                   state is a low level.
 * @param clockEdg  - SPI clock edge (0 or 1).
 *                    Example: 0x0 - Serial output data changes on transition
 *                                   from idle clock state to active clock state;
 *                             0x1 - Serial output data changes on transition
 *                                   from active clock state to idle clock state.
 *
 * @return status   - Result of the initialization procedure.
 *                    Example:  0 - if initialization was successful;
 *                             -1 - if initialization was unsuccessful.
 *******************************************************************************/
char
SPI_Init(enum CSI_Bus bus,
         char lsbFirst,
         long clockFreq,
         char clockPol,
         char clockEdg)
{
  unsigned int i;

  /* MOSI (output) */
  set_pin(serial_pinouts[bus].mosi, true);
  set_pin_mode(serial_pinouts[bus].mosi, true);

  /* MISO (input) */
  set_pin_mode(serial_pinouts[bus].miso, false);

  /* CLK (output) */
  set_pin(serial_pinouts[bus].clk, true);
  set_pin_mode(serial_pinouts[bus].clk, true);

  /* CS (output) */
  for (i = 0; i < MAX_DEVICES_PER_SPI; i++) {
	  if (cs_pinouts[bus][i]) {
		  set_pin(cs_pinouts[bus][i], true);
		  set_pin_mode(cs_pinouts[bus][i], true);
	  }
  }

#if !defined(BITBANG_SPI) || !BITBANG_SPI
  char delay = 0;
  uint16_t scr, sdr;
  uint8_t shift;

  PIOR5 = 0; /* Keep SPI functions on Port 0 pins 2-4. */

  /* Enable input clock supply. */
  if(bus <= CSI11) {
    SAU0EN = 1;
  } else {
    SAU1EN = 1;
  }

  /* After setting the SAUmEN bit to 1, be sure to set serial clock select
     register m (SPSm) after 4 or more fCLK clocks have elapsed. */
  NOP;
  NOP;
  NOP;
  NOP;

  /* Select the fCLK as input clock.  */
  if (bus <= CSI11)
	  SPS0 = 0x0000;
  else
	  SPS1 = 0x0000;

  clockPol = 1 - clockPol;
  scr = (clockEdg << 13) |
    (clockPol << 12) |
    0xC000 |                       /* Operation mode: Transmission/reception. */
    0x0007;                        /* 8-bit data length. */


  /* clockFreq =  mckFreq / (sdr * 2 + 2) */
  sdr = f_CLK / (2 * clockFreq) - 1;
  sdr <<= 9;

  *serial_pinouts[bus].scr = scr;
  *serial_pinouts[bus].sdr = sdr;
  *serial_pinouts[bus].smr = 0x0020;

  /* Set the clock and data initial level. */
  clockPol = 1 - clockPol;
  shift = bus & 0x3;

  if(bus <= CSI11) {
    SO0 &= ~(0x0101 << shift);
    SO0 |= ((clockPol << 8) | clockPol) << shift;
  } else {
    SO1 &= ~(0x0101 << shift);
    SO1 |= ((clockPol << 8) | clockPol) << shift;
  }

  /* Enable output for serial communication operation. */
  if (bus <= CSI11)
	  SOE0 |= BIT(bus & 0x3);
  else
	  SOE1 |= BIT(bus & 0x3);

  /* Wait for the changes to take place. */
  for(delay = 0; delay < 50; delay++) {
    NOP;
  }

  /* Set the SEmn bit to 1 and enter the communication wait status */
  if (bus <= CSI11)
    SS0 = BIT(bus & 0x3);
  else
    SS1 = BIT(bus & 0x3);
#endif

  return 0;
}

#if BITBANG_SPI
static uint8_t spi_byte_exchange(enum CSI_Bus bus, uint8_t tx)
{
  unsigned char rx = 0, n = 0;

  set_pin(serial_pinouts[bus].clk, 0);
  for(n = 0; n < 8; n++, tx <<= 1) {
	  set_pin(serial_pinouts[bus].mosi, tx & 0x80);

	  /* The slave samples MOSI at the rising-edge of SCLK. */
	  set_pin(serial_pinouts[bus].clk, 1);

	  rx = (rx << 1) | read_pin(serial_pinouts[bus].miso);

	  /* The slave changes the value of MISO at the falling-edge of SCLK. */
	  set_pin(serial_pinouts[bus].clk, 0);
  }

  return rx;
}
#else
static uint8_t spi_byte_exchange(enum CSI_Bus bus, uint8_t tx)
{
	volatile uint8_t *sio = serial_pinouts[bus].sio;
	volatile uint16_t *ssr = serial_pinouts[bus].ssr;
	*sio = tx;
	NOP;
	while (*ssr & 0x0040);
	return *sio;
}
#endif

void SPI_Set_CS(enum CSI_Bus bus, char slaveDeviceId, char high)
{
	if (cs_pinouts[bus][slaveDeviceId])
		set_pin(cs_pinouts[bus][slaveDeviceId], !!high);
}

int SPI_Write_NoCS(enum CSI_Bus bus, char slaveDeviceId,
		const uint8_t *data, unsigned short bytesNumber)
{
	unsigned int i;

	if (slaveDeviceId > MAX_DEVICES_PER_SPI)
		return -ENODEV;

	for(i = 0; i < bytesNumber; i++)
		spi_byte_exchange(bus, data[i]);
	return 0;
}

int SPI_Read_NoCS(enum CSI_Bus bus, char slaveDeviceId,
		uint8_t *data, unsigned short bytesNumber)
{
	unsigned int i;

	if (slaveDeviceId > MAX_DEVICES_PER_SPI)
		return -ENODEV;

	for(i = 0; i < bytesNumber; i++)
		data[i] = spi_byte_exchange(bus, 0xFF);
	return 0;
}

/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data          - Data represents the write buffer.
 * @param bytesNumber   - Number of bytes to write.
 *
 * @return Zero on success, -1 on error.
 *******************************************************************************/
int SPI_Write(enum CSI_Bus bus, char slaveDeviceId,
		const uint8_t *data, unsigned short bytesNumber)
{
	int ret;

	SPI_Set_CS(bus, slaveDeviceId, 0);
	ret = SPI_Write_NoCS(bus, slaveDeviceId, data, bytesNumber);
	SPI_Set_CS(bus, slaveDeviceId, 1);
	return ret;
}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data          - Data represents the write buffer as an input parameter
 *                        and the read buffer as an output parameter.
 * @param bytesNumber   - Number of bytes to read.
 *
 * @return Number of read bytes.
 *******************************************************************************/
int SPI_Read(enum CSI_Bus bus, char slaveDeviceId,
		uint8_t *data, unsigned short bytesNumber)
{
	int ret;

	SPI_Set_CS(bus, slaveDeviceId, 0);
	ret = SPI_Read_NoCS(bus, slaveDeviceId, data, bytesNumber);
	SPI_Set_CS(bus, slaveDeviceId, 1);
	return ret;
}

/***************************************************************************//**
 * @brief Initializes the I2C communication peripheral.
 *
 * @param clockFreq - I2C clock frequency (Hz).
 *                    Example: 100000 - SPI clock frequency is 100 kHz.
 * @return status   - Result of the initialization procedure.
 *                    Example:  0 - if initialization was successful;
 *                             -1 - if initialization was unsuccessful.
 *******************************************************************************/
char
I2C_Init(long clockFreq)
{
  long fckFreq = 32000000;
  unsigned char wlValue = 0;
  unsigned char whValue = 0;

  (void)IICA0_Interrupt;   /* Prevent an unused-function warning. */

  /* Enable interrupts */
  EI;

  /* Enable input clock supply. */
  IICA0EN = 1;

  /* Set the fast mode plus operation. */
  SMC0 = 1;

  /* Set transfer rate. */
  wlValue = (unsigned char)((0.5 * fckFreq) / clockFreq);
  whValue = (unsigned char)(wlValue - (fckFreq / (10 * clockFreq)));
  IICWL0 = wlValue;
  IICWH0 = whValue;

  STCEN0 = 1;    /* After operation is enabled, enable generation of a start */
                 /* condition without detecting a stop condition. */
  WTIM0 = 1;     /* Interrupt request is generated at the ninth clock’s */
                 /* falling edge. */

  /* Enable I2C operation. */
  IICE0 = 1;

  /* Configure SCLA0 and SDAA0 pins as digital output. */
  P6 &= ~0x03;
  PM6 &= ~0x03;

  return 0;
}
/***************************************************************************//**
 * @brief Writes data to a slave device.
 *
 * @param slaveAddress - Adress of the slave device.
 * @param dataBuffer   - Pointer to a buffer storing the transmission data.
 * @param bytesNumber  - Number of bytes to write.
 * @param stopBit      - Stop condition control.
 *                       Example: 0 - A stop condition will not be sent;
 *                                1 - A stop condition will be sent.
 *
 * @return status      - Number of read bytes or 0xFF if the slave address was
 *                       not acknowledged by the device.
 *******************************************************************************/
char
I2C_Write(char slaveAddress,
          unsigned char *dataBuffer,
          char bytesNumber,
          char stopBit)
{
  char byte = 0;
  char status = 0;

  IICAMK0 = 1;      /* Interrupt servicing disabled. */
  STT0 = 1;         /* Generate a start condition. */
  IICAMK0 = 0;      /* Interrupt servicing enabled. */

  /* Send the first byte. */
  IICA0_Flag = 0;
  IICA0 = (slaveAddress << 1);
  while(IICA0_Flag == 0) ;

  if(ACKD0) {   /* Acknowledge was detected. */
    for(byte = 0; byte < bytesNumber; byte++) {
      IICA0_Flag = 0;
      IICA0 = *dataBuffer;
      while(IICA0_Flag == 0) ;
      dataBuffer++;
    }
    status = bytesNumber;
  } else {      /* Acknowledge was not detected. */
    status = 0xFF;
  }
  if(stopBit) {
    SPT0 = 1;           /* Generate a stop condition. */
    while(IICBSY0) ;    /* Wait until the I2C bus status flag is cleared. */
  }

  return status;
}
/***************************************************************************//**
 * @brief Reads data from a slave device.
 *
 * @param slaveAddress - Adress of the slave device.
 * @param dataBuffer   - Pointer to a buffer that will store the received data.
 * @param bytesNumber  - Number of bytes to read.
 * @param stopBit      - Stop condition control.
 *                       Example: 0 - A stop condition will not be sent;
 *                                1 - A stop condition will be sent.
 *
 * @return status      - Number of read bytes or 0xFF if the slave address was
 *                       not acknowledged by the device.
 *******************************************************************************/
char
I2C_Read(char slaveAddress,
         unsigned char *dataBuffer,
         char bytesNumber,
         char stopBit)
{
  char byte = 0;
  char status = 0;

  IICAMK0 = 1;    /* Interrupt servicing disabled. */
  STT0 = 1;       /* Generate a start condition. */
  IICAMK0 = 0;          /* Interrupt servicing enabled. */

  /* Send the first byte. */
  IICA0_Flag = 0;
  IICA0 = (slaveAddress << 1) + 1;
  while(IICA0_Flag == 0) ;

  if(ACKD0) {           /* Acknowledge was detected. */
    ACKE0 = 1;          /* Enable acknowledgment. */
    for(byte = 0; byte < bytesNumber; byte++) {
      if(byte == (bytesNumber - 1)) {
        ACKE0 = 0U;                   /* Disable acknowledgment. */
      }
      WREL0 = 1U;                     /* Cancel wait. */
      IICA0_Flag = 0;
      while(IICA0_Flag == 0) ;
      *dataBuffer = IICA0;
      dataBuffer++;
    }
    status = bytesNumber;
  } else {                   /* Acknowledge was not detected. */
    status = 0xFF;
  }
  if(stopBit) {
    SPT0 = 1;               /* Generate a stop condition. */
    while(IICBSY0) ;        /* Wait until the I2C bus status flag is cleared. */
  }

  return status;
}
