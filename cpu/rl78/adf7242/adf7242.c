/*
 * Analog Devices ADF7242 Low-Power IEEE 802.15.4 Transceiver
 *
 * Copyright 2009-2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

/*
 * DEBUG LEVEL
 *     0       OFF
 *     1       INFO
 *     2       INFO + TRACE
 */

#include "adf7242.h"
#include "adf7242-firmware.h"
#include "Communication.h"
#include "contiki.h"
#include "radio.h"

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ADF_DEBUG      0
#define DBG(n, args...) do { \
	if (ADF_DEBUG >= (n)) \
	    printf("DEBUG" #n ": " args); \
} while (0)

#define MAX_POLL_LOOPS 1000

/* All Registers */

#define REG_EXT_CTRL   0x100	/* RW External LNA/PA and internal PA control configuration bits */
#define REG_TX_FSK_TEST 0x101	/* RW TX FSK test mode configuration */
#define REG_CCA1       0x105	/* RW RSSI threshold for CCA */
#define REG_CCA2       0x106	/* RW CCA mode configuration */
#define REG_BUFFERCFG  0x107	/* RW RX_BUFFER overwrite control */
#define REG_PKT_CFG    0x108	/* RW FCS evaluation configuration */
#define REG_DELAYCFG0  0x109	/* RW RC_RX command to SFD or sync word search delay */
#define REG_DELAYCFG1  0x10A	/* RW RC_TX command to TX state */
#define REG_DELAYCFG2  0x10B	/* RW Mac delay extention */
#define REG_SYNC_WORD0 0x10C	/* RW sync word bits [7:0] of [23:0]  */
#define REG_SYNC_WORD1 0x10D	/* RW sync word bits [15:8] of [23:0]  */
#define REG_SYNC_WORD2 0x10E	/* RW sync word bits [23:16] of [23:0]  */
#define REG_SYNC_CONFIG        0x10F	/* RW sync word configuration */
#define REG_RC_CFG     0x13E	/* RW RX / TX packet configuration */
#define REG_RC_VAR44   0x13F	/* RW RESERVED */
#define REG_CH_FREQ0   0x300	/* RW Channel Frequency Settings - Low Byte */
#define REG_CH_FREQ1   0x301	/* RW Channel Frequency Settings - Middle Byte */
#define REG_CH_FREQ2   0x302	/* RW Channel Frequency Settings - 2 MSBs */
#define REG_TX_FD      0x304	/* RW TX Frequency Deviation Register */
#define REG_DM_CFG0    0x305	/* RW RX Discriminator BW Register */
#define REG_TX_M       0x306	/* RW TX Mode Register */
#define REG_RX_M       0x307	/* RW RX Mode Register */
#define REG_RRB                0x30C	/* R RSSI Readback Register */
#define REG_LRB                0x30D	/* R Link Quality Readback Register */
#define REG_DR0                0x30E	/* RW bits [15:8] of [15:0] for data rate setting */
#define REG_DR1                0x30F	/* RW bits [7:0] of [15:0] for data rate setting */
#define REG_PRAMPG     0x313	/* RW RESERVED */
#define REG_TXPB       0x314	/* RW TX Packet Storage Base Address */
#define REG_RXPB       0x315	/* RW RX Packet Storage Base Address */
#define REG_TMR_CFG0   0x316	/* RW Wake up Timer Configuration Register - High Byte */
#define REG_TMR_CFG1   0x317	/* RW Wake up Timer Configuration Register - Low Byte */
#define REG_TMR_RLD0   0x318	/* RW Wake up Timer Value Register - High Byte */
#define REG_TMR_RLD1   0x319	/* RW Wake up Timer Value Register - Low Byte */
#define REG_TMR_CTRL   0x31A	/* RW Wake up Timer Timeout flag */
#define REG_PD_AUX     0x31E	/* RW Battmon enable */
#define REG_GP_CFG     0x32C	/* RW GPIO Configuration */
#define REG_GP_OUT     0x32D	/* RW GPIO Configuration */
#define REG_GP_IN      0x32E	/* R GPIO Configuration */
#define REG_SYNT       0x335	/* RW bandwidth calibration timers */
#define REG_CAL_CFG    0x33D	/* RW Calibration Settings */
#define REG_SYNT_CAL   0x371	/* RW Oscillator and Doubler Configuration */
#define REG_IIRF_CFG   0x389	/* RW BB Filter Decimation Rate */
#define REG_CDR_CFG    0x38A	/* RW CDR kVCO */
#define REG_DM_CFG1    0x38B	/* RW Postdemodulator Filter */
#define REG_AGCSTAT    0x38E	/* R RXBB Ref Osc Calibration Engine Readback */
#define REG_RXCAL0     0x395	/* RW RX BB filter tuning, LSB */
#define REG_RXCAL1     0x396	/* RW RX BB filter tuning, MSB */
#define REG_RXFE_CFG   0x39B	/* RW RXBB Ref Osc & RXFE Calibration */
#define REG_PA_RR      0x3A7	/* RW Set PA ramp rate */
#define REG_PA_CFG     0x3A8	/* RW PA enable */
#define REG_EXTPA_CFG  0x3A9	/* RW External PA BIAS DAC */
#define REG_EXTPA_MSC  0x3AA	/* RW PA Bias Mode */
#define REG_ADC_RBK    0x3AE	/* R Readback temp */
#define REG_AGC_CFG1   0x3B2	/* RW GC Parameters */
#define REG_AGC_MAX    0x3B4	/* RW Slew rate  */
#define REG_AGC_CFG2   0x3B6	/* RW RSSI Parameters */
#define REG_AGC_CFG3   0x3B7	/* RW RSSI Parameters */
#define REG_AGC_CFG4   0x3B8	/* RW RSSI Parameters */
#define REG_AGC_CFG5   0x3B9	/* RW RSSI & NDEC Parameters */
#define REG_AGC_CFG6   0x3BA	/* RW NDEC Parameters */
#define REG_OCL_CFG1   0x3C4	/* RW OCL System Parameters */
#define REG_IRQ1_EN0   0x3C7	/* RW Interrupt Mask set bits  [7:0] of [15:0] for IRQ1 */
#define REG_IRQ1_EN1   0x3C8	/* RW Interrupt Mask set bits  [15:8] of [15:0] for IRQ1 */
#define REG_IRQ2_EN0   0x3C9	/* RW Interrupt Mask set bits  [7:0] of [15:0] for IRQ2 */
#define REG_IRQ2_EN1   0x3CA	/* RW Interrupt Mask set bits  [15:8] of [15:0] for IRQ2 */
#define REG_IRQ1_SRC0  0x3CB	/* RW Interrupt Source  bits  [7:0] of [15:0] for IRQ */
#define REG_IRQ1_SRC1  0x3CC	/* RW Interrupt Source bits  [15:8] of [15:0] for IRQ */
#define REG_OCL_BW0    0x3D2	/* RW OCL System Parameters */
#define REG_OCL_BW1    0x3D3	/* RW OCL System Parameters */
#define REG_OCL_BW2    0x3D4	/* RW OCL System Parameters */
#define REG_OCL_BW3    0x3D5	/* RW OCL System Parameters */
#define REG_OCL_BW4    0x3D6	/* RW OCL System Parameters */
#define REG_OCL_BWS    0x3D7	/* RW OCL System Parameters */
#define REG_OCL_CFG13  0x3E0	/* RW OCL System Parameters */
#define REG_GP_DRV     0x3E3	/* RW I/O pads Configuration and bg trim */
#define REG_BM_CFG     0x3E6	/* RW Battery Monitor Threshold Voltage setting */
#define REG_SFD_15_4   0x3F4	/* RW Option to set non standard SFD */
#define REG_AFC_CFG    0x3F7	/* RW AFC mode and polarity */
#define REG_AFC_KI_KP  0x3F8	/* RW AFC ki and kp */
#define REG_AFC_RANGE  0x3F9	/* RW AFC range */
#define REG_AFC_READ   0x3FA	/* RW Readback frequency error */

#define REG_PAN_ID0            0x112
#define REG_PAN_ID1            0x113
#define REG_SHORT_ADDR_0       0x114
#define REG_SHORT_ADDR_1       0x115
#define REG_IEEE_ADDR_0                0x116
#define REG_IEEE_ADDR_1                0x117
#define REG_IEEE_ADDR_2                0x118
#define REG_IEEE_ADDR_3                0x119
#define REG_IEEE_ADDR_4                0x11A
#define REG_IEEE_ADDR_5                0x11B
#define REG_IEEE_ADDR_6                0x11C
#define REG_IEEE_ADDR_7                0x11D
#define REG_FFILT_CFG          0x11E
#define REG_AUTO_CFG           0x11F
#define REG_AUTO_TX1           0x120
#define REG_AUTO_TX2           0x121
#define REG_AUTO_STATUS                0x122

/* REG_FFILT_CFG */
#define ACCEPT_BEACON_FRAMES   (1 << 0)
#define ACCEPT_DATA_FRAMES     (1 << 1)
#define ACCEPT_ACK_FRAMES      (1 << 2)
#define ACCEPT_MACCMD_FRAMES   (1 << 3)
#define ACCEPT_RESERVED_FRAMES (1 << 4)
#define ACCEPT_ALL_ADDRESS     (1 << 5)

/* REG_AUTO_CFG */
#define AUTO_ACK_FRAMEPEND     (1 << 0)
#define IS_PANCOORD            (1 << 1)
#define RX_AUTO_ACK_EN         (1 << 3)
#define CSMA_CA_RX_TURNAROUND  (1 << 4)

/* REG_AUTO_TX1 */
#define MAX_FRAME_RETRIES(x)   ((x) & 0xF)
#define MAX_CCA_RETRIES(x)     (((x) & 0x7) << 4)

/* REG_AUTO_TX2 */
#define CSMA_MAX_BE(x)         ((x) & 0xF)
#define CSMA_MIN_BE(x)         (((x) & 0xF) << 4)

#define CMD_SPI_NOP            0xFF	/* No operation. Use for dummy writes */
#define CMD_SPI_PKT_WR         0x10	/* Write telegram to the Packet RAM starting from the TX packet base address pointer tx_packet_base */
#define CMD_SPI_PKT_RD         0x30	/* Read telegram from the Packet RAM starting from RX packet base address pointer rxpb.rx_packet_base */
#define CMD_SPI_MEM_WR(x)      (0x18 + (x >> 8))	/* Write data to MCR or Packet RAM sequentially */
#define CMD_SPI_MEM_RD(x)      (0x38 + (x >> 8))	/* Read data from MCR or Packet RAM sequentially */
#define CMD_SPI_MEMR_WR(x)     (0x08 + (x >> 8))	/* Write data to MCR or Packet RAM as random block */
#define CMD_SPI_MEMR_RD(x)     (0x28 + (x >> 8))	/* Read data from MCR or Packet RAM as random block */
#define CMD_SPI_PRAM_WR                0x1E	/* Write data sequentially to current PRAM page selected */
#define CMD_SPI_PRAM_RD                0x3E	/* Read data sequentially from current PRAM page selected */
#define CMD_RC_SLEEP           0xB1	/* Invoke transition of radio controller into SLEEP state */
#define CMD_RC_IDLE            0xB2	/* Invoke transition of radio controller into IDLE state */
#define CMD_RC_PHY_RDY         0xB3	/* Invoke transition of radio controller into PHY_RDY state */
#define CMD_RC_RX              0xB4	/* Invoke transition of radio controller into RX state */
#define CMD_RC_TX              0xB5	/* Invoke transition of radio controller into TX state */
#define CMD_RC_MEAS            0xB6	/* Invoke transition of radio controller into MEAS state */
#define CMD_RC_CCA             0xB7	/* Invoke Clear channel assessment */
#define CMD_RC_CSMACA          0xC1	/* initiates CSMA-CA channel access sequence and frame transmission */
#define CMD_RC_PC_RESET        0xC7	/* Reset the program counter of the firmware */
#define CMD_RC_RESET           0xC8	/* Reset the ADF7242 */

/* STATUS */

#define STAT_SPI_READY         (1 << 7)
#define STAT_IRQ_STATUS                (1 << 6)
#define STAT_RC_READY          (1 << 5)
#define STAT_CCA_RESULT                (1 << 4)
#define RC_STATUS_IDLE         1
#define RC_STATUS_MEAS         2
#define RC_STATUS_PHY_RDY      3
#define RC_STATUS_RX           4
#define RC_STATUS_TX           5
#define RC_STATUS_MASK         0xF

/* AUTO_STATUS */

#define SUCCESS                        0
#define SUCCESS_DATPEND                1
#define FAILURE_CSMACA         2
#define FAILURE_NOACK          3
#define AUTO_STATUS_MASK       0x3

#define PRAM_PAGESIZE          256

/* IRQ1 */

#define IRQ_CCA_COMPLETE       (1 << 0)
#define IRQ_SFD_RX             (1 << 1)
#define IRQ_SFD_TX             (1 << 2)
#define IRQ_RX_PKT_RCVD                (1 << 3)
#define IRQ_TX_PKT_SENT                (1 << 4)
#define IRQ_FRAME_VALID                (1 << 5)
#define IRQ_ADDRESS_VALID      (1 << 6)
#define IRQ_CSMA_CA            (1 << 7)

#define AUTO_TX_TURNAROUND     (1 << 3)
#define ADDON_EN               (1 << 4)


#define INT_RECEIVED 160

extern struct adf7242_platform_data adf7242_platform_data;
static struct adf7242_platform_data *adf7242_pdata = &adf7242_platform_data;

static unsigned char tx_buf[255];
static bool tx_led, rx_led;

static volatile bool packet_pending;
static volatile int tx_status;

static uint8_t adf7242_tx_irq(struct adf7242_platform_data *pdata)
{
	if (pdata->mode & ADF_IEEE802154_AUTO_CSMA_CA)
		return IRQ_CSMA_CA;
	else
		return IRQ_TX_PKT_SENT;
}

static int adf7242_status(struct adf7242_platform_data *pdata, uint8_t * stat)
{
	return SPI_Read(pdata->bus, pdata->device, stat, 1);
}

static int adf7242_wait_mask(struct adf7242_platform_data *pdata, uint8_t mask)
{
	uint8_t stat = 0;
	unsigned int cnt = 0;

	DBG(2, "%s :Enter\r\n", __func__);

	do {
		int ret = adf7242_status(pdata, &stat);
		if (ret < 0)
			return ret;
		cnt++;
	} while ((stat & mask) != mask && (cnt < MAX_POLL_LOOPS));

	DBG(2, "%s :Exit loops=%d\r\n", __func__, cnt);

	return cnt == MAX_POLL_LOOPS ? -ETIMEDOUT : 0;
}

static int adf7242_wait_ready(struct adf7242_platform_data *pdata)
{
	DBG(2, "%s :Enter\r\n", __func__);
	int ret = adf7242_wait_mask(pdata, STAT_RC_READY | STAT_SPI_READY);
	DBG(2, "%s :Exit\r\n", __func__);
	return ret;
}

static int adf7242_wait_status(struct adf7242_platform_data *pdata, int status)
{
	uint8_t stat = 0;
	unsigned int cnt = 0;

	DBG(2, "%s :Enter\r\n", __func__);

	do {
		int ret = adf7242_status(pdata, &stat);
		if (ret < 0)
			return ret;
		stat &= RC_STATUS_MASK;
		cnt++;
	} while ((stat != status) && (cnt < MAX_POLL_LOOPS));

	DBG(2, "%s :Exit loops=%d\r\n", __func__, cnt);

	return cnt == MAX_POLL_LOOPS ? -ETIMEDOUT : 0;
}

static int adf7242_write_fbuf(struct adf7242_platform_data *pdata,
		const uint8_t * data, uint8_t len)
{
	uint8_t buf[] = { CMD_SPI_PKT_WR, len + 2, };
	int ret;

	DBG(2, "%s :Enter\r\n", __func__);

	ret = adf7242_wait_ready(pdata);
	if (ret < 0)
		return ret;

	SPI_Set_CS(pdata->bus, pdata->device, 0);
	ret = SPI_Write_NoCS(pdata->bus, pdata->device, buf, sizeof(buf));
	if (!ret)
		ret = SPI_Write_NoCS(pdata->bus, pdata->device, data, len);
	SPI_Set_CS(pdata->bus, pdata->device, 1);

	DBG(2, "%s :Exit\r\n", __func__);
	return ret;
}

static int adf7242_read_fbuf(struct adf7242_platform_data *pdata,
		uint8_t * data, uint8_t * len, uint8_t * lqi)
{
	uint8_t buf[] = { CMD_SPI_PKT_RD, CMD_SPI_NOP, };
	int ret;

	DBG(2, "%s :Enter\r\n", __func__);

	ret = adf7242_wait_ready(pdata);
	if (ret < 0)
		return ret;

	SPI_Set_CS(pdata->bus, pdata->device, 0);
	ret = SPI_Write_NoCS(pdata->bus, pdata->device, buf, sizeof(buf));
	if (!ret)
		ret = SPI_Read_NoCS(pdata->bus, pdata->device, len, 1);
	if (!ret)
		ret = SPI_Read_NoCS(pdata->bus, pdata->device, data, *len);
	SPI_Set_CS(pdata->bus, pdata->device, 1);

	if (!ret)
		*lqi = data[*len - 1];

	DBG(2, "%s :Exit\r\n", __func__);
	return ret;
}

static int adf7242_read_reg(struct adf7242_platform_data *pdata,
		uint16_t addr, uint8_t *data)
{
	uint8_t buf[] = {
		CMD_SPI_MEM_RD(addr), addr, CMD_SPI_NOP,
	};
	int ret;

	DBG(2, "%s :Enter\r\n", __func__);

	ret = adf7242_wait_ready(pdata);
	if (ret < 0)
		return ret;

	SPI_Set_CS(pdata->bus, pdata->device, 0);
	ret = SPI_Write_NoCS(pdata->bus, pdata->device, buf, sizeof(buf));
	if (!ret)
		ret = SPI_Read_NoCS(pdata->bus, pdata->device, data, 1);
	SPI_Set_CS(pdata->bus, pdata->device, 1);

	DBG(2, "%s :Exit\r\n", __func__);
	return ret;
}

static int adf7242_write_reg(struct adf7242_platform_data *pdata,
		uint16_t addr, uint8_t data)
{
	uint8_t buf[] = { CMD_SPI_MEM_WR(addr), addr, data, };
	int ret;

	DBG(2, "%s :Enter\r\n", __func__);

	ret = adf7242_wait_ready(pdata);
	if (ret < 0)
		return ret;

	ret = SPI_Write(pdata->bus, pdata->device, buf, sizeof(buf));

	DBG(2, "%s :Exit REG 0x%hX, VAL 0x%hhX\r\n", __func__,
			addr, data);
	return ret;
}

static int adf7242_cmd(struct adf7242_platform_data *pdata, uint8_t cmd)
{
	int ret;

	DBG(2, "%s :Enter CMD=0x%X\r\n", __func__, cmd);
	ret = adf7242_wait_ready(pdata);
	if (ret < 0)
		return ret;

	ret = SPI_Write(pdata->bus, pdata->device, &cmd, 1);

	DBG(2, "%s :Exit\r\n", __func__);
	return ret;
}

static void adf7242_irqwork(void)
{
	int ret = RADIO_TX_OK;
	uint8_t irq, stat;

	adf7242_read_reg(adf7242_pdata, REG_IRQ1_SRC1, &irq);

	/* Ack the interrupt on the module side */
	adf7242_write_reg(adf7242_pdata, REG_IRQ1_SRC1, irq);

	if (irq & IRQ_RX_PKT_RCVD) {
		/* Wait until ACK is processed */
		if ((adf7242_pdata->mode & ADF_IEEE802154_HW_AACK) &&
				((stat & RC_STATUS_MASK) != RC_STATUS_PHY_RDY))
			adf7242_wait_status(adf7242_pdata, RC_STATUS_PHY_RDY);

		packet_pending = true;
	}

	if (irq & adf7242_tx_irq(adf7242_pdata)) {
		adf7242_read_reg(adf7242_pdata, REG_AUTO_STATUS, &stat);
		stat &= AUTO_STATUS_MASK;

		if (stat == FAILURE_NOACK)
			ret = RADIO_TX_NOACK;
		else if (stat != SUCCESS)
			ret = RADIO_TX_ERR;

		tx_status = ret;
		adf7242_cmd(adf7242_pdata, CMD_RC_RX);
	}
}

void __attribute__((interrupt)) p1_handler(void)
{
	/* Ack the interrupt on the CPU side */
	PIF1 = 0;
	adf7242_irqwork();
}

void __attribute__((interrupt)) p2_handler(void)
{
	/* Ack the interrupt on the CPU side */
	PIF2 = 0;
	adf7242_irqwork();
}

static int adf7242_verify_firmware(struct adf7242_platform_data *pdata,
		const uint8_t *data, uint16_t len)
{
	int ret, i, j;
	unsigned int page;
	const uint8_t cmd[] = { CMD_SPI_PRAM_RD, 0, CMD_SPI_NOP, };
	uint8_t buf[PRAM_PAGESIZE];

	for (page = 0, i = len; i >= 0; i -= PRAM_PAGESIZE, page++) {
		unsigned short nb = i >= PRAM_PAGESIZE ? PRAM_PAGESIZE : i;
		ret = adf7242_write_reg(pdata, REG_PRAMPG, page);
		if (ret < 0)
			goto err;

		SPI_Set_CS(pdata->bus, pdata->device, 0);
		ret = SPI_Write_NoCS(pdata->bus, pdata->device,
				cmd, sizeof(cmd));
		if (!ret)
			ret = SPI_Read_NoCS(pdata->bus, pdata->device,
					buf, sizeof(buf));
		SPI_Set_CS(pdata->bus, pdata->device, 1);
		if (ret)
			goto err;

		for (j = 0; j < nb; j++) {
			if (buf[j] != data[page * PRAM_PAGESIZE + j]) {
				fprintf(stderr, "ERROR: Expected 0x%02hhX, got 0x%02hhX\r\n",
						data[page * PRAM_PAGESIZE + j],
						buf[j]);
				ret = -EIO;
				goto err;
			}
		}
	}

	return 0;

err:
	fprintf(stderr, "Error while uploading firmware: %s\r\n",
			strerror(-ret));
	return ret;
}

static int adf7242_upload_firmware(struct adf7242_platform_data *pdata,
		const uint8_t *data, uint16_t len)
{
	int ret, i;
	unsigned int page;
	const uint8_t cmd[] = { CMD_SPI_PRAM_WR, 0, };

	for (page = 0, i = len; i >= 0; i -= PRAM_PAGESIZE, page++) {
		unsigned short nb = i >= PRAM_PAGESIZE ? PRAM_PAGESIZE : i;

		ret = adf7242_write_reg(pdata, REG_PRAMPG, page);
		if (ret < 0)
			goto err;

		SPI_Set_CS(pdata->bus, pdata->device, 0);
		ret = SPI_Write_NoCS(pdata->bus, pdata->device,
				cmd, sizeof(cmd));
		if (!ret)
			ret = SPI_Write_NoCS(pdata->bus, pdata->device,
					&data[page * PRAM_PAGESIZE], nb);
		SPI_Set_CS(pdata->bus, pdata->device, 1);
		if (ret)
			goto err;
	}

	DBG(1, "Firmware uploaded\r\n");
	return 0;

err:
	fprintf(stderr, "Error while uploading firmware: %s\r\n",
			strerror(-ret));
	return ret;
}

static int adf7242_reset(struct adf7242_platform_data *pdata)
{
	int ret;

	adf7242_cmd(pdata, CMD_RC_RESET);
	ret = adf7242_wait_ready(pdata);

	if (ret < 0)
		return ret;
	else
		return adf7242_cmd(pdata, CMD_RC_IDLE);
}

static int adf7242_hw_init(struct adf7242_platform_data *pdata)
{
	int ret;
	uint8_t reg;

	DBG(2, "%s :Enter\r\n", __func__);

	ret = adf7242_reset(pdata);
	if (ret < 0)
		goto err;

	/* Verify that what we read from SPI is actually correct,
	 * by reading register which reset values are known. */
	ret = adf7242_read_reg(adf7242_pdata, REG_CH_FREQ0, &reg);
	if (!ret && reg != 128)
		ret = -EIO;
	if (ret < 0)
		goto err;
	ret = adf7242_read_reg(adf7242_pdata, REG_CH_FREQ1, &reg);
	if (!ret && reg != 169)
		ret = -EIO;
	if (ret < 0)
		goto err;
	ret = adf7242_read_reg(adf7242_pdata, REG_CH_FREQ2, &reg);
	if (!ret && reg != 3)
		ret = -EIO;
	if (ret < 0)
		goto err;

	ret = adf7242_write_reg(pdata, REG_PKT_CFG, 0);
	if (ret < 0)
		goto err;

	if (pdata->mode) {
		DBG(1, "Firmware size: %u bytes\r\n", sizeof(adf7242_firmware));

		ret = adf7242_upload_firmware(pdata,
				adf7242_firmware, sizeof(adf7242_firmware));
		if (ret < 0)
			goto err;

		ret = adf7242_verify_firmware(pdata,
				adf7242_firmware, sizeof(adf7242_firmware));
		if (ret < 0)
			goto err;

		ret = adf7242_write_reg(pdata, REG_FFILT_CFG,
				  ACCEPT_BEACON_FRAMES |
				  ACCEPT_DATA_FRAMES |
				  ACCEPT_ACK_FRAMES |
				  ACCEPT_MACCMD_FRAMES |
				  (pdata->mode & ADF_IEEE802154_PROMISCUOUS_MODE
				  ? ACCEPT_ALL_ADDRESS : 0) |
				  ACCEPT_RESERVED_FRAMES);
		if (ret < 0)
			goto err;

		ret = adf7242_write_reg(pdata, REG_AUTO_TX1,
				  MAX_FRAME_RETRIES(pdata->max_frame_retries) |
				  MAX_CCA_RETRIES(pdata->max_cca_retries));
		if (ret < 0)
			goto err;

		ret = adf7242_write_reg(pdata, REG_AUTO_TX2,
				  CSMA_MAX_BE(pdata->max_csma_be) |
				  CSMA_MIN_BE(pdata->min_csma_be));
		if (ret < 0)
			goto err;

		ret = adf7242_write_reg(pdata, REG_AUTO_CFG,
				  (pdata->mode & ADF_IEEE802154_HW_AACK ?
				   RX_AUTO_ACK_EN : 0));
		if (ret < 0)
			goto err;

		ret = adf7242_write_reg(pdata, REG_PKT_CFG, ADDON_EN);
		if (ret < 0)
			goto err;
	}

	ret = adf7242_write_reg(pdata, REG_EXTPA_MSC, 0xF1);
	if (ret < 0)
		goto err;

	ret = adf7242_write_reg(pdata, REG_RXFE_CFG, 0x1D);
	if (ret < 0)
		goto err;

	ret = adf7242_write_reg(pdata, REG_IRQ1_EN0, 0);
	if (ret < 0)
		goto err;

	adf7242_write_reg(pdata, REG_IRQ1_EN1,
			IRQ_RX_PKT_RCVD | adf7242_tx_irq(pdata));
	if (ret < 0)
		goto err;

	ret = adf7242_write_reg(pdata, REG_IRQ1_SRC1, 0xFF);
	if (ret < 0)
		goto err;

	ret = adf7242_write_reg(pdata, REG_IRQ1_SRC0, 0xFF);
	if (ret < 0)
		goto err;

	ret = adf7242_cmd(pdata, CMD_RC_PHY_RDY);
	if (ret < 0)
		goto err;

	DBG(1, "ADF7242 successfully initialized.\r\n");
	DBG(2, "%s :Exit\r\n", __func__);
	return 0;

err:
	fprintf(stderr, "Error initializing hardware: %s\r\n", strerror(-ret));
	return ret;
}

static int adf7242_init(void)
{
	int ret;

	if (adf7242_pdata->mode & ADF_IEEE802154_PROMISCUOUS_MODE)
		adf7242_pdata->mode &= ~ADF_IEEE802154_HW_AACK;

	DBG(1, "Initializing SPI bus=%u device=%u\r\n",
			adf7242_pdata->bus, adf7242_pdata->device);

	/* MSB first, 2 MHz, idle at low level, serial output changes on
	 * low to high front */
	ret = SPI_Init(adf7242_pdata->bus, 0, 2000000, 0, 1);
	if (ret < 0) {
		fprintf(stderr, "Error initializing SPI: %s\r\n",
				strerror(-ret));
		return ret;
	}

	ret = adf7242_hw_init(adf7242_pdata);
	return ret ? ret : 1;
}

static int adf7242_send(const void *buf, unsigned short len)
{
#ifdef RADIO_TX_LED
	tx_led ^= 1;
	RADIO_TX_LED = tx_led;
#endif
	DBG(1, "Sending packet with size %hu...\r\n", len);

	adf7242_write_fbuf(adf7242_pdata, buf, len);

	tx_status = -1;

	if (adf7242_pdata->mode & ADF_IEEE802154_AUTO_CSMA_CA) {
		adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);
		adf7242_cmd(adf7242_pdata, CMD_RC_CSMACA);
	} else {
		adf7242_cmd(adf7242_pdata, CMD_RC_TX);
	}

	/* Wait for the transfer to complete */
	while (tx_status < 0);

	if (tx_status == RADIO_TX_OK)
		DBG(1, "Packet of 0x%hx bytes sent\r\n", len);
	else if (tx_status == FAILURE_NOACK)
		DBG(1, "Error: No ACK received\r\n");
	else
		DBG(1, "Error sending frame: %hhu\r\n", tx_status);
	return tx_status;
}

static int adf7242_prepare(const void *buf, unsigned short len)
{
	memcpy(tx_buf, buf, len < sizeof(tx_buf) ? len : sizeof(tx_buf));
	return RADIO_TX_OK;
}

static int adf7242_transmit(unsigned short len)
{
	return adf7242_send(tx_buf, len);
}

static int adf7242_read(void *buf, unsigned short buf_len)
{
	uint8_t lqi, len = buf_len;
	int ret;

#ifdef RADIO_RX_LED
	rx_led ^= 1;
	RADIO_RX_LED = rx_led;
#endif

	ret = adf7242_read_fbuf(adf7242_pdata, buf, &len, &lqi);
	adf7242_cmd(adf7242_pdata, CMD_RC_RX);
	DBG(1, "Received a packet of %hhu bytes\r\n", len - 2);
	return ret < 0 ? ret : len - 2;
}

static int adf7242_channel_clear(void)
{
	/* TODO */
	return 1;
}

static int adf7242_receiving_packet(void)
{
	unsigned char status;
	int ret;

	ret = adf7242_status(adf7242_pdata, &status);
	if (ret < 0)
		return ret;

	return ((status & 0xf) == RC_STATUS_RX) && !(status & STAT_RC_READY);
}

static int adf7242_pending_packet(void)
{
	int ret = (int) packet_pending;
	packet_pending = false;
	return ret;
}

static radio_result_t adf7242_get_value(radio_param_t param,
		radio_value_t *value)
{
	switch (param) {
	case RADIO_CONST_CHANNEL_MIN:
		*value = 11;
		return RADIO_RESULT_OK;
	case RADIO_CONST_CHANNEL_MAX:
		*value = 26;
		return RADIO_RESULT_OK;
	case RADIO_PARAM_CHANNEL: {
		uint32_t freq;
		uint8_t byte0, byte1, byte2, status;
		int ret;

		adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);
		do {
			ret = adf7242_status(adf7242_pdata, &status);
			if (ret < 0)
				return RADIO_RESULT_ERROR;
		} while (!(status & (RC_STATUS_PHY_RDY | RC_STATUS_IDLE)));

		if (adf7242_read_reg(adf7242_pdata, REG_CH_FREQ0, &byte0) < 0 ||
				adf7242_read_reg(adf7242_pdata,
					REG_CH_FREQ1, &byte1) < 0 ||
				adf7242_read_reg(adf7242_pdata,
					REG_CH_FREQ2, &byte2) < 0)
			return RADIO_RESULT_ERROR;

		freq = byte0 | ((uint32_t) byte1 << 8)
			| ((uint32_t) byte2 << 16);
		*value = (freq / 100UL - 2405UL) / 5UL + 11UL;

		adf7242_cmd(adf7242_pdata, CMD_RC_RX);
		return RADIO_RESULT_OK;
	}
	default:
		return RADIO_RESULT_NOT_SUPPORTED;
	}
}

static radio_result_t adf7242_set_value(radio_param_t param,
		radio_value_t value)
{
	switch (param) {
	case RADIO_PARAM_CHANNEL: {
		uint32_t freq;
		uint8_t byte0, byte1, byte2;

		if (value < 11 || value > 26)
			return RADIO_RESULT_INVALID_VALUE;

		/* NOTE: I do this in three steps, otherwise it doesn't
		 * calculate anything. Hurray for toolchain bugs. */
		freq = value - 11;
		freq = freq * 5UL + 2405UL;
		freq *= 100UL;

		byte0 = (uint8_t) freq;
		byte1 = freq >> 8;
		byte2 = freq >> 16;

		adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);

		if (adf7242_write_reg(adf7242_pdata, REG_CH_FREQ0, byte0) < 0 ||
				adf7242_write_reg(adf7242_pdata,
					REG_CH_FREQ1, byte1) < 0 ||
				adf7242_write_reg(adf7242_pdata,
					REG_CH_FREQ2, byte2) < 0)
			return RADIO_RESULT_ERROR;

		DBG(1, "Switching to channel %u (%lu0 kHz)\r\n",
				(unsigned int) value, freq);
		return RADIO_RESULT_OK;
	}
	case RADIO_PARAM_PAN_ID: {
		uint16_t pan_id = value;

		adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);

		adf7242_write_reg(adf7242_pdata, REG_PAN_ID0, pan_id);
		adf7242_write_reg(adf7242_pdata, REG_PAN_ID1, pan_id >> 8);
		adf7242_cmd(adf7242_pdata, CMD_RC_RX);
		return RADIO_RESULT_OK;
	}
	case RADIO_PARAM_16BIT_ADDR: {
		uint16_t addr = value;

		adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);

		adf7242_write_reg(adf7242_pdata, REG_SHORT_ADDR_0, addr);
		adf7242_write_reg(adf7242_pdata, REG_SHORT_ADDR_1, addr >> 8);
		adf7242_cmd(adf7242_pdata, CMD_RC_RX);
		return RADIO_RESULT_OK;
	}
	default:
		DBG(1, "%s tried to set param %u, but failed\r\n",
				__func__, param);
		return RADIO_RESULT_NOT_SUPPORTED;
	}
}

static int adf7242_off(void)
{
	int ret = adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);
	if (ret < 0)
		return RADIO_RESULT_ERROR;
	else
		return RADIO_POWER_MODE_OFF;
}

static int adf7242_on(void)
{
	int ret = adf7242_cmd(adf7242_pdata, CMD_RC_RX);
	if (ret < 0)
		return RADIO_RESULT_ERROR;
	else
		return RADIO_POWER_MODE_ON;
}

static radio_result_t adf7242_get_object(radio_param_t param,
		void *dest, size_t size)
{
	return RADIO_RESULT_NOT_SUPPORTED;
}

static radio_result_t adf7242_set_object(radio_param_t param,
		const void *src, size_t size)
{
	switch (param) {
	case RADIO_PARAM_64BIT_ADDR: {
		const char *addr = (const char *) src;

		adf7242_cmd(adf7242_pdata, CMD_RC_PHY_RDY);

		adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_0, addr[7]);
		adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_1, addr[6]);
		adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_2, addr[5]);
		adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_3, addr[4]);
		adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_4, addr[3]);
		adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_5, addr[2]);
		adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_6, addr[1]);
		adf7242_write_reg(adf7242_pdata, REG_IEEE_ADDR_7, addr[0]);
		adf7242_cmd(adf7242_pdata, CMD_RC_RX);
		return RADIO_RESULT_OK;
	}
	default:
		DBG(1, "%s tried to set object %u, but failed\r\n",
				__func__, param);
		return RADIO_RESULT_NOT_SUPPORTED;
	}
}

const struct radio_driver adf7242_driver = {
	.init = adf7242_init,
	.send = adf7242_send,
	.prepare = adf7242_prepare,
	.transmit = adf7242_transmit,
	.read = adf7242_read,
	.channel_clear = adf7242_channel_clear,
	.receiving_packet = adf7242_receiving_packet,
	.pending_packet = adf7242_pending_packet,
	.on = adf7242_on,
	.off = adf7242_off,
	.get_value = adf7242_get_value,
	.set_value = adf7242_set_value,
	.get_object = adf7242_get_object,
	.set_object = adf7242_set_object,
};
