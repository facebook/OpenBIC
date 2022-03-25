#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/peci.h>
#include "hal_peci.h"
#include "libutil.h"

const struct device *dev;

int peci_init()
{
	dev = device_get_binding("PECI");
	int ret;
	uint32_t bitrate = 1000;
	if (!dev) {
		printf("peci device not found");
		return 0;
	}
	ret = peci_config(dev, bitrate);
	if (ret) {
		printf("set bitrate %dKbps failed %d\n", bitrate, ret);
		return ret;
	}
	ret = peci_enable(dev);
	if (ret) {
		printf("peci enable failed %d\n", ret);
		return ret;
	}

	return ret;
}

int peci_ping(uint8_t address)
{
	struct peci_msg pkgcfg;
	int ret;

	pkgcfg.addr = address;
	pkgcfg.cmd_code = PECI_CMD_PING;
	pkgcfg.tx_buffer.buf = NULL;
	pkgcfg.tx_buffer.len = 0x0;
	pkgcfg.rx_buffer.len = 0x0;

	ret = peci_transfer(dev, &pkgcfg);
	if (ret) {
		printf("[%s] Failed to send the PECI PING command(0x%x), status: %d\n", __func__,
		       pkgcfg.cmd_code, ret);
	}

	SAFE_FREE(pkgcfg.tx_buffer.buf);
	return ret;
}

int peci_read(uint8_t cmd, uint8_t address, uint8_t u8Index, uint16_t u16Param, uint8_t u8ReadLen,
	      uint8_t *readBuf)
{
	struct peci_msg rdpkgcfg;
	int ret;

	if (readBuf == NULL) {
		printf("PECI read buffer was passed in as null\n");
		return -1;
	}
	rdpkgcfg.cmd_code = cmd;
	rdpkgcfg.addr = address;
	rdpkgcfg.tx_buffer.len = 0x05;
	rdpkgcfg.rx_buffer.len = u8ReadLen;
	rdpkgcfg.tx_buffer.buf = (uint8_t *)malloc(rdpkgcfg.tx_buffer.len * sizeof(uint8_t));
	if (rdpkgcfg.tx_buffer.buf == NULL) {
		printf("Could not initialize memory for tx_buffer\n");
		return -1;
	}
	rdpkgcfg.rx_buffer.buf = readBuf;
	rdpkgcfg.tx_buffer.buf[0] = 0x00;
	rdpkgcfg.tx_buffer.buf[1] = u8Index;
	rdpkgcfg.tx_buffer.buf[2] = u16Param & 0xff;
	rdpkgcfg.tx_buffer.buf[3] = u16Param >> 8;
	ret = peci_transfer(dev, &rdpkgcfg);

	if (DEBUG_PECI) {
		uint8_t index;
		for (index = 0; index < 5; index++)
			printf("%02x ", readBuf[index]);
		printf("\n");
	}

	if (ret) {
		printf("[%s] Failed to send PECI Command(0x%x), status: %d\n", __func__,
		       rdpkgcfg.cmd_code, ret);
	}

	SAFE_FREE(rdpkgcfg.tx_buffer.buf);
	return ret;
}

int peci_write(uint8_t cmd, uint8_t address, uint8_t u8ReadLen, uint8_t *readBuf,
	       uint8_t u8WriteLen, uint8_t *writeBuf)
{
	if ((readBuf == NULL) || (writeBuf == NULL)) {
		return -1;
	}

	struct peci_msg wrpkgcfg;
	int ret;

	wrpkgcfg.addr = address;
	wrpkgcfg.cmd_code = cmd;
	wrpkgcfg.tx_buffer.len = u8WriteLen;
	wrpkgcfg.rx_buffer.len = u8ReadLen;
	wrpkgcfg.rx_buffer.buf = readBuf;
	wrpkgcfg.tx_buffer.buf = writeBuf;

	ret = peci_transfer(dev, &wrpkgcfg);
	if (ret) {
		printf("peci write failed %d\n", ret);
		return ret;
	}

	return ret;
}

bool peci_retry_read(uint8_t cmd, uint8_t address, uint8_t u8Index, uint16_t u16Param,
		     uint8_t u8ReadLen, uint8_t *readBuf)
{
	uint8_t i, ret, retry = 5;

	if (readBuf == NULL)
		return false;

	for (i = 0; i < retry; ++i) {
		k_msleep(10);
		memcpy(&readBuf[0], 0, u8ReadLen * sizeof(uint8_t));
		ret = peci_read(cmd, address, u8Index, u16Param, u8ReadLen, readBuf);
		if (!ret) {
			if (readBuf[0] == PECI_CC_RSP_SUCCESS) {
				return true;
			}
		}
	}
	return false;
}
