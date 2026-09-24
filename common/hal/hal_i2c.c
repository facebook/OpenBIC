/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/portability/cmsis_os2.h>
#include "hal_i2c.h"
#include "timer.h"
#include "plat_i2c.h"
#include "libutil.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hal_i2c);

#if defined(CONFIG_I2C_ASPEED)
#define AST_1030_I2C_BASE 0x7e7b0080
#define AST_1030_I2C_REG_LEN 0x80
#define AST_1030_SLAVE_EN BIT(1)

/* 0x40 : Slave Device Address Register */
#define AST_I2CS_ADDR_CTRL 0x40
#define AST_I2CS_ADDR1_MASK 0x7F

#elif (CONFIG_I2C_NPCM4XX)
#define NPCM4XX_I2C_BASE 0x40006000
#define NPCM4XX_I2C_REG_LEN 0x200

#define NPCM4XX_SMBnADDR1 0x08
#define NPCM4XX_I2CS_ADDR1_MASK 0x7F
#define NPCM4XX_SMBnADDR_SAEN BIT(7)

#else /* defined(CONFIG_I2C_ASPEED) */
#endif /* defined(CONFIG_I2C_ASPEED) */

static const struct device *dev_i2c[I2C_BUS_MAX_NUM];

struct k_mutex i2c_mutex[I2C_BUS_MAX_NUM];

int i2c_freq_set(uint8_t i2c_bus, uint8_t i2c_speed_mode, uint8_t en_slave)
{
	if (check_i2c_bus_valid(i2c_bus) < 0) {
		LOG_ERR("i2c bus %d is invalid", i2c_bus);
		return -1;
	}

	uint32_t dev_config_raw;
	dev_config_raw = I2C_MODE_CONTROLLER | I2C_SPEED_SET(i2c_speed_mode);

	if (i2c_configure(dev_i2c[i2c_bus], dev_config_raw)) {
		LOG_ERR("i2c freq set failed");
		return -1;
	}

	if (en_slave) {
#if defined(CONFIG_I2C_ASPEED)
		uint32_t *addr = (uint32_t *)(AST_1030_I2C_BASE + (i2c_bus * AST_1030_I2C_REG_LEN));
		*addr |= AST_1030_SLAVE_EN;
#elif (CONFIG_I2C_NPCM4XX)
		uint8_t *addr = (uint8_t *)(NPCM4XX_I2C_BASE + NPCM4XX_SMBnADDR1 +
					    (i2c_bus * NPCM4XX_I2C_REG_LEN));
		*addr |= NPCM4XX_SMBnADDR_SAEN;
#else /* defined(CONFIG_I2C_ASPEED) */
#endif /* defined(CONFIG_I2C_ASPEED) */
	}

	return 0;
}

int i2c_addr_set(uint8_t i2c_bus, uint8_t i2c_addr)
{
	if (check_i2c_bus_valid(i2c_bus) < 0) {
		LOG_ERR("i2c bus %d is invalid", i2c_bus);
		return -1;
	}

	i2c_addr = i2c_addr >> 1; // to 7-bit target address

#if defined(CONFIG_I2C_ASPEED)
	uint32_t base = AST_1030_I2C_BASE + (i2c_bus * AST_1030_I2C_REG_LEN);
	sys_write32(i2c_addr | (sys_read32(base + AST_I2CS_ADDR_CTRL) & ~AST_I2CS_ADDR1_MASK),
		    base + AST_I2CS_ADDR_CTRL);
#elif (CONFIG_I2C_NPCM4XX)
	uint32_t base = NPCM4XX_I2C_BASE + (i2c_bus * NPCM4XX_I2C_REG_LEN);

	//i2c bus index is 0 based.
	if ((i2c_addr == 0)) {
		sys_write8((sys_read8(base + NPCM4XX_SMBnADDR1) & ~NPCM4XX_SMBnADDR_SAEN),
			   base + NPCM4XX_SMBnADDR1);
	} else if ((i2c_addr != 0)) {
		/* set slave addr 1 */
		sys_write8((i2c_addr | NPCM4XX_SMBnADDR_SAEN), base + NPCM4XX_SMBnADDR1);
	}
#else /* defined(CONFIG_I2C_ASPEED) */
#endif /* defined(CONFIG_I2C_ASPEED) */

	return 0;
}

int i2c_master_read(I2C_MSG *msg, uint8_t retry)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -1);

	LOG_DBG("bus %d, addr %x, rxlen %d, txlen %d", msg->bus, msg->target_addr, msg->rx_len,
		msg->tx_len);
	LOG_HEXDUMP_DBG(msg->data, msg->tx_len, "txbuf");

	if (check_i2c_bus_valid(msg->bus) < 0) {
		LOG_ERR("i2c bus %d is invalid", msg->bus);
		return -1;
	}

	if (msg->rx_len == 0) {
		LOG_ERR("rx_len = 0");
		return EMSGSIZE;
	}

	if (msg->rx_len > I2C_BUFF_SIZE) {
		LOG_ERR("rx_len %d is over limit %d", msg->rx_len, I2C_BUFF_SIZE);
		return -1;
	}

	if (msg->tx_len > I2C_BUFF_SIZE) {
		LOG_ERR("tx_len %d is over limit %d", msg->tx_len, I2C_BUFF_SIZE);
		return -1;
	}

	int status;
	status = k_mutex_lock(&i2c_mutex[msg->bus], K_MSEC(1000));
	if (status) {
		LOG_ERR("I2C %d master read get mutex timeout with ret %d", msg->bus, status);
		return ENOLCK;
	}

	int ret = -1;
	uint8_t *txbuf = NULL, *rxbuf = NULL;
	txbuf = (uint8_t *)malloc(I2C_BUFF_SIZE);
	if (!txbuf) {
		LOG_ERR("Failed to malloc txbuf");
		goto exit;
	}
	rxbuf = (uint8_t *)malloc(I2C_BUFF_SIZE);
	if (!rxbuf) {
		LOG_ERR("Failed to malloc rxbuf");
		goto exit;
	}
	memcpy(txbuf, &msg->data[0], msg->tx_len);

	uint8_t i;
	for (i = 0; i <= retry; i++) {
		if (msg->tx_len > 0) {
			ret = i2c_write_read(dev_i2c[msg->bus], msg->target_addr, txbuf,
					     msg->tx_len, rxbuf, msg->rx_len);
		} else {
			ret = i2c_read(dev_i2c[msg->bus], rxbuf, msg->rx_len, msg->target_addr);
		}
		if (ret == 0) { // i2c write read success
			memcpy(&msg->data[0], rxbuf, msg->rx_len);
			LOG_HEXDUMP_DBG(msg->data, msg->rx_len, "rxbuf");
			break;
		}
	}

	if (i > retry)
		LOG_ERR("I2C %d master read retry reach max with ret %d", msg->bus, ret);

exit:
	SAFE_FREE(txbuf);
	SAFE_FREE(rxbuf);

	status = k_mutex_unlock(&i2c_mutex[msg->bus]);
	if (status)
		LOG_ERR("I2C %d master read release mutex fail with ret %d", msg->bus, status);

	return ret;
}

int i2c_master_write(I2C_MSG *msg, uint8_t retry)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -1);

	LOG_DBG("bus %d, addr %x, txlen %d", msg->bus, msg->target_addr, msg->tx_len);
	LOG_HEXDUMP_DBG(msg->data, msg->tx_len, "txbuf");

	if (check_i2c_bus_valid(msg->bus) < 0) {
		LOG_ERR("i2c bus %d is invalid", msg->bus);
		return -1;
	}

	if (msg->tx_len > I2C_BUFF_SIZE) {
		LOG_ERR("tx_len %d is over limit %d", msg->tx_len, I2C_BUFF_SIZE);
		return -1;
	}

	int status;
	status = k_mutex_lock(&i2c_mutex[msg->bus], K_MSEC(1000));
	if (status) {
		LOG_ERR("I2C %d master write get mutex timeout with ret %d", msg->bus, status);
		return ENOLCK;
	}

	int ret = -1;
	uint8_t *txbuf = NULL;
	txbuf = (uint8_t *)malloc(I2C_BUFF_SIZE);
	if (!txbuf) {
		LOG_ERR("Failed to malloc txbuf");
		goto exit;
	}
	memcpy(txbuf, &msg->data[0], msg->tx_len);

	uint8_t i;
	for (i = 0; i <= retry; i++) {
		ret = i2c_write(dev_i2c[msg->bus], txbuf, msg->tx_len, msg->target_addr);
		if (ret == 0) // i2c write success
			break;
	}

	if (i > retry)
		LOG_ERR("I2C %d master write retry reach max with ret %d", msg->bus, ret);

exit:
	SAFE_FREE(txbuf);

	status = k_mutex_unlock(&i2c_mutex[msg->bus]);
	if (status)
		LOG_ERR("I2C %d master write release mutex fail with ret %d", msg->bus, status);

	return ret;
}

int i2c_master_read_without_mutex(I2C_MSG *msg, uint8_t retry)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -1);

	LOG_DBG("bus %d, addr %x, rxlen %d, txlen %d", msg->bus, msg->target_addr, msg->rx_len,
		msg->tx_len);
	LOG_HEXDUMP_DBG(msg->data, msg->tx_len, "txbuf");

	if (check_i2c_bus_valid(msg->bus) < 0) {
		LOG_ERR("i2c bus %d is invalid", msg->bus);
		return -1;
	}

	if (msg->rx_len == 0) {
		LOG_ERR("rx_len = 0");
		return EMSGSIZE;
	}

	if (msg->rx_len > I2C_BUFF_SIZE) {
		LOG_ERR("rx_len %d is over limit %d", msg->rx_len, I2C_BUFF_SIZE);
		return -1;
	}

	if (msg->tx_len > I2C_BUFF_SIZE) {
		LOG_ERR("tx_len %d is over limit %d", msg->tx_len, I2C_BUFF_SIZE);
		return -1;
	}

	int ret = -1;
	uint8_t *txbuf = NULL, *rxbuf = NULL;
	txbuf = (uint8_t *)malloc(I2C_BUFF_SIZE);
	if (!txbuf) {
		LOG_ERR("Failed to malloc txbuf");
		goto exit;
	}
	rxbuf = (uint8_t *)malloc(I2C_BUFF_SIZE);
	if (!rxbuf) {
		LOG_ERR("Failed to malloc rxbuf");
		goto exit;
	}
	memcpy(txbuf, &msg->data[0], msg->tx_len);

	uint8_t i;
	for (i = 0; i <= retry; i++) {
		if (msg->tx_len > 0) {
			ret = i2c_write_read(dev_i2c[msg->bus], msg->target_addr, txbuf,
					     msg->tx_len, rxbuf, msg->rx_len);
		} else {
			ret = i2c_read(dev_i2c[msg->bus], rxbuf, msg->rx_len, msg->target_addr);
		}
		if (ret == 0) { // i2c write read success
			memcpy(&msg->data[0], rxbuf, msg->rx_len);
			LOG_HEXDUMP_DBG(msg->data, msg->rx_len, "rxbuf");
			break;
		}
	}

	if (i > retry)
		LOG_ERR("I2C %d master read retry reach max with ret %d", msg->bus, ret);

exit:
	SAFE_FREE(txbuf);
	SAFE_FREE(rxbuf);

	return ret;
}

int i2c_master_write_without_mutex(I2C_MSG *msg, uint8_t retry)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -1);

	LOG_DBG("bus %d, addr %x, txlen %d", msg->bus, msg->target_addr, msg->tx_len);
	LOG_HEXDUMP_DBG(msg->data, msg->tx_len, "txbuf");

	if (check_i2c_bus_valid(msg->bus) < 0) {
		LOG_ERR("i2c bus %d is invalid", msg->bus);
		return -1;
	}

	if (msg->tx_len > I2C_BUFF_SIZE) {
		LOG_ERR("tx_len %d is over limit %d", msg->tx_len, I2C_BUFF_SIZE);
		return -1;
	}

	int ret = -1;
	uint8_t *txbuf = NULL;
	txbuf = (uint8_t *)malloc(I2C_BUFF_SIZE);
	if (!txbuf) {
		LOG_ERR("Failed to malloc txbuf");
		goto exit;
	}
	memcpy(txbuf, &msg->data[0], msg->tx_len);

	uint8_t i;
	for (i = 0; i <= retry; i++) {
		ret = i2c_write(dev_i2c[msg->bus], txbuf, msg->tx_len, msg->target_addr);
		if (ret == 0) // i2c write success
			break;
	}

	if (i > retry)
		LOG_ERR("I2C %d master write retry reach max with ret %d", msg->bus, ret);

exit:
	SAFE_FREE(txbuf);

	return ret;
}

int i2c_spd_reg_read(I2C_MSG *msg, bool is_nvm)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -1);

	if (check_i2c_bus_valid(msg->bus) < 0) {
		LOG_ERR("i2c bus %d is invalid", msg->bus);
		return -1;
	}

	int ret = 0;
	uint8_t addr = (msg->data[1] << 8 | msg->data[0]) % 64;
	uint8_t block_addr = (msg->data[1] << 8 | msg->data[0]) / 64;
	uint8_t offset0 = GETBIT(block_addr, 0) << 6 | (addr & GENMASK(5, 0));
	uint8_t offset1 = (block_addr >> 1) & 0xF;
	if (is_nvm) {
		offset0 = SETBIT(offset0, 7);
	} else {
		offset0 = CLEARBIT(offset0, 7);
	}
	msg->tx_len = 2;
	msg->data[0] = offset0;
	msg->data[1] = offset1;

	ret = i2c_master_read(msg, 3);
	if (ret != 0) {
		LOG_ERR("Failed to read SPD bus0x%x addr0x%x offset0x%x %x, ret: %d", msg->bus,
			msg->target_addr, offset1, offset0, ret);
	}

	return ret;
}

void i2c_scan(uint8_t bus, uint8_t *target_addr, uint8_t *target_addr_len)
{
	CHECK_NULL_ARG(target_addr);
	CHECK_NULL_ARG(target_addr_len);

	uint8_t first = 0x04, last = 0x77;
	*target_addr_len = 0;

	if (check_i2c_bus_valid(bus) < 0) {
		LOG_ERR("i2c bus %d is invalid", bus);
		return;
	}

	for (uint8_t i = 0; i <= last; i += 16) {
		for (uint8_t j = 0; j < 16; j++) {
			if (i + j < first || i + j > last) {
				continue;
			}

			struct i2c_msg msgs[1];
			uint8_t dst;

			/* Send the address to read from */
			msgs[0].buf = &dst;
			msgs[0].len = 0U;
			msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
			if (i2c_transfer(dev_i2c[bus], &msgs[0], 1, i + j) == 0) {
				target_addr[*target_addr_len] = (i + j) << 1;
				(*target_addr_len)++;
			}
		}
	}
}

#if defined(CONFIG_I2C_MCUX_LPI2C)
static void util_init_I2C_mcux_flexcomm(void)
{
	int status;

#ifdef DEV_I2C_2
	dev_i2c[I2C_BUS_LCD_HEADER] = DEVICE_DT_GET(DT_NODELABEL(flexcomm2_lpi2c2));
	status = k_mutex_init(&i2c_mutex[I2C_BUS_LCD_HEADER]);
	if (status)
		LOG_ERR("i2c%d mutex init fail", I2C_BUS_LCD_HEADER);
#endif
#ifdef DEV_I2C_3
	dev_i2c[I2C_BUS_ARDUINO_HEADER] = DEVICE_DT_GET(DT_NODELABEL(flexcomm3_lpi2c3));
	status = k_mutex_init(&i2c_mutex[I2C_BUS_ARDUINO_HEADER]);
	if (status)
		LOG_ERR("i2c%d mutex init fail", I2C_BUS_ARDUINO_HEADER);
#endif
}

#if defined(CONFIG_I2C_TARGET)
/* IPMB-over-I2C target-mode glue. IPMB frames on the wire begin with
 * the responder's own slave address (8-bit form) for checksum
 * purposes, but real I2C carries addressing out-of-band in the
 * address phase, not as a literal data byte - so on write_requested()
 * we synthesize that leading byte to match what
 * common/service/ipmb/ipmb.c's validate_checksum()/ipmb_decode()
 * expect, exactly as a real IPMB-capable I2C slave controller would
 * present it. IPMB is write-only in both directions (a "response" is
 * simply a separate write from the other node back to our address),
 * so only write_requested/write_received/stop are needed - no read
 * callback exists in this protocol.
 */
struct ipmb_target_msg {
	uint8_t len;
	uint8_t data[I2C_BUFF_SIZE];
};

struct ipmb_target_ctx {
	struct i2c_target_config cfg;
	uint8_t bus;
	bool is_ipmb_bus;
	struct ipmb_target_msg accum;
};

static struct ipmb_target_ctx ipmb_target_ctx[I2C_BUS_MAX_NUM];
static struct k_msgq ipmb_target_msgq[I2C_BUS_MAX_NUM];
static char __aligned(4)
	ipmb_target_msgq_buf[I2C_BUS_MAX_NUM][2 * sizeof(struct ipmb_target_msg)];

static int ipmb_target_write_requested(struct i2c_target_config *config)
{
	struct ipmb_target_ctx *ctx = CONTAINER_OF(config, struct ipmb_target_ctx, cfg);

	ctx->accum.data[0] = config->address << 1;
	ctx->accum.len = 1;
	return 0;
}

static int ipmb_target_write_received(struct i2c_target_config *config, uint8_t val)
{
	struct ipmb_target_ctx *ctx = CONTAINER_OF(config, struct ipmb_target_ctx, cfg);

	if (ctx->accum.len < I2C_BUFF_SIZE) {
		ctx->accum.data[ctx->accum.len++] = val;
	}
	return 0;
}

static int ipmb_target_stop(struct i2c_target_config *config)
{
	struct ipmb_target_ctx *ctx = CONTAINER_OF(config, struct ipmb_target_ctx, cfg);

	/* len > 1: more than just the synthesized address byte arrived */
	if (ctx->accum.len > 1) {
		if (k_msgq_put(&ipmb_target_msgq[ctx->bus], &ctx->accum, K_NO_WAIT)) {
			LOG_ERR("ipmb target[%d]: rx queue full, dropping message", ctx->bus);
		}
	}
	return 0;
}

static const struct i2c_target_callbacks ipmb_target_callbacks = {
	.write_requested = ipmb_target_write_requested,
	.write_received = ipmb_target_write_received,
	.stop = ipmb_target_stop,
};

int ipmb_target_register(uint8_t bus, uint8_t addr)
{
	if (bus >= I2C_BUS_MAX_NUM || !dev_i2c[bus]) {
		return -EINVAL;
	}

	k_msgq_init(&ipmb_target_msgq[bus], ipmb_target_msgq_buf[bus],
		    sizeof(struct ipmb_target_msg), 2);

	ipmb_target_ctx[bus].bus = bus;
	ipmb_target_ctx[bus].cfg.address = addr;
	ipmb_target_ctx[bus].cfg.callbacks = &ipmb_target_callbacks;

	int ret = i2c_target_register(dev_i2c[bus], &ipmb_target_ctx[bus].cfg);

	if (!ret) {
		ipmb_target_ctx[bus].is_ipmb_bus = true;
	}
	return ret;
}

int ipmb_target_read(uint8_t bus, uint8_t *buf, uint8_t *len, k_timeout_t timeout)
{
	struct ipmb_target_msg msg;

	if (bus >= I2C_BUS_MAX_NUM || !buf || !len) {
		return -EINVAL;
	}

	if (k_msgq_get(&ipmb_target_msgq[bus], &msg, timeout)) {
		return -EAGAIN;
	}

	memcpy(buf, msg.data, msg.len);
	*len = msg.len;
	return 0;
}

/* MCTP-over-SMBus target-mode glue for common/service/mctp/mctp_smbus.c's
 * i2c_target_read() dependency (the one piece of that file that isn't
 * already portable - see meta-facebook/mcx-n9xx-evk/README.md). Unlike
 * IPMB, MCTP frames carry no synthesized leading address byte, so this
 * just accumulates the raw bytes as sent. It also can't share a bus with
 * the IPMB target above: the mainline i2c_target driver only supports one
 * registered target address per bus/device instance (unlike the old
 * Aspeed hal_i2c_target.c's three), so MCTP needs its own bus - see
 * plat_mctp.c for which one this board uses.
 */
struct mctp_target_msg {
	uint16_t len;
	uint8_t data[I2C_BUFF_SIZE];
};

struct mctp_target_ctx {
	struct i2c_target_config cfg;
	uint8_t bus;
	bool is_mctp_bus;
	struct mctp_target_msg accum;
};

/* Depth needs enough headroom to hold every raw fragment of a
 * multi-packet message the I2C target hardware ACKs before
 * mctp_rx_task() gets scheduled to drain the queue - a burst of
 * writes arrives back-to-back with no gap for the consumer to run
 * between them. Was 2: fine for the 1-2 packet messages this got
 * verified against first, but a real 4-fragment message dropped
 * fragments 3 and 4 here (silent "rx queue full" - see below) even
 * though every I2C write itself succeeded. */
#define MCTP_TARGET_MSGQ_DEPTH 8

static struct mctp_target_ctx mctp_target_ctx[I2C_BUS_MAX_NUM];
static struct k_msgq mctp_target_msgq[I2C_BUS_MAX_NUM];
static char __aligned(4) mctp_target_msgq_buf[I2C_BUS_MAX_NUM][MCTP_TARGET_MSGQ_DEPTH *
								 sizeof(struct mctp_target_msg)];

static int mctp_i2c_target_write_requested(struct i2c_target_config *config)
{
	struct mctp_target_ctx *ctx = CONTAINER_OF(config, struct mctp_target_ctx, cfg);

	ctx->accum.len = 0;
	return 0;
}

static int mctp_i2c_target_write_received(struct i2c_target_config *config, uint8_t val)
{
	struct mctp_target_ctx *ctx = CONTAINER_OF(config, struct mctp_target_ctx, cfg);

	if (ctx->accum.len < I2C_BUFF_SIZE) {
		ctx->accum.data[ctx->accum.len++] = val;
	}
	return 0;
}

static int mctp_i2c_target_stop(struct i2c_target_config *config)
{
	struct mctp_target_ctx *ctx = CONTAINER_OF(config, struct mctp_target_ctx, cfg);

	LOG_DBG("mctp target[%d]: stop, accum.len=%d", ctx->bus, ctx->accum.len);

	if (ctx->accum.len > 0) {
		if (k_msgq_put(&mctp_target_msgq[ctx->bus], &ctx->accum, K_NO_WAIT)) {
			LOG_ERR("mctp target[%d]: rx queue full, dropping message", ctx->bus);
		}
	}
	return 0;
}

static const struct i2c_target_callbacks mctp_i2c_target_callbacks = {
	.write_requested = mctp_i2c_target_write_requested,
	.write_received = mctp_i2c_target_write_received,
	.stop = mctp_i2c_target_stop,
};

int mctp_i2c_target_register(uint8_t bus, uint8_t addr)
{
	if (bus >= I2C_BUS_MAX_NUM || !dev_i2c[bus]) {
		return -EINVAL;
	}

	k_msgq_init(&mctp_target_msgq[bus], mctp_target_msgq_buf[bus],
		    sizeof(struct mctp_target_msg), MCTP_TARGET_MSGQ_DEPTH);

	mctp_target_ctx[bus].bus = bus;
	mctp_target_ctx[bus].cfg.address = addr;
	mctp_target_ctx[bus].cfg.callbacks = &mctp_i2c_target_callbacks;

	int ret = i2c_target_register(dev_i2c[bus], &mctp_target_ctx[bus].cfg);

	if (!ret) {
		mctp_target_ctx[bus].is_mctp_bus = true;
	}
	return ret;
}

/* Return/argument shape matches the old hal_i2c_target.c i2c_target_read()
 * this replaces, since mctp_smbus.c calls it directly: 0 = success,
 * nonzero = failure, blocks until a frame arrives. */
uint8_t mctp_i2c_target_read(uint8_t bus, uint8_t *buf, uint16_t max_len, uint16_t *out_len)
{
	struct mctp_target_msg msg;

	if (bus >= I2C_BUS_MAX_NUM || !buf || !out_len) {
		return 1;
	}

	if (k_msgq_get(&mctp_target_msgq[bus], &msg, K_FOREVER)) {
		return 1;
	}

	if (msg.len > max_len) {
		LOG_ERR("mctp target[%d]: message len %d exceeds caller buffer %d", bus, msg.len,
			max_len);
		return 1;
	}

	memcpy(buf, msg.data, msg.len);
	*out_len = msg.len;
	return 0;
}
#endif /* CONFIG_I2C_TARGET */
#endif /* CONFIG_I2C_MCUX_LPI2C */

void util_init_I2C(void)
{
#if defined(CONFIG_I2C_MCUX_LPI2C)
	util_init_I2C_mcux_flexcomm();
}
#else
static void util_init_I2C_device_get_binding(void)
{
	int status;

#ifdef DEV_I2C_0
	dev_i2c[0] = device_get_binding("I2C_0");
	status = k_mutex_init(&i2c_mutex[0]);
	if (status)
		LOG_ERR("i2c0 mutex init fail");
#endif
#ifdef DEV_I2C_1
	dev_i2c[1] = device_get_binding("I2C_1");
	status = k_mutex_init(&i2c_mutex[1]);
	if (status)
		LOG_ERR("i2c1 mutex init fail");
#endif
#ifdef DEV_I2C_2
	dev_i2c[2] = device_get_binding("I2C_2");
	status = k_mutex_init(&i2c_mutex[2]);
	if (status)
		LOG_ERR("i2c2 mutex init fail");
#endif
#ifdef DEV_I2C_3
	dev_i2c[3] = device_get_binding("I2C_3");
	status = k_mutex_init(&i2c_mutex[3]);
	if (status)
		LOG_ERR("i2c3 mutex init fail");
#endif
#ifdef DEV_I2C_4
	dev_i2c[4] = device_get_binding("I2C_4");
	status = k_mutex_init(&i2c_mutex[4]);
	if (status)
		LOG_ERR("i2c4 mutex init fail");
#endif
#ifdef DEV_I2C_5
	dev_i2c[5] = device_get_binding("I2C_5");
	status = k_mutex_init(&i2c_mutex[5]);
	if (status)
		LOG_ERR("i2c5 mutex init fail");
#endif
#ifdef DEV_I2C_6
	dev_i2c[6] = device_get_binding("I2C_6");
	status = k_mutex_init(&i2c_mutex[6]);
	if (status)
		LOG_ERR("i2c6 mutex init fail");
#endif
#ifdef DEV_I2C_7
	dev_i2c[7] = device_get_binding("I2C_7");
	status = k_mutex_init(&i2c_mutex[7]);
	if (status)
		LOG_ERR("i2c7 mutex init fail");
#endif
#ifdef DEV_I2C_8
	dev_i2c[8] = device_get_binding("I2C_8");
	status = k_mutex_init(&i2c_mutex[8]);
	if (status)
		LOG_ERR("i2c8 mutex init fail");
#endif
#ifdef DEV_I2C_9
	dev_i2c[9] = device_get_binding("I2C_9");
	status = k_mutex_init(&i2c_mutex[9]);
	if (status)
		LOG_ERR("i2c9 mutex init fail");
#endif
#ifdef DEV_I2C_10
	dev_i2c[10] = device_get_binding("I2C_10");
	status = k_mutex_init(&i2c_mutex[10]);
	if (status)
		LOG_ERR("i2c10 mutex init fail");
#endif
#ifdef DEV_I2C_11
	dev_i2c[11] = device_get_binding("I2C_11");
	status = k_mutex_init(&i2c_mutex[11]);
	if (status)
		LOG_ERR("i2c11 mutex init fail");
#endif
#if defined(CONFIG_I2C_ASPEED)
#ifdef DEV_I2C_12
	dev_i2c[12] = device_get_binding("I2C_12");
	status = k_mutex_init(&i2c_mutex[12]);
	if (status)
		LOG_ERR("i2c12 mutex init fail");
#endif
#ifdef DEV_I2C_13
	dev_i2c[13] = device_get_binding("I2C_13");
	status = k_mutex_init(&i2c_mutex[13]);
	if (status)
		LOG_ERR("i2c13 mutex init fail");
#endif
#ifdef DEV_I2C_14
	dev_i2c[14] = device_get_binding("I2C_14");
	status = k_mutex_init(&i2c_mutex[14]);
	if (status)
		LOG_ERR("i2c14 mutex init fail");
#endif
#ifdef DEV_I2C_15
	dev_i2c[15] = device_get_binding("I2C_15");
	status = k_mutex_init(&i2c_mutex[15]);
	if (status)
		LOG_ERR("i2c15 mutex init fail");
#endif
#endif /* CONFIG_I2C_ASPEED */
}

void util_init_I2C(void)
{
	util_init_I2C_device_get_binding();
}
#endif /* CONFIG_I2C_MCUX_LPI2C */

int check_i2c_bus_valid(uint8_t bus)
{
	if (dev_i2c[bus] == NULL) {
		return -1;
	}
	return 0;
}

int i2c_master_read_without_error_log(I2C_MSG *msg, uint8_t retry)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -1);

	LOG_DBG("bus %d, addr %x, rxlen %d, txlen %d", msg->bus, msg->target_addr, msg->rx_len,
		msg->tx_len);
	LOG_HEXDUMP_DBG(msg->data, msg->tx_len, "txbuf");

	if (check_i2c_bus_valid(msg->bus) < 0) {
		return -1;
	}

	if (msg->rx_len == 0) {
		return EMSGSIZE;
	}

	if (msg->tx_len > I2C_BUFF_SIZE) {
		return -1;
	}

	int status;
	status = k_mutex_lock(&i2c_mutex[msg->bus], K_MSEC(1000));
	if (status) {
		return ENOLCK;
	}

	int ret = -1;
	uint8_t *txbuf = NULL, *rxbuf = NULL;
	txbuf = (uint8_t *)malloc(I2C_BUFF_SIZE * sizeof(uint8_t));
	if (!txbuf) {
		goto exit;
	}
	rxbuf = (uint8_t *)malloc(I2C_BUFF_SIZE * sizeof(uint8_t));
	if (!rxbuf) {
		goto exit;
	}
	memcpy(txbuf, &msg->data[0], msg->tx_len);

	uint8_t i;
	for (i = 0; i <= retry; i++) {
		if (msg->tx_len > 0) {
			ret = i2c_write_read(dev_i2c[msg->bus], msg->target_addr, txbuf,
					     msg->tx_len, rxbuf, msg->rx_len);
		} else {
			ret = i2c_read(dev_i2c[msg->bus], rxbuf, msg->rx_len, msg->target_addr);
		}
		if (ret == 0) { // i2c write read success
			memcpy(&msg->data[0], rxbuf, msg->rx_len);
			LOG_HEXDUMP_DBG(msg->data, msg->rx_len, "rxbuf");
			break;
		}
	}

exit:
	SAFE_FREE(txbuf);
	SAFE_FREE(rxbuf);

	status = k_mutex_unlock(&i2c_mutex[msg->bus]);

	return ret;
}