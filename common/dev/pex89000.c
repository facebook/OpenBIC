/* 
  PEX89000 Hardware I2C Slave UG_v1.0.pdf
  PEX89000_RM100.pdf
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include "libutil.h"

#include "sensor.h"
#include "hal_i2c.h"
#include "pex89000.h"

#define BRCM_I2C5_CMD_READ 0b100
#define BRCM_I2C5_CMD_WRITE 0b011

#define BRCM_CHIME_AXI_CSR_ADDR 0x001F0100
#define BRCM_CHIME_AXI_CSR_DATA 0x001F0104
#define BRCM_CHIME_AXI_CSR_CTL 0x001F0108

/* Control register of Chime to AXI by SMBus */
#define BRCM_REG_SMB_WR_CMD 0xFFE00004
#define BRCM_REG_SMB_WR_DATA 0xFFE00008
#define BRCM_REG_SMB_RD_CMD 0xFFE0000C
#define BRCM_REG_SMB_RD_DATA 0xFFE00010

#define BRCM_REG_TEMP_SNR0_CTL 0xFFE78504
#define BRCM_REG_TEMP_SNR0_STAT 0xFFE78538
#define BRCM_REG_CHIP_ID 0xFFF00000
#define BRCM_REG_CHIP_REVID 0xFFF00004
#define BRCM_REG_SBR_ID 0xFFF00008
#define BRCM_REG_FLASH_VER 0x100005f8

#define BRCM_VAL_TEMP_SNR0_CTL_RESET 0x000653E8

static sys_slist_t pex89000_list;

typedef struct {
	uint8_t cmd : 3;
	uint8_t reserve1 : 5;
	uint8_t oft21_14bit : 8;
	uint8_t oft11_10bit : 2;
	uint8_t be : 4;
	uint8_t oft13_12bit : 2;
	uint8_t oft9_2bit : 8;
} __packed HW_I2C_Cmd;

static uint8_t pex_dev_get(uint8_t bus, uint8_t addr, uint8_t idx, pex_dev_t *dev);
static void pex89000_i2c_encode(uint32_t oft, uint8_t be, uint8_t cmd, HW_I2C_Cmd *buf);
static uint8_t pex89000_chime_read(uint8_t bus, uint8_t addr, uint32_t oft, uint8_t *resp,
				   uint16_t resp_len);
static uint8_t pex89000_chime_write(uint8_t bus, uint8_t addr, uint32_t oft, uint8_t *data,
				    uint8_t data_len);
static uint8_t pend_for_read_valid(uint8_t bus, uint8_t addr);
static uint8_t pex89000_chime_to_axi_write(uint8_t bus, uint8_t addr, uint32_t oft, uint32_t data);
static uint8_t pex89000_chime_to_axi_read(uint8_t bus, uint8_t addr, uint32_t oft, uint32_t *resp);
static uint8_t pex89000_temp(uint8_t bus, uint8_t addr, pex_dev_t dev, uint32_t *val);
pex89000_unit *find_pex89000_from_idx(uint8_t idx);

static uint8_t pex_dev_get(uint8_t bus, uint8_t addr, uint8_t idx, pex_dev_t *dev)
{
	if (!dev) {
		printf("%s: *dev is NULL!\n", __func__);
		return pex_api_unspecific_err;
	}

	uint32_t resp;
	if (pex_access_engine(bus, addr, idx, pex_access_id, &resp)) {
		return pex_api_unspecific_err;
	};

	uint16_t dev_id = (resp >> 16) & 0xFFFF;
	if (dev_id == 0xC010 || dev_id == 0xC012)
		*dev = pex_dev_atlas1;
	else if (dev_id == 0xC030)
		*dev = pex_dev_atlas2;
	else
		*dev = pex_dev_unknown;

	return pex_api_success;
}

/*
 * be: byte enables
 * oft: Atlas register address
 * cmd: read or write command
 * buf: encoded byte array to send to pesw
 */
static void pex89000_i2c_encode(uint32_t oft, uint8_t be, uint8_t cmd, HW_I2C_Cmd *buf)
{
	if (!buf) {
		printf("%s: *buf is NULL!\n", __func__);
		return;
	}

	buf->reserve1 = 0;
	buf->cmd = cmd;
	buf->oft21_14bit = (oft >> 14) & 0xFF;
	buf->oft13_12bit = (oft >> 12) & 0x3;
	buf->be = be;
	buf->oft11_10bit = (oft >> 10) & 0x3;
	buf->oft9_2bit = (oft >> 2) & 0xFF;
}

static uint8_t pex89000_chime_read(uint8_t bus, uint8_t addr, uint32_t oft, uint8_t *resp,
				   uint16_t resp_len)
{
	if (!resp) {
		printf("%s: *resp is NULL!\n", __func__);
		return pex_api_unspecific_err;
	}

	HW_I2C_Cmd cmd;
	pex89000_i2c_encode(oft, 0xF, BRCM_I2C5_CMD_READ, &cmd);

	uint8_t retry = 5;
	I2C_MSG msg;

	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = sizeof(cmd);
	msg.rx_len = resp_len;
	memcpy(&msg.data[0], &cmd, sizeof(cmd));

	if (i2c_master_read(&msg, retry)) {
		printf("%s: pex89000 read failed!\n", __func__);
		return pex_api_unspecific_err;
	}

	memcpy(resp, &msg.data[0], resp_len);

	return pex_api_success;
}

static uint8_t pex89000_chime_write(uint8_t bus, uint8_t addr, uint32_t oft, uint8_t *data,
				    uint8_t data_len)
{
	if (!data) {
		printf("%s: *data is NULL!\n", __func__);
		return pex_api_unspecific_err;
	}

	HW_I2C_Cmd cmd;
	pex89000_i2c_encode(oft, 0xF, BRCM_I2C5_CMD_WRITE, &cmd);

	uint8_t retry = 5;
	I2C_MSG msg;

	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = sizeof(cmd) + data_len;
	memcpy(&msg.data[0], &cmd, sizeof(cmd));
	memcpy(&msg.data[4], data, data_len);

	if (i2c_master_write(&msg, retry)) {
		printf("%s: pex89000 write failed!\n", __func__);
		return pex_api_unspecific_err;
	}

	return pex_api_success;
}

static uint8_t pend_for_read_valid(uint8_t bus, uint8_t addr)
{
	uint8_t rty = 50;
	uint32_t resp = 0;

	for (int i = rty; i > 0; i--) {
		if (pex89000_chime_read(bus, addr, BRCM_CHIME_AXI_CSR_CTL, (uint8_t *)&resp,
					sizeof(resp))) {
			k_msleep(10);
			continue;
		}

		if (resp & BIT(27)) { // CHIME_to_AXI_CSR Control Status -> Read_data_vaild
			return pex_api_success;
		}

		k_msleep(10);
	}

	return pex_api_unspecific_err;
}

static uint8_t pex89000_chime_to_axi_write(uint8_t bus, uint8_t addr, uint32_t oft, uint32_t data)
{
	uint8_t rc = pex_api_unspecific_err;
	uint32_t wbuf;

	wbuf = sys_cpu_to_be32(oft);
	if (pex89000_chime_write(bus, addr, BRCM_CHIME_AXI_CSR_ADDR, (uint8_t *)&wbuf,
				 sizeof(wbuf))) {
		goto exit;
	}
	wbuf = sys_cpu_to_be32(data);
	if (pex89000_chime_write(bus, addr, BRCM_CHIME_AXI_CSR_DATA, (uint8_t *)&wbuf,
				 sizeof(wbuf))) {
		goto exit;
	}
	wbuf = sys_cpu_to_be32(0x1); // CHIME_to_AXI_CSR Control Status: write command
	if (pex89000_chime_write(bus, addr, BRCM_CHIME_AXI_CSR_CTL, (uint8_t *)&wbuf,
				 sizeof(wbuf))) {
		goto exit;
	}

	rc = pex_api_success;
exit:
	return rc;
}

static uint8_t pex89000_chime_to_axi_read(uint8_t bus, uint8_t addr, uint32_t oft, uint32_t *resp)
{
	uint8_t rc = pex_api_unspecific_err;

	if (!resp) {
		printf("%s: *resp is NULL!\n", __func__);
		return rc;
	}

	uint32_t data;
	data = sys_cpu_to_be32(oft);

	if (pex89000_chime_write(bus, addr, BRCM_CHIME_AXI_CSR_ADDR, (uint8_t *)&data,
				 sizeof(data))) {
		goto exit;
	}
	data = sys_cpu_to_be32(0x2); // CHIME_to_AXI_CSR Control Status: read command
	if (pex89000_chime_write(bus, addr, BRCM_CHIME_AXI_CSR_CTL, (uint8_t *)&data,
				 sizeof(data))) {
		goto exit;
	}

	k_msleep(10);
	if (pend_for_read_valid(bus, addr)) {
		printf("%s: read data invaild\n", __func__);
		goto exit;
	}

	if (pex89000_chime_read(bus, addr, BRCM_CHIME_AXI_CSR_DATA, (uint8_t *)resp,
				sizeof(resp))) {
		goto exit;
	}

	*resp = sys_cpu_to_be32(*resp);
	rc = pex_api_success;

exit:
	return rc;
}

uint8_t pex_access_engine(uint8_t bus, uint8_t addr, uint8_t idx, pex_access_t key, uint32_t *resp)
{
	if (!resp) {
		printf("%s: *resp is NULL!\n", __func__);
		return pex_api_unspecific_err;
	}

	pex89000_unit *p = find_pex89000_from_idx(idx);
	if (!p) {
		printk("%s: pex89000 node %d not found!\n", __func__, idx);
		return pex_api_unspecific_err;
	}

	int ret = k_mutex_lock(&p->mutex, K_MSEC(5000));
	if (ret) {
		printk("%s: pex89000 mutex %d lock failed status: %x\n", __func__, p->idx, ret);
		return pex_api_mutex_err;
	}

	uint8_t rc = pex_api_success;

	switch (key) {
	case pex_access_temp:
		if (pex89000_temp(bus, addr, p->pex_type, resp)) {
			printf("%s: TEMP access failed!\n", __func__);
			rc = pex_api_unspecific_err;
		}
		break;

	case pex_access_adc:
		printf("%s: ADC value get not support yet!\n", __func__);
		rc = pex_api_unspecific_err;
		break;

	case pex_access_id:
		if (pex89000_chime_to_axi_read(bus, addr, BRCM_REG_CHIP_ID, resp)) {
			printf("%s: ID access failed!\n", __func__);
			rc = pex_api_unspecific_err;
		}
		break;

	case pex_access_rev_id:
		if (pex89000_chime_to_axi_read(bus, addr, BRCM_REG_CHIP_REVID, resp)) {
			printf("%s: REVISION ID access failed!\n", __func__);
			rc = pex_api_unspecific_err;
		}
		break;

	case pex_access_sbr_ver:
		if (pex89000_chime_to_axi_read(bus, addr, BRCM_REG_SBR_ID, resp)) {
			printf("%s: SVR VERSION access failed!\n", __func__);
			rc = pex_api_unspecific_err;
		}
		break;

	case pex_access_flash_ver:
		if (pex89000_chime_to_axi_read(bus, addr, BRCM_REG_FLASH_VER, resp)) {
			printf("%s: FLASH VERSION access failed!\n", __func__);
			rc = pex_api_unspecific_err;
		}
		break;

	default:
		printf("%s: Invalid key %d\n", __func__, key);
		rc = pex_api_unspecific_err;
		break;
	}

	if (k_mutex_unlock(&p->mutex))
		printf("%s: pex89000 mutex %d unlock failed!\n", __func__, p->idx);

	return rc;
}

static uint8_t pex89000_temp(uint8_t bus, uint8_t addr, pex_dev_t dev, uint32_t *val)
{
	uint8_t rc = pex_api_unspecific_err;

	if (!val) {
		printf("%s: *val is NULL!\n", __func__);
		return rc;
	}

	float pre_highest_temp = 0;
	float temp = 0;
	float temp_arr[12];
	uint32_t CmdAddr;
	uint32_t resp = 0;

	if (dev == pex_dev_atlas1) {
		if (pex89000_chime_to_axi_read(bus, addr, BRCM_REG_TEMP_SNR0_CTL, &resp)) {
			printf("CHIME to AXI Read 0x%x fail!\n", BRCM_REG_TEMP_SNR0_CTL);
			goto exit;
		}
		if (resp != BRCM_VAL_TEMP_SNR0_CTL_RESET) {
			printf("ADC temperature control register1 check fail!\n");
			goto exit;
		}

		if (pex89000_chime_to_axi_write(bus, addr, BRCM_REG_TEMP_SNR0_CTL,
						BRCM_VAL_TEMP_SNR0_CTL_RESET)) {
			printf("CHIME to AXI Write 0x%x fail!\n", BRCM_REG_TEMP_SNR0_CTL);
			goto exit;
		}

		if (pex89000_chime_to_axi_read(bus, addr, BRCM_REG_TEMP_SNR0_STAT, &resp)) {
			printf("CHIME to AXI Write 0x%x fail!\n", BRCM_REG_TEMP_SNR0_STAT);
			goto exit;
		}

		temp = (resp & 0xFFFF) / 128;
	} else if (dev == pex_dev_atlas2) {
		for (int8_t i = 7; i < 12; i++) {
			CmdAddr = (0x21 << 16) | (0x4C << 8) | (0x0B);
			if (pex89000_chime_to_axi_write(bus, addr, BRCM_REG_SMB_WR_CMD, CmdAddr)) {
				printf("CHIME to AXI Write 0x%x fail!\n", BRCM_REG_SMB_WR_CMD);
				goto exit;
			}

			if (pex89000_chime_to_axi_write(bus, addr, BRCM_REG_SMB_WR_DATA,
							i | 0x10000)) {
				printf("CHIME to AXI Write 0x%x fail!\n", BRCM_REG_SMB_WR_DATA);
				goto exit;
			}

			CmdAddr = (0x22 << 16) | (0x4C << 8) | (0x14);
			if (pex89000_chime_to_axi_write(bus, addr, BRCM_REG_SMB_RD_CMD, CmdAddr)) {
				printf("CHIME to AXI Write 0x%x fail!\n", BRCM_REG_SMB_RD_CMD);
				goto exit;
			}

			if (pex89000_chime_to_axi_read(bus, addr, BRCM_REG_SMB_RD_DATA, &resp)) {
				printf("CHIME to AXI Write 0x%x fail!\n", BRCM_REG_SMB_RD_DATA);
				goto exit;
			}

			temp = (float)(366.812 - 0.23751 * (float)(resp & 0x7FF));
			temp_arr[i] = temp;

			if (temp > pre_highest_temp)
				pre_highest_temp = temp;
		}

		temp = pre_highest_temp;
	} else {
		printf("%s: device type %d not support!\n", __func__, dev);
		goto exit;
	}

	sensor_val *sval = (sensor_val *)val;
	sval->integer = (int16_t)temp;
	sval->fraction = (temp - sval->integer) * 1000;

	rc = pex_api_success;
exit:
	return rc;
}

pex89000_unit *find_pex89000_from_idx(uint8_t idx)
{
	sys_snode_t *node = NULL;
	SYS_SLIST_FOR_EACH_NODE (&pex89000_list, node) {
		pex89000_unit *p;
		p = CONTAINER_OF(node, pex89000_unit, node);
		if (p->idx == idx) {
			return p;
		}
	}

	return NULL;
}

uint8_t pex89000_read(uint8_t sensor_num, int *reading)
{
	uint8_t rc = SENSOR_UNSPECIFIED_ERROR;

	if (!reading) {
		printf("%s: *reading is NULL!\n", __func__);
		return rc;
	}

	pex89000_unit *p =
		(pex89000_unit *)sensor_config[sensor_config_index_map[sensor_num]].priv_data;

	switch (sensor_config[sensor_config_index_map[sensor_num]].offset) {
	case PEX_TEMP:
		if (pex_access_engine(sensor_config[sensor_config_index_map[sensor_num]].port,
				      sensor_config[sensor_config_index_map[sensor_num]].target_addr,
				      p->idx, pex_access_temp, reading)) {
			printf("%s: read temp fail!\n", __func__);
			rc = SENSOR_FAIL_TO_ACCESS;
			goto exit;
		}

		break;
	default:
		printf("%s: PEX89000 invalid sensor type!\n", __func__);
		goto exit;
	}

	rc = SENSOR_READ_SUCCESS;
exit:
	return rc;
}

uint8_t pex89000_init(uint8_t sensor_num)
{
	if (sensor_config[sensor_config_index_map[sensor_num]].init_args == NULL) {
		printf("%s: init_arg is NULL!\n", __func__);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	pex89000_init_arg *init_arg =
		(pex89000_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;

	pex89000_unit *p;
	p = find_pex89000_from_idx(init_arg->idx);
	if (p == NULL) {
		p = (pex89000_unit *)malloc(sizeof(pex89000_unit));
		if (!p) {
			printf("%s: pex89000_unit malloc failed!\n", __func__);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		p->idx = init_arg->idx;

		if (k_mutex_init(&p->mutex)) {
			printf("%s: pex89000 mutex %d init failed!\n", __func__, p->idx);
			SAFE_FREE(p);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		if (pex_dev_get(sensor_config[sensor_config_index_map[sensor_num]].port,
				sensor_config[sensor_config_index_map[sensor_num]].target_addr,
				p->idx, &p->pex_type)) {
			printf("%s: get pex type failed!\n", __func__);
			SAFE_FREE(p);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		sys_slist_append(&pex89000_list, &p->node);
	}

	sensor_config[sensor_config_index_map[sensor_num]].priv_data = p;
	sensor_config[sensor_config_index_map[sensor_num]].read = pex89000_read;

	return SENSOR_INIT_SUCCESS;
}
