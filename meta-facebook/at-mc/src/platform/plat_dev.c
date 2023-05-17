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

#include <stdio.h>
#include <stdlib.h>
#include "sensor.h"
#include "pmbus.h"
#include "libutil.h"
#include "hal_i2c.h"
#include "ltc2991.h"
#include "emc1412.h"
#include "plat_dev.h"
#include "util_pmbus.h"
#include "plat_isr.h"
#include "plat_fru.h"
#include "ioexp_tca9555.h"
#include "common_i2c_mux.h"
#include "plat_sensor_table.h"
#include <logging/log.h>
#include "plat_def.h"
#include "pm8702.h"
#include "cci.h"
#include "plat_mctp.h"
#include "plat_hook.h"
#include "plat_class.h"
#include "hal_gpio.h"
#include "xdpe12284c.h"
#include "util_sys.h"

LOG_MODULE_REGISTER(plat_dev);

#define NVME_NOT_AVAILABLE 0x80
#define NVME_TEMP_SENSOR_FAILURE 0x81
#define NVME_DRIVE_NOT_READY_BIT BIT(6)

#define INA233_CALIBRATION_OFFSET 0xD4

#define EMC1412_DEFAULT_TEMP_MSB_REG 0x00
#define EMC1412_DEFAULT_TEMP_LSB_REG 0x29
#define EMC1412_DEFAULT_RESOLUTION 0.125
#define EMC1412_TEMP_SHIFT_BIT 5

#define XDPE12284_VID_IDENTIFIER 1

#define CXL_IOEXP_CONFIG_REG_DEFAULT_VAL 0xFF

#define CXL_IOEXP_U14_OUTPUT_0_REG_VAL 0xFF
#define CXL_IOEXP_U14_OUTPUT_1_REG_VAL 0xFF
#define CXL_IOEXP_U14_CONFIG_0_REG_VAL 0xFF
#define CXL_IOEXP_U14_CONFIG_1_REG_VAL 0xFF

#define CXL_IOEXP_U15_OUTPUT_0_REG_VAL 0x3B
#define CXL_IOEXP_U15_OUTPUT_1_REG_VAL 0xFF
#define CXL_IOEXP_U15_CONFIG_0_REG_VAL 0x21
#define CXL_IOEXP_U15_CONFIG_1_REG_VAL 0xFE

#define CXL_IOEXP_U16_OUTPUT_0_REG_VAL 0xFF
#define CXL_IOEXP_U16_OUTPUT_1_REG_VAL 0xFF
#define CXL_IOEXP_U16_CONFIG_0_REG_VAL 0x00
#define CXL_IOEXP_U16_CONFIG_1_REG_VAL 0x00

#define CXL_IOEXP_U17_OUTPUT_0_REG_VAL 0xFF
#define CXL_IOEXP_U17_OUTPUT_1_REG_VAL 0xFF
#define CXL_IOEXP_U17_CONFIG_0_REG_VAL 0xFF
#define CXL_IOEXP_U17_CONFIG_1_REG_VAL 0xFF

#define PM8702_DEFAULT_SENSOR_NUM SENSOR_NUM_TEMP_CXL

enum XDPE12284_VID {
	XDPE12284_VR12 = 1,
	XDPE12284_VR13,
	XDPE12284_IMVP9,
};

pm8702_dev_info pm8702_table[] = {
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
};

cxl_vr_fw_info cxl_vr_info_table[] = {
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
	{ .is_init = false }, { .is_init = false }, { .is_init = false }, { .is_init = false },
};

bool pal_sensor_drive_init(uint8_t card_id, sensor_cfg *cfg, uint8_t *init_status)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(init_status, false);

	switch (cfg->type) {
	case sensor_dev_tmp75:
		*init_status = pal_tmp75_init(card_id, cfg);
		break;
	case sensor_dev_emc1412:
		*init_status = pal_emc1412_init(card_id, cfg);
		break;
	case sensor_dev_nvme:
		*init_status = pal_nvme_init(card_id, cfg);
		break;
	case sensor_dev_ina233:
		*init_status = pal_ina233_init(card_id, cfg);
		break;
	case sensor_dev_ltc2991:
		*init_status = pal_ltc2991_init(card_id, cfg);
		break;
	case sensor_dev_xdpe12284c:
		*init_status = pal_xdpe12284c_init(card_id, cfg);
		break;
	case sensor_dev_pm8702:
		*init_status = pal_pm8702_init(card_id, cfg);
		break;
	default:
		LOG_ERR("Invalid initial drive type: 0x%x", cfg->type);
		return false;
	}

	return true;
}

bool pal_sensor_drive_read(uint8_t card_id, sensor_cfg *cfg, int *reading, uint8_t *sensor_status)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(reading, false);
	CHECK_NULL_ARG_WITH_RETURN(sensor_status, false);

	switch (cfg->type) {
	case sensor_dev_tmp75:
		*sensor_status = pal_tmp75_read(card_id, cfg, reading);
		break;
	case sensor_dev_emc1412:
		*sensor_status = pal_emc1412_read(card_id, cfg, reading);
		break;
	case sensor_dev_nvme:
		*sensor_status = pal_nvme_read(card_id, cfg, reading);
		break;
	case sensor_dev_ina233:
		*sensor_status = pal_ina233_read(card_id, cfg, reading);
		break;
	case sensor_dev_ltc2991:
		*sensor_status = pal_ltc2991_read(card_id, cfg, reading);
		break;
	case sensor_dev_xdpe12284c:
		*sensor_status = pal_xdpe12284c_read(card_id, cfg, reading);
		break;
	case sensor_dev_pm8702:
		*sensor_status = pal_pm8702_read(card_id, cfg, reading);
		break;
	default:
		LOG_ERR("Invalid reading drive type: 0x%x", cfg->type);
		return false;
	}

	return true;
}

/*  Reference: Infineon spec section 8.24: VID table
 *  PMBUS spec section 8.2: VOUT mode
 */
float pal_vid_to_float(int val, uint8_t vout_mode)
{
	uint8_t mode = 0;

	//VID 0 is always 0 V
	if (val == 0) {
		return 0;
	}

	mode = (vout_mode >> 5);

	if (mode != XDPE12284_VID_IDENTIFIER) {
		LOG_ERR("Infineon VR reading with invalid VID IDENTIFIER: %x", mode);
		return -1;
	}

	switch (vout_mode & 0x1f) {
	case XDPE12284_VR12:
		if (val > 0) {
			return ((val - 1) * 5 + 250);
		}
	case XDPE12284_VR13:
		if (val > 0) {
			return ((val - 1) * 10 + 500);
		}
	case XDPE12284_IMVP9:
		if (val > 0) {
			return ((val - 1) * 10 + 200);
		}
	default:
		LOG_ERR("Infineon VR reading with invalid vout mode: %x", vout_mode);
		return -1;
	}

	return 0;
}

uint8_t pal_tmp75_read(uint8_t card_id, sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor tmp75 0x%x input parameter is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	int ret = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("cxl %d tmp75 i2c read fail ret: %d", card_id, ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int8_t)msg.data[0];
	sval->fraction = 0;
	return SENSOR_READ_SUCCESS;
}

uint8_t pal_tmp75_init(uint8_t card_id, sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	return SENSOR_INIT_SUCCESS;
}

uint8_t pal_emc1412_read(uint8_t card_id, sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor emc1412 0x%x input parameter is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	int ret = -1;
	uint8_t retry = 5;
	uint16_t val = 0;
	I2C_MSG msg = { 0 };

	if (cfg->offset != EMC1412_READ_TEMP) {
		LOG_ERR("Invalid offset: 0x%x", cfg->offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	/* Read temperature msb register */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = EMC1412_DEFAULT_TEMP_MSB_REG;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("cxl %d emc1412 i2c read fail ret: %d", card_id, ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	val = (msg.data[0] << 8);

	/* Read temperature lsb register */
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = EMC1412_DEFAULT_TEMP_LSB_REG;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("cxl %d emc1412 i2c read fail ret: %d", card_id, ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	val = (val | msg.data[0]);

	/* Temperature data is high 11-bit value */
	val = val >> EMC1412_TEMP_SHIFT_BIT;
	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (uint16_t)(val * EMC1412_DEFAULT_RESOLUTION);
	sval->fraction = ((val * EMC1412_DEFAULT_RESOLUTION) - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t pal_emc1412_init(uint8_t card_id, sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	return SENSOR_INIT_SUCCESS;
}

uint8_t pal_nvme_read(uint8_t card_id, sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor nvme 0x%x input parameter is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	int ret = 0;
	uint8_t retry = 5;
	uint8_t nvme_status = 0;
	uint8_t composite_temp = 0;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.data[0] = cfg->offset;
	msg.tx_len = 1;
	msg.rx_len = 4;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("nvme i2c read fail ret: %d", ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	nvme_status = msg.data[1];
	composite_temp = msg.data[3];

	/* Check SSD drive ready */
	if ((nvme_status & NVME_DRIVE_NOT_READY_BIT) != 0) {
		return SENSOR_NOT_ACCESSIBLE;
	}

	/* Check reading value */
	switch (composite_temp) {
	case NVME_NOT_AVAILABLE:
		return SENSOR_FAIL_TO_ACCESS;
	case NVME_TEMP_SENSOR_FAILURE:
		return SENSOR_UNSPECIFIED_ERROR;
	default:
		break;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int8_t)composite_temp;
	sval->fraction = 0;

	return SENSOR_READ_SUCCESS;
}

uint8_t pal_nvme_init(uint8_t card_id, sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	return SENSOR_INIT_SUCCESS;
}

uint8_t pal_ina233_read(uint8_t card_id, sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor 0x%x input parameter is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	ina233_init_arg *init_arg = NULL;
	if (cfg->init_args == NULL) {
		init_arg = get_pcie_init_sensor_config(card_id, cfg->num);
		CHECK_NULL_ARG_WITH_RETURN(init_arg, SENSOR_UNSPECIFIED_ERROR);
	} else {
		init_arg = cfg->init_args;
	}

	if (init_arg->is_init != true) {
		LOG_ERR("device isn't initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	int ret = 0;
	int16_t val = 0;
	float parameter = 0;
	I2C_MSG msg;
	memset(&msg, 0, sizeof(I2C_MSG));
	*reading = 0;

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("cxl %d ina233 i2c read fail ret: %d", card_id, ret);
		return SENSOR_FAIL_TO_ACCESS;
	}
	uint8_t offset = cfg->offset;
	val = (msg.data[1] << 8) | msg.data[0];
	sensor_val *sval = (sensor_val *)reading;
	switch (offset) {
	case PMBUS_READ_VOUT:
		// 1 mV/LSB, unsigned integer
		// m = 8 , b = 0 , r = 2
		// voltage convert formula = ((val / 100) - 0) / 8
		parameter = 800;
		break;
	case PMBUS_READ_IOUT:
		// 1 mA/LSB, 2's complement
		// current convert formula = val / (1 / current_lsb)
		parameter = (1 / init_arg->current_lsb);
		break;
	case PMBUS_READ_POUT:
		// 1 Watt/LSB, 2's complement
		// power convert formula = val / (1 / (current_lsb * 25))
		parameter = (1 / (init_arg->current_lsb * 25));
		break;
	default:
		LOG_ERR("Offset not supported: 0x%x", offset);
		return SENSOR_FAIL_TO_ACCESS;
		break;
	}

	sval->integer = val / parameter;
	sval->fraction = ((val / parameter) - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t pal_ina233_init(uint8_t card_id, sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	ina233_init_arg *init_arg = NULL;
	if (cfg->init_args == NULL) {
		init_arg = get_pcie_init_sensor_config(card_id, cfg->num);
		CHECK_NULL_ARG_WITH_RETURN(init_arg, SENSOR_UNSPECIFIED_ERROR);
	} else {
		init_arg = cfg->init_args;
	}

	if (init_arg->is_init != true) {
		int ret = 0, retry = 5;
		uint16_t calibration = 0;
		I2C_MSG msg = { 0 };

		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 3;
		msg.data[0] = INA233_CALIBRATION_OFFSET;

		// Calibration formula = (0.00512 / (current_lsb * r_shunt))
		calibration =
			(uint16_t)((0.00512 / (init_arg->current_lsb * init_arg->r_shunt)) + 0.5);
		memcpy(&msg.data[1], &calibration, sizeof(uint16_t));

		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			LOG_ERR("cxl %d ina233 i2c write fail ret: %d", card_id, ret);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
		init_arg->is_init = true;
	}

	return SENSOR_INIT_SUCCESS;
}

uint8_t pal_ltc2991_read(uint8_t card_id, sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor ltc2991 0x%x input parameter is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	int ret = 0;
	uint8_t retry = 5;
	uint8_t channel = 0;
	uint8_t msb_register = 0;
	uint8_t lsb_register = 0;
	uint8_t read_optional = cfg->offset;
	uint16_t temp = 0;
	int16_t val = 0;
	float parameter = 0;
	I2C_MSG msg = { 0 };

	ret = ltc2991_read_optional_to_register(read_optional, &msb_register, &lsb_register,
						&parameter, &channel);
	if (ret != 0) {
		return SENSOR_PARAMETER_NOT_VALID;
	}

	/* Enable voltage channel */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = LTC2991_ENABLE_CHANNEL_REG;
	msg.data[1] = channel;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("cxl %d ltc2991 i2c write fail ret: %d", card_id, ret);
		return false;
	}

	k_msleep(LTC2991_DATA_NOT_READY_DELAY_MS);

	/* Read MSB register */
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = msb_register;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("cxl %d ltc2991 i2c read fail ret: %d", card_id, ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	if ((msg.data[0] & LTC2991_DATA_VALID_BIT) == 0) {
		LOG_ERR("MSB data invalid");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	/* Skip data valid bit */
	temp = ((msg.data[0] & 0x7F) << 8);

	/* Read LSB register */
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = lsb_register;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("cxl %d ltc2991 i2c read fail ret: %d", card_id, ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	temp = (temp | msg.data[0]);
	if ((temp & (LTC2991_DATA_SIGN_BIT << 8)) == 0) {
		val = temp;
	} else {
		/* Convert to two's component value */
		/* Skip data sign bit */
		temp = (temp & 0x3FFF);
		val = -(~(temp) + 1);
	}

	if (cfg->arg1 == 0) {
		LOG_ERR("sensor config setting error, arg1 is 0");
		return SENSOR_PARAMETER_NOT_VALID;
	}

	parameter = parameter * cfg->arg0 / cfg->arg1;

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = val * parameter;
	sval->fraction = ((val * parameter) - sval->integer) * 1000;
	return SENSOR_READ_SUCCESS;
}

uint8_t pal_ltc2991_init(uint8_t card_id, sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	ltc2991_init_arg *init_arg = NULL;
	if (cfg->init_args == NULL) {
		init_arg = get_pcie_init_sensor_config(card_id, cfg->num);
		CHECK_NULL_ARG_WITH_RETURN(init_arg, SENSOR_UNSPECIFIED_ERROR);
	} else {
		init_arg = cfg->init_args;
	}

	if (init_arg->is_init != true) {
		int ret = 0;
		uint8_t retry = 5;
		I2C_MSG msg = { 0 };

		/* Set V1~V4 control register */
		if (init_arg->v1_v4_control_operation.value != LTC2991_KEEP_DEFAULT_SETTING) {
			memset(&msg, 0, sizeof(I2C_MSG));
			msg.bus = cfg->port;
			msg.target_addr = cfg->target_addr;
			msg.tx_len = 2;
			msg.data[0] = LTC2991_V1_V4_CONTROL_REG;
			msg.data[1] = init_arg->v1_v4_control_operation.value;

			ret = i2c_master_write(&msg, retry);
			if (ret != 0) {
				LOG_ERR("cxl %d ltc2991 i2c write fail ret: %d", card_id, ret);
				return SENSOR_INIT_UNSPECIFIED_ERROR;
			}
		}

		/* Read V1~V4 control register */
		memset(&msg, 0, sizeof(I2C_MSG));
		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 1;
		msg.rx_len = 1;
		msg.data[0] = LTC2991_V1_V4_CONTROL_REG;

		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("cxl %d ltc2991 i2c read fail ret: %d", card_id, ret);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		init_arg->v1_v4_control_operation.value = msg.data[0];

		/* Set V5~V8 control register */
		if (init_arg->v5_v8_control_operation.value != LTC2991_KEEP_DEFAULT_SETTING) {
			memset(&msg, 0, sizeof(I2C_MSG));
			msg.bus = cfg->port;
			msg.target_addr = cfg->target_addr;
			msg.tx_len = 2;
			msg.data[0] = LTC2991_V5_V8_CONTROL_REG;
			msg.data[1] = init_arg->v5_v8_control_operation.value;

			ret = i2c_master_write(&msg, retry);
			if (ret != 0) {
				LOG_ERR("ltc2991 i2c write fail ret: %d", ret);
				return SENSOR_INIT_UNSPECIFIED_ERROR;
			}
		}

		/* Read V5~V8 control register */
		memset(&msg, 0, sizeof(I2C_MSG));
		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 1;
		msg.rx_len = 1;
		msg.data[0] = LTC2991_V5_V8_CONTROL_REG;

		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("cxl %d ltc2991 i2c read fail ret: %d", card_id, ret);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		init_arg->v5_v8_control_operation.value = msg.data[0];
		init_arg->is_init = true;
	}

	return SENSOR_INIT_SUCCESS;
}

uint8_t pal_xdpe12284c_read(uint8_t card_id, sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor xdpe12284c 0x%x input parameter is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	int val = 0;
	int ret = 0;
	I2C_MSG msg = { 0 };
	float actual_value = 0;
	uint8_t offset = cfg->offset;

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = offset;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("cxl %d xdpe12284c i2c read fail ret: %d", card_id, ret);
		return SENSOR_FAIL_TO_ACCESS;
	}

	val = (msg.data[1] << 8) | msg.data[0];
	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(sensor_val));

	switch (offset) {
	case PMBUS_READ_IOUT:
	case PMBUS_READ_POUT:
	case PMBUS_READ_TEMPERATURE_1:
		actual_value = slinear11_to_float(val);
		sval->integer = actual_value;
		sval->fraction = (actual_value - sval->integer) * 1000;
		break;
	case PMBUS_READ_VOUT:
		msg.tx_len = 1;
		msg.rx_len = 1;
		msg.data[0] = PMBUS_VOUT_MODE;

		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			return SENSOR_FAIL_TO_ACCESS;
		}

		actual_value = pal_vid_to_float(val, msg.data[0]);
		actual_value /= 1000; // mV to V
		sval->integer = actual_value;
		sval->fraction = (actual_value - sval->integer) * 1000;
		break;
	default:
		return SENSOR_FAIL_TO_ACCESS;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t pal_xdpe12284c_init(uint8_t card_id, sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	uint8_t cxl_id = 0;
	pcie_card_id_to_cxl_id(card_id, &cxl_id);
	switch (cfg->target_addr) {
	case CXL_VR_A0V8_ADDR:
		cxl_id = cxl_id * 3;
		break;
	case CXL_VR_D0V8_ADDR:
		cxl_id = cxl_id * 3 + 1;
		break;
	case CXL_VR_VDDQCD_ADDR:
		cxl_id = cxl_id * 3 + 2;
		break;
	default:
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	if (!cxl_vr_info_table[cxl_id].is_init) {
		if (!xdpe12284c_get_checksum(cfg->port, cfg->target_addr,
					     (cxl_vr_info_table[cxl_id].checksum))) {
			LOG_ERR("cxl %d %s get checksum failed", card_id,
				cfg->target_addr == CXL_VR_A0V8_ADDR   ? "VR_P0V89A" :
				cfg->target_addr == CXL_VR_D0V8_ADDR   ? "VR_P0V8D_PVDDQ_AB" :
				cfg->target_addr == CXL_VR_VDDQCD_ADDR ? "VR_PVDDQ_CD" :
									       "unknown vr");
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}

		if (!xdpe12284c_get_remaining_write(
			    cfg->port, cfg->target_addr,
			    (uint16_t *)&(cxl_vr_info_table[cxl_id].remaining_write))) {
			LOG_ERR("cxl %d %s get remaining write failed", card_id,
				cfg->target_addr == CXL_VR_A0V8_ADDR   ? "VR_P0V89A" :
				cfg->target_addr == CXL_VR_D0V8_ADDR   ? "VR_P0V8D_PVDDQ_AB" :
				cfg->target_addr == CXL_VR_VDDQCD_ADDR ? "VR_PVDDQ_CD" :
									       "unknown vr");
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
		cxl_vr_info_table[cxl_id].vendor = VENDOR_INFINEON;
		cxl_vr_info_table[cxl_id].is_init = true;
	}

	return SENSOR_INIT_SUCCESS;
}

void cxl_mb_status_init(uint8_t cxl_id)
{
	/** Initial mb reset pin status by checking the IO expander on CXL module **/
	int ret = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	mux_config meb_mux = { 0 };
	mux_config cxl_mux = { 0 };

	/** MEB mux for cxl channels **/
	meb_mux.bus = MEB_CXL_BUS;
	meb_mux.target_addr = CXL_FRU_MUX0_ADDR;
	meb_mux.channel = cxl_work_item[cxl_id].cxl_channel;

	/** CXL mux for sensor channels **/
	cxl_mux.bus = MEB_CXL_BUS;
	cxl_mux.target_addr = CXL_FRU_MUX1_ADDR;
	cxl_mux.channel = CXL_IOEXP_MUX_CHANNEL;

	/** Mutex lock bus **/
	struct k_mutex *meb_mutex = get_i2c_mux_mutex(meb_mux.bus);
	if (k_mutex_lock(meb_mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS))) {
		LOG_ERR("mutex locked failed bus%u", meb_mux.bus);
		return;
	}

	/** Enable mux channel **/
	if (set_mux_channel(meb_mux, MUTEX_LOCK_ENABLE) == false) {
		k_mutex_unlock(meb_mutex);
		return;
	}

	if (set_mux_channel(cxl_mux, MUTEX_LOCK_ENABLE) == false) {
		k_mutex_unlock(meb_mutex);
		return;
	}

	/** Read cxl U15 ioexp input port0 status **/
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = CXL_IOEXP_U15_ADDR;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_INPUT_PORT_REG_0;

	ret = i2c_master_read(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		k_mutex_unlock(meb_mutex);
		return;
	}

	if ((msg.data[0] & CXL_IOEXP_MB_RESET_BIT) != 0) {
		/** CXL mux for cxl channels **/
		cxl_mux.bus = MEB_CXL_BUS;
		cxl_mux.target_addr = CXL_FRU_MUX1_ADDR;
		cxl_mux.channel = CXL_CONTROLLER_MUX_CHANNEL;

		if (set_mux_channel(cxl_mux, MUTEX_LOCK_ENABLE) == false) {
			k_mutex_unlock(meb_mutex);
			return;
		}

		get_set_cxl_endpoint(cxl_id, MCTP_EID_CXL);
	}

	k_mutex_unlock(meb_mutex);
}

bool cxl_single_ioexp_alert_reset(uint8_t ioexp_name, bool is_mutex)
{
	int ret = 0;
	uint8_t retry = 5;
	uint8_t ioexp_addr = 0;
	I2C_MSG msg = { 0 };

	switch (ioexp_name) {
	case IOEXP_U14:
		ioexp_addr = CXL_IOEXP_U14_ADDR;
		break;
	case IOEXP_U15:
		ioexp_addr = CXL_IOEXP_U15_ADDR;
		break;
	case IOEXP_U16:
		ioexp_addr = CXL_IOEXP_U16_ADDR;
		break;
	case IOEXP_U17:
		ioexp_addr = CXL_IOEXP_U17_ADDR;
		break;
	default:
		LOG_ERR("CXL ioexp name: 0x%x is invalid", ioexp_name);
		return false;
	}

	/** Read cxl ioexp input port 0 status **/
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = ioexp_addr;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_INPUT_PORT_REG_0;

	if (is_mutex) {
		ret = i2c_master_read(&msg, retry);
	} else {
		ret = i2c_master_read_without_mutex(&msg, retry);
	}

	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return false;
	}

	/** Read cxl ioexp input port 1 status **/
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = MEB_CXL_BUS;
	msg.target_addr = ioexp_addr;
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = TCA9555_INPUT_PORT_REG_1;

	if (is_mutex) {
		ret = i2c_master_read(&msg, retry);
	} else {
		ret = i2c_master_read_without_mutex(&msg, retry);
	}

	if (ret != 0) {
		LOG_ERR("Unable to read ioexp bus: %u addr: 0x%02x", msg.bus, msg.target_addr);
		return false;
	}

	return true;
}

bool cxl_single_ioexp_config_init(uint8_t ioexp_name)
{
	int ret = 0;
	uint8_t retry = 5;
	uint8_t tx_len = 2;
	uint8_t ioexp_addr = 0;
	uint8_t output_0_value = 0;
	uint8_t output_1_value = 0;
	uint8_t config_0_value = 0;
	uint8_t config_1_value = 0;
	uint8_t data[tx_len];
	I2C_MSG msg = { 0 };
	memset(data, 0, sizeof(uint8_t) * tx_len);

	switch (ioexp_name) {
	case IOEXP_U14:
		ioexp_addr = CXL_IOEXP_U14_ADDR;
		output_0_value = CXL_IOEXP_U14_OUTPUT_0_REG_VAL;
		output_1_value = CXL_IOEXP_U14_OUTPUT_1_REG_VAL;
		config_0_value = CXL_IOEXP_U14_CONFIG_0_REG_VAL;
		config_1_value = CXL_IOEXP_U14_CONFIG_1_REG_VAL;
		break;
	case IOEXP_U15:
		ioexp_addr = CXL_IOEXP_U15_ADDR;
		output_0_value = CXL_IOEXP_U15_OUTPUT_0_REG_VAL;
		output_1_value = CXL_IOEXP_U15_OUTPUT_1_REG_VAL;
		config_0_value = CXL_IOEXP_U15_CONFIG_0_REG_VAL;
		config_1_value = CXL_IOEXP_U15_CONFIG_1_REG_VAL;
		break;
	case IOEXP_U16:
		ioexp_addr = CXL_IOEXP_U16_ADDR;
		output_0_value = CXL_IOEXP_U16_OUTPUT_0_REG_VAL;
		output_1_value = CXL_IOEXP_U16_OUTPUT_1_REG_VAL;
		config_0_value = CXL_IOEXP_U16_CONFIG_0_REG_VAL;
		config_1_value = CXL_IOEXP_U16_CONFIG_1_REG_VAL;
		break;
	case IOEXP_U17:
		ioexp_addr = CXL_IOEXP_U17_ADDR;
		output_0_value = CXL_IOEXP_U17_OUTPUT_0_REG_VAL;
		output_1_value = CXL_IOEXP_U17_OUTPUT_1_REG_VAL;
		config_0_value = CXL_IOEXP_U17_CONFIG_0_REG_VAL;
		config_1_value = CXL_IOEXP_U17_CONFIG_1_REG_VAL;
		break;
	default:
		LOG_ERR("CXL ioexp name: 0x%x is invalid", ioexp_name);
		return false;
	}

	/** Write cxl ioexp output 0 register **/
	data[0] = TCA9555_OUTPUT_PORT_REG_0;
	data[1] = output_0_value;
	msg = construct_i2c_message(MEB_CXL_BUS, ioexp_addr, tx_len, data, 0);

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to write ioexp output 0 register bus: %u addr: 0x%02x", msg.bus,
			msg.target_addr);
		return false;
	}

	/** Write cxl ioexp output 1 register **/
	data[0] = TCA9555_OUTPUT_PORT_REG_1;
	data[1] = output_1_value;
	msg = construct_i2c_message(MEB_CXL_BUS, ioexp_addr, tx_len, data, 0);

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to write ioexp output 1 register bus: %u addr: 0x%02x", msg.bus,
			msg.target_addr);
		return false;
	}

	/** Write cxl ioexp config 0 register **/
	data[0] = TCA9555_CONFIG_REG_0;
	data[1] = config_0_value;
	msg = construct_i2c_message(MEB_CXL_BUS, ioexp_addr, tx_len, data, 0);

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to write ioexp config 0 register bus: %u addr: 0x%02x", msg.bus,
			msg.target_addr);
		return false;
	}

	/** Write cxl ioexp config 1 register **/
	memset(&msg, 0, sizeof(I2C_MSG));
	data[0] = TCA9555_CONFIG_REG_1;
	data[1] = config_1_value;
	msg = construct_i2c_message(MEB_CXL_BUS, ioexp_addr, tx_len, data, 0);

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Unable to write ioexp config 1 register bus: %u addr: 0x%02x", msg.bus,
			msg.target_addr);
		return false;
	}

	return true;
}

int cxl_ioexp_init(uint8_t cxl_channel)
{
	bool ret = false;
	mux_config meb_mux = { 0 };
	mux_config cxl_mux = { 0 };

	/** MEB mux for cxl channels **/
	meb_mux.bus = MEB_CXL_BUS;
	meb_mux.target_addr = CXL_FRU_MUX0_ADDR;
	meb_mux.channel = cxl_channel;

	/** CXL mux for sensor channels **/
	cxl_mux.bus = MEB_CXL_BUS;
	cxl_mux.target_addr = CXL_FRU_MUX1_ADDR;
	cxl_mux.channel = CXL_IOEXP_MUX_CHANNEL;

	/** Mutex lock bus **/
	struct k_mutex *meb_mutex = get_i2c_mux_mutex(meb_mux.bus);
	if (k_mutex_lock(meb_mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS))) {
		LOG_ERR("mutex locked failed bus%u", meb_mux.bus);
		return -1;
	}

	/** Enable mux channel **/
	ret = set_mux_channel(meb_mux, MUTEX_LOCK_ENABLE);
	if (ret == false) {
		k_mutex_unlock(meb_mutex);
		return -1;
	}

	ret = set_mux_channel(cxl_mux, MUTEX_LOCK_ENABLE);
	if (ret == false) {
		k_mutex_unlock(meb_mutex);
		return -1;
	}

	/** ALL ioexp config register initial **/
	cxl_single_ioexp_config_init(IOEXP_U14);
	cxl_single_ioexp_config_init(IOEXP_U15);
	cxl_single_ioexp_config_init(IOEXP_U16);
	cxl_single_ioexp_config_init(IOEXP_U17);

	/** ALL ioexp initial **/
	cxl_single_ioexp_alert_reset(IOEXP_U14, MUTEX_LOCK_ENABLE);
	cxl_single_ioexp_alert_reset(IOEXP_U15, MUTEX_LOCK_ENABLE);
	cxl_single_ioexp_alert_reset(IOEXP_U16, MUTEX_LOCK_ENABLE);
	cxl_single_ioexp_alert_reset(IOEXP_U17, MUTEX_LOCK_ENABLE);

	/** mutex unlock bus **/
	k_mutex_unlock(meb_mutex);

	return 0;
}

uint8_t pal_pm8702_read(uint8_t card_id, sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_UNSPECIFIED_ERROR;
	}
	uint8_t port = cfg->port;
	uint8_t address = cfg->target_addr;
	uint8_t pm8702_access = cfg->offset;

	mctp *mctp_inst = NULL;
	mctp_ext_params ext_params = { 0 };
	sensor_val *sval = (sensor_val *)reading;
	if (get_mctp_info_by_eid(port, &mctp_inst, &ext_params) == false) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, SENSOR_UNSPECIFIED_ERROR);

	switch (pm8702_access) {
	case chip_temp:
		if (cci_get_chip_temp(mctp_inst, ext_params, &sval->integer) == false) {
			return SENSOR_NOT_ACCESSIBLE;
		}
		sval->fraction = 0;
		break;
	case dimm_temp:
		if (pm8702_get_dimm_temp(mctp_inst, ext_params, address, &sval->integer,
					 &sval->fraction) == false) {
			return SENSOR_NOT_ACCESSIBLE;
		}
		break;
	default:
		LOG_ERR("Invalid access offset %d", pm8702_access);
		return SENSOR_PARAMETER_NOT_VALID;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t pal_pm8702_init(uint8_t card_id, sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	return SENSOR_INIT_SUCCESS;
}

bool pal_init_pm8702_info(uint8_t cxl_id)
{
	bool ret = false;
	uint8_t pcie_card_id = 0;

	if (cxl_id_to_pcie_card_id(cxl_id, &pcie_card_id) != 0) {
		LOG_ERR("Invalid cxl id: 0x%x", cxl_id);
		return false;
	}

	uint8_t *req_buf = NULL;
	uint8_t req_len = GET_FW_INFO_REQ_PL_LEN;
	uint8_t resp_len = sizeof(cci_fw_info_resp);
	uint8_t resp_buf[resp_len];
	memset(resp_buf, 0, sizeof(uint8_t) * resp_len);

	ret = pal_pm8702_command_handler(pcie_card_id, CCI_GET_FW_INFO, req_buf, req_len, resp_buf,
					 &resp_len);
	if (ret != true) {
		LOG_ERR("Fail to get cxl card: 0x%x fw version", cxl_id);
		return false;
	}

	memcpy(&pm8702_table[cxl_id].dev_info, resp_buf, resp_len);
	pm8702_table[cxl_id].is_init = true;
	return true;
}

bool pal_get_pm8702_hbo_status(uint8_t pcie_card_id, uint8_t *resp_buf, uint8_t *resp_len)
{
	CHECK_NULL_ARG_WITH_RETURN(resp_buf, false);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, false);

	bool ret = false;
	uint8_t *req_buf = NULL;
	uint8_t req_len = HBO_STATUS_REQ_PL_LEN;

	ret = pal_pm8702_command_handler(pcie_card_id, PM8702_HBO_STATUS, req_buf, req_len,
					 resp_buf, resp_len);
	if (ret != true) {
		LOG_ERR("Fail to get card id: 0x%x HBO status", pcie_card_id);
	}

	return ret;
}

bool pal_pm8702_transfer_fw(uint8_t pcie_card_id, uint8_t *req_buf, int req_len)
{
	CHECK_NULL_ARG_WITH_RETURN(req_buf, false);

	bool ret = false;
	uint8_t resp_buf[TRANSFER_FW_RESP_PL_LEN];
	uint8_t resp_len = 0;

	ret = pal_pm8702_command_handler(pcie_card_id, PM8702_HBO_TRANSFER_FW, req_buf, req_len,
					 resp_buf, &resp_len);
	if (ret != true) {
		LOG_ERR("Fail to transfer card id: 0x%x firmware", pcie_card_id);
	}

	return ret;
}

bool pal_set_pm8702_active_slot(uint8_t pcie_card_id, uint8_t *req_buf, int req_len)
{
	CHECK_NULL_ARG_WITH_RETURN(req_buf, false);

	bool ret = false;
	uint8_t resp_buf[ACTIVATE_FW_RESP_PL_LEN];
	uint8_t resp_len = 0;

	ret = pal_pm8702_command_handler(pcie_card_id, PM8702_HBO_ACTIVATE_FW, req_buf, req_len,
					 resp_buf, &resp_len);
	if (ret != true) {
		LOG_ERR("Fail to activate card id: 0x%x slot firmware", pcie_card_id);
	}

	return ret;
}

bool pal_pm8702_command_handler(uint8_t pcie_card_id, uint16_t opcode, uint8_t *data_buf,
				int data_len, uint8_t *response, uint8_t *response_len)
{
	if (data_len != 0) {
		CHECK_NULL_ARG_WITH_RETURN(data_buf, false);
	}

	CHECK_NULL_ARG_WITH_RETURN(response, false);
	CHECK_NULL_ARG_WITH_RETURN(response_len, false);

	bool ret = false;
	uint8_t sensor_num = PM8702_DEFAULT_SENSOR_NUM;
	mctp *mctp_inst = NULL;
	mctp_ext_params ext_params = { 0 };

	if (is_cxl_access(pcie_card_id) != true) {
		LOG_ERR("Card id: 0x%x PM8702 can't access", pcie_card_id);
		return false;
	}

	if (get_mctp_info_by_eid(MCTP_EID_CXL, &mctp_inst, &ext_params) == false) {
		LOG_ERR("Fail to get mctp info via eid: 0x%x", MCTP_EID_CXL);
		return false;
	}

	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, false);

	ret = pre_cxl_switch_mux(sensor_num, pcie_card_id);
	if (ret != true) {
		LOG_ERR("Pre switch mux fail, sensor num: 0x%x, card id: 0x%x", sensor_num,
			pcie_card_id);
		return false;
	}

	ret = pm8702_cmd_handler(mctp_inst, ext_params, opcode, data_buf, data_len, response,
				 response_len);
	if (ret != true) {
		post_cxl_switch_mux(sensor_num, pcie_card_id);
		return false;
	}

	ret = post_cxl_switch_mux(sensor_num, pcie_card_id);
	if (ret != true) {
		LOG_ERR("Post switch mux fail, sensor num: 0x%x, card id: 0x%x", sensor_num,
			pcie_card_id);
	}

	return true;
}
