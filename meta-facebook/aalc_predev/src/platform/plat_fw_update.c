#include <string.h>
#include <stdlib.h>
#include "plat_version.h"
#include "plat_modbus.h"
#include "modbus_server.h"
#include <logging/log.h>
#include "util_spi.h"
#include "libutil.h"
#include "sensor.h"
#include "nct7363.h"
#include "plat_sensor_table.h"
#include "plat_fw_update.h"

LOG_MODULE_REGISTER(plat_fw_update);

#define UPADTE_FW_DATA_LENGTH_MIN 3 // contain 2 regs(offeset)+ 1 reg(length) at least

uint8_t modbus_get_fw_reversion(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint16_t byte_val = FIRMWARE_REVISION_1 << 8 | FIRMWARE_REVISION_2;
	memcpy(cmd->data, &byte_val, sizeof(uint16_t) * cmd->cmd_size);
	return MODBUS_EXC_NONE;
}

uint8_t modbus_fw_download(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint32_t offset = cmd->data[0] << 16 | cmd->data[1]; // offset
	uint16_t msg_len = cmd->data[2] & 0x7FFF; // length
	uint8_t flag = (cmd->data[2] & BIT(15)) ? SECTOR_END_FLAG : 0;

	if (cmd->data_len <= UPADTE_FW_DATA_LENGTH_MIN)
		return MODBUS_EXC_ILLEGAL_DATA_VAL;

	if (msg_len != ((cmd->data_len - UPADTE_FW_DATA_LENGTH_MIN) * 2))
		return MODBUS_EXC_ILLEGAL_DATA_VAL;

	return fw_update(offset, msg_len, (uint8_t *)&cmd->data[UPADTE_FW_DATA_LENGTH_MIN], flag,
			 DEVSPI_FMC_CS0);
}

uint8_t all_fan_full_duty()
{
	const uint8_t fb_fan_pwm_sensors[14] = {
		SENSOR_NUM_FB_1_FAN_TACH_RPM,  SENSOR_NUM_FB_2_FAN_TACH_RPM,
		SENSOR_NUM_FB_3_FAN_TACH_RPM,  SENSOR_NUM_FB_4_FAN_TACH_RPM,
		SENSOR_NUM_FB_5_FAN_TACH_RPM,  SENSOR_NUM_FB_6_FAN_TACH_RPM,
		SENSOR_NUM_FB_7_FAN_TACH_RPM,  SENSOR_NUM_FB_8_FAN_TACH_RPM,
		SENSOR_NUM_FB_9_FAN_TACH_RPM,  SENSOR_NUM_FB_10_FAN_TACH_RPM,
		SENSOR_NUM_FB_11_FAN_TACH_RPM, SENSOR_NUM_FB_12_FAN_TACH_RPM,
		SENSOR_NUM_FB_13_FAN_TACH_RPM, SENSOR_NUM_FB_14_FAN_TACH_RPM,
	};

	const uint8_t pb_fan_pwm_sensors[3] = {
		SENSOR_NUM_PB_1_FAN_1_TACH_RPM,
		SENSOR_NUM_PB_2_FAN_1_TACH_RPM,
		SENSOR_NUM_PB_3_FAN_1_TACH_RPM,
	};

	for (int j = 0; j < ARRAY_SIZE(fb_fan_pwm_sensors); j++) {
		sensor_cfg *fan_cfg = get_common_sensor_cfg_info(fb_fan_pwm_sensors[j]);
		if (fan_cfg == NULL)
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;

        fan_cfg->port = NCT7363_17_PORT;//PWM write port
		if (nct7363_set_duty(fan_cfg, 100))
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}

	for (int j = 0; j < ARRAY_SIZE(pb_fan_pwm_sensors); j++) {
		sensor_cfg *fan_cfg = get_common_sensor_cfg_info(pb_fan_pwm_sensors[j]);
		if (fan_cfg == NULL)
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;

        fan_cfg->port = NCT7363_12_PORT;//PWM write port
		if (nct7363_set_duty(fan_cfg, 100))
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}    

	return MODBUS_EXC_NONE;
}