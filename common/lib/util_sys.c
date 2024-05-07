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

#include <zephyr.h>
#include <logging/log.h>
#include <sys/reboot.h>
#include <stdio.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "hal_gpio.h"
#include "util_sys.h"
#include "ipmi.h"
#include "libutil.h"
#include "sensor.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(util_sys);

#define SYS_RST_EVT_LOG_REG 0x7e6e2074

#define MAX_RETRY 10
#define CHECK_READY_FLAG_DELAY_MS 500

static uint8_t ME_mode = ME_INIT_MODE;

uint8_t ISL69254_DEVICE_ID[5] = { 0x04, 0x00, 0x67, 0xD2, 0x49 };
uint8_t XDPE12284C_DEVICE_ID[3] = { 0x02, 0x79, 0x02 };
uint8_t ISL69259_DEVICE_ID[5] = { 0x04, 0x00, 0x46, 0xD2, 0x49 };

static bool ac_lost = false;
/* Check AC lost */
void check_ac_lost()
{
	static bool is_read = false;
	if (!is_read) {
		if (sys_read32(SYS_RST_EVT_LOG_REG) & BIT(0)) {
			sys_write32(BIT(0), SYS_RST_EVT_LOG_REG);
			ac_lost = true;
		}
		is_read = true;
	} else {
		LOG_WRN("The system reset event log register is already read");
	}
	return;
}

bool is_ac_lost()
{
	return ac_lost;
}

/* bic warm reset work */
#define bic_warm_reset_delay 2000

void bic_warm_reset()
{
	pal_warm_reset_prepare();
	k_msleep(bic_warm_reset_delay);
	sys_reboot(SOC_RESET);
}

K_WORK_DEFINE(bic_warm_reset_work, bic_warm_reset);
void submit_bic_warm_reset()
{
	k_work_submit(&bic_warm_reset_work);
}

/* bic cold reset work */
#define bic_cold_reset_delay 100

void bic_cold_reset()
{
	pal_cold_reset_prepare();
	k_msleep(bic_cold_reset_delay);
	sys_reboot(SOC_RESET);
}

K_WORK_DEFINE(bic_cold_reset_work, bic_cold_reset);
void submit_bic_cold_reset()
{
	k_work_submit(&bic_cold_reset_work);
}

__weak void pal_warm_reset_prepare()
{
	return;
}

__weak void pal_cold_reset_prepare()
{
	return;
}

__weak int pal_submit_bmc_cold_reset()
{
	return -1;
}

__weak int pal_host_power_control(power_ctl_t ctl_type)
{
	return -1;
}

__weak bool pal_is_bmc_present()
{
	return false;
}

__weak bool pal_is_bmc_ready()
{
	return false;
}

__weak int pal_submit_12v_cycle_slot()
{
	return NOT_SUPPORT_12V_CYCLE_SLOT;
}

__weak int pal_clear_cmos()
{
	return -1;
}

__weak uint8_t get_system_class()
{
	return -1;
}

__weak int pal_get_set_add_debug_sel_mode_status(uint8_t options, uint8_t *status)
{
	return -1;
}

__weak uint8_t pal_get_bmc_interface()
{
	return BMC_INTERFACE_I2C;
}

/* The byte-2 of ME response for "Get Self-Test Result" command */
enum GET_SELF_TEST_RESULT_RESPONSE {
	NO_ERROR = 0x55,
	FIRMWARE_ENTERED_RECOVERY_MODE = 0x81,
};

enum RECOVERY_MODE_CAUSE {
	RECOVERY_MODE_CAUSE_RECOVERY_JUMPER = 0,
	RECOVERY_MODE_CAUSE_SECURITY_STRAP_OVERRIDE_JUMPER,
	RECOVERY_MODE_CAUSE_IPMI_COMMAND,
	RECOVERY_MODE_CAUSE_INVALID_FLASH_CONFIG,
	RECOVERY_MODE_CAUSE_INTERNAL_ERROR
};

int set_me_firmware_mode(uint8_t me_fw_mode)
{
	if ((me_fw_mode != ME_FW_RECOVERY) && (me_fw_mode != ME_FW_RESTORE)) {
		LOG_ERR("Unsupported ME firmware mode 0x%x setting", me_fw_mode);
		return -1;
	}

	int result = 0;
	ipmi_msg *me_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	uint8_t *data = (uint8_t *)malloc(5 * sizeof(uint8_t));
	if ((me_msg == NULL) || (data == NULL)) {
		LOG_ERR("Failed to allocate memory");
		result = -1;
		goto cleanup;
	}

	ipmb_error ret;
	uint16_t data_len = 0;
	uint8_t seq_source = 0xFF;
	int i = 0, retry = 3;

	for (i = 0; i < retry; i++) {
		seq_source = 0xFF;
		data_len = 4;
		data[0] = INTEL_IANA & 0xFF;
		data[1] = (INTEL_IANA >> 8) & 0xFF;
		data[2] = (INTEL_IANA >> 16) & 0xFF;
		data[3] = me_fw_mode;
		*me_msg = construct_ipmi_message(seq_source, NETFN_NM_REQ,
						 CMD_OEM_NM_FORCE_ME_RECOVERY, SELF, ME_IPMB,
						 data_len, data);
		ret = ipmb_read(me_msg, IPMB_inf_index_map[me_msg->InF_target]);
		if (ret != IPMB_ERROR_SUCCESS) {
			LOG_ERR("Failed to set ME firmware mode to 0x%x, ret: 0x%x", me_fw_mode,
				ret);
			continue;
		}

		switch (me_fw_mode) {
		case ME_FW_RECOVERY:
			k_msleep(2000);
			break;
		case ME_FW_RESTORE:
			k_msleep(8000);
			break;
		default:
			k_msleep(1000);
			break;
		}

		seq_source = 0xFF;
		data_len = 0;
		*me_msg = construct_ipmi_message(seq_source, NETFN_APP_REQ,
						 CMD_APP_GET_SELFTEST_RESULTS, SELF, ME_IPMB,
						 data_len, NULL);
		ret = ipmb_read(me_msg, IPMB_inf_index_map[me_msg->InF_target]);
		if (ret == IPMB_ERROR_SUCCESS) {
			switch (me_fw_mode) {
			case ME_FW_RECOVERY:
				if ((me_msg->data_len == 2) &&
				    (me_msg->data[0] == FIRMWARE_ENTERED_RECOVERY_MODE) &&
				    (me_msg->data[1] == RECOVERY_MODE_CAUSE_IPMI_COMMAND)) {
					ME_mode = ME_RECOVERY_MODE;
					result = 0;
					goto cleanup;
				}
				break;
			case ME_FW_RESTORE:
				if ((me_msg->data_len == 2) && (me_msg->data[0] == NO_ERROR) &&
				    (me_msg->data[1] == 0x00)) {
					ME_mode = ME_NORMAL_MODE;
					result = 0;
					goto cleanup;
				}
				break;
			default:
				break;
			}
		} else {
			LOG_ERR("Failed to get ME self test result after setting 0x%x mode, ret: 0x%x",
				me_fw_mode, ret);
		}
	}
	LOG_ERR("Failed to set ME firmware mode to 0x%x, retry time: %d", me_fw_mode, retry);
	result = -1;

cleanup:
	SAFE_FREE(data);
	SAFE_FREE(me_msg);
	return result;
}

void init_me_firmware()
{
	ipmi_msg me_msg;
	ipmb_error ret;
	int result = 0;
	uint8_t data_len = 0;
	uint8_t seq_source = 0xFF;

	me_msg = construct_ipmi_message(seq_source, NETFN_APP_REQ, CMD_APP_GET_SELFTEST_RESULTS,
					SELF, ME_IPMB, data_len, NULL);
	ret = ipmb_read(&me_msg, IPMB_inf_index_map[me_msg.InF_target]);
	if (ret == IPMB_ERROR_SUCCESS) {
		if ((me_msg.data_len == 2) && (me_msg.data[0] == FIRMWARE_ENTERED_RECOVERY_MODE) &&
		    (me_msg.data[1] == RECOVERY_MODE_CAUSE_IPMI_COMMAND)) {
			ME_mode = ME_RECOVERY_MODE;
			result = set_me_firmware_mode(ME_FW_RESTORE);

			if (result == 0) {
				ME_mode = ME_NORMAL_MODE;
			}
		} else if ((me_msg.data_len == 2) && (me_msg.data[0] == NO_ERROR) &&
			   (me_msg.data[1] == 0x00)) {
			ME_mode = ME_NORMAL_MODE;
		} else {
			ME_mode = ME_INIT_MODE;
		}
	} else {
		LOG_ERR("Failed to get ME self test result, ret: 0x%x", ret);
	}

	return;
}

uint8_t get_me_mode()
{
	return ME_mode;
}

void set_sys_ready_pin(uint8_t ready_gpio_name)
{
	bool is_bic_ready = false;
	int retry = 0;

	for (retry = 0; retry < MAX_RETRY; retry++) {
		k_msleep(CHECK_READY_FLAG_DELAY_MS);

		if (check_is_sensor_ready() == true) {
			is_bic_ready = true;
			break;
		}
	}

	if ((is_bic_ready == false) && (retry == MAX_RETRY)) {
		// TODO: add SEL to BMC to notify that BIC sensor polling doesn't ready
	}

	gpio_set(ready_gpio_name, GPIO_HIGH);
	LOG_INF("BIC Ready");
}
