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

#include <logging/log.h>

#include "sensor.h"
#include "hal_gpio.h"
#include "pldm.h"
#include "pmbus.h"
#include "plat_fru.h"
#include "plat_gpio.h"
#include "plat_pldm_monitor.h"
#include "plat_class.h"
#include "plat_led.h"
#include "plat_mctp.h"

LOG_MODULE_REGISTER(plat_pldm_monitor);

/* Chassis-specific entities 8192 - 16383 */
#define PLDM_OEM_ENTITY_LED 8192
#define PLDM_OEM_ENTITY_SSD_LED 8193
#define PLDM_OEM_ENTITY_NIC_TYPE 8194

struct pldm_state_effecter_info plat_state_effecter_table[] = {
	[0 ... PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX] = {
		.entity_type = PLDM_ENTITY_IO_CONTROLLER,
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 1] = { 
		.entity_type = PLDM_OEM_ENTITY_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE,
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 2] = { 
		.entity_type = PLDM_OEM_ENTITY_LED, 
		.effecter_id = (PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_FAULT_LED),
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 3] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_0,
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 4] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_1,
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 5] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_2,
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 6] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_3,
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 7] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_4,
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 8] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_5,
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 9] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_6,
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 10] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_7,
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 11] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_8, 
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 12] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_9, 
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 13] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_10,
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 14] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_11, 
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 15] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_12, 
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 16] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_13, 
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 17] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_14, 
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 18] = { 
		.entity_type = PLDM_OEM_ENTITY_SSD_LED, 
		.effecter_id = PLAT_EFFECTER_ID_LED_HIGH_BYTE | PLAT_EFFECTER_ID_LED_E1S_15, 
	},
	[PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_MAX + 19] = { 
		.entity_type = PLDM_OEM_ENTITY_NIC_TYPE, 
		.effecter_id = (PLAT_EFFECTER_ID_NIC_TYPE_HIGH_BYTE | PLAT_EFFECTER_ID_NIC_TYPE),
	},
};

uint8_t e1s_prsnt_pin[4][4] = {
	{ PRSNT_SSD0_R_N, PRSNT_SSD1_R_N, PRSNT_SSD2_R_N, PRSNT_SSD3_R_N },
	{ PRSNT_SSD4_R_N, PRSNT_SSD5_R_N, PRSNT_SSD6_R_N, PRSNT_SSD7_R_N },
	{ PRSNT_SSD8_R_N, PRSNT_SSD9_R_N, PRSNT_SSD10_R_N, PRSNT_SSD11_R_N },
	{ PRSNT_SSD12_R_N, PRSNT_SSD13_R_N, PRSNT_SSD14_R_N, PRSNT_SSD15_R_N },
};

uint8_t nic_prsnt_pin[] = {
	PRSNT_NIC0_R_N, PRSNT_NIC1_R_N, PRSNT_NIC2_R_N, PRSNT_NIC3_R_N,
	PRSNT_NIC4_R_N, PRSNT_NIC5_R_N, PRSNT_NIC6_R_N, PRSNT_NIC7_R_N,
};

uint8_t pex_sensor_num_table[] = {
	SENSOR_NUM_TEMP_PEX_0,
	SENSOR_NUM_TEMP_PEX_1,
	SENSOR_NUM_TEMP_PEX_2,
	SENSOR_NUM_TEMP_PEX_3,
};

uint8_t nic_power_ic_sensor_table[] = {
	SENSOR_NUM_VOLT_NIC_0, SENSOR_NUM_VOLT_NIC_1, SENSOR_NUM_VOLT_NIC_2, SENSOR_NUM_VOLT_NIC_3,
	SENSOR_NUM_VOLT_NIC_4, SENSOR_NUM_VOLT_NIC_5, SENSOR_NUM_VOLT_NIC_6, SENSOR_NUM_VOLT_NIC_7,
};

uint8_t pex_1v25_power_ic_sensor_table[] = {
	SENSOR_NUM_P1V25_VOLT_PEX_0,
	SENSOR_NUM_P1V25_VOLT_PEX_1,
	SENSOR_NUM_P1V25_VOLT_PEX_2,
	SENSOR_NUM_P1V25_VOLT_PEX_3,
};

uint8_t ssd_power_ic_sensor_table[] = {
	SENSOR_NUM_VOLT_E1S_0,	SENSOR_NUM_VOLT_E1S_1,	SENSOR_NUM_VOLT_E1S_2,
	SENSOR_NUM_VOLT_E1S_3,	SENSOR_NUM_VOLT_E1S_4,	SENSOR_NUM_VOLT_E1S_5,
	SENSOR_NUM_VOLT_E1S_6,	SENSOR_NUM_VOLT_E1S_7,	SENSOR_NUM_VOLT_E1S_8,
	SENSOR_NUM_VOLT_E1S_9,	SENSOR_NUM_VOLT_E1S_10, SENSOR_NUM_VOLT_E1S_11,
	SENSOR_NUM_VOLT_E1S_12, SENSOR_NUM_VOLT_E1S_13, SENSOR_NUM_VOLT_E1S_14,
	SENSOR_NUM_VOLT_E1S_15,
};

uint8_t e1s_sensor_table[SSD_MAX_NUMBER] = {
	SENSOR_NUM_TEMP_E1S_0,	SENSOR_NUM_TEMP_E1S_1,	SENSOR_NUM_TEMP_E1S_2,
	SENSOR_NUM_TEMP_E1S_3,	SENSOR_NUM_TEMP_E1S_4,	SENSOR_NUM_TEMP_E1S_5,
	SENSOR_NUM_TEMP_E1S_6,	SENSOR_NUM_TEMP_E1S_7,	SENSOR_NUM_TEMP_E1S_8,
	SENSOR_NUM_TEMP_E1S_9,	SENSOR_NUM_TEMP_E1S_10, SENSOR_NUM_TEMP_E1S_11,
	SENSOR_NUM_TEMP_E1S_12, SENSOR_NUM_TEMP_E1S_13, SENSOR_NUM_TEMP_E1S_14,
	SENSOR_NUM_TEMP_E1S_15,
};

uint8_t nic_temp_sensor_table[NIC_MAX_NUMBER] = {
	SENSOR_NUM_TEMP_NIC_0, SENSOR_NUM_TEMP_NIC_1, SENSOR_NUM_TEMP_NIC_2, SENSOR_NUM_TEMP_NIC_3,
	SENSOR_NUM_TEMP_NIC_4, SENSOR_NUM_TEMP_NIC_5, SENSOR_NUM_TEMP_NIC_6, SENSOR_NUM_TEMP_NIC_7,
};

uint8_t nic_optics_sensor_table[NIC_MAX_NUMBER] = {
	SENSOR_NUM_TEMP_NIC_OPTICS_0, SENSOR_NUM_TEMP_NIC_OPTICS_1, SENSOR_NUM_TEMP_NIC_OPTICS_2,
	SENSOR_NUM_TEMP_NIC_OPTICS_3, SENSOR_NUM_TEMP_NIC_OPTICS_4, SENSOR_NUM_TEMP_NIC_OPTICS_5,
	SENSOR_NUM_TEMP_NIC_OPTICS_6, SENSOR_NUM_TEMP_NIC_OPTICS_7,
};

static struct ssd_event_check {
	struct k_work_delayable work;
	bool init;
	uint8_t group;
	uint8_t flag;
	uint8_t alert_pin;
} ssd_event_check[2] = {
	[0] = { .alert_pin = SSD_0_7_ADC_ALERT_N },
	[1] = { .alert_pin = SSD_8_15_ADC_ALERT_N },
};

void ssd_alert_check_ina230(uint8_t idx, uint8_t *flag)
{
	CHECK_NULL_ARG(flag);

	uint8_t sensor_num = ssd_power_ic_sensor_table[idx];
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	if (!cfg) {
		LOG_ERR("The pointer of the sensor config is NULL");
		return;
	}

	if (cfg->pre_sensor_read_hook) {
		if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
			LOG_ERR("SSD%d with sensor number(0x%x) pre reading failed", idx, cfg->num);
			return;
		}
	}

	uint8_t retry = 5;
	I2C_MSG msg;
	/* If the alert latch enable bit is set to Latch mode, the ALERT pin and flag bits remain 
		active following a fault until the Mask/Enable register has been read. So if the alert is clear, 
		the non-alert log will be sent in the next worker execute */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = INA230_MSK_OFFSET;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Read E1S%d INA230 Mask/Enable register failed, address(0x%x) bus(0x%x)",
			idx, msg.target_addr, msg.bus);
		goto exec_post_exit;
	}

	uint16_t status = ((msg.data[0] << 8) | msg.data[1]);
	/* Define by INA230 datasheet, the BIT4 is the alert function flag on the Mask/Enable Register (0x06). */
	bool is_alert = ((status & BIT(4)) == 0) ? false : true;

	if (cfg->post_sensor_read_hook) {
		if (!cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL)) {
			LOG_ERR("SSD%d with sensor number(0x%x) post reading failed", idx,
				cfg->num);
		}
	}

	/* Check whether the bit is change */
	if ((*flag & BIT(idx % 8)) ^ (is_alert << (idx % 8))) {
		LOG_INF("Send SSD ADC alert event idx(%d) is_alert(%d)", idx, is_alert);

		struct pldm_sensor_event_state_sensor_state event;
		event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_STATUS;
		event.event_state = is_alert ? PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT :
					       PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL;
		event.previous_event_state = is_alert ? PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL :
							PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT;

		if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_SENSOR_E1S_0 + idx,
					     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
					     sizeof(struct pldm_sensor_event_state_sensor_state))) {
			LOG_ERR("Send SSD%d ADC alert event log failed", idx);
		}
	}

	/* Update the flag */
	WRITE_BIT(*flag, idx % 8, is_alert);
	return;

exec_post_exit:
	if (cfg->post_sensor_read_hook) {
		if (!cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL)) {
			LOG_ERR("SSD%d with sensor number(0x%x) post reading failed", idx,
				cfg->num);
		}
	}
	return;
}

void ssd_alert_check_isl28022(uint8_t idx, uint8_t *flag)
{
	CHECK_NULL_ARG(flag);

	uint8_t sensor_num = ssd_power_ic_sensor_table[idx];
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	if (!cfg) {
		LOG_ERR("The pointer of the sensor config is NULL");
		return;
	}

	if (cfg->pre_sensor_read_hook) {
		if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
			LOG_ERR("SSD%d with sensor number(0x%x) pre reading failed", idx, cfg->num);
			return;
		}
	}

	uint8_t retry = 5;
	I2C_MSG msg;
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = 0x08;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Read E1S%d ISL28022 status register failed, address(0x%x) bus(0x%x)", idx,
			msg.target_addr, msg.bus);
		goto exec_post_exit;
	}

	uint16_t status = ((msg.data[0] << 8) | msg.data[1]);
	/* Define by ISL28022 datasheet, only BIT0-BIT3 meaningful on the interrupt status register (0x08). */
	bool is_alert = ((status & (BIT(0) | BIT(1) | BIT(2) | BIT(3))) == 0) ? false : true;

	if (is_alert) {
		/* Clear the alert bit */
		msg.tx_len = 3;
		msg.data[0] = 0x08;
		msg.data[1] = status & 0xFF;
		msg.data[2] = (status >> 8) & 0xFF;

		if (i2c_master_write(&msg, retry)) {
			LOG_ERR("Clear E1S%d ISL28022 status register failed, address(0x%x) bus(0x%x)",
				idx, msg.target_addr, msg.bus);
			goto exec_post_exit;
		}
	}

	if (cfg->post_sensor_read_hook) {
		if (!cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL)) {
			LOG_ERR("SSD%d with sensor number(0x%x) post reading failed", idx,
				cfg->num);
		}
	}

	/* Check whether the bit is change */
	if ((*flag & BIT(idx % 8)) ^ (is_alert << (idx % 8))) {
		LOG_INF("Send SSD ADC alert event idx(%d) is_alert(%d)", idx, is_alert);

		struct pldm_sensor_event_state_sensor_state event;
		event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_STATUS;
		event.event_state = is_alert ? PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT :
					       PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL;
		event.previous_event_state = is_alert ? PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL :
							PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT;

		if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_SENSOR_E1S_0 + idx,
					     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
					     sizeof(struct pldm_sensor_event_state_sensor_state))) {
			LOG_ERR("Send SSD%d ADC alert event log failed", idx);
		}
	}

	/* Update the flag */
	WRITE_BIT(*flag, idx % 8, is_alert);
	return;

exec_post_exit:
	if (cfg->post_sensor_read_hook) {
		if (!cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL)) {
			LOG_ERR("SSD%d with sensor number(0x%x) post reading failed", idx,
				cfg->num);
		}
	}
	return;
}

void ssd_alert_check_handler(struct k_work *work)
{
	CHECK_NULL_ARG(work);

	struct ssd_event_check *data = CONTAINER_OF(work, struct ssd_event_check, work);

	if (!data) {
		LOG_ERR("The pointer of the SSD event check is NULL");
		return;
	}

	gt_power_monitor_ic_type_t type = get_power_moniter_ic_type();
	uint8_t start_idx = (data->group == 0 ? 0 : 8);
	uint8_t end_idx = (data->group == 0 ? 7 : 15);

	for (uint8_t i = start_idx; i <= end_idx; i++) {
		/* Pass device not present case */
		if (gpio_get(e1s_prsnt_pin[i / 4][i % 4]))
			continue;

		if (type == POWER_IC_ISL28022) {
			ssd_alert_check_isl28022(i, &data->flag);
		} else if (type == POWER_IC_INA230) {
			ssd_alert_check_ina230(i, &data->flag);
		} else {
			LOG_ERR("Can't send SSD%d event with unknown power IC(%d)", i, type);
			return;
		}
	}

	if (data->flag) {
		gpio_interrupt_conf(data->alert_pin, GPIO_INT_DISABLE);
		k_work_schedule((struct k_work_delayable *)work, K_SECONDS(5));
	} else {
		gpio_interrupt_conf(data->alert_pin, GPIO_INT_EDGE_BOTH);
	}

	light_fault_led_check();
}

void ssd_alert_check(uint8_t group)
{
	if (group > 1) {
		LOG_ERR("The argument value is invalid");
		return;
	}

	struct ssd_event_check *data = ((group == 0) ? &ssd_event_check[0] : &ssd_event_check[1]);

	if (!data) {
		LOG_ERR("The pointer of the SSD event check is NULL");
		return;
	}

	if (!data->init) {
		k_work_init_delayable(&data->work, ssd_alert_check_handler);
		data->init = true;
	}

	data->group = group;

	/* Avoid redundant workers sending */
	if (k_work_delayable_busy_get(&data->work))
		return;

	k_work_schedule(&data->work, K_SECONDS(1));
}

void ssd_present_check()
{
	for (uint8_t i = 0; i < SSD_MAX_NUMBER; i++) {
		bool is_present = !gpio_get(e1s_prsnt_pin[i / 4][i % 4]);
		struct pldm_sensor_event_state_sensor_state event;

		event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_PRESENCE;
		event.event_state =
			is_present ? PLDM_STATE_SET_PRESENT : PLDM_STATE_SET_NOT_PRESENT;
		event.previous_event_state = PLDM_STATE_SET_NOT_PRESENT;

		if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_SENSOR_E1S_0 + i,
					     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
					     sizeof(struct pldm_sensor_event_state_sensor_state))) {
			LOG_ERR("Send SSD%d presence event log failed", i);
		}
	}
}

void nic_present_check()
{
	for (uint8_t i = 0; i < NIC_MAX_NUMBER; i++) {
		bool is_present = !gpio_get(nic_prsnt_pin[i]);
		struct pldm_sensor_event_state_sensor_state event;

		event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_PRESENCE;
		event.event_state =
			is_present ? PLDM_STATE_SET_PRESENT : PLDM_STATE_SET_NOT_PRESENT;
		event.previous_event_state = PLDM_STATE_SET_NOT_PRESENT;

		if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_SENSOR_NIC_0 + i,
					     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
					     sizeof(struct pldm_sensor_event_state_sensor_state))) {
			LOG_ERR("Send NIC%d presence event log failed", i);
		}
	}
}

void vr_status_mfr_specific_check_handler(struct k_work *work)
{
	uint8_t vr_sensor_num[2] = { SENSOR_NUM_PEX_0_VR_TEMP, SENSOR_NUM_PEX_2_VR_TEMP };

	for (uint8_t i = 0; i < ARRAY_SIZE(vr_sensor_num); i++) {
		sensor_cfg *cfg = &sensor_config[sensor_config_index_map[vr_sensor_num[i]]];
		I2C_MSG i2c_msg = { 0 };

		if (!cfg) {
			LOG_ERR("The pointer to VR%d config is NULL", i);
			continue;
		}

		if (cfg->pre_sensor_read_hook) {
			if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
				LOG_ERR("Pre-sensor reading hook function failed, VR%d status mfr specific check failed",
					i);
				continue;
			}
		}

		/**
		 * Get PMBUS_STATUS_MFR_SPECIFIC
		*/
		i2c_msg.bus = cfg->port;
		i2c_msg.target_addr = cfg->target_addr;
		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 1;
		i2c_msg.data[0] = PMBUS_STATUS_MFR_SPECIFIC;

		if (i2c_master_read(&i2c_msg, 5)) {
			LOG_ERR("I2C master read failed, VR%d status mfr specific check failed", i);
			if (cfg->post_sensor_read_hook) {
				if (!cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args,
								NULL)) {
					LOG_ERR("Post-sensor reading hook function failed");
				}
			}
			continue;
		}

		uint8_t status_mfr;
		status_mfr = i2c_msg.data[0];
		LOG_DBG("status_mfr = 0x%x", status_mfr);

		/**
		 * Clear faults when bit ADCUNLOCK or BBEVENT is high
		*/
		if ((status_mfr & BIT(3)) || (status_mfr & BIT(7))) {
			LOG_INF("VR%d register PMBUS_STATUS_MFR_SPECIFIC(80h) is 0x%x, bit ADCUNLOCK or BBEVENT is high, clear faults",
				i, status_mfr);

			gpio_interrupt_conf(SMB_ALERT_PMBUS_R_N, GPIO_INT_DISABLE);

			i2c_msg.tx_len = 1;
			i2c_msg.rx_len = 0;
			i2c_msg.data[0] = PMBUS_CLEAR_FAULTS;

			if (i2c_master_write(&i2c_msg, 5)) {
				LOG_ERR("VR%d Failed to clear fault", i);
			}

			//Wait VR to clear fault, to prevent unnecessary interrupt
			k_msleep(5);
			gpio_interrupt_conf(SMB_ALERT_PMBUS_R_N, GPIO_INT_EDGE_BOTH);
		}

		if (cfg->post_sensor_read_hook) {
			if (!cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL)) {
				LOG_ERR("Post-sensor reading hook function failed");
			}
		}
	}

	bool is_alert = !gpio_get(SMB_ALERT_PMBUS_R_N);

	if (is_alert) {
		struct pldm_sensor_event_state_sensor_state event;

		event.sensor_offset = PLDM_STATE_SET_OFFSET_DEVICE_STATUS;
		event.event_state = PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT;
		event.previous_event_state = PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL;

		LOG_WRN("VR PMBUS is alert");

		if (pldm_send_platform_event(PLDM_SENSOR_EVENT, PLDM_EVENT_SENSOR_VR,
					     PLDM_STATE_SENSOR_STATE, (uint8_t *)&event,
					     sizeof(struct pldm_sensor_event_state_sensor_state))) {
			LOG_ERR("Send VR PMBUS alert event log failed");
		}
		light_fault_led_check();
		vr_alert_check();
	}
}

void vr_alert_check()
{
	const static uint8_t vr_sensor_num[2] = { SENSOR_NUM_PEX_0_VR_TEMP,
						  SENSOR_NUM_PEX_2_VR_TEMP };

	for (uint8_t i = 0; i < ARRAY_SIZE(vr_sensor_num); i++) {
		sensor_cfg *cfg = &sensor_config[sensor_config_index_map[vr_sensor_num[i]]];
		I2C_MSG i2c_msg = { 0 };

		if (!cfg) {
			LOG_ERR("The pointer to VR%d config is NULL", i);
			continue;
		}

		if (cfg->pre_sensor_read_hook) {
			if (!cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args)) {
				LOG_ERR("Pre-sensor reading hook function failed, VR%d alert check failed",
					i);
				continue;
			}
		}

		i2c_msg.bus = cfg->port;
		i2c_msg.target_addr = cfg->target_addr;
		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 2;
		i2c_msg.data[0] = PMBUS_STATUS_WORD;

		if (i2c_master_read(&i2c_msg, 5)) {
			LOG_ERR("I2C master read failed");
			if (cfg->post_sensor_read_hook) {
				if (!cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args,
								NULL)) {
					LOG_ERR("Post-sensor reading hook function failed");
				}
			}
			continue;
		}

		struct pldm_sensor_event_numeric_sensor_state event;
		uint8_t vr_sensor_id = (vr_sensor_num[i] == SENSOR_NUM_PEX_0_VR_TEMP) ?
					       PLDM_EVENT_SENSOR_VR_0 :
					       PLDM_EVENT_SENSOR_VR_1;
		uint16_t status_word;

		event.event_state = PLDM_SENSOR_WARNING;
		event.previous_event_state = PLDM_SENSOR_NORMAL;
		event.sensor_data_size = PLDM_SENSOR_DATA_SIZE_UINT16;
		status_word = i2c_msg.data[0] << 8 | i2c_msg.data[1];

		memcpy(event.present_reading, &status_word, sizeof(status_word));

		if (pldm_send_platform_event(PLDM_SENSOR_EVENT, vr_sensor_id,
					     PLDM_NUMERIC_SENSOR_STATE, (uint8_t *)&event,
					     sizeof(struct pldm_sensor_event_numeric_sensor_state) +
						     1)) {
			LOG_ERR("Send event log for VR alert status word failed");
		}

		if (cfg->post_sensor_read_hook) {
			if (!cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL)) {
				LOG_ERR("Post-sensor reading hook function failed");
			}
		}
	}
}

static void plat_set_effecter_led_handler(const uint8_t *buf, uint16_t len, uint8_t *resp,
					  uint16_t *resp_len)
{
	CHECK_NULL_ARG(buf);
	CHECK_NULL_ARG(resp);
	CHECK_NULL_ARG(resp_len);

	struct pldm_set_state_effecter_states_req *req_p =
		(struct pldm_set_state_effecter_states_req *)buf;
	uint8_t effector_id = req_p->effecter_id & BIT_MASK(8);
	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	if ((effector_id != PLAT_EFFECTER_ID_POWER_LED) &&
	    (effector_id != PLAT_EFFECTER_ID_FAULT_LED)) {
		LOG_ERR("Unsupport LED effecter ID(%d)", effector_id);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return;
	}

	if (req_p->composite_effecter_count != PLDM_PLATFORM_OEM_LED_EFFECTER_STATE_FIELD_COUNT) {
		LOG_ERR("Unsupport LED effecter count(%d)", req_p->composite_effecter_count);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return;
	}

	set_effecter_state_field_t *led_val_state = &req_p->field[0];

	if (led_val_state->set_request >= PLDM_SET_REQUEST_MAX) {
		LOG_ERR("Unsupport LED effecter set request(%d)", led_val_state->set_request);
		*completion_code_p = PLDM_PLATFORM_UNSUPPORTED_EFFECTERSTATE;
		return;
	}

	if ((led_val_state->effecter_state == EFFECTER_STATE_LED_VALUE_UNKNOWN) ||
	    (led_val_state->effecter_state >= EFFECTER_STATE_LED_VALUE_MAX)) {
		LOG_ERR("Unsupport LED effecter state(%d)", led_val_state->effecter_state);
		*completion_code_p = PLDM_PLATFORM_INVALID_STATE_VALUE;
		return;
	}

	if (led_val_state->set_request == PLDM_REQUEST_SET) {
		uint8_t val = ((led_val_state->effecter_state == EFFECTER_STATE_LED_VALUE_ON) ?
				       LED_CTRL_ON :
				       LED_CTRL_OFF);
		bool (*ctrl_func)(uint8_t, uint8_t) =
			((effector_id == PLAT_EFFECTER_ID_POWER_LED) ? &pwr_led_control :
								       &fault_led_control);

		if (ctrl_func && ctrl_func(LED_CTRL_SRC_BMC, val))
			*completion_code_p = PLDM_SUCCESS;
		else
			*completion_code_p = PLDM_ERROR;
	} else {
		*completion_code_p = PLDM_SUCCESS;
	}

	return;
}

static void plat_set_effecter_ssd_led_handler(const uint8_t *buf, uint16_t len, uint8_t *resp,
					      uint16_t *resp_len)
{
	CHECK_NULL_ARG(buf);
	CHECK_NULL_ARG(resp);
	CHECK_NULL_ARG(resp_len);

	struct pldm_set_state_effecter_states_req *req_p =
		(struct pldm_set_state_effecter_states_req *)buf;
	uint8_t effector_id = req_p->effecter_id & BIT_MASK(8);
	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	if ((effector_id < PLAT_EFFECTER_ID_LED_E1S_0) ||
	    (effector_id > PLAT_EFFECTER_ID_LED_E1S_15)) {
		LOG_ERR("Unsupport SSD LED effecter ID(%d)", effector_id);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return;
	}

	if (req_p->composite_effecter_count != PLDM_PLATFORM_OEM_LED_EFFECTER_STATE_FIELD_COUNT) {
		LOG_ERR("Unsupport LED effecter count(%d)", req_p->composite_effecter_count);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		return;
	}

	set_effecter_state_field_t *led_val_state = &req_p->field[0];

	if (led_val_state->set_request >= PLDM_SET_REQUEST_MAX) {
		LOG_ERR("Unsupport LED effecter set request(%d)", led_val_state->set_request);
		*completion_code_p = PLDM_PLATFORM_UNSUPPORTED_EFFECTERSTATE;
		return;
	}

	if ((led_val_state->effecter_state == EFFECTER_STATE_LED_VALUE_UNKNOWN) ||
	    (led_val_state->effecter_state >= EFFECTER_STATE_LED_VALUE_MAX)) {
		LOG_ERR("Unsupport LED effecter state(%d)", led_val_state->effecter_state);
		*completion_code_p = PLDM_PLATFORM_INVALID_STATE_VALUE;
		return;
	}

	if (!is_e1s_access(e1s_sensor_table[effector_id - PLAT_EFFECTER_ID_LED_E1S_0])) {
		*completion_code_p = PLDM_ERROR_NOT_READY;
		return;
	}

	if (led_val_state->set_request == PLDM_REQUEST_SET) {
		uint8_t val = ((led_val_state->effecter_state == EFFECTER_STATE_LED_VALUE_ON) ?
				       LED_CTRL_ON :
				       LED_CTRL_OFF);
		if (e1s_led_control(effector_id, val))
			*completion_code_p = PLDM_SUCCESS;
		else
			*completion_code_p = PLDM_ERROR;
	} else {
		*completion_code_p = PLDM_SUCCESS;
	}

	return;
}

static void plat_get_effecter_led_handler(const uint8_t *buf, uint16_t len, uint8_t *resp,
					  uint16_t *resp_len)
{
	CHECK_NULL_ARG(buf);
	CHECK_NULL_ARG(resp);
	CHECK_NULL_ARG(resp_len);

	struct pldm_get_state_effecter_states_req *req_p =
		(struct pldm_get_state_effecter_states_req *)buf;
	struct pldm_get_state_effecter_states_resp *res_p =
		(struct pldm_get_state_effecter_states_resp *)resp;

	uint8_t effector_id = req_p->effecter_id & BIT_MASK(8);
	get_effecter_state_field_t *state = &res_p->field[0];
	*resp_len = 1;

	if ((effector_id != PLAT_EFFECTER_ID_POWER_LED) &&
	    (effector_id != PLAT_EFFECTER_ID_FAULT_LED)) {
		LOG_ERR("Unsupport LED effecter ID(%d)", effector_id);
		res_p->completion_code = PLDM_ERROR_INVALID_DATA;
		return;
	}

	uint8_t led_type =
		(effector_id == PLAT_EFFECTER_ID_POWER_LED) ? SYS_POWER_LED : SYS_FAULT_LED;
	uint8_t status = get_sys_led_status(led_type);

	if (status < LED_CTRL_MAX) {
		state->effecter_op_state = PLDM_EFFECTER_ENABLED_NOUPDATEPENDING;
		state->present_state = state->pending_state =
			((status == LED_CTRL_ON) ? EFFECTER_STATE_LED_VALUE_ON :
						   EFFECTER_STATE_LED_VALUE_OFF);
	} else {
		state->effecter_op_state = PLDM_EFFECTER_STATUSUNKNOWN;
		state->present_state = state->pending_state = EFFECTER_STATE_LED_VALUE_UNKNOWN;
	}

	*resp_len = PLDM_GET_STATE_EFFECTER_RESP_NO_STATE_FIELD_BYTES +
		    (sizeof(get_effecter_state_field_t) *
		     PLDM_PLATFORM_OEM_LED_EFFECTER_STATE_FIELD_COUNT);
	res_p->composite_effecter_count = PLDM_PLATFORM_OEM_LED_EFFECTER_STATE_FIELD_COUNT;
	res_p->completion_code = PLDM_SUCCESS;
}

static void plat_get_effecter_ssd_led_handler(const uint8_t *buf, uint16_t len, uint8_t *resp,
					      uint16_t *resp_len)
{
	CHECK_NULL_ARG(buf);
	CHECK_NULL_ARG(resp);
	CHECK_NULL_ARG(resp_len);

	struct pldm_get_state_effecter_states_req *req_p =
		(struct pldm_get_state_effecter_states_req *)buf;
	struct pldm_get_state_effecter_states_resp *res_p =
		(struct pldm_get_state_effecter_states_resp *)resp;

	uint8_t effector_id = req_p->effecter_id & BIT_MASK(8);
	*resp_len = 1;

	if ((effector_id < PLAT_EFFECTER_ID_LED_E1S_0) ||
	    (effector_id > PLAT_EFFECTER_ID_LED_E1S_15)) {
		LOG_ERR("Unsupport SSD LED effecter ID(%d)", effector_id);
		res_p->completion_code = PLDM_ERROR_INVALID_DATA;
		return;
	}

	get_effecter_state_field_t *state = &res_p->field[0];
	bool is_access = is_e1s_access(e1s_sensor_table[effector_id - PLAT_EFFECTER_ID_LED_E1S_0]);
	uint8_t status = get_ssd_led_status(effector_id);

	if (is_access && (status < LED_CTRL_MAX)) {
		state->effecter_op_state = PLDM_EFFECTER_ENABLED_NOUPDATEPENDING;
		state->present_state = state->pending_state =
			((status == LED_CTRL_ON) ? EFFECTER_STATE_LED_VALUE_ON :
						   EFFECTER_STATE_LED_VALUE_OFF);
	} else {
		state->effecter_op_state =
			is_access ? PLDM_EFFECTER_STATUSUNKNOWN : PLDM_EFFECTER_UNAVAILABLE;
		state->present_state = state->pending_state = EFFECTER_STATE_LED_VALUE_UNKNOWN;
	}

	*resp_len = PLDM_GET_STATE_EFFECTER_RESP_NO_STATE_FIELD_BYTES +
		    (sizeof(get_effecter_state_field_t) *
		     PLDM_PLATFORM_OEM_LED_EFFECTER_STATE_FIELD_COUNT);
	res_p->composite_effecter_count = PLDM_PLATFORM_OEM_LED_EFFECTER_STATE_FIELD_COUNT;
	res_p->completion_code = PLDM_SUCCESS;
}

static void plat_get_effecter_nic_type_handler(const uint8_t *buf, uint16_t len, uint8_t *resp,
					       uint16_t *resp_len)
{
	CHECK_NULL_ARG(buf);
	CHECK_NULL_ARG(resp);
	CHECK_NULL_ARG(resp_len);

	struct pldm_get_state_effecter_states_req *req_p =
		(struct pldm_get_state_effecter_states_req *)buf;
	struct pldm_get_state_effecter_states_resp *res_p =
		(struct pldm_get_state_effecter_states_resp *)resp;

	uint8_t effector_id = req_p->effecter_id & BIT_MASK(8);
	get_effecter_state_field_t *state = &res_p->field[0];
	*resp_len = 1;

	if ((effector_id != PLAT_EFFECTER_ID_NIC_TYPE)) {
		LOG_ERR("Unsupport NIC_TYPE effecter ID(%d)", effector_id);
		res_p->completion_code = PLDM_ERROR_INVALID_DATA;
		return;
	}

	uint8_t type = get_nic_config();

	switch (type) {
	case NIC_CONFIG_UNKNOWN:
		state->effecter_op_state = PLDM_EFFECTER_ENABLED_NOUPDATEPENDING;
		state->present_state = state->pending_state = EFFECTER_STATE_NIC_TYPE_UNKNOWN;
		break;
	case NIC_CONFIG_CX7:
		state->effecter_op_state = PLDM_EFFECTER_ENABLED_NOUPDATEPENDING;
		state->present_state = state->pending_state = EFFECTER_STATE_NIC_TYPE_CX7;
		break;
	case NIC_CONFIG_IB_CX7:
		state->effecter_op_state = PLDM_EFFECTER_ENABLED_NOUPDATEPENDING;
		state->present_state = state->pending_state = EFFECTER_STATE_NIC_TYPE_CX7_IB;
		break;
	case NIC_CONFIG_THOR2:
		state->effecter_op_state = PLDM_EFFECTER_ENABLED_NOUPDATEPENDING;
		state->present_state = state->pending_state = EFFECTER_STATE_NIC_TYPE_THOR2;
		break;
	case NIC_CONFIG_POLLARA:
		state->effecter_op_state = PLDM_EFFECTER_ENABLED_NOUPDATEPENDING;
		state->present_state = state->pending_state = EFFECTER_STATE_NIC_TYPE_POLLARA;
		break;
	default:
		state->effecter_op_state = PLDM_EFFECTER_STATUSUNKNOWN;
		state->present_state = state->pending_state = EFFECTER_STATE_NIC_TYPE_UNKNOWN;
		LOG_ERR("Unsupport NIC type, (%d)", type);
		break;
	}

	*resp_len = PLDM_GET_STATE_EFFECTER_RESP_NO_STATE_FIELD_BYTES +
		    (sizeof(get_effecter_state_field_t) *
		     PLDM_PLATFORM_OEM_NIC_TYPE_EFFECTER_STATE_FIELD_COUNT);
	res_p->composite_effecter_count = PLDM_PLATFORM_OEM_NIC_TYPE_EFFECTER_STATE_FIELD_COUNT;
	res_p->completion_code = PLDM_SUCCESS;
}

uint8_t plat_pldm_set_state_effecter_state_handler(const uint8_t *buf, uint16_t len, uint8_t *resp,
						   uint16_t *resp_len,
						   struct pldm_state_effecter_info *info_p)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_ARG_WITH_RETURN(!len, PLDM_ERROR);

	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	switch (info_p->entity_type) {
	case PLDM_ENTITY_IO_CONTROLLER:
		set_effecter_state_gpio_handler(buf, len, resp, resp_len,
						(uint8_t)(info_p->effecter_id & GENMASK(7, 0)));
		break;
	case PLDM_OEM_ENTITY_LED:
		plat_set_effecter_led_handler(buf, len, resp, resp_len);
		break;
	case PLDM_OEM_ENTITY_SSD_LED:
		plat_set_effecter_ssd_led_handler(buf, len, resp, resp_len);
		break;
	default:
		LOG_ERR("Unsupport entity type, (%d)", info_p->entity_type);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		break;
	}

	return PLDM_SUCCESS;
}

uint8_t plat_pldm_get_state_effecter_state_handler(const uint8_t *buf, uint16_t len, uint8_t *resp,
						   uint16_t *resp_len,
						   struct pldm_state_effecter_info *info_p)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_ARG_WITH_RETURN(!len, PLDM_ERROR);

	uint8_t *completion_code_p = resp;
	*resp_len = 1;

	switch (info_p->entity_type) {
	case PLDM_ENTITY_IO_CONTROLLER:
		get_effecter_state_gpio_handler(buf, len, resp, resp_len,
						(uint8_t)(info_p->effecter_id & GENMASK(7, 0)));
		break;
	case PLDM_OEM_ENTITY_LED:
		plat_get_effecter_led_handler(buf, len, resp, resp_len);
		break;
	case PLDM_OEM_ENTITY_SSD_LED:
		plat_get_effecter_ssd_led_handler(buf, len, resp, resp_len);
		break;
	case PLDM_OEM_ENTITY_NIC_TYPE:
		plat_get_effecter_nic_type_handler(buf, len, resp, resp_len);
		break;
	default:
		LOG_ERR("Unsupport entity type, (%d)", info_p->entity_type);
		*completion_code_p = PLDM_ERROR_INVALID_DATA;
		break;
	}

	return PLDM_SUCCESS;
}

void plat_pldm_load_state_effecter_table(void)
{
	memcpy(state_effecter_table, plat_state_effecter_table, sizeof(plat_state_effecter_table));
	return;
}
