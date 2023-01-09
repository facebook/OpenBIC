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

#include "plat_gpio.h"
#include "plat_sensor_table.h"
#include "plat_pldm_monitor.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_pldm_monitor);

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
	SENSOR_NUM_BB_TEMP_PEX_0,
	SENSOR_NUM_BB_TEMP_PEX_1,
	SENSOR_NUM_BB_TEMP_PEX_2,
	SENSOR_NUM_BB_TEMP_PEX_3,
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

static struct ssd_event_check {
	struct k_work_delayable work;
	bool init;
	uint8_t group;
	uint8_t flag;
} ssd_event_check[2];

void ssd_alert_check_ina230(uint8_t idx, uint8_t *flag)
{
	uint8_t sensor_num = ssd_power_ic_sensor_table[idx];
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	uint8_t retry = 5;
	I2C_MSG msg;

	if (cfg->pre_sensor_read_hook) {
		if (!cfg->pre_sensor_read_hook(cfg->num, cfg->pre_sensor_read_args)) {
			LOG_ERR("SSD%d with sensor number(0x%x) pre reading failed", idx, cfg->num);
			return;
		}
	}

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
		if (!cfg->post_sensor_read_hook(sensor_num, cfg->post_sensor_read_args, NULL)) {
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
		if (!cfg->post_sensor_read_hook(sensor_num, cfg->post_sensor_read_args, NULL)) {
			LOG_ERR("SSD%d with sensor number(0x%x) post reading failed", idx,
				cfg->num);
		}
	}
	return;
}

void ssd_alert_check_isl28022(uint8_t idx, uint8_t *flag)
{
	uint8_t sensor_num = ssd_power_ic_sensor_table[idx];
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	uint8_t retry = 5;
	I2C_MSG msg;

	if (cfg->pre_sensor_read_hook) {
		if (!cfg->pre_sensor_read_hook(cfg->num, cfg->pre_sensor_read_args)) {
			LOG_ERR("SSD%d with sensor number(0x%x) pre reading failed", idx, cfg->num);
			return;
		}
	}

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
		if (!cfg->post_sensor_read_hook(sensor_num, cfg->post_sensor_read_args, NULL)) {
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
		if (!cfg->post_sensor_read_hook(sensor_num, cfg->post_sensor_read_args, NULL)) {
			LOG_ERR("SSD%d with sensor number(0x%x) post reading failed", idx,
				cfg->num);
		}
	}
	return;
}

void ssd_alert_check_handler(struct k_work *work)
{
	struct ssd_event_check *data = CONTAINER_OF(work, struct ssd_event_check, work);

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

	if (data->flag)
		k_work_schedule((struct k_work_delayable *)work, K_SECONDS(5));
}

void ssd_alert_check(uint8_t group)
{
	if (group > 1) {
		LOG_ERR("The argument value is invalid");
		return;
	}

	struct ssd_event_check *data = ((group == 0) ? &ssd_event_check[0] : &ssd_event_check[1]);

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
	for (uint8_t i = 0; i < 16; i++) {
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
	for (uint8_t i = 0; i < 8; i++) {
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