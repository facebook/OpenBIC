#include "plat_cpu.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <drivers/peci.h>
#include "intel_peci.h"
#include "ipmi.h"
#include "libipmi.h"
#include "libutil.h"
#include "hal_peci.h"
#include <logging/log.h>
#include <plat_sensor_table.h>
#include "power_status.h"
#include "hal_vw_gpio.h"
#include "fru.h"
#include "plat_fru.h"
#include "eeprom.h"
#include "hal_gpio.h"
#include "plat_gpio.h"

#define DAM_PIN_DISABLE 0
#define DAM_PIN_ENABLE 1

LOG_MODULE_REGISTER(plat_cpu);

K_THREAD_STACK_DEFINE(monitor_cpu_stack, MONITOR_CPU_STACK_SIZE);
struct k_thread monitor_cpu_thread;
k_tid_t monitor_cpu_tid;

K_THREAD_STACK_DEFINE(monitor_smiout_stack, MONITOR_SMIOUT_STACK_SIZE);
struct k_thread monitor_smiout_thread;
k_tid_t monitor_smiout_tid;

static uint8_t smi_val;

void start_monitor_cpu_thread()
{
	LOG_INF("Start thread to monitor CPU");

	monitor_cpu_tid =
		k_thread_create(&monitor_cpu_thread, monitor_cpu_stack,
				K_THREAD_STACK_SIZEOF(monitor_cpu_stack), monitor_cpu_handler, NULL,
				NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&monitor_cpu_thread, "monitor_cpu_thread");
}

void monitor_cpu_handler()
{
	uint8_t command = PECI_RD_PKG_CFG0_CMD;
	uint8_t readlen = 0x05;
	uint8_t prochot = 0, tcc_act = 0, cpu_crti_temp = 0;
	static bool tcc_act_assert = false, prochot_assert = false, cpu_crti_temp_assert = false;
	common_addsel_msg_t sel_msg;

	int ret = 0;
	uint8_t *readbuf = (uint8_t *)malloc(readlen * sizeof(uint8_t));
	if (!readbuf) {
		LOG_ERR("%s fail to allocate readbuf memory", __func__);
		return;
	}

	sel_msg.InF_target = BMC_IPMB;
	sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_CPU_THERM_STATUS;
	sel_msg.sensor_number = SENSOR_NUM_CPU0_THERM_STATUS;
	sel_msg.event_data2 = 0xFF;
	sel_msg.event_data3 = 0xFF;

	while (1) {
		// PECI can only be accessed after post complete
		if (!get_post_status()) {
			k_msleep(MONITOR_CPU_TIME_MS);
			continue;
		}

		ret = peci_read(command, CPU_PECI_ADDR, RDPKG_IDX_PKG_THERMAL_STATUS, 0, readlen,
				readbuf);
		if (ret) {
			LOG_ERR("Get cpu thermal status peci read error");
			goto cleanup;
		}

		if (readbuf[0] != PECI_CC_RSP_SUCCESS) {
			if (readbuf[0] == PECI_CC_ILLEGAL_REQUEST) {
				LOG_ERR("Get cpu thermal status unknown request");
			} else {
				LOG_ERR("Get cpu thermal status peci control hardware, firmware or associated logic error");
			}
			goto cleanup;
		}

		tcc_act = GETBIT(readbuf[1], 0);
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_TCC_ACT;
		if ((tcc_act == THERMAL_STATUS_ASSERT) && (tcc_act_assert == false)) {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			tcc_act_assert = true;
			if (!common_add_sel_evt_record(&sel_msg)) {
				LOG_ERR("Failed to add TCC Activation Assert SEL");
			}
		}
		if ((tcc_act == THERMAL_STATUS_DEASSERT) && (tcc_act_assert == true)) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
			tcc_act_assert = false;
			if (!common_add_sel_evt_record(&sel_msg)) {
				LOG_ERR("Failed to add TCC Activation Deassert SEL");
			}
		}

		prochot = GETBIT(readbuf[1], 2);
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_PROCHOT;
		if ((prochot == THERMAL_STATUS_ASSERT) && (prochot_assert == false)) {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			prochot_assert = true;
			if (!common_add_sel_evt_record(&sel_msg)) {
				LOG_ERR("Failed to add PROCHOT Assert SEL\n");
			}
		}
		if ((prochot == THERMAL_STATUS_DEASSERT) && (prochot_assert == true)) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
			prochot_assert = false;
			if (!common_add_sel_evt_record(&sel_msg)) {
				LOG_ERR("Failed to add PROCHOT Deassert SEL\n");
			}
		}

		cpu_crti_temp = GETBIT(readbuf[1], 4);
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_CPU_CRIT_TEMP;
		if ((cpu_crti_temp == THERMAL_STATUS_ASSERT) && (cpu_crti_temp_assert == false)) {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			cpu_crti_temp_assert = true;
			if (!common_add_sel_evt_record(&sel_msg)) {
				LOG_ERR("Failed to add CPU critical temperature Assert SEL\n");
			}
		}
		if ((prochot == THERMAL_STATUS_DEASSERT) && (cpu_crti_temp_assert == true)) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
			cpu_crti_temp_assert = false;
			if (!common_add_sel_evt_record(&sel_msg)) {
				LOG_ERR("Failed to add CPU critical temperature Deassert SEL\n");
			}
		}

		memset(readbuf, 0, readlen * sizeof(uint8_t));

		k_msleep(MONITOR_CPU_TIME_MS);
	}

cleanup:
	SAFE_FREE(readbuf);
	return;
}

bool pal_get_cpu_energy(uint8_t addr, uint32_t *pkg_energy, uint32_t *run_time)
{
	uint8_t command = PECI_RD_PKG_CFG0_CMD;
	uint8_t readlen = 0x09;
	uint8_t index = 0x03;
	uint16_t parameter = 0x00FF;
	int ret = 0;

	uint8_t *readbuf = (uint8_t *)malloc(readlen * sizeof(uint8_t));
	if (!readbuf) {
		LOG_ERR("%s fail to allocate readbuf memory", __func__);
		return false;
	}

	ret = peci_read(command, addr, index, parameter, readlen, readbuf);
	if (ret) {
		LOG_ERR("PECI read cpu energy and time error");
		goto cleanup;
	}
	if (readbuf[0] != PECI_CC_RSP_SUCCESS) {
		if (readbuf[0] == PECI_CC_ILLEGAL_REQUEST) {
			LOG_ERR("%s unknown request", __func__);
		} else {
			LOG_ERR("%s peci control hardware, firmware or associated logic error",
				__func__);
		}
		goto cleanup;
	}

	*pkg_energy = readbuf[4];
	*pkg_energy = (*pkg_energy << 8) | readbuf[3];
	*pkg_energy = (*pkg_energy << 8) | readbuf[2];
	*pkg_energy = (*pkg_energy << 8) | readbuf[1];

	*run_time = readbuf[8];
	*run_time = (*run_time << 8) | readbuf[7];
	*run_time = (*run_time << 8) | readbuf[6];
	*run_time = (*run_time << 8) | readbuf[5];

	SAFE_FREE(readbuf);
	return true;
cleanup:
	SAFE_FREE(readbuf);
	return false;
}

void pal_cal_cpu_power(intel_peci_unit unit_info, uint32_t diff_energy, uint32_t diff_time,
		       int *reading)
{
	float time_unit_energy = (float)((float)diff_energy * (float)CPU_TIME_UNIT);

	float pwr_scale = (float)(1 / (float)(1 << unit_info.energy_unit));

	*reading = (time_unit_energy / (float)diff_time) * pwr_scale;
}

void start_monitor_smi_thread()
{
	LOG_INF("Start thread to monitor SMIOUT");

	monitor_smiout_tid =
		k_thread_create(&monitor_smiout_thread, monitor_smiout_stack,
				K_THREAD_STACK_SIZEOF(monitor_smiout_stack), monitor_smiout_handler,
				NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&monitor_smiout_thread, "monitor_smiout_thread");
}

void monitor_smiout_handler()
{
	time_t t1, t2;
	static bool smi_assert = false;
	uint32_t sysevt_val;
	common_addsel_msg_t sel_msg;

	sel_msg.InF_target = BMC_IPMB;
	sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
	sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
	sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_SMI90s;
	sel_msg.event_data2 = 0xFF;
	sel_msg.event_data3 = 0xFF;

	while (1) {
		sysevt_val = sys_read32(AST_ESPI_BASE + AST_ESPI_SYSEVT);
		smi_val = GETBIT(sysevt_val, SMIOUT_INDEX);

		if ((smi_val == 0) && (smi_assert == false)) {
			t1 = time(NULL);
			while (1) {
				sysevt_val = sys_read32(AST_ESPI_BASE + AST_ESPI_SYSEVT);
				smi_val = GETBIT(sysevt_val, SMIOUT_INDEX);

				if (smi_val == 1) {
					break;
				}

				t2 = time(NULL);
				if ((t2 - t1) >= SMIOUT_TIMEOUT) {
					sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
					if (!common_add_sel_evt_record(&sel_msg)) {
						LOG_ERR("Failed to add SMIOUT stuck low Assert SEL");
					}
					smi_assert = true;
					break;
				}
				k_msleep(MONITOR_SMIOUT_TIME_MS);
			}
		} else if ((smi_val == 1) && (smi_assert == true)) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
			if (!common_add_sel_evt_record(&sel_msg)) {
				LOG_ERR("Failed to add SMIOUT stuck low Deassert SEL");
			}
			smi_assert = false;
		} else if ((smi_val == 0) || (smi_val == 1)) {
			k_msleep(MONITOR_SMIOUT_TIME_MS);
		} else {
			LOG_ERR("Unknown status for SMIOUT");
			return;
		}
	}
}

int pal_get_set_dam_status(uint8_t options, uint8_t *status)
{
	CHECK_NULL_ARG_WITH_RETURN(status, -1);

	uint8_t ret;
	EEPROM_ENTRY fru_entry = { 0 };

	fru_entry.config.dev_id = SYS_DAM_ID;
	fru_entry.offset = BIC_CONFIG_START + SYS_DAM_OFFSET;
	fru_entry.data_len = 1;

	uint8_t fru_index = 0;
	bool is_id_find = find_FRU_ID(fru_entry.config.dev_id, &fru_index);
	if (is_id_find == false) {
		LOG_ERR("find fru write config fail via fru id: 0x%x", fru_entry.config.dev_id);
		return FRU_INVALID_ID;
	}
	memcpy(&fru_entry.config, &fru_config[fru_index], sizeof(fru_config[fru_index]));

	switch (options) {
		case GET_STATUS:
			ret = eeprom_read(&fru_entry);
			*status = fru_entry.data[0];
			break;
		case SET_STATUS:
			fru_entry.data[0] = *status;
			ret = eeprom_write(&fru_entry);
			break;
		default:
			LOG_ERR("Debug sel mode options unkown %d", options);
			return -1;
	}

	if (ret != FRU_READ_SUCCESS) {
		return -1;
	}

	return 0;
}

// Use in BIC initial, set DAM pin status from eeprom data
void set_dam_pin()
{
	int dam_pin_status = 0;
	uint8_t dam_eeprom_status = 0;

	if (pal_get_set_dam_status(GET_STATUS, &dam_eeprom_status) != 0) {
		LOG_ERR("cannot get DAM eeprom data!");
		return;
	}

	dam_pin_status = gpio_get(DAM_BIC_R_EN); // get DAM pin status

	// if dam status record in eeprom date not same as DAM pin status, set DAM pin status
	if (dam_eeprom_status != dam_pin_status) {
		if (dam_eeprom_status == DAM_PIN_ENABLE) {
			gpio_set(DAM_BIC_R_EN, GPIO_HIGH);
			LOG_INF("BIC set DAM pin HIGH");
		} else {
			gpio_set(DAM_BIC_R_EN, GPIO_LOW);
			LOG_INF("BIC set DAM pin LOW");
		}
	} else {
		LOG_INF("BIC check DAM pin not change");
	}

	return;
}

// if BMC use IPMI command let BIC set DAM pin, record status to eeprom
void OEM_1S_RECORD_DAM_PIN_STATUS(uint8_t gpio_num, uint8_t status)
{
	uint8_t dam_eeprom_status = 0;
	if (gpio_num == DAM_BIC_R_EN) {
		if(status == DAM_PIN_ENABLE) {
			dam_eeprom_status = DAM_PIN_ENABLE;
			LOG_INF("BIC receive BMC set DAM pin high");
		} else {
			dam_eeprom_status = DAM_PIN_DISABLE;
			LOG_INF("BIC receive BMC set DAM pin low");
		}

		if (pal_get_set_dam_status(SET_STATUS, &dam_eeprom_status) != 0) {
			LOG_ERR("cannot set DAM eeprom data!");
		}
		LOG_INF("BIC set DAM status in eeprom data");
	}

	return;
}
