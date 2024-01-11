#include "plat_dimm.h"

#include <stdio.h>
#include <stdlib.h>
#include <zephyr.h>
#include <logging/log.h>
#include <errno.h>
#include "plat_sensor_table.h"
#include "ipmb.h"
#include "ipmi.h"
#include "kcs.h"
#include "sensor.h"
#include "pmic.h"
#include "power_status.h"
#include "plat_i2c.h"
#include "plat_isr.h"
#include "plat_class.h"
#include "libutil.h"
#include "rg3mxxb12.h"
#include "p3h284x.h"
#include "intel_peci.h"

LOG_MODULE_REGISTER(plat_dimm);

K_THREAD_STACK_DEFINE(get_dimm_info_stack, GET_DIMM_INFO_STACK_SIZE);
struct k_thread get_dimm_info_thread;
k_tid_t get_dimm_info_tid;

struct k_mutex i3c_dimm_mutex;

uint8_t pmic_i3c_addr_list[MAX_COUNT_DIMM / 2] = { PMIC_A0_A4_ADDR, PMIC_A1_A5_ADDR,
						   PMIC_A2_A6_ADDR, PMIC_A3_A7_ADDR };
uint8_t spd_i3c_addr_list[MAX_COUNT_DIMM / 2] = { DIMM_SPD_A0_A4_ADDR, DIMM_SPD_A1_A5_ADDR,
						  DIMM_SPD_A2_A6_ADDR, DIMM_SPD_A3_A7_ADDR };

dimm_info dimm_data[MAX_COUNT_DIMM];

static bool dimm_prsnt_inited = false;

void start_get_dimm_info_thread()
{
	LOG_INF("Start thread to get dimm information");

	get_dimm_info_tid =
		k_thread_create(&get_dimm_info_thread, get_dimm_info_stack,
				K_THREAD_STACK_SIZEOF(get_dimm_info_stack), get_dimm_info_handler,
				NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&get_dimm_info_thread, "get_dimm_info_thread");
}

bool is_dimm_prsnt_inited()
{
	return dimm_prsnt_inited;
}

bool is_dimm_ready_monitor(uint8_t dimm_id)
{
	return dimm_data[dimm_id].is_ready_monitor;
}

void get_dimm_info_handler()
{
	I3C_MSG i3c_msg = { 0 };
	int i;

	i3c_msg.bus = I3C_BUS4;
	uint16_t i3c_hub_type = I3C_HUB_TYPE_UNKNOWN;
	i3c_hub_type = get_i3c_hub_type();

	// Attach PMIC addr
	for (i = 0; i < (MAX_COUNT_DIMM / 2); i++) {
		i3c_msg.target_addr = pmic_i3c_addr_list[i];
		i3c_attach(&i3c_msg);
	}

	// Init mutex
	if (k_mutex_init(&i3c_dimm_mutex)) {
		LOG_ERR("i3c_dimm_mux_mutex mutex init fail");
	}

	// Switch I3C mux to BIC when host post complete but BIC reset
	if (get_post_status()) {
		if (k_mutex_lock(&i3c_dimm_mutex, K_MSEC(I3C_DIMM_MUTEX_TIMEOUT_MS))) {
			LOG_ERR("Failed to lock I3C dimm MUX");
			return;
		}

		switch_i3c_dimm_mux(I3C_MUX_TO_BIC);

		if (k_mutex_unlock(&i3c_dimm_mutex)) {
			LOG_ERR("Failed to unlock I3C dimm MUX");
		}
	}

	while (1) {
		int ret = 0, dimm_id;

		// Avoid to get wrong thus only monitor after post complete
		if (!get_post_status()) {
			k_msleep(GET_DIMM_INFO_TIME_MS);
			continue;
		}

		if (!is_dimm_prsnt_inited()) {
			init_i3c_dimm_prsnt_status();
		}

		if (k_mutex_lock(&i3c_dimm_mutex, K_MSEC(I3C_DIMM_MUTEX_TIMEOUT_MS))) {
			LOG_ERR("Failed to lock I3C dimm MUX");
			k_msleep(GET_DIMM_INFO_TIME_MS);
			continue;
		}

		for (dimm_id = 0; dimm_id < MAX_COUNT_DIMM; dimm_id++) {
			if (!dimm_data[dimm_id].is_present) {
				continue;
			}

			ret = switch_i3c_dimm_mux(I3C_MUX_TO_BIC);
			if (ret != 0) {
				clear_unaccessible_dimm_data(dimm_id);
				continue;
			}

			uint8_t slave_port_setting = (dimm_id / (MAX_COUNT_DIMM / 2)) ?
							     I3C_HUB_TO_DIMMEFGH :
							     I3C_HUB_TO_DIMMABCD;

			if (i3c_hub_type == RG3M87B12_DEVICE_INFO) {
				if (!rg3mxxb12_set_slave_port(I3C_BUS4,
							      RG3MXXB12_DEFAULT_STATIC_ADDRESS,
							      slave_port_setting)) {
					clear_unaccessible_dimm_data(dimm_id);
					LOG_ERR("Failed to set slave port to slave port: 0x%x",
						slave_port_setting);
					continue;
				}
			} else {
				if (!p3h284x_set_slave_port(I3C_BUS4,
							    P3H284X_DEFAULT_STATIC_ADDRESS,
							    slave_port_setting)) {
					clear_unaccessible_dimm_data(dimm_id);
					LOG_ERR("Failed to set slave port to slave port: 0x%x",
						slave_port_setting);
					continue;
				}
			}

			memset(&i3c_msg, 0, sizeof(I3C_MSG));
			i3c_msg.bus = I3C_BUS4;
			i3c_msg.target_addr = spd_i3c_addr_list[dimm_id % (MAX_COUNT_DIMM / 2)];
			i3c_attach(&i3c_msg);

			// I3C_CCC_RSTDAA: Reset dynamic address assignment
			// I3C_CCC_SETAASA: Set all addresses to static address
			ret = all_brocast_ccc(&i3c_msg);
			if (ret != 0) {
				clear_unaccessible_dimm_data(dimm_id);
				i3c_detach(&i3c_msg);
				continue;
			}

			if (!get_post_status()) {
				i3c_detach(&i3c_msg);
				break;
			}

			i3c_msg.tx_len = 1;
			i3c_msg.rx_len = MAX_LEN_I3C_GET_SPD_TEMP;
			i3c_msg.data[0] = DIMM_SPD_TEMP;

			ret = i3c_spd_reg_read(&i3c_msg, false);
			if (ret != 0) {
				clear_unaccessible_dimm_data(dimm_id);
				i3c_detach(&i3c_msg);
				LOG_ERR("Failed to read DIMM %d SPD temperature via I3C, ret= %d",
					dimm_id, ret);
			} else {
				memcpy(&dimm_data[dimm_id].spd_temp_data, &i3c_msg.data,
				       sizeof(dimm_data[dimm_id].spd_temp_data));
			}
			i3c_detach(&i3c_msg);

			// Double check before read each DIMM info
			if (!get_post_status()) {
				dimm_data[dimm_id].is_ready_monitor = false;
				break;
			}

			// Read DIMM PMIC power
			memset(&i3c_msg, 0, sizeof(I3C_MSG));
			i3c_msg.bus = I3C_BUS4;
			i3c_msg.target_addr = pmic_i3c_addr_list[dimm_id % (MAX_COUNT_DIMM / 2)];
			i3c_msg.tx_len = 1;
			i3c_msg.rx_len = MAX_LEN_I3C_GET_PMIC_PWR;
			i3c_msg.data[0] = DIMM_PMIC_SWA_PWR;

			ret = i3c_transfer(&i3c_msg);
			if (ret != 0) {
				clear_unaccessible_dimm_data(dimm_id);
				LOG_ERR("Failed to read DIMM %d PMIC power via I3C, ret= %d",
					dimm_id, ret);
				continue;
			} else {
				memcpy(&dimm_data[dimm_id].pmic_pwr_data, &i3c_msg.data,
				       sizeof(dimm_data[dimm_id].pmic_pwr_data));
			}

			// Double check before read each DIMM info
			if (!get_post_status()) {
				dimm_data[dimm_id].is_ready_monitor = false;
				break;
			}

			// Read DIMM PMIC error
			memset(&i3c_msg, 0, sizeof(I3C_MSG));
			i3c_msg.bus = I3C_BUS4;
			i3c_msg.target_addr = pmic_i3c_addr_list[dimm_id % (MAX_COUNT_DIMM / 2)];
			i3c_msg.tx_len = 1;
			i3c_msg.rx_len = MAX_LEN_I3C_GET_PMIC_ERR;
			i3c_msg.data[0] = PMIC_POR_ERROR_LOG_ADDR_VAL;

			ret = i3c_transfer(&i3c_msg);
			if (ret != 0) {
				clear_unaccessible_dimm_data(dimm_id);
				LOG_ERR("Failed to read DIMM %d PMIC error via I3C, ret= %d",
					dimm_id, ret);
				continue;
			} else {
				memcpy(&dimm_data[dimm_id].pmic_error_data, &i3c_msg.data,
				       sizeof(dimm_data[dimm_id].pmic_error_data));
			}
			// If the DIMM is ready for monitoring, BIC can send its temperature to CPU by PECI.
			dimm_data[dimm_id].is_ready_monitor = true;
		}

		if (k_mutex_unlock(&i3c_dimm_mutex)) {
			LOG_ERR("Failed to unlock I3C dimm MUX");
		}

		k_msleep(GET_DIMM_INFO_TIME_MS);
	}
}

void init_i3c_dimm_prsnt_status()
{
	int index;
	for (index = DIMM_ID_A0; index < MAX_COUNT_DIMM; index++) {
		ipmi_msg msg = { 0 };

		msg.InF_source = SELF;
		msg.InF_target = BMC_IPMB;
		msg.netfn = NETFN_OEM_Q_REQ;
		msg.cmd = CMD_OEM_Q_GET_DIMM_INFO;
		msg.data_len = 5;
		msg.data[0] = IANA_ID & 0xFF;
		msg.data[1] = (IANA_ID >> 8) & 0xFF;
		msg.data[2] = (IANA_ID >> 16) & 0xFF;
		msg.data[3] = (uint8_t)index;
		msg.data[4] = CMD_DIMM_LOCATION;

		ipmb_error ret = ipmb_read(&msg, IPMB_inf_index_map[msg.InF_target]);
		if (ret != IPMB_ERROR_SUCCESS) {
			LOG_ERR("Failed to get DIMM status, ret= %d", ret);
			return;
		}

		uint8_t status = msg.data[0];
		if (status == DIMM_PRESENT) {
			dimm_data[index].is_present = true;
		} else {
			dimm_data[index].is_present = false;
		}
	}

	dimm_prsnt_inited = true;
}

bool get_dimm_presence_status(uint8_t dimm_id)
{
	return dimm_data[dimm_id].is_present;
}

void set_dimm_presence_status(uint8_t index, uint8_t status)
{
	if (status == DIMM_PRESENT) {
		dimm_data[index].is_present = true;
	} else {
		dimm_data[index].is_present = false;
	}
}

uint8_t sensor_num_map_dimm_id(uint8_t sensor_num)
{
	uint8_t dimm_id = DIMM_ID_UNKNOWN;

	switch (sensor_num) {
	case SENSOR_NUM_MB_DIMMA0_TEMP_C:
	case SENSOR_NUM_MB_VR_DIMMA0_PMIC_PWR_W:
		dimm_id = DIMM_ID_A0;
		break;
	case SENSOR_NUM_MB_DIMMA1_TEMP_C:
	case SENSOR_NUM_MB_VR_DIMMA1_PMIC_PWR_W:
		dimm_id = DIMM_ID_A1;
		break;
	case SENSOR_NUM_MB_DIMMA2_TEMP_C:
	case SENSOR_NUM_MB_VR_DIMMA2_PMIC_PWR_W:
		dimm_id = DIMM_ID_A2;
		break;
	case SENSOR_NUM_MB_DIMMA3_TEMP_C:
	case SENSOR_NUM_MB_VR_DIMMA3_PMIC_PWR_W:
		dimm_id = DIMM_ID_A3;
		break;
	case SENSOR_NUM_MB_DIMMA4_TEMP_C:
	case SENSOR_NUM_MB_VR_DIMMA4_PMIC_PWR_W:
		dimm_id = DIMM_ID_A4;
		break;
	case SENSOR_NUM_MB_DIMMA5_TEMP_C:
	case SENSOR_NUM_MB_VR_DIMMA5_PMIC_PWR_W:
		dimm_id = DIMM_ID_A5;
		break;
	case SENSOR_NUM_MB_DIMMA6_TEMP_C:
	case SENSOR_NUM_MB_VR_DIMMA6_PMIC_PWR_W:
		dimm_id = DIMM_ID_A6;
		break;
	case SENSOR_NUM_MB_DIMMA7_TEMP_C:
	case SENSOR_NUM_MB_VR_DIMMA7_PMIC_PWR_W:
		dimm_id = DIMM_ID_A7;
		break;
	default:
		dimm_id = DIMM_ID_UNKNOWN;
		break;
	}

	return dimm_id;
}

int switch_i3c_dimm_mux(uint8_t i3c_mux_position)
{
	I2C_MSG i2c_msg = { 0 };
	int ret = 0, retry = 3;

	i2c_msg.bus = I2C_BUS1;
	i2c_msg.target_addr = CPLD_ADDR;
	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = 0;
	i2c_msg.data[0] = DIMM_I3C_MUX_CONTROL_OFFSET;

	// BIT 0: PD_SPD1_REMOTE_EN
	// 0: switch I3C mux to CPU 1: switch I3C mux to BIC
	i2c_msg.data[1] = i3c_mux_position;

	ret = i2c_master_write(&i2c_msg, retry);
	if (ret != 0) {
		LOG_ERR("Failed to switch I3C MUX: 0x%x, ret= %d", i3c_mux_position, ret);
	}

	return ret;
}

int all_brocast_ccc(I3C_MSG *i3c_msg)
{
	CHECK_NULL_ARG_WITH_RETURN(i3c_msg, -1);

	int ret = 0;

	ret = i3c_brocast_ccc(i3c_msg, I3C_CCC_RSTDAA, I3C_BROADCAST_ADDR);
	if (ret != 0) {
		return ret;
	}

	ret = i3c_brocast_ccc(i3c_msg, I3C_CCC_SETAASA, I3C_BROADCAST_ADDR);
	if (ret != 0) {
		return ret;
	}

	return ret;
}

int get_pmic_error_raw_data(int dimm_index, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, -1);

	int i = 0, fail_count = 0;

	for (i = 0; i < sizeof(dimm_data[dimm_index].pmic_error_data); i++) {
		if (dimm_data[dimm_index].pmic_error_data[i] == SENSOR_FAIL) {
			fail_count++;
		}
	}

	// PMIC error data read failed
	if (fail_count == sizeof(dimm_data[dimm_index].pmic_error_data)) {
		return -1;
	}

	memcpy(data, &dimm_data[dimm_index].pmic_error_data,
	       sizeof(dimm_data[dimm_index].pmic_error_data));

	return 0;
}

void get_pmic_power_raw_data(int dimm_index, uint8_t *data)
{
	CHECK_NULL_ARG(data);

	memcpy(data, &dimm_data[dimm_index].pmic_pwr_data,
	       sizeof(dimm_data[dimm_index].pmic_pwr_data));
}

void get_spd_temp_raw_data(int dimm_index, uint8_t *data)
{
	CHECK_NULL_ARG(data);

	memcpy(data, &dimm_data[dimm_index].spd_temp_data,
	       sizeof(dimm_data[dimm_index].spd_temp_data));
}

int pal_get_pmic_pwr(uint8_t sensor_num, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, -1);

	uint8_t dimm_id = DIMM_ID_UNKNOWN;

	dimm_id = sensor_num_map_dimm_id(sensor_num);
	if (dimm_id == DIMM_ID_UNKNOWN) {
		return -1;
	}

	get_pmic_power_raw_data(dimm_id, (uint8_t *)data);

	// If sensor data is SENSOR_FAIL, return failed
	if (data[0] == SENSOR_FAIL) {
		return -1;
	}

	return 0;
}

int pal_get_spd_temp(uint8_t sensor_num, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, -1);

	uint8_t dimm_id = DIMM_ID_UNKNOWN;

	dimm_id = sensor_num_map_dimm_id(sensor_num);
	if (dimm_id == DIMM_ID_UNKNOWN) {
		return -1;
	}

	get_spd_temp_raw_data(dimm_id, (uint8_t *)data);

	// If sensor data is SENSOR_FAIL, return failed
	if (data[0] == SENSOR_FAIL) {
		return -1;
	}

	return 0;
}

void clear_unaccessible_dimm_data(uint8_t dimm_id)
{
	memset(dimm_data[dimm_id].pmic_error_data, SENSOR_FAIL,
	       sizeof(dimm_data[dimm_id].pmic_error_data));
	memset(dimm_data[dimm_id].pmic_pwr_data, SENSOR_FAIL,
	       sizeof(dimm_data[dimm_id].pmic_pwr_data));
	memset(dimm_data[dimm_id].spd_temp_data, SENSOR_FAIL,
	       sizeof(dimm_data[dimm_id].spd_temp_data));
}

void pal_cal_total_dimm_power(intel_peci_unit unit_info, uint32_t diff_energy, uint32_t diff_time,
			      int *reading)
{
	CHECK_NULL_ARG(reading);

	float pwr_scale = (float)(1 / (float)(1 << unit_info.energy_unit));

	*reading = ((float)diff_energy / (float)diff_time) * pwr_scale;
}
