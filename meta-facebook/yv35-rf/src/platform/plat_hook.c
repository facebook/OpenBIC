#include <stdio.h>
#include <string.h>
#include "ast_adc.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
adc_asd_init_arg adc_asd_init_args[] = { [0] = { .is_init = false } };

ina233_init_arg ina233_init_args[] = {
	[0] = { .is_init = false },
	[1] = { .is_init = false },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
/*typedef struct _isl69254iraz_t_pre_arg_ {
	uint8_t vr_page;
} isl69254iraz_t_pre_arg;*/
isl69254iraz_t_pre_arg isl69254iraz_t_pre_read_args[] = {
	[0] = { 0x0 },
	[1] = { 0x1 },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_ina233_read(uint8_t sensor_num, void *args)
{
	ARG_UNUSED(args);
	ina233_init_arg *init_arg =
		(ina233_init_arg *)sensor_config[sensor_config_index_map[sensor_num]].init_args;
	if (init_arg == NULL) {
		printf("[%s] input initial pointer is NULL\n", __func__);
		return false;
	}

	if (init_arg->is_init != true) {
		int ret = 0, retry = 5;
		I2C_MSG msg;
		memset(&msg, 0, sizeof(I2C_MSG));

		msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
		msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
		msg.tx_len = 3;
		// Current_lsb = 0.001 , r_shunt = 0.005
		// Calibration formula = (0.00512 / (current_lsb * r_shunt)) = 1024(dec) = 0x400(hex)
		msg.data[0] = INA233_CALIBRATION_OFFSET;
		msg.data[1] = 0x00; // Calibration value
		msg.data[2] = 0x04; // Calibration value

		ret = i2c_master_write(&msg, retry);
		if (ret != 0) {
			printf("[%s] i2c write fail  ret: %d\n", __func__, ret);
			return false;
		}
		init_arg->is_init = true;
	}
	return true;
}

bool pre_isl69254iraz_t_read(uint8_t sensor_num, void *args)
{
	if (args == NULL) {
		printf("[%s] input args is NULL\n", __func__);
		return false;
	}

	isl69254iraz_t_pre_arg *pre_args = (isl69254iraz_t_pre_arg *)args;
	uint8_t retry = 5;
	int ret = 0;
	I2C_MSG msg;
	memset(&msg, 0, sizeof(I2C_MSG));

	/* set page */
	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 2;
	msg.data[0] = VR_PAGE_OFFSET;
	msg.data[1] = pre_args->vr_page;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		printf("[%s] i2c write fail  ret: %d\n", __func__, ret);
		return false;
	}
	return true;
}
