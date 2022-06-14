#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "plat_hook.h"
#include "plat_sensor_table.h"

#include "i2c-mux-tca9548.h"
#include "pex89000.h"

K_MUTEX_DEFINE(i2c_bus6_mutex);
K_MUTEX_DEFINE(i2c_bus9_mutex);
K_MUTEX_DEFINE(i2c_bus10_mutex);

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
/*
 * MODE: Shunt and bus, continuous
 * SADC/BADC: 128 samples
 * PG: 320 mv
 * BRNG: 16 V
 */
isl28022_init_arg nic_sensor_init_args[] = {
	[0] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[1] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[2] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[3] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[4] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[5] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[6] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[7] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
};

mp5990_init_arg mp5990_init_args[] = {
	/* Use default value by specification */
	[0] = { .is_init = false, .iout_cal_gain = 0x0140, .iout_oc_fault_limit = 0x0037 },
};

adc_asd_init_arg adc_asd_init_args[] = { [0] = { .is_init = false } };

/*
 * MODE: Shunt and bus, continuous
 * SADC/BADC: 128 samples
 * PG: 320 mv
 * BRNG: 16 V
 */
isl28022_init_arg pex_p1v25_sensor_init_args[] = {
	[0] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[1] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[2] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[3] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
};
/*
 * MODE: Shunt and bus, continuous
 * SADC/BADC: 128 samples
 * PG: 320 mv
 * BRNG: 16 V
 */
isl28022_init_arg pex_p1v8_sensor_init_args[] = {
	[0] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
};
/*
 * MODE: Shunt and bus, continuous
 * SADC/BADC: 128 samples
 * PG: 320 mv
 * BRNG: 16 V
 */
isl28022_init_arg ssd_sensor_init_args[] = {
	[0] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[1] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[2] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[3] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[4] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[5] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[6] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[7] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[8] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[9] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.r_shunt = 2,
		.is_init = false },
	[10] = { .config.fields.MODE = 0b111,
		 .config.fields.SADC = 0b1111,
		 .config.fields.BADC = 0b1111,
		 .config.fields.PG = 0b11,
		 .config.fields.BRNG = 0b00,
		 .r_shunt = 2,
		 .is_init = false },
	[11] = { .config.fields.MODE = 0b111,
		 .config.fields.SADC = 0b1111,
		 .config.fields.BADC = 0b1111,
		 .config.fields.PG = 0b11,
		 .config.fields.BRNG = 0b00,
		 .r_shunt = 2,
		 .is_init = false },
	[12] = { .config.fields.MODE = 0b111,
		 .config.fields.SADC = 0b1111,
		 .config.fields.BADC = 0b1111,
		 .config.fields.PG = 0b11,
		 .config.fields.BRNG = 0b00,
		 .r_shunt = 2,
		 .is_init = false },
	[13] = { .config.fields.MODE = 0b111,
		 .config.fields.SADC = 0b1111,
		 .config.fields.BADC = 0b1111,
		 .config.fields.PG = 0b11,
		 .config.fields.BRNG = 0b00,
		 .r_shunt = 2,
		 .is_init = false },
	[14] = { .config.fields.MODE = 0b111,
		 .config.fields.SADC = 0b1111,
		 .config.fields.BADC = 0b1111,
		 .config.fields.PG = 0b11,
		 .config.fields.BRNG = 0b00,
		 .r_shunt = 2,
		 .is_init = false },
	[15] = { .config.fields.MODE = 0b111,
		 .config.fields.SADC = 0b1111,
		 .config.fields.BADC = 0b1111,
		 .config.fields.PG = 0b11,
		 .config.fields.BRNG = 0b00,
		 .r_shunt = 2,
		 .is_init = false },
};

pex89000_init_arg pex_sensor_init_args[] = {
	[0] = { .idx = 0, .is_init = false },
	[1] = { .idx = 1, .is_init = false },
	[2] = { .idx = 2, .is_init = false },
	[3] = { .idx = 3, .is_init = false },
};
/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
struct tca9548 mux_conf_addr_0xe0[] = {
	[0] = { .addr = 0xe0, .chan = 0 }, [1] = { .addr = 0xe0, .chan = 1 },
	[2] = { .addr = 0xe0, .chan = 2 }, [3] = { .addr = 0xe0, .chan = 3 },
	[4] = { .addr = 0xe0, .chan = 4 }, [5] = { .addr = 0xe0, .chan = 5 },
	[6] = { .addr = 0xe0, .chan = 6 }, [7] = { .addr = 0xe0, .chan = 7 },
};

struct tca9548 mux_conf_addr_0xe2[] = {
	[0] = { .addr = 0xe2, .chan = 0 }, [1] = { .addr = 0xe2, .chan = 1 },
	[2] = { .addr = 0xe2, .chan = 2 }, [3] = { .addr = 0xe2, .chan = 3 },
	[4] = { .addr = 0xe2, .chan = 4 }, [5] = { .addr = 0xe2, .chan = 5 },
	[6] = { .addr = 0xe2, .chan = 6 }, [7] = { .addr = 0xe2, .chan = 7 },
};

isl69259_pre_proc_arg isl69259_pre_read_args[] = {
	[0] = { .mux_info_p = &mux_conf_addr_0xe0[6], .vr_page = 0x0 },
	[1] = { .mux_info_p = &mux_conf_addr_0xe0[6], .vr_page = 0x1 },
	[2] = { .mux_info_p = &mux_conf_addr_0xe0[6], .vr_page = 0x0 },
	[3] = { .mux_info_p = &mux_conf_addr_0xe0[6], .vr_page = 0x1 },
};
pex89000_pre_proc_arg pex89000_pre_read_args[] = {
	[0] = { .mux_info_p = &mux_conf_addr_0xe0[0] },
	[1] = { .mux_info_p = &mux_conf_addr_0xe0[1] },
	[2] = { .mux_info_p = &mux_conf_addr_0xe0[2] },
	[3] = { .mux_info_p = &mux_conf_addr_0xe0[3] },
};
/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/

/* ISL69259 pre read function
 *
 * set mux and VR page
 *
 * @param sensor_num sensor number
 * @param args pointer to isl69259_pre_proc_arg
 * @param reading pointer to reading from previous step
 * @retval true if setting mux and page is successful.
 * @retval false if setting mux or page fails.
 */
bool pre_isl69259_read(uint8_t sensor_num, void *args)
{
	if (!args) {
		return false;
	}
	isl69259_pre_proc_arg *pre_proc_args = (isl69259_pre_proc_arg *)args;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	if (!pre_i2c_bus_read(sensor_num, pre_proc_args->mux_info_p)) {
		printf("[%s] pre_i2c_bus_read fail \n", __func__);
		return false;
	}

	/* set page */
	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = pre_proc_args->vr_page;
	if (i2c_master_write(&msg, retry)) {
		printf("pre_isl69259_read, set page fail\n");
		k_mutex_unlock(&i2c_bus6_mutex);
		return false;
	}
	return true;
}

bool pre_pex89000_read(uint8_t sensor_num, void *args)
{
	if (!args) {
		return false;
	}
	pex89000_pre_proc_arg *pre_read_args = (pex89000_pre_proc_arg *)args;

	/* Can not access i2c mux and PEX89000 when DC off, this pin will change to low active in 
      next SWB CPLD firmware release */
	if (!gpio_get(SYS_PWR_READY_N))
		return false;

	if (!pre_i2c_bus_read(sensor_num, pre_read_args->mux_info_p)) {
		printf("[%s] pre_i2c_bus_read fail \n", __func__);
		return false;
	}
	return true;
}

bool pre_i2c_bus_read(uint8_t sensor_num, void *args)
{
	if (!args)
		return false;

	struct k_mutex *mutex = find_bus_mutex(sensor_num);

	if (!mutex)
		return false;

	if (k_mutex_lock(mutex, K_MSEC(10))) {
		printf("[%s] sensor number 0x%x mutex lock fail\n", __func__, sensor_num);
		return false;
	}

	if (!tca9548_select_chan(sensor_num, (struct tca9548 *)args)) {
		k_mutex_unlock(mutex);
		return false;
	}

	return true;
}

bool post_i2c_bus_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(args);
	ARG_UNUSED(reading);

	struct k_mutex *mutex = find_bus_mutex(sensor_num);

	if (!mutex)
		return false;

	if (k_mutex_unlock(mutex)) {
		printf("[%s] sensor num 0x%x mutex unlock failed!\n", __func__, sensor_num);
		return false;
	}
	return true;
}

struct k_mutex *find_bus_mutex(uint8_t sensor_num)
{
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	struct k_mutex *mutex = NULL;

	if (cfg->port == I2C_BUS6)
		mutex = &i2c_bus6_mutex;
	else if (cfg->port == I2C_BUS9)
		mutex = &i2c_bus9_mutex;
	else if (cfg->port == I2C_BUS10)
		mutex = &i2c_bus10_mutex;
	else
		return NULL;

	return mutex;
}