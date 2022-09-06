#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "plat_hook.h"
#include "plat_sensor_table.h"
#include "i2c-mux-tca9548.h"

#define ADJUST_ADM1278_CURRENT(x) (x * 0.94)
#define ADJUST_ADM1278_POWER(x) (x * 0.95)

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
adc_asd_init_arg ast_adc_init_args[] = { [0] = { .is_init = false } };

adm1278_init_arg adm1278_init_args[] = {
	[0] = { .is_init = false, .config = { 0x3F1C }, .r_sense = 0.3 }
};

ltc4282_init_arg ltc4282_init_args[] = { [0] = { .r_sense_mohm = 0.5 } };

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
struct tca9548 mux_conf_addr_0xe2[] = {
	[0] = { .addr = 0xe2, .chan = 0 }, [1] = { .addr = 0xe2, .chan = 1 },
	[2] = { .addr = 0xe2, .chan = 2 }, [3] = { .addr = 0xe2, .chan = 3 },
	[4] = { .addr = 0xe2, .chan = 4 }, [5] = { .addr = 0xe2, .chan = 5 },
	[6] = { .addr = 0xe2, .chan = 6 }, [7] = { .addr = 0xe2, .chan = 7 },
};

vr_pre_proc_arg vr_pre_read_args[] = {
	[0] = { 0x0 },
	[1] = { 0x1 },
};

apml_mailbox_init_arg apml_mailbox_init_args[] = {
	[0] = { .data = 0x00000000, .retry = 0 },  [1] = { .data = 0x00000002, .retry = 0 },
	[2] = { .data = 0x00000001, .retry = 0 },  [3] = { .data = 0x00000003, .retry = 0 },
	[4] = { .data = 0x00000010, .retry = 0 },  [5] = { .data = 0x00000012, .retry = 0 },
	[6] = { .data = 0x00000011, .retry = 0 },  [7] = { .data = 0x00000013, .retry = 0 },
	[8] = { .data = 0x00000000, .retry = 0 },  [9] = { .data = 0x00000002, .retry = 0 },
	[10] = { .data = 0x00000001, .retry = 0 }, [11] = { .data = 0x00000003, .retry = 0 },
	[12] = { .data = 0x00000010, .retry = 0 }, [13] = { .data = 0x00000012, .retry = 0 },
	[14] = { .data = 0x00000011, .retry = 0 }, [15] = { .data = 0x00000013, .retry = 0 },
	[16] = { .data = 0x00000000, .retry = 0 },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/

bool pre_nvme_read(uint8_t sensor_num, void *args)
{
	if (!args) {
		return false;
	}
	return tca9548_select_chan(sensor_num, (struct tca9548 *)args);
}

bool pre_vr_read(uint8_t sensor_num, void *args)
{
	if (args == NULL) {
		return false;
	}

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)args;
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	uint8_t retry = 5;
	I2C_MSG msg;

	/* set page */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = pre_proc_args->vr_page;
	if (i2c_master_write(&msg, retry)) {
		printf("[%s] set page fail\n", __func__);
		return false;
	}
	return true;
}

bool pre_vol_bat3v_read(uint8_t sensor_num, void *args)
{
	ARG_UNUSED(args);

	if (sensor_num == SENSOR_NUM_VOL_P3V_BAT) {
		gpio_set(P3V_BAT_SCALED_EN_R, GPIO_HIGH);
		k_msleep(1);
	}

	return true;
}

bool post_vol_bat3v_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(args);

	if (sensor_num == SENSOR_NUM_VOL_P3V_BAT) {
		gpio_set(P3V_BAT_SCALED_EN_R, GPIO_LOW);
		k_msleep(1);
	}

	return true;
}

bool post_adm1278_cur_read(uint8_t sensor_num, void *args, int *reading)
{
	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);
	val = ADJUST_ADM1278_CURRENT(val);
	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}

bool post_adm1278_pwr_read(uint8_t sensor_num, void *args, int *reading)
{
	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);
	val = ADJUST_ADM1278_POWER(val);
	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}
