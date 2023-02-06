#include <logging/log.h>
#include <drivers/peci.h>

#include "sensor.h"
#include "intel_peci.h"
#include "intel_dimm.h"
#include "power_status.h"
#include "i2c-mux-tca9548.h"
#include "libutil.h"
#include "hal_peci.h"

#include "plat_hook.h"
#include "plat_sensor_table.h"
#include "plat_gpio.h"

#define RDPKG_IDX_DIMM_TEMP 0x0E

LOG_MODULE_REGISTER(plat_hook);

xdpe15284_pre_read_arg xdpe15284_pre_read_args[] = {
	[0] = { 0x0 },
	[1] = { 0x1 },
};

adc_asd_init_arg adc_asd_init_args[] = { [0] = { .is_init = false } };
adm1278_init_arg adm1278_init_args[] = {
	[0] = { .is_init = false, .config = { 0x3F1C }, .r_sense = 0.25 }
};

struct tca9548 mux_conf_addr_0xe2[8] = {
	[0] = { .addr = 0xe2, .chan = 0 }, [1] = { .addr = 0xe2, .chan = 1 },
	[2] = { .addr = 0xe2, .chan = 2 }, [3] = { .addr = 0xe2, .chan = 3 },
	[4] = { .addr = 0xe2, .chan = 4 }, [5] = { .addr = 0xe2, .chan = 5 },
	[6] = { .addr = 0xe2, .chan = 6 }, [7] = { .addr = 0xe2, .chan = 7 },
};

bool pre_xdpe15284_read(uint8_t sensor_num, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	xdpe15284_pre_read_arg *pre_read_args = (xdpe15284_pre_read_arg *)args;
	uint8_t retry = 5;
	I2C_MSG msg;

	/* set page */
	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = pre_read_args->vr_page;
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Failed to set page");
		return false;
	}
	return true;
}

bool pre_vol_bat3v_read(uint8_t sensor_num, void *args)
{
	ARG_UNUSED(args);

	if (sensor_num == SENSOR_NUM_MB_ADC_P3V_BAT_VOLT_V) {
		gpio_set(A_P3V_BAT_SCALED_EN_R, GPIO_HIGH);
		k_msleep(1);
	}

	return true;
}

bool post_vol_bat3v_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(args);
	ARG_UNUSED(reading);

	if (sensor_num == SENSOR_NUM_MB_ADC_P3V_BAT_VOLT_V)
		gpio_set(A_P3V_BAT_SCALED_EN_R, GPIO_LOW);

	return true;
}

bool pre_nvme_read(uint8_t sensor_num, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	if (!tca9548_select_chan(sensor_num, (struct tca9548 *)args))
		return false;

	return true;
}
bool post_cpu_margin_read(uint8_t sensor_num, void *args, int *reading)
{
	if (!reading)
		return check_reading_pointer_null_is_allowed(sensor_num);
	ARG_UNUSED(args);

	sensor_val *sval = (sensor_val *)reading;
	// The margin sensor should be shown as negative value in BMC.
	sval->integer = -sval->integer;
	return true;
}

bool pre_intel_peci_dimm_read(uint8_t sensor_num, void *args)
{
	if (get_post_status() == false) {
		return true;
	}

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	const uint8_t read_type = cfg->offset;
	//Check DIMM Channel
	if (read_type <= PECI_UNKNOWN || read_type >= PECI_MAX) {
		LOG_DBG("Sensor not found");
		return false;
	}

	uint16_t dimm_channel = 0xFF;
	const uint8_t rlen = 0x05;
	uint8_t rbuf[rlen];
	memset(rbuf, 0, sizeof(rbuf));

	switch (read_type) {
	case PECI_TEMP_CHANNEL0_DIMM0:
	case PECI_TEMP_CHANNEL0_DIMM1:
		dimm_channel = 0x00;
		break;
	case PECI_TEMP_CHANNEL1_DIMM0:
	case PECI_TEMP_CHANNEL1_DIMM1:
		dimm_channel = 0x01;
		break;
	case PECI_TEMP_CHANNEL2_DIMM0:
	case PECI_TEMP_CHANNEL2_DIMM1:
		dimm_channel = 0x02;
		break;
	case PECI_TEMP_CHANNEL3_DIMM0:
	case PECI_TEMP_CHANNEL3_DIMM1:
		dimm_channel = 0x03;
		break;
	case PECI_TEMP_CHANNEL4_DIMM0:
	case PECI_TEMP_CHANNEL4_DIMM1:
		dimm_channel = 0x04;
		break;
	case PECI_TEMP_CHANNEL5_DIMM0:
	case PECI_TEMP_CHANNEL5_DIMM1:
		dimm_channel = 0x05;
		break;
	case PECI_TEMP_CHANNEL6_DIMM0:
	case PECI_TEMP_CHANNEL6_DIMM1:
		dimm_channel = 0x06;
		break;
	case PECI_TEMP_CHANNEL7_DIMM0:
	case PECI_TEMP_CHANNEL7_DIMM1:
		dimm_channel = 0x07;
		break;
	default:
		break;
	}

	//Check DIMM is present. DIMM is absent if return code is 0x90.
	if (peci_read(PECI_CMD_RD_PKG_CFG0, cfg->target_addr, RDPKG_IDX_DIMM_TEMP, dimm_channel,
		      rlen, rbuf) == 0x90) {
		/* TODO:
		 * Disable PMIC sensor polling if the DIMM is absent.
		 */

		return false;
	}

	return true;
}
