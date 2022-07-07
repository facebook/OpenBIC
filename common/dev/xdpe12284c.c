#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include "util_pmbus.h"

enum {
	VR12 = 1,
	VR13,
	IMVP9,
};

enum {
	VID_IDENTIFIER = 1,
};

/*  Reference: Infineon spec section 8.24: VID table
 *  PMBUS spec section 8.2: VOUT mode
 */
static float vid_to_float(int val, uint8_t vout_mode)
{
	uint8_t mode = 0;
	uint8_t parameter = 0;

	//VID 0 is always 0 V
	if (val == 0) {
		return 0;
	}

	mode = (vout_mode >> 5);

	if (mode != VID_IDENTIFIER) {
		printf("%s Infineon VR reading with invalid VID IDENTIFIER: %x", __func__, mode);
		return -1;
	}

	switch (vout_mode & 0x1f) {
	case VR12:
		if (val > 0) {
			return ((val - 1) * 5 + 250);
		}
	case VR13:
		if (val > 0) {
			return ((val - 1) * 10 + 500);
		}
	case IMVP9:
		if (val > 0) {
			return ((val - 1) * 10 + 200);
		}
	default:
		printf("%s Infineon VR reading with invalid vout mode: %x", __func__, vout_mode);
		return -1;
	}

	return 0;
}

uint8_t xdpe12284c_read(uint8_t sensor_num, int *reading)
{
	if (reading == NULL || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	int val = 0;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));
	float actual_value = 0;
	uint8_t offset = sensor_config[sensor_config_index_map[sensor_num]].offset;

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = offset;

	if (i2c_master_read(&msg, retry)) {
		return SENSOR_FAIL_TO_ACCESS;
	}

	val = (msg.data[1] << 8) | msg.data[0];

	switch (offset) {
	case PMBUS_READ_IOUT:
	case PMBUS_READ_POUT:
	case PMBUS_READ_TEMPERATURE_1:
		actual_value = slinear11_to_float(val);

		if (offset == PMBUS_READ_IOUT || offset == PMBUS_READ_POUT) {
			if (actual_value < 0) {
				return SENSOR_FAIL_TO_ACCESS;
			}
		}

		sval->integer = actual_value;
		sval->fraction = (actual_value - sval->integer) * 1000;
		break;
	case PMBUS_READ_VOUT:
		msg.tx_len = 1;
		msg.rx_len = 1;
		msg.data[0] = PMBUS_VOUT_MODE;

		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}

		actual_value = vid_to_float(val, msg.data[0]);
		if (actual_value < 0) {
			return SENSOR_FAIL_TO_ACCESS;
		}

		sval->integer = actual_value;
		sval->fraction = (actual_value - sval->integer) * 1000;
		break;
	default:
		return SENSOR_FAIL_TO_ACCESS;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t xdpe12284c_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = xdpe12284c_read;
	return SENSOR_INIT_SUCCESS;
}
