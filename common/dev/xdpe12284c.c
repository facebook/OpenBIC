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

bool xdpe12284c_get_checksum(uint8_t bus, uint8_t target_addr, uint8_t *checksum)
{
	if (checksum == NULL) {
		printf("<error> XDPE12284C checksum is NULL\n");
		return false;
	}

	I2C_MSG i2c_msg;
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = target_addr;
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = 0x00;
	i2c_msg.data[1] = 0x62; //set page to 0x62

	if (i2c_master_write(&i2c_msg, retry)) {
		printf("<error> XDPE12284C get checksum while set page\n");
		return false;
	}

	//Read lower word for the 32bit checksum value
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = 0x43;
	if (i2c_master_read(&i2c_msg, retry)) {
		printf("<error> XDPE12284C get checksum while i2c reading\n");
		return false;
	}

	checksum[0] = i2c_msg.data[1];
	checksum[1] = i2c_msg.data[0];

	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = 0x00;
	i2c_msg.data[1] = 0x62; //set page to 0x62

	if (i2c_master_write(&i2c_msg, retry)) {
		printf("<error> XDPE12284C get checksum while set page\n");
		return false;
	}

	//Read higher word for the 32bit checksum value
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = 0x42;
	if (i2c_master_read(&i2c_msg, retry)) {
		printf("<error> XDPE12284C get checksum while i2c reading\n");
		return false;
	}

	checksum[2] = i2c_msg.data[1];
	checksum[3] = i2c_msg.data[0];

	return true;
}

bool xdpe12284c_get_remaining_write(uint8_t bus, uint8_t target_addr, uint8_t *remain_write)
{
	I2C_MSG i2c_msg;
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = target_addr;
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = 0x00;
	i2c_msg.data[1] = 0x50; //set page to 0x50

	if (i2c_master_write(&i2c_msg, retry)) {
		printf("<error> XDPE12284C get remaining write while i2c writing\n");
		return false;
	}

	//Read the remaining writes from register address 0x82
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = 0x82;
	if (i2c_master_read(&i2c_msg, retry)) {
		printf("<error> XDPE12284C get remaining write while i2c reading\n");
		return false;
	}

	//the data residing in bit11~bit6 is the number of the remaining writes.
	*remain_write = (((i2c_msg.data[1] << 8) | i2c_msg.data[0]) & 0xFC0) >> 6;
	return true;
}

/*  Reference: Infineon spec section 8.24: VID table
 *  PMBUS spec section 8.2: VOUT mode
 */
static float vid_to_float(int val, uint8_t vout_mode)
{
	uint8_t mode = 0;

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

		actual_value /= 1000; // mV to V
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
