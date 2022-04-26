#include <stdio.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "i2c-mux-tca9548.h"

bool tca9548_select_chan(uint8_t sensor_num, void *args)
{
	if (!args || (sensor_num > SENSOR_NUM_MAX)) {
		return false;
	}

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	struct tca9548 *p = (struct tca9548 *)args;

	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	/* change address to 7-bit */
	msg.target_addr = ((p->addr) >> 1);
	msg.tx_len = 1;
	msg.data[0] = (1 << (p->chan));

	if (i2c_master_write(&msg, retry))
		return false;

	return true;
}
