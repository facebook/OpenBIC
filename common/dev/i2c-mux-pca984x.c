#include <stdio.h>
#include <logging/log.h>
#include "sensor.h"
#include "libutil.h"
#include "hal_i2c.h"
#include "i2c-mux-pca984x.h"

LOG_MODULE_REGISTER(i2c_mux_pca9846);

bool set_pca9846_channel_and_transfer(uint8_t bus, uint8_t mux_addr, uint8_t mux_channel,
				      uint8_t tran_type, I2C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);

	bool ret = true;
	int status = 0;
	int retry = 5;

	/* Set channel */
	I2C_MSG mux_msg = { 0 };
	mux_msg.bus = bus;
	mux_msg.target_addr = mux_addr;
	mux_msg.tx_len = 1;
	mux_msg.data[0] = mux_channel;

	/* Mutex lock */
	status = k_mutex_lock(&msg->lock, K_MSEC(PCA9846_MUTEX_LOCK_MS));
	if (status != 0) {
		LOG_ERR("mutex lock fail, status: %d, bus: %d, mux addr: 0x%x\n", status, bus,
			mux_addr);
		return false;
	}

	status = i2c_master_write(&mux_msg, retry);
	if (status != 0) {
		LOG_ERR("set channel fail, status: %d\n", status);
		ret = false;
		goto mutex_unlock;
	}

	/* Transfer via i2c */
	switch (tran_type) {
	case I2C_READ:
		status = i2c_master_read(msg, retry);
		break;
	case I2C_WRITE:
		status = i2c_master_write(msg, retry);
		break;
	default:
		LOG_ERR("transfer type is invalid, transfer type: %d\n", tran_type);
		ret = false;
		goto mutex_unlock;
	}

	if (status != 0) {
		LOG_ERR("transfer msg fail, status: %d\n", status);
		ret = false;
		goto mutex_unlock;
	}

	/* Disable all channels */
	mux_msg.bus = bus;
	mux_msg.target_addr = mux_addr;
	mux_msg.tx_len = 1;
	mux_msg.data[0] = PCA9846_DEFAULT_CHANNEL;

	status = i2c_master_write(&mux_msg, retry);
	if (status != 0) {
		LOG_ERR("disable all channels fail, status: %d\n", status);
		ret = false;
	}

mutex_unlock:
	status = k_mutex_unlock(&msg->lock);
	if (status != 0) {
		LOG_ERR("mutex unlock fail, status: %d\n", status);
		ret = false;
	}

	return ret;
}
