#include <stdio.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "i2c-mux-pca984x.h"

/** 
 * PCA9846 usage:
 * PCA9846 only recieves the last byte unless special condition.
 * The byte data[0] refer to below bit-map; 1 = Enable, 0 = Disable
 *     Bit 7 6 5 4 3 2 1 0
 * Channel X X X X 3 2 1 0
 * 
 * Example:
 * File1: plat_i2c.c
 * K_MUTEX_DEFINE(i2c_bus9_mutex);
 * 
 * File2: plat_i2c.h
 * extern struct k_mutex i2c_bus9_mutex;
 *
 * File3: Application
 * I2C_MSG *msg;
 * msg->data[0] = 0x1;	// Enable Channel 0 only
 * msg->data[0] = 0x3;	// Enable Channel 0 + 1
 * msg->data[0] = 0x4;	// Enable Channel 2 only
 * msg->data[0] = 0x8;	// Enable Channel 3 only
 * msg->lock = &i2c_bus9_mutex;
 * i2c_mux_pca9846_lock(msg);
 * // Accessing device code here //
 * i2c_mux_pca9846_unlock(msg);
 */

/**
 * @brief lock the specific mutex then enable mux channel.
 * 
 * Check lock_count before enable channel.
 * Set 5 seconds timeout if wait for unlock failed.
 * After timeout, the mutex will be force re-init to unlock for preventing infinte lock.
 * Once lock the mutex, switch the channel based on i2c data.
 * 
 * @param msg                   i2c message structure.
 * @param msg->bus              bus number where mux locate.
 * @param msg->target_addr      slave addr where mux locate.
 * @param msg->lock             the mutex addr for kernel reference.
 * @param msg->lock->lock_count number of lock using
 * 
 * @retval true  mutex lock and enable mux channel successfully.
 * @retval false mutex lock or i2c send failed.
 */
bool i2c_mux_pca9846_lock(I2C_MSG *msg)
{
	if (!msg) {
		printf("[%s] Received null pointer\n", __func__);
		return false;
	}

	uint8_t retry = 3;
	for (uint8_t i = 1; i <= retry; ++i) {
		if (msg->lock->lock_count) {
			k_sleep(K_SECONDS(I2C_MUX_RETRY_INTERVAL));
			if (i >= retry) {
				printf("[%s] Failed to wait for unlock, retry: %u times in %u seconds. Force re-init the mutex\n",
				       __func__, retry, I2C_MUX_RETRY_INTERVAL * retry);
				k_mutex_init(msg->lock);
			}
		} else {
			break;
		}
	}

	/* k_mutex_lock return zero or -ERRNO */
	if (k_mutex_lock(msg->lock, K_NO_WAIT)) {
		/* Re-init Mutex for preventing infinite lock */
		printf("[%s] Failed to lock bus%u mux0x%x\n", __func__, msg->bus, msg->target_addr);
		return false;
	}

	if (i2c_master_write(msg, retry)) {
		printf("[%s] i2c sent failed\n", __func__);
		return false;
	}

	return true;
}

/**
 * @brief disable all mux channels then unlock the mutex
 * 
 * Function disable all mux channels directly.
 * No matter the disable result, it will unlock the mutex.
 * Unlock can be failed by incorrect thread owner or lock_count already zero.
 * 
 * @param msg              i2c message structure
 * @param msg->bus         bus number where mux locate
 * @param msg->target_addr slave addr where mux locate
 * @param msg->lock        the mutex addr for kernel reference
 * 
 * @retval true  mutex unlock successfully
 * @retval false mutex unlock failed
 */
bool i2c_mux_pca9846_unlock(I2C_MSG *msg)
{
	if (!msg) {
		printf("[%s] Received null pointer\n", __func__);
		return false;
	}

	/* Disable all channels */
	uint8_t retry = 5;
	msg->data[0] = I2C_MUX_PCA9846_DEFAULT_CHANNEL;
	if (i2c_master_write(msg, retry)) {
		printf("[%s] i2c sent failed, mux did not disable all channels\n", __func__);
	}

	/* k_mutex_unlock return zero or -ERRNO */
	if (k_mutex_unlock(msg->lock)) {
		printf("[%s] Failed to unlock bus%u mux0x%x\n", __func__, msg->bus,
		       msg->target_addr);
		return false;
	}

	return true;
}
