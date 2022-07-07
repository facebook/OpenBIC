#ifndef I2C_MUX_PCA984X_H
#define I2C_MUX_PCA984X_H
#include "hal_i2c.h"

#define I2C_MUX_RETRY_INTERVAL 1
#define I2C_MUX_PCA9846_DEFAULT_CHANNEL 0x00

/* i2c-mux pca9846 */
bool i2c_mux_pca9846_lock(I2C_MSG *msg);
bool i2c_mux_pca9846_unlock(I2C_MSG *msg);

#endif
