#ifndef I2C_MUX_PCA984X_H
#define I2C_MUX_PCA984X_H

#define PCA9846_MUTEX_LOCK_MS 1000
#define PCA9846_DEFAULT_CHANNEL 0

enum PCA9846_CHANNEL {
	PCA9846_CHANNEL_0 = BIT(0),
	PCA9846_CHANNEL_1 = BIT(1),
	PCA9846_CHANNEL_2 = BIT(2),
	PCA9846_CHANNEL_3 = BIT(3),
};

bool set_pca9846_channel_and_transfer(uint8_t bus, uint8_t mux_addr, uint8_t mux_channel,
				      uint8_t tran_type, I2C_MSG *msg);

#endif
