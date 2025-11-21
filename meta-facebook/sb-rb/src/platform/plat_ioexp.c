#include <logging/log.h>
#include "pldm_sensor.h"
#include "plat_i2c.h"
#include "plat_util.h"
#include "plat_ioexp.h"

LOG_MODULE_REGISTER(plat_ioexp);

#define PCA6414A_BUS I2C_BUS1
#define PCA6414A_ADDR (0x40 >> 1)

bool pca6416a_i2c_read(uint8_t offset, uint8_t *data, uint8_t len)
{
	return plat_i2c_read(PCA6414A_BUS, PCA6414A_ADDR, offset, data, len);
}

bool pca6416a_i2c_write(uint8_t offset, uint8_t *data, uint8_t len)
{
	return plat_i2c_write(PCA6414A_BUS, PCA6414A_ADDR, offset, data, len);
}

bool pca6416a_init(void)
{
	uint8_t data[2] = { 0x00, 0x00 }; // all output
	if (!pca6416a_i2c_write(PCA6414A_CONFIG_0, data, 2))
		return false;

	return true;
}