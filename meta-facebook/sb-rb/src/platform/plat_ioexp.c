#include <logging/log.h>
#include "pldm_sensor.h"
#include "plat_i2c.h"
#include "plat_util.h"
#include "plat_ioexp.h"

LOG_MODULE_REGISTER(plat_ioexp);

#define PCA6414A_BUS I2C_BUS1
#define PCA6414A_ADDR (0x40 >> 1)

#define TCA6424A_BUS I2C_BUS1
#define TCA6424A_ADDR (0x44 >> 1)
#define TCA6424A_AI_BIT BIT(7) // Auto-Increment bit

// pca6414a
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

// tca6424a
bool tca6424a_i2c_read(uint8_t offset, uint8_t *data, uint8_t len)
{
	return plat_i2c_read(TCA6424A_BUS, TCA6424A_ADDR, (offset | TCA6424A_AI_BIT), data, len);
}

bool tca6424a_i2c_write(uint8_t offset, uint8_t *data, uint8_t len)
{
	return plat_i2c_write(TCA6424A_BUS, TCA6424A_ADDR, (offset | TCA6424A_AI_BIT), data, len);
}

bool tca6424a_i2c_write_bit(uint8_t offset, uint8_t bit, uint8_t val)
{
	uint8_t current_val = 0;
	uint8_t new_val;

	if (!plat_i2c_read(TCA6424A_BUS, TCA6424A_ADDR, (offset | TCA6424A_AI_BIT), &current_val,
			   1))
		return false;

	if (val)
		new_val = current_val | (1 << bit);
	else
		new_val = current_val & ~(1 << bit);

	if (new_val == current_val)
		return true;

	if (!plat_i2c_write(TCA6424A_BUS, TCA6424A_ADDR, (offset | TCA6424A_AI_BIT), &new_val, 1))
		return false;

	return true;
}

bool tca6424a_init(void)
{
	uint8_t data[3] = { 0x00, 0x00, 0x00 }; // all output
	if (!tca6424a_i2c_write(TCA6424A_CONFIG_0, data, 3))
		return false;

	return true;
}

// total
void ioexp_init(void)
{
	if (!pca6416a_init())
		LOG_ERR("pca6416a init fail");
	if (!tca6424a_init())
		LOG_ERR("tca6424a init fail");
}