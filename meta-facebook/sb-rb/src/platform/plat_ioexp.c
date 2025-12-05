#include <logging/log.h>
#include "pldm_sensor.h"
#include "plat_i2c.h"
#include "plat_util.h"
#include "plat_ioexp.h"
#include "plat_class.h"
#include "plat_pldm_sensor.h"

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

	//set HAMSA_MFIO19(VR_HOT_EVB_BIT) to default 0
	if (!tca6424a_i2c_write_bit(TCA6424A_OUTPUT_PORT_0, HAMSA_MFIO19, 0)) {
		return false;
	}

	return true;
}

// total
void ioexp_init(void)
{
	if (!pca6416a_init())
		LOG_ERR("pca6416a init fail");

	if (get_asic_board_id() == ASIC_BOARD_ID_EVB) {
		if (!tca6424a_init())
			LOG_ERR("tca6424a init fail");
	}
}
void set_pca6554apw_ioe_value(uint8_t ioe_bus, uint8_t ioe_addr, uint8_t ioe_reg, uint8_t value)
{
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	int ret = 0;
	msg.bus = ioe_bus;
	msg.target_addr = ioe_addr;
	msg.tx_len = 2;
	msg.data[0] = ioe_reg;
	msg.data[1] = value;

	ret = i2c_master_write(&msg, retry);

	if (ret != 0) {
		LOG_ERR("Failed to write IOE(0x%02X). The register is 0x%02X.", ioe_addr, ioe_reg);
		k_msleep(1000);
	}
}

int get_pca6554apw_ioe_value(uint8_t ioe_bus, uint8_t ioe_addr, uint8_t ioe_reg, uint8_t *value)
{
	int ret = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = ioe_bus;
	msg.target_addr = ioe_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = ioe_reg;

	ret = i2c_master_read(&msg, retry);

	if (ret != 0) {
		LOG_ERR("Failed to read IOE(0x%02X). The register is 0x%02X.", ioe_addr, ioe_reg);
		k_msleep(1000);
		return -1;
	}

	*value = msg.data[0];

	return 0;
}

//evb only
void init_U200052_IO()
{
	LOG_INF("init U200052 IO expander");
	// bit0 to bit5 is input (1)
	set_pca6554apw_ioe_value(U200052_IO_I2C_BUS, U200052_IO_ADDR, CONFIG, 0x3F);
	// io6,io7 default output 0
	set_pca6554apw_ioe_value(U200052_IO_I2C_BUS, U200052_IO_ADDR, OUTPUT_PORT, 0x0);
	LOG_INF("init U200070 IO expander");
	// bit3 to bit5 is input (1)
	set_pca6554apw_ioe_value(U200070_IO_I2C_BUS, U200070_IO_ADDR, CONFIG, 0x38);
	// io0,io1,io2,io7 default output 0
	set_pca6554apw_ioe_value(U200070_IO_I2C_BUS, U200070_IO_ADDR, OUTPUT_PORT, 0x0);
}