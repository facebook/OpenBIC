#include <logging/log.h>
#include "pldm_sensor.h"
#include "plat_i2c.h"
#include "plat_util.h"
#include "plat_ioexp.h"
#include "plat_class.h"
#include "plat_pldm_sensor.h"
#include "plat_hook.h"

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
bool is_tca6424a_accessible()
{
	return (get_asic_board_id() == ASIC_BOARD_ID_EVB && is_mb_dc_on());
}

bool tca6424a_i2c_read(uint8_t offset, uint8_t *data, uint8_t len)
{
	return plat_i2c_read(TCA6424A_BUS, TCA6424A_ADDR, (offset | TCA6424A_AI_BIT), data, len);
}

bool tca6424a_i2c_read_drive_value(uint8_t group, uint8_t bit, uint8_t *data)
{
	uint8_t value = 0;
	uint8_t confg_reg = 0;
	uint8_t input_reg = 0;
	uint8_t output_reg = 0;

	if (group == 0) {
		confg_reg = TCA6424A_CONFIG_0;
		input_reg = TCA6424A_INPUT_PORT_0;
		output_reg = TCA6424A_OUTPUT_PORT_0;
	} else if (group == 1) {
		confg_reg = TCA6424A_CONFIG_1;
		input_reg = TCA6424A_INPUT_PORT_1;
		output_reg = TCA6424A_OUTPUT_PORT_1;
	} else if (group == 2) {
		confg_reg = TCA6424A_CONFIG_2;
		input_reg = TCA6424A_INPUT_PORT_2;
		output_reg = TCA6424A_OUTPUT_PORT_2;
	} else {
		LOG_ERR("Wrong group!");
		return false;
	}

	if (!tca6424a_i2c_read(confg_reg, &value, 1)) {
		return false;
	}

	if (value & BIT(bit)) {
		if (!tca6424a_i2c_read(input_reg, &value, 1)) {
			return false;
		}
	} else {
		if (!tca6424a_i2c_read(output_reg, &value, 1)) {
			return false;
		}
	}
	*data = (value & BIT(bit)) >> bit;

	return true;
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

void set_hamsa_mfio_6_8_10_input()
{
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_1, HAMSA_MFIO6, 1);
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_1, HAMSA_MFIO8, 1);
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_2, HAMSA_MFIO10, 1);
}

void set_medha0_mfio_6_8_10_input()
{
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_2, MEDHA0_MFIO6, 1);
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_2, MEDHA0_MFIO8, 1);
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_2, MEDHA0_MFIO10, 1);
}

void set_medha1_mfio_6_8_10_input()
{
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_2, MEDHA1_MFIO6, 1);
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_2, MEDHA1_MFIO8, 1);
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_2, MEDHA1_MFIO10, 1);
}

void set_hamsa_mfio_6_8_10_output()
{
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_1, HAMSA_MFIO6, 0);
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_1, HAMSA_MFIO8, 0);
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_2, HAMSA_MFIO10, 0);
}

void set_medha0_mfio_6_8_10_output()
{
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_2, MEDHA0_MFIO6, 0);
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_2, MEDHA0_MFIO8, 0);
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_2, MEDHA0_MFIO10, 0);
}

void set_medha1_mfio_6_8_10_output()
{
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_2, MEDHA1_MFIO6, 0);
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_2, MEDHA1_MFIO8, 0);
	tca6424a_i2c_write_bit(TCA6424A_CONFIG_2, MEDHA1_MFIO10, 0);
}

bool tca6424a_init(void)
{
	uint8_t data[3] = { 0xFD, 0xFF, 0xFD }; // HAMSA_MFIO19 out, OWL_EW_VQPS out
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
	// read from IO, save value (if output) to table
	set_ioexp_val_to_bootstrap_table();

	if (!pca6416a_init())
		LOG_ERR("pca6416a init fail");

	if (is_tca6424a_accessible()) {
		if (!tca6424a_init())
			LOG_ERR("tca6424a init fail");
	}

	// set to output if TEST_STRAP enable
	int drive_level = 0;
	get_bootstrap_change_drive_level(STRAP_INDEX_HAMSA_TEST_STRAP_R, &drive_level);
	if (drive_level == 1) {
		set_hamsa_mfio_6_8_10_output();
	}
	get_bootstrap_change_drive_level(STRAP_INDEX_MEDHA0_TEST_STRAP, &drive_level);
	if (drive_level == 1) {
		set_medha0_mfio_6_8_10_output();
	}
	get_bootstrap_change_drive_level(STRAP_INDEX_MEDHA1_TEST_STRAP, &drive_level);
	if (drive_level == 1) {
		set_medha1_mfio_6_8_10_output();
	}

	// set value from table
	set_bootstrap_table_val_to_ioexp();
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
	set_pca6554apw_ioe_value(U200052_IO_I2C_BUS, U200052_IO_ADDR, OUTPUT_PORT,
				 U200052_IO_INIT_VAL);
}
//evb only
void init_U200053_IO()
{
	LOG_INF("init U200053 IO expander");
	// bit0 to bit5 is input (1)
	set_pca6554apw_ioe_value(U200053_IO_I2C_BUS, U200053_IO_ADDR, CONFIG, 0xBF);
	// io6 default output 1
	set_pca6554apw_ioe_value(U200053_IO_I2C_BUS, U200053_IO_ADDR, OUTPUT_PORT,
				 U200053_IO_INIT_VAL);
}
//evb2 only
void init_U200070_IO()
{
	LOG_INF("init U200070 IO expander");
	// bit3 to bit5 is input (1)
	set_pca6554apw_ioe_value(U200070_IO_I2C_BUS, U200070_IO_ADDR, CONFIG, 0x38);
	// io0,io1,io2 default output 1 io7 default output 0
	set_pca6554apw_ioe_value(U200070_IO_I2C_BUS, U200070_IO_ADDR, OUTPUT_PORT,
				 U200070_IO_INIT_VAL);
}
