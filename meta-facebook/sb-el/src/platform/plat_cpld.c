#include "plat_cpld.h"
#include "libutil.h"
#include "plat_i2c.h"
#include "plat_util.h"
#include "plat_gpio.h"
#include "plat_log.h"
#include "plat_hook.h"
#include "plat_event.h"
#include "plat_kernel_obj.h"
#include "plat_led.h"
#include <logging/log.h>

#define CPLD_ADDR (0x4C >> 1)
#define I2C_BUS_CPLD I2C_BUS11

#define POLLING_CPLD_STACK_SIZE 2048

#define CHECK_ALL_BITS 0xFF
#define CHECK_BITS_6 0x40
#define CHECK_BITS_678 0xE0
#define CHECK_BITS_78 0xC0
#define CHECK_BITS_8 0x80

LOG_MODULE_REGISTER(plat_cpld);

bool plat_read_cpld(uint8_t offset, uint8_t *data, uint8_t len)
{
	return plat_i2c_read(I2C_BUS_CPLD, CPLD_ADDR, offset, data, len);
}

bool plat_write_cpld(uint8_t offset, uint8_t *data)
{
	return plat_i2c_write(I2C_BUS_CPLD, CPLD_ADDR, offset, data, 1);
}

bool set_cpld_bit(uint8_t cpld_offset, uint8_t bit, uint8_t value)
{
	if (bit > 8) {
		LOG_ERR("Invalid bit index %d", bit);
		return false;
	}

	if (value != 0 && value != 1) {
		LOG_ERR("Invalid value %d", value);
		return false;
	}

	uint8_t original_value = 0;
	if (!plat_read_cpld(cpld_offset, &original_value, 1)) {
		LOG_ERR("offset = 0x%x, bit = %d, value = %d, read cpld fail", cpld_offset, bit,
			value);
		return false;
	}

	if (value) {
		original_value |= BIT(bit);
	} else {
		original_value &= ~BIT(bit);
	}

	if (!plat_write_cpld(cpld_offset, &original_value)) {
		LOG_ERR("offset = 0x%x, bit = %d, value = %d, write cpld fail", cpld_offset, bit,
			value);
		return false;
	}

	// check if write success
	uint8_t check_value = 0;
	if (!plat_read_cpld(cpld_offset, &check_value, 1)) {
		LOG_ERR("offset = 0x%x, bit = %d, value = %d, read cpld fail", cpld_offset, bit,
			value);
		return false;
	}

	LOG_DBG("original_value = 0x%x, check_value = 0x%x", original_value, check_value);

	if (check_value != original_value) {
		LOG_ERR("offset = 0x%x, bit = %d, value = %d, set_cpld_bit fail", cpld_offset, bit,
			value);
		return false;
	}

	return true;
}

// cpld polling
void check_cpld_handler();
K_WORK_DELAYABLE_DEFINE(check_cpld_work, check_cpld_handler);

K_THREAD_STACK_DEFINE(cpld_polling_stack, POLLING_CPLD_STACK_SIZE);
struct k_thread cpld_polling_thread;
k_tid_t cpld_polling_tid;

// cpld tbd
//void get_vr_vout_handler(struct k_work *work);
//K_WORK_DEFINE(vr_vout_work, get_vr_vout_handler);

typedef struct _vr_error_callback_info_ {
	uint8_t cpld_offset;
	uint8_t vr_status_word_access_map;
	uint8_t bit_mapping_vr_sensor_num[8];
} vr_error_callback_info;

bool vr_error_callback(cpld_info *cpld_info, uint8_t *current_cpld_value);

// clang-format off
cpld_info cpld_info_table[] = {
	{ VR_POWER_FAULT_1_REG, 			0x00, 0x00, true, 0x00, true, 0x00,  .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_ALL_BITS },
	{ VR_POWER_FAULT_2_REG, 			0x00, 0x00, true, 0x00, true, 0x00,  .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_ALL_BITS },
	{ VR_POWER_FAULT_3_REG, 			0x00, 0x00, true, 0x00, true, 0x00,  .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_ALL_BITS },
	{ VR_POWER_FAULT_4_REG, 			0x00, 0x00, true, 0x00, true, 0x00,  .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_ALL_BITS },
	{ VR_POWER_FAULT_5_REG, 			0x00, 0x00, true, 0x00, true, 0x00,  .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_ALL_BITS },
	{ VR_SMBUS_ALERT_EVENT_LOG_REG, 	0xFF, 0xFF, true, 0x00, false, 0x00,  .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_ALL_BITS },
	{ LEAK_DETECT_REG, 					0xFF, 0xFF, true, 0x00, false, 0x00, .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_BITS_6 },
	{ HBM_CATTRIP_REG, 					0xFF, 0xFF, true, 0x00, true, 0x00,  .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_BITS_78 },
	{ SYSTEM_ALERT_FAULT_REG, 			0xFF, 0xFF, true, 0x00, false, 0x00, .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_BITS_8 },
	{ ASIC_TEMP_OVER_REG, 				0xFF, 0xFF, true, 0x00, true, 0x00,  .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_BITS_8 },
	{ TEMP_IC_OVER_FAULT_REG, 			0xFF, 0xFF, true, 0x00, false, 0x00, .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_BITS_678 },
};

bool cpld_polling_enable_flag = true;


void set_cpld_polling_enable_flag(bool status)
{
	cpld_polling_enable_flag = status;
}

bool get_cpld_polling_enable_flag(void)
{
	return cpld_polling_enable_flag;
}

void reset_error_log_states(uint8_t err_type)
{
	// Reset cpld_info_table
	for (size_t i = 0; i < ARRAY_SIZE(cpld_info_table); i++) {
		cpld_info_table[i].is_fault_bit_map = 0x00;
		cpld_info_table[i].last_polling_value = 0x00;
	}

	// Remove and DEASSERT error logs with the err_type
	reset_error_log_event(err_type);

	LOG_INF("Reset error_log_states with err_type = 0x%02x", err_type);
}

bool vr_error_callback(cpld_info *cpld_info, uint8_t *current_cpld_value)
{
	CHECK_NULL_ARG_WITH_RETURN(cpld_info, false);
	CHECK_NULL_ARG_WITH_RETURN(current_cpld_value, false);

	// Get the expected value based on the current UBC status
	uint8_t expected_val =
		plat_get_ubc_status() ? cpld_info->dc_on_defaut : cpld_info->dc_off_defaut;

	// Calculate current faults and new faults
	uint8_t current_fault = (*current_cpld_value ^ expected_val) & cpld_info->bit_check_mask;
	uint8_t status_changed_bit = current_fault ^ cpld_info->is_fault_bit_map;

	if (!status_changed_bit)
		return true; // No new faults, return early

	LOG_DBG("CPLD register 0x%02X has status changed 0x%02X", cpld_info->cpld_offset,
		status_changed_bit);

	// Iterate through each bit in status_changed_bit to handle the corresponding VR
	for (uint8_t bit = 0; bit < 8; bit++) {
		if (!(status_changed_bit & BIT(bit)))
			continue;

		// Dynamically generate the error code
		uint16_t error_code = (CPLD_UNEXPECTED_VAL_TRIGGER_CAUSE << 13) | (bit << 8) |
				      cpld_info->cpld_offset;

		uint8_t bit_val = (*current_cpld_value & BIT(bit)) >> bit;
		uint8_t expected_bit_val = (expected_val & BIT(bit)) >> bit;
		LOG_DBG("cpld offset 0x%02X, bit %d, bit_val %d, expected_bit_val %d",
			cpld_info->cpld_offset, bit, bit_val, expected_bit_val);

		if (bit_val != expected_bit_val) {
			LOG_ERR("Generated error code: 0x%04X (bit %d, CPLD offset 0x%02X)",
				error_code, bit, cpld_info->cpld_offset);

			// Perform additional operations if needed
			LOG_DBG("ASSERT");
			error_log_event(error_code, LOG_ASSERT);
		} else {
			LOG_DBG("DEASSERT");
			error_log_event(error_code, LOG_DEASSERT);
		}
	}

	return true;
}

/* pdb1 power info get from bmc through CPLD */
uint16_t power_info = 0;
void get_cpld_polling_power_info(int* reading)
{
	*reading = power_info;
}
void plat_get_pdb1_pwr_from_bmc(void){
	uint8_t pwr_value_lsb = 0;
	uint8_t pwr_value_msb = 0;
	if (!plat_read_cpld(CPLD_POWER_INFO_0_REG, &pwr_value_lsb, 1)){
		LOG_ERR("LSB read from CPLD fail");
	}
	if (!plat_read_cpld(CPLD_POWER_INFO_1_REG, &pwr_value_msb, 1)){
		LOG_ERR("MSB read from CPLD fail");
	}
	power_info = (pwr_value_msb<<8)|pwr_value_lsb;
}

void plat_poll_cpld_info_table(void){
	uint8_t data = 0;

	for (size_t i = 0; i < ARRAY_SIZE(cpld_info_table); i++) {
		uint8_t expected_val = plat_get_ubc_status() ?
							cpld_info_table[i].dc_on_defaut :
							cpld_info_table[i].dc_off_defaut;

		// Read from CPLD
		if (!plat_read_cpld(cpld_info_table[i].cpld_offset, &data, 1)) {
			LOG_DBG("Failed to read CPLD register 0x%02X",
				cpld_info_table[i].cpld_offset);
			continue;
		}

		LOG_DBG("Polling CPLD 0x%02X raw=0x%02X, expected=0x%02X, mask=0x%02X",
			cpld_info_table[i].cpld_offset, data, expected_val,
			cpld_info_table[i].bit_check_mask);

		if (!cpld_info_table[i].is_fault_log)
			continue;

		uint8_t new_fault_map =
			(data ^ expected_val) & cpld_info_table[i].bit_check_mask;

		// get unrecorded fault bit map
		uint8_t is_status_changed =
			new_fault_map ^ cpld_info_table[i].is_fault_bit_map;

		if (is_status_changed) {
			if (cpld_info_table[i].status_changed_cb) {
				cpld_info_table[i].status_changed_cb(
					&cpld_info_table[i], &data);
			}
			if (cpld_info_table[i].send_to_bmc_flag) {
				process_mtia_vr_power_fault_sel(&cpld_info_table[i],
								&data);
				set_led_flag(true);
			}
			if (cpld_info_table[i].cpld_offset == VR_SMBUS_ALERT_EVENT_LOG_REG) {
				//get sensor pmbus alert status(if status_word temperature bit-2 is 1)
				uint8_t temp_data = 0;
				plat_read_cpld(VR_SMBUS_ALERT_EVENT_LOG_REG, &temp_data, 1);
				// check which VR_SMBUS_ALERT_EVENT_LOG_REG bit is changed
				LOG_INF("VR_SMBUS_ALERT_EVENT_LOG_REG: 0x%x", temp_data);
				for (int j = 0; j < 8; j++) {
					if (j == 0)
						continue; // skip bit-0
					// if temp data is changed
					if ((temp_data & BIT(j)) == 0)
					{
						LOG_WRN("SMBUS_ALERT_REG changed, bit-%d is changed", j);
						if(!check_temp_status_bit(j))
						{
							// electra board uses CPLD to control VR_HOT
							if (!plat_read_cpld(ASIC_VR_HOT_SWITCH, &data, 1)) {
								LOG_ERR("Failed to read ASIC_VR_HOT_SWITCH");
							}
							data |= BIT(0);
							if (!plat_write_cpld(ASIC_VR_HOT_SWITCH, &data)) {
								LOG_ERR("Failed to write ASIC_VR_HOT_SWITCH");
							}
							LOG_WRN("Temperature bit-%d is 1, write CPLD ASIC_VR_HOT_SWITCH bit-0 to 1", j);
							set_led_flag(true);
							break;
						}
					}
				}
			}
			// update map
			cpld_info_table[i].is_fault_bit_map = new_fault_map;
			cpld_info_table[i].last_polling_value = data;
		}
	}
}

void plat_poll_cpld_registers()
{
	// Note: cpld_polling_alert_status is abort due to the trigger mechanism change
	while (1) {
		plat_wait_for_cpld_polling_trigger();

		if (!is_update_state_idle() || !cpld_polling_enable_flag) {
			continue;
		}

		plat_get_pdb1_pwr_from_bmc();
		plat_poll_cpld_info_table();
	}
}

void check_cpld_handler()
{
	uint8_t data[4] = { 0 };
	uint32_t version = 0;
	if (!plat_i2c_read(I2C_BUS_CPLD, CPLD_ADDR, CPLD_OFFSET_USERCODE, data, 4)) {
		LOG_ERR("Failed to read cpld version from cpld");
	}
	version = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	LOG_DBG("The cpld version: %08x", version);

	k_work_schedule(&check_cpld_work, K_MSEC(5000));
}

void init_cpld_polling(void)
{
	plat_update_ubc_status();
	plat_activate_cpld_polling_semaphore_timer();
	cpld_polling_tid =
		k_thread_create(&cpld_polling_thread, cpld_polling_stack,
				K_THREAD_STACK_SIZEOF(cpld_polling_stack), plat_poll_cpld_registers,
				NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_MSEC(1000));
	k_thread_name_set(&cpld_polling_thread, "cpld_polling_thread");
}