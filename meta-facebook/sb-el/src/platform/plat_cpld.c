#include "plat_cpld.h"
#include "libutil.h"
#include "plat_i2c.h"
#include "plat_util.h"
#include "plat_gpio.h"
#include "plat_log.h"
#include "plat_hook.h"
#include <logging/log.h>

#define CPLD_ADDR (0x4C >> 1)
#define I2C_BUS_CPLD I2C_BUS11

#define POLLING_CPLD_STACK_SIZE 2048
#define CPLD_POLLING_INTERVAL_MS 1000 // 1 second polling interval

#define CHECK_ALL_BITS 0xFF

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

void check_ubc_delayed_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(init_ubc_delayed_timer, check_ubc_delayed_timer_handler, NULL);
void check_ubc_delayed(struct k_work *work);
K_WORK_DEFINE(check_ubc_delayed_work, check_ubc_delayed);
void check_ubc_delayed_timer_handler(struct k_timer *timer)
{
	k_work_submit(&check_ubc_delayed_work);
}

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
	{ VR_POWER_FAULT_1_REG, 			0x00, 0x00, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_ALL_BITS },
	{ VR_POWER_FAULT_2_REG, 			0x00, 0x00, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_ALL_BITS },
	{ VR_POWER_FAULT_3_REG, 			0x00, 0x00, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_ALL_BITS },
	{ VR_POWER_FAULT_4_REG, 			0x00, 0x00, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_ALL_BITS },
	{ VR_POWER_FAULT_5_REG, 			0x00, 0x00, true, 0x00, false, false, 0x00,  .status_changed_cb = vr_error_callback, .bit_check_mask = CHECK_ALL_BITS },
};

bool cpld_polling_alert_status = false; // only polling cpld when alert status is true
bool cpld_polling_enable_flag = true;

void check_cpld_polling_alert_status(void)
{
	cpld_polling_alert_status = (gpio_get(ALL_VR_PM_ALERT_R_N) == 0);
}

void set_cpld_polling_enable_flag(bool status)
{
	cpld_polling_enable_flag = status;
}

bool get_cpld_polling_enable_flag(void)
{
	return cpld_polling_enable_flag;
}

bool ubc_enabled_delayed_status = false;

void check_ubc_delayed(struct k_work *work)
{
	/* FM_PLD_UBC_EN_R
	 * 1 -> UBC is enabled
	 * 0 -> UBC is disabled
	 */
	bool is_ubc_enabled = (gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH);

	bool is_dc_on = is_mb_dc_on();

	if (is_ubc_enabled) {
		if (is_dc_on != is_ubc_enabled) {
			//send event to bmc
			uint16_t error_code = (POWER_ON_SEQUENCE_TRIGGER_CAUSE << 13);
			error_log_event(error_code, LOG_ASSERT);
			LOG_ERR("Generated error code: 0x%x", error_code);
		}
	}

	ubc_enabled_delayed_status = is_ubc_enabled;

	LOG_DBG("UBC enabled delayed status: %d", ubc_enabled_delayed_status);

	/* cpld tbd
	if (is_ubc_enabled == true) {
		k_work_submit(&vr_vout_work);
	} */
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
		ubc_enabled_delayed_status ? cpld_info->dc_on_defaut : cpld_info->dc_off_defaut;

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

void poll_cpld_registers()
{
	uint8_t data = 0;
	bool prev_alert_status = false;

	while (1) {
		/* Sleep for the polling interval */
		k_msleep(CPLD_POLLING_INTERVAL_MS);

		LOG_DBG("cpld_polling_alert_status = %d, cpld_polling_enable_flag = %d",
			cpld_polling_alert_status, cpld_polling_enable_flag);

		// Check for falling edge of cpld_polling_alert_status (true -> false)
		if (prev_alert_status && !cpld_polling_alert_status) {
			uint8_t err_type = CPLD_UNEXPECTED_VAL_TRIGGER_CAUSE;
			LOG_DBG("cpld_polling_alert_status: true -> false, reset_error_log_states: %x",
				err_type);
			reset_error_log_states(err_type);
		}
		// Save current alert status for next loop comparison
		prev_alert_status = cpld_polling_alert_status;

		if (!cpld_polling_alert_status || !cpld_polling_enable_flag)
			continue;

		LOG_DBG("Polling CPLD registers");

		for (size_t i = 0; i < ARRAY_SIZE(cpld_info_table); i++) {
			uint8_t expected_val = ubc_enabled_delayed_status ?
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
				// update map
				cpld_info_table[i].is_fault_bit_map = new_fault_map;
				cpld_info_table[i].last_polling_value = data;
			}
		}
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
	check_cpld_polling_alert_status();

	cpld_polling_tid =
		k_thread_create(&cpld_polling_thread, cpld_polling_stack,
				K_THREAD_STACK_SIZEOF(cpld_polling_stack), poll_cpld_registers,
				NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_MSEC(1000));
	k_thread_name_set(&cpld_polling_thread, "cpld_polling_thread");
}