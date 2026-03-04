#ifndef PLAT_CPLD_H
#define PLAT_CPLD_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define CPLD_OFFSET_ASIC_RESET 0x00
#define VR_EN_PIN_READING_5 0x05
#define CPLD_OFFSET_BOARD_REV_ID 0x14
#define CPLD_OFFSET_VR_VENDER_TYPE 0x15
#define CPLD_OFFSET_POWER_CLAMP 0x25
#define CPLD_OFFSET_USERCODE 0x32
#define ASIC_JTAG_MUX_SEL 0x39
#define CPLD_OFFSET_ASIC_BOARD_ID 0x3C

#define VR_AND_CLK_EN 0x3E
#define VR_1_EN 0x3F
#define VR_2_EN 0x40
#define VR_3_EN 0x41
#define VR_4_EN 0x42

#define VR_AND_CLK_EN_PIN_CTRL 0xA1
#define VR_PWRGD_PIN_READING_1_REG 0x07
#define VR_PWRGD_PIN_READING_2_REG 0x08
#define VR_PWRGD_PIN_READING_3_REG 0x09
#define VR_PWRGD_PIN_READING_4_REG 0x0A
#define VR_PWRGD_PIN_READING_5_REG 0x0B
#define VR_PWRGD_PIN_READING_6_REG 0x0C
#define VR_CLK_ENABLE_PIN_CTRL_REG 0xA1 // pin control (1-step only)
#define CPLD_ASIC_RESET_STATUS_REG 0xA2

#define VR_1STEP_FUNC_EN_REG 0xA9

typedef struct _cpld_info_ cpld_info;

typedef struct _cpld_info_ {
	uint8_t cpld_offset;
	uint8_t dc_off_defaut;
	uint8_t dc_on_defaut;
	bool is_fault_log; // if true, check the value is defaut or not
	uint8_t is_fault_bit_map; //flag for fault

	//flag for 1st polling
	bool is_first_polling;

	//flag for 1st polling after changing DC status
	bool is_first_polling_after_dc_change;

	//temp data for last polling
	uint8_t last_polling_value;

	bool (*status_changed_cb)(cpld_info *, uint8_t *);

	uint8_t bit_check_mask; //bit check mask

} cpld_info;

bool plat_read_cpld(uint8_t offset, uint8_t *data, uint8_t len);
bool plat_write_cpld(uint8_t offset, uint8_t *data);
bool set_cpld_bit(uint8_t cpld_offset, uint8_t bit, uint8_t value);
void init_cpld_polling(void);
void check_cpld_polling_alert_status(void);
void get_cpld_polling_power_info(int *reading);
void set_cpld_polling_enable_flag(bool status);

#endif