#ifndef PLAT_CPLD_H
#define PLAT_CPLD_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <zephyr.h>

// VR power fault registers
#define VR_POWER_FAULT_1_REG 0x0D
#define VR_POWER_FAULT_2_REG 0x0E
#define VR_POWER_FAULT_3_REG 0x0F
#define VR_POWER_FAULT_4_REG 0x10
#define VR_POWER_FAULT_5_REG 0x11

// ASIC fault registers
#define LEAK_DETECT_REG 0x24
#define VR_SMBUS_ALERT_EVENT_LOG_REG 0x26
#define HBM_CATTRIP_REG 0x27
#define SYSTEM_ALERT_FAULT_REG 0x28
#define ASIC_TEMP_OVER_REG 0x29
#define TEMP_IC_OVER_FAULT_REG 0x2A

// CPLD VR hot registers
#define ASIC_VR_HOT_SWITCH 0x12

// TMP75 type thermal sensor alert register,
// TMP432 handle by itself
#define TMP75_ALERT_CPLD_OFFSET 0x2F

// CPLD power steps on registers
#define VR_AND_CLK_EN 0x3E
#define VR_1_EN 0x3F
#define VR_2_EN 0x40
#define VR_3_EN 0x41
#define VR_4_EN 0x42
#define VR_AND_CLK_EN_PIN_CTRL 0xA1 // pin control (1-step only)
#define VR_1STEP_FUNC_EN_REG 0xA9
#define CPLD_ERRROR_INJECT 0x3D //debug only

// CPLD power good status registers
#define VR_PWRGD_PIN_READING_1_REG 0x07
#define VR_PWRGD_PIN_READING_2_REG 0x08
#define VR_PWRGD_PIN_READING_3_REG 0x09
#define VR_PWRGD_PIN_READING_4_REG 0x0A
#define VR_PWRGD_PIN_READING_5_REG 0x0B
#define VR_PWRGD_PIN_READING_6_REG 0x0C

// PDB1 power reading registers
#define CPLD_POWER_INFO_0_REG 0xB6
#define CPLD_POWER_INFO_1_REG 0xB7

// Power sequence fail log registers, CPLD not ready
#define PWRGD_EVENT_LATCH_1_REG 0xBE
#define PWRGD_EVENT_LATCH_2_REG 0xBF
#define PWRGD_EVENT_LATCH_3_REG 0xC0
#define PWRGD_EVENT_LATCH_4_REG 0xC1
#define PWRGD_EVENT_LATCH_5_REG 0xC2
#define PWRGD_EVENT_LATCH_6_REG 0xC3

// ASIC_THERMTRIP_TRIGGER_CAUSE log register
#define HBM_CATTRIP_LOG_REG 0x27

// CPLD bootstrap mapping and MFIO debug registers
#define HAMSA_STRAP 0x16
#define HAMSA_MFIO_REG 0x17
#define HAMSA_CONTROL_IO 0x18
#define HAMSA_JTAG_JTCE 0x19
#define NUWA0_STRAP 0x1A
#define NUWA0_CONTROL_IO 0x1B
#define NUWA1_STRAP 0x1C
#define NUWA1_CONTROL_IO 0x1D
#define NUWA_MFIO_REG 0x1E
#define NUWA_JTAG_JTCE 0x1F
#define OWL_CONTROL_IO 0x20
#define OWL_JTAG_SEL 0x21
#define OWL_UART_SEL 0x22
#define OWL_DVT_ENABLE 0x9E
// (High -> OUT, Low -> IN)
#define HAMSA_MFIO12_13_14_CTRL 0xB8
#define NUWA_MFIO12_13_14_CTRL 0xB9
#define HAMSA_MFIO12_13_14_INPUT 0xBA
#define NUWA_MFIO12_13_14_INPUT 0xBB

// CPLD platform info
#define CPLD_OFFSET_BOARD_REV_ID 0x14
#define CPLD_OFFSET_VR_VENDER_TYPE 0x15
#define CPLD_OFFSET_ASIC_BOARD_ID 0x3C

// CPLD power capping
#define CPLD_OFFSET_POWER_CLAMP 0x25
#define CPLD_OFFSET_POWER_CAPPING_LV1_TIME 0x36

// power information from BMC reg
#define CPLD_POWER_INFO_0_REG 0xB6
#define CPLD_POWER_INFO_1_REG 0xB7

// delay pcie perst
#define CPLD_PERST_DELAY_0_REG 0x9D
#define CPLD_PERST_DELAY_1_REG 0xB3
#define CPLD_PERST_DELAY_2_REG 0xB4
#define CPLD_PERST_DELAY_3_REG 0xB5

// delay asic rst
#define CPLD_OFFSET_ASIC_RST_DELAY 0xA4

// delay module pg
#define CPLD_OFFSET_MODULE_PG_DELAY 0xA5

// strap_control_manual
#define CPLD_OFFSE_MANUAL_CONTROL_STRAP 0xB2

// Other CPLD registers
#define CPLD_OFFSET_ASIC_RESET 0x00
#define VR_EN_PIN_READING_5 0x05
#define CPLD_OFFSET_USERCODE 0x32
#define CPLD_OFFSET_MMC_PWR_EN 0x38
#define ASIC_JTAG_MUX_SEL 0x39
#define CPLD_OFFSET_ADC_IDX 0xA0
#define CPLD_ASIC_RESET_STATUS_REG 0xA2

typedef struct _cpld_info_ cpld_info;

typedef struct _cpld_info_ {
	uint8_t cpld_offset;
	uint8_t dc_off_defaut;
	uint8_t dc_on_defaut;
	bool is_fault_log; // if true, check the value is defaut or not
	uint8_t is_fault_bit_map; //flag for fault

	/* is_send_bmc in electra */
	bool send_to_bmc_flag; //flag for sending alert to bmc

	//temp data for last polling
	uint8_t last_polling_value;

	bool (*status_changed_cb)(cpld_info *, uint8_t *);

	uint8_t bit_check_mask; //bit check mask

} cpld_info;

typedef struct {
	const char *name;
	uint8_t bit;
	uint8_t offset;
} cpld_pin_map_t;

bool plat_read_cpld(uint8_t offset, uint8_t *data, uint8_t len);
bool plat_write_cpld(uint8_t offset, uint8_t *data);
bool set_cpld_bit(uint8_t cpld_offset, uint8_t bit, uint8_t value);
void init_cpld_polling(void);
void get_cpld_polling_power_info(int *reading);
void set_cpld_polling_enable_flag(bool status);
void reset_error_log_states(uint8_t err_type);

#endif