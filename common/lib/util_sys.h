#ifndef UTIL_SYS_H
#define UTIL_SYS_H

#include <sys/reboot.h>
#include "stdbool.h"
#include "stdint.h"

#define INTEL_IANA 0x000157

enum CC_12V_CYCLE_SLOT {
	SUCCESS_12V_CYCLE_SLOT,
	NOT_SUPPORT_12V_CYCLE_SLOT,
	SLOT_OFF_FAILED,
	SLOT_ON_FAILED
};

enum ME_MODE {
	ME_INIT_MODE = 0x00,
	ME_NORMAL_MODE = 0x01,
	ME_RECOVERY_MODE = 0x02,
};

enum FORCE_ME_RECOVERY_CMD {
	ME_FW_RECOVERY = 0x01,
	ME_FW_RESTORE = 0x02,
};

enum SYSTEM_RESET_TYPE {
	// Aspeed system warm reset default setting is SOC reset, and system cold reset is full chip reset
	SOC_RESET = SYS_REBOOT_WARM,
	FULL_CHIP_RESET = SYS_REBOOT_COLD,
};

void submit_bic_cold_reset();
void bic_cold_reset();
void submit_bic_warm_reset();
void bic_warm_reset();
bool is_ac_lost();
void pal_warm_reset_prepare();
void pal_cold_reset_prepare();
int pal_submit_bmc_cold_reset();
int submit_12v_cycle_slot();

int set_me_firmware_mode(uint8_t me_fw_mode);
void init_me_firmware();
uint8_t get_me_mode();

int pal_submit_bmc_cold_reset();
int pal_submit_12v_cycle_slot();

void set_sys_ready_pin(uint8_t ready_gpio_name);

#endif
