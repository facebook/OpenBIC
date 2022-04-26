#ifndef UTIL_SYS_H
#define UTIL_SYS_H

#include "stdbool.h"
#include "stdint.h"

#define INTEL_IANA 0x000157

enum CC_12V_CYCLE_SLOT {
	SUCCESS_12V_CYCLE_SLOT,
	NOT_SUPPORT_12V_CYCLE_SLOT,
	SLOT_OFF_FAILED,
	SLOT_ON_FAILED
};

enum FORCE_ME_RECOVERY_CMD {
	ME_FW_RECOVERY = 0x01,
	ME_FW_RESTORE = 0x02,
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

void set_me_firmware_mode(uint8_t me_fw_mode);
void init_me_firmware();

int pal_submit_bmc_cold_reset();
int pal_submit_12v_cycle_slot();

#endif
