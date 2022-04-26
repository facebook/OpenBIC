#ifndef UTIL_SYS_H
#define UTIL_SYS_H

#include "stdbool.h"

enum CC_12V_CYCLE_SLOT {
	SUCCESS_12V_CYCLE_SLOT,
	NOT_SUPPORT_12V_CYCLE_SLOT,
	SLOT_OFF_FAILED,
	SLOT_ON_FAILED
};

void submit_bic_cold_reset();
void bic_cold_reset();
void submit_bic_warm_reset();
void bic_warm_reset();
bool get_boot_source_ACon();
void set_boot_source();
void pal_warm_reset_prepare();
void pal_cold_reset_prepare();
int pal_submit_bmc_cold_reset();
int submit_12v_cycle_slot();
void ME_enter_recovery();
void ME_enter_restore();
void set_ME_restore();

#endif
