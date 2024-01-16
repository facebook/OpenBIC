/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef PLDM_STATE_SET_H
#define PLDM_STATE_SET_H

/* Follow the PLDM state set specification (DSP0249_1.1.0) */
enum pldm_state_set_id_num {
	PLDM_STATE_SET_PRESENCE = 13,
	PLDM_STATE_SET_BOOT_RESTART_CAUSE = 192,

	/* OEM reserved state sets 32768 - 65535 */
	PLDM_STATE_SET_OEM_DEVICE_STATUS = 32768,
};

enum pldm_state_set_presence {
	PLDM_STATE_SET_PRESENT = 1,
	PLDM_STATE_SET_NOT_PRESENT = 2,
};

/**
 * @brief List of states for the Boot Restart Cause state set (ID 192).
 */
enum pldm_state_set_boot_restart_cause {
	PLDM_STATE_SET_BOOT_RESTART_CAUSE_POWERED_UP = 1,
	PLDM_STATE_SET_BOOT_RESTART_CAUSE_HARD_RESET = 2,
	PLDM_STATE_SET_BOOT_RESTART_CAUSE_WARM_RESET = 3,
	PLDM_STATE_SET_BOOT_RESTART_CAUSE_MANUAL_HARD_RESET = 4,
	PLDM_STATE_SET_BOOT_RESTART_CAUSE_MANUAL_WARM_RESET = 5,
	PLDM_STATE_SET_BOOT_RESTART_CAUSE_SYSTEM_RESTART = 6,
	PLDM_STATE_SET_BOOT_RESTART_CAUSE_WATCHDOG_TIMEOUT = 7
};

/**
 * @brief List of oem states for the device status (ID 32768)
 * @todo Can define other common status for device using.
 */
enum pldm_state_set_oem_device_status {
	PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL = 1,
	PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT = 2,
};

enum pldm_state_set_oem_device_power_status {
	PLDM_STATE_SET_OEM_DEVICE_NO_POWER_GOOD,
	PLDM_STATE_SET_OEM_DEVICE_POWER_GOOD_FAIL,
	PLDM_STATE_SET_OEM_DEVICE_3V3_POWER_FAULT,
	PLDM_STATE_SET_OEM_DEVICE_12V_POWER_FAULT,
	PLDM_STATE_SET_OEM_DEVICE_3V3_AUX_FAULT,
	PLDM_STATE_SET_OEM_DEVICE_POWER_GOOD_FAULT,
	PLDM_STATE_SET_OEM_DEVICE_3V3_NO_POWER_GOOD,
	PLDM_STATE_SET_OEM_DEVICE_12V_NO_POWER_GOOD,
	PLDM_STATE_SET_OEM_DEVICE_3V3_AUX_NO_POWER_GOOD,
	PLDM_STATE_SET_OEM_DEVICE_12V_AUX_POWER_FAULT,
	PLDM_STATE_SET_OEM_DEVICE_12V_AUX_NO_POWER_GOOD,
};

#endif
