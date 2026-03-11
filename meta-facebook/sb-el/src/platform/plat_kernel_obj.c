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

#include "plat_kernel_obj.h"
#include "plat_gpio.h"
#include "plat_log.h"
#include "plat_hook.h"
// pending
#include <shell_plat_power_sequence.h>
#include <logging/log.h>


LOG_MODULE_REGISTER(plat_kernel);

/* semaphore CPLD polling semaphore */
K_TIMER_DEFINE(ragular_cpld_polling_sem_timer, plat_ragular_cpld_polling_sem_handler, NULL);
static struct k_sem cpld_polling_sem;

void plat_ragular_cpld_polling_sem_handler(struct k_timer *timer){
	k_sem_give(&cpld_polling_sem);
}

void plat_activate_cpld_polling_semaphore_timer(void){
    k_sem_init(&cpld_polling_sem, 0, 1);
    k_timer_start(&ragular_cpld_polling_sem_timer, K_MSEC(1000), K_MSEC(1000));	
}

void plat_wait_for_cpld_polling_trigger(void){
    k_sem_take(&cpld_polling_sem, K_FOREVER);
}

void plat_trigger_cpld_polling(void){
    LOG_WRN("triggering CPLD polling");
    k_sem_give(&cpld_polling_sem);
}

/* Timer for dc status checking
We expect UBC ON will trigger DC ON. */
bool ubc_status = false; // "ubc_enabled_delayed_status" in rainbow
void plat_check_ubc_delayed_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(check_ubc_delayed_timer, plat_check_ubc_delayed_timer_handler, NULL);

void plat_check_ubc_delayed_timer_handler(struct k_timer *timer){
		/* FM_PLD_UBC_EN_R
	 * 1 -> UBC is enabled
	 * 0 -> UBC is disabled
	 */
	bool is_ubc_enabled = (gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH);
	ubc_status = is_ubc_enabled;
}

void plat_update_ubc_status(void){
    // delay 1 second for power sequence
    k_timer_start(&check_ubc_delayed_timer, K_MSEC(1000), K_NO_WAIT);
}

bool plat_get_ubc_status(void){
    return ubc_status;
}

/* Timer for check and handle power sequence events */
void pwr_sequence_event_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(pwr_sequence_event_work_timer, pwr_sequence_event_timer_handler, NULL);
void pwr_sequence_event(struct k_work *work);
K_WORK_DEFINE(pwr_sequence_event_work, pwr_sequence_event);

void pwr_sequence_event_timer_handler(struct k_timer *timer){
	k_work_submit(&pwr_sequence_event_work);
}

void pwr_sequence_event(struct k_work *work){
	bool is_dc_on_status = is_mb_dc_on();
	// if dc off
    // Handle power-on sequence event if failure.
	if (!is_dc_on_status) {
		plat_find_power_seq_fail();
		uint8_t idx = plat_get_power_seq_fail_id();

		uint16_t error_code = (POWER_ON_SEQUENCE_TRIGGER_CAUSE << 13);
		error_log_event(error_code, LOG_ASSERT);
		LOG_ERR("power on sequence fail, error_code: 0x%x", error_code);

		//send event to bmc
		struct pldm_addsel_data sel_msg = { 0 };
		sel_msg.assert_type = LOG_ASSERT;
		sel_msg.event_type = ARKE_FAULT;
		sel_msg.event_data_1 = ARKE_POWER_ON_SEQUENCE_FAIL;
		sel_msg.event_data_2 = idx;

		if (PLDM_SUCCESS != send_event_log_to_bmc(sel_msg)) {
			LOG_ERR("Send SEL fail: 0x%x 0x%x 0x%x 0x%x", sel_msg.assert_type,
				sel_msg.event_data_1, sel_msg.event_data_2, sel_msg.event_data_3);
		} else {
			LOG_INF("Send SEL: 0x%x 0x%x 0x%x 0x%x", sel_msg.assert_type,
				sel_msg.event_data_1, sel_msg.event_data_2, sel_msg.event_data_3);
		}
	}
}

void plat_handle_pwr_sequence_event(void){
    // delay 1 second for power sequence
    k_timer_start(&pwr_sequence_event_work_timer, K_MSEC(1000), K_NO_WAIT);
}