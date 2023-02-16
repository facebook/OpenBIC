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

#include <logging/log.h>
#include "ipmi.h"
#include "ipmb.h"
#include "pldm.h"
#include "plat_isr.h"
#include "plat_gpio.h"
#include "plat_mctp.h"
#include "util_worker.h"

LOG_MODULE_REGISTER(plat_isr);

K_WORK_DELAYABLE_DEFINE(fio_power_button_work, fio_power_button_work_handler);
void fio_power_button_work_handler()
{
	int ret = 0;
	uint8_t gpio_num = FIO_PWRBTN_N_R;
	uint8_t button_status = gpio_get(gpio_num);

	/* Check FIO button press time for power control */
	if (button_status == LOW_ACTIVE) {
		ipmi_msg msg = { 0 };
		msg.InF_source = SELF;
		msg.InF_target = MCTP;
		msg.netfn = NETFN_OEM_1S_REQ;
		msg.cmd = CMD_OEM_1S_SEND_INTERRUPT_TO_BMC;

		msg.data_len = 6;
		msg.data[0] = IANA_ID & 0xFF;
		msg.data[1] = (IANA_ID >> 8) & 0xFF;
		msg.data[2] = (IANA_ID >> 16) & 0xFF;
		msg.data[3] = gpio_num;
		msg.data[4] = button_status;
		msg.data[5] = OPTIONAL_AC_OFF;

		ret = pal_pldm_send_ipmi_request(&msg, MCTP_EID_BMC);
		if (ret < 0) {
			LOG_ERR("Failed to send GPIO interrupt event to BMC, gpio number(%d) ret(%d)",
				gpio_num, ret);
		}
	}
}

void ISR_FIO_BUTTON()
{
	k_work_schedule_for_queue(&plat_work_q, &fio_power_button_work,
				  K_MSEC(PRESS_FIO_BUTTON_DELAY_MS));
}
