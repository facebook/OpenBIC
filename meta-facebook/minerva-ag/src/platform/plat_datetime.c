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

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <logging/log.h>
#include <sys/timeutil.h>
#include "plat_datetime.h"
#include "pldm_firmware_update.h"
#include "mctp.h"
#include "pldm.h"

LOG_MODULE_REGISTER(plat_datetime);

#define BMC_EID 0x08
#define RTC_SYNC_INTERVAL_SEC 10
#define RTC_WORKQ_STACK_SIZE 2048
#define RTC_WORKQ_PRIORITY CONFIG_MAIN_THREAD_PRIORITY + 2

static struct tm base_tm;
static int64_t base_uptime_ms;
static bool rtc_synced;

struct k_work_q rtc_workq;
K_THREAD_STACK_DEFINE(rtc_workq_stack, RTC_WORKQ_STACK_SIZE);

static void rtc_sync_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(rtc_sync_work, rtc_sync_work_handler);

static uint8_t bcd_to_dec(uint8_t bcd)
{
	return ((bcd >> 4) & 0x0F) * 10 + (bcd & 0x0F);
}

int get_datetime_from_bmc(struct tm *tm_now)
{
	if (!tm_now)
		return -1;

	static uint8_t resp_buf[PLDM_MAX_DATA_SIZE];
	memset(resp_buf, 0, sizeof(resp_buf));
	pldm_msg pmsg = {
		.hdr = {
			.msg_type = MCTP_MSG_TYPE_PLDM,
			.pldm_type = 0x03,
			.cmd = 0x0C,
			.rq = PLDM_REQUEST,
		},
		.len = 0,
		.buf = NULL,
	};

	mctp *mctp_inst;
	if (!get_mctp_info_by_eid(BMC_EID, &mctp_inst, &pmsg.ext_params)) {
		LOG_ERR("Failed to get mctp info by eid 0x%x", BMC_EID);
		return -1;
	}

	uint16_t resp_len = mctp_pldm_read(mctp_inst, &pmsg, resp_buf, sizeof(resp_buf));
	if (!resp_len || resp_buf[0] != 0) {
		LOG_WRN("Failed to get valid MCTP-PLDM response");
		return -1;
	}

	tm_now->tm_sec = bcd_to_dec(resp_buf[1]);
	tm_now->tm_min = bcd_to_dec(resp_buf[2]);
	tm_now->tm_hour = bcd_to_dec(resp_buf[3]);
	tm_now->tm_mday = bcd_to_dec(resp_buf[4]);
	tm_now->tm_mon = bcd_to_dec(resp_buf[5]) - 1;
	tm_now->tm_year = bcd_to_dec(resp_buf[6]) + bcd_to_dec(resp_buf[7]) * 100 - 1900;
	tm_now->tm_isdst = 0;

	return 0;
}

static void rtc_sync_from_bmc(void)
{
	struct tm tm_now;
	if (get_datetime_from_bmc(&tm_now) == 0) {
		base_tm = tm_now;
		base_uptime_ms = k_uptime_get();
		rtc_synced = true;

		LOG_DBG("RTC sync: %04d-%02d-%02d %02d:%02d:%02d", base_tm.tm_year + 1900,
			base_tm.tm_mon + 1, base_tm.tm_mday, base_tm.tm_hour, base_tm.tm_min,
			base_tm.tm_sec);
	} else {
		LOG_DBG("RTC sync failed, keep last base time");
	}
}

static void rtc_sync_work_handler(struct k_work *work)
{
	if (is_update_state_idle() == false) {
		k_work_schedule_for_queue(&rtc_workq, &rtc_sync_work,
					  K_SECONDS(RTC_SYNC_INTERVAL_SEC));
		return;
	}
	rtc_sync_from_bmc();

	k_work_schedule_for_queue(&rtc_workq, &rtc_sync_work, K_SECONDS(RTC_SYNC_INTERVAL_SEC));
}

void rtc_init_once(void)
{
	k_work_queue_start(&rtc_workq, rtc_workq_stack, K_THREAD_STACK_SIZEOF(rtc_workq_stack),
			   RTC_WORKQ_PRIORITY, NULL);

	k_work_schedule_for_queue(&rtc_workq, &rtc_sync_work, K_SECONDS(RTC_SYNC_INTERVAL_SEC));
}

time_t rtc_time(time_t *tloc)
{
	time_t now;

	if (!rtc_synced) {
		now = (time_t)(k_uptime_get() / 1000);
	} else {
		int64_t elapsed_s = (k_uptime_get() - base_uptime_ms) / 1000;
		now = timeutil_timegm(&base_tm) + elapsed_s;
	}

	if (tloc)
		*tloc = now;

	return now;
}

void rtc_get_tm(struct tm *tm_now)
{
	time_t now = rtc_time(NULL);
	if (tm_now)
		*tm_now = *gmtime(&now);
}
