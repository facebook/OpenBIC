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

#include <drivers/adc.h>
#include "libutil.h"
#include <logging/log.h>

#include "plat_adc.h"
#include "plat_class.h"
#include "plat_cpld.h"

LOG_MODULE_REGISTER(plat_adc);

#define BUFFER_SIZE 2
#define ADC_STACK_SIZE 1024

#define MEDHA0_ADC_CHANNEL 8
#define MEDHA1_ADC_CHANNEL 6

K_THREAD_STACK_DEFINE(adc_thread_stack, ADC_STACK_SIZE);
struct k_thread adc_poll_thread;

static bool adc_poll_flag = true;

typedef struct {
	uint16_t avg_times; // 20ms at a time
	uint16_t buf[ADC_AVERGE_TIMES_MAX];
	uint8_t buf_idx;
	uint16_t avg_val;
	uint32_t sum;
	uint16_t ucr; // amps
	bool ucr_status; // over current
	struct k_work ucr_work;
} adc_info_t;

adc_info_t adc_info[ADC_IDX_MAX] = { { .avg_times = 1, .ucr = 980 },
				     { .avg_times = 3, .ucr = 895 },
				     { .avg_times = 30, .ucr = 538 },
				     { .avg_times = 40, .ucr = 497 } };

static void adc_poll_init()
{
	for (uint8_t i = ADC_IDX_MEDHA0_1; i < ADC_IDX_MAX; i++) {
		adc_info[i].sum = 0;
		adc_info[i].buf_idx = 0;
		adc_info[i].avg_val = 0;
		memset(adc_info[i].buf, 0, sizeof(adc_info[i].buf));
	}
}

uint16_t get_adc_averge_val(uint8_t idx)
{
	return adc_info[idx].avg_val;
}

uint16_t *get_adc_buf(uint8_t idx)
{
	return adc_info[idx].buf;
}

uint16_t get_adc_averge_times(uint8_t idx)
{
	return adc_info[idx].avg_times;
}
void adc_set_averge_times(uint8_t idx, uint16_t time)
{
	if (time >= ADC_AVERGE_TIMES_MIN && time <= ADC_AVERGE_TIMES_MAX) {
		adc_info[idx].avg_times = time;
		adc_poll_init();
	} else {
		LOG_WRN("invalid adc %d poll times: %d\n", idx, time);
	}
}

uint16_t get_adc_ucr(uint8_t idx)
{
	return adc_info[idx].ucr;
}
void set_adc_ucr(uint8_t idx, uint16_t ucr)
{
	adc_info[idx].ucr = ucr;
}

bool get_adc_ucr_status(uint8_t idx)
{
	return adc_info[idx].ucr_status;
}
void set_adc_ucr_status(uint8_t idx, bool status)
{
	adc_info[idx].ucr_status = status;
}

float adc_raw_mv_to_apms(uint16_t mv)
{
	return (get_vr_module() == VR_MODULE_MPS) ? 2 * mv * 0.796 : 2 * mv * 0.797;
}

static bool adc_ucr_handler(uint8_t idx, bool state) // state is ucr or not
{
	uint8_t data = 0;
	if (!plat_read_cpld(CPLD_OFFSET_POWER_CLAMP, &data, 1))
		return false;

	uint8_t bit = (idx == ADC_IDX_MEDHA0_1) ? 7 :
		      (idx == ADC_IDX_MEDHA1_1) ? 6 :
		      (idx == ADC_IDX_MEDHA0_2) ? 5 :
		      (idx == ADC_IDX_MEDHA1_2) ? 4 :
						  0xFF;
	if (bit == 0xFF)
		return false;

	// if ucr, pull up
	if (state)
		data |= (1 << bit);
	else
		data &= ~(1 << bit);

	if (!plat_write_cpld(CPLD_OFFSET_POWER_CLAMP, &data))
		return false;

	return true;
}
static void adc_ucr_work_handler(struct k_work *work)
{
	const adc_info_t *adc = CONTAINER_OF(work, adc_info_t, ucr_work);

	uint8_t idx = adc - adc_info;
	adc_ucr_handler(idx, adc->ucr_status);
}
K_WORK_DEFINE(adc_ucr_work, adc_ucr_work_handler);

static void plat_adc_read(void)
{
	uint16_t m_sample_buffer[BUFFER_SIZE];
	const struct device *adc_dev;
	int retval;

	adc_dev = device_get_binding("ADC_0");
	if (adc_dev == NULL) {
		LOG_INF("ADC device not found\n");
		return;
	}

	const struct adc_sequence sequence = {
		.channels = (BIT(MEDHA0_ADC_CHANNEL) | BIT(MEDHA1_ADC_CHANNEL)),
		.buffer = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution = 10,
		.calibrate = 1,
	};

	retval = adc_read(adc_dev, &sequence);

	for (uint8_t i = ADC_IDX_MEDHA0_1; i < ADC_IDX_MAX; i++) {
		adc_info_t *adc = &adc_info[i];
		adc->sum -= adc->buf[adc->buf_idx];
		adc->buf[adc->buf_idx] = ((i == ADC_IDX_MEDHA0_1) || (i == ADC_IDX_MEDHA0_2)) ?
						 m_sample_buffer[1] :
						 m_sample_buffer[0];
		adc->sum += adc->buf[adc->buf_idx];
		adc->avg_val = adc->sum / adc->avg_times;
		adc->buf_idx = (adc->buf_idx + 1) % adc->avg_times;

		// check status
		bool curr_status = (adc_raw_mv_to_apms(adc->avg_val) >= adc->ucr);
		if (adc->ucr_status != curr_status) {
			adc->ucr_status = curr_status;
			k_work_submit(&adc->ucr_work);
		}
	}
}

bool adc_get_poll_flag()
{
	return adc_poll_flag;
}
void adc_set_poll_flag(uint8_t onoff)
{
	adc_poll_flag = onoff ? true : false;
	if (!adc_poll_flag)
		adc_poll_init();
}

void adc_polling_handler(void *p1, void *p2, void *p3)
{
	while (1) {
		if (adc_poll_flag)
			plat_adc_read();
		k_msleep(1);
	}
}

void plat_adc_init(void)
{
	for (uint8_t i = 0; i < ADC_IDX_MAX; i++) {
		k_work_init(&adc_info[i].ucr_work, adc_ucr_work_handler);
	}

	k_thread_create(&adc_poll_thread, adc_thread_stack, ADC_STACK_SIZE, adc_polling_handler,
			NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(&adc_poll_thread, "platform adc read");

	LOG_INF("ADC polling thread started...\n");
}
