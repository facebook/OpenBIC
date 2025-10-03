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

enum {
	ADC_IDX_MEDHA0_1 = 0, // 20ms
	ADC_IDX_MEDHA1_1, // 60ms
	ADC_IDX_MEDHA0_2, // 600ms
	ADC_IDX_MEDHA1_2, // 800ms
	ADC_IDX_MAX,
};

#define ADC_AVERGE_TIMES_MIN 1
#define ADC_AVERGE_TIMES_MAX 100

void plat_adc_init(void);
void adc_set_poll_flag(uint8_t onoff);
bool adc_get_poll_flag();
float adc_raw_mv_to_apms(uint16_t mv);
uint16_t get_adc_averge_val(uint8_t idx);
void adc_set_averge_times(uint8_t idx, uint16_t time);
uint16_t *get_adc_buf(uint8_t idx);
uint16_t get_adc_averge_times(uint8_t idx);
uint16_t get_adc_ucr(uint8_t idx);
void set_adc_ucr(uint8_t idx, uint16_t ucr);
bool get_adc_ucr_status(uint8_t idx);
