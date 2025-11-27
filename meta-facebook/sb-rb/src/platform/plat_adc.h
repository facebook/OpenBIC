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

enum {
	ADC_RB_IDX_MEDHA0 = 0,
	ADC_RB_IDX_MEDHA1,
	ADC_RB_IDX_MAX,
};

#define ADC_AVERGE_TIMES_MIN 1
#define ADC_AVERGE_TIMES_MAX 1000

#define ADI_AD4058 0x0
#define TIC_ADS7066 0x1

void plat_adc_init(void);
void adc_set_poll_flag(uint8_t onoff);
bool adc_get_poll_flag();
float adc_raw_mv_to_apms(uint16_t v, float vref);
uint16_t get_adc_averge_val(uint8_t idx);
void adc_set_averge_times(uint8_t idx, uint16_t time);
uint16_t get_adc_averge_times(uint8_t idx);
uint16_t get_adc_ucr(uint8_t idx);
void set_adc_ucr(uint8_t idx, uint16_t ucr);
bool get_adc_ucr_status(uint8_t idx);
void plat_adc_rainbow_init(void);
void get_ads7066_voltage();
void get_ad4058_voltage();
uint8_t get_adc_type();
float get_ads7066_vref();
float get_ad4058_vref();
float get_adc_vr_pwr(uint8_t idx);
uint16_t float_voltage_transfer_to_uint16(float temp_voltage_value);
uint16_t *get_adc_buf(uint16_t idx);
float uint16_voltage_transfer_to_float(uint16_t temp_voltage_value);
int ads7066_read_reg(uint8_t reg, uint8_t idx);
int ads7066_write_reg(uint8_t reg, uint8_t write_val, uint8_t idx);
int ad4058_read_reg(uint8_t reg, uint8_t idx);
int ad4058_write_reg(uint8_t reg, uint8_t write_val, uint8_t idx);
uint16_t *get_vr_buf(uint16_t idx);
void read_adc_info();
