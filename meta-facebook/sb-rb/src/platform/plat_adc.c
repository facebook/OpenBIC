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
#include <drivers/spi.h>
#include "libutil.h"
#include <logging/log.h>
#include "plat_gpio.h"
#include "plat_adc.h"
#include "plat_class.h"
#include "plat_cpld.h"
#include <device.h>
#include "util_sys.h"
#include "plat_i2c_target.h"
#include "plat_pldm_sensor.h"
#include "plat_power_capping.h"

LOG_MODULE_REGISTER(plat_adc);

#define BUFFER_SIZE 2
#define ADC_STACK_SIZE 1024

#define MEDHA0_ADC_CHANNEL 8
#define MEDHA1_ADC_CHANNEL 6

#define ADC_SPI_FREQ 6000000

K_THREAD_STACK_DEFINE(adc_rainbow_thread_stack, ADC_STACK_SIZE);
struct k_thread adc_rainbow_poll_thread;

static bool adc_poll_flag = true;
uint8_t adc_idx_read = 0;
float ad4058_val_0 = 0;
float ad4058_val_1 = 0;
float ads7066_val_0 = 0;
float ads7066_val_1 = 0;
const float ads7066_vref = 2.5;
const float ad4058_vref = 2.5;
static uint8_t adc_good_status[2] = { 0xFF, 0xFF };
static uint8_t final_ucr_status = 0;

typedef struct {
	uint16_t avg_times; // 20ms at a time
	uint16_t buf[ADC_AVERGE_TIMES_MAX];
	uint16_t buf_idx;
	uint16_t avg_val;
	uint32_t sum;
	uint16_t ucr; // pwr
	uint16_t vr_voltage_buf[ADC_AVERGE_TIMES_MAX]; //ex. 0.8523 will save as 0.85 and 0.0023
	float pwr_avg_val;
	float vr_sum;
	bool ucr_status; // over current
} adc_info_t;

//MEDHA0: level 2 , level3
//MEDHA1: level 2 , level3
adc_info_t adc_info[ADC_IDX_MAX] = { { .avg_times = 20, .ucr = 1600 },
				     { .avg_times = 60, .ucr = 1600 },
				     { .avg_times = 600, .ucr = 1070 },
				     { .avg_times = 800, .ucr = 1070 } };

static const struct device *spi_dev;

uint8_t get_adc_good_status(uint8_t idx)
{
	return adc_good_status[idx];
}

uint8_t get_final_ucr_status()
{
	return (final_ucr_status & 0xF0);
}

void adc_poll_init()
{
	for (uint8_t i = ADC_IDX_MEDHA0_1; i < ADC_IDX_MAX; i++) {
		adc_info[i].sum = 0;
		adc_info[i].buf_idx = 0;
		adc_info[i].avg_val = 0;
		adc_info[i].vr_sum = 0;
		adc_info[i].pwr_avg_val = 0;
		memset(adc_info[i].buf, 0, sizeof(uint16_t) * ADC_AVERGE_TIMES_MAX);
		memset(adc_info[i].vr_voltage_buf, 0, sizeof(uint16_t) * ADC_AVERGE_TIMES_MAX);
	}
}

float get_ads7066_vref()
{
	return ads7066_vref;
}

float get_ad4058_vref()
{
	return ad4058_vref;
}
uint16_t get_adc_averge_val(uint8_t idx)
{
	return adc_info[idx].avg_val;
}

uint16_t *get_adc_buf(uint16_t idx)
{
	return adc_info[idx].buf;
}

uint16_t *get_vr_buf(uint16_t idx)
{
	return adc_info[idx].vr_voltage_buf;
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

float adc_raw_mv_to_apms(uint16_t v, float vref)
{
	float temp_v = ((float)v / 0xffff) * vref;
	return (get_vr_module() == VR_MODULE_MPS) ? 1000 * temp_v * 0.796 : 1000 * temp_v * 0.797;
}

uint16_t float_voltage_transfer_to_uint16(float temp_voltage_value)
{
	// Upper:ex. 0.85 → 85
	uint8_t upper = (uint8_t)(temp_voltage_value * 100);
	// Lower: (0.8523 - 0.85) = 0.0023 → 23
	uint16_t lower_raw = (uint16_t)((temp_voltage_value * 10000) - (upper * 100));
	uint8_t lower = (uint8_t)(lower_raw & 0xFF);
	// Pack into one uint16
	uint16_t packed = ((uint16_t)upper << 8) | lower;
	return packed;
}
float uint16_voltage_transfer_to_float(uint16_t temp_voltage_value)
{
	uint8_t upper = (temp_voltage_value >> 8) & 0xFF;
	uint8_t lower = temp_voltage_value & 0xFF;
	// restore upper (2 digit)
	float upper_val = upper / 100.0f;
	// restore lower（2 digit）
	float lower_val = lower / 10000.0f;
	float restored = upper_val + lower_val;
	return restored;
}

static void update_adc_info(uint16_t raw_data, uint8_t base_idx, float vref)
{
	LOG_DBG("base_idx: %d", base_idx);
	for (uint8_t i = base_idx; i < ADC_IDX_MAX; i += 2) {
		uint16_t m_sample_buffer[BUFFER_SIZE];
		m_sample_buffer[0] = raw_data;
		m_sample_buffer[1] = raw_data;
		adc_info_t *adc = &adc_info[i];
		// current averge
		adc->sum -= adc->buf[adc->buf_idx];
		adc->buf[adc->buf_idx] = ((i == ADC_IDX_MEDHA0_1) || (i == ADC_IDX_MEDHA0_2)) ?
						 m_sample_buffer[1] :
						 m_sample_buffer[0];
		adc->sum += adc->buf[adc->buf_idx];
		adc->avg_val = adc->sum / adc->avg_times;
		// voltage averge
		float temp_voltage_value = 0;
		if (base_idx == ADC_RB_IDX_MEDHA0)
			temp_voltage_value = get_cached_sensor_reading_by_sensor_number(
						     SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_VOLT_V) /
					     1000.0;
		else if (base_idx == ADC_RB_IDX_MEDHA1)
			temp_voltage_value = get_cached_sensor_reading_by_sensor_number(
						     SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_VOLT_V) /
					     1000.0;
		// transfer to uint16_t
		uint16_t voltage_packed = float_voltage_transfer_to_uint16(temp_voltage_value);
		adc->vr_sum -= uint16_voltage_transfer_to_float(adc->vr_voltage_buf[adc->buf_idx]);
		adc->vr_voltage_buf[adc->buf_idx] = voltage_packed;
		adc->vr_sum += uint16_voltage_transfer_to_float(adc->vr_voltage_buf[adc->buf_idx]);
		// average pwr = average voltage * average current
		adc->pwr_avg_val =
			(adc->vr_sum / adc->avg_times) * adc_raw_mv_to_apms(adc->avg_val, vref);

		// decrease buffer idx
		adc->buf_idx = (adc->buf_idx + 1) % adc->avg_times;

		// check status
		bool pwr_status = (adc->pwr_avg_val >= adc->ucr);
		adc->ucr_status = pwr_status;
		if (pwr_status) {
			final_ucr_status |= (1 << (7 - i));
		} else {
			final_ucr_status &= ~(1 << (7 - i));
		}
	}
}

static void update_vr_base_power_info()
{
	int int_value = 0;
	uint16_t val_medha0 = 0;
	uint16_t val_medha1 = 0;
	int_value =
		get_cached_sensor_reading_by_sensor_number(SENSOR_NUM_ASIC_P0V85_MEDHA0_VDD_PWR_W);
	val_medha0 = (int_value + 500) / 1000;
	int_value =
		get_cached_sensor_reading_by_sensor_number(SENSOR_NUM_ASIC_P0V85_MEDHA1_VDD_PWR_W);
	val_medha1 = (int_value + 500) / 1000;

	for (uint8_t i = 0; i < ADC_IDX_MAX; i++) {
		adc_info_t *adc = &adc_info[i];
		adc->sum -= adc->buf[adc->buf_idx];
		adc->buf[adc->buf_idx] = ((i == ADC_IDX_MEDHA0_1) || (i == ADC_IDX_MEDHA0_2)) ?
						 val_medha0 :
						 val_medha1;
		adc->sum += adc->buf[adc->buf_idx];
		adc->avg_val = adc->sum / adc->avg_times;

		// decrease buffer idx
		adc->buf_idx = (adc->buf_idx + 1) % adc->avg_times;

		// check status
		bool pwr_status = (adc->avg_val >= adc->ucr);
		adc->ucr_status = pwr_status;
		if (pwr_status) {
			final_ucr_status |= (1 << (7 - i));
		} else {
			final_ucr_status &= ~(1 << (7 - i));
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

void read_adc_info()
{
	uint8_t adc_idx = 0;
	plat_read_cpld(CPLD_OFFSET_ADC_IDX, &adc_idx, 1);
	adc_idx_read = adc_idx;

	/* read VENDOR_L to determine*/
	uint8_t value = 0;
	ad4058_write_reg(0xA8, 0x00, 0);
	ad4058_read_reg(0x0C, 0, &value);
	if (value == 0x56) {
		adc_idx_read = 0;
	} else {
		adc_idx_read = 1;
	}
}

uint8_t get_adc_type()
{
	return adc_idx_read;
}

float get_adc_vr_pwr(uint8_t idx)
{
	return adc_info[idx].pwr_avg_val;
}

float get_vr_vol_sum(uint8_t idx)
{
	return adc_info[idx].vr_sum;
}
int ads7066_read_reg(uint8_t reg, uint8_t idx, uint8_t *out_data)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}
	// Set GPIO73 as CS control pin SPI_ADC_CS0_N
	struct spi_cs_control cs_ctrl = {
		.gpio_dev = device_get_binding("GPIO_7"),
		.gpio_pin = 3,
		.gpio_dt_flags = GPIO_ACTIVE_LOW,
		.delay = 0, // No delay
	};
	switch (idx) {
	case ADC_RB_IDX_MEDHA0:
		// do nothing
		break;
	case ADC_RB_IDX_MEDHA1:
		// Set GPIO73 as CS control pin SPI_ADC_CS1_N
		cs_ctrl.gpio_dev = device_get_binding("GPIO_C");
		cs_ctrl.gpio_pin = 1;
		cs_ctrl.gpio_dt_flags = GPIO_ACTIVE_LOW;
		cs_ctrl.delay = 0; // No delay
		break;
	default:
		LOG_ERR("Invalid ADC index %d", idx);
		break;
	}

	const struct spi_config spi_cfg = {
		.frequency = 6000000, // 6MHz
		.operation =
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
		.slave = 0,
		.cs = &cs_ctrl,
	};

	uint8_t tx_buf[3] = { 0x10, reg, 0x00 }; // bit15=1: read
	uint8_t rx_buf[3] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	int ret = spi_write(spi_dev, &spi_cfg, &tx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		return ret;
	}
	ret = spi_read(spi_dev, &spi_cfg, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI read failed: %d", ret);
		return ret;
	}

	*out_data = rx_buf[0];
	LOG_INF("medha%d ADS7066 read reg 0x%02x: 0x%02x 0x%02x 0x%02x", idx, reg, rx_buf[0],
		rx_buf[1], rx_buf[2]);
	return 0;
}
int ads7066_write_reg(uint8_t reg, uint8_t write_val, uint8_t idx)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}
	// Set GPIO73 as CS control pin SPI_ADC_CS0_N
	struct spi_cs_control cs_ctrl = {
		.gpio_dev = device_get_binding("GPIO_7"),
		.gpio_pin = 3,
		.gpio_dt_flags = GPIO_ACTIVE_LOW,
		.delay = 0, // No delay
	};
	switch (idx) {
	case ADC_RB_IDX_MEDHA0:
		// do nothing
		break;
	case ADC_RB_IDX_MEDHA1:
		// Set GPIO73 as CS control pin SPI_ADC_CS1_N
		cs_ctrl.gpio_dev = device_get_binding("GPIO_C");
		cs_ctrl.gpio_pin = 1;
		cs_ctrl.gpio_dt_flags = GPIO_ACTIVE_LOW;
		cs_ctrl.delay = 0; // No delay
		break;
	default:
		LOG_ERR("Invalid ADC index %d", idx);
		break;
	}

	const struct spi_config spi_cfg = {
		.frequency = 6000000, // 6MHz
		.operation =
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
		.slave = 0,
		.cs = &cs_ctrl,
	};

	uint8_t tx_buf[3] = { 0x08, reg, write_val };

	struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	int ret = spi_write(spi_dev, &spi_cfg, &tx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		return ret;
	}
	LOG_INF("medha%d ADS7066 write reg 0x%02x", idx, reg);
	return 0;
}

static void ads7066_read_voltage(uint8_t idx)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
		return;
	}
	// Set GPIO73 as CS control pin SPI_ADC_CS0_N
	struct spi_cs_control cs_ctrl = {
		.gpio_dev = device_get_binding("GPIO_7"),
		.gpio_pin = 3,
		.gpio_dt_flags = GPIO_ACTIVE_LOW,
		.delay = 0, // No delay
	};
	switch (idx) {
	case ADC_RB_IDX_MEDHA0:
		// do nothing
		break;
	case ADC_RB_IDX_MEDHA1:
		// Set GPIOC1 as CS control pin SPI_ADC_CS1_N
		cs_ctrl.gpio_dev = device_get_binding("GPIO_C");
		cs_ctrl.gpio_pin = 1;
		cs_ctrl.gpio_dt_flags = GPIO_ACTIVE_LOW;
		cs_ctrl.delay = 0; // No delay
		break;
	default:
		LOG_ERR("Invalid ADC index %d", idx);
		break;
	}

	const struct spi_config spi_cfg = {
		.frequency = 6000000, // 6MHz
		.operation =
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
		.slave = 0,
		.cs = &cs_ctrl,
	};

	uint8_t tx_buf[3] = { 0x00, 0x00, 0x00 };
	uint8_t rx_buf[3] = { 0 };
	uint8_t out_buf[3] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	memset(rx_buf, 0, 3);
	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI failed: %d", ret);
		return;
	}

	memcpy(out_buf, rx_buf, 3);

	LOG_HEXDUMP_DBG(out_buf, 3, "ads7066_read_voltage_value");
	uint16_t raw_value = out_buf[0] << 8 | out_buf[1];
	if (idx == ADC_RB_IDX_MEDHA0) {
		ads7066_val_0 = ((float)raw_value / 65536) * ads7066_vref;
		update_adc_info(raw_value, ADC_RB_IDX_MEDHA0, ads7066_vref);
	} else if (idx == ADC_RB_IDX_MEDHA1) {
		ads7066_val_1 = ((float)raw_value / 65536) * ads7066_vref;
		update_adc_info(raw_value, ADC_RB_IDX_MEDHA1, ads7066_vref);
	}

	return;
}
int ad4058_read_reg(uint8_t reg, uint8_t idx, uint8_t *out_data)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}

	uint8_t cnv_pin = 0;
	// Set GPIO73 as CS control pin SPI_ADC_CS0_N
	struct spi_cs_control cs_ctrl = {
		.gpio_dev = device_get_binding("GPIO_7"),
		.gpio_pin = 3,
		.gpio_dt_flags = GPIO_ACTIVE_LOW,
		.delay = 0, // No delay
	};
	switch (idx) {
	case ADC_RB_IDX_MEDHA0:
		// do nothing
		cnv_pin = MEDHA0_CNV;
		break;
	case ADC_RB_IDX_MEDHA1:
		// Set GPIO73 as CS control pin SPI_ADC_CS1_N
		cs_ctrl.gpio_dev = device_get_binding("GPIO_C");
		cs_ctrl.gpio_pin = 1;
		cs_ctrl.gpio_dt_flags = GPIO_ACTIVE_LOW;
		cs_ctrl.delay = 0; // No delay
		cnv_pin = MEDHA1_CNV;
		break;
	default:
		LOG_ERR("Invalid ADC index %d", idx);
		break;
	}

	const struct spi_config spi_cfg = {
		.frequency = 6000000, // 6MHz
		.operation =
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
		.slave = 0,
		.cs = &cs_ctrl,
	};

	uint8_t tx_buf[3] = { 0x80, 0x00, 0x00 };
	uint8_t rx_buf[3] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = 3 };
	struct spi_buf rx = { .buf = rx_buf, .len = 3 };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	tx_buf[0] += reg;

	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		return ret;
	}

	*out_data = rx_buf[1];
	LOG_HEXDUMP_INF(rx_buf, 3, "ad4058_read_reg");
	return 0;
}
int ad4058_write_reg(uint8_t reg, uint8_t write_val, uint8_t idx)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}
	// Set GPIO73 as CS control pin SPI_ADC_CS0_N
	struct spi_cs_control cs_ctrl = {
		.gpio_dev = device_get_binding("GPIO_7"),
		.gpio_pin = 3,
		.gpio_dt_flags = GPIO_ACTIVE_LOW,
		.delay = 0, // No delay
	};
	switch (idx) {
	case ADC_RB_IDX_MEDHA0:
		// do nothing
		break;
	case ADC_RB_IDX_MEDHA1:
		// Set GPIO73 as CS control pin SPI_ADC_CS1_N
		cs_ctrl.gpio_dev = device_get_binding("GPIO_C");
		cs_ctrl.gpio_pin = 1;
		cs_ctrl.gpio_dt_flags = GPIO_ACTIVE_LOW;
		cs_ctrl.delay = 0; // No delay
		break;
	default:
		LOG_ERR("Invalid ADC index %d", idx);
		break;
	}

	const struct spi_config spi_cfg = {
		.frequency = 6000000, // 6MHz
		.operation =
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
		.slave = 0,
		.cs = &cs_ctrl,
	};

	uint8_t tx_buf[2] = { reg, write_val };
	uint8_t rx_buf[2] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = 2 };
	struct spi_buf rx = { .buf = rx_buf, .len = 2 };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		return ret;
	}

	LOG_HEXDUMP_DBG(tx_buf, 2, "ad4058_write_reg");
	return 0;
}

static void ad4058_read_voltage(uint8_t idx)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}

	uint8_t cnv_pin = 0;
	// Set GPIO73 as CS control pin SPI_ADC_CS0_N
	struct spi_cs_control cs_ctrl = {
		.gpio_dev = device_get_binding("GPIO_7"),
		.gpio_pin = 3,
		.gpio_dt_flags = GPIO_ACTIVE_LOW,
		.delay = 0, // No delay
	};
	switch (idx) {
	case ADC_RB_IDX_MEDHA0:
		// do nothing
		cnv_pin = MEDHA0_CNV;
		break;
	case ADC_RB_IDX_MEDHA1:
		// Set GPIO73 as CS control pin SPI_ADC_CS1_N
		cs_ctrl.gpio_dev = device_get_binding("GPIO_C");
		cs_ctrl.gpio_pin = 1;
		cs_ctrl.gpio_dt_flags = GPIO_ACTIVE_LOW;
		cs_ctrl.delay = 0; // No delay
		cnv_pin = MEDHA1_CNV;
		break;
	default:
		LOG_ERR("Invalid ADC index %d", idx);
		break;
	}

	const struct spi_config spi_cfg = {
		.frequency = 6000000, // 6MHz
		.operation =
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
		.slave = 0,
		.cs = &cs_ctrl,
	};

	uint8_t tx_buf[2] = { 0 };
	uint8_t rx_buf[3] = { 0 };
	uint8_t out_buf[3] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = 2 };
	struct spi_buf rx = { .buf = rx_buf, .len = 3 };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };
	// set cnv_pin to low
	gpio_set(cnv_pin, 0);

	memset(rx_buf, 0, 3);

	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI failed: %d", ret);
		return;
	}

	memcpy(out_buf, rx_buf, 3);
	/*
	(Read_back 16 bits data / 0xFFFF ) * Vref 
	example: 0b 62 83
	get 0xb628
	0xb628 / 0xffff = (0xb628 / 0xffff) * 3.3
	*/
	uint8_t high = (uint8_t)((out_buf[0] << 4) | (out_buf[1] >> 4));
	uint8_t low = (uint8_t)(((out_buf[1] & 0x0F) << 4) | (out_buf[2] >> 4));

	uint16_t raw_value = (uint16_t)((high << 8) | low);
	if (idx == ADC_RB_IDX_MEDHA0) {
		ad4058_val_0 = (float)raw_value / 65536 * ad4058_vref;
		update_adc_info(raw_value, ADC_RB_IDX_MEDHA0, ad4058_vref);
	} else if (idx == ADC_RB_IDX_MEDHA1) {
		ad4058_val_1 = (float)raw_value / 65536 * ad4058_vref;
		update_adc_info(raw_value, ADC_RB_IDX_MEDHA1, ad4058_vref);
	}

	// set cnv_pin to high
	gpio_set(cnv_pin, 1);

	return;
}

void ads7066_mode_init()
{
	//set auto-sequence mode
	// medha0 & medha1
	for (int i = 0; i < ADC_RB_IDX_MAX; i++) {
		ads7066_write_reg(0, 0x1, i);
		// if rainbow board revid >= EVT1B, disable internal Volt reference
		if (get_asic_board_id() == ASIC_BOARD_ID_RAINBOW &&
		    get_board_rev_id() >= REV_ID_DVT)
			ads7066_write_reg(0x1, 0x2, i);
		else
			ads7066_write_reg(0x1, 0x82, i);
		ads7066_write_reg(0x12, 0x1, i);
		ads7066_write_reg(0x3, 0x6, i);

		//check and update good status
		uint8_t value = 0;
		ads7066_read_reg(0x3, i, &value);
		adc_good_status[i] = (value & 0x07) == 0x06 ? 0 : 0xFF;

		ads7066_write_reg(0x4, 0x8, i);
		ads7066_write_reg(0x10, 0x11, i);
		ads7066_write_reg(0x2, 0x10, i);
	}
	LOG_INF("ads7066 mode init done");
}

void ad4058_mode_init()
{
	/*
		set ad4058 to burst averaging mode 
		sample rate: 300KHz = 3.33us
		Averaging ratio: 256
		3.33us * 256 = 0.8ms per sample
	*/
	for (int i = 0; i < ADC_RB_IDX_MAX; i++) {
		// exit to config mode, check product id
		ad4058_write_reg(0xA8, 0x00, i);
		uint8_t value = 0;
		ad4058_read_reg(0x03, i, &value);
		adc_good_status[i] = (value & 0x0F) == 0x07 ? 0 : 0xFF;

		ad4058_write_reg(0x27, 0x20, i);
		ad4058_write_reg(0x23, 0x7, i);
		ad4058_write_reg(0x21, 0x1, i);
		ad4058_write_reg(0x20, 0x1, i);
	}
	LOG_INF("ad4058 mode init done");
}

void adc_rainbow_polling_handler(void *p1, void *p2, void *p3)
{
	read_adc_info();
	LOG_INF("adc index is %d", adc_idx_read);
	if (adc_idx_read == ADI_AD4058)
		ad4058_mode_init();
	else if (adc_idx_read == TIC_ADS7066)
		ads7066_mode_init();
	else
		LOG_ERR("Invalid ADC index %d", adc_idx_read);

	while (1) {
		if (get_power_capping_source() == CAPPING_SOURCE_ADC) {
			if (adc_poll_flag) {
				switch (adc_idx_read) {
				case ADI_AD4058:
					ad4058_read_voltage(ADC_RB_IDX_MEDHA0);
					ad4058_read_voltage(ADC_RB_IDX_MEDHA1);
					break;
				case TIC_ADS7066:
					ads7066_read_voltage(ADC_RB_IDX_MEDHA0);
					ads7066_read_voltage(ADC_RB_IDX_MEDHA1);
					break;
				default:
					LOG_DBG("Invalid ADC index %d", adc_idx_read);
					break;
				}
			}
		} else if (get_power_capping_source() == CAPPING_SOURCE_VR) {
			update_vr_base_power_info();
		}
		k_msleep(0);
	}
}

void plat_adc_rainbow_init(void)
{
	k_thread_create(&adc_rainbow_poll_thread, adc_rainbow_thread_stack, ADC_STACK_SIZE,
			adc_rainbow_polling_handler, NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY,
			0, K_NO_WAIT);

	k_thread_name_set(&adc_rainbow_poll_thread, "platform adc(rainbow) read");

	LOG_INF("ADC(rainbow) polling thread started...\n");
}
