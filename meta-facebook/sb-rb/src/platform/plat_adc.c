#include <drivers/adc.h>
#include "libutil.h"
#include <logging/log.h>

#include "plat_adc.h"

LOG_MODULE_REGISTER(plat_adc);

#define BUFFER_SIZE 2
#define ADC_STACK_SIZE 1024

#define MEDHA0_ADC_CHANNEL 8
#define MEDHA1_ADC_CHANNEL 6

K_THREAD_STACK_DEFINE(adc_thread_stack, ADC_STACK_SIZE);
struct k_thread adc_poll_thread;

static bool adc_poll_flag = true;
static uint16_t adc_averge_times[ADC_IDX_MAX] = { 2, 6, 60, 80 }; // 10ms each time

static uint16_t adc_buf[ADC_IDX_MAX][ADC_AVERGE_TIMES_MAX];
static uint8_t adc_buf_idx[ADC_IDX_MAX] = { 0 };
static uint16_t adc_averge_val[ADC_IDX_MAX] = { 0 };
static uint32_t adc_sum[ADC_IDX_MAX] = { 0 };

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
		adc_sum[i] -= adc_buf[i][adc_buf_idx[i]];
		adc_buf[i][adc_buf_idx[i]] = ((i == ADC_IDX_MEDHA0_1) || (i == ADC_IDX_MEDHA0_2)) ?
						     m_sample_buffer[1] :
						     m_sample_buffer[0];
		adc_sum[i] += adc_buf[i][adc_buf_idx[i]];
		adc_averge_val[i] = adc_sum[i] / adc_averge_times[i];
		adc_buf_idx[i] = (adc_buf_idx[i] + 1) % adc_averge_times[i];
	}
}

uint16_t get_adc_averge_val(uint8_t idx)
{
	return adc_averge_val[idx];
}

uint16_t *get_adc_buf(uint8_t idx)
{
	return adc_buf[idx];
}

uint16_t get_adc_averge_times(uint8_t idx)
{
	return adc_averge_times[idx];
}

static void adc_poll_init()
{
	for (uint8_t i = ADC_IDX_MEDHA0_1; i < ADC_IDX_MAX; i++) {
		adc_sum[i] = 0;
		adc_buf_idx[i] = 0;
		memset(adc_buf[i], 0, sizeof(adc_buf[i]));
	}
}

void adc_set_averge_times(uint8_t idx, uint16_t time)
{
	if (time >= ADC_AVERGE_TIMES_MIN && time <= ADC_AVERGE_TIMES_MAX) {
		adc_averge_times[idx] = time;
		adc_sum[idx] = 0;
		adc_buf_idx[idx] = 0;
		memset(adc_buf[idx], 0, sizeof(adc_buf[idx]));
	} else {
		LOG_WRN("invalid adc %d poll times: %d\n", idx, time);
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
	k_thread_create(&adc_poll_thread, adc_thread_stack, ADC_STACK_SIZE, adc_polling_handler,
			NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(&adc_poll_thread, "platform adc read");

	LOG_INF("ADC polling thread started...\n");
}