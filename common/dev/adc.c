#include "adc.h"

#include <cmsis_os2.h>
#include <drivers/adc.h>
#include <stdio.h>

#include "sensor.h"
#include "plat_gpio.h"
#include "plat_sensor_table.h"

#define ADC_CHAN_NUM 8
#define ADC_NUM 2
#define BUFFER_SIZE 1

#if DT_NODE_EXISTS(DT_NODELABEL(adc0))
#define DEV_ADC0
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(adc1))
#define DEV_ADC1
#endif

#define ADC_RESOLUTION 10
#define ADC_CALIBRATE 0
#define ADC_GAIN ADC_GAIN_1
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT

static struct device *dev_adc[ADC_NUM];
static int16_t sample_buffer[BUFFER_SIZE];

struct adc_channel_cfg channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = 0,
	.differential = 0,
};

struct adc_sequence sequence = {
	.channels = 0,
	.buffer = sample_buffer,
	.buffer_size = sizeof(sample_buffer),
	.resolution = ADC_RESOLUTION,
	.calibrate = ADC_CALIBRATE,
};

enum {
	ADC0,
	ADC1,
};

static void init_adc_dev(void)
{
#ifdef DEV_ADC0
	dev_adc[ADC0] = (struct device *)device_get_binding("ADC0");
#endif
#ifdef DEV_ADC1
	dev_adc[ADC1] = (struct device *)device_get_binding("ADC1");
#endif
}

bool adc_init()
{
	init_adc_dev();
	if (!(device_is_ready(dev_adc[0]) && device_is_ready(dev_adc[1]))) {
		printf("ADC device not found\n");
		return false;
	}
	for (uint8_t i = 0; i < ADC_NUM; i++) {
		channel_cfg.channel_id = i;
		adc_channel_setup(dev_adc[i], &channel_cfg);
		sequence.channels |= BIT(i);
	}
	return true;
}

int bat_3v_set_gpio(uint8_t sensor_num, void *arg)
{
	SET_GPIO_VALUE_CFG *cfg = arg;
	gpio_set(cfg->gpio_num, cfg->gpio_value);
	if (cfg->gpio_value == GPIO_HIGH) {
		osDelay(1);
	}

	return 0;
}

static bool adc_read_mv(uint8_t sensor_num, uint32_t index, uint32_t channel, int *adc_val)
{
	int err, retval;
	sequence.channels = BIT(channel);
	channel_cfg.channel_id = channel;
	retval = adc_channel_setup(dev_adc[index], &channel_cfg);
	if (retval) {
		printf("ADC sensor %x channel set fail\n", sensor_num);
		return false;
	}
	err = adc_read(dev_adc[index], &sequence);
	if (err != 0) {
		printf("ADC sensor %x reading fail with error %d\n", sensor_num, err);
		return false;
	}
	int32_t raw_value = sample_buffer[0];
	if (adc_get_ref(dev_adc[index]) > 0) {
		*adc_val = raw_value;
		adc_raw_to_millivolts(adc_get_ref(dev_adc[index]), channel_cfg.gain,
				      sequence.resolution, adc_val);
		return true;
	}
	return false;
}

bool adc_sensor_read(uint8_t sensor_num, float *reading)
{
	snr_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	uint8_t chip = cfg->port / ADC_CHAN_NUM;
	uint8_t number = cfg->port % ADC_CHAN_NUM;
	int val = 1;
	int ret = adc_read_mv(sensor_num, chip, number, &val);
	if (ret) {
		*reading = (float)val * cfg->arg0 / cfg->arg1 / 1000;
		cfg->cache = *reading;
		cfg->cache_status = SENSOR_READ_SUCCESS;
		return true;
	} else {
		return false;
	}
}
