#include <stdio.h>
#include "sensor.h"
#include "sensor_def.h"
#include "pal.h"
#include "plat_gpio.h"
#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/adc.h>

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
int32_t adc_vref;

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
	adc0,
	adc1,
};

void init_adc_dev(void)
{
#ifdef DEV_ADC0
	dev_adc[adc0] = device_get_binding("ADC0");
#endif
#ifdef DEV_ADC1
	dev_adc[adc1] = device_get_binding("ADC1");
#endif
}

bool adc_init()
{
	init_adc_dev();
	if (!(device_is_ready(dev_adc[0]) && device_is_ready(dev_adc[1]))) {
		printk("ADC device not found\n");
		return false;
	}
	for (uint8_t i = 0; i < ADC_NUM; i++) {
		channel_cfg.channel_id = i;
		adc_channel_setup(dev_adc[i], &channel_cfg);
		sequence.channels |= BIT(i);
	}
	return true;
}

static bool adc_read_mv(uint32_t index, uint32_t channel, int *adc_val)
{
	int err;
	adc_vref = adc_get_ref(dev_adc[index]);
	sequence.channels = BIT(channel);
	channel_cfg.channel_id = channel;
	adc_channel_setup(dev_adc[index], &channel_cfg);
	err = adc_read(dev_adc[index], &sequence);
	if (err != 0) {
		printk("ADC reading failed with error %d.\n", err);
		return false;
	}
	int32_t raw_value = sample_buffer[0];
	if (adc_vref > 0) {
		*adc_val = raw_value;
		adc_raw_to_millivolts(adc_get_ref(dev_adc[index]), ADC_GAIN, ADC_RESOLUTION,
				      adc_val);
		return true;
	}
	return false;
}
bool pal_adc_read(uint8_t sensor_num, int *reading)
{
	uint8_t snrcfg_sensor_num = SnrNum_SnrCfg_map[sensor_num];
	uint8_t chip = sensor_config[snrcfg_sensor_num].port / ADC_CHAN_NUM;
	uint8_t number = sensor_config[snrcfg_sensor_num].port % ADC_CHAN_NUM;
	int val = 1;
	int ret = adc_read_mv(chip, number, &val);
	if (ret) {
		val = val * sensor_config[snrcfg_sensor_num].arg0 /
		      sensor_config[snrcfg_sensor_num].arg1;
		if (sensor_num == SENSOR_NUM_CUR_P12V_FAN) {
			val = (float)val / 0.22 / 0.665;
		}
		*reading = (calculate_MBR(sensor_num, val) / 1000) & 0xFF;
		sensor_config[snrcfg_sensor_num].cache = *reading;
		sensor_config[snrcfg_sensor_num].cache_status = SNR_READ_SUCCESS;
		return true;
	} else {
		return false;
	}
}
