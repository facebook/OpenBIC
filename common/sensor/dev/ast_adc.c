#include <stdio.h>
#include "sensor.h"
#include "sensor_def.h"
#include "pal.h"
#include "plat_gpio.h"
#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/adc.h>

enum adc_device_idx{
  adc0,
  adc1,
  ADC_NUM
};

#define ADC_CHAN_NUM 8
#define BUFFER_SIZE 1

#if DT_NODE_EXISTS(DT_NODELABEL(adc0))
#define DEV_ADC0
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(adc1))
#define DEV_ADC1
#endif

#define ADC_RESOLUTION          10
#define ADC_CALIBRATE           0
#define ADC_GAIN                ADC_GAIN_1
#define ADC_REFERENCE           ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME    ADC_ACQ_TIME_DEFAULT

static const struct device *dev_adc[ADC_NUM];
static int16_t sample_buffer[BUFFER_SIZE];
static int is_ready[2];

static void init_adc_dev() 
{
#ifdef DEV_ADC0
  dev_adc[adc0] = device_get_binding("ADC0");
  if ( !( device_is_ready(dev_adc[adc0]) ) )
    printk("<warn> ADC[%d] device not ready!\n", adc0);
  else
    is_ready[adc0] = 1;
#endif

#ifdef DEV_ADC1
  dev_adc[adc1] = device_get_binding("ADC1");
  if ( !( device_is_ready(dev_adc[adc1]) ) )
    printk("<warn> ADC[%d] device not ready!\n", adc1);
  else
    is_ready[adc1] = 1;
#endif
}

static bool adc_read_mv(uint8_t sensor_num, uint32_t index, uint32_t channel, int *adc_val)
{
  if (!adc_val)
    return false;

  if (index >= ADC_NUM) {
    printk("<error> ADC[%d] is invalid device!\n", index);
    return false;
  }

  if ( !is_ready[index] ) {
    printk("<error> ADC[%d] is not ready to read!\n", index);
    return false;
  }

  int err, retval;

  static struct adc_sequence sequence;
  sequence.channels    = BIT(channel);
  sequence.buffer      = sample_buffer;
  sequence.buffer_size = sizeof(sample_buffer);
  sequence.resolution  = ADC_RESOLUTION;
  sequence.calibrate   = ADC_CALIBRATE;

  static struct adc_channel_cfg channel_cfg;
  channel_cfg.gain = ADC_GAIN;
  channel_cfg.reference = ADC_REFERENCE;
  channel_cfg.acquisition_time = ADC_ACQUISITION_TIME;
  channel_cfg.channel_id = channel;
  channel_cfg.differential = 0;

  retval = adc_channel_setup(dev_adc[index], &channel_cfg);

  if (retval) {
    printk("<error> ADC[%d] with sensor[0x%x] channel set fail\n", index, sensor_num);
    return false;
  }

  err = adc_read(dev_adc[index], &sequence);
  if (err != 0) {
    printk("<error> ADC[%d] with sensor[0x%x] reading fail with error %d\n", index, sensor_num, err);
    return false;
  }

  int32_t raw_value = sample_buffer[0];
  int32_t ref_mv = adc_get_ref(dev_adc[index]);
  if (ref_mv <= 0) {
    printk("<error> ADC[%d] with sensor[0x%x] ref-mv get fail\n", index, sensor_num);
    return false;
  }

  *adc_val = raw_value;
  adc_raw_to_millivolts( ref_mv, channel_cfg.gain, sequence.resolution, adc_val);

  return true;
}

uint8_t ast_adc_read(uint8_t sensor_num, int *reading)
{
  if(!reading)
    return SNR_UNSPECIFIED_ERROR;

  uint8_t snrcfg_sensor_num = SnrNum_SnrCfg_map[sensor_num];
  uint8_t chip = sensor_config[snrcfg_sensor_num].port / ADC_CHAN_NUM;
  uint8_t number = sensor_config[snrcfg_sensor_num].port % ADC_CHAN_NUM;
  int val = 1;

  if ( !adc_read_mv(sensor_num, chip, number, &val) )
    return SNR_FAIL_TO_ACCESS;

  val = val * sensor_config[snrcfg_sensor_num].arg0 / sensor_config[snrcfg_sensor_num].arg1;

  sen_val *sval = (sen_val *)reading;
  sval->integer = (val/1000) & 0xFFFF;
  sval->fraction = (val%1000) & 0xFFFF;

  return SNR_READ_SUCCESS;
}

uint8_t ast_adc_init(uint8_t sensor_num)
{
  if ( !sensor_config[SnrNum_SnrCfg_map[sensor_num]].init_args ) {
    printk("<error> ADC init args not provide!\n");
    return false;
  }

  adc_asd_init_arg *init_args = (adc_asd_init_arg *) sensor_config[SnrNum_SnrCfg_map[sensor_num]].init_args;
  if ( init_args->is_init )
    goto skip_init;

  init_adc_dev();

  if (!is_ready[0] && !is_ready[1]) {
    printk("<error> Both of ADC0 and ADC1 are not ready to use!\n");
    return false;
  }

  static struct adc_channel_cfg channel_cfg;
  channel_cfg.gain = ADC_GAIN;
  channel_cfg.reference = ADC_REFERENCE;
  channel_cfg.acquisition_time = ADC_ACQUISITION_TIME;
  channel_cfg.channel_id = 0;
  channel_cfg.differential = 0;

  static struct adc_sequence sequence;
  sequence.channels    = 0;
  sequence.buffer      = sample_buffer;
  sequence.buffer_size = sizeof(sample_buffer);
  sequence.resolution  = ADC_RESOLUTION;
  sequence.calibrate   = ADC_CALIBRATE;

  for (uint8_t i = 0; i < ADC_NUM; i++) {
    if (!is_ready[i])
      continue;

    channel_cfg.channel_id = i;
    adc_channel_setup(dev_adc[i], &channel_cfg);
    sequence.channels |= BIT(i);
  }

  init_args->is_init = true;

skip_init:
  sensor_config[SnrNum_SnrCfg_map[sensor_num]].read = ast_adc_read;

  return true;
}