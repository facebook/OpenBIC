#ifndef ADC_H
#define ADC_H

#include <stdbool.h>
#include <stdint.h>

enum {
	ADC_PORT0 = 0,
	ADC_PORT1,
	ADC_PORT2,
	ADC_PORT3,
	ADC_PORT4,
	ADC_PORT5,
	ADC_PORT6,
	ADC_PORT7,
	ADC_PORT8,
	ADC_PORT9,
	ADC_PORT10,
	ADC_PORT11,
	ADC_PORT12,
	ADC_PORT13,
	ADC_PORT14,
	ADC_PORT15,
};

int bat_3v_set_gpio(uint8_t sensor_num, void *arg);
bool adc_init();
bool adc_sensor_read(uint8_t sensor_num, float *reading);

#endif
