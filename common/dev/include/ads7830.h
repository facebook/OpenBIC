#ifndef _ADS7830_H_
#define _ADS7830_H_

#include <stdint.h>
#include <stdbool.h>
#include "sensor.h"

#define ADC_CH0 0x8C // CH0: SD=1, C2=0, C1=0, C0=0, PD=11
#define ADC_CH1 0xCC
#define ADC_CH2 0x9C
#define ADC_CH3 0xDC
#define ADC_CH4 0xAC
#define ADC_CH5 0xEC
#define ADC_CH6 0xBC
#define ADC_CH7 0xFC

uint8_t ads7830_init(sensor_cfg *cfg);
uint8_t ads7830_read(sensor_cfg *cfg, int *reading);

#endif /* _ADS7830_H_ */
