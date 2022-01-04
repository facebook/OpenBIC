#ifndef SENSOR_DEF_H
#define SENSOR_DEF_H

enum {
  adc_port0 = 0,
  adc_port1,
  adc_port2,
  adc_port3,
  adc_port4,
  adc_port5,
  adc_port6,
  adc_port7,
  adc_port8,
  adc_port9,
  adc_port10,
  adc_port11,
  adc_port12,
  adc_port13,
  adc_port14,
  adc_port15,
};

/*  define config for sensors  */
#define tmp75_in_addr          (0x9C >> 1)
#define tmp75_out_addr         (0x9E >> 1)
#define HSC_addr               (0x80 >> 1)
#define medusa_addr            (0x88 >> 1)
#define fan_addr               0x00

#define TMP75_TEMP_OFFSET           0x00
#define HSC_VOL_OFFSET              0x88
#define HSC_CUR_OFFSET              0x8C
#define HSC_TEMP_OFFSET             0x8D
#define HSC_PWR_OFFSET              0x97
#define MEDUSA_VOL_OUT_OFFSET       0x3A
#define MEDUSA_VOL_IN_OFFSET        0x3A
#define MEDUSA_CUR_OFFSET           0x40
#define MEDUSA_PWR_OFFSET           0x46

/* Temperature sensor number, 1 based  */
#define SENSOR_NUM_TEMP_TMP75_IN          0xD1
#define SENSOR_NUM_TEMP_TMP75_OUT         0xD2
#define SENSOR_NUM_TEMP_HSC               0xD3

/* Voltage sensor number */
#define SENSOR_NUM_VOL_P5V_STBY           0xD4
#define SENSOR_NUM_VOL_P12V_STBY          0xD5
#define SENSOR_NUM_VOL_P3V3_STBY          0xD6
#define SENSOR_NUM_VOL_P5V_USB            0xD7
#define SENSOR_NUM_VOL_P1V2_BIC_STBY      0xD8
#define SENSOR_NUM_VOL_P1V0_STBY          0xD9
#define SENSOR_NUM_VOL_MEDUSA_12V_IN      0xDA
#define SENSOR_NUM_VOL_MEDUSA_12V_OUT     0xDB
#define SENSOR_NUM_VOL_HSCIN              0xDC

/* Power sensor number */
#define SENSOR_NUM_PWR_MEDUSA_12V         0xDD
#define SENSOR_NUM_PWR_HSCIN              0xDE

/* Current sensor number */
#define SENSOR_NUM_CUR_MEDUSA_IOUT        0xDF
#define SENSOR_NUM_CUR_HSCOUT             0xE0
#define SENSOR_NUM_CUR_P12V_FAN           0xE1

/* Fan sensor number */
#define SENSOR_NUM_DUAL_FAN_BMC_TACH_0    0xE2
#define SENSOR_NUM_DUAL_FAN_BMC_TACH_1    0xE3
#define SENSOR_NUM_DUAL_FAN_BMC_TACH_2    0xE4
#define SENSOR_NUM_DUAL_FAN_BMC_TACH_3    0xE5
#define SENSOR_NUM_DUAL_FAN_BMC_TACH_4    0xE6
#define SENSOR_NUM_DUAL_FAN_BMC_TACH_5    0xE7
#define SENSOR_NUM_DUAL_FAN_BMC_TACH_6    0xE8
#define SENSOR_NUM_DUAL_FAN_BMC_TACH_7    0xE9
#define SENSOR_NUM_SINGLE_FAN_BMC_TACH_0  0xEA
#define SENSOR_NUM_SINGLE_FAN_BMC_TACH_1  0xEB
#define SENSOR_NUM_SINGLE_FAN_BMC_TACH_2  0xEC
#define SENSOR_NUM_SINGLE_FAN_BMC_TACH_3  0xED

#define HSC_DEVICE_READY_DELAY_ms 2000

#endif

