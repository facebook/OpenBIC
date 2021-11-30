#ifndef _pal_H
#define _pal_H

#include "ipmb.h"

// USB
void pal_usb_handler(uint8_t *rx_buff, int rx_len);

// IPMI
bool pal_is_to_ipmi_handler(uint8_t netfn, uint8_t cmd);
bool pal_is_not_return_cmd(uint8_t netfn, uint8_t cmd);

// IPMI CHASSIS
void pal_CHASSIS_GET_CHASSIS_STATUS(ipmi_msg *msg);

// IPMI SENSOR
void pal_SENSOR_GET_SENSOR_READING(ipmi_msg *msg);

// IPMI APP
void pal_APP_GET_DEVICE_ID(ipmi_msg *msg);
void pal_APP_WARM_RESET(ipmi_msg *msg);
void pal_APP_GET_SELFTEST_RESULTS(ipmi_msg *msg);
void pal_APP_GET_SYSTEM_GUID(ipmi_msg *msg);
void pal_APP_MASTER_WRITE_READ(ipmi_msg *msg);

// IPMI STORAGE
void pal_STORAGE_GET_FRUID_INFO(ipmi_msg *msg);
void pal_STORAGE_READ_FRUID_DATA(ipmi_msg *msg);
void pal_STORAGE_WRITE_FRUID_DATA(ipmi_msg *msg);
void pal_STORAGE_RSV_SDR(ipmi_msg *msg);
void pal_STORAGE_GET_SDR(ipmi_msg *msg);

// IPMI OEM
void pal_OEM_MSG_OUT(ipmi_msg *msg);
void pal_OEM_GET_GPIO(ipmi_msg *msg);
void pal_OEM_SET_GPIO(ipmi_msg *msg);
void pal_OEM_SENSOR_POLL_EN(ipmi_msg *msg);
void pal_OEM_FW_UPDATE(ipmi_msg *msg);
void pal_OEM_GET_FW_VERSION(ipmi_msg *msg);
void pal_OEM_ACCURACY_SENSNR(ipmi_msg *msg);
void pal_OEM_GET_SET_GPIO(ipmi_msg *msg);
void pal_OEM_SET_SYSTEM_GUID(ipmi_msg *msg);
void pal_OEM_I2C_DEV_SCAN(ipmi_msg *msg);

// init
void pal_I2C_init(void);
void pal_BIC_init(void);
bool pal_load_IPMB_config(void);

// sensor
void pal_set_sensor_poll_interval(int *interval_ms);

// sensor accessible
uint8_t pal_load_sdr_table(void);
bool pal_load_snr_config(void);

// fru
void pal_load_fru_config(void);

// sensor read
bool pal_tmp75_read(uint8_t sensor_num, int *reading);
bool pal_adc_read(uint8_t sensor_num, int *reading);
bool pal_peci_read(uint8_t sensor_num, int *reading);
bool pal_vr_read(uint8_t sensor_num, int *reading);
bool pal_pch_read(uint8_t sensor_num, int *reading);
bool pal_hsc_read(uint8_t sensor_num, int *reading);
bool pal_nvme_read(uint8_t sensor_num, int *reading);

// gpio
bool pal_load_gpio_config(void);
void gpio_AD_callback_handler(uint32_t pins);
void gpio_EH_callback_handler(uint32_t pins);
void gpio_IL_callback_handler(uint32_t pins);
void gpio_MP_callback_handler(uint32_t pins);
void gpio_QT_callback_handler(uint32_t pins);
void gpio_UV_callback_handler(uint32_t pins);
#endif
