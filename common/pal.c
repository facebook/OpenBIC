#include <stdio.h>
#include <stdbool.h>
#include "cmsis_os2.h"
#include "ipmi.h"
#include "pal.h"

/***********************************************************
*
* Create weak function here
* All weak functions should be define in project for usage
*
* *********************************************************/

// USB
__weak void pal_usb_handler(uint8_t *rx_buff, int rx_len)
{
	return;
}

// IPMI

__weak bool pal_is_to_ipmi_handler(uint8_t netfn, uint8_t cmd)
{
	return 0;
}

__weak bool pal_ME_is_to_ipmi_handler(uint8_t netfn, uint8_t cmd)
{
	return 0;
}

__weak bool pal_is_not_return_cmd(uint8_t netfn, uint8_t cmd)
{
	return 0;
}

// IPMI CHASSIS
__weak void pal_CHASSIS_GET_CHASSIS_STATUS(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

// IPMI SENSOR
__weak void pal_SENSOR_GET_SENSOR_READING(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

// IPMI APP
__weak void pal_APP_GET_DEVICE_ID(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_APP_COLD_RESET(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_APP_WARM_RESET(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_APP_GET_SELFTEST_RESULTS(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_APP_GET_SYSTEM_GUID(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_APP_MASTER_WRITE_READ(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

// IPMI STORAGE
__weak void pal_STORAGE_GET_FRUID_INFO(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_STORAGE_READ_FRUID_DATA(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_STORAGE_WRITE_FRUID_DATA(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_STORAGE_RSV_SDR(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_STORAGE_GET_SDR(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

// IPMI OEM
__weak void pal_OEM_SENSOR_READ(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_SET_SYSTEM_GUID(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_GET_MB_INDEX(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_SET_FAN_DUTY_MANUAL(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_GET_SET_FAN_CTRL_MODE(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

// IPMI OEM 1S
__weak void pal_OEM_1S_MSG_OUT(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_GET_GPIO(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_SET_GPIO(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_SEND_INTERRUPT_TO_BMC(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_SENSOR_POLL_EN(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_FW_UPDATE(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_GET_POST_CODE(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_SET_VR_MONITOR_STATUS(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_GET_VR_MONITOR_STATUS(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_GET_FW_VERSION(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_RESET_BMC(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_SET_JTAG_TAP_STA(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_JTAG_DATA_SHIFT(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_ASD_INIT(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_ACCURACY_SENSOR_READING(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_PECIaccess(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_GET_SET_GPIO(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_I2C_DEV_SCAN(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_GET_BIC_STATUS(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_RESET_BIC(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_12V_CYCLE_SLOT(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_READ_BIC_REGISTER(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_WRITE_BIC_REGISTER(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_SET_FAN_DUTY_AUTO(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_GET_FAN_DUTY(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

__weak void pal_OEM_1S_GET_FAN_RPM(ipmi_msg *msg)
{
	msg->completion_code = CC_UNSPECIFIED_ERROR;
	return;
}

// init
__weak void pal_I2C_init(void)
{
	return;
}

__weak void pal_BIC_init(void)
{
	return;
}

__weak bool pal_load_IPMB_config(void)
{
	return 0;
}

// sensor

__weak void pal_set_sensor_poll_interval(int *interval_ms)
{
	*interval_ms = 1000;
	return;
}

// sensor accessible
__weak uint8_t pal_load_sdr_table(void)
{
	return 0;
}

__weak bool pal_load_sensor_config(void)
{
	return 0;
}

__weak void pal_fix_fullSDR_table(void)
{
	return;
}

__weak void pal_fix_Sensorconfig(void)
{
	return;
}

// fru
__weak void pal_load_fru_config(void)
{
	return;
}

// sensor read
__weak bool pal_tmp75_read(uint8_t sensor_num, int *reading)
{
	return 0;
}

__weak bool pal_adc_read(uint8_t sensor_num, int *reading)
{
	return 0;
}

__weak bool pal_peci_read(uint8_t sensor_num, int *reading)
{
	return 0;
}

__weak bool pal_vr_read(uint8_t sensor_num, int *reading)
{
	return 0;
}

__weak bool pal_pch_read(uint8_t sensor_num, int *reading)
{
	return 0;
}

__weak bool pal_hsc_read(uint8_t sensor_num, int *reading)
{
	return 0;
}

__weak bool pal_nvme_read(uint8_t sensor_num, int *reading)
{
	return 0;
}

// gpio
__weak bool pal_load_gpio_config(void)
{
	return 0;
}

__weak void gpio_AD_callback_handler(uint32_t pins)
{
	return;
}

__weak void gpio_EH_callback_handler(uint32_t pins)
{
	return;
}

__weak void gpio_IL_callback_handler(uint32_t pins)
{
	return;
}

__weak void gpio_MP_callback_handler(uint32_t pins)
{
	return;
}

__weak void gpio_QT_callback_handler(uint32_t pins)
{
	return;
}

__weak void gpio_UV_callback_handler(uint32_t pins)
{
	return;
}

// platform
__weak void pal_warm_reset_prepare()
{
	return;
}

__weak void pal_cold_reset_prepare()
{
	return;
}
