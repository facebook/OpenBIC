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

// init
__weak void pal_I2C_init(void)
{
	return;
}

__weak void pal_BIC_init(void)
{
	return;
}

__weak bool pal_load_ipmb_config(void)
{
	return false;
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

__weak bool pal_load_snr_config(void)
{
	return false;
}

__weak void pal_fix_fullSDR_table(void)
{
	return;
}

__weak void pal_fix_Snrconfig(void)
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
	return false;
}

__weak bool pal_adc_read(uint8_t sensor_num, int *reading)
{
	return false;
}

__weak bool pal_peci_read(uint8_t sensor_num, int *reading)
{
	return false;
}

__weak bool pal_vr_read(uint8_t sensor_num, int *reading)
{
	return false;
}

__weak bool pal_pch_read(uint8_t sensor_num, int *reading)
{
	return false;
}

__weak bool pal_hsc_read(uint8_t sensor_num, int *reading)
{
	return false;
}

__weak bool pal_nvme_read(uint8_t sensor_num, int *reading)
{
	return false;
}

// gpio
__weak bool pal_load_gpio_config(void)
{
	return false;
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
