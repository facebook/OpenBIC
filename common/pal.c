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

__weak bool pal_load_IPMB_config(void)
{
	return 0;
}

// fru
__weak void pal_load_fru_config(void)
{
	return;
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
