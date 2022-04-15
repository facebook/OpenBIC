#include "hal_i2c.h"
#include "timer.h"
#include "sensor.h"
#include "ipmi.h"
#include "kcs.h"
#include "usb.h"
#include "fru.h"

__weak void pal_pre_init()
{
	return;
}

__weak void pal_post_init()
{
	return;
}

__weak void pal_device_init()
{
	return;
}

__weak void pal_set_sys_status()
{
	return;
}

void main(void)
{
	printf("Hello, welcome to %s %s %x%x.%x.%x\n", PLATFORM_NAME, PROJECT_NAME,
	       BIC_FW_YEAR_MSB, BIC_FW_YEAR_LSB, BIC_FW_WEEK, BIC_FW_VER);

	util_init_timer();
	util_init_I2C();
	pal_pre_init();
	sensor_init();
	FRU_init();
	ipmi_init();
#ifdef CONFIG_IPMI_KCS_ASPEED
	kcs_init();
#endif
#ifdef CONFIG_USB
	usb_dev_init();
#endif
	pal_device_init();
	pal_set_sys_status();
	pal_post_init();
}
