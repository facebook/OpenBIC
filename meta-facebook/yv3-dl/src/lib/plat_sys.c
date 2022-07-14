#include "plat_sys.h"

#include "util_sys.h"
#include "hal_gpio.h"
#include "plat_gpio.h"

/* BMC reset */
void BMC_reset_handler()
{
	gpio_set(RST_BMC_R_N, GPIO_LOW);
	k_msleep(10);
	gpio_set(RST_BMC_R_N, GPIO_HIGH);
}

K_WORK_DELAYABLE_DEFINE(BMC_reset_work, BMC_reset_handler);
int pal_submit_bmc_cold_reset()
{
	k_work_schedule(&BMC_reset_work, K_MSEC(1000));
	return 0;
}
/* BMC reset */

void clear_cmos_handler()
{
	gpio_set(FM_BIC_RST_RTCRST, GPIO_HIGH);
	k_msleep(200);
	gpio_set(FM_BIC_RST_RTCRST, GPIO_LOW);
}

K_WORK_DEFINE(clear_cmos_work, clear_cmos_handler);
int pal_clear_cmos()
{
	k_work_submit(&clear_cmos_work);
	return 0;
}