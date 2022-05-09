#include "plat_sys.h"

#include "util_sys.h"
#include "hal_gpio.h"
#include "plat_gpio.h"

/* BMC reset */
void BMC_reset_handler()
{
	/* TBD */
}

K_WORK_DELAYABLE_DEFINE(BMC_reset_work, BMC_reset_handler);
int pal_submit_bmc_cold_reset()
{
	k_work_schedule(&BMC_reset_work, K_MSEC(1000));
	return 0;
}
/* BMC reset */
