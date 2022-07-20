#include "plat_isr.h"
#include "plat_gpio.h"
#include "plat_sensor_table.h"
#include "libipmi.h"
#include "power_status.h"
#include "ipmi.h"

void ISR_POST_COMPLETE()
{
	set_post_status(FM_BIOS_POST_CMPLT_BIC_N);
}

K_WORK_DELAYABLE_DEFINE(set_DC_on_5s_work, set_DC_on_delayed_status);
#define DC_ON_5_SECOND 5
void ISR_DC_ON()
{
	set_DC_status(PWRGD_CPU_LVC3);
	if (get_DC_status() == true) {
		k_work_schedule(&set_DC_on_5s_work, K_SECONDS(DC_ON_5_SECOND));
	} else {
		set_DC_on_delayed_status();
	}
}
