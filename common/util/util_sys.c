#include <zephyr.h>
#include <sys/reboot.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "hal_gpio.h"
#include "pal.h"

/* get bic boot source through from SRST */
#define SYS_RST_EVT_LOG_REG 0x7e6e2074
#define SRST_POWER_ON_SET BIT(0)

static bool is_boot_ACon = 0;

void set_boot_source()
{
	uint32_t sys_rst_evt;

	sys_rst_evt = sys_read32(SYS_RST_EVT_LOG_REG);
	is_boot_ACon = sys_rst_evt & 0x1;
	sys_write32(SRST_POWER_ON_SET, SYS_RST_EVT_LOG_REG);
}

bool get_boot_source_ACon()
{
	return is_boot_ACon;
}
/* get bic boot source through from SRST */

/* bic warm reset work */
#define bic_warm_reset_delay 100

void bic_warm_reset()
{
	pal_warm_reset_prepare();
	k_msleep(bic_warm_reset_delay);
	sys_reboot(SYS_REBOOT_WARM);
}

K_WORK_DEFINE(bic_warm_reset_work, bic_warm_reset);
void submit_bic_warm_reset()
{
	k_work_submit(&bic_warm_reset_work);
}
/* bic warm reset work */

/* bic cold reset work */
#define bic_cold_reset_delay 100

void bic_cold_reset()
{
	pal_cold_reset_prepare();
	k_msleep(bic_cold_reset_delay);
	sys_reboot(SYS_REBOOT_COLD);
}

K_WORK_DEFINE(bic_cold_reset_work, bic_cold_reset);
void submit_bic_cold_reset()
{
	k_work_submit(&bic_cold_reset_work);
}
/* bic cold reset work */
