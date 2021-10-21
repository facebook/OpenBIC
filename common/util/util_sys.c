#include <zephyr.h>
#include <sys/reboot.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "hal_gpio.h"
#include "pal.h"


/* bic warm reset work */
#define bic_warm_reset_delay 100

void bic_warm_reset() {
  k_msleep(bic_warm_reset_delay);
  sys_reboot(0); // arvg unused
}

K_WORK_DEFINE(bic_warm_reset_work, bic_warm_reset);
void submit_bic_warm_reset() {
  k_work_submit(&bic_warm_reset_work);
}
/* bic warm reset work */
