#include <zephyr.h>
#include "plat_gpio.h"


/* BMC reset */
void BMC_reset_handler() {
  gpio_set(RST_BMC_R_N, GPIO_LOW);
  k_msleep(10);
  gpio_set(RST_BMC_R_N, GPIO_HIGH);
}

K_DELAYED_WORK_DEFINE(BMC_reset_work, BMC_reset_handler);
void submit_bmc_warm_reset() {
  k_work_schedule(&BMC_reset_work, K_MSEC(1000));
}
/* BMC reset */
