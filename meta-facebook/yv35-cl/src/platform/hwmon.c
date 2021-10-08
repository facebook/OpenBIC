#include "plat_func.h"
#include "plat_gpio.h"

static bool is_DC_on;


void ISR_slp3(uint32_t tmp0, uint32_t tmp1) {
  printk("slp3\n");
}

void ISR_DC_on() {
  set_DC_status();
}

void set_DC_status() {
  is_DC_on = gpio_get(PWRGD_SYS_PWROK);
  printk("set dc status %d\n", is_DC_on);
}

bool get_DC_status() {
  return is_DC_on;
}
