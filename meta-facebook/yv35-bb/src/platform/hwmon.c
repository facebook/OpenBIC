#include <stdint.h>
#include "plat_func.h"
#include "plat_gpio.h"

void ISR_PWROK_SLOT1() {
  set_BIC_slot_isolator(PWROK_STBY_BIC_SLOT1_R, FM_BIC_SLOT1_ISOLATED_EN_R);
}

void ISR_PWROK_SLOT3() {
  set_BIC_slot_isolator(PWROK_STBY_BIC_SLOT3_R, FM_BIC_SLOT3_ISOLATED_EN_R);
}


void set_BIC_slot_isolator(uint8_t pwr_state_gpio_num, uint8_t isolator_gpio_num) {
  int ret = 0;
  uint8_t slot_pwr_status = GPIO_LOW;

  slot_pwr_status = gpio_get(pwr_state_gpio_num);
  
  if (slot_pwr_status == GPIO_HIGH) {
    ret = gpio_set(isolator_gpio_num, GPIO_HIGH);
  } else if (slot_pwr_status == GPIO_LOW) {
    ret = gpio_set(isolator_gpio_num, GPIO_LOW);
  }

  if (ret < 0) {
    printk("failed to set slot isolator due to set gpio %d is failed\n", isolator_gpio_num);
  }
}
