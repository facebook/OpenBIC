#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "hal_gpio.h"
#include "plat_gpio.h"

#define gpio_name_to_num(x) #x,

#undef gpio_name_to_num

GPIO_CFG plat_gpio_cfg[] = {
	//  chip,      number,   is_init, is_latch, direction,    status,     property,    int_type,              int_cb
};

bool pal_load_gpio_config(void)
{
	return 1;
};