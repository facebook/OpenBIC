#include "plat_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hal_gpio.h"
#include "hal_i2c.h"
#include "libutil.h"
#include "plat_gpio.h"
#include "plat_i2c.h"

static uint8_t system_class = SYS_CLASS_1;

uint8_t get_system_class()
{
	return system_class;
}

void init_platform_config()
{
	if (gpio_get(SYS_SKU_ID0) == GPIO_HIGH)
		system_class = SYS_CLASS_2;
	else
		system_class = SYS_CLASS_1;

	printf("SYS_SKU: %s Compute System\n", system_class == SYS_CLASS_2 ? "Single" : "Dual");
	printf("BRD_SKU: %s\n", gpio_get(FM_BOARD_SKU_ID0) == GPIO_HIGH ? "VR-INS" : "VR-RTT");
}
