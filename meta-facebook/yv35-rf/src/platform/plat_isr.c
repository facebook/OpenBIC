#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include "plat_power_seq.h"
#include "power_status.h"
#include "plat_gpio.h"
#include "plat_isr.h"

void ISR_DC_STATE()
{
	set_DC_status(PWRGD_CARD_PWROK);
}

void ISR_MB_RST()
{
	if (gpio_get(RST_MB_N) == HIGH_ACTIVE) {
		// Enable ASIC reset pin
		gpio_set(ASIC_PERST0_N, HIGH_ACTIVE);
	} else {
		// Disable ASIC reset pin
		gpio_set(ASIC_PERST0_N, HIGH_DEACTIVE);
	}
}
