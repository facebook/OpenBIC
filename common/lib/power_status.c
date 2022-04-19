#include "power_status.h"

#include <stdio.h>

#include "hal_gpio.h"
#include "snoop.h"

static bool is_DC_on = false;
static bool is_DC_on_delayed = false;
static bool is_DC_off_delayed = false;
static bool is_CPU_power_good = false;
static bool is_post_complete = false;

void set_DC_status(uint8_t gpio_num)
{
	is_DC_on = (gpio_get(gpio_num) == 1) ? true : false;
	printf("[%s] gpio number(%d) status(%d)\n", __func__, gpio_num, is_DC_on);
}

bool get_DC_status()
{
	return is_DC_on;
}

void set_DC_on_delayed_status()
{
	is_DC_on_delayed = is_DC_on;
}

bool get_DC_on_delayed_status()
{
	return is_DC_on_delayed;
}

void set_DC_off_delayed_status()
{
	is_DC_off_delayed = !is_DC_on;
}

bool get_DC_off_delayed_status()
{
	return is_DC_off_delayed;
}

void set_post_status(uint8_t gpio_num)
{
	is_post_complete = (gpio_get(gpio_num) == 1) ? false : true;
	printf("[%s] gpio number(%d) status(%d)\n", __func__, gpio_num, is_post_complete);
}

bool get_post_status()
{
	return is_post_complete;
}

void set_CPU_power_status(uint8_t gpio_num)
{
	is_CPU_power_good = gpio_get(gpio_num);
}

bool CPU_power_good()
{
	return is_CPU_power_good;
}

void set_post_thread()
{
	if (CPU_power_good() == true) {
		init_snoop_thread();
		init_send_postcode_thread();
	}
}
