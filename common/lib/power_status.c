#include "power_status.h"

#include <stdio.h>

#include "hal_gpio.h"
#include "snoop.h"

static bool is_DC_on = false;
static bool is_DC_on_delayed = false;
static bool is_DC_off_delayed = false;
static bool is_post_complete = false;

void set_DC_status(uint8_t gpio_num)
{
	is_DC_on = gpio_get(gpio_num);
	printf("set dc status %d\n", is_DC_on);
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
	is_post_complete = !(gpio_get(gpio_num));
	printf("set is_post_complete %d\n", is_post_complete);

	if (is_post_complete) {
		snoop_abort_thread();
	}
}

bool get_post_status()
{
	return is_post_complete;
}

void set_post_thread()
{
	if ((get_DC_status() == 1) && (get_post_status() == 0)) {
		snoop_start_thread();
		init_send_postcode_thread();
	}
}
