#ifndef POWER_STATUS_H
#define POWER_STATUS_H

#include <stdbool.h>
#include <stdint.h>

void set_DC_status(uint8_t gpio_num);
bool get_DC_status();
void set_DC_on_delayed_status();
bool get_DC_on_delayed_status();
void set_DC_off_delayed_status();
bool get_DC_off_delayed_status();
void set_post_status(uint8_t gpio_num);
bool get_post_status();
void set_CPU_power_status(uint8_t gpio_num);
bool CPU_power_good();
void set_post_thread();

#endif
