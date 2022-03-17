#ifndef UTIL_SYS_H
#define UTIL_SYS_H

#include <zephyr.h>

void set_boot_source();
bool get_boot_source_ACon();
void submit_bic_warm_reset();
void submit_bic_cold_reset();

#endif
