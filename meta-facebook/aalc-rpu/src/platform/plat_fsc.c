#include "plat_fsc.h"
#include <logging/log.h>

static uint8_t fsc_enable_flag = 1;

uint8_t get_fsc_enable_flag(void)
{
	return fsc_enable_flag;
}

void set_fsc_enable_flag(uint8_t flag)
{
	fsc_enable_flag = flag;
}

LOG_MODULE_REGISTER(plat_fsc);