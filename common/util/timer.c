#include "cmsis_os2.h"
#include <stdio.h>

static uint32_t sys_tick_freq;

uint32_t util_get_us_tick(uint32_t time)
{
	return time * sys_tick_freq / 1000000;
}

uint32_t util_get_ms_tick(uint32_t time)
{
	return time * sys_tick_freq / 1000;
}

uint32_t util_get_s_tick(uint32_t time)
{
	return time * sys_tick_freq;
}

void util_init_timer(void)
{
	sys_tick_freq = osKernelGetSysTimerFreq(); // get sys tick per second
	return;
}
