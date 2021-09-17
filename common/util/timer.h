#ifndef TIMER_H
#define TIMER_H

#define time_1000_ms 1000
#define time_500_ms  500
#define time_100_ms  100

uint32_t util_get_us_tick(uint32_t time);
uint32_t util_get_ms_tick(uint32_t time);
uint32_t util_get_s_tick(uint32_t time);
void util_init_timer(void);

#endif
