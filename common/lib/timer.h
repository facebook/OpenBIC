#ifndef TIMER_H
#define TIMER_H

uint32_t util_get_us_tick(uint32_t time);
uint32_t util_get_ms_tick(uint32_t time);
uint32_t util_get_s_tick(uint32_t time);
void util_init_timer(void);

#endif
