#ifndef HAL_WDT_H
#define HAL_WDT_H

#define WDT_DEVICE_NAME "wdt2"
#define WDT_TIMEOUT (15 * 1000) // 15s
#define WDT_FEED_DELAY_MS (10 * 1000) // 10s

#define WDT_THREAD_STACK_SIZE 256

void wdt_init();
void wdt_handler(void *arug0, void *arug1, void *arug2);
void set_wdt_continue_feed(bool value);
#endif
