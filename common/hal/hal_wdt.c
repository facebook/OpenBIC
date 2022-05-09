#include <zephyr.h>
#include <device.h>
#include <stdio.h>
#include <drivers/watchdog.h>
#include "hal_wdt.h"

struct k_thread wdt_thread;
K_KERNEL_STACK_MEMBER(wdt_thread_stack, WDT_THREAD_STACK_SIZE);

const struct device *wdt_dev = NULL;

void wdt_handler(void *arug0, void *arug1, void *arug2)
{
	while (1) {
		wdt_feed(wdt_dev, 0);
		k_sleep(K_MSEC(WDT_FEED_DELAY_MS));
	}
}

void wdt_init()
{
	int ret = 0;
	struct wdt_timeout_cfg wdt_config;

	wdt_dev = device_get_binding(WDT_DEVICE_NAME);
	if (!wdt_dev) {
		printf("%s: cannot find %s device.\n", __func__, WDT_DEVICE_NAME);
		return;
	}

	wdt_config.window.min = 0U;
	wdt_config.window.max = WDT_TIMEOUT;
	// Pass NULL to use system reset handler
	wdt_config.callback = NULL;
	// Install new timeout: This function must be used before wdt_setup().
	ret = wdt_install_timeout(wdt_dev, &wdt_config);
	if (ret != 0) {
		printf("%s: fail to install %s timeout\n", __func__, WDT_DEVICE_NAME);
		return;
	}

	//This function is used for configuring global watchdog settings that affect all timeouts.
	ret = wdt_setup(wdt_dev, WDT_FLAG_RESET_CPU_CORE);
	if (ret != 0) {
		printf("%s: fail to setup %s\n", __func__, WDT_DEVICE_NAME);
		return;
	}

	k_thread_create(&wdt_thread, wdt_thread_stack, K_THREAD_STACK_SIZEOF(wdt_thread_stack),
			wdt_handler, NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&wdt_thread, "WDT_thread");
}
