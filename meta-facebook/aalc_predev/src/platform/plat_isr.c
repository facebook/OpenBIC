#include "plat_isr.h"
#include "plat_gpio.h"

static void deassert_all_rpu_ready_pin(void)
{
	gpio_set(BIC_RPU_READY0, 1);
	gpio_set(BIC_RPU_READY1, 1);
	gpio_set(BIC_RPU_READY2, 1);
	gpio_set(BIC_RPU_READY3, 1);
}



/* TO DO: 
	1. turn off pump
	2. change hx fan to min speed
	3. store to non volatile memory
*/
#define IT_LEAK_ALERT_HANDLER(idx)                                                                 \
	void it_leak_alert_##idx(void)                                                             \
	{                                                                                          \
		deassert_all_rpu_ready_pin();                                                      \
		gpio_set(RPU_LEAK_ALERT_N, 0);                                                     \
	}

IT_LEAK_ALERT_HANDLER(0);
IT_LEAK_ALERT_HANDLER(1);
IT_LEAK_ALERT_HANDLER(2);
IT_LEAK_ALERT_HANDLER(3);