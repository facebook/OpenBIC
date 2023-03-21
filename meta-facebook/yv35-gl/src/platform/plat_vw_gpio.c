#include "plat_vw_gpio.h"
#include "plat_isr.h"

vw_gpio plat_vw_gpio_cfg[] = {
	{ 0, VW_GPIO_ENABLE, VW_GPIO_INPUT, VW_GPIO_UNKNOWN, ISR_POST_COMPLETE },
	{ 2, VW_GPIO_ENABLE, VW_GPIO_INPUT, VW_GPIO_UNKNOWN, ISR_FM_ADR_MODE0 },
};

bool pal_load_vw_gpio_config(void)
{
	return vw_gpio_init(plat_vw_gpio_cfg, ARRAY_SIZE(plat_vw_gpio_cfg));
};
