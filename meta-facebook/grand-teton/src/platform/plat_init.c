#include "hal_gpio.h"
#include "hal_peci.h"
#include "power_status.h"
#include "util_sys.h"
#include "plat_class.h"
#include "plat_gpio.h"
#include "plat_i2c_slave.h"

SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e6e2618, 0x00FF0000 },
};

void pal_pre_init()
{
	/* init i2c slave */
	for (int index = 0; index < MAX_SLAVE_NUM; index++) {
		if (I2C_SLAVE_ENABLE_TABLE[index])
			i2c_slave_control(
				index, (struct _i2c_slave_config *)&I2C_SLAVE_CONFIG_TABLE[index],
				1);
	}
	scu_init(scu_cfg, sizeof(scu_cfg) / sizeof(SCU_CFG));
}

void pal_post_init()
{
	plat_mctp_init();
}

void pal_set_sys_status()
{
	gpio_set(BIC_SYS_READY_N, GPIO_LOW);
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
