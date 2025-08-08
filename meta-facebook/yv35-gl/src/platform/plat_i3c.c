#include <stdio.h>
#include <logging/log.h>
#include "plat_i3c.h"
#include "plat_class.h"
#include "rg3mxxb12.h"
#include "p3h284x.h"
#include <errno.h>

LOG_MODULE_REGISTER(plat_i3c);

const uint8_t rg3mxxb12_cmd_initial[][2] = {
	{ RG3MXXB12_VOLT_LDO_SETTING, LDO_VOLT },
	{ RG3MXXB12_SSPORTS_AGENT_ENABLE, 0x0 },
	{ RG3MXXB12_SSPORTS_GPIO_ENABLE, 0x0 },
	{ RG3MXXB12_SLAVE_PORT_ENABLE, 0x0 },
	{ RG3MXXB12_SSPORTS_PULLUP_SETTING, rg3mxxb12_pullup_500_ohm },
	{ RG3MXXB12_SSPORTS_PULLUP_ENABLE, 0xFF },
	{ RG3MXXB12_SSPORTS_OD_ONLY, 0x0 },
	{ RG3MXXB12_SLAVE_PORT_ENABLE, 0xFF },
	{ RG3MXXB12_SSPORTS_HUB_NETWORK_CONNECTION, 0xFF },
};

const uint8_t p3h284x_cmd_initial[][2] = {
	// VCCIO LDO Setting
	{ P3H284X_VOLT_LDO_SETTING, P3H2840_I3C_LDO_VOLT },
	// On-die LDO Enable and Pull up Resistor Value Setting
	{ P3H284X_TARGET_PORTS_PULLUP_SETTING, 0xA0 },
	// Target Port Enable
	{ P3H284X_TARGET_PORT_ENABLE, 0xC5 },
	// Target Port Hub Network Connection Enable
	{ P3H284X_TARGET_PORTSHUB_NETWORK_CONNECTION, 0xC5 },
	// Target Port Pull Up Enable
	{ P3H284X_TARGET_PORTS_PULLUP_ENABLE, 0xFF },
};

void init_i3c_hub()
{
	I3C_MSG i3c_msg = { 0 };
	i3c_msg.bus = I3C_BUS4;
	uint16_t i3c_hub_type = I3C_HUB_TYPE_UNKNOWN;

	int ret = 0;
	int i;
	for (i = 0; i < RSTDAA_COUNT; i++) {
		ret = i3c_brocast_ccc(&i3c_msg, I3C_CCC_RSTDAA, I3C_BROADCAST_ADDR);
		if (ret != 0) {
			LOG_ERR("Error to reset daa. count = %d", i);
		}
	}

	ret = i3c_brocast_ccc(&i3c_msg, I3C_CCC_SETAASA, I3C_BROADCAST_ADDR);
	if (ret != 0) {
		LOG_ERR("Error to set daa");
	}

	i3c_msg.target_addr = RG3MXXB12_DEFAULT_STATIC_ADDRESS;
	i3c_attach(&i3c_msg);
	init_i3c_hub_type();
	i3c_hub_type = get_i3c_hub_type();

	switch (i3c_hub_type) {
	case RG3M87B12_DEVICE_INFO:
		if (!rg3mxxb12_i3c_mode_only_init(&i3c_msg, rg3mxxb12_cmd_initial,
						  RG3MXXB12_CMD_INITIAL_SIZE)) {
			LOG_ERR("Failed to initialize rg3mxxb12 i3c hub");
		}

		if (!rg3mxxb12_set_slave_port(I3C_BUS4, RG3MXXB12_DEFAULT_STATIC_ADDRESS,
					      DEFAULT_SLAVE_PORT_SETTING)) {
			LOG_ERR("Error to set rg3mxxb12 i3c hub slave port");
		}
		break;
	case P3H2840_DEVICE_INFO:
		if (!p3h284x_i3c_mode_only_init(&i3c_msg, p3h284x_cmd_initial,
						P3H284X_CMD_INITIAL_SIZE)) {
			LOG_ERR("Failed to initialize p3h284x i3c hub");
		}

		if (!p3h284x_set_slave_port(I3C_BUS4, P3H284X_DEFAULT_STATIC_ADDRESS,
					    DEFAULT_SLAVE_PORT_SETTING)) {
			LOG_ERR("Error to set p3h284x i3c hub slave port");
		}
		break;
	default:
		LOG_ERR("Get i3c hub type failed - initialize I3C HUB");
		break;
	}
}
