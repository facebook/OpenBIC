/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RG3MXXB12_H
#define RG3MXXB12_H

#include "hal_i2c.h"
#include "hal_i3c.h"

#define V_LDO_SETTING(vios1, vios0, viom1, viom0)                                                  \
	((vios1 << VIOS1_OFFSET) | (vios0 << VIOS0_OFFSET) | (viom1 << VIOM1_OFFSET) |             \
	 (viom0 << VIOM0_OFFSET))

#define RG3MXXB12_DEFAULT_STATIC_ADDRESS 0x70
#define RG3M87B12_DEVICE_INFO 0x1287
#define RG3M88B12_DEVICE_INFO 0x1288
#define RG3M47B12_DEVICE_INFO 0x1247
#define RG3M4812_DEVICE_INFO 0x1248

#define RG3MXXB12_DEVICE_INFO0_REG 0x0
#define RG3MXXB12_DEVICE_INFO1_REG 0x1
#define RG3MXXB12_PROTECTION_REG 0x10
#define RG3MXXB12_MASTER_PORT_CONFIG 0x11
#define RG3MXXB12_SLAVE_PORT_ENABLE 0x12
#define RG3MXXB12_HUB_NETWORK_OPERATION_MODE 0x15
#define RG3MXXB12_VOLT_LDO_SETTING 0x16
#define RG3MXXB12_SSPORTS_OD_ONLY 0x17
#define RG3MXXB12_SSPORTS_AGENT_ENABLE 0x18
#define RG3MXXB12_SSPORTS_PULLUP_SETTING 0x19
#define RG3MXXB12_SSPORTS_GPIO_ENABLE 0x1E
#define RG3MXXB12_SSPORTS_HUB_NETWORK_CONNECTION 0x51
#define RG3MXXB12_SSPORTS_PULLUP_ENABLE 0x53

/* 0x10 : Unlock Device Configuration Protection Code */
#define RG3MXXB12_PROTECTION_LOCK 0x00
#define RG3MXXB12_PROTECTION_UNLOCK 0x69

/* 0x15 : Master Side Port Configuration */
#define RG3MXXB12_HUB_NETWORK_ALWAYS_I3C 5

/* 0x16 : Interface Voltage LDO Setting */
#define VIOS1_OFFSET 6
#define VIOS0_OFFSET 4
#define VIOM1_OFFSET 2
#define VIOM0_OFFSET 0

/* 0x11 : Master Side Port Configuration */
#define MPORT_OD_ONLY 4

/* 0x19 : Pull-up Resistor Value */
#define SSPORTS_RESISTOR0_OFFSET 6
#define SSPORTS_RESISTOR1_OFFSET 4

#define RG3MXXB12_SSPORTS_ALL_DISCONNECT 0x00

enum rg3mxxb12_ldo_volt {
	rg3mxxb12_ldo_1_0_volt = 0,
	rg3mxxb12_ldo_1_1_volt,
	rg3mxxb12_ldo_1_2_volt,
	rg3mxxb12_ldo_1_8_volt,
};

enum rg3mxxb12_pull_up_resistor {
	rg3mxxb12_pullup_250_ohm = 0,
	rg3mxxb12_pullup_500_ohm,
	rg3mxxb12_pullup_1k_ohm,
	rg3mxxb12_pullup_2k_ohm,
};

bool rg3mxxb12_i2c_mode_only_init(uint8_t bus, uint8_t slave_port, uint8_t ldo_volt,
				  uint8_t pullup_resistor);
bool rg3mxxb12_select_slave_port_connect(uint8_t bus, uint8_t slave_port);
bool rg3mxxb12_i3c_mode_only_init(I3C_MSG *i3c_msg, uint8_t ldo_volt);
bool rg3mxxb12_set_slave_port(uint8_t bus, uint8_t addr, uint8_t setting);
bool rg3mxxb12_get_device_info(uint8_t bus, uint16_t *i3c_hub_type);

#endif
