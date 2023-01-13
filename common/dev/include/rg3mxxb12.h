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

#define RG3MXXB12_DEFAULT_STATIC_ADDRESS 0x70

#define RG3MXXB12_PROTECTION_REG 0x10
#define RG3MXXB12_MASTER_PORT_CONFIG 0x11
#define RG3MXXB12_SLAVE_PORT_ENABLE 0x12
#define RG3MXXB12_HUB_NETWORK_OPERATION_MODE 0x15
#define RG3MXXB12_VOLT_LDO_SETTING 0x16
#define RG3MXXB12_SSPORTS_OD_ONLY 0x17
#define RG3MXXB12_SSPORTS_AGENT_ENABLE 0x18
#define RG3MXXB12_SSPORTS_GPIO_ENABLE 0x1E
#define RG3MXXB12_SSPORTS_HUB_NETWORK_CONNECTION 0x51
#define RG3MXXB12_SSPORTS_PULLUP_ENABLE 0x53

/* 0x10 : Unlock Device Configuration Protection Code */
#define PROTECTION_LOCK 0x00
#define PROTECTION_UNLOCK 0x69

/* 0x15 : Master Side Port Configuration */
#define HUB_NETWORK_ALWAYS_I3C BIT(5)

/* 0x16 : Interface Voltage LDO Setting */
#define VIOS1_OFFSET 6
#define VIOS0_OFFSET 4
#define VIOM1_OFFSET 2
#define VIOM0_OFFSET 0

/* 0x17 : Master Side Port Configuration */
#define MPORT_OD_ONLY BIT(4)

#define RG3MXXB12_SSPORTS_ALL_DISCONNECT 0x00

enum ldo_volt {
	ldo_1_0_volt = 0,
	ldo_1_1_volt,
	ldo_1_2_volt,
	ldo_1_8_volt,
};

bool rg3mxxb12_i2c_mode_only_init(uint8_t bus, uint8_t slave_port);
bool rg3mxxb12_select_slave_port_connect(uint8_t bus, uint8_t slave_port);

#endif
