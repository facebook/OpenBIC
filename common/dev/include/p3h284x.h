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

#ifndef P3H284X_H
#define P3H284X_H

#include "hal_i2c.h"
#include "hal_i3c.h"

#define P3H284X_DEFAULT_STATIC_ADDRESS 0x70

#define P3H284X_DEVICE_INFO0_REG 0x0
#define P3H284X_DEVICE_INFO1_REG 0x1
#define P3H284X_PROTECTION_REG 0x10
#define P3H284X_CONTROLLER_PORT_CONFIG 0x11
#define P3H284X_TARGET_PORT_ENABLE 0x12
#define P3H284X_HUB_NETWORK_OPERATION_MODE 0x15
#define P3H284X_VOLT_LDO_SETTING 0x16
#define P3H284X_TARGET_PORTS_MODE_SETTING 0x17
#define P3H284X_TARGET_PORTS_AGENT_ENABLE 0x18
#define P3H284X_TARGET_PORTS_PULLUP_SETTING 0x19
#define P3H284X_TARGET_PORTS_GPIO_ENABLE 0x1E
#define P3H284X_TARGET_PORTSHUB_NETWORK_CONNECTION 0x51
#define P3H284X_TARGET_PORTS_PULLUP_ENABLE 0x53

/* 0x10 : Unlock Device Configuration Protection Code */
#define P3H284X_PROTECTION_LOCK 0x00
#define P3H284X_PROTECTION_UNLOCK 0x6A

/* 0x15 : Master Side Port Configuration */
#define P3H284X_HUB_NETWORK_ALWAYS_I3C 5

/* 0x16 : Interface Voltage LDO Setting */
#define TP_VCCIO1_OFFSET 6
#define TP_VCCIO0_OFFSET 4
#define CP1_OFFSET 2
#define CP0_OFFSET 0

/* 0x17 : Master Side Port Configuration */
#define CP_OD_ONLY 4

/* 0x19 : Pull-up Resistor Value */
#define TARGET_PORTS_VCCIO0_PULLUP_OFFSET 6
#define TARGET_PORTS_VCCIO1_PULLUP_OFFSET 4

#define P3H284X_SSPORTS_ALL_DISCONNECT 0x00

enum p3g284x_ldo_volt {
	p3g284x_ldo_1_0_volt = 0,
	p3g284x_ldo_1_1_volt,
	p3g284x_ldo_1_2_volt,
	p3g284x_ldo_1_8_volt,
};

enum p3g284x_pull_up_resistor {
	p3g284x_pullup_250_ohm = 0,
	p3g284x_pullup_500_ohm,
	p3g284x_pullup_1k_ohm,
	p3g284x_pullup_2k_ohm,
};

bool p3h284x_i2c_mode_only_init(uint8_t bus, uint8_t slave_port, uint8_t ldo_volt,
				uint8_t pullup_resistor);
bool p3h284x_select_slave_port_connect(uint8_t bus, uint8_t slave_port);
bool p3h284x_i3c_mode_only_init(I3C_MSG *i3c_msg, uint8_t ldo_volt);
bool p3h284x_set_slave_port(uint8_t bus, uint8_t addr, uint8_t setting);
bool p3h284x_get_device_info(uint8_t bus, uint16_t *i3c_hub_type);
bool p3h284x_get_device_info_i3c(uint8_t bus, uint16_t *i3c_hub_type);

#endif
