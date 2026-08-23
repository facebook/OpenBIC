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
 *
 * Minimal link-completeness stubs for genuinely unreachable/unportable
 * dependencies - see README.md's "What's stubbed, and why" section for
 * the full reasoning behind each one. Nothing here is ever actually
 * exercised at runtime (empty sensor table), so these only need to
 * satisfy the linker honestly (fail/no-op), not behave correctly.
 */

#include <stdint.h>
#include "sensor.h"

/* cx7.c (CXL retimer PLDM-over-MCTP sensor) needs
 * common/service/pldm/pldm_monitor.c's pldm_platform_monitor_read(),
 * which itself needs common/hal/hal_gpio.c - a hand-rolled,
 * register-level GPIO driver hardcoded to Aspeed/NPCM4XX physical
 * addresses (REG_GPIO_BASE etc.), not a portable Zephyr GPIO wrapper.
 * Porting it means writing a brand-new low-level GPIO driver for
 * MCXN947's own register layout - real driver development, not a
 * mechanical fix, and out of scope here. Same reasoning excludes
 * nv_satmc.c/mpro.c/pm8702.c/vistara.c (also pldm_monitor.c
 * consumers) from this build entirely; cx7_init is the only one of
 * the five actually required by sensor.c's dispatch table.
 */
uint8_t cx7_init(sensor_cfg *cfg)
{
	(void)cfg;
	return SENSOR_INIT_UNSPECIFIED_ERROR;
}

/* intel_peci.c's peci_read()/peci_init() are OpenBIC/Aspeed-fork
 * convenience wrappers (distinct from mainline Zephyr's own
 * peci_transfer()/peci_enable() API) with no mainline equivalent.
 * Unlike the I2C/I3C target-mode gaps, this one is a genuine hardware
 * absence, not a missing driver: MCXN947 has no PECI peripheral at
 * all (PECI is an Intel-platform-specific interface), so there is no
 * real implementation possible here regardless of driver support.
 */
int peci_init(void)
{
	return -1;
}

int peci_read(uint8_t command, uint8_t addr, uint8_t index, uint16_t param, uint8_t readlen,
	      uint8_t *readbuf)
{
	(void)command;
	(void)addr;
	(void)index;
	(void)param;
	(void)readlen;
	(void)readbuf;
	return -1;
}
