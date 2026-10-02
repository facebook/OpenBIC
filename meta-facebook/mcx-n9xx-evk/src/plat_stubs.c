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

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include "sensor.h"
#include "ipmi.h"
#include "pldm.h"
#include "plat_fan.h"
#include "util_sys.h"

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

/* pldm_monitor_handler_query() (the PLDM_TYPE_PLAT_MON_CTRL entry in
 * pldm.c's query_tbl) is no longer stubbed here - src/plat_pldm_monitor.c
 * provides a real board-local implementation backed by the MCXN947
 * on-die temperature sensor (and, in later phases, the SW2 button and
 * an on-board LED). common/service/pldm/pldm_monitor.c stays excluded
 * from the build (it needs the unportable hal_gpio.c). */

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

/* oem_1s_handler.c implements Meta's proprietary OEM-1S NetFn (0x38)
 * command set - fan control, GPIO, JTAG, PECI, APML, and other
 * x86-host-adjacent BMC/BIC functionality specific to Meta's own
 * server platforms. None of it applies to this EVK, and the file
 * unconditionally includes hal_gpio.h/hal_peci.h (both genuinely
 * unportable - see hal_gpio.c's own exclusion above and README.md),
 * so it isn't compiled here. ipmi.c's dispatch table calls this
 * function unconditionally for NETFN_OEM_1S_REQ, so this link stub
 * honestly reports "not implemented" via the real completion code
 * rather than silently no-oping.
 */
void IPMI_OEM_1S_handler(ipmi_msg *msg)
{
	msg->completion_code = CC_INVALID_CMD;
	msg->data_len = 0;
}

/* This board has no fans - oem_handler.c's fan-control commands need
 * these three pal_* hooks to link, but MAX_FAN_PWM_INDEX_COUNT is 0
 * (see plat_fan.h) so they're never actually reachable through a
 * valid pwm_id; genuine hardware absence, not a missing driver.
 */
int pal_set_fan_duty(uint8_t pwm_id, uint8_t duty, uint8_t slot_index)
{
	(void)pwm_id;
	(void)duty;
	(void)slot_index;
	return -1;
}

int pal_get_fan_ctrl_mode(uint8_t *mode)
{
	(void)mode;
	return -1;
}

void pal_set_fan_ctrl_mode(uint8_t mode)
{
	(void)mode;
}

/* chassis_handler.c's CHASSIS Get Chassis Status command needs
 * get_DC_status() from common/lib/power_status.c - not built here
 * since that file needs hal_gpio.c (Aspeed-only, see above) and
 * snoop.c (also excluded). This board has no real host-power-good
 * GPIO wired up, so reporting "off" is the honest answer, not a
 * placeholder for real power sequencing logic.
 */
bool get_DC_status(void)
{
	return false;
}

/* common/lib/util_sys.c isn't built here - it unconditionally includes
 * cmsis_os.h/soc_common.h (Aspeed-fork-only headers with no mainline
 * equivalent anywhere in this tree) and hal_gpio.h alongside its cold/
 * warm reset logic. That reset logic itself is genuinely portable
 * (just sys_reboot(), a real mainline Zephyr API), so it's
 * reimplemented here directly instead of dragging in the whole file -
 * this is real, working functionality for IPMI's Cold/Warm Reset
 * commands (app_handler.c), not a stub.
 */
#define BIC_WARM_RESET_DELAY_MS 2000
#define BIC_COLD_RESET_DELAY_MS 100

static void bic_warm_reset_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	k_msleep(BIC_WARM_RESET_DELAY_MS);
	sys_reboot(SOC_RESET);
}

static void bic_cold_reset_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	k_msleep(BIC_COLD_RESET_DELAY_MS);
	sys_reboot(SOC_RESET);
}

K_WORK_DEFINE(bic_warm_reset_work, bic_warm_reset_work_handler);
K_WORK_DEFINE(bic_cold_reset_work, bic_cold_reset_work_handler);

void submit_bic_warm_reset(void)
{
	k_work_submit(&bic_warm_reset_work);
}

void submit_bic_cold_reset(void)
{
	k_work_submit(&bic_cold_reset_work);
}
