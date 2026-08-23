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
 * Minimal "plat" shell command group, standing in for the equivalent
 * commands under common/shell/commands (not buildable here yet - see
 * top-level README).
 */

#include <zephyr/shell/shell.h>

#include "hal_wdt.h"
#include "ipmi.h"
#include "plat_gpio.h"
#include "plat_mbox.h"
#include "plat_storage.h"
#include "plat_version.h"

static int cmd_version(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "OpenBIC / %s %s, fw %x%x.%x.%x", PLATFORM_NAME, PROJECT_NAME,
		    BIC_FW_YEAR_MSB, BIC_FW_YEAR_LSB, BIC_FW_WEEK, BIC_FW_VER);
	return 0;
}

static int cmd_gpio_mon0(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "mon0 (SW2): %s", plat_gpio_mon0_get() ? "asserted" : "deasserted");
	return 0;
}

static int cmd_wdt_starve(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Stopping watchdog feed - SoC will reset shortly.");
	set_wdt_continue_feed(false);
	return 0;
}

static int cmd_storage_bootcount(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "persistent boot count: %u", plat_storage_boot_count());
	return 0;
}

static int cmd_mbox_ping(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Sending ping to cpu1...");
	plat_mbox_ping();
	return 0;
}

/* Exercises the real IPMI dispatch pipeline (notify_ipmi_client ->
 * ipmi_msgq -> IPMI_handler -> ipmi_cmd_handle -> netfn/cmd handler ->
 * response routed back via the "SELF" InF_source/InF_target case in
 * ipmi_cmd_handle()) without needing a second real device on the wire
 * - see README.md's "IPMI transport" section for why a genuine
 * wire-level IPMB test needs one and is documented separately as
 * untested. This proves the transport's message-routing/handler
 * plumbing is real and working, end to end, in isolation.
 */
static int cmd_ipmi_selftest(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct ipmi_msg_cfg req = { 0 };

	req.buffer.InF_source = SELF;
	req.buffer.InF_target = SELF;
	req.buffer.netfn = NETFN_APP_REQ;
	req.buffer.cmd = CMD_APP_GET_DEVICE_ID;
	req.buffer.data_len = 0;

	shell_print(sh, "Submitting IPMI Get Device ID via SELF...");
	if (notify_ipmi_client(&req) != IPMB_ERROR_SUCCESS) {
		shell_error(sh, "Failed to submit request to ipmi_msgq");
		return -EIO;
	}

	struct ipmi_msg_cfg resp;

	if (k_msgq_get(&self_ipmi_msgq, &resp, K_SECONDS(2))) {
		shell_error(sh, "Timed out waiting for response on self_ipmi_msgq");
		return -ETIMEDOUT;
	}

	shell_print(sh, "Response: netfn=0x%02x cmd=0x%02x cc=0x%02x data_len=%u",
		    resp.buffer.netfn, resp.buffer.cmd, resp.buffer.completion_code,
		    resp.buffer.data_len);
	shell_hexdump(sh, resp.buffer.data, resp.buffer.data_len);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_plat_gpio, SHELL_CMD(mon0, NULL, "Read mon0 (SW2) live state",
							 cmd_gpio_mon0),
				SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_plat_mbox,
				SHELL_CMD(ping, NULL, "Send a ping to cpu1 over the mailbox",
					  cmd_mbox_ping),
				SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_plat_storage,
				SHELL_CMD(bootcount, NULL,
					  "Show the persistent boot counter (NVS)",
					  cmd_storage_bootcount),
				SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_plat_wdt,
				SHELL_CMD(starve, NULL,
					  "Stop feeding the watchdog on purpose (triggers a reset)",
					  cmd_wdt_starve),
				SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_plat,
				SHELL_CMD(version, NULL, "Show OpenBIC platform/firmware version",
					  cmd_version),
				SHELL_CMD(gpio, &sub_plat_gpio, "GPIO status commands", NULL),
				SHELL_CMD(wdt, &sub_plat_wdt, "Watchdog commands", NULL),
				SHELL_CMD(storage, &sub_plat_storage, "Persistent storage commands",
					  NULL),
				SHELL_CMD(mbox, &sub_plat_mbox, "Inter-core mailbox commands", NULL),
				SHELL_CMD(ipmi_selftest, NULL,
					  "Round-trip an IPMI Get Device ID through the real "
					  "dispatch pipeline via the SELF interface",
					  cmd_ipmi_selftest),
				SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(plat, &sub_plat, "OpenBIC platform commands", NULL);
