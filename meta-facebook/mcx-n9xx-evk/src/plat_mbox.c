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
 * Inter-core mailbox subsystem: cpu0's side of a ping/pong exchange
 * with cpu1 (remote/src/main.c) over the MU (Messaging Unit) hardware,
 * via mainline Zephyr's generic mbox API. Only exercised in the
 * dual-core sysbuild - see README.md and sysbuild.cmake/Kconfig.sysbuild.
 *
 * This mirrors Zephyr's own samples/drivers/mbox sample for this exact
 * board (verified upstream), adapted into this app's plat_* module
 * style instead of a standalone sample.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/logging/log.h>

#include "plat_mbox.h"

LOG_MODULE_REGISTER(plat_mbox, LOG_LEVEL_INF);

static const struct mbox_dt_spec tx_channel = MBOX_DT_SPEC_GET(DT_PATH(mbox_consumer), tx);
static const struct mbox_dt_spec rx_channel = MBOX_DT_SPEC_GET(DT_PATH(mbox_consumer), rx);

static void mbox_callback(const struct device *dev, mbox_channel_id_t channel_id, void *user_data,
			   struct mbox_msg *data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	ARG_UNUSED(data);

	LOG_INF("pong from cpu1 (channel %d)", channel_id);
}

void plat_mbox_init(void)
{
	int ret;

	if (!device_is_ready(tx_channel.dev) || !device_is_ready(rx_channel.dev)) {
		LOG_ERR("mbox device not ready");
		return;
	}

	ret = mbox_register_callback_dt(&rx_channel, mbox_callback, NULL);
	if (ret < 0) {
		LOG_ERR("mbox_register_callback_dt failed: %d", ret);
		return;
	}

	ret = mbox_set_enabled_dt(&rx_channel, true);
	if (ret < 0) {
		LOG_ERR("mbox_set_enabled_dt failed: %d", ret);
		return;
	}

	LOG_INF("mbox ready (tx channel %d, rx channel %d)", tx_channel.channel_id,
		rx_channel.channel_id);
}

void plat_mbox_ping(void)
{
	int ret = mbox_send_dt(&tx_channel, NULL);

	if (ret < 0) {
		LOG_ERR("mbox_send_dt failed: %d", ret);
	} else {
		LOG_INF("ping sent to cpu1 (channel %d)", tx_channel.channel_id);
	}
}
