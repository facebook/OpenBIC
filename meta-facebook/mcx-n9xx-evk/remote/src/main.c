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
 * cpu1's image for the dual-core subsystem. Headless (no wired
 * console - see prj.conf): on receiving a ping from cpu0 over the
 * mailbox, immediately echoes a reply back, so cpu0's own console log
 * ("pong from cpu1") is direct proof cpu1 actually received and
 * reacted to it - not just two independent unrelated timers.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/mbox.h>

static const struct mbox_dt_spec tx_channel = MBOX_DT_SPEC_GET(DT_PATH(mbox_consumer), tx);
static const struct mbox_dt_spec rx_channel = MBOX_DT_SPEC_GET(DT_PATH(mbox_consumer), rx);

static void mbox_callback(const struct device *dev, mbox_channel_id_t channel_id, void *user_data,
			   struct mbox_msg *data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);
	ARG_UNUSED(user_data);
	ARG_UNUSED(data);

	mbox_send_dt(&tx_channel, NULL);
}

int main(void)
{
	if (!device_is_ready(tx_channel.dev) || !device_is_ready(rx_channel.dev)) {
		return 0;
	}

	if (mbox_register_callback_dt(&rx_channel, mbox_callback, NULL) < 0) {
		return 0;
	}

	mbox_set_enabled_dt(&rx_channel, true);

	return 0;
}
