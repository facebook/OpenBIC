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

#include <logging/log.h>
#include "hal_i3c.h"
#include "plat_mctp.h"
#include "plat_i3c.h"

LOG_MODULE_REGISTER(plat_i3c);

k_tid_t tid;
K_THREAD_STACK_DEFINE(send_setaasa_stack, SEND_SETAASA_STACK_SIZE);
struct k_thread send_setaasa_thread;

void start_setaasa()
{
	LOG_INF("Start a thread to send setaasa");
	tid = k_thread_create(&send_setaasa_thread, send_setaasa_stack,
			      K_THREAD_STACK_SIZEOF(send_setaasa_stack), send_setaasa, NULL, NULL,
			      NULL, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
	k_thread_name_set(&send_setaasa_thread, "send_setaasa_thread");
}

void send_setaasa()
{
	I3C_MSG i3c_msg = { 0 };
	i3c_msg.bus = I3C_BUS_HUB;

	while (1) {
		// I3C_CCC_SETAASA: Set all addresses to static address
		i3c_brocast_ccc(&i3c_msg, I3C_CCC_SETAASA, I3C_BROADCAST_ADDR);
		k_msleep(SEND_SETAASA_TIME_MS);
	}
}
