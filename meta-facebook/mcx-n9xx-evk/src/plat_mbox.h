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

#ifndef PLAT_MBOX_H
#define PLAT_MBOX_H

/* Registers the RX callback for the inter-core mailbox. Only meaningful
 * in the dual-core (sysbuild) build - see remote/ and README.md.
 */
void plat_mbox_init(void);

/* Sends one "ping" message to cpu1 over the mailbox. */
void plat_mbox_ping(void);

#endif
