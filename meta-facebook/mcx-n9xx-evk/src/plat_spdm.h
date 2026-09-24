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

#ifndef PLAT_SPDM_H
#define PLAT_SPDM_H

#include <stdbool.h>
#include <stdint.h>

#include "mctp.h"

/* Bring up the libspdm responder context (capabilities, algorithms,
 * embedded dev cert). Call once at boot, after plat_mctp_init(). */
void plat_spdm_init(void);

/* Handle an inbound SPDM-over-MCTP message (MCTP message type 0x05).
 * buf[0] is the MCTP type byte; buf[1..] is the SPDM message. Runs one
 * libspdm responder dispatch; the reply is sent from within. */
void plat_spdm_handler(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params);

/* Last SPDM request code seen (for `plat spdm` shell status). Returns
 * false if no SPDM request has arrived yet. */
bool plat_spdm_last_request(uint8_t *code);

#endif /* PLAT_SPDM_H */
