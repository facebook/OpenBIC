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

#ifndef PLAT_MCTP_H
#define PLAT_MCTP_H

/* This EVK's own MCTP endpoint address on flexcomm3_lpi2c3, 7-bit form.
 * Arbitrary placeholder - unlike IPMB's 0x20 BMC convention, MCTP-over-
 * SMBus has no single universal "well-known" endpoint address; this
 * needs a real value once this board is wired to an actual MCTP bus
 * master/other endpoints instead of standing alone. See README.md.
 */
#define PLAT_MCTP_I2C_TARGET_ADDR 0x10

/* Arbitrary placeholder starting EID, reported until a real bus owner
 * reassigns one via MCTP Control's Set Endpoint ID (see
 * plat_get_eid()/plat_get_mctp_port_count()/plat_get_mctp_port() in
 * plat_mctp.c - the board-level hooks that make that actually work,
 * instead of both Get and Set Endpoint ID failing/no-oping).
 */
#define PLAT_MCTP_EID 0x09

void plat_mctp_init(void);

#endif
