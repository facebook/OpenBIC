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

/* This EVK's own MCTP endpoint address, 7-bit form. Shares the sideband
 * bus (flexcomm2_lpi2c2) with IPMB: IPMB answers at 0x20, this MCTP
 * endpoint at 0x10, as two slave-address matches on the one LPI2C
 * instance (see plat_i2c.h). Arbitrary placeholder - unlike IPMB's 0x20
 * BMC convention, MCTP-over-SMBus has no single universal "well-known"
 * endpoint address; a real bus owner can reassign the EID via Set
 * Endpoint ID regardless. See README.md.
 */
#define PLAT_MCTP_I2C_TARGET_ADDR 0x10

/* Arbitrary placeholder starting EID, reported until a real bus owner
 * reassigns one via MCTP Control's Set Endpoint ID (see
 * plat_get_eid()/plat_get_mctp_port_count()/plat_get_mctp_port() in
 * plat_mctp.c - the board-level hooks that make that actually work,
 * instead of both Get and Set Endpoint ID failing/no-oping).
 */
#define PLAT_MCTP_EID 0x09

/* Test-only Vendor Defined (PCI) message type (DSP0236 0x7E), used
 * purely to exercise multi-packet fragmentation/reassembly in the
 * OpenBIC->peer direction (the peer side already has a real command
 * - Get Endpoint ID - large enough to force fragmentation the other
 * way). Not a real vendor protocol - "vendor ID" here is just a
 * fixed marker distinguishing this test command from anything else
 * that might show up as a VDM in the future.
 *
 * Request: [type_byte][vendor_id hi][vendor_id lo][cmd][len hi][len lo]
 * Response (cmd == ECHO only): [type_byte][vendor_id hi][vendor_id lo]
 *                               [cmd][status][data x len]
 * `data[i] = i & 0xFF` - a fixed, checkable pattern so the receiver
 * can verify byte-for-byte correctness of reassembly, not just length.
 */
#define PLAT_MCTP_TEST_VENDOR_ID 0xFFFF
#define PLAT_MCTP_TEST_CMD_ECHO 0x01
#define PLAT_MCTP_TEST_MAX_ECHO_LEN 512
#define PLAT_MCTP_TEST_STATUS_SUCCESS 0x00
#define PLAT_MCTP_TEST_STATUS_ERROR 0x01

void plat_mctp_init(void);

#endif
