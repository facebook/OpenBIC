/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "hal_def.h"
#include "objects.h"

hal_status_t aspeed_mac_reset(mac_t *obj);
hal_status_t aspeed_mac_set_loopback(mac_t *obj, uint8_t enable);
hal_status_t aspeed_mac_init(mac_t *obj);
hal_status_t aspeed_mac_deinit(mac_t *obj);
hal_status_t aspeed_mac_xmit(mac_t *obj, void *packet, int length);
hal_status_t aspeed_mac_recv(mac_t *obj, void **packet, uint32_t *rxlen);
hal_status_t aspeed_mac_set_speed(mac_t *obj, uint32_t speed);