/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __RESET_ASPEED_H__
#define __RESET_ASPEED_H__
#include "device.h"
/**
 * @brief device reset assert
 * @param [IN] device - pointer to the device
 * @return 0: success -1: device ID not found
*/
int aspeed_reset_deassert(aspeed_device_t *device);

/**
 * @brief device reset assert
 * * @param [IN] device - pointer to the device
 * @return 0: success -1: device ID not found
*/
int aspeed_reset_assert(aspeed_device_t *device);

#endif /* #ifndef __CLK_ASPEED_H__ */