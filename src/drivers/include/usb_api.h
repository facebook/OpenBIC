/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef USB_API_H
#define USB_API_H
#include "usb_aspeed.h"
int usb_init(usb_t *obj);
int usb_acquire(usb_t *obj);
int usb_release(usb_t *obj);
int usb_connect(usb_t *obj);
int usb_disconnect(usb_t *obj);
int usb_read(usb_t *obj, int ep_num, uint8_t *buff, int buff_len);
int usb_write(usb_t *obj, int ep_num, uint8_t *buff, int buff_len);
#endif