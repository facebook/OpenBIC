/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ASPEED_USB_API_H
#define ASPEED_USB_API_H

#include <stdint.h>
#include "objects.h"
#include "pinmap.h"
#include "buffer.h"
#include "hal_def.h"
#include "usb_aspeed.h"

struct ep_config {
	int 	ep_isout;
	uint8_t *ep_tx_dma;
	int		ep_tx_len;
	int		ep_tx_last;
	int		ep_rx_len;
	uint8_t ep_rx_dma[1024];
};

struct usb_s {
	aspeed_device_t *device;
	osEventFlagsId_t evt_id;	
	uint16_t		addr;
	int				pullup;

	//ep0
	uint8_t  		bmRequestType;
	int 			configure;
	int 			zero_len_tx;
	struct ep_config ep[5];
	
	//mutex
	osMutexId_t lock;
};

/** Non-asynch UDC HAL structure
 */
typedef struct usb_s usb_t;

hal_status_t aspeed_usb_init(usb_t *obj);

void aspeed_usb_connect(usb_t *obj);
void aspeed_usb_disconnect(usb_t *obj);
void aspeed_usb_handler(usb_t *obj);
int aspeed_ep_read(usb_t *obj, int ep_num, uint8_t *buff, int buff_len);
int aspeed_ep_write(usb_t *obj, int ep_num, uint8_t *buff, int buff_len);
#endif
