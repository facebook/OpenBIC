/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "common.h"
#include "util.h"
#include "hal_def.h"
#include "wait.h"
#include <string.h>
#include "log.h"
#include "usb_cdc.h"
#include "usb_aspeed.h"

const osMutexAttr_t usb_mutex_attr = {
   "usb_device_lock", 
   // human readable mutex name
   osMutexRecursive, 
   // attr_bits
    NULL, 
   // memory for control block 
    0U 
   // size for control block
   };

int usb_init(usb_t *obj)
{
	int ret;
	ret = aspeed_usb_init(obj);
	usbtty_init_strings();
	aspeed_usb_connect(obj);
	if (ret == HAL_OK) {
		obj->lock = osMutexNew (&usb_mutex_attr);
	}
	return ret;
}

int usb_acquire(usb_t *obj)
{
    return osMutexAcquire(obj->lock, osWaitForever);
}

int usb_release(usb_t *obj)
{
    return osMutexRelease(obj->lock);
}

int usb_connect(usb_t *obj)
{
    osThreadId_t task_id;
	task_id = osMutexGetOwner(obj->lock);
    if (task_id != osThreadGetId()) {
		return osErrorResource;
	} else {
	    aspeed_usb_connect(obj);
		return osOK;
	}
}

int usb_disconnect(usb_t *obj)
{
    osThreadId_t task_id;
	task_id = osMutexGetOwner(obj->lock);
    if (task_id != osThreadGetId()) {
		return osErrorResource;
	} else {
		aspeed_usb_disconnect(obj);
        return osOK;
	}
}

int usb_read(usb_t *obj, int ep_num, uint8_t *buff, int buff_len)
{
	osThreadId_t task_id;
	task_id = osMutexGetOwner(obj->lock);
    if (task_id != osThreadGetId()) {
		return osErrorResource;
	} else {
        if (obj->configure)
		    return aspeed_ep_read(obj, ep_num, buff, buff_len);
        else
			return 0;
	}
}

int usb_write(usb_t *obj, int ep_num, uint8_t *buff, int buff_len)
{
	osThreadId_t task_id;
	task_id = osMutexGetOwner(obj->lock);
    if (task_id != osThreadGetId()) {
		return osErrorResource;
	} else {
        if (obj->configure)
		    return aspeed_ep_write(obj, ep_num, buff, buff_len);
        else
			return -1;
	}
}