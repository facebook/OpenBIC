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

#ifdef CONFIG_C2H_NPCM

#include <zephyr.h>
#include <device.h>
#include <stdlib.h>
#include "hal_c2h.h"
#include <drivers/misc/npcm/c2h_npcm.h>
#include "libutil.h"
#include <logging/log.h>
#include "util_sys.h"

LOG_MODULE_REGISTER(c2h);

const struct device *c2h_dev;

void c2h_npcm_init(void)
{
	c2h_dev = DEVICE_DT_GET(DT_NODELABEL(c2h));
}

int get_chip_id(void)
{
	int chip_id = (c2h_read_io_cfg_reg(c2h_dev, CR20_SID_H) << 8) |
		      c2h_read_io_cfg_reg(c2h_dev, CR21_SID_L);
	LOG_INF("chip id 0x%x", chip_id);
	return 0;
}

int get_chip_rev(void)
{
	int chip_rev = c2h_read_io_cfg_reg(c2h_dev, CR24_CHPREV);
	LOG_INF("chip revision 0x%x", chip_rev);
	return 0;
}

int get_device_id(void)
{
	int dev_id = c2h_read_io_cfg_reg(c2h_dev, CR2A_DEVICE_ID);
	LOG_INF("device id 0x%x", dev_id);
	return 0;
}

#endif
