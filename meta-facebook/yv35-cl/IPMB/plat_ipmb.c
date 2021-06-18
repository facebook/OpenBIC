/*
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include "cmsis_os2.h"
#include "board_device.h"
#include "objects.h"
#include "hal_i2c.h"
#include "plat_i2c.h"
#include "ipmi.h"
#include <string.h>
#include "wait.h"
#include "ipmi_def.h"

IPMB_config pal_IPMB_config_table[] = {
  //   index             interface         interface_source  bus              Target_addr          EnStatus  slave_addr            Rx_attr_name          Tx_attr_name//	
     { BMC_IPMB_IDX,     I2C_IF,           BMC_IPMB_IFs,     IPMB_I2C_BMC,    BMC_I2C_ADDRESS,     Enable,   Self_I2C_ADDRESS,     "RX_BMC_IPMB_TASK",   "TX_BMC_IPMB_TASK"  },
     { ME_IPMB_IDX,      I2C_IF,           ME_IPMB_IFs,      IPMB_ME_BUS,     ME_I2C_ADDRESS,      Enable,   Self_I2C_ADDRESS,     "RX_ME_IPMB_TASK",    "TX_ME_IPMB_TASK"   },
     { EXP1_IPMB_IDX,    I2C_IF,           EXP1_IPMB_IFs,    IPMB_EXP1_BUS,   BIC0_I2C_ADDRESS,    Disable,  BIC1_I2C_ADDRESS,     "RX_EPX0_IPMB_TASK",  "TX_EXP0_IPMB_TASK" },
     { EXP2_IPMB_IDX,    I2C_IF,           EXP2_IPMB_IFs,    IPMB_EXP2_BUS,   BIC1_I2C_ADDRESS,    Disable,  BIC0_I2C_ADDRESS,     "RX_EPX1_IPMB_TASK",  "TX_EXP1_IPMB_TASK" },
     { MAX_IPMB_IDX,     Reserve_IF,       Reserve_IFs,      Reserve_BUS,     Reserve_ADDRESS,     Disable,  Reserve_ADDRESS,      "Reserve_ATTR",       "Reserve_ATTR"      },
};
