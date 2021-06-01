/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "FreeRTOS_CLI.h"
const CLI_Command_Definition_t i3c_cmd;
void i3c_cmd_register_obj(i3c_t *obj);
void print_i3c_msg(i3c_msg_t *msg);