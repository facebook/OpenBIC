/*
  NAME: I2C SLAVE INIT
  FILE: plat_i2c_slave.c
  DESCRIPTION: Provide i2c slave EN/CFG table "I2C_SLAVE_EN_TABLE[]/I2C_SLAVE_CFG_TABLE[]" for init slave config.
  AUTHOR: MouchenHung
  DATE/VERSION: 2021.11.26 - v1.1
  Note: 
    (1) "plat_i2c_slave.h" is included by "hal_i2c_slave.h"
*/

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include "plat_i2c_slave.h"

/* I2C slave init-enable table */
const bool I2C_SLAVE_ENABLE_TABLE[MAX_SLAVE_NUM] = {
	SLAVE_ENABLE,  SLAVE_DISABLE, SLAVE_DISABLE, SLAVE_DISABLE, SLAVE_DISABLE, SLAVE_DISABLE,
	SLAVE_ENABLE,  SLAVE_DISABLE, SLAVE_ENABLE,  SLAVE_DISABLE, SLAVE_DISABLE, SLAVE_DISABLE,
	SLAVE_DISABLE, SLAVE_DISABLE, SLAVE_DISABLE, SLAVE_DISABLE,
};

/* I2C slave init-config table */
const struct _i2c_slave_config I2C_SLAVE_CONFIG_TABLE[MAX_SLAVE_NUM] = {
	{ 0x40, 0xA }, { 0x40, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA },
	{ 0x40, 0x5 }, { 0xFF, 0xA }, { 0x40, 0x4 }, { 0xFF, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA },
	{ 0xFF, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA },
};