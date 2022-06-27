/*
  NAME: I2C TARGET INIT
  FILE: plat_i2c_target.c
  DESCRIPTION: Provide i2c target EN/CFG table "I2C_TARGET_EN_TABLE[]/I2C_TARGET_CFG_TABLE[]" for init target config.
  AUTHOR: MouchenHung
  DATE/VERSION: 2021.11.26 - v1.1
  Note: 
    (1) "plat_i2c_target.h" is included by "hal_i2c_target.h"
*/

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include "plat_i2c_target.h"

/* I2C target init-enable table */
const bool I2C_TARGET_ENABLE_TABLE[MAX_TARGET_NUM] = {
	TARGET_ENABLE,	TARGET_ENABLE,	TARGET_ENABLE,	TARGET_ENABLE,
	TARGET_DISABLE, TARGET_DISABLE, TARGET_ENABLE,	TARGET_DISABLE,
	TARGET_ENABLE,	TARGET_DISABLE, TARGET_ENABLE,	TARGET_ENABLE,
	TARGET_ENABLE,	TARGET_ENABLE,	TARGET_DISABLE, TARGET_DISABLE,
};

/* I2C target init-config table */
const struct _i2c_target_config I2C_TARGET_CONFIG_TABLE[MAX_TARGET_NUM] = {
	{ 0x40, 0xA }, { 0x40, 0xA }, { 0x40, 0xA }, { 0x40, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA },
	{ 0x40, 0x5 }, { 0xFF, 0xA }, { 0x40, 0x4 }, { 0xFF, 0xA }, { 0x40, 0xA }, { 0x40, 0xA },
	{ 0x40, 0xA }, { 0x40, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA },
};