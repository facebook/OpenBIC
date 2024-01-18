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

#include <stdio.h>
#include <string.h>
#include <logging/log.h>
#include "sensor.h"
#include "hal_i2c.h"

LOG_MODULE_REGISTER(dev_ads112c);

// Command Byte to control device
#define CMD_RESET 0x06 //000 011x(06h)
#define CMD_START_SYNC  0x08 //000 100x(08h)
#define CMD_POWERDOWN  0x02 //000 001x(08h)
#define CMD_RDATA  0x10 //001 000x(10h)
#define CMD_RREG  0x20 //010 000x(20h)
#define CMD_WREG  0x40 //100 000x(40h)

// Configuration Registers offset
#define CFG_REG_OFFSET0 0x00 
#define CFG_REG_OFFSET1 0x04
#define CFG_REG_OFFSET2 0x08
#define CFG_REG_OFFSET3 0x12

uint8_t ads112c_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 3;
	I2C_MSG msg;
	uint8_t offset = cfg->offset;

	while (1) {
		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;
		msg.tx_len = 1;
		msg.data[0] = CMD_RREG || CFG_REG_OFFSET2; //RREG command: 0010 rrxx (rr register address = 10 (0x02), DRDY -> Conversion result ready flag.) 		
		msg.rx_len = 1;		
		int ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Read register on ads112c failed");
			return SENSOR_UNSPECIFIED_ERROR;
		}

		if (msg.data[0] & 0x80) { //the 7th (DRDY) bit in Configuration Register 2 
			break;
		} else {
			k_msleep(10);
			memset(&msg, 0, sizeof(msg));
			continue;
		}
	}

	memset(&msg, 0, sizeof(msg));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.data[0] = CMD_RDATA; // Send the RDATA command (10h)
	msg.rx_len = 2;// read 2 bytes of conversion data

	if (i2c_master_read(&msg, retry)) {
		return SENSOR_FAIL_TO_ACCESS;
	}	

	int rawValue = (msg.data[1] << 8) | msg.data[0];
	double val;
	double v_val, flow_Pmax = 400, flow_Pmin = 10, press_Pmax = 50,
    press_Pmin = 0;	

	switch (offset) {
	case ADS112C_FLOW_OFFSET://Flow_Rate_LPM
		v_val = 5 - ((32767 - rawValue) * 0.000153);
		val = (((v_val / 5) - 0.1) / (0.8 / (flow_Pmax - flow_Pmin))) + 10;
		val = (val - 7.56494) * 0.294524;
		val = 1.2385 * ((2.5147 * val) - 2.4892);
		val = (1.7571 * val) - 0.8855;
		break;

	case ADS112C_PRESS_OFFSET://Filter_P/Outlet_P/Inlet_P
		v_val = 5 - ((32767 - rawValue) * 0.000153);
		val = (((v_val / 5) - 0.1) / (0.8 / (press_Pmax - press_Pmin))) + 10;
		val = ((0.9828 * val) - 9.9724) * 6.894759;
		break;

	case ADS112C_TEMP_OFFSET://RDHx_Hot_Liq_T/CDU_Inlet_Liq_T
		val = (rawValue - 15888) * 0.015873;
		val = (1.1685 * val) - 4.5991;
		break;

	default:
		val = rawValue * 1.0;
		break;
	}	

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;	

	return SENSOR_READ_SUCCESS;
}


uint8_t ads112c_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 3;
	I2C_MSG msg;
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.data[0] = CMD_RESET; // RESET Command 000 011x(06h) 

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Reset ADS112C failed to the default state");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	memset(&msg, 0, sizeof(msg));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 4;
	msg.data[0] = CMD_WREG || CFG_REG_OFFSET0; //WREG command: 0100 rrxx (rr register address = 00) 
	msg.data[1] = 0x08; //configuration register settings are changed to gain = 16
	msg.data[2] = CMD_WREG || CFG_REG_OFFSET1; //WREG command: 0100 rrxx (rr register address = 01)(spec sample is 0x42)	
	msg.data[3] = 0x08; //continuous conversion mode.		
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Write the respective register configurations failed");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}


	memset(&msg, 0, sizeof(msg));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.data[0] = CMD_START_SYNC; // START/SYNC Command 000 100x(08h)
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Failed to start conversion mode on ADS112C");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}


	cfg->read = ads112c_read;
	return SENSOR_INIT_SUCCESS;	
}