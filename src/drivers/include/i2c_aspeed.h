/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ASPEED_I2C_API_H
#define ASPEED_I2C_API_H

#include "objects.h"
#include "pinmap.h"
#include "buffer.h"
#include "hal_def.h"

#define I2C_SNOOP_MODE

#ifdef I2C_SNOOP_MODE
#define I2C_SNOOP_BUFF_SIZE		(32 * 1024)
#endif

enum i2c_xfer_mode {
	DMA_MODE = 0,
	BUFF_MODE,
	BYTE_MODE,
};

struct i2c_s {
	aspeed_device_t *device;
	uint32_t global_reg;
	enum i2c_xfer_mode mode;
	int 		xfer_complete;
	int			clk_div_mode;	//0: old mode, 1: new mode	
	uint32_t	apb_clk;
	uint32_t	bus_frequency;	
	/* master configuration */
	bool multi_master;
	uint32_t	bus_recover;
	int			cmd_err;
	size_t		buf_index;	//buffer mode idx 	
	int			master_xfer_cnt;	//total xfer count
	uint16_t	flags;
	uint8_t 	*buf;
	int			len;	//master xfer count
	/* Buffer mode */
	uint8_t		*buf_base;
	size_t		buf_size;

	/* snoop mode */
#ifdef I2C_SNOOP_MODE	
	uint8_t		*snoop_buf;
	size_t		snoop_size;
#endif

	/* slave configuration */
	uint8_t 	slave_addr;
	uint32_t	slave_xfer_len;
	uint32_t	slave_xfer_cnt;

	/* slave mqueue */
	int			fifo_fetch_idx;
	int			fifo_rx_idx;
	int			fifo_full;
	struct i2c_slave_mqueue		slave_mq[I2C_SLAVE_MQ_FIFO_SIZE];
	osEventFlagsId_t evt_id;
};

/** Non-asynch I2C HAL structure
 */
typedef struct i2c_s i2c_t;

/* 
 * Initialize the I2C peripheral. It sets the default parameters for I2C
 *  peripheral, and configures its specifieds pins.
 *  @param obj  The I2C object 
 */

hal_status_t i2c_init(i2c_t *obj);

void aspeed_i2c_bus_reset(struct i2c_s *obj);
uint8_t aspeed_i2c_recover_bus(struct i2c_s *obj);

void i2c_frequency(i2c_t *obj, int hz);	

/** Blocking reading data
 *
 *  @param obj     The I2C object
 *  @param address 7-bit address (last bit is 1)
 *  @param data    The buffer for receiving
 *  @param length  Number of bytes to read
 *  @param stop    Stop to be generated after the transfer is done
 *  @return Number of read bytes
 */
#define I2C_M_RECV_LEN		0x0400	/* length will be first received byte */
#define I2C_M_STOP		0x8000	/* if I2C_FUNC_PROTOCOL_MANGLING */

int i2c_read(i2c_t *obj, int address, uint8_t *data, int length, uint16_t flags);

/** Blocking sending data
 *
 *  @param obj     The I2C object
 *  @param address 7-bit address (last bit is 0)
 *  @param data    The buffer for sending
 *  @param length  Number of bytes to write
 *  @param stop    Stop to be generated after the transfer is done
 *  @return
 *      zero or non-zero - Number of written bytes
 *      negative - I2C_ERROR_XXX status
 */
int i2c_write(i2c_t *obj, int address, uint8_t *data, int length, uint16_t flags);
void i2c_reset(i2c_t *obj);
void i2c_slave_mode(i2c_t *obj, int enable_slave);
int i2c_slave_mqueue_read(i2c_t *obj, uint8_t *data);

/** Configure I2C address. support 3 slave address idx 0 ~ 2 */
void i2c_slave_address(i2c_t *obj, int idx, uint8_t address);
#endif
