
/** \addtogroup hal */
/** @{*/
/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
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
#ifndef MBED_I3C_API_H
#define MBED_I3C_API_H

#include "objects.h"
#include "pinmap.h"
#include "buffer.h"

#if DEVICE_I3C_ASYNCH
#include "dma_api.h"
#endif

#if CONFIG_DEVICE_I3C

/**
 * @defgroup hal_i3cEvents i3c Events Macros
 *
 * @{
 */
#define i3c_EVENT_ERROR               (1 << 1)
#define i3c_EVENT_ERROR_NO_SLAVE      (1 << 2)
#define i3c_EVENT_TRANSFER_COMPLETE   (1 << 3)
#define i3c_EVENT_TRANSFER_EARLY_NACK (1 << 4)
#define i3c_EVENT_ALL                 (i3c_EVENT_ERROR |  i3c_EVENT_TRANSFER_COMPLETE | i3c_EVENT_ERROR_NO_SLAVE | i3c_EVENT_TRANSFER_EARLY_NACK)

/**@}*/

#if DEVICE_I3C_ASYNCH
/** Asynch I3C HAL structure
 */
typedef struct {
    struct i3c_s    i3c;     /**< Target specific i3c structure */
    struct buffer_s tx_buff; /**< Tx buffer */
    struct buffer_s rx_buff; /**< Rx buffer */
} i3c_t;

#else
/** Non-asynch i3c HAL structure
 */
typedef struct i3c_s i3c_t;

#endif

enum {
    i3c_ERROR_NO_SLAVE = -1,
    i3c_ERROR_BUS_BUSY = -2
};

typedef struct {
    int peripheral;
    PinName sda_pin;
    int sda_function;
    PinName scl_pin;
    int scl_function;
} i3c_pinmap_t;


enum i3c_error_code {
	I3C_ERROR_UNKNOWN = 0,
	I3C_ERROR_M0 = 1,
	I3C_ERROR_M1,
	I3C_ERROR_M2,
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup hal_Generali3c i3c Configuration Functions
 *
 * # Defined behavior
 * * ::i3c_init initializes i3c_t control structure
 * * ::i3c_init configures the pins used by i3c
 * * ::i3c_free returns the pins owned by the i3c object to their reset state
 * * ::i3c_frequency configure the i3c frequency
 * * ::i3c_start sends START command
 * * ::i3c_read reads `length` bytes from the i3c slave specified by `address` to the `data` buffer
 * * ::i3c_read reads generates a stop condition on the bus at the end of the transfer if `stop` parameter is non-zero
 * * ::i3c_read reads returns the number of symbols received from the bus
 * * ::i3c_write writes `length` bytes to the i3c slave specified by `address` from the `data` buffer
 * * ::i3c_write generates a stop condition on the bus at the end of the transfer if `stop` parameter is non-zero
 * * ::i3c_write returns zero on success, error code otherwise
 * * ::i3c_reset resets the i3c peripheral
 * * ::i3c_byte_read reads and return one byte from the specfied i3c slave
 * * ::i3c_byte_read uses `last` parameter to inform the slave that all bytes have been read
 * * ::i3c_byte_write writes one byte to the specified i3c slave
 * * ::i3c_byte_write returns 0 if NAK was received, 1 if ACK was received, 2 for timeout
 * * ::i3c_slave_mode enables/disables I2S slave mode
 * * ::i3c_slave_receive returns: 1 - read addresses, 2 - write to all slaves, 3 write addressed, 0 - the slave has not been addressed
 * * ::i3c_slave_read reads `length` bytes from the i3c master to the `data` buffer
 * * ::i3c_slave_read returns non-zero if a value is available, 0 otherwise
 * * ::i3c_slave_write writes `length` bytes to the i3c master from the `data` buffer
 * * ::i3c_slave_write returns non-zero if a value is available, 0 otherwise
 * * ::i3c_slave_address configures i3c slave address
 * * ::i3c_transfer_asynch starts i3c asynchronous transfer
 * * ::i3c_transfer_asynch writes `tx_length` bytes to the i3c slave specified by `address` from the `tx` buffer
 * * ::i3c_transfer_asynch reads `rx_length` bytes from the i3c slave specified by `address` to the `rx` buffer
 * * ::i3c_transfer_asynch generates a stop condition on the bus at the end of the transfer if `stop` parameter is non-zero
 * * The callback given to ::i3c_transfer_asynch is invoked when the transfer completes
 * * ::i3c_transfer_asynch specifies the logical OR of events to be registered
 * * The ::i3c_transfer_asynch function may use the `DMAUsage` hint to select the appropriate async algorithm
 * * ::i3c_irq_handler_asynch returns event flags if a transfer termination condition was met, otherwise returns 0.
 * * ::i3c_active returns non-zero if the i3c module is active or 0 if it is not
 * * ::i3c_abort_asynch aborts an on-going async transfer
 *
 * # Undefined behavior
 * * Calling ::i3c_init multiple times on the same `i3c_t`
 * * Calling any function other than ::i3c_init on a non-initialized `i3c_t`
 * * Initialising the `i3c` peripheral with invalid `SDA` and `SCL` pins.
 * * Passing pins that cannot be on the same peripheral
 * * Passing an invalid pointer as `obj` to any function
 * * Use of a `null` pointer as an argument to any function.
 * * Initialising the peripheral in slave mode if slave mode is not supported
 * * Operating the peripheral in slave mode without first specifying and address using ::i3c_slave_address
 * * Setting an address using i3c_slave_address after initialising the peripheral in master mode
 * * Setting an address to an `i3c` reserved value
 * * Specifying an invalid address when calling any `read` or `write` functions
 * * Setting the length of the transfer or receive buffers to larger than the buffers are
 * * Passing an invalid pointer as `handler`
 * * Calling ::i3c_abort_async when no transfer is currently in progress
 *
 *
 * @{
 */

/**
 * \defgroup hal_Generali3c_tests i3c hal tests
 * The i3c HAL tests ensure driver conformance to defined behaviour.
 *
 * To run the i3c hal tests use the command:
 *
 *     mbed test -t <toolchain> -m <target> -n tests-mbed_hal_fpga_ci_test_shield-i3c
 *
 */

/** Initialize the i3c peripheral. It sets the default parameters for i3c
 *  peripheral, and configures its specifieds pins.
 *
 *  @param obj  The i3c object
 *  @param pinmap  Pinmap pointer to structure which holds static pinmap
 */
void i3c_init_direct(i3c_t *obj, const i3c_pinmap_t *pinmap);

/** Initialize the i3c peripheral. It sets the default parameters for i3c
 *  peripheral, and configures its specifieds pins.
 *
 *  @param obj  The i3c object
 *  @param buf  The buffer for message queues
 */
void i3c_init(i3c_t *obj);

/** Release a i3c object
 *
 * Return the pins owned by the i3c object to their reset state
 * @param obj The i3c object to deinitialize
 */
void i3c_free(i3c_t *obj);

/** Configure the i3c frequency
 *
 *  @param obj The i3c object
 *  @param hz  Frequency in Hz
 */
void i3c_frequency(i3c_t *obj, int hz);

/** Send START command
 *
 *  @param obj The i3c object
 */
int  i3c_start(i3c_t *obj);

/** Send STOP command
 *
 *  @param obj The i3c object
 */
int  i3c_stop(i3c_t *obj);

/** Blocking reading data
 *
 *  @param obj     The i3c object
 *  @param address 7-bit address (last bit is 1)
 *  @param data    The buffer for receiving
 *  @param length  Number of bytes to read
 *  @param stop    Stop to be generated after the transfer is done
 *  @return Number of read bytes
 */
//int i3c_read(i3c_t *obj, int address, char *data, int length, int stop);
void i3c_spd_read(i3c_t *obj, uint32_t idx, uint32_t addr, uint32_t len, uint8_t *buf);

/** Blocking sending data
 *
 *  @param obj     The i3c object
 *  @param address 7-bit address (last bit is 0)
 *  @param data    The buffer for sending
 *  @param length  Number of bytes to write
 *  @param stop    Stop to be generated after the transfer is done
 *  @return
 *      zero or non-zero - Number of written bytes
 *      negative - i3c_ERROR_XXX status
 */
//int i3c_write(i3c_t *obj, int address, const char *data, int length, int stop);
void i3c_spd_write(i3c_t *obj, uint32_t idx, uint32_t addr, uint32_t len, uint8_t *buf);

/** Reset i3c peripheral. TODO: The action here. Most of the implementation sends stop()
 *
 *  @param obj The i3c object
 */
void i3c_reset(i3c_t *obj);

/** Read one byte
 *
 *  @param obj The i3c object
 *  @param last Acknoledge
 *  @return The read byte
 */
//int i3c_byte_read(i3c_t *obj, int last);
uint8_t i3c_spd_byte_read(i3c_t *obj, uint32_t idx, uint32_t addr);

/** Write one byte
 *
 *  @param obj The i3c object
 *  @param data Byte to be written
 *  @return 0 if NAK was received, 1 if ACK was received, 2 for timeout.
 */
//int i3c_byte_write(i3c_t *obj, int data);
void i3c_spd_byte_write(i3c_t *obj, uint32_t idx, uint32_t addr, uint8_t data);


/**@}*/

#if 1 //DEVICE_I3CSLAVE

/**
 * \defgroup Synchi3c Synchronous i3c Hardware Abstraction Layer for slave
 * @{
 */

/** Configure i3c as slave or master.
 *  @param obj The i3c object
 *  @param enable_slave Enable i3c hardware so you can receive events with ::i3c_slave_receive
 *  @return non-zero if a value is available
 */
void i3c_slave_mode(i3c_t *obj, int enable_slave);

/** Check to see if the i3c slave has been addressed.
 *  @param obj The i3c object
 *  @return The status - 1 - read addresses, 2 - write to all slaves,
 *         3 write addressed, 0 - the slave has not been addressed
 */
int  i3c_mqueue_receive(i3c_t *obj, i3c_msg_t *msg);
#define  i3c_slave_receive	i3c_mqueue_receive
#define  i3c_ibi_receive	i3c_mqueue_receive

/** Configure i3c as slave or master.
 *  @param obj The i3c object
 *  @param data    The buffer for receiving
 *  @param length  Number of bytes to read
 *  @return non-zero if a value is available
 */
int  i3c_slave_read(i3c_t *obj, char *data, int length);

/** Configure i3c as slave or master.
 *  @param obj The i3c object
 *  @param data    The buffer for sending
 *  @param length  Number of bytes to write
 *  @return non-zero if a value is available
 */
int  i3c_slave_write(i3c_t *obj, const char *data, int length);

/** Configure i3c address.
 *  @param obj     The i3c object
 *  @param idx     Currently not used
 *  @param address The address to be set
 *  @param mask    Currently not used
 */
void i3c_slave_address(i3c_t *obj, int idx, uint32_t address, uint32_t mask);

#endif

/**@}*/

#if 0 //DEVICE_I3C_ASYNCH

/**
 * \defgroup hal_Asynchi3c Asynchronous i3c Hardware Abstraction Layer
 * @{
 */

/** Start i3c asynchronous transfer
 *
 *  @param obj       The i3c object
 *  @param tx        The transmit buffer
 *  @param tx_length The number of bytes to transmit
 *  @param rx        The receive buffer
 *  @param rx_length The number of bytes to receive
 *  @param address   The address to be set - 7bit or 9bit
 *  @param stop      If true, stop will be generated after the transfer is done
 *  @param handler   The i3c IRQ handler to be set
 *  @param event     Event mask for the transfer. See \ref hal_i3cEvents
 *  @param hint      DMA hint usage
 */
void i3c_transfer_asynch(i3c_t *obj, const void *tx, size_t tx_length, void *rx, size_t rx_length, uint32_t address, uint32_t stop, uint32_t handler, uint32_t event, DMAUsage hint);

/** The asynchronous IRQ handler
 *
 *  @param obj The i3c object which holds the transfer information
 *  @return Event flags if a transfer termination condition was met, otherwise return 0.
 */
uint32_t i3c_irq_handler_asynch(i3c_t *obj);

/** Attempts to determine if the i3c peripheral is already in use
 *
 *  @param obj The i3c object
 *  @return Non-zero if the i3c module is active or zero if it is not
 */
uint8_t i3c_active(i3c_t *obj);

/** Abort asynchronous transfer
 *
 *  This function does not perform any check - that should happen in upper layers.
 *  @param obj The i3c object
 */
void i3c_abort_asynch(i3c_t *obj);

#endif

/**@}*/

#ifdef __cplusplus
}
#endif

#endif

#endif

/** @}*/
