#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cmsis_os2.h"
#include "hal_i2c.h"
#include "timer.h"
#include "plat_i2c.h"
#include "libutil.h"

static const struct device *dev_i2c[I2C_BUS_MAX_NUM];

struct k_mutex i2c_mutex[I2C_BUS_MAX_NUM];

void i2c_freq_set(uint8_t i2c_bus, uint8_t i2c_speed_mode)
{
	uint32_t dev_config_raw;

	dev_config_raw = I2C_MODE_MASTER | I2C_SPEED_SET(i2c_speed_mode);
	i2c_configure(dev_i2c[i2c_bus], dev_config_raw);
}

int i2c_master_read(I2C_MSG *msg, uint8_t retry)
{
	uint8_t i;
	uint8_t *txbuf, *rxbuf;
	int ret, status;
	if (DEBUG_I2C) {
		printf("i2c_master_read: bus %d, addr %x, rxlen %d, txlen %d, txbuf:", msg->bus,
		       msg->target_addr, msg->rx_len, msg->tx_len);
		for (int i = 0; i < msg->tx_len; i++) {
			printf(" %x", msg->data[i]);
		}
		printf("\n");
	}

	if (msg->rx_len == 0) {
		printf("i2c_master_read with rx_len = 0\n");
		return EMSGSIZE;
	}

	if (check_i2c_bus_valid(msg->bus) < 0) {
		printf("i2c bus %d is invalid\n", msg->bus);
		return -1;
	}

	do { // break while getting mutex success but tranmission fail
		status = k_mutex_lock(&i2c_mutex[msg->bus], K_MSEC(1000));
		if (status == osOK) {
			for (i = 0; i < retry; i++) {
				txbuf = (uint8_t *)malloc(I2C_BUFF_SIZE * sizeof(uint8_t));
				rxbuf = (uint8_t *)malloc(I2C_BUFF_SIZE * sizeof(uint8_t));

				memcpy(txbuf, &msg->data[0], msg->tx_len);

				ret = i2c_write_read(dev_i2c[msg->bus], msg->target_addr, txbuf,
						     msg->tx_len, rxbuf, msg->rx_len);

				memcpy(&msg->data[0], rxbuf, msg->rx_len);

				if (DEBUG_I2C) {
					printf("rxbuf:");
					for (int i = 0; i < msg->rx_len; i++) {
						printf(" %x", msg->data[i]);
					}
					printf("\n");
				}

				SAFE_FREE(txbuf);
				SAFE_FREE(rxbuf);

				status = k_mutex_unlock(&i2c_mutex[msg->bus]);
				if (status != osOK) {
					printf("I2C %d master read release mutex fail\n", msg->bus);
				}
				return ret; // i2c write and read success
			}
			printf("I2C %d master read retry reach max\n", msg->bus);
			status = k_mutex_unlock(&i2c_mutex[msg->bus]);
			if (status != osOK) {
				printf("I2C %d master read release mutex fail\n", msg->bus);
			}
			return EPIPE;
		} else {
			printf("I2C %d master read get mutex timeout\n", msg->bus);
			return ENOLCK;
		}
	} while (0);

	return ECANCELED; // should not reach here
}

int i2c_master_write(I2C_MSG *msg, uint8_t retry)
{
	uint8_t i;
	uint8_t *txbuf;
	int status, ret;

	if (DEBUG_I2C) {
		printf("i2c_master_write: bus %d, addr %x, txlen %d, txbuf:", msg->bus,
		       msg->target_addr, msg->tx_len);
		for (int i = 0; i < msg->tx_len; i++) {
			printf(" %x", msg->data[i]);
		}
		printf("\n");
	}

	if (check_i2c_bus_valid(msg->bus) < 0) {
		printf("i2c bus %d is invalid\n", msg->bus);
		return -1;
	}

	status = k_mutex_lock(&i2c_mutex[msg->bus], K_MSEC(1000));
	if (status == osOK) {
		for (i = 0; i < retry; i++) {
			txbuf = (uint8_t *)malloc(I2C_BUFF_SIZE * sizeof(uint8_t));
			memcpy(txbuf, &msg->data[0], msg->tx_len);
			ret = i2c_write(dev_i2c[msg->bus], txbuf, msg->tx_len, msg->target_addr);
			if (ret) {
				SAFE_FREE(txbuf);
				continue;
			} else { // i2c write success
				status = k_mutex_unlock(&i2c_mutex[msg->bus]);
				if (status != osOK) {
					printf("I2C %d master write release mutex fail\n",
					       msg->bus);
				}
				SAFE_FREE(txbuf);
				return ret;
			}
		}
		printf("I2C %d master write retry reach max\n", msg->bus);
		status = k_mutex_unlock(&i2c_mutex[msg->bus]);
		if (status != osOK) {
			printf("I2C %d master write release mutex fail\n", msg->bus);
		}
		return EPIPE;

	} else {
		printf("I2C %d master write get mutex timeout\n", msg->bus);
		return ENOLCK;
	}

	return ECANCELED;
}

void i2c_scan(uint8_t bus, uint8_t *target_addr, uint8_t *target_addr_len)
{
	uint8_t first = 0x04, last = 0x77;
	*target_addr_len = 0;

	if (check_i2c_bus_valid(bus) < 0) {
		printf("i2c bus %d is invalid\n", bus);
		return;
	}

	for (uint8_t i = 0; i <= last; i += 16) {
		for (uint8_t j = 0; j < 16; j++) {
			if (i + j < first || i + j > last) {
				continue;
			}

			struct i2c_msg msgs[1];
			uint8_t dst;

			/* Send the address to read from */
			msgs[0].buf = &dst;
			msgs[0].len = 0U;
			msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
			if (i2c_transfer(dev_i2c[bus], &msgs[0], 1, i + j) == 0) {
				target_addr[*target_addr_len] = (i + j) << 1;
				(*target_addr_len)++;
			}
		}
	}
}

void util_init_I2C(void)
{
	int status;

#ifdef DEV_I2C_0
	dev_i2c[0] = device_get_binding("I2C_0");
	status = k_mutex_init(&i2c_mutex[0]);
	if (status)
		printf("i2c0 mutex init fail\n");
#endif
#ifdef DEV_I2C_1
	dev_i2c[1] = device_get_binding("I2C_1");
	status = k_mutex_init(&i2c_mutex[1]);
	if (status)
		printf("i2c1 mutex init fail\n");
#endif
#ifdef DEV_I2C_2
	dev_i2c[2] = device_get_binding("I2C_2");
	status = k_mutex_init(&i2c_mutex[2]);
	if (status)
		printf("i2c2 mutex init fail\n");
#endif
#ifdef DEV_I2C_3
	dev_i2c[3] = device_get_binding("I2C_3");
	status = k_mutex_init(&i2c_mutex[3]);
	if (status)
		printf("i2c3 mutex init fail\n");
#endif
#ifdef DEV_I2C_4
	dev_i2c[4] = device_get_binding("I2C_4");
	status = k_mutex_init(&i2c_mutex[4]);
	if (status)
		printf("i2c4 mutex init fail\n");
#endif
#ifdef DEV_I2C_5
	dev_i2c[5] = device_get_binding("I2C_5");
	status = k_mutex_init(&i2c_mutex[5]);
	if (status)
		printf("i2c5 mutex init fail\n");
#endif
#ifdef DEV_I2C_6
	dev_i2c[6] = device_get_binding("I2C_6");
	status = k_mutex_init(&i2c_mutex[6]);
	if (status)
		printf("i2c6 mutex init fail\n");
#endif
#ifdef DEV_I2C_7
	dev_i2c[7] = device_get_binding("I2C_7");
	status = k_mutex_init(&i2c_mutex[7]);
	if (status)
		printf("i2c7 mutex init fail\n");
#endif
#ifdef DEV_I2C_8
	dev_i2c[8] = device_get_binding("I2C_8");
	status = k_mutex_init(&i2c_mutex[8]);
	if (status)
		printf("i2c8 mutex init fail\n");
#endif
#ifdef DEV_I2C_9
	dev_i2c[9] = device_get_binding("I2C_9");
	status = k_mutex_init(&i2c_mutex[9]);
	if (status)
		printf("i2c9 mutex init fail\n");
#endif
#ifdef DEV_I2C_10
	dev_i2c[10] = device_get_binding("I2C_10");
	status = k_mutex_init(&i2c_mutex[10]);
	if (status)
		printk("i2c10 mutex init fail\n");
#endif
#ifdef DEV_I2C_11
	dev_i2c[11] = device_get_binding("I2C_11");
	status = k_mutex_init(&i2c_mutex[11]);
	if (status)
		printk("i2c11 mutex init fail\n");
#endif
#ifdef DEV_I2C_12
	dev_i2c[12] = device_get_binding("I2C_12");
	status = k_mutex_init(&i2c_mutex[12]);
	if (status)
		printk("i2c12 mutex init fail\n");
#endif
#ifdef DEV_I2C_13
	dev_i2c[13] = device_get_binding("I2C_13");
	status = k_mutex_init(&i2c_mutex[13]);
	if (status)
		printk("i2c13 mutex init fail\n");
#endif
#ifdef DEV_I2C_14
	dev_i2c[14] = device_get_binding("I2C_14");
	status = k_mutex_init(&i2c_mutex[14]);
	if (status)
		printk("i2c14 mutex init fail\n");
#endif
#ifdef DEV_I2C_15
	dev_i2c[15] = device_get_binding("I2C_15");
	status = k_mutex_init(&i2c_mutex[15]);
	if (status)
		printk("i2c15 mutex init fail\n");
#endif
}

int check_i2c_bus_valid(uint8_t bus)
{
	if (dev_i2c[bus] == NULL) {
		return -1;
	}
	return 0;
}
