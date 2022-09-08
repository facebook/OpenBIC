#include "hal_i3c.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr.h>
#include <logging/log.h>
#include "libutil.h"

LOG_MODULE_REGISTER(hal_i3c);

static const struct device *dev_i3c[I3C_MAX_NUM];
static const struct device *dev_i3c_smq[I3C_MAX_NUM];

int i3c_slave_mqueue_read(const struct device *dev, uint8_t *dest, int budget);
int i3c_slave_mqueue_write(const struct device *dev, uint8_t *src, int size);

/**
 * @brief api to read i3c message from target message queue
 * 
 * @param msg i3c message structure
 * @return 0: complete read data from target message
 */
int i3c_smq_read(I3C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -EINVAL);
	
	int ret;
	if (!dev_i3c[msg->bus]) {
		LOG_ERR("[%s] bus%u did not define\n", __func__, msg->bus);
		return -ENODEV;
	}

	ret = i3c_slave_mqueue_read(dev_i3c_smq[msg->bus], &msg->data[0], msg->rx_len);
	if (ret < 0) {
		LOG_ERR("[%s] bus%u message queue was empty\n", __func__, msg->bus);
		return -ENODATA;
	}

	return I3C_SMQ_SUCCESS;
}

/**
 * @brief api to write i3c message to target message queue
 * 
 * @param msg i3c message structure
 * @return 0: api to write i3c message to target message queue
 */
int i3c_smq_write(I3C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -EINVAL);

	int ret;
	if (!dev_i3c[msg->bus]) {
		LOG_ERR("[%s] bus%u did not define\n", __func__, msg->bus);
		return -ENODEV;
	}

	ret = i3c_slave_mqueue_write(dev_i3c[msg->bus], &msg->data[0], msg->tx_len);
	return ret;
}

void util_init_i3c(void)
{
#ifdef DEV_I3C_0
	dev_i3c[0] = device_get_binding("I3C_0");
#endif
#ifdef DEV_I3C_1
	dev_i3c[1] = device_get_binding("I3C_1");
#endif
#ifdef DEV_I3C_2
	dev_i3c[2] = device_get_binding("I3C_2");
#endif
#ifdef DEV_I3C_3
	dev_i3c[3] = device_get_binding("I3C_3");
#endif
#ifdef DEV_I3C_4
	dev_i3c[4] = device_get_binding("I3C_4");
#endif
#ifdef DEV_I3C_5
	dev_i3c[5] = device_get_binding("I3C_5");
#endif
#ifdef DEV_I3C_6
	dev_i3c[6] = device_get_binding("I3C_6");
#endif
#ifdef DEV_I3C_7
	dev_i3c[7] = device_get_binding("I3C_7");
#endif

#ifdef DEV_I3CSMQ_0
	dev_i3c_smq[0] = device_get_binding("I3C_SMQ_0");
#endif
#ifdef DEV_I3CSMQ_1
	dev_i3c_smq[1] = device_get_binding("I3C_SMQ_1");
#endif
#ifdef DEV_I3CSMQ_2
	dev_i3c_smq[2] = device_get_binding("I3C_SMQ_2");
#endif
#ifdef DEV_I3CSMQ_3
	dev_i3c_smq[3] = device_get_binding("I3C_SMQ_3");
#endif
#ifdef DEV_I3CSMQ_4
	dev_i3c_smq[4] = device_get_binding("I3C_SMQ_4");
#endif
#ifdef DEV_I3CSMQ_5
	dev_i3c_smq[5] = device_get_binding("I3C_SMQ_5");
#endif
#ifdef DEV_I3CSMQ_6
	dev_i3c_smq[6] = device_get_binding("I3C_SMQ_6");
#endif
#ifdef DEV_I3CSMQ_7
	dev_i3c_smq[7] = device_get_binding("I3C_SMQ_7");
#endif
}
