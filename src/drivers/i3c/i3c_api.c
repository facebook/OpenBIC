#include "i3c_api.h"
#include "hal_def.h"
#include "objects.h"
#include "i3c_aspeed.h"
#include "log.h"

#define ALIGN_16(x)			(((x) + 15) >> 4) << 4	
/* slave device RX queues */
#define I3C_RX_Q_ENTRIES		4
#define I3C_RX_BUF_SIZE			ALIGN_16(CONFIG_I3C_MRL)

/* master device IBI queues */
#define I3C_IBI_Q_ENTRIES		4
#define I3C_IBI_BUF_SIZE		ALIGN_16(CONFIG_I3C_IBI_MAX_PAYLOAD)

void i3c_init_direct(i3c_t *obj, const i3c_pinmap_t *pinmap)
{
	UNUSED(pinmap);
}

void i3c_spd_read(i3c_t *obj, uint32_t idx, uint32_t addr, uint32_t len, uint8_t *buf)
{
	i3c_usr_xfer_t xfers[2];
	uint8_t tx_buf[2] = {addr, 0};

	xfers[0].rnw = 0;
	xfers[0].data.in = tx_buf;
	xfers[0].len = 2;
	
	xfers[1].rnw = 1;
	xfers[1].data.out = buf;
	xfers[1].len = len;
	aspeed_i3c_priv_xfer(obj, idx,  xfers, 2);
}

uint8_t i3c_spd_byte_read(i3c_t *obj, uint32_t idx, uint32_t addr)
{
	uint32_t tmp;
	
	i3c_spd_read(obj, idx, addr, 1, (uint8_t *)&tmp);
	return (uint8_t)tmp;
}

void i3c_spd_write(i3c_t *obj, uint32_t idx, uint32_t addr, uint32_t len, uint8_t *buf)
{
	i3c_usr_xfer_t xfer;
	uint8_t *tx_buf;

	tx_buf = malloc(len + 2);
	memset(tx_buf, 0, len + 2);
	tx_buf[0] = addr;
	memcpy(&tx_buf[2], buf, len);

	xfer.rnw = 0;
	xfer.data.in = tx_buf;
	xfer.len = len + 2;
	aspeed_i3c_priv_xfer(obj, idx,  &xfer, 1);

	free(tx_buf);
}

void i3c_spd_byte_write(i3c_t *obj, uint32_t idx, uint32_t addr, uint8_t data)
{
	uint32_t tmp = (uint32_t)data;
	
	i3c_spd_write(obj, idx, addr, 1, (uint8_t *)&tmp);
}

static void i3c_init_mqueue(i3c_t *obj, int entries, int buf_size)
{
	int i;
	uint8_t *buf, *msgs;

	msgs = malloc(sizeof(i3c_msg_t) * entries);
	buf = malloc(entries * buf_size);

	obj->mq.id = osMessageQueueNew(entries, sizeof(i3c_msg_t), NULL);;
	obj->mq.ptr = 0;
	obj->mq.entries = entries;
	obj->mq.msgs = (i3c_msg_t *)msgs;

	for (i = 0 ; i < entries; i++) {
		obj->mq.msgs[i].buf = buf + (buf_size * i);
	}
}

void i3c_init(i3c_t *obj)
{
	if (obj->role) {
		/* i3c slave driver init */
		i3c_init_mqueue(obj, I3C_RX_Q_ENTRIES, I3C_RX_BUF_SIZE);
	} else {
		/* i3c master driver init */
		i3c_init_mqueue(obj, I3C_IBI_Q_ENTRIES, I3C_IBI_BUF_SIZE);
	}

	if (!obj->global->device->init)
		aspeed_i3c_global_init(obj->global);
	
	aspeed_i3c_init(obj);
}

void i3c_free(i3c_t *obj)
{
	free(obj->mq.msgs);
	free(obj->mq.msgs[0].buf);
}

int i3c_mqueue_receive(i3c_t *obj, i3c_msg_t *msg)
{	
	osMessageQueueGet(obj->mq.id, msg, 0, osWaitForever);
#if 0
	log_trace("i3c mqueue: msg.len = %d\n", msg->len);
	for (int i = 0; i < msg->len; i++) {
		log_trace("i3c mqueue: msg.buf[%d] = %02x\n", i, msg->buf[i]);
	}
#endif
	return 0;
}