/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "common.h"
#include "util.h"
#include "hal_def.h"
#include "reset_aspeed.h"
#include "clk_aspeed.h"
#include "cmsis_os.h"
#include "io.h"
#include "wait.h"
#include <string.h>
#include "device.h"
#include "log.h"
#include "usb_aspeed.h"
#include "irq_aspeed.h"
#include "usb_cdc.h"

/* number of endpoints on this UDC */
#define UDC_MAX_ENDPOINTS	24

/*************************************************************************************/
#define ASPEED_USB_CTRL 				0x00
#define ASPEED_USB_CONF 				0x04
#define ASPEED_USB_IER 				0x08
#define ASPEED_USB_ISR 				0x0C
#define ASPEED_USB_EP_ACK_IER 		0x10
#define ASPEED_USB_EP_NAK_IER 		0x14
#define ASPEED_USB_EP_ACK_ISR 		0x18
#define ASPEED_USB_EP_NAK_ISR 		0x1C
#define ASPEED_USB_DEV_RESET 			0x20
#define ASPEED_USB_USB_STS 			0x24
#define ASPEED_USB_EP_DATA 			0x28
#define ASPEED_USB_ISO_TX_FAIL		0x2C
#define ASPEED_USB_EP0_CTRL			0x30
#define ASPEED_USB_EP0_DATA_BUFF		0x34
#define ASPEED_USB_EP1_CTRL			0x38
#define ASPEED_USB_EP1_STS_CHG		0x3C

#define ASPEED_USB_SETUP_DATA0		0x80
#define ASPEED_USB_SETUP_DATA1		0x84

/*  ************************************************************************************/
/* ASPEED_USB_CTRL 		0x00 */
#define ROOT_PHY_CLK_EN				BIT(31)
#define ROOT_PHY_SELF_TEST_EN		BIT(25)
#define ROOT_DN_15K_EN				BIT(24)
#define ROOT_DP_15K_EN				BIT(23)
#define ROOT_FIFO_DYNP_EN			BIT(19)
#define ROOT_EP_LONG_DESC			BIT(18)
#define ROOT_ISO_IN_NULL_RESP		BIT(17)
#define ROOT_SPLIT_IN				BIT(16)
#define ROOT_LOOP_TEST_PASS			BIT(15)
#define ROOT_LOOP_TEST_FINISH		BIT(14)
#define ROOT_BIST_TEST_PASS			BIT(13)
#define ROOT_BIST_ON				BIT(12)
#define ROOT_PHY_RESET_DIS			BIT(11)
#define ROOT_TEST_MODE(x)			((x) << 8)
#define ROOT_FORCE_TIMER_HS			BIT(7)
#define ROOT_FORCE_HS				BIT(6)
#define ROOT_REMOTE_WAKEUP_12MS		BIT(5)
#define ROOT_REMOTE_WAKEUP_EN		BIT(4)
#define ROOT_AUTO_REMOTE_WAKEUP_EN	BIT(3)
#define ROOT_STOP_CLK_IN_SUPEND		BIT(2)
#define ROOT_UPSTREAM_FS			BIT(1)
#define ROOT_UPSTREAM_EN			BIT(0)

/* ASPEED_USB_CONF 		0x04 */
#define CONF_GET_DMA_STS(x)			((x >> 16) & 0xff)
#define CONF_GET_DEV_ADDR(x)		(x & 0x7f)

/* ASPEED_USB_IER 		0x08	*/
#define ISR_USB_CMD_DEADLOCK		BIT(18)
#define ISR_EP_NAK					BIT(17)
#define ISR_EP_ACK_STALL			BIT(16)

#define ISR_SUSPEND_RESUME			BIT(8)
#define ISR_BUS_SUSPEND 			BIT(7)
#define ISR_BUS_RESET 				BIT(6)
#define ISR_EP1_IN_DATA_ACK		BIT(5)
#define ISR_EP0_IN_DATA_NAK		BIT(4)
#define ISR_EP0_IN_ACK_STALL	BIT(3)
#define ISR_EP0_OUT_NAK			BIT(2)
#define ISR_EP0_OUT_ACK_STALL	BIT(1)
#define ISR_EP0_SETUP			BIT(0)

/* ASPEED_USB_EP_ACK_IER 		0x10 */
#define EP14_ISR					BIT(14)
#define EP13_ISR					BIT(13)
#define EP12_ISR					BIT(12)
#define EP11_ISR					BIT(11)
#define EP10_ISR					BIT(10)
#define EP9_ISR						BIT(9)
#define EP8_ISR						BIT(8)
#define EP7_ISR						BIT(7)
#define EP6_ISR						BIT(6)
#define EP5_ISR						BIT(5)
#define EP4_ISR						BIT(4)
#define EP3_ISR						BIT(3)
#define EP2_ISR						BIT(2)
#define EP1_ISR						BIT(1)
#define EP0_ISR						BIT(0)

/* ASPEED_USB_DEV_RESET_ISR 		0x20 */
#define DEV5_SOFT_RESET				BIT(5)
#define DEV4_SOFT_RESET				BIT(4)
#define DEV3_SOFT_RESET				BIT(3)
#define DEV2_SOFT_RESET				BIT(2)
#define DEV1_SOFT_RESET				BIT(1)
#define ROOT_HUB_SOFT_RESET			BIT(0)

/* ASPEED_USB_EP0_CTRL			0x30 */
#define EP0_GET_RX_LEN(x)			((x >> 16) & 0x7f)
#define EP0_TX_LEN(x)				((x & 0x7f) << 8)
#define EP0_RX_BUFF_RDY				BIT(2)
#define EP0_TX_BUFF_RDY				BIT(1)
#define EP0_STALL					BIT(0)

#define AST_UDC_DEV_CTRL 			0x00
#define AST_UDC_DEV_ISR				0x04
#define AST_UDC_DEV_EP0_CTRL		0x08
#define AST_UDC_DEV_EP0_DATA_BUFF	0x0C

/*  ************************************************************************************/
//#define AST_UDC_DEV_CTRL 				0x00
#define DEV_CTRL_DEV_ADDR_MASK		(0x3f << 8)
#define DEV_CTRL_IN_NAK_EN			BIT(6)
#define DEV_CTRL_IN_ACK_STALL_EN	BIT(5)
#define DEV_CTRL_OUT_NAK_EN			BIT(4)
#define DEV_CTRL_OUT_ACK_STALL_EN	BIT(3)
#define DEV_CTRL_SETUP_EN			BIT(2)
#define DEV_CTRL_HIGH_SPEED_MODE	BIT(1)
#define DEV_CTRL_DEV_EN				BIT(0)

//define AST_UDC_DEV_ISR				0x04
#define DEV_CTRL_IN_NAK_INT			BIT(4)
#define DEV_CTRL_IN_ACK_STALL_INT	BIT(3)
#define DEV_CTRL_OUT_NAK_INT		BIT(2)
#define DEV_CTRL_OUT_ACK_STALL_INT	BIT(1)
#define DEV_CTRL_SETUP_INT			BIT(0)

//#define AST_UDC_DEV_EP0_CTRL			0x08
#define DEV_EP0_GET_RX_SIZE(x)			((x >> 16) & 0x7f)
#define DEV_EP0_TX_SIZE_MASK			(0x7f << 8)
#define DEV_EP0_SET_TX_SIZE(x)			((x & 0x7f) << 8)
#define DEV_EP0_RX_BUFF_RDY			BIT(2)
#define DEV_EP0_TX_BUFF_RDY			BIT(1)
#define DEV_EP0_STALL				BIT(0)

/*************************************************************************************/
#define AST_EP_CONFIG					0x00
#define AST_EP_DMA_CTRL				0x04
#define AST_EP_DMA_BUFF				0x08
#define AST_EP_DMA_STS					0x0C

/*************************************************************************************/
//#define AST_EP_CONFIG					0x00
#define EP_SET_MAX_PKT(x)				((x & 0x3ff) << 16)

#define EP_AUTO_DATA_DISABLE			(0x1 << 13)
#define EP_SET_EP_STALL					(0x1 << 12)

#define EP_SET_EP_NUM(x)				((x & 0xf) << 8)

#define EP_SET_TYPE_MASK(x)				((x) << 4)
#define EP_TYPE_BULK_IN					(0x2 << 4)
#define EP_TYPE_BULK_OUT				(0x3 << 4)
#define EP_TYPE_INT_IN					(0x4 << 4)
#define EP_TYPE_INT_OUT					(0x5 << 4)
#define EP_TYPE_ISO_IN					(0x6 << 4)
#define EP_TYPE_ISO_OUT					(0x7 << 4)

#define EP_ALLOCATED_MASK				(0x7 << 1)

#define EP_ENABLE					BIT(0)

//#define AST_EP_DMA_STS					0x0C


//#define AST_EP_DMA_CTRL				0x04

#define EP_SINGLE_DMA_MODE			(0x1 << 1)
/*************************************************************************************/
//#define ASPEED_USB_DEBUG

#ifdef ASPEED_USB_DEBUG
#define USB_DBUG(fmt, args...) printf("%s() " fmt, __FUNCTION__, ## args)
#else
#define USB_DBUG(fmt, args...)
#endif

//#define ASPEED_USB_SETUP_DEBUG

#ifdef ASPEED_USB_SETUP_DEBUG
#define USB_SBUG(fmt, args...) printf("%s() " fmt, __FUNCTION__, ## args)
#else
#define USB_SBUG(fmt, args...)
#endif

//#define ASPEED_USB_EP_DEBUG
#ifdef ASPEED_USB_EP_DEBUG
#define USB_EPDBUG(fmt, args...) printf("%s() " fmt, __FUNCTION__, ## args)
#else
#define USB_EPDBUG(fmt, args...)
#endif
/*************************************************************************************/
void aspeed_usb_ep_handle(usb_t *obj, int ep_num)
{
	uint32_t ret;
	uint32_t flag;

	USB_EPDBUG("aspeed_usb_ep_handle [%d] isout %d \n", ep_num, obj->ep[ep_num].ep_isout);
	if(obj->ep[ep_num].ep_isout) {
		//read flag
		flag = 0x00000002U;
	} else {
		//write flag
		flag = 0x00000001U;
	}
	ret = osEventFlagsSet(obj->evt_id, flag);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);

}

void aspeed_usb_setup_handle(usb_t *obj)
{
	aspeed_device_t *device = obj->device;
	struct usb_device_request *request = (void *) (device->base + ASPEED_USB_SETUP_DATA0);

	USB_SBUG(" --> Setup ---\n");

	if (request->bmRequestType & USB_DIR_IN) {
		obj->ep[0].ep_isout = 0;
		USB_SBUG("Device -> Host \n");
	} else {
		obj->ep[0].ep_isout = 1;
		USB_SBUG("Host -> Device \n");
	}

	obj->ep[0].ep_tx_len = 0;
	obj->ep[0].ep_tx_last = 0;

	/* Check direction */
	switch(request->bRequest) {
		case USB_REQ_SET_ADDRESS:
			USB_SBUG("SetAddr %x \n", request->wValue);
			writel((request->wValue & 0x7f), device->base + ASPEED_USB_CONF);
			writel(EP0_TX_BUFF_RDY, device->base + ASPEED_USB_EP0_CTRL);
			break;
		case USB_REQ_SET_CONFIGURATION:
			USB_SBUG("set configuration \n");
			writel(EP0_TX_BUFF_RDY, device->base + ASPEED_USB_EP0_CTRL);
			obj->configure = 1;
			break;
		default:
			aspeed_cdc_setup(request, &obj->ep[0]);
			if(obj->ep[0].ep_isout && request->wLength) {
				writel(TO_PHY_ADDR(obj->ep[0].ep_rx_dma), device->base + ASPEED_USB_EP0_DATA_BUFF);
				writel(EP0_RX_BUFF_RDY, device->base + ASPEED_USB_EP0_CTRL);
				USB_SBUG("trigger rx\n");
			} else {
				int ep0_tx_len = 0;
				if(obj->ep[0].ep_tx_len > request->wLength) {
					ep0_tx_len = request->wLength;
					obj->ep[0].ep_tx_len = request->wLength;
				} else {
					if(obj->ep[0].ep_tx_len > EP0_MAX_PACKET_SIZE) {
						ep0_tx_len = EP0_MAX_PACKET_SIZE;
					} else
						ep0_tx_len = obj->ep[0].ep_tx_len;
				}
				obj->ep[0].ep_tx_last = ep0_tx_len;
				USB_SBUG("trigger tx : [%d/%d] \n", ep0_tx_len, obj->ep[0].ep_tx_len);
				writel(TO_PHY_ADDR(obj->ep[0].ep_tx_dma), device->base + ASPEED_USB_EP0_DATA_BUFF);
				writel(EP0_TX_LEN(ep0_tx_len), device->base + ASPEED_USB_EP0_CTRL);
				writel(EP0_TX_LEN(ep0_tx_len) | EP0_TX_BUFF_RDY, device->base + ASPEED_USB_EP0_CTRL);
			}

			break;
	}

	USB_SBUG(" --- Setup <---- \n");
}

void aspeed_usb_handler(usb_t *obj)
{
	aspeed_device_t *device = obj->device;
	uint32_t isr = readl(device->base + ASPEED_USB_ISR);
	uint32_t ep_isr = 0;
	int i = 0;

	if(!(isr & 0x1ffff))
		return;

	if(isr & ISR_BUS_RESET) {
//		USB_DBUG("ISR_BUS_RESET \n");
		obj->configure = 0;
		writel(ISR_BUS_RESET, device->base + ASPEED_USB_ISR);
	}

	if(isr & ISR_BUS_SUSPEND) {
		//Suspend, we don't handle this in sample
//		USB_DBUG("ISR_BUS_SUSPEND \n");
		writel(ISR_BUS_SUSPEND, device->base + ASPEED_USB_ISR);
	}

	if(isr & ISR_SUSPEND_RESUME) {
		//Suspend, we don't handle this in sample
//		USB_DBUG("ISR_SUSPEND_RESUME \n");
		writel(ISR_SUSPEND_RESUME, device->base + ASPEED_USB_ISR);
	}

	if(isr & ISR_EP0_IN_ACK_STALL) {
		USB_SBUG("EP0_IN_ACK_STALL \n");
		writel(ISR_EP0_IN_ACK_STALL, device->base + ASPEED_USB_ISR);

		if(obj->ep[0].ep_tx_last != obj->ep[0].ep_tx_len) {
			int tx_len = obj->ep[0].ep_tx_len - obj->ep[0].ep_tx_last;
			if(tx_len > EP0_MAX_PACKET_SIZE)
				tx_len = EP0_MAX_PACKET_SIZE;
			writel(TO_PHY_ADDR(obj->ep[0].ep_tx_dma + obj->ep[0].ep_tx_last),
						device->base + ASPEED_USB_EP0_DATA_BUFF);
			writel(EP0_TX_LEN(tx_len), device->base + ASPEED_USB_EP0_CTRL);
			writel(EP0_TX_LEN(tx_len) | EP0_TX_BUFF_RDY, device->base + ASPEED_USB_EP0_CTRL);
			USB_SBUG("next tx trigger %d \n", tx_len);
			obj->ep[0].ep_tx_last += tx_len;
		} else {
			if(!obj->ep[0].ep_isout) {
				USB_SBUG("rx trigger \n");
				writel(TO_PHY_ADDR(obj->ep[0].ep_rx_dma), device->base + ASPEED_USB_EP0_DATA_BUFF);
				writel(EP0_RX_BUFF_RDY, device->base + ASPEED_USB_EP0_CTRL);
			}
		}
	}

	if(isr & ISR_EP0_OUT_NAK) {
		USB_SBUG("ISR_EP0_OUT_NAK \n");
		writel(ISR_EP0_OUT_NAK, device->base + ASPEED_USB_ISR);
	}

	if(isr & ISR_EP0_OUT_ACK_STALL) {
		USB_SBUG("ISR_EP0_OUT_ACK_STALL \n");
		writel(ISR_EP0_OUT_ACK_STALL, device->base + ASPEED_USB_ISR);
		//TODO check leng
		if(obj->ep[0].ep_isout) {
			writel(EP0_TX_BUFF_RDY, device->base + ASPEED_USB_EP0_CTRL);
		}
	}

	if(isr & ISR_EP0_IN_DATA_NAK) {
		//IN NAK, we don't handle this in sample
//		USB_SBUG("ISR_EP0_IN_DATA_NAK \n");
		writel(ISR_EP0_IN_DATA_NAK, device->base + ASPEED_USB_ISR);
	}

	if(isr & ISR_EP0_SETUP) {
		writel(ISR_EP0_SETUP, device->base + ASPEED_USB_ISR);
		aspeed_usb_setup_handle(obj);
	}

	if(isr & ISR_EP1_IN_DATA_ACK) {
		//HUB Bitmap control
		USB_DBUG("ERROR EP1 IN\n");
		writel(ISR_EP1_IN_DATA_ACK, device->base + ASPEED_USB_ISR);
		writel(0x00, device->base + ASPEED_USB_EP1_STS_CHG);
	}

	if(isr & ISR_EP_ACK_STALL) {
//		USB_DBUG("ISR_EP_ACK_STALL\n");
		ep_isr = readl(device->base + ASPEED_USB_EP_ACK_ISR);
		for(i = 0; i < 15; i++) {
			if(ep_isr & (0x1 << i)) {
				writel(0x1 <<  i, device->base + ASPEED_USB_EP_ACK_ISR);
				aspeed_usb_ep_handle(obj, i + 1);
			}
		}
	}

	if(isr & ISR_EP_NAK) {
		printf("ISR_EP_NAK\n");
		USB_DBUG("ISR_EP_NAK \n");
		writel(ISR_EP_NAK, device->base + ASPEED_USB_ISR);
	}

}

void aspeed_usb_isr(void)
{
	struct usb_s *obj;
	obj = (struct usb_s *)aspeed_irq_get_isr_context(USB_IRQn);
	aspeed_usb_handler(obj);
}

int aspeed_ep_read(usb_t *obj, int ep_num, uint8_t *buff, int buff_len)
{
	uint32_t ep_reg = 0;
	aspeed_device_t *device = obj->device;
	ep_reg = device->base + 0x200 + (0x10 * (ep_num - 1));
	int nbytes;

	USB_EPDBUG("[%x] aspeed_ep_read ep[%d] buff %x len : %d \n", ep_reg, ep_num, buff, buff_len);
	//rx dma
	writel(TO_PHY_ADDR(buff), ep_reg + AST_EP_DMA_BUFF);
	writel(0x1, ep_reg + AST_EP_DMA_STS);

	uint32_t ret;
	ret = osEventFlagsWait(obj->evt_id, 0x00000002U, osFlagsWaitAll, osWaitForever);
	if (!(ret & 0x2))
	{
		log_error("osError: %d\n", ret);
		return 0;
	}

	nbytes = (readl(ep_reg + AST_EP_DMA_STS) >> 16) & 0x7ff;
	USB_EPDBUG("[%d] rx %d bytes \n", ep_num, nbytes);

    return nbytes;
}

int aspeed_ep_write(usb_t *obj, int ep_num, uint8_t *buff, int buff_len)
{
	uint32_t ret;
	uint32_t ep_reg = 0;
	aspeed_device_t *device = obj->device;
	ep_reg = device->base + 0x200 + (0x10 * (ep_num - 1));

	USB_EPDBUG("[%x] aspeed_ep_write ep[%d] buff %x len : %d \n",ep_reg, ep_num, buff, buff_len);
	//tx dma
	writel(TO_PHY_ADDR(buff), ep_reg + AST_EP_DMA_BUFF);
	writel(buff_len << 16, ep_reg + AST_EP_DMA_STS);
	writel(buff_len << 16 | 0x1, ep_reg + AST_EP_DMA_STS);

	ret = osEventFlagsWait(obj->evt_id, 0x00000001U, osFlagsWaitAny, osWaitForever);
	if (!(ret & 0x1))
	{
		log_error("osError: %d\n", ret);
		return 1;
	}

    return 0;
}

void aspeed_usb_ep_configure(usb_t *obj)
{
	aspeed_device_t *device = obj->device;
	uint32_t ep_conf = 0;
	uint32_t ep_reg = 0;
	int ep_num = 0;

	//ep1 : bulk in 1024 byte
	ep_num = 1;
	obj->ep[ep_num].ep_isout = 0;
	ep_conf = EP_SET_MAX_PKT(0);
	ep_conf |= EP_SET_EP_NUM(ep_num);
	ep_conf |= EP_TYPE_BULK_IN;

	ep_reg = device->base + 0x200 + (0x10 * (ep_num - 1));

	writel(0x4, ep_reg + AST_EP_DMA_CTRL);
	writel(0x2, ep_reg + AST_EP_DMA_CTRL);
	writel(0, ep_reg + AST_EP_DMA_STS);
	writel(ep_conf | EP_ENABLE, ep_reg + AST_EP_CONFIG);
	log_debug("ep config [%d] %x \n", ep_num, ep_conf);
	//ep2 : buik out 1024 byte
	ep_num = 2;
	obj->ep[ep_num].ep_isout = 1;
	ep_conf = EP_SET_MAX_PKT(0);
	ep_conf |= EP_SET_EP_NUM(ep_num);
	ep_conf |= EP_TYPE_BULK_OUT;

	ep_reg = device->base + 0x200 + (0x10 * (ep_num - 1));

	writel(0x4, ep_reg + AST_EP_DMA_CTRL);
	writel(0x2, ep_reg + AST_EP_DMA_CTRL);
	writel(0, ep_reg + AST_EP_DMA_STS);
	writel(ep_conf | EP_ENABLE, ep_reg + AST_EP_CONFIG);
	log_debug("ep config [%d] %x \n", ep_num, ep_conf);

	//rx dma
	writel(TO_PHY_ADDR(obj->ep[ep_num].ep_rx_dma), ep_reg + AST_EP_DMA_BUFF);
	writel(0 << 16, ep_reg + AST_EP_DMA_STS);
	writel((0 << 16) | 0x1, ep_reg + AST_EP_DMA_STS);

	//ep3 : int in 64byte
	ep_num = 3;
	obj->ep[ep_num].ep_isout = 0;
	ep_conf = EP_SET_MAX_PKT(64);
	ep_conf |= EP_SET_EP_NUM(ep_num);
	ep_conf |= EP_TYPE_INT_IN;

	ep_reg = device->base + 0x200 + (0x10 * (ep_num - 1));

	writel(0x4, ep_reg + AST_EP_DMA_CTRL);
	writel(0x2, ep_reg + AST_EP_DMA_CTRL);
	writel(0, ep_reg + AST_EP_DMA_STS);
	writel(ep_conf | EP_ENABLE, ep_reg + AST_EP_CONFIG);
	log_debug("ep config [%d] %x \n", ep_num, ep_conf);


}

/* Connect the USB device to the bus */
void aspeed_usb_connect(usb_t *obj)
{
	aspeed_device_t *device = obj->device;

	USB_DBUG("UDC connect \n");
	writel(readl(device->base + ASPEED_USB_CTRL) | ROOT_UPSTREAM_EN, device->base + ASPEED_USB_CTRL);

}

/* Disconnect the USB device to the bus */
void aspeed_usb_disconnect(usb_t *obj)
{
	aspeed_device_t *device = obj->device;

	USB_DBUG("UDC disconnect \n");
	writel(readl(device->base + ASPEED_USB_CTRL) & ~ROOT_UPSTREAM_EN, device->base + ASPEED_USB_CTRL);
}

hal_status_t aspeed_usb_init(usb_t *obj)
{
	aspeed_device_t *device = obj->device;

	if (device->init) {
		printf("usb is occupied\n");
		return HAL_BUSY;
	}

	aspeed_reset_assert(obj->device);
	aspeed_clk_enable(obj->device);
	aspeed_wait_ms(10);
	aspeed_reset_deassert(obj->device);

	device->init = 1;
	obj->configure = 0;

	log_debug("aspeed usb init usb base %x \n", device->base);

	writel(ROOT_PHY_CLK_EN | ROOT_PHY_RESET_DIS, device->base + ASPEED_USB_CTRL);

	aspeed_wait_ms(1);
	writel(0, device->base + ASPEED_USB_DEV_RESET);

	writel(0x1ffff, device->base + ASPEED_USB_IER);
	writel(0x7ffff, device->base + ASPEED_USB_ISR);

	writel(0x7ffff, device->base + ASPEED_USB_EP_ACK_ISR);
	writel(0x7ffff, device->base + ASPEED_USB_EP_ACK_IER);

	writel(0, device->base + ASPEED_USB_EP0_CTRL);
	writel(0, device->base + ASPEED_USB_EP1_CTRL);

	aspeed_usb_ep_configure(obj);
	/* init event ID for ISR */
	obj->evt_id = osEventFlagsNew(NULL);
	if (obj->evt_id == NULL)
		log_error("fail to create evt_id\n");

	aspeed_irq_register(USB_IRQn, (uint32_t)aspeed_usb_isr, obj);

	return 0;
}

