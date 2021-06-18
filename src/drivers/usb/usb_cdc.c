#include <stdio.h>
#include "common.h"
#include "util.h"
#include "hal_def.h"
#include "cmsis_os.h"
#include "io.h"
#include <string.h>
#include "device.h"
#include "log.h"
#include "usbdevice.h"
#include "usb_cdc.h"

//#define ASPEED_CDC_SETUP_DEBUG

#ifdef ASPEED_CDC_SETUP_DEBUG
#define USB_CDCSBUG(fmt, args...) printf("%s() " fmt, __FUNCTION__, ## args)
#else
#define USB_CDCSBUG(fmt, args...)
#endif

static struct usb_string_descriptor *usbtty_string_table[STR_COUNT];

/*
 * Serial number
 */
static char serial_number[16];

/* USB Descriptor Strings */
static uint8_t wstrLang[4] = {4,USB_DT_STRING,0x9,0x4};
static uint8_t wstrManufacturer[2 + 2*(sizeof(CONFIG_USBD_MANUFACTURER)-1)];
static uint8_t wstrProduct[2 + 2*(sizeof(CONFIG_USBD_PRODUCT_NAME)-1)];
static uint8_t wstrSerial[2 + 2*(sizeof(serial_number) - 1)];
static uint8_t wstrConfiguration[2 + 2*(sizeof(CONFIG_USBD_CONFIGURATION_STR)-1)];
static uint8_t wstrDataInterface[2 + 2*(sizeof(CONFIG_USBD_DATA_INTERFACE_STR)-1)];
static uint8_t wstrCtrlInterface[2 + 2*(sizeof(CONFIG_USBD_DATA_INTERFACE_STR)-1)];

static struct usb_device_descriptor device_descriptor = {
	.bLength = sizeof(struct usb_device_descriptor),
	.bDescriptorType =	USB_DT_DEVICE,
	.bcdUSB =		USB_BCD_VERSION,
	.bDeviceClass = 0x0,
	.bDeviceSubClass =	0x00,
	.bDeviceProtocol =	0x00,
	.bMaxPacketSize0 =	EP0_MAX_PACKET_SIZE,
	.idVendor =		CONFIG_USBD_VENDORID,
	.idProduct = (CONFIG_USBD_PRODUCTID_CDCACM),
	.bcdDevice =		USBTTY_BCD_DEVICE,
	.iManufacturer =	STR_MANUFACTURER,
	.iProduct =		STR_PRODUCT,
	.iSerialNumber =	STR_SERIAL,
	.bNumConfigurations =	NUM_CONFIGS
};

static struct rs232_emu rs232_desc ={
	.dter		=	115200,
	.stop_bits	=	0x00,
	.parity		=	0x00,
	.data_bits	=	0x08
};

static struct acm_config_desc acm_configuration_descriptors[] = {
	{
		.configuration_desc ={
			.bLength =
				sizeof(struct usb_configuration_descriptor),
			.bDescriptorType = USB_DT_CONFIG,
			.wTotalLength =
				(sizeof(struct acm_config_desc)),
			.bNumInterfaces = NUM_ACM_INTERFACES,
			.bConfigurationValue = 1,
			.iConfiguration = STR_CONFIG,
			.bmAttributes =
				BMATTRIBUTE_SELF_POWERED|BMATTRIBUTE_RESERVED,
			.bMaxPower = USBTTY_MAXPOWER
		},

		.assoc_desc = {
			.bLength = sizeof(struct usb_interface_assoc_descriptor),
			.bDescriptorType =	USB_DT_INTERFACE_ASSOCIATION,

			/* .bFirstInterface =	DYNAMIC, */
			.bInterfaceCount =	2,	// control + data
			.bFunctionClass =	USB_CLASS_COMM,
			.bFunctionSubClass =	USB_CDC_SUBCLASS_ACM,
			.bFunctionProtocol =	USB_CDC_ACM_PROTO_AT_V25TER,
			/* .iFunction = 	DYNAMIC */
		},
		
		/* Interface 1 */
		.interface_desc = {
			.bLength  = sizeof(struct usb_interface_descriptor),
			.bDescriptorType = USB_DT_INTERFACE,
			.bInterfaceNumber = 0,
			.bAlternateSetting = 0,
			.bNumEndpoints = 0x01,
			.bInterfaceClass =
				COMMUNICATIONS_INTERFACE_CLASS_CONTROL,
			.bInterfaceSubClass = COMMUNICATIONS_ACM_SUBCLASS,
			.bInterfaceProtocol = COMMUNICATIONS_V25TER_PROTOCOL,
			.iInterface = STR_DATA_INTERFACE,
		},
		.usb_class_header = {
			.bFunctionLength	=
				sizeof(struct usb_class_header_function_descriptor),
			.bDescriptorType	= CS_INTERFACE,
			.bDescriptorSubtype	= USB_ST_HEADER,
			.bcdCDC	= (110),
		},
		.usb_class_call_mgt = {
			.bFunctionLength	=
				sizeof(struct usb_class_call_management_descriptor),
			.bDescriptorType	= CS_INTERFACE,
			.bDescriptorSubtype	= USB_ST_CMF,
			.bmCapabilities		= 0x00,
			.bDataInterface		= 0x01,
		},
		.usb_class_acm = {
			.bFunctionLength	=
				sizeof(struct usb_class_abstract_control_descriptor),
			.bDescriptorType	= CS_INTERFACE,
			.bDescriptorSubtype	= USB_ST_ACMF,
			.bmCapabilities		= 0x00,
		},
		.usb_class_union = {
			.bFunctionLength	=
				sizeof(struct usb_class_union_function_descriptor),
			.bDescriptorType	= CS_INTERFACE,
			.bDescriptorSubtype	= USB_ST_UF,
			.bMasterInterface	= 0x00,
			.bSlaveInterface0	= 0x01,
		},
		.notification_endpoint = {
			.bLength =
				sizeof(struct usb_endpoint_descriptor),
			.bDescriptorType	= USB_DT_ENDPOINT,
			.bEndpointAddress	= UDC_INT_ENDPOINT | USB_DIR_IN,
			.bmAttributes		= USB_ENDPOINT_XFER_INT,
			.wMaxPacketSize
				= (CONFIG_USBD_SERIAL_INT_PKTSIZE),
			.bInterval		= 0x09,
		},

		/* Interface 2 */
		.data_class_interface = {
			.bLength		=
				sizeof(struct usb_interface_descriptor),
			.bDescriptorType	= USB_DT_INTERFACE,
			.bInterfaceNumber	= 0x01,
			.bAlternateSetting	= 0x00,
			.bNumEndpoints		= 0x02,
			.bInterfaceClass	=
				COMMUNICATIONS_INTERFACE_CLASS_DATA,
			.bInterfaceSubClass	= DATA_INTERFACE_SUBCLASS_NONE,
			.bInterfaceProtocol	= DATA_INTERFACE_PROTOCOL_NONE,
			.iInterface		= STR_CTRL_INTERFACE,
		},
		.data_endpoints = {
			{
				.bLength		=
					sizeof(struct usb_endpoint_descriptor),
				.bDescriptorType	= USB_DT_ENDPOINT,
				.bEndpointAddress	= UDC_OUT_ENDPOINT | USB_DIR_OUT,
				.bmAttributes		=
					USB_ENDPOINT_XFER_BULK,
				.wMaxPacketSize		=
					(CONFIG_USBD_SERIAL_BULK_PKTSIZE),
				.bInterval		= 0x00,
			},
			{
				.bLength		=
					sizeof(struct usb_endpoint_descriptor),
				.bDescriptorType	= USB_DT_ENDPOINT,
				.bEndpointAddress	= UDC_IN_ENDPOINT | USB_DIR_IN,
				.bmAttributes		=
					USB_ENDPOINT_XFER_BULK,
				.wMaxPacketSize		=
					(CONFIG_USBD_SERIAL_BULK_PKTSIZE),
				.bInterval		= 0x00,
			},
		},
	},
};

static struct usb_qualifier_descriptor qualifier_descriptor = {
	.bLength = sizeof(struct usb_qualifier_descriptor),
	.bDescriptorType =	USB_DT_QUAL,
	.bcdUSB =		(USB_BCD_VERSION),
	.bDeviceClass =		COMMUNICATIONS_DEVICE_CLASS,
	.bDeviceSubClass =	0x00,
	.bDeviceProtocol =	0x00,
	.bMaxPacketSize0 =	EP0_MAX_PACKET_SIZE,
	.bNumConfigurations =	NUM_CONFIGS
};


unsigned char string_descriptor0[] = { // available languages  descriptor
    0x04, USB_DESCRIPTOR_TYPE_STRING, //
    0x09, 0x04, //
};

unsigned char string_descriptor1[] = { //
    0x0E, USB_DESCRIPTOR_TYPE_STRING, // bLength, bDscType
    'T', 0x00, //
    'e', 0x00, //
    's', 0x00, //
    't', 0x00, //
    'i', 0x00, //
    '!', 0x00, //
};

unsigned char string_descriptor2[] = { //
    0x20, USB_DESCRIPTOR_TYPE_STRING, //
    'U', 0x00, //
    'S', 0x00, //
    'B', 0x00, //
    ' ', 0x00, //
    'G', 0x00, //
    'e', 0x00, //
    'n', 0x00, //
    'e', 0x00, //
    'r', 0x00, //
    'i', 0x00, //
    'c', 0x00, //
    ' ', 0x00, //
    'C', 0x00, //
    'D', 0x00, //
    'C', 0x00, //
};

void aspeed_cdc_setup(struct usb_device_request *request, struct ep_config *ep)
{
	USB_CDCSBUG(" --> Setup ---\n");

	/* Check direction */
	if ((request->bRequest & USB_REQ_TYPE_MASK) == USB_TYPE_STANDARD) {
		USB_CDCSBUG("xxrequest->bRequest [%x]\n", request->bRequest);
		switch(request->bRequest) {
			case USB_REQ_GET_DESCRIPTOR:
				{
				unsigned char descriptor_type = (request->wValue) >> 8;
				unsigned char descriptor_Index = (request->wValue) & 0xff;
				USB_CDCSBUG("type : %d, idx %d \n", descriptor_type, descriptor_Index);
				switch (descriptor_type) {
					case USB_DESCRIPTOR_TYPE_DEVICE:
						ep->ep_tx_dma = (uint8_t *)&device_descriptor;
						ep->ep_tx_len = sizeof(device_descriptor);
						break;
					case USB_DESCRIPTOR_TYPE_CONFIGURATION:
						ep->ep_tx_dma = (uint8_t *)&acm_configuration_descriptors;
						ep->ep_tx_len = sizeof(acm_configuration_descriptors);
						break;
					case USB_DESCRIPTOR_TYPE_STRING:
						{
						struct usb_string_descriptor *string = usbtty_string_table[descriptor_Index];
						ep->ep_tx_dma = (uint8_t *)string;
						ep->ep_tx_len = string->bLength;
						}
						break;
					case USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER:
						ep->ep_tx_dma = (uint8_t *)&qualifier_descriptor;
						ep->ep_tx_len = sizeof(qualifier_descriptor);
						break;
				};
				}
				break;
			default:
				printf("TODO xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
				break;
		}
	} else {
		USB_CDCSBUG("class request->bRequest [%x]\n", request->bRequest);
		
		switch (request->bRequest){
			case ACM_SET_CONTROL_LINE_STATE:	/* Implies DTE ready */
			case ACM_GET_ENCAPSULATED_RESPONSE :	/* request response */				
				ep->ep_tx_len = 0;
				break;
			case ACM_SEND_ENCAPSULATED_COMMAND :	/* Required */
				printf("todo xxxx xxx\n");
				break;
			case ACM_SET_LINE_ENCODING :		/* DTE stop/parity bits per character */
//				printf("todo out ");
#if 0								 	
				LineCoding.bitrate	  = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
				LineCoding.format	  = pbuf[4];
				LineCoding.paritytype = pbuf[5];
				LineCoding.datatype   = pbuf[6];
				static struct rs232_emu rs232_desc={
						.dter		=	115200,
						.stop_bits	=	0x00,
						.parity 	=	0x00,
						.data_bits	=	0x08
				};
				rs232_desc.data_bits 
#endif								 	
				break;
			case ACM_GET_LINE_ENCODING :		/* request DTE rate, stop/parity bits */
				ep->ep_tx_dma = (uint8_t *)&rs232_desc;
				ep->ep_tx_len = sizeof(rs232_desc);
				break;
			default:
				printf("TODO ~~~~~~~~~~~~ xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
				return;
		}
	}

	USB_CDCSBUG(" --- Setup <---- \n");


}

/* utility function for converting char* to wide string used by USB */
static void str2wide(char *str, uint16_t * wide)
{
	int i;
	for (i = 0; i < strlen (str) && str[i]; i++){
			wide[i] = (uint16_t) str[i];
	}
}

void usbtty_init_strings(void)
{
	struct usb_string_descriptor *string;

	usbtty_string_table[STR_LANG] =
		(struct usb_string_descriptor*)wstrLang;

	string = (struct usb_string_descriptor *) wstrManufacturer;
	string->bLength = sizeof(wstrManufacturer);
	string->bDescriptorType = USB_DT_STRING;
	str2wide (CONFIG_USBD_MANUFACTURER, string->wData);
	usbtty_string_table[STR_MANUFACTURER]=string;


	string = (struct usb_string_descriptor *) wstrProduct;
	string->bLength = sizeof(wstrProduct);
	string->bDescriptorType = USB_DT_STRING;
	str2wide (CONFIG_USBD_PRODUCT_NAME, string->wData);
	usbtty_string_table[STR_PRODUCT]=string;

	char * sn = "000000000000";
	int snlen = strlen(sn);
	if (snlen > sizeof(serial_number) - 1) {
		snlen = sizeof(serial_number) - 1;
	}
	memcpy (serial_number, sn, snlen);
	serial_number[snlen] = '\0';

	string = (struct usb_string_descriptor *) wstrSerial;
	string->bLength = sizeof(serial_number);
	string->bDescriptorType = USB_DT_STRING;
	str2wide (serial_number, string->wData);
	usbtty_string_table[STR_SERIAL]=string;

	string = (struct usb_string_descriptor *) wstrConfiguration;
	string->bLength = sizeof(wstrConfiguration);
	string->bDescriptorType = USB_DT_STRING;
	str2wide (CONFIG_USBD_CONFIGURATION_STR, string->wData);
	usbtty_string_table[STR_CONFIG]=string;

	string = (struct usb_string_descriptor *) wstrDataInterface;
	string->bLength = sizeof(wstrDataInterface);
	string->bDescriptorType = USB_DT_STRING;
	str2wide (CONFIG_USBD_DATA_INTERFACE_STR, string->wData);
	usbtty_string_table[STR_DATA_INTERFACE]=string;

	string = (struct usb_string_descriptor *) wstrCtrlInterface;
	string->bLength = sizeof(wstrCtrlInterface);
	string->bDescriptorType = USB_DT_STRING;
	str2wide (CONFIG_USBD_CTRL_INTERFACE_STR, string->wData);
	usbtty_string_table[STR_CTRL_INTERFACE]=string;

}

