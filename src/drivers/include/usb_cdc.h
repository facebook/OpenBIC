#include <stdio.h>
#include "usbdevice.h"
#include "usb_aspeed.h"


#ifndef CONFIG_USBD_VENDORID
#define CONFIG_USBD_VENDORID		0x1D6B	/* Linux Foundation */
#endif

#define CONFIG_USBD_PRODUCTID_CDCACM	0x0104	/* CDC ACM */

#define CONFIG_USBD_MANUFACTURER	"AST1030"
#define CONFIG_USBD_PRODUCT_NAME	"AST1030"
#define CONFIG_USBD_CONFIGURATION_STR	"TTY via USB"

#define USB_CDC_SUBCLASS_ACM			0x02
#define USB_CDC_SUBCLASS_ETHERNET		0x06
#define USB_CDC_SUBCLASS_WHCM			0x08
#define USB_CDC_SUBCLASS_DMM			0x09
#define USB_CDC_SUBCLASS_MDLM			0x0a
#define USB_CDC_SUBCLASS_OBEX			0x0b
#define USB_CDC_SUBCLASS_EEM			0x0c
#define USB_CDC_SUBCLASS_NCM			0x0d
#define USB_CDC_SUBCLASS_MBIM			0x0e

#define USB_CDC_PROTO_NONE			0

#define USB_CDC_ACM_PROTO_AT_V25TER		1
#define USB_CDC_ACM_PROTO_AT_PCCA101		2
#define USB_CDC_ACM_PROTO_AT_PCCA101_WAKE	3
#define USB_CDC_ACM_PROTO_AT_GSM		4
#define USB_CDC_ACM_PROTO_AT_3G			5
#define USB_CDC_ACM_PROTO_AT_CDMA		6
#define USB_CDC_ACM_PROTO_VENDOR		0xff

#define USB_CDC_PROTO_EEM			7

#define USB_CDC_NCM_PROTO_NTB			1
#define USB_CDC_MBIM_PROTO_NTB			2

/*
 * Static CDC ACM specific descriptors
 */

struct acm_config_desc {
	struct usb_configuration_descriptor configuration_desc;

	struct usb_interface_assoc_descriptor assoc_desc;
	/* Master Interface */
	struct usb_interface_descriptor interface_desc;

	struct usb_class_header_function_descriptor usb_class_header;
	struct usb_class_call_management_descriptor usb_class_call_mgt;
	struct usb_class_abstract_control_descriptor usb_class_acm;
	struct usb_class_union_function_descriptor usb_class_union;
	struct usb_endpoint_descriptor notification_endpoint;

	/* Slave Interface */
	struct usb_interface_descriptor data_class_interface;
	struct usb_endpoint_descriptor data_endpoints[2];
} __attribute__((packed));


/* ACM Control Requests */
#define ACM_SEND_ENCAPSULATED_COMMAND	0x00
#define ACM_GET_ENCAPSULATED_RESPONSE	0x01
#define ACM_SET_COMM_FEATURE		0x02
#define ACM_GET_COMM_FEATRUE		0x03
#define ACM_CLEAR_COMM_FEATURE		0x04
#define ACM_SET_LINE_ENCODING		0x20
#define ACM_GET_LINE_ENCODING		0x21
#define ACM_SET_CONTROL_LINE_STATE	0x22
#define ACM_SEND_BREAK			0x23

/* ACM Notification Codes */
#define ACM_NETWORK_CONNECTION		0x00
#define ACM_RESPONSE_AVAILABLE		0x01
#define ACM_SERIAL_STATE		0x20

/* Format of response expected by a ACM_GET_LINE_ENCODING request */
struct rs232_emu{
		unsigned long dter;
		unsigned char stop_bits;
		unsigned char parity;
		unsigned char data_bits;
}__attribute__((packed));


void aspeed_cdc_setup(struct usb_device_request *request, struct ep_config *ep);
void usbtty_init_strings(void);

