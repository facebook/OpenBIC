#ifndef USBLIB_H
#define USBLIB_H

#include <stdint.h>

/*
 * Defines
 */
#define NUM_CONFIGS    1
#define MAX_INTERFACES 2

#define ACM_TX_ENDPOINT 3
#define ACM_RX_ENDPOINT 2
#define GSERIAL_TX_ENDPOINT 2
#define GSERIAL_RX_ENDPOINT 1
#define NUM_ACM_INTERFACES 2
#define NUM_GSERIAL_INTERFACES 1
#define CONFIG_USBD_DATA_INTERFACE_STR "Bulk Data Interface"
#define CONFIG_USBD_CTRL_INTERFACE_STR "Control Interface"

#define USBTTY_DEVICE_CLASS	COMMUNICATIONS_DEVICE_CLASS

#define USBTTY_BCD_DEVICE	0x00
#define USBTTY_MAXPOWER		0x00

#define STR_LANG		0x00
#define STR_MANUFACTURER	0x01
#define STR_PRODUCT		0x02
#define STR_SERIAL		0x03
#define STR_CONFIG		0x04
#define STR_DATA_INTERFACE	0x05
#define STR_CTRL_INTERFACE	0x06
#define STR_COUNT		0x07

/* Endpoint parameters */
#define MAX_ENDPOINTS		4

#define EP0_MAX_PACKET_SIZE     64

#define UDC_INT_ENDPOINT				3
#define UDC_INT_PACKET_SIZE				64
#define UDC_OUT_ENDPOINT				2
#define UDC_OUT_PACKET_SIZE				512
#define UDC_IN_ENDPOINT					1
#define UDC_IN_PACKET_SIZE				64
#define UDC_BULK_PACKET_SIZE			512
#define UDC_BULK_HS_PACKET_SIZE			512

#define CONFIG_USBD_SERIAL_OUT_ENDPOINT UDC_OUT_ENDPOINT
#define CONFIG_USBD_SERIAL_OUT_PKTSIZE	512
#define CONFIG_USBD_SERIAL_IN_ENDPOINT	UDC_IN_ENDPOINT
#define CONFIG_USBD_SERIAL_IN_PKTSIZE	512
#define CONFIG_USBD_SERIAL_INT_ENDPOINT UDC_INT_ENDPOINT
#define CONFIG_USBD_SERIAL_INT_PKTSIZE	64
#define CONFIG_USBD_SERIAL_BULK_PKTSIZE	512

/* Endpoint configuration
 *
 * Per endpoint configuration data. Used to track which function driver owns
 * an endpoint.
 *
 */
struct usb_endpoint_instance {
	int endpoint_address;	/* logical endpoint address */

	/* control */
	int status;		/* halted */
	int state;		/* available for use by bus interface driver */

	/* receive side */
//	struct urb_link rcv;	/* received urbs */
//	struct urb_link rdy;	/* empty urbs ready to receive */
//	struct urb *rcv_urb;	/* active urb */
	int rcv_attributes;	/* copy of bmAttributes from endpoint descriptor */
	int rcv_packetSize;	/* maximum packet size from endpoint descriptor */
	int rcv_transferSize;	/* maximum transfer size from function driver */
	int rcv_queue;

	/* transmit side */
//	struct urb_link tx;	/* urbs ready to transmit */
//	struct urb_link done;	/* transmitted urbs */
//	struct urb *tx_urb;	/* active urb */
	int tx_attributes;	/* copy of bmAttributes from endpoint descriptor */
	int tx_packetSize;	/* maximum packet size from endpoint descriptor */
	int tx_transferSize;	/* maximum transfer size from function driver */
	int tx_queue;

	int sent;		/* data already sent */
	int last;		/* data sent in last packet XXX do we need this */
};

/*
 * communications class types
 *
 * c.f. CDC  USB Class Definitions for Communications Devices
 * c.f. WMCD USB CDC Subclass Specification for Wireless Mobile Communications Devices
 *
 */

#define CLASS_BCD_VERSION		0x0110

/* c.f. CDC 4.1 Table 14 */
#define COMMUNICATIONS_DEVICE_CLASS	0x02

/* c.f. CDC 4.2 Table 15 */
#define COMMUNICATIONS_INTERFACE_CLASS_CONTROL	0x02
#define COMMUNICATIONS_INTERFACE_CLASS_DATA		0x0A
#define COMMUNICATIONS_INTERFACE_CLASS_VENDOR	0x0FF

/* c.f. CDC 4.3 Table 16 */
#define COMMUNICATIONS_NO_SUBCLASS		0x00
#define COMMUNICATIONS_DLCM_SUBCLASS	0x01
#define COMMUNICATIONS_ACM_SUBCLASS		0x02
#define COMMUNICATIONS_TCM_SUBCLASS		0x03
#define COMMUNICATIONS_MCCM_SUBCLASS	0x04
#define COMMUNICATIONS_CCM_SUBCLASS		0x05
#define COMMUNICATIONS_ENCM_SUBCLASS	0x06
#define COMMUNICATIONS_ANCM_SUBCLASS	0x07

/* c.f. WMCD 5.1 */
#define COMMUNICATIONS_WHCM_SUBCLASS	0x08
#define COMMUNICATIONS_DMM_SUBCLASS		0x09
#define COMMUNICATIONS_MDLM_SUBCLASS	0x0a
#define COMMUNICATIONS_OBEX_SUBCLASS	0x0b

/* c.f. CDC 4.4 Table 17 */
#define COMMUNICATIONS_NO_PROTOCOL		0x00
#define COMMUNICATIONS_V25TER_PROTOCOL	0x01	/*Common AT Hayes compatible*/

/* c.f. CDC 4.5 Table 18 */
#define DATA_INTERFACE_CLASS		0x0a

/* c.f. CDC 4.6 No Table */
#define DATA_INTERFACE_SUBCLASS_NONE	0x00	/* No subclass pertinent */

/* c.f. CDC 4.7 Table 19 */
#define DATA_INTERFACE_PROTOCOL_NONE	0x00	/* No class protcol required */


/* c.f. CDC 5.2.3 Table 24 */
#define CS_INTERFACE		0x24
#define CS_ENDPOINT			0x25

/*
 * bDescriptorSubtypes
 *
 * c.f. CDC 5.2.3 Table 25
 * c.f. WMCD 5.3 Table 5.3
 */

#define USB_ST_HEADER		0x00
#define USB_ST_CMF			0x01
#define USB_ST_ACMF			0x02
#define USB_ST_DLMF			0x03
#define USB_ST_TRF			0x04
#define USB_ST_TCLF			0x05
#define USB_ST_UF			0x06
#define USB_ST_CSF			0x07
#define USB_ST_TOMF			0x08
#define USB_ST_USBTF		0x09
#define USB_ST_NCT			0x0a
#define USB_ST_PUF			0x0b
#define USB_ST_EUF			0x0c
#define USB_ST_MCMF			0x0d
#define USB_ST_CCMF			0x0e
#define USB_ST_ENF			0x0f
#define USB_ST_ATMNF		0x10

#define USB_ST_WHCM			0x11
#define USB_ST_MDLM			0x12
#define USB_ST_MDLMD		0x13
#define USB_ST_DMM			0x14
#define USB_ST_OBEX			0x15
#define USB_ST_CS			0x16
#define USB_ST_CSD			0x17
#define USB_ST_TCM			0x18

/* endpoint modifiers
 * static struct usb_endpoint_description function_default_A_1[] = {
 *
 *     {this_endpoint: 0, attributes: CONTROL,	 max_size: 8,  polling_interval: 0 },
 *     {this_endpoint: 1, attributes: BULK,	 max_size: 64, polling_interval: 0, direction: IN},
 *     {this_endpoint: 2, attributes: BULK,	 max_size: 64, polling_interval: 0, direction: OUT},
 *     {this_endpoint: 3, attributes: INTERRUPT, max_size: 8,  polling_interval: 0},
 *
 *
 */
#define OUT		0x00
#define IN		0x80

#define CONTROL		0x00
#define ISOCHRONOUS	0x01
#define BULK		0x02
#define INTERRUPT	0x03


/* configuration modifiers
 */
#define BMATTRIBUTE_RESERVED		0x80
#define BMATTRIBUTE_SELF_POWERED	0x40

/*
 * standard usb descriptor structures
 */

struct usb_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;	/* 0x5 */
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
} __attribute__ ((packed));

struct usb_interface_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;	/* 0x04 */
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
} __attribute__ ((packed));

/* USB_DT_INTERFACE_ASSOCIATION: groups interfaces */
struct usb_interface_assoc_descriptor {
	uint8_t  bLength;
	uint8_t  bDescriptorType;

	uint8_t  bFirstInterface;
	uint8_t  bInterfaceCount;
	uint8_t  bFunctionClass;
	uint8_t  bFunctionSubClass;
	uint8_t  bFunctionProtocol;
	uint8_t  iFunction;
} __attribute__ ((packed));

#define USB_DT_INTERFACE_ASSOCIATION_SIZE	8

struct usb_configuration_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;	/* 0x2 */
	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration;
	uint8_t bmAttributes;
	uint8_t bMaxPower;
} __attribute__ ((packed));

struct usb_device_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;	/* 0x01 */
	uint16_t bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize0;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t iManufacturer;
	uint8_t iProduct;
	uint8_t iSerialNumber;
	uint8_t bNumConfigurations;
} __attribute__ ((packed));


struct usb_qualifier_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;

	uint16_t bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize0;
	uint8_t bNumConfigurations;
	uint8_t breserved;
} __attribute__ ((packed));


struct usb_string_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;	/* 0x03 */
	uint16_t wData[0];
} __attribute__ ((packed));

struct usb_generic_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
} __attribute__ ((packed));


/*
 * communications class descriptor structures
 *
 * c.f. CDC 5.2 Table 25c
 */

struct usb_class_function_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
} __attribute__ ((packed));

struct usb_class_function_descriptor_generic {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bmCapabilities;
} __attribute__ ((packed));

struct usb_class_header_function_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x00 */
	uint16_t bcdCDC;
} __attribute__ ((packed));

struct usb_class_call_management_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x01 */
	uint8_t bmCapabilities;
	uint8_t bDataInterface;
} __attribute__ ((packed));

struct usb_class_abstract_control_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x02 */
	uint8_t bmCapabilities;
} __attribute__ ((packed));

struct usb_class_direct_line_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x03 */
} __attribute__ ((packed));

struct usb_class_telephone_ringer_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x04 */
	uint8_t bRingerVolSeps;
	uint8_t bNumRingerPatterns;
} __attribute__ ((packed));

struct usb_class_telephone_call_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x05 */
	uint8_t bmCapabilities;
} __attribute__ ((packed));

struct usb_class_union_function_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x06 */
	uint8_t bMasterInterface;
	/* uint8_t bSlaveInterface0[0]; */
	uint8_t bSlaveInterface0;
} __attribute__ ((packed));

struct usb_class_country_selection_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x07 */
	uint8_t iCountryCodeRelDate;
	uint16_t wCountryCode0[0];
} __attribute__ ((packed));


struct usb_class_telephone_operational_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x08 */
	uint8_t bmCapabilities;
} __attribute__ ((packed));


struct usb_class_usb_terminal_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x09 */
	uint8_t bEntityId;
	uint8_t bInterfaceNo;
	uint8_t bOutInterfaceNo;
	uint8_t bmOptions;
	uint8_t bChild0[0];
} __attribute__ ((packed));

struct usb_class_network_channel_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x0a */
	uint8_t bEntityId;
	uint8_t iName;
	uint8_t bChannelIndex;
	uint8_t bPhysicalInterface;
} __attribute__ ((packed));

struct usb_class_protocol_unit_function_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x0b */
	uint8_t bEntityId;
	uint8_t bProtocol;
	uint8_t bChild0[0];
} __attribute__ ((packed));

struct usb_class_extension_unit_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x0c */
	uint8_t bEntityId;
	uint8_t bExtensionCode;
	uint8_t iName;
	uint8_t bChild0[0];
} __attribute__ ((packed));

struct usb_class_multi_channel_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x0d */
	uint8_t bmCapabilities;
} __attribute__ ((packed));

struct usb_class_capi_control_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x0e */
	uint8_t bmCapabilities;
} __attribute__ ((packed));

struct usb_class_ethernet_networking_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x0f */
	uint8_t iMACAddress;
	uint32_t bmEthernetStatistics;
	uint16_t wMaxSegmentSize;
	uint16_t wNumberMCFilters;
	uint8_t bNumberPowerFilters;
} __attribute__ ((packed));

struct usb_class_atm_networking_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x10 */
	uint8_t iEndSystermIdentifier;
	uint8_t bmDataCapabilities;
	uint8_t bmATMDeviceStatistics;
	uint16_t wType2MaxSegmentSize;
	uint16_t wType3MaxSegmentSize;
	uint16_t wMaxVC;
} __attribute__ ((packed));


struct usb_class_mdlm_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x12 */
	uint16_t bcdVersion;
	uint8_t bGUID[16];
} __attribute__ ((packed));

struct usb_class_mdlmd_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;	/* 0x13 */
	uint8_t bGuidDescriptorType;
	uint8_t bDetailData[0];

} __attribute__ ((packed));

/*
 * HID class descriptor structures
 *
 * c.f. HID 6.2.1
 */

struct usb_class_hid_descriptor {
    uint8_t 	      bLength;
    uint8_t 	      bDescriptorType;
    uint16_t	      bcdCDC;
    uint8_t 	      bCountryCode;
    uint8_t 	      bNumDescriptors;	/* 0x01 */
    uint8_t 	      bDescriptorType0;
    uint16_t	      wDescriptorLength0;
    /* optional descriptors are not supported. */
} __attribute__((packed));

struct usb_class_report_descriptor {
    uint8_t 	      bLength;	/* dummy */
    uint8_t 	      bDescriptorType;
    uint16_t	      wLength;
    uint8_t 		bData[0];
} __attribute__((packed));

/*
 * descriptor union structures
 */

struct usb_descriptor {
	union {
		struct usb_generic_descriptor generic;
		struct usb_endpoint_descriptor endpoint;
		struct usb_interface_descriptor interface;
		struct usb_configuration_descriptor configuration;
		struct usb_device_descriptor device;
		struct usb_string_descriptor string;
	} descriptor;

} __attribute__ ((packed));

struct usb_class_descriptor {
	union {
		struct usb_class_function_descriptor function;
		struct usb_class_function_descriptor_generic generic;
		struct usb_class_header_function_descriptor header_function;
		struct usb_class_call_management_descriptor call_management;
		struct usb_class_abstract_control_descriptor abstract_control;
		struct usb_class_direct_line_descriptor direct_line;
		struct usb_class_telephone_ringer_descriptor telephone_ringer;
		struct usb_class_telephone_operational_descriptor telephone_operational;
		struct usb_class_telephone_call_descriptor telephone_call;
		struct usb_class_union_function_descriptor union_function;
		struct usb_class_country_selection_descriptor country_selection;
		struct usb_class_usb_terminal_descriptor usb_terminal;
		struct usb_class_network_channel_descriptor network_channel;
		struct usb_class_extension_unit_descriptor extension_unit;
		struct usb_class_multi_channel_descriptor multi_channel;
		struct usb_class_capi_control_descriptor capi_control;
		struct usb_class_ethernet_networking_descriptor ethernet_networking;
		struct usb_class_atm_networking_descriptor atm_networking;
		struct usb_class_mdlm_descriptor mobile_direct;
		struct usb_class_mdlmd_descriptor mobile_direct_detail;
		struct usb_class_hid_descriptor hid;
	} descriptor;

} __attribute__ ((packed));

/*
 * Device and/or Interface Class codes
 */
#define USB_CLASS_PER_INTERFACE		0	/* for DeviceClass */
#define USB_CLASS_AUDIO			1
#define USB_CLASS_COMM			2
#define USB_CLASS_HID			3
#define USB_CLASS_PHYSICAL		5
#define USB_CLASS_PRINTER		7
#define USB_CLASS_MASS_STORAGE		8
#define USB_CLASS_HUB			9
#define USB_CLASS_DATA			10
#define USB_CLASS_APP_SPEC		0xfe
#define USB_CLASS_VENDOR_SPEC		0xff

/*
 * USB types
 */
#define USB_TYPE_STANDARD		(0x00 << 5)
#define USB_TYPE_CLASS			(0x01 << 5)
#define USB_TYPE_VENDOR			(0x02 << 5)
#define USB_TYPE_RESERVED		(0x03 << 5)

/*
 * USB recipients
 */
#define USB_RECIP_DEVICE		0x00
#define USB_RECIP_INTERFACE		0x01
#define USB_RECIP_ENDPOINT		0x02
#define USB_RECIP_OTHER			0x03

/*
 * USB directions
 */
#define USB_DIR_OUT			0
#define USB_DIR_IN			0x80

/*
 * Descriptor types
 */
#define USB_DT_DEVICE			0x01
#define USB_DT_CONFIG			0x02
#define USB_DT_STRING			0x03
#define USB_DT_INTERFACE		0x04
#define USB_DT_ENDPOINT			0x05

#define USB_DT_QUAL			0x06

#define USB_DT_OTHER_SPEED_CONFIG	0x07
#define USB_DT_INTERFACE_POWER		0x08
/* these are from a minor usb 2.0 revision (ECN) */
#define USB_DT_OTG			0x09
#define USB_DT_DEBUG			0x0a
#define USB_DT_INTERFACE_ASSOCIATION	0x0b

#define USB_DT_HID			(USB_TYPE_CLASS | 0x01)
#define USB_DT_REPORT			(USB_TYPE_CLASS | 0x02)
#define USB_DT_PHYSICAL			(USB_TYPE_CLASS | 0x03)
#define USB_DT_HUB			(USB_TYPE_CLASS | 0x09)

/*
 * Descriptor sizes per descriptor type
 */
#define USB_DT_DEVICE_SIZE		18
#define USB_DT_CONFIG_SIZE		9
#define USB_DT_INTERFACE_SIZE		9
#define USB_DT_ENDPOINT_SIZE		7
#define USB_DT_ENDPOINT_AUDIO_SIZE	9	/* Audio extension */
#define USB_DT_HUB_NONVAR_SIZE		7
#define USB_DT_HID_SIZE			9

/*
 * Endpoints
 */
#define USB_ENDPOINT_NUMBER_MASK	0x0f	/* in bEndpointAddress */
#define USB_ENDPOINT_DIR_MASK		0x80

#define USB_ENDPOINT_XFERTYPE_MASK	0x03	/* in bmAttributes */
#define USB_ENDPOINT_XFER_CONTROL	0
#define USB_ENDPOINT_XFER_ISOC		1
#define USB_ENDPOINT_XFER_BULK		2
#define USB_ENDPOINT_XFER_INT		3

/*
 * USB Packet IDs (PIDs)
 */
#define USB_PID_UNDEF_0			       0xf0
#define USB_PID_OUT			       0xe1
#define USB_PID_ACK			       0xd2
#define USB_PID_DATA0			       0xc3
#define USB_PID_PING			       0xb4	/* USB 2.0 */
#define USB_PID_SOF			       0xa5
#define USB_PID_NYET			       0x96	/* USB 2.0 */
#define USB_PID_DATA2			       0x87	/* USB 2.0 */
#define USB_PID_SPLIT			       0x78	/* USB 2.0 */
#define USB_PID_IN			       0x69
#define USB_PID_NAK			       0x5a
#define USB_PID_DATA1			       0x4b
#define USB_PID_PREAMBLE		       0x3c	/* Token mode */
#define USB_PID_ERR			       0x3c	/* USB 2.0: handshake mode */
#define USB_PID_SETUP			       0x2d
#define USB_PID_STALL			       0x1e
#define USB_PID_MDATA			       0x0f	/* USB 2.0 */

/*
 * Standard requests
 */
#define USB_REQ_GET_STATUS		0x00
#define USB_REQ_CLEAR_FEATURE		0x01
#define USB_REQ_SET_FEATURE		0x03
#define USB_REQ_SET_ADDRESS		0x05
#define USB_REQ_GET_DESCRIPTOR		0x06
#define USB_REQ_SET_DESCRIPTOR		0x07
#define USB_REQ_GET_CONFIGURATION	0x08
#define USB_REQ_SET_CONFIGURATION	0x09
#define USB_REQ_GET_INTERFACE		0x0A
#define USB_REQ_SET_INTERFACE		0x0B
#define USB_REQ_SYNCH_FRAME		0x0C

#define USBD_DEVICE_REQUESTS(x) (((unsigned int)x <= USB_REQ_SYNCH_FRAME) ? usbd_device_requests[x] : "UNKNOWN")

/*
 * HID requests
 */
#define USB_REQ_GET_REPORT		0x01
#define USB_REQ_GET_IDLE		0x02
#define USB_REQ_GET_PROTOCOL		0x03
#define USB_REQ_SET_REPORT		0x09
#define USB_REQ_SET_IDLE		0x0A
#define USB_REQ_SET_PROTOCOL		0x0B


/*
 * USB Spec Release number
 */

#define USB_BCD_VERSION			0x0200

/*
 * Device Requests	(c.f Table 9-2)
 */

#define USB_REQ_DIRECTION_MASK		0x80
#define USB_REQ_TYPE_MASK		0x60
#define USB_REQ_RECIPIENT_MASK		0x1f

#define USB_REQ_DEVICE2HOST		0x80
#define USB_REQ_HOST2DEVICE		0x00

#define USB_REQ_TYPE_STANDARD		0x00
#define USB_REQ_TYPE_CLASS		0x20
#define USB_REQ_TYPE_VENDOR		0x40

#define USB_REQ_RECIPIENT_DEVICE	0x00
#define USB_REQ_RECIPIENT_INTERFACE	0x01
#define USB_REQ_RECIPIENT_ENDPOINT	0x02
#define USB_REQ_RECIPIENT_OTHER		0x03

/*
 * get status bits
 */

#define USB_STATUS_SELFPOWERED		0x01
#define USB_STATUS_REMOTEWAKEUP		0x02

#define USB_STATUS_HALT			0x01

/*
 * descriptor types
 */

#define USB_DESCRIPTOR_TYPE_DEVICE			0x01
#define USB_DESCRIPTOR_TYPE_CONFIGURATION		0x02
#define USB_DESCRIPTOR_TYPE_STRING			0x03
#define USB_DESCRIPTOR_TYPE_INTERFACE			0x04
#define USB_DESCRIPTOR_TYPE_ENDPOINT			0x05
#define USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER		0x06
#define USB_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION	0x07
#define USB_DESCRIPTOR_TYPE_INTERFACE_POWER		0x08
#define USB_DESCRIPTOR_TYPE_HID				0x21
#define USB_DESCRIPTOR_TYPE_REPORT			0x22

#define USBD_DEVICE_DESCRIPTORS(x) (((unsigned int)x <= USB_DESCRIPTOR_TYPE_INTERFACE_POWER) ? \
		usbd_device_descriptors[x] : "UNKNOWN")

/*
 * standard feature selectors
 */
#define USB_ENDPOINT_HALT		0x00
#define USB_DEVICE_REMOTE_WAKEUP	0x01
#define USB_TEST_MODE			0x02


/* USB Requests
 *
 */

struct usb_device_request {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} __attribute__ ((packed));


/* USB Status
 *
 */
typedef enum urb_send_status {
	SEND_IN_PROGRESS,
	SEND_FINISHED_OK,
	SEND_FINISHED_ERROR,
	RECV_READY,
	RECV_OK,
	RECV_ERROR
} urb_send_status_t;

/*
 * Device State (c.f USB Spec 2.0 Figure 9-1)
 *
 * What state the usb device is in.
 *
 * Note the state does not change if the device is suspended, we simply set a
 * flag to show that it is suspended.
 *
 */
typedef enum usb_device_state {
	STATE_INIT,		/* just initialized */
	STATE_CREATED,		/* just created */
	STATE_ATTACHED,		/* we are attached */
	STATE_POWERED,		/* we have seen power indication (electrical bus signal) */
	STATE_DEFAULT,		/* we been reset */
	STATE_ADDRESSED,	/* we have been addressed (in default configuration) */
	STATE_CONFIGURED,	/* we have seen a set configuration device command */
	STATE_UNKNOWN,		/* destroyed */
} usb_device_state_t;

#define USBD_DEVICE_STATE(x) (((unsigned int)x <= STATE_UNKNOWN) ? usbd_device_states[x] : "UNKNOWN")

/*
 * Device status
 *
 * Overall state
 */
typedef enum usb_device_status {
	USBD_OPENING,		/* we are currently opening */
	USBD_OK,		/* ok to use */
	USBD_SUSPENDED,		/* we are currently suspended */
	USBD_CLOSING,		/* we are currently closing */
} usb_device_status_t;

#define USBD_DEVICE_STATUS(x) (((unsigned int)x <= USBD_CLOSING) ? usbd_device_status[x] : "UNKNOWN")


#endif
