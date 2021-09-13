#ifndef IPMB_H
#define IPMB_H

#include <devicetree.h>

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb0))
#define DEV_IPMB_0
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb1))
#define DEV_IPMB_1
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb2))
#define DEV_IPMB_2
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb3))
#define DEV_IPMB_3
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb4))
#define DEV_IPMB_4
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb5))
#define DEV_IPMB_5
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb6))
#define DEV_IPMB_6
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb7))
#define DEV_IPMB_7
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb8))
#define DEV_IPMB_8
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(ipmb9))
#define DEV_IPMB_9
#endif

#define DEBUG_IPMI 0

#define SEQ_NUM 64
#define MAX_DATA_QUENE 15
#define MAX_IPMB_IDX 5
#define IPMB_timeout_S 1
#define SELF_IPMB_IF  0x00
#define IPMI_DATA_MAX_LENGTH 512
#define IPMB_REQ_HEADER_LENGTH      6
#define IPMB_RESP_HEADER_LENGTH     7
#define IPMI_MSG_MAX_LENGTH (IPMI_DATA_MAX_LENGTH + IPMB_RESP_HEADER_LENGTH)
#define IPMB_MAX_RETRIES 5
#define IPMB_TXQUEUE_LEN        10
#define IPMB_RXQUEUE_LEN        1
#define IPMI_HEADER_CHECKSUM_POSITION 2 
#define IPMB_NETFN_MASK         0xFC
#define IPMB_DEST_LUN_MASK      0x03
#define IPMB_SEQ_MASK           0xFC
#define IPMB_SRC_LUN_MASK       0x03
#define CLIENT_NOTIFY_TIMEOUT   5
#define IPMB_RETRY_DELAY_ms 500
#define IPMB_MQUEUE_POLL_DELAY_ms 10
#define IPMB_SEQ_TIMEOUT_ms 1000
#define IPMB_SEQ_TIMEOUT_STACK_SIZE 500

#define Enable  1
#define Disable 0

#define IS_RESPONSE(msg) (msg.netfn & 0x01) 

enum{
  Self_IFs = 0x0,
  ME_IPMB_IFs = 0x01,
  BMC_IPMB_IFs = 0x02,
  HOST_KCS_IFs = 0x03,
  SERVER_IPMB_IFs = 0x04,
  EXP1_IPMB_IFs = 0x05,
  EXP2_IPMB_IFs = 0x06,

  BMC_USB_IFs = 0x10,
  Reserve_IFs,
};

enum{
  I2C_IF,
  I3C_IF,
  Reserve_IF,
};

typedef struct IPMB_config {
  uint8_t index;
  uint8_t Inf;
  uint8_t Inf_source;
  uint8_t bus;
  uint8_t target_addr;
  bool EnStatus;
  uint8_t slave_addr;
  char *Rx_attr_name;
  char *Tx_attr_name;
} IPMB_config;

extern IPMB_config *IPMB_config_table;

typedef enum ipmb_error {
  ipmb_error_success = 0,             /**< Generic no-error flag  */
  ipmb_error_unknown,                 /**< Unknown error */
  ipmb_error_failure,                 /**< Generic failure on IPMB */
  ipmb_error_timeout,                 /**< Error raised when a message takes too long to be responded */
  ipmb_error_invalid_req,             /**< A invalid request was received */
  ipmb_error_hdr_chksum,              /**< Invalid header checksum from incoming message */
  ipmb_error_msg_chksum,              /**< Invalid message checksum from incoming message */
  ipmb_error_queue_creation,          /**< Client queue couldn't be created. Invalid pointer to handler was given */
  ipmb_error_get_messageQueue,        /**< Failure on getting queue message */
} ipmb_error;

typedef struct ipmi_msg {
  uint8_t dest_addr;                  /**< Destination slave address */
  uint8_t netfn;                      /**< Net Function */
  uint8_t dest_LUN;                   /**< Destination LUN (Logical Unit Number) */
  uint8_t hdr_chksum;                 /**< Connection Header Checksum */
  uint8_t src_addr;                   /**< Source slave address */
  uint8_t seq_source;                 /**< Source sequence Number */
  uint8_t seq_target;                 /**< Target sequence Number */
  uint8_t seq;                        /**< Sequence Number */
  uint8_t InF_source;                 /**< Source bridge interface */
  uint8_t InF_target;                 /**< Target bridge interface */
  uint8_t src_LUN;                    /**< Source LUN (Logical Unit Number) */
  uint8_t cmd;                        /**< Command */
  uint8_t completion_code;            /**< Completion Code*/
  uint16_t data_len;                   /**< Amount of valid bytes in #data buffer */
  uint8_t data[IPMI_MSG_MAX_LENGTH];  /**< Data buffer >
                                         * Data field has 24 bytes:
                                         * 32 (Max IPMI msg len) - 7 header bytes - 1 final chksum byte
                                         */
  uint32_t timestamp;                 /**< Tick count at the beginning of the process */
  uint8_t msg_chksum;                 /**< Message checksum */
} __packed __aligned(4) ipmi_msg;

typedef struct ipmi_msg_cfg {
  ipmi_msg buffer;                    /**< IPMI Message */
  uint8_t retries;                    /**< Current retry counter */
  struct ipmi_msg_cfg *next;
} __packed __aligned(4) ipmi_msg_cfg;


void ipmb_init ( void );
ipmb_error ipmb_send_request ( ipmi_msg * req, uint8_t bus );
ipmb_error ipmb_send_response ( ipmi_msg * resp, uint8_t bus );
ipmb_error ipmb_read ( ipmi_msg * msg, uint8_t bus );

#endif
