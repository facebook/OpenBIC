#ifndef _PLDM_H
#define _PLDM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mctp.h"
#include "pldm_base.h"
#include "pldm_oem.h"
#include <stdint.h>
#include <sys/printk.h>
#include <zephyr.h>

#define PLDM_DEBUG 1

#define PLDM_SUCCESS 0
#define PLDM_ERROR 1
#define PLDM_LATER_RESP 2

#define MONITOR_THREAD_STACK_SIZE 1024

/* generic pldm completion codes  */
#define PLDM_BASE_CODES_SUCCESS 0x00
#define PLDM_BASE_CODES_ERROR 0x01
#define PLDM_BASE_CODES_ERROR_INVALID_DATA 0x02
#define PLDM_BASE_CODES_ERROR_INVALID_LENGTH 0x03
#define PLDM_BASE_CODES_ERROR_NOT_READY 0x04
#define PLDM_BASE_CODES_ERROR_UNSUPPORT_PLDM_CMD 0x05
#define PLDM_BASE_CODES_ERROR_UNSUPPORT_PLDM_TYPE 0x20

#define PLDM_MAX_DATA_SIZE 256

typedef uint8_t (*pldm_cmd_proc_fn)(void *, uint8_t *, uint16_t, uint8_t *, uint16_t *, void *);

typedef enum {
	PLDM_TYPE_BASE = 0x00,
	PLDM_TYPE_SMBIOS,
	PLDM_TYPE_PLAT_MON_CTRL,
	PLDM_TYPE_BIOS_CTRL_CONF,
	PLDM_TYPE_FW_UPDATE = 0x05,
	PLDM_TYPE_OEM = 0x3F
} PLDM_TYPE;

typedef struct _pldm_cmd_handler {
	uint8_t cmd_code;
	pldm_cmd_proc_fn fn;
} pldm_cmd_handler;

typedef struct __attribute__((packed)) {
	uint8_t msg_type : 7;
	uint8_t ic : 1;

	union {
		struct {
			uint8_t inst_id : 5;
			uint8_t rsvd : 1;
			uint8_t d : 1;
			uint8_t rq : 1;
		};
		uint8_t req_d_id;
	};

	uint8_t pldm_type : 6;
	uint8_t ver : 2;
	uint8_t cmd;
} pldm_hdr;

typedef struct __attribute__((packed)) {
	pldm_hdr common_hdr;
	uint8_t resp_comp_code;
} pldm_resp_hdr;

typedef struct _pldm_msg {
	pldm_hdr hdr;
	uint8_t *buf;
	uint16_t len;
	mctp_ext_params ext_params;
	void (*recv_resp_cb_fn)(void *, uint8_t *, uint16_t);
	void *recv_resp_cb_args;
	uint16_t timeout_ms;
	void (*timeout_cb_fn)(void *);
	void *timeout_cb_fn_args;
} pldm_msg;

typedef struct _pldm {
	/* pldm message response timeout prcoess resource */
	k_tid_t monitor_task;
	struct k_thread thread_data;
	K_KERNEL_STACK_MEMBER(monitor_thread_stack, MONITOR_THREAD_STACK_SIZE);

	/* store the msg that are not yet to receive the response */
	sys_slist_t wait_recv_resp_list;
	struct k_mutex wait_recv_resp_list_mutex;

	/* store the msg that are not yet to send the response */
	sys_slist_t wait_send_resp_list;
	struct k_mutex wait_send_resp_list_mutex;

	void *interface; /* the pldm module over which interface, as mctp */
	uint8_t user_idx; /* the alias index for this pldm instance from application
                       layer */
} pldm_t;

struct pldm_variable_field {
	uint8_t *ptr;
	size_t length;
};

struct pldm_get_firmware_parameters_resp {
	uint8_t completion_code;
	union {
		struct {
			uint8_t fail_recovery : 1;
			uint8_t fail_retry : 1;
			uint8_t func_during_update : 1;
			uint8_t partial_update : 1;
			uint8_t update_mode_restrict : 4;
			/* Bit [31:8] reserved */
			uint8_t : 8;
			uint16_t : 16;
		};
		uint32_t capabilities_during_update;
	};
	uint16_t comp_count;
	uint8_t active_comp_image_set_ver_str_type;
	uint8_t active_comp_image_set_ver_str_len;
	uint8_t pending_comp_image_set_ver_str_type;
	uint8_t pending_comp_image_set_ver_str_len;
} __attribute__((packed));

/* the pldm command handler */
uint8_t mctp_pldm_cmd_handler(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params);

/* send the pldm command message through mctp */
uint8_t mctp_pldm_send_msg(void *mctp_p, pldm_msg *msg);

pldm_t *pldm_init(void *interface, uint8_t user_idx);

#ifdef __cplusplus
}
#endif

#endif /* _PLDM_H */