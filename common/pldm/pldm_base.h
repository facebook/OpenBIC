#ifndef _PLDM_BASE_H
#define _PLDM_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pldm.h"
#include <stdint.h>

/* commands of pldm type 0x00 : PLDM_TYPE_CTRL_DISCOV */
#define PLDM_BASE_CMD_CODE_SETTID 0x01
#define PLDM_BASE_CMD_CODE_GETTID 0x02
#define PLDM_BASE_CMD_CODE_GET_PLDM_VER 0x03
#define PLDM_BASE_CMD_CODE_GET_PLDM_TYPE 0x04
#define PLDM_BASE_CMD_CODE_GET_PLDM_CMDS 0x05

#define DEFAULT_TID 0x86

struct _set_tid_req {
	uint8_t tid;
} __attribute__((packed));

struct _set_tid_resp {
	uint8_t completion_code;
} __attribute__((packed));

struct _get_tid_resp {
	uint8_t completion_code;
	uint8_t tid;
} __attribute__((packed));

uint8_t pldm_base_handler_query(uint8_t code, void **ret_fn);

#ifdef __cplusplus
}
#endif

#endif /* _PLDM_BASE_H */