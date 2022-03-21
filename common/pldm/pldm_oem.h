#ifndef _PLDM_OEM_H
#define _PLDM_OEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pldm.h"
#include <stdint.h>

#define FIANA 0x00A015
#define IANA_LEN 0x03

/* commands of pldm type 0x3F : PLDM_TYPE_OEM */
#define PLDM_OEM_CMD_ECHO 0x00
#define PLDM_OEM_IPMI_BRIDGE 0x01

struct _cmd_echo_req {
	uint8_t iana[IANA_LEN];
	uint8_t first_data;
} __attribute__((packed));

struct _cmd_echo_resp {
	uint8_t completion_code;
	uint8_t iana[IANA_LEN];
	uint8_t first_data;
} __attribute__((packed));

struct _ipmi_cmd_req {
	uint8_t iana[IANA_LEN];
	uint8_t netfn_lun;
	uint8_t cmd;
	uint8_t first_data;
} __attribute__((packed));

struct _ipmi_cmd_resp {
	uint8_t completion_code;
	uint8_t iana[IANA_LEN];
	uint8_t netfn_lun;
	uint8_t cmd;
	uint8_t ipmi_comp_code;
	uint8_t first_data;
} __attribute__((packed));

uint8_t check_iana(uint8_t *iana);
uint8_t set_iana(uint8_t *buf, uint8_t buf_len);

uint8_t pldm_oem_handler_query(uint8_t code, void **ret_fn);

#ifdef __cplusplus
}
#endif

#endif /* _PLDM_OEM_H */