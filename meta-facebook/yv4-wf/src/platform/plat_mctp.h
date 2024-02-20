#ifndef _PLAT_MCTP_h
#define _PLAT_MCTP_h
#include <kernel.h>
#include "storage_handler.h"
#include "pldm.h"

/* init the mctp moduel for platform */
void plat_mctp_init(void);
uint8_t plat_get_mctp_port_count();
mctp_port *plat_get_mctp_port(uint8_t index);
void send_cmd_to_dev_handler(struct k_work *work);
void send_cmd_to_dev(struct k_timer *timer);
uint8_t plat_get_eid();

#endif /* _PLAT_MCTP_h */
