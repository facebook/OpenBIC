#ifndef _PLAT_MCTP_h
#define _PLAT_MCTP_h

/* init the mctp moduel for platform */
void plat_mctp_init(void);
void send_cmd_to_dev(struct k_timer *timer);
void send_cmd_to_dev_handler(struct k_work *work);

extern struct pldm_variable_field nic_vesion[];

#endif /* _PLAT_MCTP_h */
