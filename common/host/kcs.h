#ifndef KCS_H
#define KCS_H

#define KCS_POLL_stack_STACK_SIZE 2500
#define KCS_POLLING_INTERVAL 100
#define KCS_buff_size 256

#define DEBUG_KCS 0

struct kcs_request {
	uint8_t netfn;
	uint8_t cmd;
	uint8_t data[0];
};

struct kcs_response {
	uint8_t netfn;
	uint8_t cmd;
	uint8_t cmplt_code;
	uint8_t data[0];
};

void kcs_write(uint8_t *buf, uint32_t buf_sz);
void kcs_init(void);
bool get_kcs_ok();
void reset_kcs_ok();

#endif
