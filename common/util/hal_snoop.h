#ifndef SNOOP_H
#define SNOOP_H

#define SNOOP_STACK_SIZE 400
#define SENDPOSTCODE_STACK_SIZE 1000
#define SNOOP_MAX_LEN 244

enum {
	copy_all_postcode,
	copy_specific_postcode,
};

extern int snoop_read_num;
void copy_snoop_read_buffer(uint8_t offset, int size_num, uint8_t *buffer, uint8_t copy_mode);
bool get_postcode_ok();
void reset_postcode_ok();
void snoop_start_thread();
void snoop_abort_thread();
void init_send_postcode_thread();

#endif
