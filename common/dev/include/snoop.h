#ifndef SNOOP_H
#define SNOOP_H

#define SNOOP_STACK_SIZE 400
#define SENDPOSTCODE_STACK_SIZE 1000
#define SNOOP_MAX_LEN 244

enum POSTCODE_COPY_TYPES {
	COPY_ALL_POSTCODE,
	COPY_SPECIFIC_POSTCODE,
};

extern int snoop_read_num;
void copy_snoop_read_buffer(uint8_t offset, int size_num, uint8_t *buffer, uint8_t copy_mode);
bool get_postcode_ok();
void reset_postcode_ok();
void init_snoop_thread();
void abort_snoop_thread();
void init_send_postcode_thread();

#endif
