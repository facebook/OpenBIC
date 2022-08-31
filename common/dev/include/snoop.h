#ifndef SNOOP_H
#define SNOOP_H

#ifdef CONFIG_SNOOP_ASPEED

#define SENDPOSTCODE_STACK_SIZE 1024
#define SNOOP_STACK_SIZE 512
#define SNOOP_MAX_LEN 240

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

#endif
