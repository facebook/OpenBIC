#ifndef SNOOP_H
#define SNOOP_H

#define SNOOP_STACK_SIZE 500
#define SNOOP_MAX_LEN 236

extern int snoop_read_num;
void copy_snoop_read_buffer( uint8_t offset, int size_num, uint8_t *buffer );
bool get_postcode_ok();
void reset_postcode_ok();

#endif
