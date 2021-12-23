#ifndef SNOOP_H
#define SNOOP_H

#define SNOOP_STACK_SIZE 500
#define SNOOP_MAX_LEN 244

extern int snoop_read_num;
void copy_snoop_read_buffer( uint8_t offset, int size_num, uint8_t *buffer );

#endif
