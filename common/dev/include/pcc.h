#ifndef PCC_H
#define PCC_H

#ifdef CONFIG_PCC_ASPEED

#define PCC_STACK_SIZE 512
#define PCC_BUFFER_LEN 1024

uint16_t copy_pcc_read_buffer(uint16_t start, uint16_t length, uint8_t *buffer,
			      uint16_t buffer_len);
void pcc_init();
void reset_pcc_buffer();
bool get_4byte_postcode_ok();
void reset_4byte_postcode_ok();

#endif

#endif
