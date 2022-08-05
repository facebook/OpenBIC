#ifndef ISL69254IRAZ_T_H
#define ISL69254IRAZ_T_H

#define isl69254iraz_t_checksum_length 4

bool isl69254iraz_t_get_checksum(uint8_t bus, uint8_t target_addr, uint8_t *checksum);
bool isl69254iraz_t_get_remaining_write(uint8_t bus, uint8_t target_addr, uint8_t *remain_write);
#endif
