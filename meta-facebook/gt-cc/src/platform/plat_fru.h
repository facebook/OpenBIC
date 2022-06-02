#ifndef PLAT_FRU_H
#define PLAT_FRU_H

#define SWB_FRU_PORT 0x05
#define SWB_FRU_ADDR (0xA8 >> 1)
#define SWB_FRU_MUX_ADDR (0xE0 >> 1)
#define SWB_FRU_MUX_CHAN 7
#define FIO_FRU_PORT 0x04
#define FIO_FRU_ADDR (0xA2 >> 1)

enum {
	SWB_FRU_ID,
	FIO_FRU_ID,
	// OTHER_FRU_ID,
	MAX_FRU_ID,
};

#endif
