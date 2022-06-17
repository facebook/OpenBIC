#ifndef PLAT_FRU_H
#define PLAT_FRU_H

#define MB_FRU_BUS 0x05
#define MB_FRU_ADDR 0x54

#define RISER_FRU_BUS 0x04
#define RISER_FRU_ADDR 0x51

enum {
	MB_FRU_ID,
	RISER_FRU_ID,
	// OTHER_FRU_ID,
	MAX_FRU_ID,
};

#endif
