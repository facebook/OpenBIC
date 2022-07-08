#ifndef PLAT_FRU_H
#define PLAT_FRU_H

#define MB_FRU_PORT 0x01
#define MB_FRU_ADDR 0x54

#define DPV2_FRU_PORT 0x08
#define DPV2_FRU_ADDR 0x51

enum {
	MB_FRU_ID,
	DPV2_FRU_ID,
	// OTHER_FRU_ID,
	MAX_FRU_ID,
};

#endif
