#ifndef PLAT_FRU_H
#define PLAT_FRU_H

#define MB_FRU_PORT 0x01
#define MB_FRU_ADDR 0x50

#define IOM_FRU_PORT 0x07
#define IOM_FRU_ADDR 0x50

enum {
	MB_FRU_ID,
	IOM_FRU_ID,
	// OTHER_FRU_ID,
	MAX_FRU_ID,
};

#endif