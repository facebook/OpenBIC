#ifndef PLAT_DEF_H
#define PLAT_DEF_H

#define sys_class_1 0
#define sys_class_2 1

enum BIC_BOARD_REVISION {
	SYS_BOARD_POC = 0x0,
	SYS_BOARD_EVT,
	SYS_BOARD_EVT2,
	SYS_BOARD_EVT3_HOTSWAP,
	SYS_BOARD_EVT3_EFUSE,
	SYS_BOARD_DVT_HOTSWAP,
	SYS_BOARD_DVT_EFUSE,
	SYS_BOARD_MP_HOTSWAP,
	SYS_BOARD_MP_EFUSE,
};

enum {
	type_2ou_exp = 0x1,
	type_2ou_spe = 0x2,
	type_2ou_dpv2 = 0x77,
	type_2ou_dpv2_8 = 0x7,
	type_2ou_dpv2_16 = 0x70,
};

#endif
