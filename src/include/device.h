#ifndef _DEVICE_H_
#define _DEVICE_H_

#include <stdint.h>
#include "util.h"
#include "device_id.h"
#include "memory_map.h"

struct aspeed_reset_s {
	uint32_t reg_assert;
	uint32_t reg_deassert;
	uint32_t bits;
};

struct aspeed_clk_s {
	uint32_t reg_enable;
	uint32_t reg_disable;
	uint32_t bits;
};

typedef struct aspeed_device_s {
	uint32_t dev_id;
	uint32_t base;
	struct aspeed_reset_s *reset;
	struct aspeed_clk_s *clk;
	int init;
	void *private;
} aspeed_device_t;

#define DECLARE_DEV_CLK(_name, _reg_en, _reg_dis, _bits)                       \
	struct aspeed_clk_s CONCAT(_name, _clk) = {                                \
		.reg_enable = _reg_en,                                                 \
		.reg_disable = _reg_dis,                                               \
		.bits = _bits,                                                         \
	}

#define DECLARE_DEV_RESET(_name, _reg_assert, _reg_deassert, _bits)            \
	struct aspeed_reset_s CONCAT(_name, _reset) = {                            \
		.reg_assert =  _reg_assert,                                            \
		.reg_deassert = _reg_deassert,                                         \
		.bits = _bits,                                                         \
	}

#define DECLARE_DEV(_name, _id, _base, _priv_data)                             \
	struct aspeed_device_s _name = {                                           \
		.dev_id = _id,                                                         \
		.base = _base,                                                         \
		.reset = &CONCAT(_name, _reset),                                       \
		.clk = &CONCAT(_name, _clk),                                           \
		.init = 0,                                                             \
		.private = _priv_data,                                                 \
	}
#endif /* end of "ifndef _DEVICE_H_" */