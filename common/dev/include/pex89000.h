#ifndef PEX_89000_H
#define PEX_89000_H

#include <stdint.h>

typedef enum pex_dev { pex_dev_atlas1, pex_dev_atlas2, pex_dev_unknown } pex_dev_t;

typedef enum pex_access {
	pex_access_temp,
	pex_access_adc,
	pex_access_id,
	pex_access_rev_id,
	pex_access_sbr_ver,
	pex_access_flash_ver,
	pex_access_unknown
} pex_access_t;

enum pex_api_ret {
	pex_api_success,
	pex_api_unspecific_err,
	pex_api_mutex_err,
};

/* sensor offset */
enum pex_sensor_offset {
	PEX_TEMP,
	PEX_ADC,
};

typedef struct {
	uint8_t idx; // Create index based on init variable
	struct k_mutex mutex;
	pex_dev_t pex_type;
	sys_snode_t node; // linked list node
} pex89000_unit;

/* Note: Could be used only after pex89000 sensor init successed */
uint8_t pex_access_engine(uint8_t bus, uint8_t addr, uint8_t idx, pex_access_t key, uint32_t *resp);

#endif
