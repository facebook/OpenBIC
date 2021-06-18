
#ifndef __FMC_SPI_ASPEED__
#define __FMC_SPI_ASPEED__

#include "common.h"
#include "objects.h"

#define MAX_CS                    3

#define CTRL_CMD_MODE_NORMAL      0
#define CTRL_CMD_MODE_FREAD       1
#define CTRL_CMD_MODE_WRITE       2
#define CTRL_CMD_MODE_USER        3

#define CTRL_IO_SINGLE_DATA       0
#define CTRL_IO_DUAL_DATA         2
#define CTRL_IO_QUAD_DATA         4

/* used for HW SAFS setting */
#define OFFSET_HOST_DIRECT_ACCESS_CMD_CTRL4	0x6c
#define OFFSET_HOST_DIRECT_ACCESS_CMD_CTRL2	0x74

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t ce_flash_type: 6; /* [0 - 5] */
		volatile uint32_t : 10; /* [6 - 15] */
		volatile uint32_t ce_write_type: 3; /* [16 - 18] */
		volatile uint32_t : 13; /* [19 - 31] */

	} fields;
} spi_ce_type_setting_reg_t; /* 00000000 */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t addr_mode: 3; /* [0 - 2] */
		volatile uint32_t : 1; /* [3 - 3] */
		volatile uint32_t ce_4b_auto_read_cmd: 3; /* [4 - 6] */
		volatile uint32_t : 1; /* [7 - 7] */
		volatile uint32_t ce_min_inactive_timing: 6; /* [8 - 13] */
		volatile uint32_t :18; /* [14 - 31] */
	} fields;
} spi_addr_mode_ctrl_reg_t; /* 00000004 */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t : 1; /* [0 - 0] */
		volatile uint32_t write_addr_protect_intr_enable: 1; /* [1 - 1] */
		volatile uint32_t spi_cmd_abort_intr_enable: 1; /* [2 - 2] */
		volatile uint32_t dma_intr_enable: 1; /* [3 - 3] */
		volatile uint32_t dma_buf_mode_fifo_intr_enable: 1; /* [4 - 4] */
		volatile uint32_t : 4; /* [5 - 8] */
		volatile uint32_t write_addr_protect_status: 1; /* [9 - 9] */
		volatile uint32_t spi_cmd_abort_status: 1; /* [10 - 10] */
		volatile uint32_t dma_status: 1; /* [11 - 11] */
		volatile uint32_t dma_buf_mode_fifo_empty_status: 1; /* [12 - 12] */
		volatile uint32_t dma_buf_mode_fifo_full_status: 1; /* [13 - 13] */
		volatile uint32_t : 18; /* [14 - 31] */
	} fields;
} spi_intr_ctrl_status_reg_t; /* 00000008 */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t data_byte_disable:4; /* [0 - 3] */
		volatile uint32_t addr_byte_disable:4; /* [4 - 7] */
		volatile uint32_t : 24; /* [8 - 31] */
	} fields;
} spi_cmd_ctrl_reg_t; /* 0000000c */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t spi_cmd_mode: 2; /* [0 - 1] */
		volatile uint32_t cs_inactive_ctrl: 1; /* [2 - 2] */
		volatile uint32_t : 1; /* [3 - 3] */
		volatile uint32_t : 1; /* [4 - 4] */
		volatile uint32_t endian_ctrl: 1; /* [5 - 5] */
		volatile uint32_t spi_read_dummy_cycles_low_bits: 2; /* [6 - 7] */
		volatile uint32_t spi_clk_div: 4; /* [8 - 11] */
		volatile uint32_t spi_rw_cmd_merge_disable: 1; /* [12 - 12] */
		volatile uint32_t : 1; /* [13 - 13] */
		volatile uint32_t spi_read_dummy_cycles_high_bits: 1; /* [14 - 14] */
		volatile uint32_t spi_write_dummy_cycle_cmd: 1; /* [15 - 15] */
		volatile uint32_t spi_cmd: 8; /* [16 - 23] */
		volatile uint32_t spi_base_clk: 4; /* [24 - 27] */
		volatile uint32_t io_mode: 4; /* [28 - 31] */
	} fields;
} spi_ce_ctrl_reg_t; /* 00000010 */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t ce_start_addr: 16; /* [0 - 15] */
		volatile uint32_t ce_end_addr: 16; /* [16 - 31] */
	} fields;
} spi_ce_addr_decoding_reg_t; /* 00000030 */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t gen_soft_reset_cmd: 1; /* [0 - 0] */
		volatile uint32_t wait_spi_wip_idle_enable: 1; /* [1 - 1] */
		volatile uint32_t : 6; /* [2 - 7] */
		volatile uint32_t latest_spi_status_reg: 8; /* [8 - 15] */
		volatile uint32_t : 16; /* [16 - 31] */
	} fields;
} spi_auto_soft_rst_ctrl_reg_t; /* 00000050 */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t spi_dummy_cycle_output_data: 8; /* [0 - 7] */
		volatile uint32_t : 24; /* [8 - 31] */
	} fields;
} spi_dummy_cycle_data_reg_t; /* 00000054 */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t enable_watchdog: 1; /* [0 - 0] */
		volatile uint32_t : 7; /* [1 - 7] */
		volatile uint32_t watchdog_active_event_counter: 4; /* [8 - 11] */
		volatile uint32_t : 20; /* [12 - 31] */
	} fields;
} fmc_wdt1_ctrl_reg_t; /* 00000060 */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t enable_watchdog: 1; /* [0 - 0] */
		volatile uint32_t : 3; /* [1 - 3] */
		volatile uint32_t boot_flash_indicator: 1; /* [4 - 4] */
		volatile uint32_t single_chip_boot_mode_indicator: 1; /* [5 - 5] */
		volatile uint32_t abr_mode: 1; /* [6 - 6] */
		volatile uint32_t : 1; /* [7 - 7] */
		volatile uint32_t watchdog_active_event_counter: 8; /* [8 - 15] */
		volatile uint32_t : 16; /* [16 - 31] */
	} fields;
} fmc_wdt2_ctrl_reg_t; /* 00000064 */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t reload_val: 13; /* [0 - 12] */
		volatile uint32_t : 3; /* [13 - 15] */
		volatile uint32_t counter_val: 16; /* [16 - 31] */

	} fields;
} fmc_wdt2_timer_reload_reg_t; /* 00000068 */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t restart_magic: 16; /* [0 - 15] */
		volatile uint32_t : 16; /* [16 - 31] */

	} fields;
} fmc_wdt2_timer_restart_reg_t; /* 0000006c */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t fifo_len_status: 7; /* [0 - 6] */
	} fields;
} spi_dma_buf_mode_len_reg_t; /* 0000007c */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t dma_enable: 1; /* [0 - 0] */
		volatile uint32_t dma_dir: 1; /* [1 - 1] */
		volatile uint32_t checksum_only: 1; /* [2 - 2] */
		volatile uint32_t calibration_mode: 1; /* [3 - 3] */
		volatile uint32_t dma_buffer_mode: 1; /* [4 - 4] */
		volatile uint32_t : 3; /* [5 - 7] */
		volatile uint32_t spi_timing_compensation: 8; /* [8 - 15] */
		volatile uint32_t spi_clk_div: 4; /* [16 - 19] */
		volatile uint32_t : 12; /* [20 - 31] */
	} fields;
} spi_dma_ctrl_status_reg_t; /* 00000080 */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t : 2; /* [0 - 1] */
		volatile uint32_t start_addr: 26; /* [2 - 27] */
		volatile uint32_t :4; /* [28 - 31] */
	} fields;
} spi_dma_flash_addr_reg_t; /* 00000084 */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t : 2; /*[0 - 1]*/
		volatile uint32_t start_addr: 30; /* [2 - 31] */
	} fields;
} spi_dma_ram_addr_reg_t; /* 00000088 */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t dma_len: 25; /* [0 - 24] */
		volatile uint32_t : 7; /* [25 - 31] */
	} fields;
} spi_dma_len_reg_t; /* 0000008c */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t data: 32; /* [0 - 31] */
	} fields;
} spi_dma_checksum_calculate_reg_t; /* 00000090 */

typedef union {
	volatile uint32_t val;
	struct {
		volatile uint32_t hclk_2: 8; /* [0 - 7] */
		volatile uint32_t hclk_3: 8; /* [8 - 15] */
		volatile uint32_t hclk_4: 8; /* [16 - 23] */
		volatile uint32_t hclk_5: 8; /* [24 - 31] */
	} fields;
} spi_ce_timing_compensation_reg_t; /* 00000094 */

typedef struct {
	spi_ce_type_setting_reg_t spi_ce_type_setting_reg; /* 00000000 */
	spi_addr_mode_ctrl_reg_t spi_addr_mode_ctrl_reg; /* 00000004 */
	spi_intr_ctrl_status_reg_t spi_intr_ctrl_status_reg; /* 00000008 */
	spi_cmd_ctrl_reg_t spi_cmd_ctrl_reg; /* 0000000c */
	spi_ce_ctrl_reg_t spi_ce_ctrl_reg[3]; /* 00000010 - 00000018*/
	uint32_t rsv00[5]; /* 0000001c ~ 0000002c*/
	spi_ce_addr_decoding_reg_t spi_ce_addr_decoding_reg[3]; /* 00000030 */
	uint32_t rsv01[5]; /* 0000003c ~ 0000004c*/
	spi_auto_soft_rst_ctrl_reg_t spi_auto_soft_rst_ctrl_reg; /* 00000050 */
	spi_dummy_cycle_data_reg_t spi_dummy_cycle_data_reg; /* 00000054 */
	uint32_t rsv02[2]; /* 00000058 ~ 0000005c*/
	fmc_wdt1_ctrl_reg_t fmc_wdt1_ctrl_reg; /* 00000060 */
	fmc_wdt2_ctrl_reg_t fmc_wdt2_ctrl_reg; /* 00000064 */
	fmc_wdt2_timer_reload_reg_t fmc_wdt2_timer_reload_reg; /* 00000068 */
	fmc_wdt2_timer_restart_reg_t fmc_wdt2_timer_restart_reg; /* 0000006c */
	uint32_t rsv03[3]; /* 00000070 ~ 00000078*/
	spi_dma_buf_mode_len_reg_t spi_dma_buf_mode_len_reg; /* 0000007c */
	spi_dma_ctrl_status_reg_t spi_dma_ctrl_status_reg; /* 00000080 */
	spi_dma_flash_addr_reg_t spi_dma_flash_addr_reg; /* 00000084 */
	spi_dma_ram_addr_reg_t spi_dma_ram_addr_reg; /* 00000088 */
	spi_dma_len_reg_t spi_dma_len_reg; /* 0000008c */
	spi_dma_checksum_calculate_reg_t spi_dma_checksum_calculate_reg; /* 00000090 */
	spi_ce_timing_compensation_reg_t spi_ce_timing_compensation_reg[3]; /* 00000094 - 0000009C */
} fmc_spi_ctrl_t;

typedef struct chip_s {
	uint32_t ahb_start_addr;
	uint32_t max_freq;
	uint32_t tx_bus_width;
	uint32_t rx_bus_width;
	uint32_t cs;
	bool enable;
	/* if the slave component is spi-flash device */
	bool flash_component;
	bool flash_reinit_ctrl;
	bool probed;
	uint32_t ctrl_base_setting;
	void *ctx;
} chip_t;

typedef struct {
	char *name;
	uint32_t ctrl_base;
	uint32_t ahb_base;
	uint32_t max_cs;
	chip_t chipes[MAX_CS];
	uint32_t (*segment_start)(uint32_t val);
	uint32_t (*segment_end)(uint32_t val);
	uint32_t (*segment_value)(uint32_t start, uint32_t end);
} fmc_spi_priv_t;

#endif
