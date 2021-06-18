/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _JTAG_ASPEED_H_
#define _JTAG_ASPEED_H_
#include "hal_def.h"
#include "objects.h"

/**********************************************************
 * Jtag register fields
 *********************************************************/
typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t data:32; /*[0-31]*/ 

	} fields;
} data_port_register_t; /* 00000000-00000004 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t dr_xfer_en:1; /*[0-0]*/ 
		volatile uint32_t ir_xfer_en:1; /*[1-1]*/ 
        volatile uint32_t :2; /*[2-3]*/ 
		volatile uint32_t last_xfer:1; /*[4-4]*/ 
		volatile uint32_t terminating_xfer:1; /*[5-5]*/ 
		volatile uint32_t msb_first:1; /*[6-6]*/ 
		volatile uint32_t :1; /*[7-7]*/ 
		volatile uint32_t xfer_len:10; /*[8-17]*/ 
		volatile uint32_t :2; /*[18-19]*/ 
		volatile uint32_t internal_fifo_mode:1; /*[20-20]*/ 
		volatile uint32_t reset_internal_fifo:1; /*[21-21]*/ 
		volatile uint32_t :7; /*[22-28]*/ 
		volatile uint32_t reset_to_tlr:1; /*[29-29]*/ 
		volatile uint32_t engine_output_enable:1; /*[30-30]*/ 
		volatile uint32_t engine_enable:1; /*[31-31]*/ 

	} fields;
} mode_1_control_t; /* 00000008 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t enable_of_data_xfer_completed:1; /*[0-0]*/ 
		volatile uint32_t enable_of_data_xfer_pause:1; /*[1-1]*/ 
		volatile uint32_t enable_of_instr_xfer_completed:1; /*[2-2]*/ 
		volatile uint32_t enable_of_instr_xfer_pause:1; /*[3-3]*/ 
		volatile uint32_t :12; /*[4-15]*/ 
		volatile uint32_t data_xfer_completed:1; /*[16-16]*/ 
		volatile uint32_t data_xfer_pause:1; /*[17-17]*/ 
		volatile uint32_t instr_xfer_completed:1; /*[18-18]*/ 
		volatile uint32_t instr_xfer_pause:1; /*[19-19]*/ 
		volatile uint32_t :12; /*[20-31]*/ 

	} fields;
} mode_1_int_ctrl_t; /* 0000000c */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t engine_idle:1; /*[0-0]*/ 
		volatile uint32_t data_xfer_pause:1; /*[1-1]*/ 
		volatile uint32_t instr_xfer_pause:1; /*[2-2]*/ 
		volatile uint32_t :13; /*[3-15]*/ 
		volatile uint32_t software_tdi_and_tdo:1; /*[16-16]*/ 
		volatile uint32_t software_tms:1; /*[17-17]*/ 
		volatile uint32_t software_tck:1; /*[18-18]*/ 
		volatile uint32_t software_mode_enable:1; /*[19-19]*/ 
		volatile uint32_t :12; /*[20-31]*/ 

	} fields;
} software_mode_and_status_t; /* 00000010 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t tck_divisor:11; /*[0-10]*/ 
		volatile uint32_t :20; /*[11-30]*/ 
		volatile uint32_t tck_inverse:1; /*[31-31]*/ 

	} fields;
} tck_control_t; /* 00000014 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t to_irt_state:1; /*[0-0]*/ 
		volatile uint32_t :30; /*[1-30]*/ 
		volatile uint32_t control_of_trstn:1; /*[31-31]*/ 

	} fields;
} engine_control_1_t; /* 00000018 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t :32; /*[0-31]*/ 

	} fields;
} reserved_t; /* 0000001c */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t pre_padding_number:9; /*[0-8]*/ 
		volatile uint32_t :3; /*[9-11]*/ 
		volatile uint32_t post_padding_number:9; /*[12-20]*/ 
		volatile uint32_t :3; /*[21-23]*/ 
		volatile uint32_t padding_data:1; /*[24-24]*/ 
		volatile uint32_t :7; /*[25-31]*/ 

	} fields;
} padding_control_0_t; /* 00000028 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t pre_padding_number:9; /*[0-8]*/ 
		volatile uint32_t :3; /*[9-11]*/ 
		volatile uint32_t post_padding_number:9; /*[12-20]*/ 
		volatile uint32_t :3; /*[21-23]*/ 
		volatile uint32_t padding_data:1; /*[24-24]*/ 
		volatile uint32_t :7; /*[25-31]*/ 

	} fields;
} padding_control_1_t; /* 0000002c */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t lower_data_shift_number:7; /*[0-6]*/ 
		volatile uint32_t start_of_shift:1; /*[7-7]*/ 
		volatile uint32_t end_of_shift:1; /*[8-8]*/ 
		volatile uint32_t paddign_selection:1; /*[9-9]*/ 
		volatile uint32_t pre_tms_shift_number:3; /*[10-12]*/ 
		volatile uint32_t post_tms_shift_number:3; /*[13-15]*/ 
		volatile uint32_t tms_value:14; /*[16-29]*/ 
		volatile uint32_t enable_static_shift:1; /*[30-30]*/ 
		volatile uint32_t enable_free_run_tck:1; /*[31-31]*/ 

	} fields;
} shift_control_t; /* 00000030 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t clock_divisor:12; /*[0-11]*/ 
		volatile uint32_t :3; /*[12-14]*/ 
		volatile uint32_t trst_value:1; /*[15-15]*/ 
		volatile uint32_t static_shift_value:1; /*[16-16]*/ 
		volatile uint32_t :3; /*[17-19]*/ 
		volatile uint32_t upper_data_shift_number:3; /*[20-22]*/ 
		volatile uint32_t :1; /*[23-23]*/ 
		volatile uint32_t internal_fifo_mode:1; /*[24-24]*/ 
		volatile uint32_t reset_internal_fifo:1; /*[25-25]*/ 
		volatile uint32_t :3; /*[26-28]*/ 
		volatile uint32_t reset_to_tlr:1; /*[29-29]*/ 
		volatile uint32_t engine_output_enable:1; /*[30-30]*/ 
		volatile uint32_t engine_enable:1; /*[31-31]*/ 

	} fields;
} mode_2_control_t; /* 00000034 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t shift_complete_interrupt_status:1; /*[0-0]*/ 
		volatile uint32_t :15; /*[1-15]*/ 
		volatile uint32_t shift_complete_interrupt_enable:1; /*[16-16]*/ 
		volatile uint32_t :15; /*[17-31]*/ 

	} fields;
} mode_2_int_ctrl_t; /* 00000038 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t engine_idle:1; /*[0-0]*/ 
		volatile uint32_t :31; /*[1-31]*/ 

	} fields;
} fsm_status_t; /* 0000003c */

typedef struct{
	data_port_register_t data_for_hw_mode_1[2]; /* 00000000, 00000004 */
	mode_1_control_t mode_1_control; /* 00000008 */
	mode_1_int_ctrl_t mode_1_int_ctrl; /* 0000000c */
	software_mode_and_status_t software_mode_and_status; /* 00000010 */
	tck_control_t tck_control; /* 00000014 */
	engine_control_1_t engine_control_1; /* 00000018 */
	uint32_t reserved0[1]; /* 0000001c~0000001c*/
	data_port_register_t data_for_hw_mode_2[2]; /* 00000020, 00000024 */
	padding_control_0_t padding_control_0; /* 00000028 */
	padding_control_1_t padding_control_1; /* 0000002c */
	shift_control_t shift_control; /* 00000030 */
	mode_2_control_t mode_2_control; /* 00000034 */
	mode_2_int_ctrl_t mode_2_int_ctrl; /* 00000038 */
	fsm_status_t fsm_status; /* 0000003c */
} jtag_register_t;

/**********************************************************
 * JTAG macro
 *********************************************************/
/**
 * Defines JTAG Test Access Port states.
 *
 * These definitions were gleaned from the ARM7TDMI-S Technical
 * Reference Manual and validated against several other ARM core
 * technical manuals.
 *
 * FIXME some interfaces require specific numbers be used, as they
 * are handed-off directly to their hardware implementations.
 * Fix those drivers to map as appropriate ... then pick some
 * sane set of numbers here (where 0/uninitialized == INVALID).
 */
typedef enum tap_state {
	TAP_INVALID = -1,
	/* Proper ARM recommended numbers */
	TAP_DREXIT2 = 0x0,
	TAP_DREXIT1 = 0x1,
	TAP_DRSHIFT = 0x2,
	TAP_DRPAUSE = 0x3,
	TAP_IRSELECT = 0x4,
	TAP_DRUPDATE = 0x5,
	TAP_DRCAPTURE = 0x6,
	TAP_DRSELECT = 0x7,
	TAP_IREXIT2 = 0x8,
	TAP_IREXIT1 = 0x9,
	TAP_IRSHIFT = 0xa,
	TAP_IRPAUSE = 0xb,
	TAP_IDLE = 0xc,
	TAP_IRUPDATE = 0xd,
	TAP_IRCAPTURE = 0xe,
	TAP_RESET = 0x0f,
} tap_state_t;

/**
 * This structure defines a single scan field in the scan. It provides
 * fields for the field's width and pointers to scan input and output
 * values.
 *
 * In addition, this structure includes a value and mask that is used by
 * jtag_add_dr_scan_check() to validate the value that was scanned out.
 */
typedef struct scan_field_s {
	/** The number of bits this field specifies */
	int num_bits;
	/** A pointer to value to be scanned into the device */
	const uint8_t *out_value;
	/** A pointer to a 32-bit memory location for data scanned out */
	uint8_t *in_value;

	/** The value used to check the data scanned out. */
	uint8_t *check_value;
	/** The mask to go with check_value */
	uint8_t *check_mask;
} scan_field_t;

/**
 * The scan_command provide a means of encapsulating a set of scan_field_s
 * structures that should be scanned in/out to the device.
 */
typedef struct scan_command_s {
	/** instruction/not data scan */
	bool ir_scan;
	/** number of fields in *fields array */
	int num_fields;
	/** pointer to an array of data scan fields */
	scan_field_t *fields;
	/** state in which JTAG commands should finish */
	tap_state_t end_state;
} scan_command_t;

typedef struct svf_xxr_para_s {
	int len;
	int data_mask;
	uint8_t *tdi;
	uint8_t *tdo;
	uint8_t *mask;
	uint8_t *smask;
} svf_xxr_para_t;

/**********************************************************
 * JTAG private data
 *********************************************************/
typedef struct aspeed_jtag_priv_s {
	uint8_t init;
	uint32_t fifo_length;
	uint32_t freq;
	tap_state_t state;
} aspeed_jtag_priv_t;

/**********************************************************
 * JTAG export api
 *********************************************************/
const char *tap_state_name(tap_state_t state);
tap_state_t tap_state_by_name(const char *name);
tap_state_t aspeed_jtag_get_tap_state(jtag_t *obj);
uint8_t aspeed_jtag_tap_is_stable(tap_state_t state);
hal_status_t aspeed_jtag_ir_scan(jtag_t *obj, int num_bits,
								 const uint8_t *out_value, uint8_t *in_value,
								 tap_state_t state);
hal_status_t aspeed_jtag_dr_scan(jtag_t *obj, int num_bits,
								 const uint8_t *out_value, uint8_t *in_value,
								 tap_state_t state);
hal_status_t aspeed_jtag_set_tap_state(jtag_t *obj, tap_state_t state);
void aspeed_jtag_run_test(jtag_t *obj, uint32_t num_cycles);
void aspeed_jtag_set_clk(jtag_t *obj, uint32_t freq);
hal_status_t aspeed_jtag_target_init(jtag_t *obj);
hal_status_t aspeed_jtag_init(jtag_t *obj);

#endif /* end of "#ifndef _JTAG_ASPEED_H_" */