#ifndef _GPIO_ASPEED_H_
#define _GPIO_ASPEED_H_
#include "hal_def.h"
#include "objects.h"

/**
 * GPIO register fields
 */
typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t data[4];
	} fields;
} gpio_data_value_register_t; /* 00000000 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t direction[4];
	} fields;
} gpio_direction_register_t; /* 00000004 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t int_en[4];
	} fields;
} gpio_int_en_register_t; /* 00000008 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t int_sens_type[4];
	} fields;
} gpio_int_sens_type_register_t; /* 0000000c */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t int_status[4];
	} fields;
} gpio_int_status_register_t; /* 00000018 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t wdt_reset_tolerant_en[4];
	} fields;
} gpio_wdt_reset_tolerant_register_t; /* 0000001c */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t debounce_sel[4];
	} fields;
} gpio_debounce_sel_register_t; /* 00000040 */


typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t debounce_time:24; /*[0-23]*/ 
		volatile uint32_t :8; /*[24-31]*/ 

	} fields;
} debounce_time_register_t; /* 00000050 */


typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t cmd_src[4];
	} fields;
} gpio_cmd_src_t; /* 00000060 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t input_mask[4];
	} fields;
} gpio_input_mask_register_t; /* 000000b8 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t rd_data[4];
	} fields;
} gpio_read_data_register_t; /* 000000c0 */

typedef union
{
    volatile uint8_t byte;
    struct
    {
        volatile unsigned other_permit:1;
        volatile unsigned mst1_permit:1;
        volatile unsigned mst2_permit:1;
        volatile unsigned mst3_permit:1;
        volatile unsigned mst4_permit:1;
        volatile unsigned mst5_permit:1;
        volatile unsigned mode_sel:1;
        volatile unsigned lock:1;
    } bits;
} gpio_set_permit_t;

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t sets[4]; /*[0-7]*/ 
	} fields;
} gpio_new_cmd_src_t; 

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t mst1:5; /*[0-4]*/ 
		volatile uint32_t mst2:5; /*[5-9]*/ 
		volatile uint32_t mst3:5; /*[10-14]*/ 
		volatile uint32_t mst4:5; /*[15-19]*/ 
		volatile uint32_t mst5:5; /*[20-24]*/ 
		volatile uint32_t :6; /*[25-30]*/ 
		volatile uint32_t lock:1; /*[31]*/ 
	} fields;
} gpio_cmd_src_sel_t; 

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t index_number:8; /*[0-7]*/ 
		volatile uint32_t :4; /*[8-11]*/ 
		volatile uint32_t index_command:1; /*[12-12]*/ 
		volatile uint32_t :3; /*[13-15]*/ 
		volatile uint32_t index_type:4; /*[16-19]*/ 
		volatile uint32_t index_data:5; /*[20-24]*/ 
		volatile uint32_t :7; /*[25-31]*/ 
	} fields;
} gpio_index_register_t; /* 000002ac */

/**
 * READ 1/2/4 bytes, WRITE 4 bytes only
 */
typedef struct{
	gpio_data_value_register_t          group0_data; /* 00000000 */
	gpio_direction_register_t           group0_dir; /* 00000004 */
	gpio_int_en_register_t              group0_int_en; /* 00000008 */
	gpio_int_sens_type_register_t       group0_int_sens_type[3]; /* 0000000c~00000014 */
	gpio_int_status_register_t          group0_int_status; /* 00000018 */
	gpio_wdt_reset_tolerant_register_t  group0_rst_tolerant; /* 0000001c */
	gpio_data_value_register_t          group1_data; /* 00000020 */
	gpio_direction_register_t           group1_dir; /* 00000024 */
	gpio_int_en_register_t              group1_int_en; /* 00000028 */
	gpio_int_sens_type_register_t       group1_int_sens_type[3]; /* 0000002c~00000034 */
	gpio_int_status_register_t          group1_int_status; /* 00000038 */
	gpio_wdt_reset_tolerant_register_t  group1_rst_tolerant; /* 0000003c */
	gpio_debounce_sel_register_t        group0_debounce_sel[2]; /* 00000040~00000044 */
	gpio_debounce_sel_register_t        group1_debounce_sel[2]; /* 00000048~0000004c */
	debounce_time_register_t            debounce_time[3]; /* 00000050~00000058 */
	uint32_t reserved0[1]; /* 0000005c~0000005c*/
	gpio_cmd_src_t                      group0_cmd_src[2]; /* 00000060~00000064*/
	gpio_cmd_src_t                      group1_cmd_src[2]; /* 00000068~0000006c*/
	gpio_data_value_register_t          group2_data; /* 00000070 */
	gpio_direction_register_t           group2_dir; /* 00000074 */
	gpio_data_value_register_t          group3_data; /* 00000078 */
	gpio_direction_register_t           group3_dir; /* 0000007c */
	gpio_data_value_register_t          group4_data; /* 00000080 */
	gpio_direction_register_t           group4_dir; /* 00000084 */
	gpio_data_value_register_t          group5_data; /* 00000088 */
	gpio_direction_register_t           group5_dir; /* 0000008c */
	gpio_cmd_src_t                      group2_cmd_src[2]; /* 00000090~00000094*/
	gpio_int_en_register_t              group2_int_en; /* 00000098 */
	gpio_int_sens_type_register_t       group2_int_sens_type[3]; /* 0000009c~000000a4 */
	gpio_int_status_register_t          group2_int_status; /* 000000a8 */
	gpio_wdt_reset_tolerant_register_t  group2_rst_tolerant; /* 000000ac */
	gpio_debounce_sel_register_t        group2_debounce_sel[2]; /* 000000b0~000000b4 */
	gpio_input_mask_register_t          group2_input_mask; /* 000000b8 */
	uint32_t reserved1[1]; /* 000000bc~000000bc*/
	gpio_read_data_register_t           group0_rd_data; /* 000000c0 */
	gpio_read_data_register_t           group1_rd_data; /* 000000c4 */
	gpio_read_data_register_t           group2_rd_data; /* 000000c8 */
	gpio_read_data_register_t           group3_rd_data; /* 000000cc */
	gpio_read_data_register_t           group4_rd_data; /* 000000d0 */
	gpio_read_data_register_t           group5_rd_data; /* 000000d4 */
	gpio_read_data_register_t           group6_rd_data; /* 000000d8 */
	uint32_t reserved2[1]; /* 000000dc~000000dc*/
	gpio_cmd_src_t                      group3_cmd_src[2]; /* 000000e0~000000e4*/
    gpio_int_en_register_t              group3_int_en; /* 000000e8 */
	gpio_int_sens_type_register_t       group3_int_sens_type[3]; /* 000000ec~000000f4 */
	gpio_int_status_register_t          group3_int_status; /* 000000f8 */
	gpio_wdt_reset_tolerant_register_t  group3_rst_tolerant; /* 000000fc */
	gpio_debounce_sel_register_t        group3_debounce_sel[2]; /* 00000100~00000104 */
	gpio_input_mask_register_t          group3_input_mask; /* 00000108 */
	uint32_t reserved3[1]; /* 0000010c~0000010c*/
	gpio_cmd_src_t                      group4_cmd_src[2]; /* 00000110~00000114*/
    gpio_int_en_register_t              group4_int_en; /* 00000118 */
	gpio_int_sens_type_register_t       group4_int_sens_type[3]; /* 0000011c~00000124 */
	gpio_int_status_register_t          group4_int_status; /* 00000128 */
	gpio_wdt_reset_tolerant_register_t  group4_rst_tolerant; /* 0000012c */
	gpio_debounce_sel_register_t        group4_debounce_sel[2]; /* 00000130~00000134 */
	gpio_input_mask_register_t          group4_input_mask; /* 00000138 */
	uint32_t reserved4[1]; /* 0000013c~0000013c*/
	gpio_cmd_src_t                      group5_cmd_src[2]; /* 00000140~00000144*/
    gpio_int_en_register_t              group5_int_en; /* 00000148 */
	gpio_int_sens_type_register_t       group5_int_sens_type[3]; /* 0000014c~00000154 */
	gpio_int_status_register_t          group5_int_status; /* 00000158 */
	gpio_wdt_reset_tolerant_register_t  group5_rst_tolerant; /* 0000015c */
	gpio_debounce_sel_register_t        group5_debounce_sel[2]; /* 00000160~00000164 */
	gpio_input_mask_register_t          group5_input_mask; /* 00000168 */
	uint32_t reserved5[1]; /* 0000016c~0000016c*/
	gpio_cmd_src_t                      group6_cmd_src[2]; /* 00000170~00000174*/
    gpio_int_en_register_t              group6_int_en; /* 00000178 */
	gpio_int_sens_type_register_t       group6_int_sens_type[3]; /* 0000017c~00000184 */
	gpio_int_status_register_t          group6_int_status; /* 00000188 */
	gpio_wdt_reset_tolerant_register_t  group6_rst_tolerant; /* 0000018c */
	gpio_debounce_sel_register_t        group6_debounce_sel[2]; /* 00000190~00000194 */
	gpio_input_mask_register_t          group6_input_mask; /* 00000198 */
	uint32_t reserved6[13]; /* 0000019c~000001cc*/
	gpio_input_mask_register_t          group0_input_mask; /* 000001d0 */
	gpio_input_mask_register_t          group1_input_mask; /* 000001d4 */
	uint32_t reserved7[2]; /* 000001d8~000001dc*/
	gpio_data_value_register_t          group6_data; /* 000001e0 */
	gpio_direction_register_t           group6_dir; /* 000001e4 */
	uint32_t reserved8[17]; /* 000001e8~00000228*/
	gpio_new_cmd_src_t 					group0_write_cmd_src; /* 0000022c */
	gpio_new_cmd_src_t 					group0_read_cmd_src; /* 00000230 */
	uint32_t reserved9[18]; /* 00000234~00000278*/
	gpio_new_cmd_src_t 					group1_write_cmd_src; /* 0000027c */
	gpio_new_cmd_src_t 					group1_read_cmd_src; /* 00000280 */
	uint32_t reserved10[10]; /* 00000284~000002a8*/
	gpio_index_register_t 				index; /* 000002ac */
	uint32_t reserved11[8]; /* 000002b0~000002cc*/
	gpio_cmd_src_sel_t 					cmd_src_sel; /* 000002d0 */
	uint32_t reserved12[22]; /* 000002d4~00000328*/
	gpio_new_cmd_src_t 					group2_write_cmd_src; /* 0000032c */
	gpio_new_cmd_src_t 					group2_read_cmd_src; /* 00000330 */
	uint32_t reserved13[18]; /* 00000334~00000378*/
	gpio_new_cmd_src_t 					group3_write_cmd_src; /* 0000037c */
	gpio_new_cmd_src_t 					group3_read_cmd_src; /* 00000380 */
	uint32_t reserved14[18]; /* 00000384~000003c8*/
	gpio_new_cmd_src_t 					group4_write_cmd_src; /* 000003cc */
	gpio_new_cmd_src_t 					group4_read_cmd_src; /* 000003d0 */
	uint32_t reserved15[18]; /* 000003d4~00000418*/
	gpio_new_cmd_src_t 					group5_write_cmd_src; /* 0000041c */
	gpio_new_cmd_src_t 					group5_read_cmd_src; /* 00000420 */
	uint32_t reserved16[18]; /* 00000424~00000468*/
	gpio_new_cmd_src_t 					group6_write_cmd_src; /* 0000046c */
	gpio_new_cmd_src_t 					group6_read_cmd_src; /* 00000470 */
} gpio_register_t;


typedef enum {
	GPIO_DATA = 0,
	GPIO_DIRECTION,
	GPIO_INTERRUPT,
	GPIO_DEBOUCE,
	GPIO_TOLERANCE,
	GPIO_CMD_SRC,
	GPIO_INPUT_MASK,
	GPIO_RESERVED,
	GPIO_NEW_W_CMD_SRC,
	GPIO_NEW_R_CMD_SRC,
} aspeed_gpio_index_type_t;

typedef enum {
	GPIO_INPUT = 0,
	GPIO_OUTPUT,
} aspeed_gpio_dir_type_t;

typedef enum {
	GPIO_INDEX_WRITE = 0,
	GPIO_INDEX_READ,
} aspeed_gpio_index_cmd_t;

typedef enum {
	GPIO_CMD_SRC_ARM = 0,
	GPIO_CMD_SRC_MST2,
	GPIO_CMD_SRC_MST3,
	GPIO_CMD_SRC_RESEVED,
} aspeed_gpio_cmd_src_t;

typedef enum {
	GPIO_SEL_PRI = 1,
	GPIO_SEL_SSP = 6,
	GPIO_SEL_LPC = 19,
} aspeed_gpio_cmd_src_sel_t;

typedef enum {
	GPIO_FALLING_EDGE = 0,
	GPIO_RAISING_EDGE,
	GPIO_LEVEL_LOW,
	GPIO_LEVEL_HIGH,
	GPIO_DUAL_EDGE,
} aspeed_gpio_int_type_t;

typedef void (*gpio_cb_t)(void *, uint32_t gpio_num);

typedef struct aspeed_gpio_adapter_s{
    gpio_cb_t cb;
    void *para;
	uint8_t int_type;
	osEventFlagsId_t evt_id;
} aspeed_gpio_adapter_t;


typedef struct aspeed_gpio_priv_s {
	uint32_t cmd_source;
	uint32_t gpio_num;
	uint32_t irq;
	aspeed_gpio_adapter_t *adapter;
} aspeed_gpio_priv_t;

/*
 * idx: Select gpio controller index: 0 or 1
 * base: gpio base address
 * irq_n: IRQ number
 * gpio_number: The number of gpio
 * g_dedicated: dedicated gpio group for minibmc, [31:0] = group 0 ~ 31
 */
#define GPIO_DEVICE_DECLARE(idx, base, irq_n, gpio_number, g_dedicated)      \
	aspeed_gpio_priv_t gpio##idx##_priv = {                                \
		.cmd_source = g_dedicated,                                             \
		.irq = irq_n,                                                          \
		.gpio_num = gpio_number,                                               \
	};                                                                         \
	DECLARE_DEV_CLK(gpio##idx, 0, 0, 0);                                       \
	DECLARE_DEV_RESET(gpio##idx, 0, 0, 0);                                     \
	DECLARE_DEV(gpio##idx, ASPEED_DEV_GPIO##idx, base, &gpio##idx##_priv);


hal_status_t aspeed_gpio_set(gpio_t *obj, int gpio_number, int val);
int aspeed_gpio_get(gpio_t *obj, int gpio_number);
hal_status_t aspeed_gpio_set_direction(gpio_t *obj, int gpio_number,
									   int direct);
hal_status_t aspeed_gpio_int_cb_hook(gpio_t *obj, int gpio_number,
									 uint8_t int_type,
									 gpio_cb_t cb, void *para);
hal_status_t aspeed_gpio_int_cb_unhook(gpio_t *obj, int gpio_number);
hal_status_t aspeed_gpio_cmd_src_set(gpio_t *obj, int gpio_number,
									 uint8_t cmd_src);
hal_status_t aspeed_gpio_init(gpio_t *obj);

#endif /* end of "#ifndef _GPIO_ASPEED_H_" */