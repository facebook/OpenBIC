#ifndef _SGPIOM_ASPEED_H_
#define _SGPIOM_ASPEED_H_

/**
 * SGPIOM register fields
 */
typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t data[4];
	} fields;
} sgpiom_data_value_register_t; /* 00000000 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t int_en[4];
	} fields;
} sgpiom_int_en_register_t; /* 00000004 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t int_sens_type[4];
	} fields;
} sgpiom_int_sens_type_register_t; /* 00000008~00000010 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t int_status[4];
	} fields;
} sgpiom_int_status_register_t; /* 00000014 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t wdt_reset_tolerant_en[4];
	} fields;
} sgpiom_wdt_reset_tolerant_register_t; /* 00000018 */


typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t input_mask[4];
	} fields;
} sgpiom_input_mask_register_t; /* 000000b8 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint8_t rd_data[4];
	} fields;
} sgpiom_read_data_register_t; /* 000000c0 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t enable:1; /*[0-0]*/ 
		volatile uint32_t :5; /*[1-5]*/ 
		volatile uint32_t numbers:5; /*[6-10]*/ 
		volatile uint32_t :5; /*[10-15]*/ 
		volatile uint32_t division:16; /*[16-31]*/ 

	} fields;
} sgpiom_config_register_t;

typedef struct
{
    sgpiom_data_value_register_t          data; /* 00000000 */
	sgpiom_int_en_register_t              int_en; /* 00000004 */
	sgpiom_int_sens_type_register_t       int_sens_type[3]; /* 00000008~00000010 */
	sgpiom_int_status_register_t          int_status; /* 00000014 */
	sgpiom_wdt_reset_tolerant_register_t  rst_tolerant; /* 00000018 */
} sgpiom_gather_register_t;

typedef struct {
	sgpiom_gather_register_t 			  	gather[3]; /* 00000000~00000050 */
	sgpiom_config_register_t 			  	config; /* 00000054 */
	sgpiom_input_mask_register_t          	input_mask[4]; /* 00000058~00000064 */
	uint32_t 								reserved0[2]; /* 00000068~0000006c*/
	sgpiom_read_data_register_t           	rd_data[4]; /* 00000070~0000007c */
	uint32_t 								reserved1[4]; /* 00000080~0000008c*/
	sgpiom_gather_register_t          		gather4; /* 00000090~000000a8 */
	uint32_t 								reserved2[21]; /* 000000ac~000000fc*/
} sgpiom_register_t;

typedef void (*gpio_cb_t)(void *, uint32_t gpio_num);

typedef struct aspeed_sgpiom_adapter_s{
    gpio_cb_t cb;
    void *para;
	uint8_t int_type;
	osEventFlagsId_t evt_id;
} aspeed_sgpiom_adapter_t;


typedef struct aspeed_sgpiom_priv_s {
	uint32_t target_clock;
	uint32_t gpio_num;
	uint32_t irq;
	aspeed_sgpiom_adapter_t *adapter;
} aspeed_sgpiom_priv_t;

/*
 * idx: Select sgpio master index: 0 or 1
 * clock: target clock of sgpio
 * irq_n: IRQ number
 * gpio_number: The number of gpio
 */
#define SGPIOM_DEVICE_DECLARE(idx, clock, gpio_number)                         \
	aspeed_sgpiom_priv_t sgpiom##idx##_priv = {                                \
		.target_clock = clock,                                                 \
		.irq = Sgpiom##idx##_IRQn,                                             \
		.gpio_num = gpio_number,                                               \
	};                                                                         \
	DECLARE_DEV_CLK(sgpiom##idx, 0, 0, 0);                                     \
	DECLARE_DEV_RESET(sgpiom##idx, 0, 0, 0);                                   \
	DECLARE_DEV(sgpiom##idx, ASPEED_DEV_SGPIOM##idx, SGPIOM##idx##_BASE,       \
				&sgpiom##idx##_priv);

hal_status_t aspeed_sgpiom_init(gpio_t *obj);

#endif /* end of "#ifndef _SGPIOM_ASPEED_H_" */