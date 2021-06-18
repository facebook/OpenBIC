#ifndef _OBJECTS_H_
#define _OBJECTS_H_
#include "common.h"
#include "device.h"
#include "hal_def.h"
#include "cmsis_os.h"

/* Aspeed driver not used, just make compiler happy */
typedef uint32_t PinName;
typedef uint32_t PinMode;

typedef struct serial_cfg_s {
	uint32_t baud;
	uint8_t word_length;
	uint8_t parity;
	uint8_t stop_bit;
	uint8_t flow_ctrl;
} serial_cfg_t;

struct serial_s {
	aspeed_device_t *device;
	serial_cfg_t cfg;
	uint8_t init;
};

/* I2C */
#define CONFIG_I2C_MQUEUE_SLAVE

#define I2C_SLAVE_BUFF_SIZE 	256
#define I2C_SLAVE_MQ_FIFO_SIZE 	8


struct i2c_slave_mqueue {
	int			length;
	uint8_t	data[I2C_SLAVE_BUFF_SIZE];
};

typedef struct aspeed_i2c_priv_s {
	void *parent;
	uint32_t bus_clk;
	uint32_t irq;
	uint32_t buff_addr;	
} aspeed_i2c_priv_t;

typedef struct i2c_global_s {
	aspeed_device_t *device;
} i2c_global_t;

/* I3C */
typedef struct i3c_global_s {
	aspeed_device_t *device;
} i3c_global_t;

typedef struct i3c_slave_s {
	uint32_t static_addr;
	uint32_t dynamic_addr;
	uint32_t assign_dynamic_addr;
	uint32_t i2c_mode;
} i3c_slave_t;

typedef struct i3c_msg_s {
	uint32_t len;
	uint8_t *buf;
} i3c_msg_t;

enum dma_xfer_direction {
	DMA_DEV_TO_MEM,
	DMA_MEM_TO_DEV,
	DMA_XFER_NONE,
};
typedef struct i3cdma_s {
	aspeed_device_t *device;
	uint8_t in_used;
} i3cdma_t;

typedef struct i3cdma_desc_s {
	i3cdma_t *dma;			/* DMA engine that provides the service */
	aspeed_device_t *slave;	/* device that requests the DMA channel */
	enum dma_xfer_direction direction;
	uint32_t channel;
	uint32_t dst_addr;
	uint32_t src_addr;
	uint32_t nbytes;
} i3cdma_desc_t;

struct i3c_s {
	aspeed_device_t *device;
	i3c_global_t *global;
	uint32_t self_addr;
	i3c_slave_t *slaves;
	i3cdma_desc_t *tx_dma_desc;
	i3cdma_desc_t *rx_dma_desc;
	uint32_t n_slaves;
	uint32_t free_pos;
	uint16_t maxdevs;
	uint16_t datstartaddr;
	uint16_t i2c_mode;
	uint16_t bus_context;
	uint8_t addrs[32];
	uint8_t role;	/* 0: master, else: slave */
	uint8_t tx_dma_en;
	uint8_t rx_dma_en;
	struct {
		osMessageQId id;
		uint32_t ptr;
		uint32_t entries;		/* must be pow of 2 */
		i3c_msg_t *msgs;
	} mq;
};

typedef struct ipi_s {
	aspeed_device_t *device;
} ipi_t;

typedef struct espi_s {

	struct {
		void (*handler)(void *);
		void *arg;
	} ch_isr[4];

	struct {
		void (*handler)(void *);
		void *arg;
	} ch_reset_isr[4];

	aspeed_device_t *device;
} espi_t;

typedef struct peci_s {
	aspeed_device_t *device;
} peci_t;

typedef struct adc_s {
	aspeed_device_t *device;
} adc_t;

typedef struct pwm_s {
	aspeed_device_t *device;
} pwm_t;

typedef struct tach_s {
	aspeed_device_t *device;
} tach_t;

typedef struct jtag_s {
	aspeed_device_t *device;
} jtag_t;

typedef struct gpio_s gpio_t;
typedef void (*gpio_cb_t)(void *, uint32_t gpio_num);
/**
 * gpio_t - abstract a GPIO controller
 * @device: point to the gpio type device.
 * @set: assigns output value for signal "gpio_number".
 * @get: return value for signal "gpio_number"
 * @set_direction: configure the direction of signal "gpio_number", 0=input, 1=output.
 * @int_cb_hook: connect the interrupt of signal "gpio_number" for the callback function.
 * @int_cb_unhook: disconnect the interrupt of signal "gpio_number".
 * @info: Get the information of this gpio device.
 * @cmd_src_set: Set the master of the gpio pin to avoid the race condition.
 */
typedef struct gpio_s {
	aspeed_device_t *device;
	hal_status_t (*set)(gpio_t *obj, int gpio_number, int val);
	int (*get)(gpio_t *obj, int gpio_number);
	int (*get_direction)(gpio_t *obj, int gpio_number);
	hal_status_t (*set_direction)(gpio_t *obj, int gpio_number, int direct);
	hal_status_t (*int_cb_hook)(gpio_t *obj, int gpio_number, uint8_t int_type,
								gpio_cb_t cb, void *para);
	hal_status_t (*int_cb_unhook)(gpio_t *obj, int gpio_number);
	hal_status_t (*info)(gpio_t *obj);
	hal_status_t (*cmd_src_set)(gpio_t *obj, int gpio_number, uint8_t cmd_src);
	hal_status_t (*init)(gpio_t *obj);
} gpio_t;

typedef struct kcs_s {
	uint32_t idr;
	uint32_t odr;
	uint32_t str;

	uint8_t *ibuf;
	uint32_t ibuf_idx;
	uint32_t ibuf_sz;
	uint32_t ibuf_avail;

	uint8_t *obuf;
	uint32_t obuf_idx;
	uint32_t obuf_sz;
	uint32_t obuf_data_sz;

	uint32_t phase;
	uint32_t error;

#ifdef USE_OS_FLAG_FOR_WAIT
	osEventFlagsId_t evt_id;
#endif

	aspeed_device_t *device;
} kcs_t;

typedef struct bt_s {
	aspeed_device_t *device;
} bt_t;

typedef struct snoop_s {
	osMessageQueueId_t msg_q[2];
	aspeed_device_t *device;
} snoop_t;

typedef struct pcc_s {
	osMessageQueueId_t msg_q;
	struct {
		uint8_t *virt;
		uint32_t virt_idx;
		uint32_t size;
		uint32_t addr;
	} dma;
	aspeed_device_t *device;
} pcc_t;

typedef struct ast_timer_s {
	aspeed_device_t *device;
	uint32_t tick_per_1us;
} ast_timer_t;

typedef struct mdio_s {
	aspeed_device_t *device;
	uint32_t phy_addr;
} mdio_t;

typedef struct phy_s {
	mdio_t *mdio;				/* mdio device to control PHY */
	uint32_t speed;
	uint32_t duplex;
	uint32_t autoneg;
	uint32_t link;
	uint32_t phy_mode;			/* RGMII, RGMII_RXID or RGMII_TXID */
	uint16_t id[2];
	uint8_t loopback;
	void (*config)(struct phy_s *obj);
} phy_t;

typedef struct mac_txdes_s {
	uint32_t txdes0;
	uint32_t txdes1;
	uint32_t txdes2;
	uint32_t txdes3;
} mac_txdes_t;

typedef struct mac_rxdes_s {
	uint32_t rxdes0;
	uint32_t rxdes1;
	uint32_t rxdes2;
	uint32_t rxdes3;
} mac_rxdes_t;

typedef struct mac_s {
	aspeed_device_t *device;	/* MAC device self */
	phy_t *phy;					/* mdio device to control PHY */
	mac_txdes_t *txdes;			/* pointer to the TX descriptors */
	mac_rxdes_t *rxdes;			/* pointer to the RX descriptors */
	uint32_t txptr;
	uint32_t rxptr;
	uint32_t n_txdes;			/* number of TX descriptors */
	uint32_t n_rxdes;			/* number of RX descriptors */
	uint8_t **rx_pkt_buf;
	uint8_t *mac_addr;
	uint8_t is_rgmii;
} mac_t;

typedef struct spi_s {
	aspeed_device_t *device;
} spi_t;

typedef struct hace_s {
	aspeed_device_t *device;
} hace_t;

typedef struct rsa_s {
	aspeed_device_t *device;
} rsa_t;

#endif /* end of "ifndef _OBJECTS_H_" */
