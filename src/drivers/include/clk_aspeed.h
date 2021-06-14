#ifndef __CLK_ASPEED_H__
#define __CLK_ASPEED_H__

#include "objects.h"

/**
 * @brief enable device clock
 * @param [IN] device - pointer to the device
 * @return 0: success -1: device not found
*/
int aspeed_clk_enable(aspeed_device_t *device);

/**
 * @brief disable device clock
 * @param [IN] device - pointer to the device
 * @return 0: success -1: device not found
*/
int aspeed_clk_disable(aspeed_device_t *device);

/**
 * @brief get UART clock frequecy value
 * @param [IN] device - pointer to the device
 * @return UART clock frequency in Hz
*/
uint32_t aspeed_clk_get_uart_clk(aspeed_device_t *device);

/**
 * @brief get system HCLK clock frequecy value
 * @return UART clock frequency in Hz
*/
uint32_t aspeed_clk_get_hclk(void);

/**
 * @brief get system PCLK clock frequecy value
 * @return UART clock frequency in Hz
*/
uint32_t aspeed_clk_get_pclk(void);

#ifdef CONFIG_AST2600_SERIES
/**
 * @brief get system APB2 clock frequecy value
 * @return clock frequency in Hz
*/
uint32_t aspeed_clk_get_apb2(void);
#endif

/**
 * @brief set mac tx/rx rgmii clock delay
 * @param [IN] macdev: mac device ID
 * @param [IN] speed: 10, 100 or 1000
 * @param [IN] tx: tx delay value
 * @param [IN] rx: rx delay value
*/
void aspeed_clk_set_rgmii_delay(ASPEED_DEV_ID macdev, int speed, uint32_t tx,
								uint32_t rx);
/**
 * @brief get mac tx/rx rgmii clock delay
 * @param [IN] macdev: mac device ID
 * @param [IN] speed: 10, 100 or 1000
 * @param [OU] tx: tx delay value
 * @param [OU] rx: rx delay value
*/
void aspeed_clk_get_rgmii_delay(ASPEED_DEV_ID macdev, int speed,
								uint32_t *tx, uint32_t *rx);
#endif /* #ifndef __CLK_ASPEED_H__ */