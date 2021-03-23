/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "common.h"
#include "util.h"
#include "hal_def.h"
#include "reset_aspeed.h"
#include "clk_aspeed.h"
#include "cmsis_os.h"
#include "io.h"
#include <string.h>
#include "device.h"
#include "log.h"
#include "wait.h"
#include "i2c_aspeed.h"
#include "pinctrl_aspeed.h"

#define USE_OS_FLAG_FOR_WAIT
#ifdef USE_OS_FLAG_FOR_WAIT
#include "irq_aspeed.h"
#define DEV_ID_TO_EVENT_FLAG(x)	(1 << DEV_ID_TO_I2C_INDEX(x))
#define CONFIG_DEVICE_I2C_TIMEOUT 30
#endif

/***************************************************************************/
#define ASPEED_I2CG_ISR				0x00
#define ASPEED_I2CG_SLAVE_ISR		0x04	/* ast2600 */
#define ASPEED_I2CG_OWNER			0x08
#define ASPEED_I2CG_CTRL			0x0C
#define ASPEED_I2CG_CLK_DIV_CTRL	0x10	/* ast2600 */

#define ASPEED_I2CG_MBX_FIFO_ADDR	0x14	/* ast1060 */

/* 0x0C : I2CG SRAM Buffer Enable  */
#define ASPEED_I2CG_SRAM_BUFFER_ENABLE		BIT(0)

/*ast2600 */
#define ASPEED_I2CG_SLAVE_PKT_NAK		BIT(4)
#define ASPEED_I2CG_M_S_SEPARATE_INTR	BIT(3)
#define ASPEED_I2CG_CTRL_NEW_REG		BIT(2)
#define ASPEED_I2CG_CTRL_NEW_CLK_DIV	BIT(1)

/* ASPEED_I2CG_MBX_FIFO_ADDR	0x14	ast1060 FIFO register */
#define AST_I2CG_FIFO_ADDR_MASK			0xffff
#define AST_I2CG_FIFO_SIZE(x)			(x << 16)



/***************************************************************************/

//AST2600 reg
#define AST_I2CC_FUN_CTRL			0x00 	/* 0x00 : I2CC Master/Slave Function Control Register  */
#define AST_I2CC_SLAVE_MAILBOX_EN 		BIT(24)
#define AST_I2CC_SLAVE_MAILBOX_SRAM_EN 	BIT(23)

#define AST_I2CC_SLAVE_ADDR_RX_EN 		BIT(20)
#define AST_I2CC_MASTER_RETRY_MASK		(0x3 << 18)
#define AST_I2CC_MASTER_RETRY(x) 		((x & 0x3) << 18)
#define AST_I2CC_BUS_AUTO_RELEASE		BIT(17)	
#define AST_I2CC_M_SDA_LOCK_EN			BIT(16)
#define AST_I2CC_MULTI_MASTER_DIS		BIT(15)
#define AST_I2CC_M_SCL_DRIVE_EN			BIT(14)
#define AST_I2CC_MSB_STS				BIT(9)
#define AST_I2CC_SDA_DRIVE_1T_EN		BIT(8)
#define AST_I2CC_M_SDA_DRIVE_1T_EN		BIT(7)
#define AST_I2CC_M_HIGH_SPEED_EN		BIT(6)
/* reserver 5 : 2 */
#define AST_I2CC_SLAVE_EN				BIT(1)
#define AST_I2CC_MASTER_EN				BIT(0)

#define AST_I2CC_AC_TIMING			0x04	/* 0x04 : I2CC Master/Slave Clock and AC Timing Control Register #1 */
#define AST_I2CC_tTIMEOUT(x)			((x & 0x1f) << 24)	// 0~7
#define AST_I2CC_tCKHIGHMin(x)			((x & 0xf) << 20)	// 0~f 
#define AST_I2CC_tCKHIGH(x)				((x & 0xf) << 16)	// 0~7 
#define AST_I2CC_tCKLOW(x)				((x & 0xf) << 12)	// 0~7 
#define AST_I2CC_tHDDAT(x)				((x & 0x3) << 10)	// 0~3 
#define AST_I2CC_toutBaseCLK(x)			((x & 0x3) << 8)	//0~3
#define AST_I2CC_tBaseCLK(x)			(x & 0xf)	// 0~0xf

#define AST_I2CC_STS_AND_BUFF		0x08	/* 0x08 : I2CC Master/Slave Transmit/Receive Byte Buffer Register */
#define AST_I2CC_TX_DIR_MASK			(0x7 << 29)
#define AST_I2CC_SDA_OE					BIT(28)
#define AST_I2CC_SDA_O					BIT(27)
#define AST_I2CC_SCL_OE					BIT(26)
#define AST_I2CC_SCL_O					BIT(25)

// Tx State Machine
#define AST_I2CM_IDLE	 				0x0
#define AST_I2CM_MACTIVE				0x8
#define AST_I2CM_MSTART					0x9
#define AST_I2CM_MSTARTR				0xa
#define AST_I2CM_MSTOP					0xb
#define AST_I2CM_MTXD					0xc  
#define AST_I2CM_MRXACK					0xd
#define AST_I2CM_MRXD 					0xe
#define AST_I2CM_MTXACK 				0xf
#define AST_I2CM_SWAIT					0x1
#define AST_I2CM_SRXD 					0x4
#define AST_I2CM_STXACK 				0x5
#define AST_I2CM_STXD					0x6
#define AST_I2CM_SRXACK 				0x7
#define AST_I2CM_RECOVER 				0x3

#define AST_I2CC_SCL_LINE_STS			BIT(18)
#define AST_I2CC_SDA_LINE_STS			BIT(17)
#define AST_I2CC_BUS_BUSY_STS			BIT(16)

#define AST_I2CC_GET_RX_BUFF(x)			((x >> 8) & 0xff)

#define AST_I2CC_BUFF_CTRL		0x0C	/* 0x0C : I2CC Master/Slave Pool Buffer Control Register  */
#define AST_I2CC_GET_RX_BUF_LEN(x)		((x >> 24) & 0x3f)
#define AST_I2CC_SET_RX_BUF_LEN(x)		(((x - 1) & 0x1f) << 16)
#define AST_I2CC_SET_TX_BUF_LEN(x)		(((x - 1) & 0x1f) << 8)
#define AST_I2CC_GET_TX_BUF_LEN(x)		(((x >> 8) & 0x1f) + 1)

#define AST_I2CM_IER			0x10	/* 0x10 : I2CM Master Interrupt Control Register */
#define AST_I2CM_ISR			0x14	/* 0x14 : I2CM Master Interrupt Status Register   : WC */

#define AST_I2CM_PKT_TIMEOUT			BIT(18)
#define AST_I2CM_PKT_ERROR				BIT(17)
#define AST_I2CM_PKT_DONE				BIT(16)

#define AST_I2CM_BUS_RECOVER_FAIL		BIT(15)
#define AST_I2CM_SDA_DL_TO				BIT(14)
#define AST_I2CM_BUS_RECOVER			BIT(13)
#define AST_I2CM_SMBUS_ALT				BIT(12)

#define AST_I2CM_SCL_LOW_TO				BIT(6)
#define AST_I2CM_ABNORMAL				BIT(5)
#define AST_I2CM_NORMAL_STOP			BIT(4)
#define AST_I2CM_ARBIT_LOSS				BIT(3)
#define AST_I2CM_RX_DONE				BIT(2)
#define AST_I2CM_TX_NAK					BIT(1)
#define AST_I2CM_TX_ACK					BIT(0)

#define AST_I2CM_CMD_STS		0x18	/* 0x18 : I2CM Master Command/Status Register   */
#define AST_I2CM_PKT_ADDR(x)			((x & 0x7f) << 24)	
#define AST_I2CM_PKT_EN					BIT(16)
#define AST_I2CM_SDA_OE_OUT_DIR			BIT(15)
#define AST_I2CM_SDA_O_OUT_DIR			BIT(14)
#define AST_I2CM_SCL_OE_OUT_DIR			BIT(13)
#define AST_I2CM_SCL_O_OUT_DIR			BIT(12)
#define AST_I2CM_RECOVER_CMD_EN			BIT(11)

#define AST_I2CM_RX_DMA_EN				BIT(9)
#define AST_I2CM_TX_DMA_EN				BIT(8)
#define AST_I2CM_RX_BUFF_EN				BIT(7)
#define AST_I2CM_TX_BUFF_EN				BIT(6)

/* Command Bit */
#define AST_I2CM_RX_BUFF_EN				BIT(7)
#define AST_I2CM_TX_BUFF_EN				BIT(6)
#define AST_I2CM_STOP_CMD				BIT(5)
#define AST_I2CM_RX_CMD_LAST			BIT(4)
#define AST_I2CM_RX_CMD					BIT(3)

#define AST_I2CM_TX_CMD					BIT(1)
#define AST_I2CM_START_CMD				BIT(0)

#define AST_I2CM_DMA_LEN		0x1C	/* 0x1C : I2CM Master DMA Transfer Length Register   */
#define AST_I2CM_SET_RX_DMA_LEN(x)		((((x) & 0xfff) << 16) | BIT(31))	/* 1 ~ 4096 */
#define AST_I2CM_SET_TX_DMA_LEN(x)		(((x) & 0xfff) | BIT(15))			/* 1 ~ 4096 */

#define AST_I2CS_IER			0x20	/* 0x20 : I2CS Slave Interrupt Control Register   */
#define AST_I2CS_ISR			0x24	/* 0x24 : I2CS Slave Interrupt Status Register   */

#define AST_I2CS_ADDR_INDICAT_MASK	(3 << 30)

#define AST_I2CS_Wait_TX_DMA		BIT(25)
#define AST_I2CS_Wait_RX_DMA		BIT(24)


#define AST_I2CS_ADDR3_NAK			BIT(22)
#define AST_I2CS_ADDR2_NAK			BIT(21)
#define AST_I2CS_ADDR1_NAK			BIT(20)

#define AST_I2CS_ADDR_MASK			(3 << 18)
#define AST_I2CS_PKT_ERROR			BIT(17)
#define AST_I2CS_PKT_DONE			BIT(16)
#define AST_I2CS_INACTIVE_TO		BIT(15)
//
#define AST_I2CS_SLAVE_MATCH		BIT(7)
//
#define AST_I2CS_ABNOR_STOP			BIT(5)
#define AST_I2CS_STOP				BIT(4)
#define AST_I2CS_RX_DONE_NAK		BIT(3)
#define AST_I2CS_RX_DONE			BIT(2)
#define AST_I2CS_TX_NAK				BIT(1)
#define AST_I2CS_TX_ACK				BIT(0)

#define AST_I2CS_CMD_STS		0x28	/* 0x28 : I2CS Slave CMD/Status Register   */
#define AST_I2CS_ACTIVE_ALL				(0x3 << 17)
#define AST_I2CS_PKT_MODE_EN			BIT(16)
#define AST_I2CS_AUTO_NAK_NOADDR		BIT(15)
#define AST_I2CS_AUTO_NAK_EN			BIT(14)

/* new for i2c snoop */
#define AST_I2CS_SNOOP_LOOP				BIT(12)
#define AST_I2CS_SNOOP_EN				BIT(11)

#define AST_I2CS_ALT_EN					BIT(10)
#define AST_I2CS_RX_DMA_EN				BIT(9)
#define AST_I2CS_TX_DMA_EN				BIT(8)
#define AST_I2CS_RX_BUFF_EN				BIT(7)
#define AST_I2CS_TX_BUFF_EN				BIT(6)
#define AST_I2CS_RX_CMD_LAST			BIT(4)

#define AST_I2CS_TX_CMD					BIT(2)

#define AST_I2CS_DMA_LEN		0x2C
#define AST_I2CS_SET_RX_DMA_LEN(x)		((((x - 1) & 0xfff) << 16) | BIT(31))
#define AST_I2CS_RX_DMA_LEN_MASK		(0xfff << 16)

#define AST_I2CS_SET_TX_DMA_LEN(x)		(((x - 1) & 0xfff) | BIT(15))
#define AST_I2CS_TX_DMA_LEN_MASK		0xfff

#define AST_I2CM_TX_DMA			0x30 	/* I2CM Master DMA Tx Buffer Register   */
#define AST_I2CM_RX_DMA			0x34	/* I2CM Master DMA Rx Buffer Register   */
#define AST_I2CS_TX_DMA			0x38 	/* I2CS Slave DMA Tx Buffer Register   */
#define AST_I2CS_RX_DMA			0x3C	/* I2CS Slave DMA Rx Buffer Register   */

/* 0x40 : Slave Device Address Register */
#define AST_I2CS_ADDR_CTRL		0x40

#define AST_I2CS_ADDR3_MBX_TYPE(x)		(x << 28)
#define AST_I2CS_ADDR2_MBX_TYPE(x)		(x << 26)
#define AST_I2CS_ADDR1_MBX_TYPE(x)		(x << 24)
#define AST_I2CS_ADDR3_ENABLE			BIT(23)
#define AST_I2CS_ADDR3(x)				((x & 0x7f) << 16)
#define AST_I2CS_ADDR2_ENABLE			BIT(15)
#define AST_I2CS_ADDR2(x)				((x & 0x7f) << 8)
#define AST_I2CS_ADDR1_ENABLE			BIT(7)
#define AST_I2CS_ADDR1(x)				(x & 0x7f)

#define	AST_I2CS_ADDR3_MASK		(0x7f << 16)
#define	AST_I2CS_ADDR2_MASK		(0x7f << 8)
#define	AST_I2CS_ADDR1_MASK		0x7f

#define AST_I2CM_DMA_LEN_STS	0x48
#define AST_I2CS_DMA_LEN_STS	0x4C

#define AST_I2C_GET_TX_DMA_LEN(x)		(x & 0x1fff)
#define AST_I2C_GET_RX_DMA_LEN(x)		((x >> 16) & 0x1fff)

/* new for i2c snoop */
#define AST_I2CS_SNOOP_DMA_WPT	0x50
#define AST_I2CS_SNOOP_DMA_RPT	0x58
/***************************************************************************/
/* Use platform_data instead of module parameters */
/* Fast Mode = 400 kHz, Standard = 100 kHz */
//static int clock = 100; /* Default: 100 kHz */
/***************************************************************************/
#define AST_LOCKUP_DETECTED 	BIT(15)
#define AST_I2C_LOW_TIMEOUT 	0x07
/***************************************************************************/
#define ASPEED_I2C_DMA_SIZE		4096
/***************************************************************************/

#define DEV_ID_TO_I2C_INDEX(x)	(x - ASPEED_DEV_I2C0)

struct i2c_client {
	unsigned short flags;		/* div., see below		*/
	unsigned short addr;		/* chip address - NOTE: 7bit	*/
					/* addresses are stored in the	*/
					/* _LOWER_ 7 bits		*/
	int init_irq;			/* irq set at initialization	*/
	int irq;			/* irq issued by device		*/
};

struct ast_i2c_timing_table {
	uint32_t divisor;
	uint32_t timing;
};

static struct ast_i2c_timing_table aspeed_old_i2c_timing_table[] = {
	/* Divisor : Base Clock : tCKHighMin : tCK High : tCK Low  */
	/* Divisor :	  [3:0] : [23: 20]   :   [19:16]:   [15:12] */
	{6,	0x00000300 | (0x0) | (0x2 << 20) | (0x2 << 16) | (0x2 << 12) },
	{7,	0x00000300 | (0x0) | (0x3 << 20) | (0x3 << 16) | (0x2 << 12) },
	{8,	0x00000300 | (0x0) | (0x3 << 20) | (0x3 << 16) | (0x3 << 12) },
	{9,	0x00000300 | (0x0) | (0x4 << 20) | (0x4 << 16) | (0x3 << 12) },
	{10, 	0x00000300 | (0x0) | (0x4 << 20) | (0x4 << 16) | (0x4 << 12) },
	{11, 	0x00000300 | (0x0) | (0x5 << 20) | (0x5 << 16) | (0x4 << 12) },
	{12, 	0x00000300 | (0x0) | (0x5 << 20) | (0x5 << 16) | (0x5 << 12) },
	{13, 	0x00000300 | (0x0) | (0x6 << 20) | (0x6 << 16) | (0x5 << 12) },
	{14, 	0x00000300 | (0x0) | (0x6 << 20) | (0x6 << 16) | (0x6 << 12) },
	{15, 	0x00000300 | (0x0) | (0x7 << 20) | (0x7 << 16) | (0x6 << 12) },
	{16, 	0x00000300 | (0x0) | (0x7 << 20) | (0x7 << 16) | (0x7 << 12) },
	{17, 	0x00000300 | (0x0) | (0x8 << 20) | (0x8 << 16) | (0x7 << 12) },
	{18, 	0x00000300 | (0x0) | (0x8 << 20) | (0x8 << 16) | (0x8 << 12) },
	{19, 	0x00000300 | (0x0) | (0x9 << 20) | (0x9 << 16) | (0x8 << 12) },
	{20, 	0x00000300 | (0x0) | (0x9 << 20) | (0x9 << 16) | (0x9 << 12) },
	{21, 	0x00000300 | (0x0) | (0xa << 20) | (0xa << 16) | (0x9 << 12) },
	{22, 	0x00000300 | (0x0) | (0xa << 20) | (0xa << 16) | (0xa << 12) },
	{23, 	0x00000300 | (0x0) | (0xb << 20) | (0xb << 16) | (0xa << 12) },
	{24, 	0x00000300 | (0x0) | (0xb << 20) | (0xb << 16) | (0xb << 12) },
	{25, 	0x00000300 | (0x0) | (0xc << 20) | (0xc << 16) | (0xb << 12) },
	{26, 	0x00000300 | (0x0) | (0xc << 20) | (0xc << 16) | (0xc << 12) },
	{27, 	0x00000300 | (0x0) | (0xd << 20) | (0xd << 16) | (0xc << 12) },
	{28, 	0x00000300 | (0x0) | (0xd << 20) | (0xd << 16) | (0xd << 12) },
	{29, 	0x00000300 | (0x0) | (0xe << 20) | (0xe << 16) | (0xd << 12) },
	{30, 	0x00000300 | (0x0) | (0xe << 20) | (0xe << 16) | (0xe << 12) },
	{31, 	0x00000300 | (0x0) | (0xf << 20) | (0xf << 16) | (0xe << 12) },
	{32, 	0x00000300 | (0x0) | (0xf << 20) | (0xf << 16) | (0xf << 12) },

	{34, 	0x00000300 | (0x1) | (0x8 << 20) | (0x8 << 16) | (0x7 << 12) },
	{36, 	0x00000300 | (0x1) | (0x8 << 20) | (0x8 << 16) | (0x8 << 12) },
	{38, 	0x00000300 | (0x1) | (0x9 << 20) | (0x9 << 16) | (0x8 << 12) },
	{40, 	0x00000300 | (0x1) | (0x9 << 20) | (0x9 << 16) | (0x9 << 12) },
	{42, 	0x00000300 | (0x1) | (0xa << 20) | (0xa << 16) | (0x9 << 12) },
	{44, 	0x00000300 | (0x1) | (0xa << 20) | (0xa << 16) | (0xa << 12) },
	{46, 	0x00000300 | (0x1) | (0xb << 20) | (0xb << 16) | (0xa << 12) },
	{48, 	0x00000300 | (0x1) | (0xb << 20) | (0xb << 16) | (0xb << 12) },
	{50, 	0x00000300 | (0x1) | (0xc << 20) | (0xc << 16) | (0xb << 12) },
	{52, 	0x00000300 | (0x1) | (0xc << 20) | (0xc << 16) | (0xc << 12) },
	{54, 	0x00000300 | (0x1) | (0xd << 20) | (0xd << 16) | (0xc << 12) },
	{56, 	0x00000300 | (0x1) | (0xd << 20) | (0xd << 16) | (0xd << 12) },
	{58, 	0x00000300 | (0x1) | (0xe << 20) | (0xe << 16) | (0xd << 12) },
	{60, 	0x00000300 | (0x1) | (0xe << 20) | (0xe << 16) | (0xe << 12) },
	{62, 	0x00000300 | (0x1) | (0xf << 20) | (0xf << 16) | (0xe << 12) },
	{64, 	0x00000300 | (0x1) | (0xf << 20) | (0xf << 16) | (0xf << 12) },

	{68, 	0x00000300 | (0x2) | (0x8 << 20) | (0x8 << 16) | (0x7 << 12) },
	{72, 	0x00000300 | (0x2) | (0x8 << 20) | (0x8 << 16) | (0x8 << 12) },
	{76, 	0x00000300 | (0x2) | (0x9 << 20) | (0x9 << 16) | (0x8 << 12) },
	{80, 	0x00000300 | (0x2) | (0x9 << 20) | (0x9 << 16) | (0x9 << 12) },
	{84, 	0x00000300 | (0x2) | (0xa << 20) | (0xa << 16) | (0x9 << 12) },
	{88, 	0x00000300 | (0x2) | (0xa << 20) | (0xa << 16) | (0xa << 12) },
	{92, 	0x00000300 | (0x2) | (0xb << 20) | (0xb << 16) | (0xa << 12) },
	{96, 	0x00000300 | (0x2) | (0xb << 20) | (0xb << 16) | (0xb << 12) },
	{100, 	0x00000300 | (0x2) | (0xc << 20) | (0xc << 16) | (0xb << 12) },
	{104, 	0x00000300 | (0x2) | (0xc << 20) | (0xc << 16) | (0xc << 12) },
	{108, 	0x00000300 | (0x2) | (0xd << 20) | (0xd << 16) | (0xc << 12) },
	{112, 	0x00000300 | (0x2) | (0xd << 20) | (0xd << 16) | (0xd << 12) },
	{116, 	0x00000300 | (0x2) | (0xe << 20) | (0xe << 16) | (0xd << 12) },
	{120, 	0x00000300 | (0x2) | (0xe << 20) | (0xe << 16) | (0xe << 12) },
	{124, 	0x00000300 | (0x2) | (0xf << 20) | (0xf << 16) | (0xe << 12) },
	{128, 	0x00000300 | (0x2) | (0xf << 20) | (0xf << 16) | (0xf << 12) },

	{136, 	0x00000300 | (0x3) | (0x8 << 20) | (0x8 << 16) | (0x7 << 12) },
	{144, 	0x00000300 | (0x3) | (0x8 << 20) | (0x8 << 16) | (0x8 << 12) },
	{152, 	0x00000300 | (0x3) | (0x9 << 20) | (0x9 << 16) | (0x8 << 12) },
	{160, 	0x00000300 | (0x3) | (0x9 << 20) | (0x9 << 16) | (0x9 << 12) },
	{168, 	0x00000300 | (0x3) | (0xa << 20) | (0xa << 16) | (0x9 << 12) },
	{176, 	0x00000300 | (0x3) | (0xa << 20) | (0xa << 16) | (0xa << 12) },
	{184, 	0x00000300 | (0x3) | (0xb << 20) | (0xb << 16) | (0xa << 12) },
	{192, 	0x00000300 | (0x3) | (0xb << 20) | (0xb << 16) | (0xb << 12) },
	{200, 	0x00000300 | (0x3) | (0xc << 20) | (0xc << 16) | (0xb << 12) },
	{208, 	0x00000300 | (0x3) | (0xc << 20) | (0xc << 16) | (0xc << 12) },
	{216, 	0x00000300 | (0x3) | (0xd << 20) | (0xd << 16) | (0xc << 12) },
	{224, 	0x00000300 | (0x3) | (0xd << 20) | (0xd << 16) | (0xd << 12) },
	{232, 	0x00000300 | (0x3) | (0xe << 20) | (0xe << 16) | (0xd << 12) },
	{240, 	0x00000300 | (0x3) | (0xe << 20) | (0xe << 16) | (0xe << 12) },
	{248, 	0x00000300 | (0x3) | (0xf << 20) | (0xf << 16) | (0xe << 12) },
	{256, 	0x00000300 | (0x3) | (0xf << 20) | (0xf << 16) | (0xf << 12) },

	{272, 	0x00000300 | (0x4) | (0x8 << 20) | (0x8 << 16) | (0x7 << 12) },
	{288, 	0x00000300 | (0x4) | (0x8 << 20) | (0x8 << 16) | (0x8 << 12) },
	{304, 	0x00000300 | (0x4) | (0x9 << 20) | (0x9 << 16) | (0x8 << 12) },
	{320, 	0x00000300 | (0x4) | (0x9 << 20) | (0x9 << 16) | (0x9 << 12) },
	{336, 	0x00000300 | (0x4) | (0xa << 20) | (0xa << 16) | (0x9 << 12) },
	{352, 	0x00000300 | (0x4) | (0xa << 20) | (0xa << 16) | (0xa << 12) },
	{368, 	0x00000300 | (0x4) | (0xb << 20) | (0xb << 16) | (0xa << 12) },
	{384, 	0x00000300 | (0x4) | (0xb << 20) | (0xb << 16) | (0xb << 12) },
	{400, 	0x00000300 | (0x4) | (0xc << 20) | (0xc << 16) | (0xb << 12) },
	{416, 	0x00000300 | (0x4) | (0xc << 20) | (0xc << 16) | (0xc << 12) },
	{432, 	0x00000300 | (0x4) | (0xd << 20) | (0xd << 16) | (0xc << 12) },
	{448, 	0x00000300 | (0x4) | (0xd << 20) | (0xd << 16) | (0xd << 12) },
	{464, 	0x00000300 | (0x4) | (0xe << 20) | (0xe << 16) | (0xd << 12) },
	{480, 	0x00000300 | (0x4) | (0xe << 20) | (0xe << 16) | (0xe << 12) },
	{496, 	0x00000300 | (0x4) | (0xf << 20) | (0xf << 16) | (0xe << 12) },
	{512, 	0x00000300 | (0x4) | (0xf << 20) | (0xf << 16) | (0xf << 12) },

	{544, 	0x00000300 | (0x5) | (0x8 << 20) | (0x8 << 16) | (0x7 << 12) },
	{576, 	0x00000300 | (0x5) | (0x8 << 20) | (0x8 << 16) | (0x8 << 12) },
	{608, 	0x00000300 | (0x5) | (0x9 << 20) | (0x9 << 16) | (0x8 << 12) },
	{640, 	0x00000300 | (0x5) | (0x9 << 20) | (0x9 << 16) | (0x9 << 12) },
	{672, 	0x00000300 | (0x5) | (0xa << 20) | (0xa << 16) | (0x9 << 12) },
	{704, 	0x00000300 | (0x5) | (0xa << 20) | (0xa << 16) | (0xa << 12) },
	{736, 	0x00000300 | (0x5) | (0xb << 20) | (0xb << 16) | (0xa << 12) },
	{768, 	0x00000300 | (0x5) | (0xb << 20) | (0xb << 16) | (0xb << 12) },
	{800, 	0x00000300 | (0x5) | (0xc << 20) | (0xc << 16) | (0xb << 12) },
	{832, 	0x00000300 | (0x5) | (0xc << 20) | (0xc << 16) | (0xc << 12) },
	{864, 	0x00000300 | (0x5) | (0xd << 20) | (0xd << 16) | (0xc << 12) },
	{896, 	0x00000300 | (0x5) | (0xd << 20) | (0xd << 16) | (0xd << 12) },
	{928, 	0x00000300 | (0x5) | (0xe << 20) | (0xe << 16) | (0xd << 12) },
	{960, 	0x00000300 | (0x5) | (0xe << 20) | (0xe << 16) | (0xe << 12) },
	{992, 	0x00000300 | (0x5) | (0xf << 20) | (0xf << 16) | (0xe << 12) },
	{1024, 	0x00000300 | (0x5) | (0xf << 20) | (0xf << 16) | (0xf << 12) },

	{1088, 	0x00000300 | (0x6) | (0x8 << 20) | (0x8 << 16) | (0x7 << 12) },
	{1152, 	0x00000300 | (0x6) | (0x8 << 20) | (0x8 << 16) | (0x8 << 12) },
	{1216, 	0x00000300 | (0x6) | (0x9 << 20) | (0x9 << 16) | (0x8 << 12) },
	{1280, 	0x00000300 | (0x6) | (0x9 << 20) | (0x9 << 16) | (0x9 << 12) },
	{1344, 	0x00000300 | (0x6) | (0xa << 20) | (0xa << 16) | (0x9 << 12) },
	{1408, 	0x00000300 | (0x6) | (0xa << 20) | (0xa << 16) | (0xa << 12) },
	{1472, 	0x00000300 | (0x6) | (0xb << 20) | (0xb << 16) | (0xa << 12) },
	{1536, 	0x00000300 | (0x6) | (0xb << 20) | (0xb << 16) | (0xb << 12) },
	{1600, 	0x00000300 | (0x6) | (0xc << 20) | (0xc << 16) | (0xb << 12) },
	{1664, 	0x00000300 | (0x6) | (0xc << 20) | (0xc << 16) | (0xc << 12) },
	{1728, 	0x00000300 | (0x6) | (0xd << 20) | (0xd << 16) | (0xc << 12) },
	{1792, 	0x00000300 | (0x6) | (0xd << 20) | (0xd << 16) | (0xd << 12) },
	{1856, 	0x00000300 | (0x6) | (0xe << 20) | (0xe << 16) | (0xd << 12) },
	{1920, 	0x00000300 | (0x6) | (0xe << 20) | (0xe << 16) | (0xe << 12) },
	{1984, 	0x00000300 | (0x6) | (0xf << 20) | (0xf << 16) | (0xe << 12) },
	{2048, 	0x00000300 | (0x6) | (0xf << 20) | (0xf << 16) | (0xf << 12) },

	{2176, 	0x00000300 | (0x7) | (0x8 << 20) | (0x8 << 16) | (0x7 << 12) },
	{2304, 	0x00000300 | (0x7) | (0x8 << 20) | (0x8 << 16) | (0x8 << 12) },
	{2432, 	0x00000300 | (0x7) | (0x9 << 20) | (0x9 << 16) | (0x8 << 12) },
	{2560, 	0x00000300 | (0x7) | (0x9 << 20) | (0x9 << 16) | (0x9 << 12) },
	{2688, 	0x00000300 | (0x7) | (0xa << 20) | (0xa << 16) | (0x9 << 12) },
	{2816, 	0x00000300 | (0x7) | (0xa << 20) | (0xa << 16) | (0xa << 12) },
	{2944, 	0x00000300 | (0x7) | (0xb << 20) | (0xb << 16) | (0xa << 12) },
	{3072, 	0x00000300 | (0x7) | (0xb << 20) | (0xb << 16) | (0xb << 12) },
};

//#define AST1030
//#define ASPEED_I2C_DEBUG

#ifdef ASPEED_I2C_DEBUG
#define I2C_DBUG(fmt, args...) printf("%s() " fmt, __FUNCTION__, ## args)
#else
#define I2C_DBUG(fmt, args...)
#endif

//#define ASPEED_I2CS_DEBUG

#ifdef ASPEED_I2CS_DEBUG
#define I2CS_DBUG(fmt, args...) printf("%s() " fmt, __FUNCTION__, ## args)
#else
#define I2CS_DBUG(fmt, args...)
#endif

void i2c_global_init(aspeed_device_t *i2c_global)
{
	uint32_t base = i2c_global->base;

	if(i2c_global->init)
		return;
	
	log_debug("i2c global base %x \n", base);
#ifdef CONFIG_AST2600_SERIES
	aspeed_reset_deassert(i2c_global);
#else
	aspeed_reset_assert(i2c_global);
	/* OS is not start yet, use cpu looping for waiting */
	aspeed_wait_ms(1);
	aspeed_reset_deassert(i2c_global);
#endif
	//TODO define
	writel(0x14, base + 0x0C);

	i2c_global->init = 1;
}

static int aspeed_new_i2c_is_irq_error(uint32_t irq_status)
{
	if (irq_status & AST_I2CM_ARBIT_LOSS)
	    return 1;
	if (irq_status & (AST_I2CM_SDA_DL_TO |
	              AST_I2CM_SCL_LOW_TO))
	    return 1;
	if (irq_status & (AST_I2CM_ABNORMAL))
	    return 1;

	return 0;
}

int aspeed_i2c_master_irq(i2c_t *obj)
{
	aspeed_device_t *device = obj->device;	
	uint32_t sts = readl(device->base + AST_I2CM_ISR);
	uint32_t cmd = AST_I2CM_PKT_EN;
	int xfer_len;
	uint8_t wbuf[4];	
	int i;

	I2C_DBUG("M-%d sts %x\n", device->dev_id, sts);
	
	obj->cmd_err = 0;

	if (AST_I2CM_BUS_RECOVER_FAIL & sts) {
		I2C_DBUG("M clear isr: AST_I2CM_BUS_RECOVER_FAIL= %x\n", sts);
		writel(AST_I2CM_BUS_RECOVER_FAIL, device->base + AST_I2CM_ISR);
		if (obj->bus_recover)
			obj->bus_recover = 0;
		else
			I2C_DBUG("Error !! Bus revover\n");
		return 1;
	}
	
	if (AST_I2CM_BUS_RECOVER & sts) {
		I2C_DBUG("M clear isr: AST_I2CM_BUS_RECOVER %x\n", sts);
		writel(AST_I2CM_BUS_RECOVER, device->base + AST_I2CM_ISR);
		obj->cmd_err = 0;
		if (obj->bus_recover)
			obj->bus_recover = 0;
		else 
			I2C_DBUG("Error !! Bus revover\n");

		return 1;
	}

	if (AST_I2CM_SMBUS_ALT & sts) {
		I2C_DBUG("M clear isr: AST_I2CM_SMBUS_ALT= %x\n", sts);
		//Disable ALT INT
		writel(readl(device->base + AST_I2CM_IER) &
			      ~AST_I2CM_SMBUS_ALT, 
			      device->base + AST_I2CM_IER);
		writel(AST_I2CM_SMBUS_ALT, device->base + AST_I2CM_ISR);
		printf("TODO aspeed_master_alert_recv bus id %d, Disable Alt, Please Imple \n");
		return 1;
	}

	obj->cmd_err = aspeed_new_i2c_is_irq_error(sts);
	if(obj->cmd_err) {
		I2C_DBUG("received error interrupt: 0x%02x\n", sts);
		writel(AST_I2CM_PKT_DONE, device->base + AST_I2CM_ISR);
		return 1;
	}

	if (AST_I2CM_PKT_DONE & sts) {
		sts &= ~AST_I2CM_PKT_DONE;
		writel(AST_I2CM_PKT_DONE, device->base + AST_I2CM_ISR);
		switch (sts) {
			case AST_I2CM_PKT_ERROR | AST_I2CM_TX_NAK | AST_I2CM_NORMAL_STOP:
				I2C_DBUG("M : TX NAK | NORMAL STOP \n");
				obj->cmd_err = 1;
				obj->xfer_complete = 1;
				break;
			case AST_I2CM_NORMAL_STOP:
				//write 0 byte only have stop isr
				I2C_DBUG("M clear isr: AST_I2CM_NORMAL_STOP = %x\n", sts);
				obj->xfer_complete = 1;
				break;
			case AST_I2CM_TX_ACK:
				I2C_DBUG("M : AST_I2CM_TX_ACK = %x\n", sts);
			case AST_I2CM_TX_ACK | AST_I2CM_NORMAL_STOP:
				if(AST_I2CM_NORMAL_STOP & sts)
					I2C_DBUG("M : AST_I2CM_TX_ACK | AST_I2CM_NORMAL_STOP= %x\n", sts);
				
				if (obj->mode == DMA_MODE) {
					xfer_len = AST_I2C_GET_TX_DMA_LEN(readl(device->base + AST_I2CM_DMA_LEN_STS));
				} else if (obj->mode == BUFF_MODE) {
					xfer_len = AST_I2CC_GET_TX_BUF_LEN(readl(device->base + AST_I2CC_BUFF_CTRL));
				} else {
					xfer_len = 1;
				}
				obj->master_xfer_cnt += xfer_len;
				I2C_DBUG("M : %d tx done [%d/%d]\n", xfer_len, obj->master_xfer_cnt, obj->len);
				if (obj->master_xfer_cnt == obj->len) {
					obj->xfer_complete = 1;
				} else {
					//do next tx
					cmd |= AST_I2CM_TX_CMD;
					if (obj->mode == DMA_MODE) {
						cmd |= AST_I2CS_TX_DMA_EN;
						xfer_len = obj->len - obj->master_xfer_cnt;
						if(xfer_len > ASPEED_I2C_DMA_SIZE) {
							xfer_len = ASPEED_I2C_DMA_SIZE;
						} else {
							//TODO
							#if 0
							if(obj->msgs_index + 1 == i2c_bus->msgs_count) {
								I2C_DBUG("M: STOP \n");
								cmd |= AST_I2CM_STOP_CMD;
							}
							#endif
						}
						writel(AST_I2CM_SET_TX_DMA_LEN(xfer_len - 1), device->base + AST_I2CM_DMA_LEN);
						I2C_DBUG("next tx xfer_len: %d, offset %d \n", xfer_len, obj->master_xfer_cnt);
//						writel(i2c_bus->master_dma_addr + i2c_bus->master_xfer_cnt, device->base + AST_I2CM_TX_DMA);
					} else if (obj->mode == BUFF_MODE) {
						cmd |= AST_I2CS_RX_BUFF_EN;
						xfer_len = obj->len - obj->master_xfer_cnt;
						if(xfer_len > obj->buf_size) {
							xfer_len = obj->buf_size;
						} else {
							if(obj->flags) {
								I2C_DBUG("M:| STOP \n");
								cmd |= AST_I2CM_STOP_CMD;
							}
						}
						for(i = 0; i < xfer_len; i++) {
							wbuf[i % 4] = obj->buf[obj->master_xfer_cnt + i];
							if (i % 4 == 3)
								writel(*(uint32_t *)wbuf,
									   obj->buf_base + i - 3);
							I2C_DBUG("[%02x] \n", obj->buf[obj->master_xfer_cnt + i]);
						}
						if (--i % 4 != 3)
							writel(*(uint32_t *)wbuf,
								   obj->buf_base + i - (i % 4));
						writel(AST_I2CC_SET_TX_BUF_LEN(xfer_len), AST_I2CC_BUFF_CTRL);
					} else {
						//byte
						#if 0
						if((i2c_bus->msgs_index + 1 == i2c_bus->msgs_count) && ((i2c_bus->master_xfer_cnt + 1) == msg->len)) {
							I2C_DBUG("M: STOP \n");
							cmd |= AST_I2CM_STOP_CMD;
						}
						I2C_DBUG("tx buff[%x] \n", msg->buf[i2c_bus->master_xfer_cnt]);
						writel(msg->buf[i2c_bus->master_xfer_cnt], AST_I2CC_STS_AND_BUFF);
						#endif
					}
					I2C_DBUG("next tx cmd: %x\n", cmd);					
					writel(cmd, device->base + AST_I2CM_CMD_STS);
				}
				break;
			case AST_I2CM_RX_DONE:
				I2C_DBUG("M : AST_I2CM_RX_DONE = %x\n", sts);
			case AST_I2CM_RX_DONE | AST_I2CM_NORMAL_STOP:				
				I2C_DBUG("M : AST_I2CM_RX_DONE | AST_I2CM_NORMAL_STOP = %x\n", sts);
				//do next rx
				if (obj->mode == DMA_MODE) {
					xfer_len = AST_I2C_GET_RX_DMA_LEN(readl(device->base + AST_I2CM_DMA_LEN_STS));
				} else if (obj->mode == BUFF_MODE) {
					xfer_len = AST_I2CC_GET_RX_BUF_LEN(readl(device->base + AST_I2CC_BUFF_CTRL));
					for(i = 0; i < xfer_len; i++)
						obj->buf[obj->master_xfer_cnt + i] = readb(obj->buf_base + i);
				} else {
					xfer_len = 1;
					obj->buf[obj->master_xfer_cnt] = AST_I2CC_GET_RX_BUFF(readl(device->base + AST_I2CC_STS_AND_BUFF));
				}

                if (obj->flags & I2C_M_RECV_LEN) {
                    I2C_DBUG("smbus first len = %x\n", obj->buf[0]);
					obj->len = obj->buf[0] + 1;
                    obj->flags &= ~I2C_M_RECV_LEN;
				}
				obj->master_xfer_cnt += xfer_len;
				I2C_DBUG("xfer_len %d, master_xfer_cnt [%d/%d] \n", xfer_len, obj->master_xfer_cnt, obj->len);

				if (obj->master_xfer_cnt == obj->len) {
					for (i = 0; i < obj->len; i++) {
							 I2C_DBUG("M: r %d:[%x] \n", i, obj->buf[i]);
					}
					obj->xfer_complete = 1;
                } else {
					//next rx
					cmd |= AST_I2CM_RX_CMD;
					if (obj->mode == DMA_MODE) {
						cmd |= AST_I2CM_RX_DMA_EN;
						xfer_len = obj->len - obj->master_xfer_cnt;
						if(xfer_len > ASPEED_I2C_DMA_SIZE) {
							xfer_len = ASPEED_I2C_DMA_SIZE;
						} else {
							cmd |= AST_I2CM_RX_CMD_LAST | AST_I2CM_STOP_CMD;
						}
						I2C_DBUG("M: next rx len [%d/%d] , cmd %x  \n", xfer_len, obj->len, cmd);
						writel(AST_I2CM_SET_RX_DMA_LEN(xfer_len - 1), device->base + AST_I2CM_DMA_LEN);
#ifdef AST1030
						writel((uint32_t) (obj->buf + obj->master_xfer_cnt) | 0x80000000, device->base + AST_I2CM_RX_DMA);
#else
						writel((uint32_t) obj->buf + obj->master_xfer_cnt, device->base + AST_I2CM_RX_DMA);
#endif

					} else if (obj->mode == BUFF_MODE) {
						cmd |= AST_I2CM_RX_BUFF_EN;
						xfer_len = obj->len - obj->master_xfer_cnt;
						if(xfer_len > obj->buf_size) {
							xfer_len = obj->buf_size;
						} else {
							I2C_DBUG("last stop \n");
							cmd |= AST_I2CM_RX_CMD_LAST | AST_I2CM_STOP_CMD;
						}
						writel(AST_I2CC_SET_RX_BUF_LEN(xfer_len), device->base + AST_I2CC_BUFF_CTRL);
					} else {
						if ((obj->master_xfer_cnt + 1) == obj->len) {
							I2C_DBUG("last stop \n");
							cmd |= AST_I2CM_RX_CMD_LAST | AST_I2CM_STOP_CMD;
						}
					}
					I2C_DBUG("M: next rx len %d, cmd %x \n", xfer_len, cmd);
					writel(cmd, device->base + AST_I2CM_CMD_STS);
                }
				break;
			default:
				printf("TODO care -- > sts %x \n", sts);
				break;
		}

		return 1;
	}

	if(readl(device->base + AST_I2CM_ISR)) {
		printf("TODO care -- > sts %x \n", readl(device->base + AST_I2CM_ISR));
		writel(readl(device->base + AST_I2CM_ISR), device->base + AST_I2CM_ISR);
	}

	return 0;
}

int aspeed_i2c_slave_irq(i2c_t *obj)
{
	uint32_t cmd = 0;
	int ret = 0;
	int i = 0;
	aspeed_device_t *device = obj->device;	
	uint32_t sts = 0;
	uint8_t byte_data = 0;
	int slave_rx_len = 0;
	int slave_rx_idx = 0;

	if((obj->mode == DMA_MODE) || (obj->mode == BUFF_MODE)) {
		if(!(readl(device->base + AST_I2CS_ISR) & AST_I2CS_PKT_DONE))
			return 0;
	} 

	sts = readl(device->base + AST_I2CS_ISR);
	if (!sts) return 0;

	sts &= ~AST_I2CS_ADDR_INDICAT_MASK;

	if (AST_I2CS_ADDR1_NAK & sts)
		sts &= ~AST_I2CS_ADDR1_NAK;

	if (AST_I2CS_ADDR2_NAK & sts)
		sts &= ~AST_I2CS_ADDR2_NAK;

	if (AST_I2CS_ADDR3_NAK & sts)
		sts &= ~AST_I2CS_ADDR3_NAK;

	if (AST_I2CS_ADDR_MASK & sts)
		sts &= ~AST_I2CS_ADDR_MASK;

	if (AST_I2CS_PKT_DONE & sts) {
		sts &= ~(AST_I2CS_PKT_DONE | AST_I2CS_PKT_ERROR);
		switch (sts) {
			case AST_I2CS_SLAVE_MATCH | AST_I2CS_STOP:
				I2CS_DBUG("S : Sw | P \n");
				cmd = AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_MODE_EN;
				if (obj->mode == DMA_MODE) {
					cmd |= AST_I2CS_RX_DMA_EN;
					writel(AST_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_BUFF_SIZE), device->base + AST_I2CS_DMA_LEN);
					//dma addr xxxx
				} else if (obj->mode == BUFF_MODE) {
					cmd |= AST_I2CS_RX_BUFF_EN;
					writel(AST_I2CC_SET_RX_BUF_LEN(obj->buf_size), device->base + AST_I2CC_BUFF_CTRL);
				} else {
					cmd &= ~AST_I2CS_PKT_MODE_EN;
				}
				writel(cmd, device->base + AST_I2CS_CMD_STS);
				break;
			case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE | AST_I2CS_Wait_RX_DMA:
			case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE | AST_I2CS_STOP:
			case AST_I2CS_RX_DONE | AST_I2CS_STOP:				
				if(sts & AST_I2CS_STOP) {
					if(sts & AST_I2CS_SLAVE_MATCH)
						I2CS_DBUG("S : Sw|D|P \n");
					else
						I2CS_DBUG("S : D|P \n");
				} else
					I2CS_DBUG("S : Sw|D \n");
				
				cmd = AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_MODE_EN;
				if (obj->mode == DMA_MODE) {
					cmd |= AST_I2CS_RX_DMA_EN;
					slave_rx_len = AST_I2C_GET_RX_DMA_LEN(readl(device->base + AST_I2CS_DMA_LEN_STS));
#ifdef CONFIG_I2C_MQUEUE_SLAVE
//					I2CS_DBUG("idx : %d \n ", obj->fifo_rx_idx);
#if 0				
					printf("%d - ", slave_rx_len);
					for (i = 0; i < slave_rx_len; i++) {
						printf("[%02x]", obj->slave_mq[obj->fifo_rx_idx].data[slave_rx_idx + i]);
					}
					printf("\n");
#endif
#endif
					obj->slave_mq[obj->fifo_rx_idx].length += slave_rx_len;
					if(sts & AST_I2CS_STOP) {
						if(((obj->fifo_rx_idx + 1) % I2C_SLAVE_MQ_FIFO_SIZE) == (obj->fifo_fetch_idx)) {
							printf("rx fifo full \n");
							obj->fifo_full = 1;
							cmd = AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_MODE_EN | AST_I2CS_AUTO_NAK_EN;
							writel(0, device->base + AST_I2CS_IER);
						} else {
							obj->fifo_rx_idx++;
							obj->fifo_rx_idx %= I2C_SLAVE_MQ_FIFO_SIZE;
//							writel((uint32_t) obj->slave_mq[obj->fifo_rx_idx].data | 0x80000000, device->base + AST_I2CS_RX_DMA); AST1030
							writel(TO_PHY_ADDR(obj->slave_mq[obj->fifo_rx_idx].data), device->base + AST_I2CS_RX_DMA);
						}
					}
					writel(AST_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_BUFF_SIZE), device->base + AST_I2CS_DMA_LEN);
				} else if (obj->mode == BUFF_MODE) {
					cmd |= AST_I2CS_RX_BUFF_EN;
					slave_rx_len = AST_I2CC_GET_RX_BUF_LEN(readl(device->base + AST_I2CC_BUFF_CTRL));
#ifdef CONFIG_I2C_MQUEUE_SLAVE
					slave_rx_idx = obj->slave_mq[obj->fifo_rx_idx].length;
#if 1
					printf("%d - ", slave_rx_len);
					for (i = 0; i < slave_rx_len; i++) {
						obj->slave_mq[obj->fifo_rx_idx].data[slave_rx_idx + i] = readb(obj->buf_base + i);
						printf("[%x]", obj->slave_mq[obj->fifo_rx_idx].data[slave_rx_idx + i]);
					}
					printf("\n");
#endif
					obj->slave_mq[obj->fifo_rx_idx].length += slave_rx_len;
					if(sts & AST_I2CS_STOP) {
						obj->fifo_rx_idx++;
						obj->fifo_rx_idx %= I2C_SLAVE_MQ_FIFO_SIZE;
					}
					slave_rx_len = AST_I2CC_GET_RX_BUF_LEN(readl(device->base + AST_I2CC_BUFF_CTRL));
#endif
					writel(AST_I2CC_SET_RX_BUF_LEN(obj->buf_size), device->base + AST_I2CC_BUFF_CTRL);
				} else {
					//first address match is address 
					cmd &= ~AST_I2CS_PKT_MODE_EN;
					slave_rx_idx = obj->slave_mq[obj->fifo_rx_idx].length;
					byte_data = AST_I2CC_GET_RX_BUFF(readl(device->base + AST_I2CC_STS_AND_BUFF));
					obj->slave_mq[obj->fifo_rx_idx].data[slave_rx_idx] = byte_data;
					I2CS_DBUG("rx [%x]", byte_data);
					obj->slave_mq[obj->fifo_rx_idx].length++;
				}
				writel(cmd, device->base + AST_I2CS_CMD_STS);
				break;
			//it is Mw data Mr coming -> it need send tx
			case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE | AST_I2CS_Wait_TX_DMA:
				//it should be repeat start read 
				I2CS_DBUG("S: AST_I2CS_Wait_TX_DMA | AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE\n");
				printf("not go to here \n");
				cmd = AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_MODE_EN;
				if (obj->mode == DMA_MODE) {
					cmd |= AST_I2CS_TX_DMA_EN;
					slave_rx_len = AST_I2C_GET_RX_DMA_LEN(readl(device->base + AST_I2CS_DMA_LEN_STS));
#ifdef CONFIG_I2C_MQUEUE_SLAVE	
					printf("not go to here \n");
					slave_rx_idx = obj->slave_mq[obj->fifo_rx_idx].length;
					for (i = 0; i < slave_rx_len; i++) {
						I2CS_DBUG("[%x]", obj->slave_mq[obj->fifo_rx_idx].data[slave_rx_idx + i]);
					}
					obj->slave_mq[obj->fifo_rx_idx].length += slave_rx_len;
					obj->fifo_rx_idx++;
					obj->fifo_rx_idx %= I2C_SLAVE_MQ_FIFO_SIZE;					
#endif
					writel(AST_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_BUFF_SIZE), device->base + AST_I2CS_DMA_LEN);
				} else if (obj->mode == BUFF_MODE) {
					cmd |= AST_I2CS_TX_BUFF_EN;
					slave_rx_len = AST_I2CC_GET_RX_BUF_LEN(readl(device->base + AST_I2CC_BUFF_CTRL));
#ifdef CONFIG_I2C_MQUEUE_SLAVE
					slave_rx_idx = obj->slave_mq[obj->fifo_rx_idx].length;
					for (i = 0; i < slave_rx_len; i++) {
						obj->slave_mq[obj->fifo_rx_idx].data[slave_rx_idx + i] = readb(obj->buf_base + i);
						I2CS_DBUG("[%x]", obj->slave_mq[obj->fifo_rx_idx].data[slave_rx_idx + i]);
					}
					obj->slave_mq[obj->fifo_rx_idx].length += slave_rx_len;
					obj->fifo_rx_idx++;
					obj->fifo_rx_idx %= I2C_SLAVE_MQ_FIFO_SIZE;
#endif
					writel(AST_I2CC_SET_TX_BUF_LEN(obj->buf_size), device->base + AST_I2CC_BUFF_CTRL);
				} else {
					cmd &= ~AST_I2CS_PKT_MODE_EN;
					cmd |= AST_I2CS_TX_CMD;
					byte_data = AST_I2CC_GET_RX_BUFF(readl(device->base + AST_I2CC_STS_AND_BUFF));
					I2CS_DBUG("rx : [%02x]", byte_data);
					I2CS_DBUG("tx : [%02x]", byte_data);
					writel(byte_data, device->base + AST_I2CC_STS_AND_BUFF);
				}
				writel(cmd, device->base + AST_I2CS_CMD_STS);

				break;
			case AST_I2CS_RX_DONE | AST_I2CS_Wait_TX_DMA:
				//it should be repeat start read 
				I2CS_DBUG("S: AST_I2CS_Wait_TX_DMA | AST_I2CS_RX_DONE\n");
				printf("not go to here \n");
#if 0
				cmd = AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_MODE_EN;
				if (obj->mode == DMA_MODE) {
					cmd |= AST_I2CS_TX_DMA_EN;
					slave_rx_len = AST_I2C_GET_RX_DMA_LEN(readl(device->base + AST_I2CS_DMA_LEN_STS));
					i2c_slave_event(obj, I2C_SLAVE_WRITE_REQUESTED, &value);
					for (i = 0; i < slave_rx_len; i++) {
						I2CS_DBUG("[%02x]", obj->slave_rx_buf[i]);
						i2c_slave_event(obj, I2C_SLAVE_WRITE_RECEIVED);
					}
					i2c_slave_event(obj, I2C_SLAVE_READ_REQUESTED);
					I2CS_DBUG("tx : [%02x]", obj->slave_tx_buf[0]);
					writel(0, device->base + AST_I2CS_DMA_LEN_STS);				
					writel(AST_I2CS_SET_TX_DMA_LEN(1), device->base + AST_I2CS_DMA_LEN);
				} else if (obj->mode == BUFF_MODE) {
					cmd |= AST_I2CS_TX_BUFF_EN;
					slave_rx_len = AST_I2CC_GET_RX_BUF_LEN(readl(device->base + AST_I2CC_BUFF_CTRL));
					i2c_slave_event(obj, I2C_SLAVE_WRITE_REQUESTED);
					for (i = 0; i < slave_rx_len; i++) {
						value = readb(obj->buf_base + i);
						I2CS_DBUG("rx : [%02x]", value);
						i2c_slave_event(obj, I2C_SLAVE_WRITE_RECEIVED);
					}
					i2c_slave_event(obj, I2C_SLAVE_READ_REQUESTED);
					I2CS_DBUG("TODO tx : [%02x]", value);
					writeb(value, obj->buf_base);					
					writel(AST_I2CC_SET_TX_BUF_LEN(1), device->base + AST_I2CC_BUFF_CTRL);
				} else {
					cmd &= ~AST_I2CS_PKT_MODE_EN;
					cmd |= AST_I2CS_TX_CMD;
					byte_data = AST_I2CC_GET_RX_BUFF(readl(device->base + AST_I2CC_STS_AND_BUFF));
					I2CS_DBUG("rx : [%02x]", byte_data);
					i2c_slave_event(obj, I2C_SLAVE_WRITE_REQUESTED);
					i2c_slave_event(obj, I2C_SLAVE_WRITE_RECEIVED);
					i2c_slave_event(obj, I2C_SLAVE_READ_REQUESTED);
					I2CS_DBUG("tx : [%02x]", byte_data);
					writel(byte_data, device->base + AST_I2CC_STS_AND_BUFF);
				}
				writel(cmd, AST_I2CS_CMD_STS);
#endif
				break;
			case AST_I2CS_SLAVE_MATCH | AST_I2CS_Wait_TX_DMA:
				//First Start read
				I2CS_DBUG("S: AST_I2CS_SLAVE_MATCH | AST_I2CS_Wait_TX_DMA\n");
				printf("not go to here \n");
#if 0
				cmd = AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_MODE_EN;
				if (obj->mode == DMA_MODE) {
					cmd |= AST_I2CS_TX_DMA_EN;
					i2c_slave_event(obj, I2C_SLAVE_READ_REQUESTED);
					I2CS_DBUG("ssif tx len: [%x]\n", obj->slave_tx_buf[0]);
					for( i = 1; i < obj->slave_tx_buf[0] + 1; i++) {
						i2c_slave_event(obj, I2C_SLAVE_READ_PROCESSED);
					}
					writel(AST_I2CS_SET_TX_DMA_LEN(obj->slave_tx_buf[0]), device->base + AST_I2CS_DMA_LEN);
				} else if (obj->mode == BUFF_MODE) {
					cmd |= AST_I2CS_TX_BUFF_EN;
					i2c_slave_event(obj, I2C_SLAVE_READ_REQUESTED);
					I2CS_DBUG("TODO ssif tx len: [%x]\n", byte_data);
					writeb(byte_data, obj->buf_base);
					for (i = 1; i < byte_data + 1; i++) {
						i2c_slave_event(obj, I2C_SLAVE_READ_PROCESSED);
						writeb(value, obj->buf_base + i);
					}
					writel(AST_I2CC_SET_TX_BUF_LEN(byte_data), device->base + AST_I2CC_BUFF_CTRL);
				} else {
					cmd &= ~AST_I2CS_PKT_MODE_EN;
					cmd |= AST_I2CS_TX_CMD;
					i2c_slave_event(obj, I2C_SLAVE_READ_REQUESTED);
					writel(byte_data, device->base + AST_I2CC_STS_AND_BUFF);
				}
				writel(cmd, AST_I2CS_CMD_STS);			
#endif
				break;	
			case AST_I2CS_Wait_TX_DMA:
				//it should be next start read 
				I2CS_DBUG("S: AST_I2CS_Wait_TX_DMA \n");
				printf("not go to here \n");
#if 0
				cmd = AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_MODE_EN;
				if (obj->mode == DMA_MODE) {
					cmd |= AST_I2CS_TX_DMA_EN;
					i2c_slave_event(obj, I2C_SLAVE_READ_PROCESSED, &obj->slave_tx_buf[0]);
					I2CS_DBUG("tx : [%02x]", obj->slave_tx_buf[0]);
					writel(0, AST_I2CS_DMA_LEN_STS);
					writel(AST_I2CS_SET_TX_DMA_LEN(1), device->base + AST_I2CS_DMA_LEN);
				} else if (obj->mode == BUFF_MODE) {	
					cmd |= AST_I2CS_TX_BUFF_EN;
					i2c_slave_event(obj, I2C_SLAVE_READ_PROCESSED);
					I2CS_DBUG("TODO tx: [%02x]\n", value);
					writeb(value, obj->buf_base);
					writel(AST_I2CC_SET_TX_BUF_LEN(1), device->base + AST_I2CC_BUFF_CTRL);
				} else {
					cmd &= ~AST_I2CS_PKT_MODE_EN;
					cmd |= AST_I2CS_TX_CMD;
					i2c_slave_event(obj, I2C_SLAVE_READ_PROCESSED);
					I2CS_DBUG("tx: [%02x]\n", byte_data);
					writel(byte_data, device->base + AST_I2CC_STS_AND_BUFF);					
				}
				writel(cmd, AST_I2CS_CMD_STS);
#endif
				break;
			case AST_I2CS_TX_NAK | AST_I2CS_STOP:
				//it just tx complete
				I2CS_DBUG("S: AST_I2CS_TX_NAK | AST_I2CS_STOP \n");
				printf("not go to here \n");
#if 0
				cmd = AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_MODE_EN;
				i2c_slave_event(obj, I2C_SLAVE_STOP);
				if (obj->mode == DMA_MODE) {
					cmd |= AST_I2CS_RX_DMA_EN;
					writel(0, device->base + AST_I2CS_DMA_LEN_STS);
					writel(AST_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_MSG_BUF_SIZE), device->base + AST_I2CS_DMA_LEN);
				} else if (obj->mode == BUFF_MODE) {
					cmd |= AST_I2CS_RX_BUFF_EN;
					writel(AST_I2CC_SET_RX_BUF_LEN(obj->buf_size), device->base + AST_I2CC_BUFF_CTRL);
				} else {
					cmd &= ~AST_I2CS_PKT_MODE_EN;
				}
				writel(cmd, device->base + AST_I2CS_CMD_STS);
#endif
				break;
#if 0
			case AST_I2CS_SLAVE_MATCH | AST_I2CS_Wait_RX_DMA:
				I2CS_DBUG("S: AST_I2CS_SLAVE_MATCH | AST_I2CS_Wait_RX_DMA \n");
				i2c_bus->slave_event = I2C_SLAVE_START_WRITE;
				slave_rx_len = AST_I2C_GET_RX_DMA_LEN(readl(device->base + AST_I2CS_DMA_LEN_STS));
				I2CS_DBUG("slave_rx_len : %d", slave_rx_len);
				i2c_slave_event(obj, I2C_SLAVE_WRITE_REQUESTED, &value);
				for (i = 0; i < slave_rx_len; i++) {
					I2CS_DBUG("[%x]", i2c_bus->slave_dma_buf[i]);
					i2c_slave_event(obj, I2C_SLAVE_WRITE_RECEIVED, &i2c_bus->slave_dma_buf[i]);
				}
				writel(AST_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_MSG_BUF_SIZE), device->base + AST_I2CS_DMA_LEN);
				writel(readl(device->base + AST_I2CS_CMD_STS) | AST_I2CS_RX_DMA_EN, device->base + AST_I2CS_CMD_STS);
				break;
			// it is repeat start write coming
			case AST_I2CS_RX_DONE | AST_I2CS_Wait_TX_DMA:
				I2CS_DBUG("S: AST_I2CS_RX_DONE | AST_I2CS_Wait_TX_DMA TODO\n");
				slave_rx_len = AST_I2C_GET_RX_DMA_LEN(readl(device->base + AST_I2CS_DMA_LEN_STS));
				I2CS_DBUG("S: rx len %d \n", slave_rx_len);
				for (i = 0; i < slave_rx_len; i++) {
					I2CS_DBUG("[%x]", i2c_bus->slave_dma_buf[i]);
					i2c_slave_event(obj, I2C_SLAVE_WRITE_RECEIVED, &i2c_bus->slave_dma_buf[i]);
				}
				writel(AST_I2CS_SET_TX_DMA_LEN(1), device->base + AST_I2CS_DMA_LEN);
				writel(readl(device->base + AST_I2CS_CMD_STS) | AST_I2CS_RX_DMA_EN | AST_I2CS_TX_BUFF_EN, device->base + AST_I2CS_CMD_STS);
				break;
#endif
			default:
				printf("TODO slave sts case %x\n", readl(device->base + AST_I2CS_ISR));
				break;
		}
		writel(AST_I2CS_PKT_DONE, device->base + AST_I2CS_ISR);
		ret = 1;
	} else {
		printf("byte mode todo check \n");
		//only coming for byte mode 
		cmd = AST_I2CS_ACTIVE_ALL;
		switch (sts) {
			case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE | AST_I2CS_Wait_RX_DMA:
				I2CS_DBUG("S : Sw|D \n");
				//first address match is address 
				byte_data = AST_I2CC_GET_RX_BUFF(readl(device->base + AST_I2CC_STS_AND_BUFF));
				I2CS_DBUG("addr [%x]", byte_data);
				break;
			case AST_I2CS_RX_DONE | AST_I2CS_Wait_RX_DMA:
				I2CS_DBUG("S : D \n");
				byte_data = AST_I2CC_GET_RX_BUFF(readl(device->base + AST_I2CC_STS_AND_BUFF));
				I2CS_DBUG("rx [%x]", byte_data);

				break;
			case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE | AST_I2CS_Wait_TX_DMA:
				cmd |= AST_I2CS_TX_CMD;
				I2CS_DBUG("S : Sr|D \n");
				byte_data = AST_I2CC_GET_RX_BUFF(readl(device->base + AST_I2CC_STS_AND_BUFF));
				I2CS_DBUG("addr : [%02x]", byte_data);
//				i2c_slave_event(obj, I2C_SLAVE_READ_REQUESTED);
				I2CS_DBUG("tx: [%02x]\n", byte_data);
				writel(byte_data, AST_I2CC_STS_AND_BUFF);
				break;
			case AST_I2CS_TX_ACK | AST_I2CS_Wait_TX_DMA:
				cmd |= AST_I2CS_TX_CMD;
				I2CS_DBUG("S : D \n");
//				i2c_slave_event(obj, I2C_SLAVE_READ_PROCESSED);
				I2CS_DBUG("tx: [%02x]\n", byte_data);
				writel(byte_data, AST_I2CC_STS_AND_BUFF);
				break;
			case AST_I2CS_STOP:
			case AST_I2CS_STOP | AST_I2CS_TX_NAK:
				I2CS_DBUG("S : P \n");
//				i2c_slave_event(obj, I2C_SLAVE_STOP);
				break;
			default:
				printf("TODO no pkt_done intr ~~~ ***** sts %x \n", sts);
				break;
		}
		writel(cmd, device->base + AST_I2CS_CMD_STS);
		writel(sts, device->base + AST_I2CS_ISR);
		ret = 1;
	}

	return ret;
}

int i2c_slave_mqueue_read(i2c_t *obj, uint8_t *data)
{
	aspeed_device_t *device = obj->device;
	int length;
	int i = 0;

	aspeed_i2c_slave_irq(obj);

	if(obj->fifo_rx_idx != obj->fifo_fetch_idx) {
		length = obj->slave_mq[obj->fifo_fetch_idx].length;
		I2CS_DBUG("fifo_rx_idx %d, fifo_fetch_idx %d len %d\n", obj->fifo_rx_idx, obj->fifo_fetch_idx, length);
		for(i = 0; i < length; i++) {
			data[i] =obj->slave_mq[obj->fifo_fetch_idx].data[i];
		}
		obj->slave_mq[obj->fifo_fetch_idx].length = 0;
		obj->fifo_fetch_idx++;
		obj->fifo_fetch_idx %= I2C_SLAVE_MQ_FIFO_SIZE;
		if(obj->fifo_full) {
			obj->fifo_rx_idx++;
			obj->fifo_rx_idx %= I2C_SLAVE_MQ_FIFO_SIZE;
//			writel((uint32_t) obj->slave_mq[obj->fifo_rx_idx].data | 0x80000000, device->base + AST_I2CS_RX_DMA);
			writel(TO_PHY_ADDR(obj->slave_mq[obj->fifo_rx_idx].data), device->base + AST_I2CS_RX_DMA);
			writel(AST_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_BUFF_SIZE), device->base + AST_I2CS_DMA_LEN);
			obj->fifo_full = 0;
			writel(AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_MODE_EN | AST_I2CS_RX_DMA_EN, device->base + AST_I2CS_CMD_STS);
			writel(AST_I2CS_PKT_DONE, device->base + AST_I2CS_IER);
		}
		return length;
	} else
		return 0;

	return 0;
}

void i2c_slave_mode(i2c_t *obj, int enable_slave)
{
	aspeed_device_t *device = obj->device;

	if(enable_slave) {
//		printf("slave enable : isr %x isr %x \n", readl(device->base + AST_I2CM_ISR), readl(device->base + AST_I2CS_ISR));
		uint32_t cmd = AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_MODE_EN;

		//trigger rx buffer
		if(obj->mode == DMA_MODE) {
			cmd |= AST_I2CS_RX_DMA_EN;
#ifdef CONFIG_I2C_MQUEUE_SLAVE
			obj->fifo_fetch_idx = 0;
			obj->fifo_rx_idx = 0;
			writel(TO_PHY_ADDR(obj->slave_mq[0].data), device->base + AST_I2CS_RX_DMA);
//			writel((uint32_t) obj->slave_mq[0].data | 0x80000000, device->base + AST_I2CS_RX_DMA); AST1030
			writel(AST_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_BUFF_SIZE), device->base + AST_I2CS_DMA_LEN);
#endif
		} else if (obj->mode == BUFF_MODE) {
			cmd |= AST_I2CS_RX_BUFF_EN;
#ifdef CONFIG_I2C_MQUEUE_SLAVE
			obj->fifo_fetch_idx = 0;
			obj->fifo_rx_idx = 0;
			writel(AST_I2CC_SET_RX_BUF_LEN(obj->buf_size), device->base + AST_I2CC_BUFF_CTRL);
#endif
		} else {
			cmd &= ~AST_I2CS_PKT_MODE_EN;
		}

		writel(AST_I2CS_AUTO_NAK_EN, device->base + AST_I2CS_CMD_STS);
		writel(AST_I2CC_SLAVE_EN | readl(device->base + AST_I2CC_FUN_CTRL), device->base + AST_I2CC_FUN_CTRL);
		writel(cmd, device->base + AST_I2CS_CMD_STS);
	} else
		writel(~AST_I2CC_SLAVE_EN & readl(device->base + AST_I2CC_FUN_CTRL), device->base + AST_I2CC_FUN_CTRL);

	return;
}

void i2c_slave_address(i2c_t *obj, int idx, uint8_t address)
{
	uint32_t mask_addr;
	aspeed_device_t *device = obj->device;
	/* Set slave addr. */
	switch(idx) {
		case 0:
			mask_addr = readl(device->base + AST_I2CS_ADDR_CTRL) & ~AST_I2CS_ADDR1_MASK;
			writel(AST_I2CS_ADDR1(address) | mask_addr, device->base + AST_I2CS_ADDR_CTRL);
			break;
		case 1:
			mask_addr = readl(device->base + AST_I2CS_ADDR_CTRL) & ~AST_I2CS_ADDR2_MASK;
			writel(AST_I2CS_ADDR2(address) | mask_addr, device->base + AST_I2CS_ADDR_CTRL);
			break;
		case 2:
			mask_addr = readl(device->base + AST_I2CS_ADDR_CTRL) & ~AST_I2CS_ADDR3_MASK;
			writel(AST_I2CS_ADDR3(address) | mask_addr, device->base + AST_I2CS_ADDR_CTRL);
			break;			
	}
}

/* new i2c snoop mode */
void i2c_snoop_mode(i2c_t *obj, int enable)
{
	aspeed_device_t *device = obj->device;
	obj->snoop_buf = pvPortMallocNc(I2C_SNOOP_BUFF_SIZE);

	if(enable) {
		writel(TO_PHY_ADDR(obj->snoop_buf), device->base + AST_I2CS_RX_DMA);
		writel(AST_I2CS_SET_RX_DMA_LEN(I2C_SNOOP_BUFF_SIZE), device->base + AST_I2CS_DMA_LEN);
		writel(AST_I2CS_SNOOP_EN, device->base + AST_I2CS_CMD_STS);
	} else
		writel(~AST_I2CS_SNOOP_EN & readl(device->base + AST_I2CS_CMD_STS), device->base + AST_I2CS_CMD_STS);

	return;
}

void i2c_snoop_read(i2c_t *obj)
{
	aspeed_device_t *device = obj->device;
	uint8_t *buff = obj->snoop_buf;

	if(readl(device->base + AST_I2CS_SNOOP_DMA_RPT) != readl(device->base + AST_I2CS_SNOOP_DMA_WPT)) {
		printf("%x \n", buff[readl(device->base + AST_I2CS_SNOOP_DMA_RPT)]);
		writel(readl(device->base + AST_I2CS_SNOOP_DMA_RPT) + 1, device->base + AST_I2CS_SNOOP_DMA_RPT);
	}
}

void i2c_mailbox_address(i2c_t *obj, int idx, uint8_t address)
{
	uint32_t mask_addr;
	aspeed_device_t *device = obj->device;
	/* Set slave addr. */
	switch(idx) {
		case 0:
			mask_addr = readl(device->base + AST_I2CS_ADDR_CTRL) & ~AST_I2CS_ADDR1_MASK;
			writel(AST_I2CS_ADDR1(address) | mask_addr | AST_I2CS_ADDR1_MBX_TYPE(1), device->base + AST_I2CS_ADDR_CTRL);
			break;
		case 1:
			mask_addr = readl(device->base + AST_I2CS_ADDR_CTRL) & ~AST_I2CS_ADDR2_MASK;
			writel(AST_I2CS_ADDR2(address) | mask_addr | AST_I2CS_ADDR2_MBX_TYPE(1), device->base + AST_I2CS_ADDR_CTRL);
			break;
		case 2:
			mask_addr = readl(device->base + AST_I2CS_ADDR_CTRL) & ~AST_I2CS_ADDR3_MASK;
			writel(AST_I2CS_ADDR3(address) | mask_addr | AST_I2CS_ADDR3_MBX_TYPE(1), device->base + AST_I2CS_ADDR_CTRL);
			break;			
	}
}

void i2c_mailbox_mode(i2c_t *obj, int enable_slave)
{
	aspeed_device_t *device = obj->device;

	if(enable_slave) {
		writel(AST_I2CC_SLAVE_MAILBOX_EN | AST_I2CC_SLAVE_MAILBOX_SRAM_EN |
				readl(device->base + AST_I2CC_FUN_CTRL), device->base + AST_I2CC_FUN_CTRL);
	} else
		writel(~AST_I2CC_SLAVE_MAILBOX_EN & readl(device->base + AST_I2CC_FUN_CTRL), device->base + AST_I2CC_FUN_CTRL);

	return;
}

/* only support 5 i2c device */
void i2c_white_list(i2c_t *obj, int idx, uint32_t *white_list_table)
{
	int i = 0;
	uint32_t white_list_base = obj->global_reg + 0x20000;
	/* Set slave addr. */
	for(i = 0; i < 64; i++)
		writel(white_list_table[i], white_list_base + 0x200 + (idx * 0x100));
}

static void aspeed_i2c_irq_handler(i2c_t *obj)
{
	aspeed_device_t *device = obj->device;

#ifndef USE_OS_FLAG_FOR_WAIT
	if((obj->mode == DMA_MODE) || (obj->mode == BUFF_MODE)) {
		if(!(readl(device->base + AST_I2CM_ISR) & AST_I2CM_PKT_DONE))
			return;
	} else
		printf("byte mode check \n");
#endif

	if(readl(device->base + AST_I2CC_FUN_CTRL) & AST_I2CC_SLAVE_EN) {
		if(aspeed_i2c_slave_irq(obj)) {
			I2CS_DBUG("slave handle \n");
			return;
		}
	}

	aspeed_i2c_master_irq(obj);
}

#ifdef USE_OS_FLAG_FOR_WAIT
void aspeed_i2c0_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c0_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}

void aspeed_i2c1_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c1_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}

void aspeed_i2c2_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c2_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}

void aspeed_i2c3_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c3_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}

void aspeed_i2c4_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c4_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}

void aspeed_i2c5_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c5_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}

void aspeed_i2c6_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c6_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}

void aspeed_i2c7_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c7_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}

void aspeed_i2c8_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c8_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}

void aspeed_i2c9_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c9_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}

void aspeed_i2c10_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c10_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}

void aspeed_i2c11_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c11_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}

void aspeed_i2c12_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c12_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}

void aspeed_i2c13_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c13_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}

void aspeed_i2c14_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c14_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}

void aspeed_i2c15_isr(void)
{
	struct i2c_s *obj;
	uint32_t ret;
	obj = (struct i2c_s *)aspeed_irq_get_isr_context(I2c15_IRQn);

	aspeed_i2c_irq_handler(obj);

	ret = osEventFlagsSet(obj->evt_id, 0x00000001U);
	if (ret < 0)
		log_error("set Flag fail: %d\n", ret);
}
#endif

void i2c_reset(i2c_t *obj) {
	aspeed_device_t *device = obj->device;	
	uint32_t ctrl = readl(device->base + AST_I2CC_FUN_CTRL);

	writel(ctrl & ~(AST_I2CC_MASTER_EN | AST_I2CC_SLAVE_EN),
				device->base + AST_I2CC_FUN_CTRL);
	writel(ctrl, device->base + AST_I2CC_FUN_CTRL);
}

int i2c_dma_read(i2c_t *obj, int address, uint8_t *data, int length, uint16_t flags)
{
	int	xfer_len;
	aspeed_device_t *device = obj->device;	
	uint32_t cmd = AST_I2CM_PKT_EN | AST_I2CM_PKT_ADDR(address) | AST_I2CM_START_CMD | AST_I2CM_RX_CMD | AST_I2CM_RX_DMA_EN;

	I2C_DBUG("i2c-dma [%x] reading %d byte flags[%x] addr:0x%02x\n", device->base, length, flags, address);

	if(flags & I2C_M_RECV_LEN) {
		I2C_DBUG("smbus read \n");
		xfer_len = 1;
	} else {
		if (length > ASPEED_I2C_DMA_SIZE) {
			xfer_len = ASPEED_I2C_DMA_SIZE;
		} else {
			xfer_len = length;
			if(flags & I2C_M_STOP) {
				I2C_DBUG("| stop \n");
				cmd |= AST_I2CM_RX_CMD_LAST | AST_I2CM_STOP_CMD;
			}
		}
	}

	writel(AST_I2CM_SET_RX_DMA_LEN(xfer_len - 1), device->base + AST_I2CM_DMA_LEN);
	//writel((uint32_t)data | 0x80000000, device->base + AST_I2CM_RX_DMA); AST1030
	writel(TO_PHY_ADDR(data), device->base + AST_I2CM_RX_DMA);
	I2C_DBUG("trigger isr %x cmd %x dma %x xfer_len %d \n", readl(device->base + AST_I2CM_ISR), cmd, data, xfer_len);

	obj->flags = flags;
	obj->buf = data;
	obj->len = length;

	obj->master_xfer_cnt = 0;
	obj->cmd_err = 0;
	obj->xfer_complete = 0;

	writel(cmd, device->base + AST_I2CM_CMD_STS);

#ifdef USE_OS_FLAG_FOR_WAIT
	uint32_t ret;
	ret = osEventFlagsWait(obj->evt_id, 0x00000001U, osFlagsWaitAny, CONFIG_DEVICE_I2C_TIMEOUT);
	if (ret != 0x1)
	{
		log_error("osError: %d\n", ret);
		return 1;
	}
#else
	while(!obj->xfer_complete) {
		aspeed_i2c_irq_handler(obj);
	}
#endif

	I2C_DBUG("done \n");

	if(obj->cmd_err)
		return 1;
	else
		return 0;

}

int i2c_buff_read(i2c_t *obj, int address, uint8_t *data, int length, uint16_t flags)
{
	int i = 0;
	int	xfer_len;
	aspeed_device_t *device = obj->device;	
	uint32_t cmd = AST_I2CM_PKT_EN | AST_I2CM_PKT_ADDR(address) | AST_I2CM_START_CMD | AST_I2CM_RX_CMD | AST_I2CM_RX_BUFF_EN;

	I2C_DBUG("i2c-buff [%x] reading %d byte flags[%x] addr:0x%02x\n", device->base, length, flags, address);

	if(flags & I2C_M_RECV_LEN) {
		I2C_DBUG("smbus read \n");
		xfer_len = 1;
	} else {
		if (length > obj->buf_size) {
			xfer_len = obj->buf_size;
		} else {
			xfer_len = length;
			if(flags & I2C_M_STOP) {
				I2C_DBUG("| stop \n");
				cmd |= AST_I2CM_RX_CMD_LAST | AST_I2CM_STOP_CMD;
			}
		}
	}
	writel(AST_I2CC_SET_RX_BUF_LEN(xfer_len), device->base + AST_I2CC_BUFF_CTRL);
	for(i = 0; i < xfer_len; i++)
		obj->buf_base[i] = data[i];
	
	I2C_DBUG("trigger isr %x cmd %x dma %x \n", readl(device->base + AST_I2CM_ISR), cmd, data);

	obj->flags = flags;
	obj->buf = data;
	obj->len = length;

	obj->master_xfer_cnt = 0;
	obj->cmd_err = 0;
	obj->xfer_complete = 0;

	writel(cmd, device->base + AST_I2CM_CMD_STS);

#ifdef USE_OS_FLAG_FOR_WAIT
	uint32_t ret;
	ret = osEventFlagsWait(obj->evt_id, 0x00000001U, osFlagsWaitAny, CONFIG_DEVICE_I2C_TIMEOUT);
	if (ret != 0x1)
	{
		log_error("osError: %d\n", ret);
		return 1;
	}
#else
	while(!obj->xfer_complete) {
		aspeed_i2c_irq_handler(obj);
	}
#endif

	I2C_DBUG("done \n");

	if(obj->cmd_err)
		return 1;
	else
		return 0;

}

int i2c_byte_read(i2c_t *obj, int address, uint8_t *data, int length, uint16_t flags)
{
	aspeed_device_t *device = obj->device;	
	uint32_t cmd = AST_I2CM_PKT_EN | AST_I2CM_PKT_ADDR(address) | AST_I2CM_START_CMD | AST_I2CM_RX_CMD;

	I2C_DBUG("i2c-byte [%x] reading %d byte flags[%x] addr:0x%02x\n", device->base, length, flags, address);

	if(flags & I2C_M_RECV_LEN) {
		I2C_DBUG("smbus read \n");
	} else {
		if(flags & I2C_M_STOP) {
			I2C_DBUG("| stop \n");
			cmd |= AST_I2CM_RX_CMD_LAST | AST_I2CM_STOP_CMD;
		} 
	}
	
	I2C_DBUG("trigger isr %x cmd %x dma %x \n", readl(device->base + AST_I2CM_ISR), cmd, data);

	obj->flags = flags;
	obj->buf = data;
	obj->len = length;

	obj->master_xfer_cnt = 0;
	obj->cmd_err = 0;
	obj->xfer_complete = 0;

	writel(cmd, device->base + AST_I2CM_CMD_STS);

#ifdef USE_OS_FLAG_FOR_WAIT
	uint32_t ret;
	ret = osEventFlagsWait(obj->evt_id, 0x00000001U, osFlagsWaitAny, CONFIG_DEVICE_I2C_TIMEOUT);
	if (ret != 0x1)
	{
		log_error("osError: %d\n", ret);
		return 1;
	}
#else
	while(!obj->xfer_complete) {
		aspeed_i2c_irq_handler(obj);
	}
#endif
	I2C_DBUG("done \n");

	if(obj->cmd_err)
		return 1;
	else
		return 0;

}

int i2c_read(i2c_t *obj, int address, uint8_t *data, int length, uint16_t flags)
{
	if(obj->mode == DMA_MODE) {
		return i2c_dma_read(obj, address, data, length, flags);
	} else if (obj->mode == BUFF_MODE) {
		return i2c_buff_read(obj, address, data, length, flags);
	} else {
		return i2c_byte_read(obj, address, data, length, flags);
	}

}

int i2c_dma_write(i2c_t *obj, int address, uint8_t *data, int length, uint16_t flags)
{
	int	xfer_len = 0;
	aspeed_device_t *device = obj->device;

	uint32_t cmd = AST_I2CM_PKT_EN | AST_I2CM_PKT_ADDR(address) | AST_I2CM_START_CMD;

	//send start
	I2C_DBUG("i2c-dma [%x] writing %d byte flags[%x] addr:0x%02x\n", device->base, length, flags, address);

	//dma mode
	if(length > ASPEED_I2C_DMA_SIZE)
		xfer_len = ASPEED_I2C_DMA_SIZE;
	else {
		xfer_len = length;
		if(flags & I2C_M_STOP) {
			I2C_DBUG("| stop \n");
			cmd |= AST_I2CM_STOP_CMD;
		}
	}

	if(xfer_len) {
		cmd |= AST_I2CM_TX_DMA_EN | AST_I2CM_TX_CMD;
		writel(AST_I2CM_SET_TX_DMA_LEN(xfer_len - 1), device->base + AST_I2CM_DMA_LEN);
		//writel((uint32_t)data | 0x80000000, device->base + AST_I2CM_TX_DMA); ast1030
		writel(TO_PHY_ADDR(data), device->base + AST_I2CM_TX_DMA);
	}

	I2C_DBUG("trigger isr %x cmd %x \n", readl(device->base + AST_I2CM_ISR), cmd);

	obj->flags = flags;
	obj->buf = data;
	obj->len = length;

	obj->master_xfer_cnt = 0;
	obj->cmd_err = 0;
	obj->xfer_complete = 0;

	writel(cmd, device->base + AST_I2CM_CMD_STS);

#ifdef USE_OS_FLAG_FOR_WAIT
	uint32_t ret;
	ret = osEventFlagsWait(obj->evt_id, 0x00000001U, osFlagsWaitAny, CONFIG_DEVICE_I2C_TIMEOUT);
	if (ret != 0x1)
	{
		log_error("osError: %d\n", ret);
		return 1;
	}
#else
	while(!obj->xfer_complete) {
		aspeed_i2c_irq_handler(obj);
	}
#endif

	I2C_DBUG("xfer done [%d]\n", obj->cmd_err);

	if(obj->cmd_err)
		return 1;
	else
		return 0;

}

int i2c_buff_write(i2c_t *obj, int address, uint8_t *data, int length, uint16_t flags)
{
	int i = 0;
	uint8_t wbuf[4];
	int	xfer_len;
	aspeed_device_t *device = obj->device;	
	uint32_t cmd = AST_I2CM_PKT_EN | AST_I2CM_PKT_ADDR(address) | AST_I2CM_START_CMD;

	//send start
	I2C_DBUG("i2c-buff [%x] writing %d byte flags[%x] addr:0x%02x\n", device->base, length, flags, address);

	if (length > obj->buf_size) {
		xfer_len = obj->buf_size;
	} else {
		if(flags & I2C_M_STOP) {
			I2C_DBUG("| stop \n");
			cmd |= AST_I2CM_STOP_CMD;
		}
		xfer_len = length;
	}
	
	if(xfer_len) {
		cmd |= AST_I2CM_TX_BUFF_EN | AST_I2CM_TX_CMD;
		writel(AST_I2CC_SET_TX_BUF_LEN(xfer_len), device->base + AST_I2CC_BUFF_CTRL);
		for(i = 0; i < xfer_len; i++) {
			wbuf[i % 4] = data[i];
			if (i % 4 == 3)
				writel(*(uint32_t *)wbuf,
					   obj->buf_base + i - 3);
			I2C_DBUG("[%02x] \n", data[i]);
		}
		if (--i % 4 != 3)
			writel(*(uint32_t *)wbuf,
				   obj->buf_base + i - (i % 4));
	}

	I2C_DBUG("trigger isr %x cmd %x \n", readl(device->base + AST_I2CM_ISR), cmd);

	obj->flags = flags;
	obj->buf = data;
	obj->len = length;

	obj->master_xfer_cnt = 0;
	obj->cmd_err = 0;
	obj->xfer_complete = 0;

	writel(cmd, device->base + AST_I2CM_CMD_STS);

#ifdef USE_OS_FLAG_FOR_WAIT
	uint32_t ret;
	ret = osEventFlagsWait(obj->evt_id, 0x00000001U, osFlagsWaitAny, CONFIG_DEVICE_I2C_TIMEOUT);
	if (ret != 0x1)
	{
		log_error("osError: %d\n", ret);
		return 1;
	}
#else
	while(!obj->xfer_complete) {
		aspeed_i2c_irq_handler(obj);
	}
#endif
	I2C_DBUG("xfer done \n");

	if(obj->cmd_err)
		return 1;
	else
		return 0;
}

int i2c_byte_write(i2c_t *obj, int address, uint8_t *data, int length, uint16_t flags)
{
	aspeed_device_t *device = obj->device;	
	uint32_t cmd = AST_I2CM_PKT_EN | AST_I2CM_PKT_ADDR(address) | AST_I2CM_START_CMD;

		//byte mode
		if(flags & I2C_M_STOP) {
			I2C_DBUG("with stop \n");
			cmd |= AST_I2CM_STOP_CMD;
		}

	if(length) {
		cmd |= AST_I2CM_TX_CMD;
		writel(data[0], device->base + AST_I2CC_STS_AND_BUFF);
	}

	I2C_DBUG("trigger isr %x cmd %x \n", readl(device->base + AST_I2CM_ISR), cmd);

	obj->flags = flags;
	obj->buf = data;
	obj->len = length;

	obj->master_xfer_cnt = 0;
	obj->cmd_err = 0;
	obj->xfer_complete = 0;

	writel(cmd, device->base + AST_I2CM_CMD_STS);

#ifdef USE_OS_FLAG_FOR_WAIT
	uint32_t ret;
	ret = osEventFlagsWait(obj->evt_id, 0x00000001U, osFlagsWaitAny, CONFIG_DEVICE_I2C_TIMEOUT);
	if (ret != 0x1)
	{
		log_error("osError: %d\n", ret);
		return 1;
	}
#else
	while(!obj->xfer_complete) {
		aspeed_i2c_irq_handler(obj);
	}
#endif
	I2C_DBUG("done \n");

	if(obj->cmd_err)
		return 1;
	else
		return 0;

}

/* return 0 : pass, 1: fail */
int i2c_write(i2c_t *obj, int address, uint8_t *data, int length, uint16_t flags)
{
	//send start
	if(obj->mode == DMA_MODE) {
		return i2c_dma_write(obj, address, data, length, flags);
	} else if (obj->mode == BUFF_MODE) {
		return i2c_buff_write(obj, address, data, length, flags);
	} else {
		return i2c_byte_write(obj, address, data, length, flags);
	}
	
	return 1;
}

static uint32_t aspeed_select_i2c_clock(struct i2c_s *obj)
{
	int i;
	uint32_t data;
	int div = 0;
	int divider_ratio = 0;
	uint32_t clk_div_reg;
	int inc = 0;
	unsigned long base_clk1, base_clk2, base_clk3, base_clk4;
	uint32_t scl_low, scl_high;
	
	if(obj->clk_div_mode) {
		clk_div_reg = readl(obj->global_reg + ASPEED_I2CG_CLK_DIV_CTRL);
		base_clk1 = obj->apb_clk / (((clk_div_reg & 0xff) + 2) / 2);
		base_clk2 = obj->apb_clk / ((((clk_div_reg >> 8) & 0xff) + 2) / 2);
		base_clk3 = obj->apb_clk / ((((clk_div_reg >> 16) & 0xff) + 2) / 2);
		base_clk4 = obj->apb_clk / ((((clk_div_reg >> 24) & 0xff) + 2) / 2);
//		printf("base_clk1 %ld, base_clk2 %ld, base_clk3 %ld, base_clk4 %ld \n", base_clk1, base_clk2, base_clk3, base_clk4);
		if((obj->apb_clk / obj->bus_frequency) <= 32) {
			div = 0;
			divider_ratio = obj->apb_clk / obj->bus_frequency;
		} else if ((base_clk1 / obj->bus_frequency) <= 32) {
			div = 1;
			divider_ratio = base_clk1 / obj->bus_frequency;
		} else if ((base_clk2 / obj->bus_frequency) <= 32) {
			div = 2;
			divider_ratio = base_clk2 / obj->bus_frequency;
		} else if ((base_clk3 / obj->bus_frequency) <= 32) {
			div = 3;
			divider_ratio = base_clk3 / obj->bus_frequency;
		} else {
			div = 4;
			divider_ratio = base_clk4 / obj->bus_frequency;
			inc = 0;
			while((divider_ratio + inc) > 32) {
				inc |= divider_ratio & 0x1;
				divider_ratio >>= 1;
				div++;
			}
			divider_ratio += inc;
		}
		div &= 0xf;
		scl_low = ((divider_ratio >> 1) - 1) & 0xf;
		scl_high = (divider_ratio - scl_low - 2) & 0xf;
		/* Divisor : Base Clock : tCKHighMin : tCK High : tCK Low  */		
		data = (scl_high << 20) | (scl_high << 16) | (scl_low << 12) | (div);
	} else {
		for (i = 0; i < ARRAY_SIZE(aspeed_old_i2c_timing_table); i++) {
			if ((obj->apb_clk / aspeed_old_i2c_timing_table[i].divisor) <
			    obj->bus_frequency) {
				break;
			}
		}
		i++;
		data = aspeed_old_i2c_timing_table[i].timing;
//		printf("divisor [%d], timing [%x] \n", aspeed_old_i2c_timing_table[i].divisor, aspeed_old_i2c_timing_table[i].timing);
	} 

	return data;
}

void i2c_frequency(i2c_t *obj, int hz)
{
	aspeed_device_t *device = obj->device;
	/* Set AC Timing */

	writel(aspeed_select_i2c_clock(obj), device->base + AST_I2CC_AC_TIMING);
}

uint8_t aspeed_i2c_recover_bus(struct i2c_s *obj)
{
	uint32_t ctrl, state;
	uint32_t ret;
	
	aspeed_device_t *device = obj->device;

	if(!(readl(device->base + AST_I2CC_STS_AND_BUFF) & AST_I2CC_BUS_BUSY_STS))
		return 0;
	
	printf("i2c_recover_bus [%x] \n", readl(device->base + AST_I2CC_STS_AND_BUFF));

	ctrl = readl(device->base + AST_I2CC_FUN_CTRL);

	writel(ctrl & ~(AST_I2CC_MASTER_EN | AST_I2CC_SLAVE_EN), device->base + AST_I2CC_FUN_CTRL);

	writel(readl(device->base + AST_I2CC_FUN_CTRL) | AST_I2CC_MASTER_EN, device->base + AST_I2CC_FUN_CTRL);

	//Check 0x14's SDA and SCL status
	state = readl(device->base + AST_I2CC_STS_AND_BUFF);
	if (!(state & AST_I2CC_SDA_LINE_STS) && (state & AST_I2CC_SCL_LINE_STS)) {
		writel(AST_I2CM_RECOVER_CMD_EN, AST_I2CM_CMD_STS);
		ret = osEventFlagsWait(obj->evt_id, 0x00000001U, osFlagsWaitAny, CONFIG_DEVICE_I2C_TIMEOUT);
		if (ret != 0x1)
		{
			log_error("osError: %d\n", ret);
			return 1;
		}
		if (obj->bus_recover) {
			printf("recovery error \n");
			ret = 1;
		}
	} else {
		printf("can't recovery this situation\n");
		ret = 1;
	}
	printf("Recovery done [%x]\n", readl(device->base + AST_I2CC_STS_AND_BUFF));

	return ret;
}

void aspeed_i2c_bus_reset(struct i2c_s *obj)
{
	aspeed_device_t *device = obj->device;

	uint32_t i2c_crtl = readl(device->base + AST_I2CC_FUN_CTRL);
	//I2C Reset
	writel(0, device->base + AST_I2CC_FUN_CTRL);	
	writel(i2c_crtl, device->base + AST_I2CC_FUN_CTRL);

}

static void aspeed_i2c_bus_init(struct i2c_s *obj)
{
	int i = 0;
	aspeed_device_t *device = obj->device;
	aspeed_i2c_priv_t *priv = (aspeed_i2c_priv_t *)device->private;

	uint32_t fun_ctrl = AST_I2CC_BUS_AUTO_RELEASE | AST_I2CC_MASTER_EN;
	obj->global_reg = device->base & 0xfffff000;

	//ast1030 fpga pclk 12Mhz, ast2600 use pclk
#ifdef CONFIG_AST2600_SERIES
	obj->apb_clk = aspeed_clk_get_apb2();
#else
	obj->apb_clk = aspeed_clk_get_pclk();
#endif
	obj->bus_frequency = priv->bus_clk;
	log_debug("priv->bus_clk %d, priv->irq %d \n", priv->bus_clk, priv->irq);

	//I2C Reset
	writel(0, device->base + AST_I2CC_FUN_CTRL);

	//get global control register
	if (readl(obj->global_reg + ASPEED_I2CG_CTRL) & ASPEED_I2CG_CTRL_NEW_CLK_DIV)
		obj->clk_div_mode = 1;

	if(!obj->multi_master)
		fun_ctrl |= AST_I2CC_MULTI_MASTER_DIS;

	/* Enable Master Mode */
	writel(fun_ctrl, device->base + AST_I2CC_FUN_CTRL);

	/* Set AC Timing  TODO */
	writel(aspeed_select_i2c_clock(obj), device->base + AST_I2CC_AC_TIMING);

	//Clear Interrupt
	writel(0xfffffff, device->base + AST_I2CM_ISR);

	/* Set interrupt generation of I2C master controller */
	writel(AST_I2CM_PKT_DONE | AST_I2CM_BUS_RECOVER |
					AST_I2CM_SMBUS_ALT, 
					device->base + AST_I2CM_IER);

	writel(0xfffffff, device->base + AST_I2CM_ISR);

#ifdef CONFIG_I2C_MQUEUE_SLAVE
	obj->fifo_fetch_idx = 0;
	obj->fifo_rx_idx = 0;
	for(i = 0; i < I2C_SLAVE_MQ_FIFO_SIZE; i++)
		obj->slave_mq[i].length = 0;
#endif

	if (obj->mode == BYTE_MODE)
		writel(0xffff, device->base + AST_I2CS_IER);
	else {
		/* Set interrupt generation of I2C slave controller */
		writel(AST_I2CS_PKT_DONE, device->base + AST_I2CS_IER);
	}

#ifdef USE_OS_FLAG_FOR_WAIT
	/* init event ID for ISR */
	obj->evt_id = osEventFlagsNew(NULL);
	if (obj->evt_id == NULL)
		log_error("fail to create evt_id\n");

	switch(device->dev_id) {
		case ASPEED_DEV_I2C0:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c0_isr, obj);
			break;
		case ASPEED_DEV_I2C1:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c1_isr, obj);
			break;
		case ASPEED_DEV_I2C2:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c2_isr, obj);
			break;
		case ASPEED_DEV_I2C3:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c3_isr, obj);
			break;
		case ASPEED_DEV_I2C4:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c4_isr, obj);
			break;
		case ASPEED_DEV_I2C5:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c5_isr, obj);
			break;
		case ASPEED_DEV_I2C6:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c6_isr, obj);
			break;
		case ASPEED_DEV_I2C7:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c7_isr, obj);
			break;
		case ASPEED_DEV_I2C8:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c8_isr, obj);
			break;
		case ASPEED_DEV_I2C9:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c9_isr, obj);
			break;
		case ASPEED_DEV_I2C10:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c10_isr, obj);
			break;
		case ASPEED_DEV_I2C11:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c11_isr, obj);
			break;
		case ASPEED_DEV_I2C12:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c12_isr, obj);
			break;
		case ASPEED_DEV_I2C13:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c13_isr, obj);
			break;
		case ASPEED_DEV_I2C14:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c14_isr, obj);
			break;
		case ASPEED_DEV_I2C15:
			aspeed_irq_register(priv->irq, (uint32_t)aspeed_i2c15_isr, obj);
			break;
		default:
			printf("irq error \n");
			break;
	}
	
#endif

}

hal_status_t i2c_init(i2c_t *obj)
{
	aspeed_device_t *i2c = obj->device;
	aspeed_i2c_priv_t *priv = (aspeed_i2c_priv_t *)i2c->private;
	aspeed_device_t *parent = (aspeed_device_t *)priv->parent;
	obj->buf_base = (uint8_t *)priv->buff_addr;
	obj->buf_size = 0x20;
	
	uint32_t i2c_idx = DEV_ID_TO_I2C_INDEX(i2c->dev_id);

	log_debug("i2c bus %x buf base %x \n", obj->buf_base);
	if (i2c->init) {
		printf("I2C%d is occupied\n", i2c_idx + 1);
		return HAL_BUSY;
	}

	i2c_global_init(parent);
	aspeed_i2c_bus_init(obj);

	i2c->init = 1;

	return HAL_OK;
}
