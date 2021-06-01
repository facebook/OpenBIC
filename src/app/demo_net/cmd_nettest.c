/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "cmsis_os.h"
#include "objects.h"
#include "peci_aspeed.h"
#include "log.h"
#include "mdio_aspeed.h"
#include "mac_aspeed.h"
#include "cache_aspeed.h"
#include "clk_aspeed.h"
#include "phy.h"
#include "internal.h"
#include "getopt.h"

/* topologic */
#define NETDIAG_CTRL_LOOPBACK_EXT		0	/* PHY MDI loopback */
#define NETDIAG_CTRL_LOOPBACK_PHY		1	/* PHY PCS loopback */
#define NETDIAG_CTRL_LOOPBACK_MAC		2	/* MAC internal loopback */
#define NETDIAG_CTRL_LOOPBACK_MII		3	/* RMII/RGMII loopback */
#define NETDIAG_CTRL_LOOPBACK_OFF		4	/* loopback off, pure TX */

#define MODE_SCAN						0	/* timing scan */
#define MODE_MARGIN						1	/* margin check */


#define IS_INTERFACE_ARG_RMII(x)                                               \
	(strncmp((x), "rmii", strlen("rmii")) == 0)
#define IS_INTERFACE_ARG_RGMII(x)                                              \
	(strncmp((x), "rgmii", strlen("rgmii")) == 0)
#define IS_INTERFACE_ARG_RGMII_RXID(x)                                         \
	(strncmp((x), "rgmii-rxid", strlen("rgmii-rxid")) == 0)
#define IS_INTERFACE_ARG_RGMII_TXID(x)                                         \
	(strncmp((x), "rgmii-txid", strlen("rgmii-txid")) == 0)
#define IS_INTERFACE_ARG_RGMII_ID(x)                                           \
	(strncmp((x), "rgmii-id", strlen("rgmii-id")) == 0)

#define PKT_PER_TEST	4
#define PAYLOAD_LENGTH	60
static mac_txdes_t txdes[PKT_PER_TEST] NON_CACHED_BSS_ALIGN16;
static mac_rxdes_t rxdes[PKT_PER_TEST] NON_CACHED_BSS_ALIGN16;
static uint8_t pkt_bufs[PKT_PER_TEST * 2][0x600] NON_CACHED_BSS_ALIGN16;

//#define DUMP_TX_PACKET
//#define DUMP_RX_PACKET

typedef struct mac_adaptor_s {
	uint32_t nobjs;
	mac_t *objs[N_DEVICE];
} mac_adaptor_t;
static mac_adaptor_t mac_adaptor = { .nobjs = 0 };

void mac_cmd_register_obj(mac_t *obj)
{
	mac_adaptor.objs[mac_adaptor.nobjs++] = obj;
	if (mac_adaptor.nobjs > N_DEVICE)
		log_error("exceed MAX MAC objects\n");
}

#define ETH_SIZE_DA			6
#define ETH_SIZE_SA			6
#define ETH_SIZE_TYPE_LENG	2
#define ETH_SIZE_HEADER		(ETH_SIZE_DA + ETH_SIZE_SA + ETH_SIZE_TYPE_LENG)

#define ETH_OFFSET_DA		0
#define ETH_OFFSET_SA		(ETH_OFFSET_DA + ETH_SIZE_DA)

static void prepare_tx_packet(uint8_t *pkt, int length)
{
	int j;
	uint8_t *ptr = pkt;

	/* DA: broadcast */
	for (j = 0; j < ETH_SIZE_DA; j++)
		*ptr++ = 0xff;

	/* SA: skip */
	ptr += ETH_SIZE_SA;
	
	/* length */
	*ptr++ = (length >> 8) & 0xff;
	*ptr++ = length & 0xff;

	/* payload */
	for (j = 0; j < length - ETH_SIZE_HEADER; j++)
		*ptr++ = j;

#ifdef DUMP_TX_PACKET
	ptr = pkt;
	for (j = 0; j < length; j++) {
		printf("%02x ", *ptr++);
		if ((j & 0x7) == 0x7)
			printf("\n");
	}
#endif
}

static int nettest(mac_t *mac_obj, uint32_t speed, uint32_t control, int loop)
{
	int i, j, k;
	uint32_t rxlen;
	hal_status_t status;
	hal_status_t ret = HAL_OK;

	uint8_t *rx_pkt_buf[PKT_PER_TEST];
	uint8_t *tx_pkt_buf[PKT_PER_TEST];
	uint8_t *ptr;
	uint8_t mac_addr[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};

	for (i = 0; i < PKT_PER_TEST; i++) {
		tx_pkt_buf[i] = &pkt_bufs[i][0];
		rx_pkt_buf[i] = &pkt_bufs[PKT_PER_TEST + i][0];
	}
	
	log_debug("address of txdes: %08x\n", (uint32_t)&txdes[0]);
	log_debug("address of rxdes: %08x\n", (uint32_t)&rxdes[0]);

	mac_obj->txdes = txdes;
	mac_obj->rxdes = rxdes;
	mac_obj->n_txdes = PKT_PER_TEST;
	mac_obj->n_rxdes = PKT_PER_TEST;
	mac_obj->rx_pkt_buf = &rx_pkt_buf[0];
	mac_obj->mac_addr = mac_addr;
	
	aspeed_mac_init(mac_obj);
	if (control == NETDIAG_CTRL_LOOPBACK_MAC)
		aspeed_mac_set_loopback(mac_obj, 1);
	else
		aspeed_mac_set_loopback(mac_obj, 0);

	k = 0;
loop_start:
	/* prepare the first packet */
	prepare_tx_packet(&tx_pkt_buf[0][0], PAYLOAD_LENGTH);
	aspeed_mac_txpkt_add(mac_obj, tx_pkt_buf[0], PAYLOAD_LENGTH);

	for (i = 1; i < PKT_PER_TEST; i++) {
		memcpy(&tx_pkt_buf[i][0], &tx_pkt_buf[0][0], PAYLOAD_LENGTH);

		/* every tx packet has its own SA for identification */
		ptr = &tx_pkt_buf[i][ETH_OFFSET_SA];
		memset(ptr, i, ETH_SIZE_SA);
		aspeed_mac_txpkt_add(mac_obj, tx_pkt_buf[i], PAYLOAD_LENGTH);
	}
	__DSB();
	aspeed_mac_xmit(mac_obj);

	ret = HAL_OK;
	for (i = 0; i < PKT_PER_TEST; i++) {
		status = net_get_packet(mac_obj, (void **)&rx_pkt_buf[i], &rxlen, 10);

		if (status == HAL_OK) {
			log_debug("RX packet %d: length=%d addr=%08x\n", i, rxlen, (uint32_t)rx_pkt_buf[i]);
			/* examine if rx packets are in sequence by checking SA */
			j = memcmp(&rx_pkt_buf[i][ETH_OFFSET_SA], &tx_pkt_buf[i][ETH_OFFSET_SA], ETH_SIZE_SA);
			if (j) {
				log_error("\n\n");
				for (int k = 0; k < rxlen; k++)
					log_error("pkt#%02x[%02x] got:%02x expected:%02x\n", i, k, rx_pkt_buf[i][k], tx_pkt_buf[i][k]);
				ret = HAL_ERROR;
			}
		} else {
			log_debug("pkt#%02x: error: %d\n", i, status);
			ret = status;
		}
	}
	aspeed_mac_init_tx_desc(mac_obj);
	aspeed_mac_init_rx_desc(mac_obj);

	if ((++k < loop) && (ret == HAL_OK))
		goto loop_start;

	return ret;
}

static void config_clock(void)
{
#ifdef CONFIG_AST2600_SERIES
	writel(0x10004077, SCU_BASE + 0x240);
	writel(0x8000003b, SCU_BASE + 0x244);
	clrsetbits(SCU_BASE + 0x304, GENMASK(23, 16), 0x74 << 16);
	clrsetbits(SCU_BASE + 0x310, GENMASK(26, 24), 0 << 24);
	clrsetbits(SCU_BASE + 0x340, GENMASK(31, 28), 0x9 << 28);
#else
	clrsetbits(SCU_BASE + 0x310, GENMASK(26, 24), 1 << 24);
	clrsetbits(SCU_BASE + 0x310, GENMASK(19, 16), 9 << 16);
	/* bit[31]: 1=select HPLL as clock source
	 * bit[23:20]: 7=HPLL clock / 8 = 1000/8 = 125M
	 */
	clrsetbits(SCU_BASE + 0x310, BIT(31) | GENMASK(23,20), 7 << 20);
	/* bit[31]: 1=select internal clock source */
	setbits(SCU_BASE + 0x350, BIT(31));
#endif
}

static void print_xticks(int start, int end, int center)
{
	int i;
	char ytick[] = "    ";

	printf("%s", ytick);
	for (i = start; i <= end; i++)
		printf("%1x", (i >> 4) & 0xf);

	printf("\n%s", ytick);
	for (i = start; i <= end; i++)
		printf("%1x", i & 0xf);

	printf("\n%s", ytick);
	for (i = start; i <= end; i++) {
		printf("%c", (i == center) ? '|' : ' ');
	}
	printf("\n");
}

static void netdiag_cmd_handler(int argc, char *argv[])
{
	mac_t *obj = NULL;
	phy_t *phy;
	int i, j;
	int mac_index = 0, mdio_index = 0;
	int speed = 1000;
	int interface = PHY_INTERFACE_MODE_RGMII;
	int control = NETDIAG_CTRL_LOOPBACK_EXT;
	int mode = MODE_MARGIN;
	int margin = 2;
	int loop = 100;
	int tx_c, tx_s, tx_e, rx_c, rx_s, rx_e;
	int has_error = 0;
	char opt;
	char *data_ptrs[2];
	hal_status_t status;

	optind = 0;
	while ((opt = getopt(argc, argv, "o:i:l:s:m:k:h")) != (char)-1) {
		switch (opt) {
		case 'o':
			data_ptrs[0] = strtok(optarg, ",");
			data_ptrs[1] = strtok(NULL, ",");
			if (data_ptrs[0])
				mac_index = strtoul(data_ptrs[0], NULL, 16);
			else
				return;
			
			if (data_ptrs[1])
				mdio_index = strtoul(data_ptrs[1], NULL, 16);
			else
				mdio_index = mac_index;
			break;
		case 'i':
			if (IS_INTERFACE_ARG_RMII(optarg))
				interface = PHY_INTERFACE_MODE_RMII;
			else if (IS_INTERFACE_ARG_RGMII_RXID(optarg))
				interface = PHY_INTERFACE_MODE_RGMII_RXID;
			else if (IS_INTERFACE_ARG_RGMII_TXID(optarg))
				interface = PHY_INTERFACE_MODE_RGMII_TXID;
			else if (IS_INTERFACE_ARG_RGMII_ID(optarg))
				interface = PHY_INTERFACE_MODE_RGMII_ID;
			break;
		case 'l':
			if (strncmp(optarg, "phy", strlen("phy")) == 0)
				control = NETDIAG_CTRL_LOOPBACK_PHY;
			else if (strncmp(optarg, "mac", strlen("mac")) == 0)
				control = NETDIAG_CTRL_LOOPBACK_MAC;
			else if (strncmp(optarg, "mii", strlen("mii")) == 0)
				control = NETDIAG_CTRL_LOOPBACK_MII;
			else if (strncmp(optarg, "tx", strlen("tx")) == 0)
				control = NETDIAG_CTRL_LOOPBACK_OFF;
			break;
		case 's':
			speed = strtoul(optarg, NULL, 10);
			break;
		case 'm':
			data_ptrs[0] = strtok(optarg, ",");
			data_ptrs[1] = strtok(NULL, ",");
			if (data_ptrs[0] && (strncmp(data_ptrs[0], "scan", strlen("scan")) == 0))
				mode = MODE_SCAN;

			if (data_ptrs[1] && (mode == MODE_MARGIN))
				margin = strtoul(data_ptrs[1], NULL, 10);
			break;
		case 'k':
			loop = strtoul(optarg, NULL, 10);
			break;
		case 'h':
		default:
			printf("%s", netdiag_cmd.pcHelpString);
			return;
		}
	}

	if (mac_index >= mac_adaptor.nobjs)
		return;

	log_debug("args:\nmac: %d, mdio: %d\nspeed %d, control: %d, interface: %d, mode: %d, count: %d\n",
		   mac_index, mdio_index, speed, control, interface, mode, loop);
	obj = mac_adaptor.objs[mac_index];
	phy = obj->phy;
	net_connect_mdio(mac_index, mdio_index);

	/* configure phy */
	phy->speed = speed;
	phy->duplex = 1;
	phy->autoneg = 0;
	phy->phy_mode = interface;
	if (control == NETDIAG_CTRL_LOOPBACK_PHY)
		phy->loopback = PHY_LOOPBACK_INT;
	else if (control == NETDIAG_CTRL_LOOPBACK_EXT)
		phy->loopback = PHY_LOOPBACK_EXT;
	else
		phy->loopback = PHY_LOOPBACK_OFF;

	/* if mac loopback or mii loopback, no need to init phy */
	if ((control != NETDIAG_CTRL_LOOPBACK_MAC) && 
		(control != NETDIAG_CTRL_LOOPBACK_MII)) {
		phy_init(phy);
		net_enable_mdio_pin(obj->phy->mdio->device->dev_id - ASPEED_DEV_MDIO0);
	}

	/* configure mac */
	config_clock();
	obj->is_rgmii = (obj->phy->phy_mode == PHY_INTERFACE_MODE_RMII) ? 0 : 1;
	if (obj->is_rgmii)
		net_enable_rgmii_pin(obj->device->dev_id - ASPEED_DEV_MAC0);
	else
		net_enable_rmii_pin(obj->device->dev_id - ASPEED_DEV_MAC0);

	aspeed_clk_get_rgmii_delay(obj->device->dev_id, speed, (uint32_t *)&tx_c, (uint32_t *)&rx_c);
	if (mode == MODE_MARGIN) {
		tx_s = MAX(tx_c - margin, 0);
		tx_e = MIN(tx_c + margin, MAX_DELAY_TAPS_RGMII_TX);
		rx_s = MAX(rx_c - margin, 0);
		rx_e = MIN(rx_c + margin, MAX_DELAY_TAPS_RGMII_RX);
	} else {
		tx_s = 0;
		tx_e = MAX_DELAY_TAPS_RGMII_TX;
		rx_s = 0;
		rx_e = MAX_DELAY_TAPS_RGMII_RX;
	}
	log_debug("%d %d %d %d %d %d\n", tx_s, tx_c, tx_e, rx_s, rx_c, rx_e);

	print_xticks(rx_s, rx_e, rx_c);

	for (i = tx_s; i <= tx_e; i++) {
		printf("%02x:%c", i, (i == tx_c) ? '-' : ' ');
		for (j = rx_s; j <= rx_e; j++) {
			aspeed_clk_set_rgmii_delay(obj->device->dev_id, speed, i, j);
			status = nettest(obj, speed, control, loop);
			if (status == HAL_OK) {
				printf("o");
			} else if (status == HAL_ERROR) {
				printf("x");
				has_error = 1;
			} else {
				printf(".");
				has_error = 1;
			}
		}
		printf("\n");
	}

	if (has_error && (mode == MODE_MARGIN)) {
		printf("\nnetdiag FAIL: margin not enough\n");
	} else {
		printf("\nnetdiag PASS\n");
	}

	/* restore the delay setting */
	aspeed_clk_set_rgmii_delay(obj->device->dev_id, speed, tx_c, rx_c);
}
CLI_FUNC_DECL(netdiag, netdiag_cmd_handler);

const CLI_Command_Definition_t netdiag_cmd = {
    "netdiag",
    "\r\nnetdiag: network diagnostic tool.\r\n\
    -o <mac>,<mdio> | mac (mandatory):  index of the mac object\r\n\
                    | mdio (optional, default mdio = mac): index of the mdio object\r\n\
    -s <speed>      | speed (optional, default 1000)\r\n\
                    |   1000\r\n\
                    |   100\r\n\
                    |   10\r\n\
    -l <loopback>   | loopback (optional, default ext)\r\n\
                    |   ext = PHY MDI (PHY external) loopback\r\n\
                    |   phy = PHY PCS (PHY internal) loopback\r\n\
                    |   mii = RMII/RGMII loopback\r\n\
                    |   mac = MAC loopback\r\n\
                    |   tx  = TX only\r\n\
    -i <interface>  | interface (optional, default rgmii)\r\n\
                    |   rmii       = set interface as RMII\r\n\
                    |   rgmii      = set interface as RGMII w/o PHY TX & RX delay\r\n\
                    |   rgmii-id   = set interface as RGMII w/  PHY TX & RX delay\r\n\
                    |   rgmii-txid = set interface as RGMII w/  PHY TX delay\r\n\
                    |   rgmii-rxid = set interface as RGMII w/  PHY RX delay\r\n\
    -m <mode>,<taps>| mode (optional, default margin)\r\n\
                    |   margin     = check margin of the current delay setting\r\n\
                    |   scan       = scan full delay taps\r\n\
                    | taps: (optional, default 2): number of the delay taps to be\r\n\
                    |       checked in margin check mode\r\n\
    -k <count>      | count (optional, default 100)\r\n",
    CLI_FUNC_SYM(netdiag),
	-1
};