/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "objects.h"
#include "mdio_aspeed.h"
#include "log.h"
#include "internal.h"
#include "getopt.h"

typedef struct mdio_adaptor_s {
	uint32_t nobjs;
	mdio_t *objs[4];
} mdio_adaptor_t;
static mdio_adaptor_t mdio_adaptor = { .nobjs = 0 };

void mdio_cmd_register_obj(mdio_t *obj)
{
	mdio_adaptor.objs[mdio_adaptor.nobjs++] = obj;
	if (mdio_adaptor.nobjs > 4)
		log_error("exceed MAX MDIO objects\n");
}

static void mdio_cmd_handler(int argc, char *argv[])
{
	mdio_t *obj = NULL;
	int i;
	int phy_addr = 0, reg_addr = 0, length = 1, type = 0, data;
	char opt;
	char *data_ptrs[2];

	optind = 0;
	while ((opt = getopt(argc, argv, "o:a:r:w:h")) != (char)-1) {
		switch (opt) {
		case 'o':
			i = atoi(optarg);
			if (i >= mdio_adaptor.nobjs)
				return;
			obj = mdio_adaptor.objs[i];
			break;
		case 'a':
			data_ptrs[0] = strtok(optarg, ",");
			data_ptrs[1] = strtok(NULL, ",");
			phy_addr = strtoul(data_ptrs[0], NULL, 16);
			reg_addr = strtoul(data_ptrs[1], NULL, 16);
			break;
		case 'r':
			length = atoi(optarg);
			type = 0;
			break;
		case 'w':
			data = strtoul(optarg, NULL, 16);
			type = 1;
			break;
		case 'h':
		default:
			printf("%s", mdio_cmd.pcHelpString);
			return;
		}
	}

	if (obj == NULL) {
		printf("mdio object not assigned\n");
		return;
	}

	if (type == 1) {
		aspeed_mdio_write(obj, phy_addr, reg_addr, data);
	} else {
		for (i = 0; i < length; i++)
			printf("phy %02x reg %04x: %04x\n", phy_addr, reg_addr + i,
				   aspeed_mdio_read(obj, phy_addr, reg_addr + i));
	}
}

CLI_FUNC_DECL(mdio, mdio_cmd_handler);

const CLI_Command_Definition_t mdio_cmd = {
    "mdio",
    "\r\nmdio:\r\n mdio read/write command.\r\n\
	-h                     : help, show this message\r\n\
	-o <object index>      : assign the index of the mdio object\r\n\
	-a <phyaddr>,<regaddr> : assign phy and reg addresses\r\n\
	-r <length>            : number of registers to be read\r\n\
	-w <data>              : 16-bit write data\r\n",
    CLI_FUNC_SYM(mdio),
	-1
};