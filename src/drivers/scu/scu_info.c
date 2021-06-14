/* SoC mapping Table */
#define SOC_ID(str, rev) { .name = str, .rev_id = rev, }

struct soc_id {
        const char *name;
        u64 rev_id;
};

static struct soc_id soc_map_table[] = {
        SOC_ID("AST2600-A0", 0x0500030305000303),
        SOC_ID("AST2600-A1", 0x0501030305010303),
        SOC_ID("AST2620-A1", 0x0501020305010203),
        SOC_ID("AST2600-A2", 0x0502030305010303),
        SOC_ID("AST2620-A2", 0x0502020305010203),
        SOC_ID("AST2605-A2", 0x0502010305010103),
        SOC_ID("AST2600-A3", 0x0503030305030303),
        SOC_ID("AST2620-A3", 0x0503020305030203),
        SOC_ID("AST2605-A3", 0x0503010305030103),
};

void aspeed_print_soc_id(void)
{
        int i;
        u64 rev_id;

        rev_id = readl(ASPEED_REVISION_ID0);
        rev_id = ((u64)readl(ASPEED_REVISION_ID1) << 32) | rev_id;

        for (i = 0; i < ARRAY_SIZE(soc_map_table); i++) {
                if (rev_id == soc_map_table[i].rev_id)
                        break;
        }
        if (i == ARRAY_SIZE(soc_map_table))
                printf("UnKnow-SOC: %llx\n", rev_id);
        else
                printf("SOC: %4s \n",soc_map_table[i].name);
}


