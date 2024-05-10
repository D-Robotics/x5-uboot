// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright(C) 2024, D-Robotics Co., Ltd. All rights reserved
 */

#include <dm.h>
#include <common.h>
#include <command.h>
#include <malloc.h>
#include <asm/arch/hardware.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <asm/arch/hb_aon.h>
#include <asm/arch/hb_hsio.h>

/**
 * @brief Initialize all pin voltage here, the actual configuration
 * should be board specific.
 *
 * @param None
 * @retval None
 */
void init_io_vol(void)
{
	return;
}

int board_early_init_f(void)
{
	uint32_t value = 0;
	/* clear boot count */
	value = readl(AON_STATUS_REG0);
	value &= ~(AON_BOOTCOUNT_MASK << AON_BOOTCOUNT_OFFSET);
	writel(value, AON_STATUS_REG0);

	/* Reset eMMC Pinctrl to default */
	writel(EMMC_DAT_IOCTL_DEFAULT, EMMC_DAT0_3_IO_CFG_REG);
	writel(EMMC_DAT_IOCTL_DEFAULT, EMMC_DAT4_7_IO_CFG_REG);
	writel(EMMC_CTL_IOCTL_DEFAULT, EMMC_CTL_IO_CFG_REG);
	/* Reset eMMC DLL to default */
	writel(EMMC_DLL_DEFAULT, EMMC_DLL_MASTER_CTL);

	/* Reset SD Pinctrl to default */
	writel(SD_DAT_IOCTL_DEFAULT, SD_DAT_IO_CFG_REG);
	writel(SD_CTL_IOCTL_DEFAULT, SD_CTL_IO_CFG_REG);
	/* Reset SD DLL to default */
	writel(SD_DLL_DEFAULT, SD_DLL_MASTER_CTL);

	// set enet clk turn on
	value = readl(HSIO_SYS_REG_BASE + 0);
	value |= 0x8;
	writel(value, HSIO_SYS_REG_BASE + 0);
	return 0;
}

struct ddr_size {
	u32 info;
	u32 size; /* ddr size in GB */
};

static const struct ddr_size ddr_size_mapping[] = {
	{.info = 0, .size = 1},
	{.info = 1, .size = 2},
	{.info = 2, .size = 4},
	{.info = 3, .size = 8},
};

static u32 get_mapping_size(u32 info)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ddr_size_mapping); i++) {
		if (info == ddr_size_mapping[i].info) {
			return ddr_size_mapping[i].size;
		}
	}
	return 0;
}

static u64 get_ddr_size(void)
{
    int val;
    u64 ddr_size;

    val = readl(AON_STATUS_REG3);

    ddr_size = ((val & DDR_SIZE_BIT_MASK) >> DDR_SIZE_BIT_OFFSET);

    ddr_size = get_mapping_size(ddr_size);
    // printf("ddr_size: %llx GB\n", ddr_size);

    ddr_size = (ddr_size << 30);
    // printf("ddr_size: %llx\n", ddr_size);

    /* check ecc enabled */
    if(val & DDR_ECC_BIT_MASK) {
        ddr_size = (ddr_size - ddr_size / 8);
    }

    ddr_size = (ddr_size -  DDR_RESERVED_SIZE);
    return ddr_size;
}

int dram_init(void)
{
	gd->ram_size = get_ddr_size();
	return 0;
}

int dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = (gd->ram_size > PHYS_SDRAM_1_SIZE) ? PHYS_SDRAM_1_SIZE : gd->ram_size;
	debug("start:0x%llx, size:0x%llx\n",gd->bd->bi_dram[0].start,gd->bd->bi_dram[0].size);
#ifdef PHYS_SDRAM_2
	gd->bd->bi_dram[1].start = (gd->ram_size > PHYS_SDRAM_1_SIZE) ? PHYS_SDRAM_2 : 0;
	gd->bd->bi_dram[1].size = (gd->ram_size > PHYS_SDRAM_1_SIZE) ? (gd->ram_size - PHYS_SDRAM_1_SIZE): 0;
	debug("start:0x%llx, size:0x%llx\n",gd->bd->bi_dram[1].start,gd->bd->bi_dram[1].size);

#endif
	return 0;
}

int board_init(void)
{
	init_io_vol();
	return 0;
}

#ifdef X5_USABLE_RAM_TOP
ulong board_get_usable_ram_top(ulong total_size)
{
	return X5_USABLE_RAM_TOP;
}
#endif
