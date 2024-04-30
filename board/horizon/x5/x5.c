// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 VeriSilicon Holdings Co., Ltd.
 */

#include <dm.h>
#include <common.h>
#include <command.h>
#include <malloc.h>
#include <asm/arch/hardware.h>
#include <asm/io.h>
#include <linux/delay.h>

int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_1_SIZE;
	return 0;
}

int board_early_init_f(void)
{
	return 0;
}

int dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;

#ifdef PHYS_SDRAM_2
	gd->bd->bi_dram[1].start = PHYS_SDRAM_2;
	gd->bd->bi_dram[1].size = PHYS_SDRAM_2_SIZE;
#endif
	return 0;
}

int board_init(void)
{
	return 0;
}
