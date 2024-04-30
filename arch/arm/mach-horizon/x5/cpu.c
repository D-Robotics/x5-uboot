// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (C) 2023 VeriSilicon Holdings Co., Ltd.
 */

#include <common.h>
#include <asm/io.h>
#include <asm/armv8/mmu.h>
#include <asm/arch/hardware.h>

static struct mm_region x5_mem_map[] = {
	{
		.virt = PHYS_SDRAM_1,
		.phys = PHYS_SDRAM_1,
		.size = PHYS_SDRAM_1_SIZE,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			PTE_BLOCK_OUTER_SHARE
	},
#ifdef PHYS_SDRAM_2
	{
		.virt = PHYS_SDRAM_2,
		.phys = PHYS_SDRAM_2,
		.size = PHYS_SDRAM_2_SIZE,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			PTE_BLOCK_OUTER_SHARE
	},
#endif
	{
		.virt = X5_PERI_BASE,
		.phys = X5_PERI_BASE,
		.size = X5_PERI_SIZE,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			PTE_BLOCK_NON_SHARE
			| PTE_BLOCK_PXN | PTE_BLOCK_UXN
	},
	{
		/* List terminator */
		0,
	}
};

struct mm_region *mem_map = x5_mem_map;

int arch_cpu_init(void)
{
	/* Nothing to do */
	return 0;
}

/*
 * Perform the low-level reset.
 */
void reset_cpu(void)
{
	printf("do chip SW reset...\n");
}

int print_cpuinfo(void)
{
	return 0;
}
