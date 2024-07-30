/*
 * fdt_setup.c --- Description
 *
 * Copyright (C) 2023, horizon, all rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <common.h>
#include <dm.h>
#include <errno.h>
#include <malloc.h>
#include <asm/sections.h>
#include <asm/io.h>
#include <linux/libfdt.h>
#include <fdt_support.h>
#include <mtd_node.h>
#include <jffs2/load_kernel.h>
#include <console.h>
#include <asm/arch/hb_efuse.h>
#ifndef CONFIG_SPL_BUILD
#include <env.h>
#endif
#include <linux/sizes.h>
#include <log.h>
#include <asm/arch/hb_strappin.h>

#ifdef CONFIG_CONSOLE_RECORD
static void membuff_setup(void *blob)
{
	int nodeoff;

	nodeoff = fdt_path_offset(blob, "/soc");
	if (nodeoff < 0) {
		pr_err("cannot find /soc\n");
		return;
	}
	nodeoff = fdt_add_subnode(blob, nodeoff, "membuff");
	if (nodeoff < 0) {
		pr_err("cannot create subnode\n");
		return;
	}

	fdt_setprop_string(blob, nodeoff,
			"compatible", "hobot,uboot-log");
	fdt_setprop_u64(blob, nodeoff,
	                "membuff-start", (uint64_t)gd->console_out.start);
	fdt_setprop_u64(blob, nodeoff,
	                "membuff-size",
	                (uint64_t)CONFIG_CONSOLE_RECORD_OUT_SIZE);
}
#endif

#if defined (CONFIG_OF_LIBFDT) && defined (CONFIG_OF_BOARD_SETUP)
__weak int hb_fdt_set_board_info(void *fdt_blob)
{
	debug(" %s is not implemented\n", __func__);
	return 0;
}

static void bonding_setup(void *blob)
{
	debug("to do implement\n");
}


static void fdt_set_status_by_env(void *fdt_blob)
{
	struct {
		const char *listname;
		enum fdt_status status;
        } fdt_status[] = {
		{ "fdt-blacklist", FDT_STATUS_DISABLED },
		{ "fdt-whitelist", FDT_STATUS_OKAY },
	};
	char *current, *temp;
	int i;

	for (i = 0; i < ARRAY_SIZE(fdt_status); i++) {
		const char *listname = env_get(fdt_status[i].listname);

		if (!listname)
			continue;

		temp = strdup(listname);
		current = temp;

		current = strtok(temp, ";");
		while (current) {
			fdt_set_status_by_pathf(
				fdt_blob, fdt_status[i].status, current);
			current = strtok(NULL, ";");
		}

		free(temp);
	}
}

static void check_cpu_1_8g_support(void *fdt)
{
        int offs, ret;
        int enable;
        int efuse;
        int i;
        char node[128] = {0};

        enable = env_get_yesno("enable_cpu_18g");

        /* enable opp table according to efuse info */
        ret = hb_read_efuse(EFUSE_CPU_OPPTABLE_OFFSET, 4, (char *)&efuse);
        if (ret) {
            printf("read efuse cpu type failed\n");
            return;
        }
        efuse &= EFUSE_CPU_OPPTABLE_MASK;
        efuse = (efuse >> EFUSE_CPU_OPPTABLE_BIT);

        /* update pll table if 1.8G supported */
        if (enable && efuse == 0) {
                /* enable corresponding opp table */
                memset(node, 0, sizeof(node));
                sprintf(node, "/cpu-opp-table-%d/", efuse);
                offs = fdt_path_offset(fdt, node);
                if (offs < 0) {
                        printf("failed to get sub_node!");
                        return;
                }

                ret = fdt_setprop_string(fdt, offs, "status", "okay");
                if (ret < 0) {
                        printf("failed to update cpu opp node status!");
                        return;
                }

                /* enable pll table for 1.8G support */
                offs = fdt_path_offset(fdt, "/soc/hps-clock-controller@34210000");
                if (offs < 0) {
                        printf("failed to get hps clock node!");
                        return;
                }
                ret = fdt_setprop_u32(fdt, offs, "pll-table", 1);
                if (ret < 0) {
                        printf("failed to update cpu opp node status!");
                        return;
                }
                printf("%s CPU 1.8G!\n", enable ? "enable" : "disable");
        } else {
                char opp_node[128] = {0};
                int opp_offs;
                u32 phandle;

                // TODO: remove hardcode after efuse burned
                efuse = 1;
                /* enable pll table for 1.8G support */
                opp_offs = fdt_path_offset(fdt, "/soc/hps-clock-controller@34210000");
                if (opp_offs < 0) {
                        printf("failed to get hps clock node!");
                        return;
                }
                ret = fdt_setprop_u32(fdt, opp_offs, "pll-table", 0);
                if (ret < 0) {
                        printf("failed to update cpu opp node status!");
                        return;
                }
                /* enable corresponding opp table */
                memset(opp_node, 0, sizeof(opp_node));
                sprintf(opp_node, "/cpu-opp-table-%d/", efuse);
                //printf("opp_node: %s\n", opp_node);

                opp_offs = fdt_path_offset(fdt, opp_node);
                if (opp_offs < 0) {
                        printf("failed to get opp_node!");
                        return;
                }

                ret = fdt_setprop_string(fdt, opp_offs, "status", "okay");
                if (ret < 0) {
                        printf("failed to update cpu opp node status!");
                        return;
                }

                /* create phandle */
                phandle = fdt_get_phandle(fdt, opp_offs);
                if (phandle == 0) {
                        int ret;

                        ret = fdt_generate_phandle(fdt, &phandle);
                        if (ret < 0) {
                                printf("Can't generate phandle: %s\n", fdt_strerror(ret));
                                return;
                        }

                        ret = fdt_set_phandle(fdt, opp_offs, phandle);
                        if (ret < 0) {
                                printf("Can't set phandle %u: %s\n", phandle, fdt_strerror(ret));
                                return;
                        }
                }
                /* update each cpu opp with new phandle */
                for (i = 0; i < MAX_CPU; i++) {
                        memset(node, 0, sizeof(node));
                        sprintf(node, "/cpus/cpu@%d", i);
                        //printf("node: %s\n", node);

                        offs = fdt_path_offset(fdt, node);
                        if (offs < 0) {
                                printf("failed to get cpu opp node!");
                                return;
                        }

                        ret = fdt_setprop_u32(fdt, offs, "operating-points-v2", phandle);
                        if (ret < 0) {
                                printf("failed to update cpu opp node status!");
                                return;
                        }
                }
        }
}

int hb_set_ion_cma_size(void *blob, u64 size, const char *name, u64 start, u64 range_size)
{
	int rsv_offset, nodeoffset;
	u32 dma_ranges[4];
	int ret;

	rsv_offset = fdt_find_or_add_subnode(blob, 0, "reserved-memory");
	if (rsv_offset < 0) {
		/*
		 * Not found or something else bad happened.
		 */
		log_err("%s: libfdt fdt_find_or_add_subnode() returned %s\n", __func__,
			fdt_strerror(rsv_offset));
		return 1;
	}
	nodeoffset = fdt_find_or_add_subnode(blob, rsv_offset, name);
	if (nodeoffset < 0) {
		/*
		 * Not found or something else bad happened.
		 */
		log_err("%s: libfdt fdt_find_or_add_subnode() returned %s\n", __func__,
			fdt_strerror(nodeoffset));
		return 1;
	}

	ret = fdt_setprop_u64(blob, nodeoffset, "size", size);
	if (ret < 0) {
		log_err("%s: setup ion_cma memory_size(): %s\n", __func__,
			fdt_strerror(ret));
		return 1;
	}

	if (size == 0) {
		ret = fdt_setprop_string(blob, nodeoffset, "status", "disabled");
		if (ret < 0) {
			log_err("%s: setup status failed: %s\n", __func__,
				fdt_strerror(ret));
			return 1;
		}
	}

	ret = fdt_setprop_string(blob, nodeoffset, "compatible", "ion-cma");
	if (ret < 0) {
		log_err("%s: setup compatible failed: %s\n", __func__,
			fdt_strerror(ret));
		return 1;
	}

	ret = fdt_setprop_u64(blob, nodeoffset, "alignment", 0x2000);
	if (ret < 0) {
		log_err("%s: setup alignment failed: %s\n", __func__,
			fdt_strerror(ret));
		return 1;
	}

	ret = fdt_setprop_empty(blob, nodeoffset, "reusable");
	if (ret < 0) {
		log_err("%s: setup reusable failed: %s\n", __func__,
			fdt_strerror(ret));
		return 1;
	}

	dma_ranges[0] =	cpu_to_fdt32((start >> 32) & 0xffffffff);
	dma_ranges[1] =	cpu_to_fdt32(start & 0xffffffff);
	dma_ranges[2] =	cpu_to_fdt32((size >> 32) & 0xffffffff);
	dma_ranges[3] =	cpu_to_fdt32(size & 0xffffffff);
	ret = fdt_setprop(blob, nodeoffset, "reg", &dma_ranges[0],
			sizeof(dma_ranges));
	if (ret < 0) {
		log_err("%s: setup reusable failed: %s\n", __func__,
			fdt_strerror(ret));
		return 1;
	}

	log_info("Set Reserve Mem [%s] Size to 0x%016llx @0x%llx\n", name, size, start);

	return 0;
}

int hb_set_ion_reserved_size(void *blob, u64 size, const char *name, u64 start, u64 range_size)
{
	int rsv_offset, nodeoffset;
	u32 dma_ranges[4];
	int ret;

	rsv_offset = fdt_find_or_add_subnode(blob, 0, "reserved-memory");
	if (rsv_offset < 0) {
		/*
		 * Not found or something else bad happened.
		 */
		log_err("%s: libfdt fdt_find_or_add_subnode() returned %s\n", __func__,
			fdt_strerror(rsv_offset));
		return 1;
	}

	nodeoffset = fdt_find_or_add_subnode(blob, rsv_offset, name);
	if (nodeoffset < 0) {
		/*
		 * Not found or something else bad happened.
		 */
		log_err("%s: libfdt fdt_find_or_add_subnode() returned %s\n", __func__,
			fdt_strerror(nodeoffset));
		return 1;
	}

	ret = fdt_setprop_u64(blob, nodeoffset, "size", size);
	if (ret < 0) {
		log_err("%s: setup ion_reserved memory_size(): %s\n", __func__,
			fdt_strerror(ret));
		return 1;
	}

	ret = fdt_setprop_string(blob, nodeoffset, "compatible", "ion-pool");
	if (ret < 0) {
		log_err("%s: setup compatible failed: %s\n", __func__,
			fdt_strerror(ret));
		return 1;
	}

	dma_ranges[0] =	cpu_to_fdt32((start >> 32) & 0xffffffff);
	dma_ranges[1] =	cpu_to_fdt32(start & 0xffffffff);
	dma_ranges[2] =	cpu_to_fdt32((size >> 32) & 0xffffffff);
	dma_ranges[3] =	cpu_to_fdt32(size & 0xffffffff);
	ret = fdt_setprop(blob, nodeoffset, "reg", &dma_ranges[0],
			sizeof(dma_ranges));
	if (ret < 0) {
		log_err("%s: setup reusable failed: %s\n", __func__,
			fdt_strerror(ret));
		return 1;
	}

	log_info("Set Reserved carveout Mem [%s] Size to 0x%016llx @0x%llx\n", name, size, start);

	return 0;
}

int hb_set_ion_carveout_size(void *blob, u64 size, const char *name, u64 start, u64 range_size)
{
	int rsv_offset, nodeoffset;
	u32 dma_ranges[4];
	int ret;

	rsv_offset = fdt_find_or_add_subnode(blob, 0, "reserved-memory");
	if (rsv_offset < 0) {
		/*
		 * Not found or something else bad happened.
		 */
		log_err("%s: libfdt fdt_find_or_add_subnode() returned %s\n", __func__,
			fdt_strerror(rsv_offset));
		return 1;
	}
	nodeoffset = fdt_find_or_add_subnode(blob, rsv_offset, name);
	if (nodeoffset < 0) {
		/*
		 * Not found or something else bad happened.
		 */
		log_err("%s: libfdt fdt_find_or_add_subnode() returned %s\n", __func__,
			fdt_strerror(nodeoffset));
		return 1;
	}

	ret = fdt_setprop_u64(blob, nodeoffset, "size", size);
	if (ret < 0) {
		log_err("%s: setup ion_cma memory_size(): %s\n", __func__,
			fdt_strerror(ret));
		return 1;
	}

	ret = fdt_setprop_string(blob, nodeoffset, "compatible", "ion-carveout");
	if (ret < 0) {
		log_err("%s: setup compatible failed: %s\n", __func__,
			fdt_strerror(ret));
		return 1;
	}

	dma_ranges[0] =	cpu_to_fdt32((start >> 32) & 0xffffffff);
	dma_ranges[1] =	cpu_to_fdt32(start & 0xffffffff);
	dma_ranges[2] =	cpu_to_fdt32((size >> 32) & 0xffffffff);
	dma_ranges[3] =	cpu_to_fdt32(size & 0xffffffff);
	ret = fdt_setprop(blob, nodeoffset, "reg", &dma_ranges[0],
			sizeof(dma_ranges));
	if (ret < 0) {
		log_err("%s: setup reusable failed: %s\n", __func__,
		fdt_strerror(ret));
		return 1;
	}

	log_info("Set Carveout Mem [%s] Size to 0x%016llx @0x%llx\n", name, size, start);

	return 0;
}

#define DDR_SIZE_2GB ((uint64_t)2 * SZ_1G)
#define ION_MIN_SIZE (0x4000000)  /* 64 MiB */

static uint64_t hb_ion_size_validate(uint64_t val, uint64_t max)
{
	/*
	 * The ion heap size should be ION_MIN_SIZE at least,
	 * also it should be the multiple of ION_MIN_SIZE.
	 */
	val = (val > 0) ? roundup(val, ION_MIN_SIZE) : ION_MIN_SIZE;

	if (val > max)
		val = max;

	return val;
}

static inline void hb_ion_set_region_size(uint64_t *ion_reserved_size, uint64_t ion_reserved_default,
							  uint64_t *ion_carveout_size, uint64_t ion_carveout_default,
							  uint64_t *ion_cma_size, uint64_t ion_cma_default, const uint64_t ddr_size)
{
	uint64_t ion_total = 0;
	bool printed = false;

	*ion_reserved_size = env_get_ulong("ion_reserved_size", 16, \
		ion_reserved_default);
	log_debug("Get ion reserved size, env:%#llx, default:%#llx\n",
			  *ion_reserved_size, ion_reserved_default);
	*ion_reserved_size = hb_ion_size_validate(*ion_reserved_size, \
		ion_reserved_default);

	*ion_carveout_size = env_get_ulong("ion_carveout_size", 16, \
		ion_carveout_default);
	log_debug("Get ion reserved size, env:%#llx, default:%#llx\n",
			  *ion_carveout_size, ion_carveout_default);
	*ion_carveout_size = hb_ion_size_validate(*ion_carveout_size, \
		ion_carveout_default);

	*ion_cma_size = env_get_ulong("ion_cma_size", 16, \
		ion_cma_default);
	log_debug("Get ion reserved size, env:%#llx, default:%#llx\n",
			  *ion_cma_size, ion_cma_default);
	*ion_cma_size = hb_ion_size_validate(*ion_cma_size, \
		ion_cma_default);

	do {
		ion_total = (*ion_reserved_size) + (*ion_carveout_size) + (*ion_cma_size);
		if ((ion_total + DEFAULT_ION_REGION_START + DEFAULT_KERNEL_MIN_HEAP) <= ((ddr_size))) {
			break;
		}
		*ion_reserved_size -= (100 * SZ_1M);
		*ion_carveout_size -= (100 * SZ_1M);
		*ion_cma_size -= (100 * SZ_1M);
		if (!printed) {
			pr_err("ION Total size exceeds DDR total size, shrinking!\n");
			printed = true;
		}
	} while (true);

	return;
}

static int hb_setup_ion_size(void *blob)
{
	uint64_t ddr_size = ((uint64_t)4 * SZ_1G);
	uint32_t num_banks = 0;
	uint64_t cma_size, reserved_size, carveout_size, offset_size; // in byte

#ifdef CONFIG_NR_DRAM_BANKS
	{
		int i;
		phys_size_t size = 0;

		for (i = 0; i < CONFIG_NR_DRAM_BANKS; ++i) {
			if (gd->bd->bi_dram[i].size) {
				size += gd->bd->bi_dram[i].size;
				num_banks++;
			}
		}

		/* X5 stores bi_dram[x].size with reserved_size removed */
		ddr_size = (size + DDR_RESERVED_SIZE);
	}
#endif
	/* TODO: Handle the case where ECC is enabled
	 * and ion regions must be shrinked.
	 */

	log_debug("%s: Get ddr_size:%lld\n", __func__, ddr_size);
	if (ddr_size <= DDR_SIZE_2GB) {
		hb_ion_set_region_size(&reserved_size, DEFAULT_ION_RESERVED_SIZE_LE_2G,
							   &carveout_size, DEFAULT_ION_CARVEOUT_SIZE_LE_2G,
							   &cma_size, DEFAULT_ION_CMA_SIZE_LE_2G, ddr_size);
	} else {
		hb_ion_set_region_size(&reserved_size, DEFAULT_ION_RESERVED_SIZE_GT_2G,
							   &carveout_size, DEFAULT_ION_CARVEOUT_SIZE_GT_2G,
							   &cma_size, DEFAULT_ION_CMA_SIZE_GT_2G, ddr_size);
	}

	offset_size = DEFAULT_ION_REGION_START;
	(void) hb_set_ion_reserved_size(blob, reserved_size, ION_RESERVED_NAME,
				gd->bd->bi_dram[0].start + offset_size,
				gd->bd->bi_dram[0].size - offset_size);
	offset_size = offset_size + reserved_size;
	(void) hb_set_ion_carveout_size(blob, carveout_size, ION_CARVEOUT_NAME,
				gd->bd->bi_dram[0].start + offset_size,
				gd->bd->bi_dram[0].size - offset_size);
	offset_size = offset_size + carveout_size;
	(void) hb_set_ion_cma_size(blob, cma_size, ION_CMA_NAME,
				gd->bd->bi_dram[0].start + offset_size,
				gd->bd->bi_dram[0].size - offset_size);

	return 0;
}

static unsigned int get_boot_src(void)
{
	unsigned int ustrap_pin_info = readl(BOOT_STRAP_PIN_REG);
        return ((ustrap_pin_info & BOOT_MODE_MASK) >> BOOT_MODE_SHIFT);
}

static void update_boot_mode(void *fdt)
{
	int offs, ret;
	unsigned int boot_src = get_boot_src();

        if(boot_src == BOOT_SRC_QSPI_NOR ||
		boot_src == BOOT_SRC_QSPI_NAND ) {
		/* enable qspi boot in clock node */
		offs = fdt_path_offset(fdt, "/soc/hps-clock-controller@34210000");
		if (offs < 0) {
			printf("failed to get hps clock node!");
			return;
		}
		ret = fdt_setprop_u32(fdt, offs, "qspi-boot", 1);
		if (ret < 0) {
			printf("failed to update hps clock node status!");
			return;
		}
		printf("enable qspi boot!\n");
	}
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	/*
	 * Add a subnode(membuff) under the soc node
	 */
#ifdef CONFIG_CONSOLE_RECORD
	membuff_setup(blob);
	fdt_add_mem_rsv(blob, (uintptr_t) gd->console_out.start,
	                (uint64_t)CONFIG_CONSOLE_RECORD_OUT_SIZE);
#endif
	bonding_setup(blob);
#ifdef CONFIG_CMD_SEND_ID
	hb_fdt_set_board_info(blob);
#endif
	fdt_set_status_by_env(blob);
	hb_setup_ion_size(blob);
	update_boot_mode(blob);
	check_cpu_1_8g_support(blob);
	return 0;
}
#endif
