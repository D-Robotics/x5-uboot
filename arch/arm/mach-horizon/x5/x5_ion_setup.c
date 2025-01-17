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
#include <env.h>
#include <dm.h>
#include <fdtdec.h>
#include <fdt_support.h>
#include <errno.h>
#include <linux/libfdt.h>
#include <linux/sizes.h>
#include <log.h>

#ifdef CONFIG_OF_LIBFDT_OVERLAY
#include <fs.h>
#endif

/* NOTICE: "*_NAME" refers to dts node name not dts label,
 * "*_COMPATIBLE" refers to dts "compatible" property.
*/
#define ION_CMA_NAME "ion_cma"
#define ION_CMA_COMPATIBLE "ion-cma"
#define ION_RESERVED_NAME "ion_reserved"
#define ION_RESERVED_COMPATIBLE "ion-pool"
#define ION_CARVEOUT_NAME "ion_carveout"
#define ION_CARVEOUT_COMPATIBLE "ion-carveout"

/* 200MiB is reserved for kernel heaps by default */
#define DEFAULT_KERNEL_MIN_HEAP (0xC800000u)

#define ION_NODE_NAME_MAX_LEN (64u)
#define ION_NODE_SUFFIX_MAX_LEN (32u)
#define ION_REG_PROP_NAME "reg"

#define DDR_SIZE_1GB ((uint64_t)1 * SZ_1G)
#define DDR_SIZE_1GB_SUFFIX "_1g"
#define DDR_SIZE_2GB ((uint64_t)2 * SZ_1G)
#define DDR_SIZE_2GB_SUFFIX "_2g"
#define DDR_SIZE_4GB ((uint64_t)4 * SZ_1G)
#define DDR_SIZE_GE4GB_SUFFIX "_ge4g"
#define DDR_SIZE_RDK_SUFFIX "_rdk"
#define ION_MIN_SIZE (0x4000000)  /* 64 MiB */

#define DEFAULT_ION_REGION_START (0xA4100000)
#define RDK_DEFAULT_ION_RESERVED_SIZE (0x14000000) /* 320MiB */
#define RDK_DEFAULT_ION_CARVEOUT_SIZE (0x14000000) /* 320MiB */
#define RDK_DEFAULT_ION_CMA_SIZE (0x8000000) /* 128MiB */

#define SIZE_ZERO (0x0u)
static int rsv_offset;
static uint64_t ion_region_start, ion_reserved_size, ion_carveout_size, ion_cma_size;
static uint64_t ion_reserved_size_dft, ion_carveout_size_dft, ion_cma_size_dft;
static char ion_reserved_name[ION_NODE_NAME_MAX_LEN];
static char ion_carveout_name[ION_NODE_NAME_MAX_LEN];
static char ion_cma_name[ION_NODE_NAME_MAX_LEN];
static char node_suffix[ION_NODE_SUFFIX_MAX_LEN];

int hb_set_ion_region_size(void *blob, const char *name, const char *compatible,
							 u64 size, u64 start, u64 range_size)
{
	int nodeoffset;
	u32 dma_ranges[4];
	int ret;

	/* nodeoffset could be changed by the following fdt operations
	 * Update nodeoffset before any operations.
	 */
	nodeoffset = fdt_find_or_add_subnode(blob, rsv_offset, name);
	ret = fdt_setprop_u64(blob, nodeoffset, "size", size);
	if (ret < 0) {
		log_err("%s: Failed to set %s(%d) memory_size(%llx): %s\n",
				__func__, name, nodeoffset, size,
				fdt_strerror(ret));
		return 1;
	}

	ret = fdt_setprop_string(blob, nodeoffset, "compatible", compatible);
	if (ret < 0) {
		log_err("%s: setup compatible(%s) failed: %s\n", __func__, compatible,
			fdt_strerror(ret));
		return 1;
	}

	ret = fdt_setprop_string(blob, nodeoffset, "status", "okay");
	if (ret < 0) {
		log_err("%s: setup compatible failed: %s\n", __func__,
			fdt_strerror(ret));
		return 1;
	}

	dma_ranges[0] =	cpu_to_fdt32((start >> 32) & 0xffffffff);
	dma_ranges[1] =	cpu_to_fdt32(start & 0xffffffff);
	dma_ranges[2] =	cpu_to_fdt32((size >> 32) & 0xffffffff);
	dma_ranges[3] =	cpu_to_fdt32(size & 0xffffffff);
	ret = fdt_setprop(blob, nodeoffset, ION_REG_PROP_NAME, &dma_ranges[0],
			sizeof(dma_ranges));
	if (ret < 0) {
		log_err("%s: setup reusable failed: %s\n", __func__,
			fdt_strerror(ret));
		return 1;
	}

	log_info("Set Mem[%s] Size to 0x%016llx@0x%llx\n", name, size, start);

	return 0;
}


int hb_set_ion_cma_size(void *blob, const char *name, const char *compatible,
							 u64 size, u64 start, u64 range_size)
{
	int ret, nodeoffset;

	hb_set_ion_region_size(blob, name, compatible, size, start, range_size);
	nodeoffset = fdt_find_or_add_subnode(blob, rsv_offset, name);
	if (size == 0) {
		ret = fdt_setprop_string(blob, nodeoffset, "status", "disabled");
		if (ret < 0) {
			log_err("%s: setup status failed: %s\n", __func__,
				fdt_strerror(ret));
			return 1;
		}
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

	return 0;
}

static inline void hb_ion_set_region_size(void* blob, const uint64_t ddr_size)
{
	uint64_t ion_total = 0;
	bool printed = false;

	ion_reserved_size = env_get_ulong("ion_reserved_size", 16, \
		ion_reserved_size_dft);
	log_debug("Get ion reserved size, env:%#llx, default:%#llx\n",
				ion_reserved_size, ion_reserved_size_dft);

	ion_carveout_size = env_get_ulong("ion_carveout_size", 16, \
		ion_carveout_size_dft);
	log_debug("Get ion carveout size, env:%#llx, default:%#llx\n",
				ion_carveout_size, ion_carveout_size_dft);

	ion_cma_size = env_get_ulong("ion_cma_size", 16, \
		ion_cma_size_dft);
	log_debug("Get ion cma size, env:%#llx, default:%#llx\n",
				ion_cma_size, ion_cma_size_dft);

	/* One of the environment variable is set, configure the total ion size */
	do {
		ion_total = (ion_reserved_size) + (ion_carveout_size) + (ion_cma_size);
		if ((ion_total + ion_region_start - PHYS_SDRAM_1 + DEFAULT_KERNEL_MIN_HEAP) <= ((ddr_size))) {
			break;
		} else {
			if (ion_reserved_size == ION_MIN_SIZE &&
				ion_carveout_size == ION_MIN_SIZE &&
				ion_cma_size == ION_MIN_SIZE) {
					pr_err("ERROR: ION shrinked to minimum but still exceeds DDR total size!\n");
					pr_err("\tion_region_start_addr:%llx\n", ion_region_start);
					pr_err("\tion_region_size:%d\n", ION_MIN_SIZE);
					break;
				}
		}

		if (ion_reserved_size > ION_MIN_SIZE)
			ion_reserved_size -= SZ_16M;

		if (ion_carveout_size > ION_MIN_SIZE)
			ion_carveout_size -= SZ_16M;

		if (ion_cma_size > ION_MIN_SIZE)
			ion_cma_size -= SZ_16M;

		if (!printed) {
			pr_err("ION Total size exceeds DDR total size, shrinking!\n");
			printed = true;
		}
	} while (true);

	return;
}

static long hb_parse_ion_dts(void *blob)
{
	int ion_reserved_offset, ion_carveout_offset, ion_cma_offset;

	rsv_offset = fdt_find_or_add_subnode(blob, 0, "reserved-memory");
	if (rsv_offset < 0) {
		/*
		 * Not found or something else bad happened.
		 */
		log_err("%s: libfdt fdt_find_or_add_subnode() returned %s\n", __func__,
			fdt_strerror(rsv_offset));
		return rsv_offset;
	}

	(void)snprintf(ion_reserved_name, ION_NODE_NAME_MAX_LEN - 1,
		"%s%s", ION_RESERVED_NAME, node_suffix);
	ion_reserved_offset = fdt_find_or_add_subnode(blob, rsv_offset, ion_reserved_name);
	log_debug("%s: ion_reserved_name:%s, nodeoffset:%d\n",
				__func__, ion_reserved_name, ion_reserved_offset);
	if (ion_reserved_offset < 0) {
		/*
		 * Not found or something else bad happened.
		 */
		log_err("%s: libfdt fdt_find_or_add_subnode() for node(%s) returned %s\n", __func__,
			ion_reserved_name,
			fdt_strerror(ion_reserved_offset));
		return ion_reserved_offset;
	}

	(void)snprintf(ion_carveout_name, ION_NODE_NAME_MAX_LEN - 1,
		"%s%s", ION_CARVEOUT_NAME, node_suffix);
	ion_carveout_offset = fdt_find_or_add_subnode(blob, rsv_offset, ion_carveout_name);
	log_debug("%s: ion_carvout_name:%s, nodeoffset:%d\n",
				__func__, ion_carveout_name, ion_carveout_offset);
	if (ion_carveout_offset < 0) {
		/*
		 * Not found or something else bad happened.
		 */
		log_err("%s: libfdt fdt_find_or_add_subnode() for node(%s) returned %s\n", __func__,
			ion_carveout_name,
			fdt_strerror(ion_carveout_offset));
		return ion_carveout_offset;
	}

	(void)snprintf(ion_cma_name, ION_NODE_NAME_MAX_LEN - 1,
		"%s%s", ION_CMA_NAME, node_suffix);
	ion_cma_offset = fdt_find_or_add_subnode(blob, rsv_offset, ion_cma_name);
	log_debug("%s: ion_cma_name:%s, nodeoffset:%d\n",
				__func__, ion_cma_name, ion_cma_offset);
	if (ion_cma_offset < 0) {
		/*
		 * Not found or something else bad happened.
		 */
		log_err("%s: libfdt fdt_find_or_add_subnode() for node(%s) returned %s\n", __func__,
			ion_cma_name,
			fdt_strerror(ion_cma_offset));
		return ion_cma_offset;
	}

	/* Assume the first ion region is always ion_reserved, use ion_reserved for ion region start */
	ion_region_start = fdtdec_get_addr_size_auto_parent(blob, rsv_offset, ion_reserved_offset,
											ION_REG_PROP_NAME, 0, &ion_reserved_size_dft, false);
	if (ion_region_start == FDT_ADDR_T_NONE) {
		pr_err("Start of ion_reserved not found!\n");
		return FDT_ADDR_T_NONE;
	}

	if (fdtdec_get_addr_size_auto_parent(blob, rsv_offset, ion_carveout_offset,
			ION_REG_PROP_NAME, 0, &ion_carveout_size_dft, false) == FDT_ADDR_T_NONE) {
		pr_err("ion_carveout not found!\n");
		return FDT_ADDR_T_NONE;
	}

	if (fdtdec_get_addr_size_auto_parent(blob, rsv_offset, ion_cma_offset,
			ION_REG_PROP_NAME, 0, &ion_cma_size_dft, false) == FDT_ADDR_T_NONE) {
		pr_err("ion_cma not found!\n");
		return FDT_ADDR_T_NONE;
	}

	return 0;
}


int hb_setup_ion_size(void *blob)
{
	uint64_t ddr_size = ((uint64_t)4 * SZ_1G);
	uint64_t offset_size;
	uint32_t num_banks = 0;
	char *board_id = NULL;

	memset(ion_reserved_name, 0, sizeof(node_suffix));
	memset(ion_carveout_name, 0, sizeof(node_suffix));
	memset(ion_cma_name, 0, sizeof(node_suffix));
	memset(node_suffix, 0, sizeof(node_suffix));

	rsv_offset = 0;

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

	board_id = env_get("hb_board_id");
	log_debug("%s: Get board_id:%s\n", __func__, board_id);
	
	log_debug("%s: Get ddr_size:%lld\n", __func__, ddr_size);
	if (strcmp(board_id,"0x0301") == 0 || strcmp(board_id,"0x0302") == 0) {
	strncpy(node_suffix, DDR_SIZE_RDK_SUFFIX, ION_NODE_SUFFIX_MAX_LEN - 1);
	} else if (ddr_size <= DDR_SIZE_1GB) {
		strncpy(node_suffix, DDR_SIZE_1GB_SUFFIX, ION_NODE_SUFFIX_MAX_LEN - 1);
	} else if (ddr_size <= DDR_SIZE_2GB) {
		strncpy(node_suffix, DDR_SIZE_2GB_SUFFIX, ION_NODE_SUFFIX_MAX_LEN - 1);
	} else {
		strncpy(node_suffix, DDR_SIZE_GE4GB_SUFFIX, ION_NODE_SUFFIX_MAX_LEN - 1);
	}
	log_debug("ION region suffix:%s\n", node_suffix);
	if (hb_parse_ion_dts(blob)) {
		pr_err("ION fdt parse failed!\n");
		ion_reserved_size_dft = RDK_DEFAULT_ION_RESERVED_SIZE;
		ion_carveout_size_dft = RDK_DEFAULT_ION_CARVEOUT_SIZE;
		ion_cma_size_dft = RDK_DEFAULT_ION_CMA_SIZE;
		ion_region_start = DEFAULT_ION_REGION_START;
		printf("Use default ION size ion_reserved_size_dft:%#llx, ion_carveout_size_dft:%#llx, ion_cma_size_dft:%#llx, ion_region_start:%#llx\n", ion_reserved_size_dft, ion_carveout_size_dft, ion_cma_size_dft,ion_region_start);
	}
	hb_ion_set_region_size(blob, ddr_size);

	offset_size = ion_region_start;
	(void) hb_set_ion_region_size(blob, ion_reserved_name,
				ION_RESERVED_COMPATIBLE, ion_reserved_size,
				offset_size,
				gd->bd->bi_dram[0].size - offset_size);
	offset_size = offset_size + ion_reserved_size;
	(void) hb_set_ion_region_size(blob, ion_carveout_name,
				ION_CARVEOUT_COMPATIBLE, ion_carveout_size,
				offset_size,
				gd->bd->bi_dram[0].size - offset_size);
	offset_size = offset_size + ion_carveout_size;
	(void) hb_set_ion_cma_size(blob, ion_cma_name,
				ION_CMA_COMPATIBLE, ion_cma_size,
				offset_size,
				gd->bd->bi_dram[0].size - offset_size);

	return 0;
}
