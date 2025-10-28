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
#ifdef CONFIG_OF_LIBFDT_OVERLAY
#include <fs.h>
#endif

extern int hb_setup_ion_size(void *blob);

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

static void fdt_rm_by_env(void *fdt_blob)
{
	int nodeoffset, err;
	char *rm_list = env_get("fdt_remove");
	char *node_item_full, *node_item, *prop_item;
	if (rm_list == NULL) {
		return;
	}

	node_item_full = rm_list;
	while (rm_list != NULL) {
		debug("%s\n", rm_list);
		node_item_full = strsep(&rm_list, ";");
		do {
			/* Handle multiple spaces */
			node_item = strsep(&node_item_full, " ");
		} while (*node_item == '\0');

		debug("%s, %s\n", node_item_full, node_item);
		nodeoffset = fdt_path_offset(fdt_blob, node_item);
		if (nodeoffset < 0) {
			/*
			* Not found or something else bad happened.
			*/
			printf("libfdt fdt_path_offset() returned %s searching for %s\n",
				fdt_strerror(nodeoffset), node_item);
			continue;
		}
		if (node_item_full == NULL) {
			/* no space found, deleting node */
			err = fdt_del_node(working_fdt, nodeoffset);
			if (err < 0) {
				printf("libfdt fdt_del_node(%s): %s\n", node_item,
					fdt_strerror(err));
				continue;
			}
			printf("fdt node:%s removed\n", node_item);
		} else {
			/* space found, deleting prop */
			prop_item = node_item_full;
			err = fdt_delprop(working_fdt, nodeoffset, prop_item);
			if (err < 0) {
				printf("libfdt fdt_delprop(%s): %s\n", prop_item,
					fdt_strerror(err));
				continue;
			}
			printf("fdt prop:%s %s removed\n", node_item, prop_item);
		}
	}

	return;
}

static void check_cpu_1_8g_support(void *fdt)
{
        int offs, ret;
        int enable;
        uint32_t chip_type = 0;
        int i;
        char node[128] = {0};

        enable = env_get_yesno("enable_cpu_18g");

        /* enable opp table according to efuse info */
        ret = get_chip_type(&chip_type);
        if (ret) {
            printf("read efuse chip type failed\n");
            return;
        }

        /* update pll table if 1.8G supported */
        if (enable && ((chip_type == CHIP_X5_H) || (chip_type == CHIP_X5_U))) {
                /* enable corresponding opp table */
                offs = fdt_path_offset(fdt, "/cpu-opp-table-0/");
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
                int opp_offs;
                u32 phandle;

                /* disable pll table for 1.8G support */
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

                opp_offs = fdt_path_offset(fdt, "/cpu-opp-table-1/");
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

#ifdef CONFIG_OF_LIBFDT_OVERLAY
static void hb_do_fdt_overlay(void *blob)
{
	char *dtbo_fs;
	char *dtbo_dev;
	char *dtbo_part;
	char *dtbo_file_path;
	ulong dtbo_load_addr;
	loff_t loaded_file_size = 0;

	dtbo_file_path = env_get("dtbo_file_path");
	if (dtbo_file_path == NULL)
		return;

	dtbo_fs = env_get("dtbo_fs");
	if (dtbo_fs == NULL) {
		dtbo_fs = "ext4";
	}

	dtbo_dev = env_get("dtbo_dev");
	if (dtbo_dev == NULL) {
		dtbo_dev = "mmc";
	}

	dtbo_part = env_get("dtbo_part");
	if (dtbo_part == NULL) {
		dtbo_part = "0:d";
	}

	dtbo_load_addr = env_get_hex("dtbo_load_addr", 0x90000000);

	if (!strncmp("ext", dtbo_fs, strlen("ext"))) {
		fs_set_blk_dev(dtbo_dev, dtbo_part, FS_TYPE_EXT);
		fs_read(dtbo_file_path, dtbo_load_addr, 0x0, 0x0, &loaded_file_size);
		log_info("Applying %s from %s %s in fs %s\n", dtbo_file_path, dtbo_dev, dtbo_part, dtbo_fs);
	} else {
		log_info("Filesystem %s not supported!\n", dtbo_fs);
		return;
	}

	fdt_shrink_to_minimum(working_fdt, loaded_file_size);

	if (fdt_overlay_apply_verbose(blob, (void *)dtbo_load_addr)) {
		log_err("ERROR: FDT overlay apply failed!\n");
		return;
	}

	return;
}
#endif

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
	hb_do_fdt_overlay(blob);
	fdt_rm_by_env(blob);
	return 0;
}
#endif
