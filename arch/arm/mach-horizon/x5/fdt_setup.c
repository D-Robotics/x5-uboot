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
static int x5_setup_cma_size(void *blob)
{
	/* TODO after memory map is fixed */
	return 0;
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
	x5_setup_cma_size(blob);
	check_cpu_1_8g_support(blob);
	return 0;
}
#endif