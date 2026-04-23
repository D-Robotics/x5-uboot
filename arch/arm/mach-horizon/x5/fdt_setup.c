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
#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
#include <fdtdec.h>
#endif
#include <mtd_node.h>
#include <jffs2/load_kernel.h>
#include <console.h>
#include <asm/arch/hb_efuse.h>
#ifndef CONFIG_SPL_BUILD
#include <env.h>
#endif
#include <linux/sizes.h>
#include <linux/kernel.h>
#include <asm/byteorder.h>
#include <log.h>
#include <asm/arch/hb_strappin.h>
#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
#include <video.h>
#endif
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

void fdt_rm_by_env(void *fdt_blob)
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
        if (enable && ((chip_type == CHIP_X5_H) || (chip_type == CHIP_X5_UKNOWN))) {
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


#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
#include <hb_display_log.h>
#include <linux/string.h>

#define SEAMLESS_STATE_PROP "d-robotics,seamless-display-state"
/* Must match U-Boot DC8000 / video ARGB8888 (see x5_display + dc8000_nano). */
#define SEAMLESS_SIMPLEFB_FORMAT "a8r8g8b8"

/*
 * Default seamless-display carveout base when DT lacks horizon,dc8000 /
 * framebuffer-base. Same physical region as reserved-memory in
 * arch/arm/dts/x5.dtsi (display_reserved) and kernel x5-memory.dtsi
 * (seamless_fb_reserved).
 */
#define SEAMLESS_FB_FALLBACK_ADDR 0xA2080000u

/*
 * reserved-memory framebuffer node name uses the same base as dc8000
 * framebuffer-base (arch/arm/dts/x5.dtsi). Build path from DT instead of
 * hardcoding the address in C.
 */
static void seamless_display_fill_fb_fallback_path(void *blob, char *path, size_t path_sz)
{
	int node;
	const fdt32_t *prop;
	int len;
	u32 fb_base;

	if (!path || path_sz == 0)
		return;

	node = fdt_node_offset_by_compatible(blob, -1, "horizon,dc8000");
	if (node >= 0) {
		prop = fdt_getprop(blob, node, "framebuffer-base", &len);
		if (prop && len >= (int)sizeof(fdt32_t)) {
			fb_base = fdt32_to_cpu(*prop);
			/* Match reserved-memory node unit-address (same spelling as kernel DTS). */
			snprintf(path, path_sz, "/reserved-memory/framebuffer@%08X", fb_base);
			return;
		}
	}
	/* Last resort if compatible/property missing (older or broken DT). */
	snprintf(path, path_sz, "/reserved-memory/framebuffer@%08X", SEAMLESS_FB_FALLBACK_ADDR);
}

static void seamless_display_set_fb_fallback_status(void *blob, bool enable)
{
	char path[80];
	int node;
	int ret;

	seamless_display_fill_fb_fallback_path(blob, path, sizeof(path));
	node = fdt_path_offset(blob, path);
	if (node < 0)
		return;

	ret = fdt_setprop_string(blob, node, "status", enable ? "okay" : "disabled");
	if (ret < 0)
		SEAMLESS_LOG_WARNING("failed to set fallback fb status=%s (%d)\n",
				     enable ? "okay" : "disabled", ret);
}

static void seamless_display_remove_simplefb(void *blob, int chosen_off)
{
	int node, found;

	if (chosen_off < 0)
		return;

	/*
	* Remove kernel simple-framebuffer nodes under /chosen (stale from prior boot or
	* static DTS). Linux fbcon uses the node U-Boot adds when seamless FB reserve succeeds.
	*/
	while (true) {
		found = -1;
		fdt_for_each_subnode(node, blob, chosen_off) {
			if (fdt_node_check_compatible(blob, node,
						     "simple-framebuffer") == 0) {
				found = node;
				break;
			}
		}
		if (found < 0)
			break;
		if (fdt_del_node(blob, found) < 0) {
			SEAMLESS_LOG_WARNING("failed to remove simple-framebuffer node\n");
			break;
		}
	}
}

/*
 * Pass the *actual* U-Boot video FB (base/size from plat) and mode (xsize/ysize/stride)
 * so the kernel needs no fixed resolution in DTS. Matches DC8000 ARGB8888 in x5_display.
 */
static int seamless_display_add_simplefb(void *blob, int chosen_off, uintptr_t base,
					 ulong size, u32 width, u32 height, u32 stride)
{
	char name[48];
	int node, ret;
	__be32 reg[4];

	if (!width || !height || !stride) {
		SEAMLESS_LOG_WARNING("simplefb: invalid geometry\n");
		return -EINVAL;
	}
	if ((u64)stride * (u64)height > (u64)size) {
		SEAMLESS_LOG_WARNING("simplefb: stride*height exceeds fb size\n");
		return -EINVAL;
	}

	snprintf(name, sizeof(name), "framebuffer@%llX",
		 (unsigned long long)base);

	seamless_display_remove_simplefb(blob, chosen_off);

	node = fdt_add_subnode(blob, chosen_off, name);
	if (node < 0) {
		SEAMLESS_LOG_WARNING("simplefb: fdt_add_subnode failed (%d)\n", node);
		return node;
	}

	ret = fdt_setprop_string(blob, node, "compatible", "simple-framebuffer");
	if (ret < 0)
		return ret;

	reg[0] = cpu_to_be32((u64)base >> 32);
	reg[1] = cpu_to_be32((u32)base);
	reg[2] = cpu_to_be32((u64)size >> 32);
	reg[3] = cpu_to_be32((u32)size);
	ret = fdt_setprop(blob, node, "reg", reg, sizeof(reg));
	if (ret < 0)
		return ret;

	ret = fdt_setprop_u32(blob, node, "width", width);
	if (ret < 0)
		return ret;
	ret = fdt_setprop_u32(blob, node, "height", height);
	if (ret < 0)
		return ret;
	ret = fdt_setprop_u32(blob, node, "stride", stride);
	if (ret < 0)
		return ret;

	ret = fdt_setprop_string(blob, node, "format", SEAMLESS_SIMPLEFB_FORMAT);
	if (ret < 0)
		return ret;

	SEAMLESS_LOG_DEBUG("simplefb %s %ux%u stride=%u base=0x%lx size=0x%lx\n",
			   name, width, height, stride, (ulong)base, size);
	return 0;
}

static int seamless_display_fdt_setup(void *blob)
{
	const char *seamless_env;
	struct udevice *dev;
	struct video_uc_plat *plat;
	struct video_priv *uc_priv;
	struct fdt_memory mem;
	bool fb_reserved = false;
	uintptr_t sf_base = 0;
	ulong sf_size = 0;
	u32 sf_w = 0, sf_h = 0, sf_stride = 0;
	int chosen_offset;
	int ret;

	ret = fdt_increase_size(blob, SZ_8K);
	if (ret < 0) {
		SEAMLESS_LOG_WARNING("fdt_increase_size failed (%d)\n", ret);
		return ret;
	}

	chosen_offset = fdt_path_offset(blob, "/chosen");
	if (chosen_offset < 0) {
		SEAMLESS_LOG_WARNING("/chosen not found in FDT\n");
		return chosen_offset;
	}

	/*
	 * simple-framebuffer is a child of /chosen with a 64-bit reg (2+2 cells).
	 * Without these, Linux OF does not build IORESOURCE_MEM and simplefb_probe
	 * fails with "No memory resource" (-EINVAL).
	 */
	ret = fdt_setprop_u32(blob, chosen_offset, "#address-cells", 2);
	if (ret < 0)
		SEAMLESS_LOG_WARNING("chosen #address-cells failed (%d)\n", ret);
	ret = fdt_setprop_u32(blob, chosen_offset, "#size-cells", 2);
	if (ret < 0)
		SEAMLESS_LOG_WARNING("chosen #size-cells failed (%d)\n", ret);

	/*
	 * of_address_to_resource() uses of_translate_address(), which stops at a
	 * parent with no "ranges" (drivers/of/address.c, non-PPC). /chosen has no
	 * ranges by default, so translation fails, of_device_alloc() gets num_reg==0,
	 * and simple-framebuffer probes with "No memory resource". An empty
	 * "ranges" property means 1:1 mapping (same as arch/arm/mach-omap2/fdt-common.c).
	 */
	ret = fdt_setprop(blob, chosen_offset, "ranges", NULL, 0);
	if (ret < 0)
		SEAMLESS_LOG_WARNING("chosen empty ranges failed (%d)\n", ret);

	ret = fdt_setprop_u32(blob, chosen_offset, SEAMLESS_STATE_PROP, 0);
	if (ret < 0) {
		SEAMLESS_LOG_WARNING("failed to set %s=0 (%d)\n",
				     SEAMLESS_STATE_PROP, ret);
		return ret;
	}
	seamless_display_set_fb_fallback_status(blob, false);

	seamless_env = env_get("seamless_display");
	if (!seamless_env || strcmp(seamless_env, "1") != 0) {
		SEAMLESS_LOG_DEBUG("disabled by env\n");
		seamless_display_remove_simplefb(blob, chosen_offset);
		return 0;
	}

	ret = uclass_first_device_err(UCLASS_VIDEO, &dev);
	if (ret || !dev) {
		SEAMLESS_LOG_DEBUG("no active video device, skip FB reserve\n");
	} else {
		plat = dev_get_uclass_plat(dev);
		uc_priv = dev_get_uclass_priv(dev);
		if (!plat || !uc_priv || !plat->base || !plat->size ||
		    !uc_priv->xsize || !uc_priv->ysize) {
			SEAMLESS_LOG_DEBUG("video device not fully initialized, skip FB reserve\n");
		} else {
			SEAMLESS_LOG_DEBUG("fb 0x%lx %dx%d reserve %lu bytes\n",
					   (unsigned long)plat->base, uc_priv->xsize,
					   uc_priv->ysize, (unsigned long)plat->size);

			mem.start = plat->base;
			mem.end = plat->base + plat->size - 1;
			ret = fdtdec_add_reserved_memory(blob, "framebuffer", &mem, NULL, 0, NULL,
							 FDTDEC_RESERVED_MEMORY_NO_MAP);
			if (ret < 0) {
				SEAMLESS_LOG_DEBUG("fdtdec_add_reserved_memory failed (%d), fallback to mem_rsv\n",
						   ret);
				ret = fdt_add_mem_rsv(blob, plat->base, plat->size);
				if (ret < 0) {
					SEAMLESS_LOG_WARNING("fdt_add_mem_rsv failed (%d)\n", ret);
				} else {
					fb_reserved = true;
				}
			} else {
				fb_reserved = true;
			}
			if (fb_reserved) {
				sf_base = plat->base;
				sf_size = plat->size;
				sf_w = uc_priv->xsize;
				sf_h = uc_priv->ysize;
				sf_stride = uc_priv->line_length ?
						(u32)uc_priv->line_length :
						(u32)uc_priv->xsize * 4;
			}
		}
	}

	/*
	 * Kernel seamless_fb_reserved (x5-memory.dtsi): must end as status "okay"
	 * whenever we rely on that carveout for the FB PA.
	 *
	 * fdtdec_add_reserved_memory("framebuffer", ...) usually *matches* the existing
	 * seamless_fb_reserved node by address/size and returns without adding a second
	 * node. Leaving that node "disabled" (we clear it at the start of this function)
	 * drops no-map → RAM re-enters the buddy allocator → simplefb cannot reserve /
	 * ioremap_wc hits ioremap_allowed() on DRAM (see drivers/video/fbdev/simplefb.c).
	 *
	 * When U-Boot could not reserve FB (fb_reserved false), still enable the static
	 * carveout so the kernel can use the fallback region without /chosen simplefb.
	 */
	seamless_display_set_fb_fallback_status(blob, true);

	ret = fdt_setprop_u32(blob, chosen_offset, SEAMLESS_STATE_PROP, fb_reserved ? 1 : 0);
	if (ret < 0) {
		SEAMLESS_LOG_WARNING("failed to set %s=%d (%d)\n",
				     SEAMLESS_STATE_PROP, fb_reserved ? 1 : 0, ret);
		return ret;
	}

	if (fb_reserved) {
		ret = seamless_display_add_simplefb(blob, chosen_offset, sf_base, sf_size,
						    sf_w, sf_h, sf_stride);
		if (ret < 0)
			SEAMLESS_LOG_WARNING("simplefb: failed (%d)\n", ret);
		SEAMLESS_LOG_DEBUG("FDT setup complete (state=1)\n");
	} else {
		seamless_display_remove_simplefb(blob, chosen_offset);
		SEAMLESS_LOG_DEBUG("FDT setup complete (state=0)\n");
	}

	return 0;
}
#else
static int seamless_display_fdt_setup(void *blob)
{
	return 0;
}
#endif

__weak int ft_board_setup(void *blob, struct bd_info *bd)
{
	int ret;

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
	ret = seamless_display_fdt_setup(blob);
	if (ret < 0)
		log_warning("seamless_display: setup failed (%d), continuing boot\n", ret);
	return 0;
}
#endif
