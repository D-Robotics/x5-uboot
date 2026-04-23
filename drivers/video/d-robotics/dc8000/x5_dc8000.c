// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 *
 * X5 DC8000 Nano Driver
 * Ported from: kernel/drivers/gpu/drm/verisilicon/dc/dc_hw/dc8000nano/dc_8000_nano.c
 */

#include <common.h>
#include <errno.h>
#include <linux/types.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <fdtdec.h>
#include "x5_dc8000.h"
#include "x5_dc8000_regs.h"
#include "../clock/x5_crm.h"
#include <hb_display_log.h>

static struct dc8000_config g_dc_config;

/*
 * Kernel dc_8000_nano_reg.h names; 0x02138/0x0213C overlap other symbols in
 * x5_dc8000_regs.h (cursor) — use these only for IRQ snapshot.
 */
#define DC8000_REG_DISPLAY_INTR		0x02138
#define DC8000_REG_DISPLAY_INTR_EN	0x0213C

/* GCREG_VIDEO_ALPHA_BLEND_CONFIG: premultiplied alpha (kernel plane_update_blend) */
#define DC_BLEND_PREMULTI			0x00006989u

/**
 * dc_write - Write to DC8000 register
 */
static inline void dc_write(u32 reg, u32 value)
{
	writel(value, (void __iomem *)(ulong)(DC8000_REG_BASE + reg));
}

/**
 * dc_read - Read from DC8000 register
 */
static inline u32 dc_read(u32 reg)
{
	return readl((void __iomem *)(ulong)(DC8000_REG_BASE + reg));
}

/**
 * dc_set_clear - Set and clear bits in register
 */
static inline void dc_set_clear(u32 reg, u32 set, u32 clear)
{
	u32 value = dc_read(reg);
	value &= ~clear;
	value |= set;
	dc_write(reg, value);
}

/**
 * x5_dc8000_dump_hw_state - Snapshot key DC8000 registers for kernel dmesg diff.
 * Format matches kernel dc8000_nano_dump_hw_state() ([X5_DISP] dc8000_regs ...).
 *
 * Stride line: kernel dc_8000_nano plane_update_fb() writes
 *   reg_stride = fb->y_stride * get_tile_height(tile_mode)
 * so U-Boot linear ARGB (tile linear, byte y_stride) often shows 0x1e00 while
 * kernel may show 0x780 (e.g. 1920 * tile scale) — compare plane/tile state, not only the number.
 */
static void x5_dc8000_dump_hw_state(const char *tag)
{
	X5_DISP_LOG_INFO(
		"dc8000_regs %s: FB cfg=0x%08x addr=0x%08x str=0x%08x sz=0x%08x tile=0x%08x\n",
		tag,
		dc_read(GCREG_FRAME_BUFFER_CONFIG_Address),
		dc_read(GCREG_FRAME_BUFFER_ADDRESS_Address),
		dc_read(GCREG_FRAME_BUFFER_STRIDE_Address),
		dc_read(GCREG_FRAME_BUFFER_SIZE_Address),
		dc_read(GCREG_DC_TILE_IN_CFG_Address));
	X5_DISP_LOG_INFO(
		"dc8000_regs %s: HDIS=0x%08x HSYNC=0x%08x VDIS=0x%08x VSYNC=0x%08x cur=0x%08x\n",
		tag,
		dc_read(GCREG_HDISPLAY_Address),
		dc_read(GCREG_HSYNC_Address),
		dc_read(GCREG_VDISPLAY_Address),
		dc_read(GCREG_VSYNC_Address),
		dc_read(GCREG_CURRENT_LOCATION_Address));
	X5_DISP_LOG_INFO(
		"dc8000_regs %s: PANEL cfg=0x%08x ctl=0x%08x fn=0x%08x work=0x%08x state=0x%08x\n",
		tag,
		dc_read(GCREG_PANEL_CONFIG_Address),
		dc_read(GCREG_PANEL_CONTROL_Address),
		dc_read(GCREG_PANEL_FUNCTION_Address),
		dc_read(GCREG_PANEL_WORKING_Address),
		dc_read(GCREG_PANEL_STATE_Address));
	X5_DISP_LOG_INFO(
		"dc8000_regs %s: DPI=0x%08x DBI=0x%08x dbg_cnt=0x%08x intr=0x%08x intr_en=0x%08x vid_a=0x%08x blend=0x%08x tl=0x%08x\n",
		tag,
		dc_read(GCREG_DPI_CONFIG_Address),
		dc_read(GCREG_DBI_CONFIG_Address),
		dc_read(GCREG_DEBUG_CNT_VALUE_Address),
		dc_read(DC8000_REG_DISPLAY_INTR),
		dc_read(DC8000_REG_DISPLAY_INTR_EN),
		dc_read(GCREG_VIDEO_GLOBAL_ALPHA_Address),
		dc_read(GCREG_VIDEO_ALPHA_BLEND_CONFIG_Address),
		dc_read(GCREG_VIDEO_TL_Address));
	X5_DISP_LOG_INFO(
		"dc8000_regs %s: blend_order=0x%08x ov_cfg=0x%08x ov1_cfg=0x%08x fb_bg=0x%08x curs_cfg=0x%08x\n",
		tag,
		dc_read(GCREG_BLEND_STACK_ORDER_Address),
		dc_read(GCREG_OVERLAY_CONFIG_Address),
		dc_read(GCREG_OVERLAY_CONFIG1_Address),
		dc_read(GCREG_FRAME_BUFFER_BACKGROUND_Address),
		dc_read(GCREG_CURSOR_CONFIG_Address));
}

/**
 * x5_dc8000_read_chip_info - Read and display chip information
 * Based on kernel's dc_hw_init_display()
 */
static void x5_dc8000_read_chip_info(void)
{
	u32 rev, date, patch, product;

	rev = dc_read(GCREG_DC_CHIP_REV_Address);
	date = dc_read(GCREG_DC_CHIP_DATE_Address);
	patch = dc_read(GCREG_DC_CHIP_PATCH_REV_Address);
	product = dc_read(GCREG_DC_PRODUCT_ID_Address);

	DC8000_LOG_DEBUG("Rev=0x%08x Date=0x%08x Patch=0x%08x Product=0x%08x\n",
		   rev, date, patch, product);
}

/*
 * After reset, overlay planes / cursor may be left enabled; blend then exposes
 * stale memory or default chroma (often green on HDMI). Kernel disables unused
 * planes in atomic commit — U-Boot must do the same once.
 * Also set primary backdrop to opaque black for A=0 pixels (premultiplied path).
 */
static void dc8000_quiesce_unused_layers(void)
{
	dc_write(GCREG_BLEND_STACK_ORDER_Address, 0);
	dc_set_clear(GCREG_OVERLAY_CONFIG_Address, 0, BIT(3));
	dc_set_clear(GCREG_OVERLAY_CONFIG1_Address, 0, BIT(3));
	dc_write(GCREG_CURSOR_CONFIG_Address, 0);
	dc_write(GCREG_FRAME_BUFFER_BACKGROUND_Address, 0xff000000u);

	DC8000_LOG_INFO("Compositor: blend_stack=0 ov0/ov1=off cursor=off bg ARGB=0xff000000\n");
}

/**
 * dc_hw_init_display - Initialize display controller
 * Direct port from kernel's dc_hw_init_display()
 */
static int dc_hw_init_display(void)
{
	u32 config;

	/* Configure panel: DE=1, polarities=0, clock=1 */
	config = VS_SET_FIELD(0, GCREG_PANEL_CONFIG, DE, 1) |
			 VS_SET_FIELD(0, GCREG_PANEL_CONFIG, DE_POLARITY, 0) |
			 VS_SET_FIELD(0, GCREG_PANEL_CONFIG, DATA_POLARITY, 0) |
			 VS_SET_FIELD(0, GCREG_PANEL_CONFIG, CLOCK_POLARITY, 0) |
			 VS_SET_FIELD(0, GCREG_PANEL_CONFIG, CLOCK, 1);

	dc_write(GCREG_PANEL_CONFIG_Address, config);

	DC8000_LOG_DEBUG("Panel config=0x%08x\n", config);

	return 0;
}

/**
 * dc_hw_enable_shadow - Enable shadow register loading
 * Direct port from kernel
 */
static void dc_hw_enable_shadow(bool enable)
{
	u32 config;

	config = VS_SET_FIELD(0, GCREG_PANEL_CONTROL, VALID, enable ? 1 : 0);
	dc_write(GCREG_PANEL_CONTROL_Address, config);
}

/**
 * disp_update_scan - Configure display timing
 * Direct port from kernel's disp_update_scan()
 *
 * IMPORTANT: Follow EXACT kernel sequence!
 * Kernel code (dc_8000_nano.c line 421-429):
 *   1. Set timing registers (HDISPLAY, HSYNC, VDISPLAY, VSYNC)
 *   2. dc_hw_init_display() - Panel config
 *   3. dc_hw_enable_shadow(true) - PANEL_CONTROL.VALID = 1
 *   4. PANEL_WORKING = 1
 *
 * Note: Kernel sets PANEL_CONTROL.VALID BEFORE PANEL_WORKING!
 */
static void disp_update_scan(struct display_timing *timing)
{
	u32 config;
	u32 h_active, h_total, h_sync_start, h_sync_end;
	u32 v_active, v_total, v_sync_start, v_sync_end;
	bool h_sync_polarity, v_sync_polarity;

	h_active = timing->hactive.typ;
	h_total = h_active + timing->hfront_porch.typ +
			  timing->hback_porch.typ + timing->hsync_len.typ;
	h_sync_start = h_active + timing->hfront_porch.typ;
	h_sync_end = h_sync_start + timing->hsync_len.typ;

	v_active = timing->vactive.typ;
	v_total = v_active + timing->vfront_porch.typ +
			  timing->vback_porch.typ + timing->vsync_len.typ;
	v_sync_start = v_active + timing->vfront_porch.typ;
	v_sync_end = v_sync_start + timing->vsync_len.typ;

	/* Polarity: kernel uses (flag ? 0 : 1) for inversion */
	h_sync_polarity = (timing->flags & DISPLAY_FLAGS_HSYNC_HIGH) ? 0 : 1;
	v_sync_polarity = (timing->flags & DISPLAY_FLAGS_VSYNC_HIGH) ? 0 : 1;

	DC8000_LOG_DEBUG("Timing H[%d+%d+%d+%d=%d] V[%d+%d+%d+%d=%d]\n",
		   h_active, timing->hfront_porch.typ, timing->hsync_len.typ,
		   timing->hback_porch.typ, h_total,
		   v_active, timing->vfront_porch.typ, timing->vsync_len.typ,
		   timing->vback_porch.typ, v_total);

	config = VS_SET_FIELD(0, GCREG_HDISPLAY, DISPLAY_END, h_active) |
			 VS_SET_FIELD(0, GCREG_HDISPLAY, TOTAL, h_total);
	dc_write(GCREG_HDISPLAY_Address, config);

	config = VS_SET_FIELD(0, GCREG_HSYNC, START, h_sync_start) |
			 VS_SET_FIELD(0, GCREG_HSYNC, END, h_sync_end) |
			 VS_SET_FIELD(0, GCREG_HSYNC, PULSE, 1) |
			 VS_SET_FIELD(0, GCREG_HSYNC, POLARITY, h_sync_polarity);
	dc_write(GCREG_HSYNC_Address, config);

	config = VS_SET_FIELD(0, GCREG_VDISPLAY, DISPLAY_END, v_active) |
			 VS_SET_FIELD(0, GCREG_VDISPLAY, TOTAL, v_total);
	dc_write(GCREG_VDISPLAY_Address, config);

	config = VS_SET_FIELD(0, GCREG_VSYNC, START, v_sync_start) |
			 VS_SET_FIELD(0, GCREG_VSYNC, END, v_sync_end) |
			 VS_SET_FIELD(0, GCREG_VSYNC, PULSE, 1) |
			 VS_SET_FIELD(0, GCREG_VSYNC, POLARITY, v_sync_polarity);
	dc_write(GCREG_VSYNC_Address, config);

	/* Initialize display config (sets PANEL_CONFIG with DE=1, CLOCK=1) */
	dc_hw_init_display();

	/*
	 * Set PANEL_CONTROL.VALID = 1 BEFORE PANEL_WORKING
	 * This is the KERNEL sequence - VALID first, then WORKING!
	 */
	dc_hw_enable_shadow(true);

	/*
	 * Write PANEL_WORKING = 1 to start transfer
	 * This copies all registers to working set and resets counters
	 */
	config = VS_SET_FIELD(0, GCREG_PANEL_WORKING, WORKING, 0x1);
	dc_write(GCREG_PANEL_WORKING_Address, config);

	g_dc_config.fb_width = h_active;
	g_dc_config.fb_height = v_active;
}

/**
 * plane_update_fb - Configure framebuffer
 * Based on kernel's plane_update_fb()
 *
 * Kernel code:
 *   dc_write(hw, reg->y_address, fb->y_address);
 *   dc_write(hw, reg->size, fb->width | (fb->height << 16));
 *   update_tile_config(hw, info, fb);
 *   dc_write(hw, reg->y_stride, stride);
 *   dc_set_clear(hw, reg->config,
 *                (fb->uv_swizzle << 19) | (fb->swizzle << 17) | BIT(3) | fb->format,
 *                BIT(19) | (0x3 << 17) | 0x7);
 */
static int plane_update_fb(u32 fb_addr, u32 format)
{
	u32 stride, bpp;

	if (!fb_addr)
	{
		DC8000_LOG_ERROR("Invalid framebuffer address\n");
		return -EINVAL;
	}

	switch (format)
	{
	case GCREG_FRAME_BUFFER_CONFIG_FORMAT_A8R8G8B8:
		bpp = 4;
		break;
	case GCREG_FRAME_BUFFER_CONFIG_FORMAT_R5G6B5:
		bpp = 2;
		break;
	default:
		DC8000_LOG_ERROR("Unsupported format 0x%x\n", format);
		return -EINVAL;
	}

	stride = g_dc_config.fb_width * bpp;

	DC8000_LOG_DEBUG("FB addr=0x%08x size=%dx%d stride=%d format=0x%x\n",
		   fb_addr, g_dc_config.fb_width, g_dc_config.fb_height,
		   stride, format);

	dc_write(GCREG_FRAME_BUFFER_ADDRESS_Address, fb_addr);

	dc_write(GCREG_FRAME_BUFFER_SIZE_Address,
			 g_dc_config.fb_width | (g_dc_config.fb_height << 16));

	dc_write(GCREG_DC_TILE_IN_CFG_Address, 0);

	dc_write(GCREG_FRAME_BUFFER_STRIDE_Address, stride);

	/*
	 * Configure FB_CONFIG using dc_set_clear (CRITICAL!)
	 * Kernel: dc_set_clear(hw, reg->config,
	 *                      (uv_swizzle << 19) | (swizzle << 17) | BIT(3) | format,
	 *                      BIT(19) | (0x3 << 17) | 0x7);
	 *
	 * For RGB (swizzle=0, uv_swizzle=0):
	 *   set = BIT(3) | format = 0x8 | 0x4 = 0x0C for ARGB8888
	 *   clear = 0x000E0007
	 */
	dc_set_clear(GCREG_FRAME_BUFFER_CONFIG_Address,
				 BIT(3) | format,			   /* set: enable + format */
				 BIT(19) | (0x3 << 17) | 0x7); /* clear mask */

	DC8000_LOG_DEBUG("FB_CONFIG=0x%08x\n", dc_read(GCREG_FRAME_BUFFER_CONFIG_Address));

	g_dc_config.fb_addr = fb_addr;
	g_dc_config.fb_format = format;
	g_dc_config.fb_stride = stride;

	return 0;
}

/**
 * plane_update_blend - Configure alpha blending
 * Based on kernel's plane_update_blend()
 *
 * Kernel code:
 *   dc_write(hw, reg->alpha_global_color, (blend->alpha << 8) | blend->alpha);
 *   dc_write(hw, reg->blend_config, 0x6989);  // DC_BLEND_PREMULTI
 */
static void plane_update_blend(void)
{
	/*
	 * Configure alpha: fully opaque (0xFF)
	 * alpha_global_color = (alpha << 8) | alpha = 0xFFFF
	 */
	dc_write(GCREG_VIDEO_GLOBAL_ALPHA_Address, 0x0000FFFF);

	/*
	 * Configure blend mode: DC_BLEND_PREMULTI (see #define)
	 * This is the default blend mode in kernel
	 */
	dc_write(GCREG_VIDEO_ALPHA_BLEND_CONFIG_Address, DC_BLEND_PREMULTI);
}

/**
 * plane_update_win - Configure window position
 * Based on kernel's plane_update_win()
 *
 * For fullscreen display, top-left is (0, 0)
 */
static void plane_update_win(void)
{
	dc_write(GCREG_VIDEO_TL_Address, 0);
}

/**
 * disp_update_output_internal - Configure output path (DPI)
 * Based on kernel's disp_update_output() + setup_output_dpi()
 *
 * MUST be called BEFORE disp_update_scan() per kernel sequence!
 */
static void disp_update_output_internal(void)
{
	u32 config;

	/*
	 * Disable YUV422 output (for RGB output)
	 * Kernel: disable_yuv422_output() clears BIT(11) in FB_CONFIG
	 */
	dc_set_clear(GCREG_FRAME_BUFFER_CONFIG_Address, 0, BIT(11));

	/*
	 * Configure PANEL_FUNCTION OUTPUT (from kernel's disp_update_output)
	 * OUTPUT = 1 to enable pixel output
	 * CRITICAL: This MUST be set BEFORE disp_update_scan()!
	 */
	config = dc_read(GCREG_PANEL_FUNCTION_Address);
	config = VS_SET_FIELD(config, GCREG_PANEL_FUNCTION, OUTPUT, 1);
	dc_write(GCREG_PANEL_FUNCTION_Address, config);

	/*
	 * Configure DPI output (from kernel's setup_output_dpi)
	 * For RGB888: dpi_cfg = 5
	 */
	dc_write(GCREG_DPI_CONFIG_Address, GCREG_DPI_CONFIG_DPI_DATA_FORMAT_D24);

	/*
	 * Configure DBI (from kernel's setup_output_dpi)
	 * BUS_OUTPUT_SEL = 0 for DPI output
	 */
	config = VS_SET_FIELD(0, GCREG_DBI_CONFIG, BUS_OUTPUT_SEL, 0);
	dc_write(GCREG_DBI_CONFIG_Address, config);

	DC8000_LOG_DEBUG("Output path configured (DPI RGB888)\n");
}

/**
 * x5_dc8000_init - Initialize DC8000 display controller
 * @timing: Display timing parameters
 *
 * Main initialization function following EXACT kernel sequence:
 *
 * Kernel dc_hw_proc_enable_disp() does:
 *   1. clk_prepare_enable(pix_clk)
 *   2. clk_set_rate(pix_clk, ...)
 *   3. dc_hw_update(DC_PROP_DISP_OUTPUT) -> disp_update_output()
 *   4. dc_hw_update(DC_PROP_DISP_SCAN) -> disp_update_scan()
 *
 * So the order is: OUTPUT first, then SCAN!
 */
int x5_dc8000_init(struct display_timing *timing)
{
	DC8000_LOG_INFO("Initializing display controller...\n");

	if (!timing) {
		DC8000_LOG_ERROR("display_timing is NULL\n");
		return -EINVAL;
	}

	/*
	 * NOTE: Do NOT soft reset DC8000 here!
	 */

	x5_dc8000_read_chip_info();

	dc8000_quiesce_unused_layers();

	/*
	 * CRITICAL: Follow EXACT kernel order!
	 * Kernel calls disp_update_output() BEFORE disp_update_scan()
	 * This sets PANEL_FUNCTION.OUTPUT = 1 before timing configuration
	 */

	/* Configure output path FIRST (kernel order!) */
	disp_update_output_internal();

	/* Configure display timing (includes dc_hw_init_display + shadow enable + panel working) */
	disp_update_scan(timing);

	DC8000_LOG_INFO("Initialization complete\n");

	return 0;
}

/**
 * x5_dc8000_setup_output - Configure output path (DPI/DBI)
 * Based on kernel's disp_update_output() + setup_output_dpi()
 *
 * NOTE: Most of the output configuration is now done in x5_dc8000_init()
 * via disp_update_output_internal() to match kernel order.
 * Kept for compatibility, just prints status.
 */
void x5_dc8000_setup_output(void)
{
	DC8000_LOG_DEBUG("Output configured (DPI RGB888)\n");
}

/**
 * x5_dc8000_set_framebuffer - Set framebuffer address and enable display
 * @fb_addr: Physical address of framebuffer
 * @format: Pixel format
 *
 * Following kernel's dc_8000_nano_proc_update() sequence:
 * 1. Disable shadow (PANEL_CONTROL VALID = 0)
 * 2. Update framebuffer config (plane_update_fb)
 * 3. Update window position (plane_update_win)
 * 4. Update blend config (plane_update_blend)
 * 5. Enable shadow (PANEL_CONTROL VALID = 1) - BEFORE PANEL_WORKING!
 * 6. Trigger panel working
 *
 * IMPORTANT: Kernel order is PANEL_CONTROL.VALID=1 THEN PANEL_WORKING=1
 */
int x5_dc8000_set_framebuffer(u32 fb_addr, u32 format)
{
	int ret;
	u32 config;

	dc_hw_enable_shadow(false);

	ret = plane_update_fb(fb_addr, format);
	if (ret)
		return ret;

	plane_update_win();

	plane_update_blend();

	/* Enable shadow register loading BEFORE PanelWorking (kernel order!) */
	dc_hw_enable_shadow(true);

	config = VS_SET_FIELD(0, GCREG_PANEL_WORKING, WORKING, 0x1);
	dc_write(GCREG_PANEL_WORKING_Address, config);

	x5_dc8000_dump_hw_state("post_set_fb");

	return 0;
}

/**
 * x5_dc8000_enable - Enable display output
 *
 * IMPORTANT: Follow KERNEL order!
 * Kernel disp_update_scan() does:
 *   1. Set timing registers
 *   2. dc_hw_init_display()
 *   3. dc_hw_enable_shadow(true) - PANEL_CONTROL.VALID = 1
 *   4. PANEL_WORKING = 1
 *
 * Kernel dc_8000_nano_proc_commit() sets PANEL_CONTROL.VALID = 1 after update.
 *
 * CRITICAL: The PANEL_CONTROL.VALID bit is automatically cleared by hardware
 * when Vsync occurs. If it stays at 1, it means Vsync never happened.
 */
void x5_dc8000_enable(void)
{
	u32 config;
	int i;

	x5_crm_dump_noc_idle();
	x5_crm_dump_dc_iommu();

	/*
	 * Ensure PANEL_CONFIG is correct:
	 * - CLOCK = 1 (enable pixel clock output)
	 * - DE = 1 (enable data enable)
	 * - All polarities = 0 (active high)
	 */
	config = VS_SET_FIELD(0, GCREG_PANEL_CONFIG, DE, 1) |
			 VS_SET_FIELD(0, GCREG_PANEL_CONFIG, DE_POLARITY, 0) |
			 VS_SET_FIELD(0, GCREG_PANEL_CONFIG, DATA_POLARITY, 0) |
			 VS_SET_FIELD(0, GCREG_PANEL_CONFIG, CLOCK_POLARITY, 0) |
			 VS_SET_FIELD(0, GCREG_PANEL_CONFIG, CLOCK, 1);
	dc_write(GCREG_PANEL_CONFIG_Address, config);
	DC8000_LOG_DEBUG("PANEL_CONFIG=0x%08x\n", config);

	/* Set PANEL_CONTROL.VALID = 1 BEFORE PanelWorking (kernel order!) */
	dc_hw_enable_shadow(true);
	DC8000_LOG_DEBUG("PANEL_CONTROL set to 1 (shadow enable)\n");

	config = VS_SET_FIELD(0, GCREG_PANEL_WORKING, WORKING, 0x1);
	dc_write(GCREG_PANEL_WORKING_Address, config);
	DC8000_LOG_DEBUG("PANEL_WORKING triggered\n");

	/*
	 * Wait for timing to be loaded and display to start scanning.
	 * Poll PANEL_CONTROL.VALID to see when it gets cleared (Vsync occurred).
	 * Optimized: poll every 5ms instead of 20ms, max 50ms timeout
	 */
	DC8000_LOG_DEBUG("Waiting for Vsync (PANEL_CONTROL should auto-clear)...\n");
	for (i = 0; i < 10; i++)
	{
		mdelay(5); /* Poll every 5ms */

		config = dc_read(GCREG_PANEL_CONTROL_Address);
		DC8000_LOG_DEBUG("[%d] PANEL_CONTROL=0x%08x\n", i, config);

		if (config == 0)
		{
			DC8000_LOG_DEBUG("Vsync occurred! PANEL_CONTROL auto-cleared.\n");
			break;
		}
	}

	/* CURRENT_LOCATION: X=bits[15:0], Y=bits[31:16] per DC8000 spec */
	config = dc_read(GCREG_CURRENT_LOCATION_Address);
	DC8000_LOG_DEBUG("CURRENT_LOCATION=0x%08x (X=%d, Y=%d)\n",
		   config, config & 0xFFFF, (config >> 16) & 0xFFFF);

	config = dc_read(GCREG_PANEL_STATE_Address);
	DC8000_LOG_DEBUG("PANEL_STATE=0x%08x\n", config);

	config = dc_read(GCREG_DEBUG_CNT_VALUE_Address);
	DC8000_LOG_DEBUG("DEBUG_CNT_VALUE=0x%08x (frame count)\n", config);

	x5_dc8000_dump_hw_state("post_enable");

	DC8000_LOG_INFO("Display enabled\n");
}

/**
 * x5_dc8000_disable - Disable display output
 * Based on kernel's disable_display()
 */
void x5_dc8000_disable(void)
{
	u32 config;

	config = dc_read(GCREG_PANEL_CONFIG_Address);
	config = VS_SET_FIELD(config, GCREG_PANEL_CONFIG, CLOCK, 0);
	dc_write(GCREG_PANEL_CONFIG_Address, config);

	dc_write(GCREG_SOFT_RESET_Address, GCREG_SOFT_RESET_RESET_RESET);

	DC8000_LOG_INFO("Display disabled\n");
}

/**
 * x5_dc8000_get_config - Get current DC8000 configuration
 */
struct dc8000_config *x5_dc8000_get_config(void)
{
	return &g_dc_config;
}
