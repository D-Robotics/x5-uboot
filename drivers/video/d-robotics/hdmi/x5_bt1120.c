// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * Ported from: kernel/drivers/gpu/drm/verisilicon/dc_bt1120/vs_bt1120.c
 *              kernel/drivers/gpu/drm/verisilicon/dc_bt1120/vs_bt1120_reg.h
 *
 * Online mode: RGB from DC8000 DPI -> BT1120 -> YUV422 parallel -> LT8618.
 */

#include <common.h>
#include <errno.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/string.h>
#include <fdtdec.h>
#include <panel.h>
#include <media_bus_format.h>

#include "x5_bt1120.h"
#include <hb_display_log.h>

/* Register offsets from vs_bt1120_reg.h */
#define REG_BT1120_CTL				0x00
#define REG_BT1120_DMA_BLENGTH_CTL		0x04
/* Kernel vs_bt1120 default bt1120_display_info.dma_burst_len */
#define BT1120_DMA_BURST_LEN_DEFAULT		0x0a
#define REG_BT1120_LINE_OFFSET_CTL		0x08
#define REG_BT1120_WORD_OFFSET_CTL		0x0c
#define REG_BT1120_IMG_IN_BADDR_Y_CTL		0x10
#define REG_BT1120_IMG_IN_BADDR_UV_CTL		0x14
#define REG_BT1120_IMG_PIX_HSIZE_CTL		0x18
#define REG_BT1120_IMG_PIX_VSIZE_CTL		0x1c
#define REG_BT1120_IMG_PIX_HSTRIDE_CTL		0x20
#define REG_BT1120_IMG_PIX_ZONE_CTL		0x24
#define REG_BT1120_HSYNC_ZONE_CTL		0x28
#define REG_BT1120_VSYNC_ZONE_CTL		0x2c
#define REG_BT1120_DISP_WIDTH_CTL		0x30
#define REG_BT1120_DISP_HEIGHT_CTL		0x34
#define REG_BT1120_DISP_XZONE_CTL		0x38
#define REG_BT1120_DISP_YZONE_CTL		0x3c
#define REG_BT1120_HFP_RANGE_CTL		0x40
#define REG_BT1120_SCAN_FORMAT_CTL		0x44
/* Progressive scan (kernel bt1120_set_online_configs / IP default for HDMI path) */
#define BT1120_SCAN_FORMAT_PROGRESSIVE		0x1
#define REG_BT1120_INT_HEIGHT_OFFSET_CTL	0x48
#define REG_BT1120_DDR_STORE_FORMAT_CTL		0x4c
/*
 * Horizon X5 BT1120 Spec v0.5 §1.10.21 BT1120_IRQ_EN (0x50), §1.10.22 BT1120_IRQ_STATUS (0x54).
 * Bits 0–3: frame_start, dma_done, buf_underflow, online_mismatch (EN and STATUS use same layout).
 * TOC 0x2c/0x30/0x34 duplicates are wrong — those offsets are VSYNC_ZONE / DISP_WIDTH in this IP.
 */
#define REG_BT1120_IRQ_EN_CTL			0x50
#define REG_BT1120_IRQ_STATUS			0x54
#define BT1120_FRAME_START_IRQ_MASK		BIT(0)
#define BT1120_DMA_DONE_IRQ_MASK		BIT(1)
#define BT1120_BUF_UNDERFLOW_IRQ_MASK		BIT(2)
#define BT1120_ONLINE_MISMATCH_IRQ_MASK		BIT(3)
#define REG_BT1120_CSC_COEFF			0x58
#define REG_BT1120_OUTPUT_CRC_FRAME0		0x88
#define REG_BT1120_OUTPUT_CRC_COUNT		8
#define BT1120_CSC_COEFF_CNT	12
#define XSTOP_SHIFT		16
#define YSTOP_SHIFT		16

/* RGB->YUV BT.601 full range, same as kernel RGB2YUV601[] */
static const s16 rgb2yuv601[BT1120_CSC_COEFF_CNT] = {
	263, 516, 100, 16, -152, -298, 450, 128, 450, -377, -74, 128,
};

static inline void bt1120_write(void __iomem *base, u32 reg, u32 val)
{
	writel(val, base + reg);
}

static inline u32 bt1120_read(void __iomem *base, u32 reg)
{
	return readl(base + reg);
}

static inline void bt1120_set_clear(void __iomem *base, u32 reg, u32 set,
				     u32 clear)
{
	u32 v = bt1120_read(base, reg);

	v &= ~clear;
	v |= set;
	bt1120_write(base, reg, v);
}

static void bt1120_set_csc_coeff(void __iomem *base)
{
	u32 i;

	for (i = 0; i < BT1120_CSC_COEFF_CNT; i++)
		bt1120_write(base, REG_BT1120_CSC_COEFF + i * 4,
			     (u32)rgb2yuv601[i] & 0xffff);
}

/* Spec v0.5 §1.10.21–22; grep [BT1120_IRQ] to match kernel. */
void x5_bt1120_log_irq_status(void __iomem *base, const char *where)
{
	u32 en = bt1120_read(base, REG_BT1120_IRQ_EN_CTL);
	u32 st = bt1120_read(base, REG_BT1120_IRQ_STATUS);

	DISP_LOG_INFO("[BT1120_IRQ] %s: IRQ_EN(0x50)=0x%08x IRQ_STATUS(0x54)=0x%08x | "
		      "EN[fs,dma,uf,mm]=%u%u%u%u STS[fs,dma,uf,mm]=%u%u%u%u\n",
		      where, en, st,
		      !!(en & BT1120_FRAME_START_IRQ_MASK),
		      !!(en & BT1120_DMA_DONE_IRQ_MASK),
		      !!(en & BT1120_BUF_UNDERFLOW_IRQ_MASK),
		      !!(en & BT1120_ONLINE_MISMATCH_IRQ_MASK),
		      !!(st & BT1120_FRAME_START_IRQ_MASK),
		      !!(st & BT1120_DMA_DONE_IRQ_MASK),
		      !!(st & BT1120_BUF_UNDERFLOW_IRQ_MASK),
		      !!(st & BT1120_ONLINE_MISMATCH_IRQ_MASK));
}

void x5_bt1120_disable(void __iomem *base)
{
	u32 ctl = bt1120_read(base, REG_BT1120_CTL);

	X5_DISP_LOG_INFO("bt1120_disable: CTL before=0x%08x (clear bit0)\n", ctl);
	bt1120_set_clear(base, REG_BT1120_CTL, 0, BIT(0));
}

/*
 * HDMI bridge online path — match kernel bt1120_set_online_configs() exactly
 * (not bt1120_disp_update_scan() used by crtc-bt1120).
 *
 * @t->flags low nibble must match kernel drm_display_mode.flags (drm/modes.h):
 *   PHSYNC=BIT(0), NHSYNC=BIT(1), PVSYNC=BIT(2), NVSYNC=BIT(3).
 * U-Boot struct display_timing uses DISPLAY_FLAGS_* (different positions); callers
 * on the HDMI path must convert (see x5_output_hdmi.c display_timing_sync_to_drm_low_nibble).
 */
#define DRM_MODE_FLAG_PHSYNC	BIT(0)
#define DRM_MODE_FLAG_PVSYNC	BIT(2)

int x5_bt1120_set_online_mode(void __iomem *base, const struct display_timing *t)
{
	u32 hact;
	u32 hfp;
	u32 hsync;
	u32 hbp;
	u32 vact;
	u32 vfp;
	u32 vsync;
	u32 vbp;

	if (!base || !t)
		return -EINVAL;

	hact = t->hactive.typ;
	hfp = t->hfront_porch.typ;
	hsync = t->hsync_len.typ;
	hbp = t->hback_porch.typ;
	vact = t->vactive.typ;
	vfp = t->vfront_porch.typ;
	vsync = t->vsync_len.typ;
	vbp = t->vback_porch.typ;

	u32 hdisplay = hact;
	u32 hsync_start = hdisplay + hfp;
	u32 hsync_end = hsync_start + hsync;
	u32 htotal = hsync_end + hbp;

	u32 vdisplay = vact;
	u32 vsync_start = vdisplay + vfp;
	u32 vsync_end = vsync_start + vsync;
	u32 vtotal = vsync_end + vbp;
	u32 refresh_hz;
	unsigned long long px;

	struct {
		u16 h_sync_start;
		u16 h_sync_stop;
		u16 h_total;
		u16 h_active_start;
		u16 h_active_stop;
		u16 v_sync_start;
		u16 v_sync_stop;
		u16 v_total;
		u16 v_active_start;
		u16 v_active_stop;
		u8 hfp_range;
	} scan;

	px = t->pixelclock.typ;
	refresh_hz = htotal && vtotal ? (u32)(px / ((unsigned long long)htotal * vtotal)) : 0U;

	/* Kernel bt1120_bridge_get_input_bus_fmts(): upstream output_fmt is MEDIA_BUS_FMT_FIXED */
	X5_DISP_LOG_INFO("bt1120_bridge get_input_bus_fmts: crtc=crtc-dc8000 -> is_online=1 output_fmt=0x%x\n",
			 MEDIA_BUS_FMT_FIXED);
	X5_DISP_LOG_INFO("bt1120_bridge mode_set: is_online=1 %ux%u@%u dotclock_khz=%u htot=%u vtot=%u flags=0x%x hsync(%u-%u) vsync(%u-%u)\n",
			 hdisplay, vdisplay, refresh_hz, t->pixelclock.typ / 1000U,
			 htotal, vtotal, t->flags,
			 hsync_start, hsync_end, vsync_start, vsync_end);

	bt1120_write(base, REG_BT1120_DMA_BLENGTH_CTL, BT1120_DMA_BURST_LEN_DEFAULT);

	/* Online mode from DC8000 (kernel: is_online = true) */
	bt1120_set_clear(base, REG_BT1120_CTL, BIT(3), 0);

	scan.h_sync_start = 1;
	scan.h_sync_stop = scan.h_sync_start + (hsync_end - hsync_start);
	scan.h_total = htotal;
	scan.h_active_start = scan.h_sync_stop + (htotal - hsync_end);
	scan.h_active_stop = scan.h_active_start + hdisplay - 1;

	scan.v_sync_start = 1;
	scan.v_sync_stop = scan.v_sync_start + (vsync_end - vsync_start);
	scan.v_total = vtotal;
	scan.v_active_start = scan.v_sync_stop + (vtotal - vsync_end);
	scan.v_active_stop = scan.v_active_start + vdisplay - 1;

	if (scan.h_total - scan.h_active_stop > 0)
		scan.hfp_range = 1;
	else
		scan.hfp_range = 0;

	/* set_online_configs: HSYNC/VSYNC = start only (not packed stop<<16) */
	bt1120_write(base, REG_BT1120_HSYNC_ZONE_CTL, scan.h_sync_start);
	bt1120_write(base, REG_BT1120_VSYNC_ZONE_CTL, scan.v_sync_start);

	bt1120_write(base, REG_BT1120_DISP_XZONE_CTL,
		     scan.h_active_start | (scan.h_active_stop << XSTOP_SHIFT));
	bt1120_write(base, REG_BT1120_DISP_YZONE_CTL,
		     scan.v_active_start | (scan.v_active_stop << YSTOP_SHIFT));

	/*
	 * bt1120_set_online_configs() does not touch these, but after BT1120 reset
	 * they are often 0 — line/frame counters need htot/vtot (kernel dumps often
	 * show 0x898/0x465 = 2200/1125 from prior state). Program explicitly.
	 */
	bt1120_write(base, REG_BT1120_DISP_WIDTH_CTL, htotal);
	bt1120_write(base, REG_BT1120_DISP_HEIGHT_CTL, vtotal);

	bt1120_write(base, REG_BT1120_HFP_RANGE_CTL, scan.hfp_range);
	bt1120_write(base, REG_BT1120_SCAN_FORMAT_CTL, BT1120_SCAN_FORMAT_PROGRESSIVE);
	bt1120_write(base, REG_BT1120_INT_HEIGHT_OFFSET_CTL, 0x0);

	if (t->flags & DRM_MODE_FLAG_PHSYNC)
		bt1120_set_clear(base, REG_BT1120_CTL, BIT(5), 0);
	else
		bt1120_set_clear(base, REG_BT1120_CTL, 0, BIT(5));

	if (t->flags & DRM_MODE_FLAG_PVSYNC)
		bt1120_set_clear(base, REG_BT1120_CTL, BIT(6), 0);
	else
		bt1120_set_clear(base, REG_BT1120_CTL, 0, BIT(6));

	bt1120_set_csc_coeff(base);

	/* Enable BT1120 output */
	bt1120_set_clear(base, REG_BT1120_CTL, BIT(0), 0);

	X5_DISP_LOG_INFO("bt1120_set_online_configs: DC->HDMI online scan h ss=%u se=%u act=%u-%u tot=%u hfp_range=%u | v ss=%u se=%u act=%u-%u tot=%u | mode flags=0x%x\n",
			 scan.h_sync_start, scan.h_sync_stop,
			 scan.h_active_start, scan.h_active_stop, scan.h_total,
			 scan.hfp_range,
			 scan.v_sync_start, scan.v_sync_stop,
			 scan.v_active_start, scan.v_active_stop, scan.v_total,
			 t->flags);
	X5_DISP_LOG_INFO("bt1120_disp_enable: %ux%u@%u dotclock_khz=%u pix_clk_hz=%lu (after set_rate)\n",
			 hdisplay, vdisplay, refresh_hz, t->pixelclock.typ / 1000U,
			 (unsigned long)t->pixelclock.typ);
	X5_DISP_LOG_INFO("bt1120_disp_enable: CTL=0x%08x (bit0 en)\n",
			 bt1120_read(base, REG_BT1120_CTL));

	return 0;
}

/*
 * Full BT1120 snapshot (vs_bt1120_reg.h / X5 BT1120 spec). Same layout as kernel
 * bt1120_registers_dump_compare(); grep [BT1120_REGS] to diff.
 */
void x5_bt1120_log_timing_regs(void __iomem *base, const char *where,
			       const struct display_timing *t)
{
	u32 htot = t->hactive.typ + t->hfront_porch.typ + t->hsync_len.typ +
		   t->hback_porch.typ;
	u32 vtot = t->vactive.typ + t->vfront_porch.typ + t->vsync_len.typ +
		   t->vback_porch.typ;
	unsigned int i;
	char cscbuf[180];
	char crbuf[160];
	size_t pos = 0;
	size_t cpos = 0;

	DISP_LOG_INFO("[BT1120_REGS] %s: timing %ux%u dotclock_khz=%u pixclk_hz=%u flags=0x%x htot=%u vtot=%u\n",
		      where, t->hactive.typ, t->vactive.typ,
		      (unsigned int)(t->pixelclock.typ / 1000U), t->pixelclock.typ,
		      t->flags, htot, vtot);

	DISP_LOG_INFO("[BT1120_REGS] %s: CTL=0x%08x DMA_BL=0x%08x LOFF=0x%08x WOFF=0x%08x\n",
		      where,
		      bt1120_read(base, REG_BT1120_CTL),
		      bt1120_read(base, REG_BT1120_DMA_BLENGTH_CTL),
		      bt1120_read(base, REG_BT1120_LINE_OFFSET_CTL),
		      bt1120_read(base, REG_BT1120_WORD_OFFSET_CTL));

	DISP_LOG_INFO("[BT1120_REGS] %s: BADDR_Y=0x%08x BADDR_UV=0x%08x HSIZE=0x%08x VSIZE=0x%08x HSTRIDE=0x%08x PZONE=0x%08x\n",
		      where,
		      bt1120_read(base, REG_BT1120_IMG_IN_BADDR_Y_CTL),
		      bt1120_read(base, REG_BT1120_IMG_IN_BADDR_UV_CTL),
		      bt1120_read(base, REG_BT1120_IMG_PIX_HSIZE_CTL),
		      bt1120_read(base, REG_BT1120_IMG_PIX_VSIZE_CTL),
		      bt1120_read(base, REG_BT1120_IMG_PIX_HSTRIDE_CTL),
		      bt1120_read(base, REG_BT1120_IMG_PIX_ZONE_CTL));

	DISP_LOG_INFO("[BT1120_REGS] %s: HSYNC=0x%08x VSYNC=0x%08x DISP_W=0x%08x DISP_H=0x%08x XZ=0x%08x YZ=0x%08x HFP=0x%08x SCAN=0x%08x INTOFF=0x%08x DDR_FMT=0x%08x\n",
		      where,
		      bt1120_read(base, REG_BT1120_HSYNC_ZONE_CTL),
		      bt1120_read(base, REG_BT1120_VSYNC_ZONE_CTL),
		      bt1120_read(base, REG_BT1120_DISP_WIDTH_CTL),
		      bt1120_read(base, REG_BT1120_DISP_HEIGHT_CTL),
		      bt1120_read(base, REG_BT1120_DISP_XZONE_CTL),
		      bt1120_read(base, REG_BT1120_DISP_YZONE_CTL),
		      bt1120_read(base, REG_BT1120_HFP_RANGE_CTL),
		      bt1120_read(base, REG_BT1120_SCAN_FORMAT_CTL),
		      bt1120_read(base, REG_BT1120_INT_HEIGHT_OFFSET_CTL),
		      bt1120_read(base, REG_BT1120_DDR_STORE_FORMAT_CTL));

	x5_bt1120_log_irq_status(base, where);

	for (i = 0; i < BT1120_CSC_COEFF_CNT && pos < sizeof(cscbuf) - 6; i++)
		pos += snprintf(cscbuf + pos, sizeof(cscbuf) - pos, "%04x ",
				(unsigned int)(bt1120_read(base, REG_BT1120_CSC_COEFF + i * 4) & 0xffff));
	DISP_LOG_INFO("[BT1120_REGS] %s: CSC[12]=%s\n", where, cscbuf);

	for (i = 0; i < REG_BT1120_OUTPUT_CRC_COUNT && cpos < sizeof(crbuf) - 12; i++)
		cpos += snprintf(crbuf + cpos, sizeof(crbuf) - cpos, "%08x ",
				 bt1120_read(base, REG_BT1120_OUTPUT_CRC_FRAME0 + i * 4));
	DISP_LOG_INFO("[BT1120_REGS] %s: OUTPUT_CRC[0x88-0xa4 x8]=%s\n", where, crbuf);
}

/*
 * Dump all BT1120-related registers for U-Boot vs kernel comparison.
 * BT1120 MMIO: 0x00-0x54, CSC 0x58-0x84, OUTPUT_CRC 0x88-0xa4 (spec table 1).
 *
 * MMIO sizes (spec + vs_bt1120_reg.h / x5.dtsi):
 *   BT1120 @ 0x3e010000: defined regs 0x00–0xa7 (168 B, 42 u32); 0xa8+ may bus fault.
 *   disp_sys_con @ 0x3e0a0000: DTS 0x130 B (76 u32).
 * U-Boot md uses hex count:  md.l 0x3e010000 2a   md.l 0x3e0a0000 4c
 * (Do not use md.l ... 40 — 0x40 = 64 words and crosses into dead space at 0xa8.)
 *
 * In kernel, devmem examples:
 *   # SYSCON (spot checks; full window: devmem in a loop 0..0x12c step 4 or dump 76 words)
 *   for off in 10 14 54 58 5c 60 64 68 6c 70 74 78 7c 80 84 88 8c 90 94 98 9c a0; do
 *     devmem 0x3e0a00${off}; done
 *   # CRM clock generators
 *   devmem 0x34210098; devmem 0x3421009c
 *   devmem 0x342114a0; devmem 0x342114c0; devmem 0x34211500; devmem 0x34211520
 *   # DISP_PLL
 *   devmem 0x34210030; devmem 0x34210034; devmem 0x34210038; devmem 0x3421003c
 *   # NOC idle
 *   devmem 0x31032000; devmem 0x31032008
 */
void x5_bt1120_dump_all_regs(void __iomem *bt1120_base)
{
	void __iomem *base;
	u32 val;
	int i;

	DISP_DUMP_LOG_INFO("\n============ BT1120 Register Dump (U-Boot) ============\n");

	/* --- 1. BT1120 Module Registers (0x3E010000) --- */
	base = bt1120_base;
	DISP_DUMP_LOG_INFO("\n[BT1120 Module @ 0x3E010000]\n");
	DISP_DUMP_LOG_INFO("  CTL            (0x%02x) = 0x%08x\n", 0x00, bt1120_read(base, 0x00));
	DISP_DUMP_LOG_INFO("  DMA_BLENGTH    (0x%02x) = 0x%08x\n", 0x04, bt1120_read(base, 0x04));
	DISP_DUMP_LOG_INFO("  LINE_OFFSET    (0x%02x) = 0x%08x\n", 0x08, bt1120_read(base, 0x08));
	DISP_DUMP_LOG_INFO("  WORD_OFFSET    (0x%02x) = 0x%08x\n", 0x0c, bt1120_read(base, 0x0c));
	DISP_DUMP_LOG_INFO("  BADDR_Y        (0x%02x) = 0x%08x\n", 0x10, bt1120_read(base, 0x10));
	DISP_DUMP_LOG_INFO("  BADDR_UV       (0x%02x) = 0x%08x\n", 0x14, bt1120_read(base, 0x14));
	DISP_DUMP_LOG_INFO("  PIX_HSIZE      (0x%02x) = 0x%08x\n", 0x18, bt1120_read(base, 0x18));
	DISP_DUMP_LOG_INFO("  PIX_VSIZE      (0x%02x) = 0x%08x\n", 0x1c, bt1120_read(base, 0x1c));
	DISP_DUMP_LOG_INFO("  PIX_HSTRIDE    (0x%02x) = 0x%08x\n", 0x20, bt1120_read(base, 0x20));
	DISP_DUMP_LOG_INFO("  PIX_ZONE       (0x%02x) = 0x%08x\n", 0x24, bt1120_read(base, 0x24));
	DISP_DUMP_LOG_INFO("  HSYNC_ZONE     (0x%02x) = 0x%08x\n", 0x28, bt1120_read(base, 0x28));
	DISP_DUMP_LOG_INFO("  VSYNC_ZONE     (0x%02x) = 0x%08x\n", 0x2c, bt1120_read(base, 0x2c));
	DISP_DUMP_LOG_INFO("  DISP_WIDTH     (0x%02x) = 0x%08x\n", 0x30, bt1120_read(base, 0x30));
	DISP_DUMP_LOG_INFO("  DISP_HEIGHT    (0x%02x) = 0x%08x\n", 0x34, bt1120_read(base, 0x34));
	DISP_DUMP_LOG_INFO("  DISP_XZONE     (0x%02x) = 0x%08x\n", 0x38, bt1120_read(base, 0x38));
	DISP_DUMP_LOG_INFO("  DISP_YZONE     (0x%02x) = 0x%08x\n", 0x3c, bt1120_read(base, 0x3c));
	DISP_DUMP_LOG_INFO("  HFP_RANGE      (0x%02x) = 0x%08x\n", 0x40, bt1120_read(base, 0x40));
	DISP_DUMP_LOG_INFO("  SCAN_FORMAT    (0x%02x) = 0x%08x\n", 0x44, bt1120_read(base, 0x44));
	DISP_DUMP_LOG_INFO("  INT_H_OFFSET   (0x%02x) = 0x%08x\n", 0x48, bt1120_read(base, 0x48));
	DISP_DUMP_LOG_INFO("  DDR_STORE_FMT  (0x%02x) = 0x%08x\n", 0x4c, bt1120_read(base, 0x4c));
	DISP_DUMP_LOG_INFO("  BT1120_IRQ_EN     (0x50 spec §1.10.21) = 0x%08x\n",
	       bt1120_read(base, REG_BT1120_IRQ_EN_CTL));
	DISP_DUMP_LOG_INFO("  BT1120_IRQ_STATUS (0x54 spec §1.10.22) = 0x%08x\n",
	       bt1120_read(base, REG_BT1120_IRQ_STATUS));
	x5_bt1120_log_irq_status(base, "dump_all_regs");

	DISP_DUMP_LOG_INFO("\n  CSC_COEFF (0x58-0x84):\n");
	for (i = 0; i < BT1120_CSC_COEFF_CNT; i++)
		DISP_DUMP_LOG_INFO("    [0x%02x] = 0x%08x\n", REG_BT1120_CSC_COEFF + i * 4,
		       bt1120_read(base, REG_BT1120_CSC_COEFF + i * 4));

	DISP_DUMP_LOG_INFO("\n  OUTPUT_CRC (0x88-0xa4):\n");
	for (i = 0; i < REG_BT1120_OUTPUT_CRC_COUNT; i++)
		DISP_DUMP_LOG_INFO("    [0x%02x] = 0x%08x\n", REG_BT1120_OUTPUT_CRC_FRAME0 + i * 4,
		       bt1120_read(base, REG_BT1120_OUTPUT_CRC_FRAME0 + i * 4));

	/* --- 2. SYSCON Registers (0x3E0A0000) --- */
	base = (void __iomem *)0x3e0a0000UL;
	DISP_DUMP_LOG_INFO("\n[SYSCON @ 0x3E0A0000]\n");
	DISP_DUMP_LOG_INFO("  DC2CSI_DSI_EN  (0x%02x) = 0x%08x\n", 0x10, readl(base + 0x10));
	DISP_DUMP_LOG_INFO("  DC2BT1120_EN   (0x%02x) = 0x%08x\n", 0x14, readl(base + 0x14));
	DISP_DUMP_LOG_INFO("  PIN_MUX_CTRL0  (0x%02x) = 0x%08x\n", 0x9c, readl(base + 0x9c));
	DISP_DUMP_LOG_INFO("  PIN_MUX_CTRL1  (0x%02x) = 0x%08x\n", 0xa0, readl(base + 0xa0));

	/* --- 3. IO Ring Pad Registers (IOMUXC @ SYSCON+0x54) --- */
	DISP_DUMP_LOG_INFO("\n[IO Ring Pads @ 0x3E0A0054]\n");
	DISP_DUMP_LOG_INFO("  PIXELCLK_PAD   (0x%02x) = 0x%08x\n", 0x54, readl(base + 0x54));
	for (i = 0; i < 16; i++) {
		u32 off = 0x58 + i * 4;

		DISP_DUMP_LOG_INFO("  DATA%d_PAD%s   (0x%02x) = 0x%08x\n",
		       i, (i < 10) ? " " : "", off, readl(base + off));
	}
	DISP_DUMP_LOG_INFO("  MODE_SELECT    (0x%02x) = 0x%08x\n", 0x98, readl(base + 0x98));

	/* --- 4. CRM Clock Enables & Reset (0x34210000) --- */
	base = (void __iomem *)0x34210000UL;
	DISP_DUMP_LOG_INFO("\n[CRM @ 0x34210000]\n");
	val = readl(base + 0x98);
	DISP_DUMP_LOG_INFO("  HPS_MIX_CLK_ENB(0x%02x) = 0x%08x\n", 0x98, val);
	DISP_DUMP_LOG_INFO("    BT1120_PCLK=%d  BT1120_ACLK=%d  DC8000_PCLK=%d  DC8000_ACLK=%d\n",
	       !!(val & BIT(4)), !!(val & BIT(5)), !!(val & BIT(2)), !!(val & BIT(3)));
	DISP_DUMP_LOG_INFO("    SIF_ACLK=%d  SIF_PCLK=%d  DSI_PCLK=%d  DPHY_CFG=%d\n",
	       !!(val & BIT(7)), !!(val & BIT(8)), !!(val & BIT(1)), !!(val & BIT(6)));
	val = readl(base + 0x9c);
	DISP_DUMP_LOG_INFO("  HPS_MIX_SW_RST (0x%02x) = 0x%08x\n", 0x9c, val);
	DISP_DUMP_LOG_INFO("    BT1120_RST=%d  DC8000_2=%d  DC8000_3=%d  DSI_TX=%d\n",
	       !!(val & BIT(4)), !!(val & BIT(2)), !!(val & BIT(3)), !!(val & BIT(1)));

	/* --- 5. Clock Generators (0x34211000) --- */
	base = (void __iomem *)0x34211000UL;
	DISP_DUMP_LOG_INFO("\n[Clock Generators @ 0x34211000]\n");

	val = readl(base + 0x4a0);
	DISP_DUMP_LOG_INFO("  BT1120_PIX_CLK (0x4a0) = 0x%08x  [EN=%d MUX=%d PRE=%d POST=%d]\n",
	       val, !!(val & BIT(28)), (val >> 24) & 0x7,
	       ((val >> 16) & 0x7) + 1, (val & 0x3f) + 1);

	val = readl(base + 0x4c0);
	DISP_DUMP_LOG_INFO("  DC8000_PIX_CLK (0x4c0) = 0x%08x  [EN=%d MUX=%d PRE=%d POST=%d]\n",
	       val, !!(val & BIT(28)), (val >> 24) & 0x7,
	       ((val >> 16) & 0x7) + 1, (val & 0x3f) + 1);

	val = readl(base + 0x4e0);
	DISP_DUMP_LOG_INFO("  DISP_SIF_ACLK  (0x4e0) = 0x%08x  [EN=%d MUX=%d PRE=%d POST=%d]\n",
	       val, !!(val & BIT(28)), (val >> 24) & 0x7,
	       ((val >> 16) & 0x7) + 1, (val & 0x3f) + 1);

	val = readl(base + 0x500);
	DISP_DUMP_LOG_INFO("  BT1120_ACLK    (0x500) = 0x%08x  [EN=%d MUX=%d PRE=%d POST=%d]\n",
	       val, !!(val & BIT(28)), (val >> 24) & 0x7,
	       ((val >> 16) & 0x7) + 1, (val & 0x3f) + 1);

	val = readl(base + 0x520);
	DISP_DUMP_LOG_INFO("  DC8000_ACLK    (0x520) = 0x%08x  [EN=%d MUX=%d PRE=%d POST=%d]\n",
	       val, !!(val & BIT(28)), (val >> 24) & 0x7,
	       ((val >> 16) & 0x7) + 1, (val & 0x3f) + 1);

	/* --- 6. DISP_PLL Status (0x34210030) --- */
	base = (void __iomem *)0x34210000UL;
	DISP_DUMP_LOG_INFO("\n[DISP_PLL @ 0x34210030]\n");
	DISP_DUMP_LOG_INFO("  PLL_CFG        (0x%02x) = 0x%08x\n", 0x30, readl(base + 0x30));
	DISP_DUMP_LOG_INFO("  PLL_FBDIV      (0x%02x) = 0x%08x\n", 0x34, readl(base + 0x34));
	DISP_DUMP_LOG_INFO("  PLL_POSTDIV    (0x%02x) = 0x%08x\n", 0x38, readl(base + 0x38));
	val = readl(base + 0x3c);
	DISP_DUMP_LOG_INFO("  PLL_LOCK       (0x%02x) = 0x%08x  [locked=%d]\n", 0x3c, val, !!(val & BIT(0)));

	/* --- 7. NOC Idle Status (0x31032000) --- */
	base = (void __iomem *)0x31032000UL;
	DISP_DUMP_LOG_INFO("\n[NOC Idle @ 0x31032000]\n");
	val = readl(base + 0x00);
	DISP_DUMP_LOG_INFO("  NOC_IDLE_REQ   (0x%02x) = 0x%08x  [BT1120=%d DC8000=%d SIF=%d]\n",
	       0x00, val, !!(val & BIT(27)), !!(val & BIT(28)), !!(val & BIT(29)));
	val = readl(base + 0x08);
	DISP_DUMP_LOG_INFO("  NOC_IDLE_STS   (0x%02x) = 0x%08x  [BT1120=%d DC8000=%d SIF=%d]\n",
	       0x08, val, !!(val & BIT(27)), !!(val & BIT(28)), !!(val & BIT(29)));

	/* --- 8. BT1120 IOMMU (0x3E0A0144) --- */
	base = (void __iomem *)0x3e0a0144UL;
	DISP_DUMP_LOG_INFO("\n[BT1120 IOMMU @ 0x3E0A0144]\n");
	DISP_DUMP_LOG_INFO("  CTRL           (0x%02x) = 0x%08x\n", 0x00, readl(base + 0x00));
	DISP_DUMP_LOG_INFO("  MAP0           (0x%02x) = 0x%08x\n", 0x04, readl(base + 0x04));
	DISP_DUMP_LOG_INFO("  MAP1           (0x%02x) = 0x%08x\n", 0x08, readl(base + 0x08));
	DISP_DUMP_LOG_INFO("  MAP2           (0x%02x) = 0x%08x\n", 0x0c, readl(base + 0x0c));
	DISP_DUMP_LOG_INFO("  MAP3           (0x%02x) = 0x%08x\n", 0x10, readl(base + 0x10));

	/* --- 9. DC IOMMU (0x3E0A0130) --- */
	base = (void __iomem *)0x3e0a0130UL;
	DISP_DUMP_LOG_INFO("\n[DC IOMMU @ 0x3E0A0130]\n");
	DISP_DUMP_LOG_INFO("  CTRL           (0x%02x) = 0x%08x\n", 0x00, readl(base + 0x00));
	DISP_DUMP_LOG_INFO("  MAP0           (0x%02x) = 0x%08x\n", 0x04, readl(base + 0x04));
	DISP_DUMP_LOG_INFO("  MAP1           (0x%02x) = 0x%08x\n", 0x08, readl(base + 0x08));
	DISP_DUMP_LOG_INFO("  MAP2           (0x%02x) = 0x%08x\n", 0x0c, readl(base + 0x0c));
	DISP_DUMP_LOG_INFO("  MAP3           (0x%02x) = 0x%08x\n", 0x10, readl(base + 0x10));

	DISP_DUMP_LOG_INFO("\n========================================================\n");
	DISP_DUMP_LOG_INFO("To compare in kernel, use devmem to read the same addresses.\n");
	DISP_DUMP_LOG_INFO("========================================================\n\n");
}
