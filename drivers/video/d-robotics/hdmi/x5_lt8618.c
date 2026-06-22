// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * U-Boot port of kernel/drivers/gpu/drm/bridge/lontium-lt8618.c
 * (BT1120 / embedded-sync parallel input -> HDMI).
 *
 * Register map: page in high byte, sub-address in low byte; page select via 0xFF.
 */

#include <common.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <dm/ofnode.h>
#include <fdtdec.h>
#include <i2c.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <edid.h>
#include <panel.h>
#include <asm/gpio.h>
#include <asm/io.h>

#include "x5_lt8618.h"
#include <hb_display_log.h>

#define LT8618_REG_PAGE_SELECT		0xff
#define LT8618_REG_ENABLE		0x80ee
#define LT8618_REG_CHIP_ENABLE_MSK	BIT(0)
#define ENABLE_REG_BANK			0x01

#define LT8618_REG_INPUT_VIDEO_TYPE	0x800a
#define LT8618_REG_INT_CFG		0x8210
#define LT8618_REG_INPUT_DATA_LANE_SEQ	0x8245
#define LT8618_REG_INPUT_VIDEO_SYNC_GEN	0x8247
#define LT8618_REG_EMBEDDED_SYNC_MODE_INPUT_ENABLE 0x8248
#define LT8618_REG_INPUT_SIGNAL_SAMPLE_TYPE 0x824f
#define LT8618_REG_INPUT_SRC_SELECT	0x8250
#define LT8618_REG_VIDEO_CHECK_SELECT	0x8251
#define LT8618_REG_CHIP_VERSION_BASE	0x8000
#define LT8618_REG_INPUT_VIDEO_TIMING_BASE	0x8270
#define LT8618_INPUT_VIDEO_TIMING_LEN		18U
#define LT8618_REG_FREQ_METER2_BASE		0x821d
#define LT8618_FREQ_METER2_LEN			3U

#define RGB_BT1120			7
/* Must match kernel enum input_data_lane_seq: RBG = 6 (not 3 = BRG) */
#define DATA_LANE_SEQ_RBG		6
#define SDR_CLK				0
/* Match kernel lontium-lt8618.c — BT1120 path uses SDR, not DDR sample clock */
#define USE_DDR_CLK			0

#define LT8618_VER_U3			3

/* DDC / EDID — kernel drivers/gpu/drm/bridge/lontium-lt8618.c */
#define LT8618_REG_LINK_STATUS		0x825e
#define LT8618_LINK_OUT_DC_POS		0U

#define LT8618_REG_DDC_BASE		0x8502
#define LT8618_REG_DDC_CMD		0x8507
#define LT8618_REG_DDC_STATUS		0x8540
#define LT8618_REG_DDC_BUS_DONE_POS	1U
#define LT8618_REG_FIFO_CONTENT		0x8583
#define LT8618_FIFO_MAX_LENGTH		32U
#define LT8618_EDID_DDC_ADDR		0xa0

/* Single bridge instance; no concurrent access in U-Boot video init. */
static struct udevice *g_chip;
static struct gpio_desc g_reset;
static bool g_have_reset;
static bool g_probed;
static u8 g_chip_ver = LT8618_VER_U3;

static int lt8618_wr(u16 reg, u8 val)
{
	u8 page = reg >> 8;
	u8 off = reg & 0xff;
	int ret;

	if (!g_chip)
		return -ENODEV;

	ret = dm_i2c_write(g_chip, LT8618_REG_PAGE_SELECT, &page, 1);
	if (ret)
		return ret;
	return dm_i2c_write(g_chip, off, &val, 1);
}

static int lt8618_rd(u16 reg, u8 *val)
{
	u8 page = reg >> 8;
	u8 off = reg & 0xff;
	int ret;

	if (!g_chip || !val)
		return -ENODEV;

	ret = dm_i2c_write(g_chip, LT8618_REG_PAGE_SELECT, &page, 1);
	if (ret)
		return ret;
	return dm_i2c_read(g_chip, off, val, 1);
}

/* Same as kernel lt8618_get_hpd_status: enable I2C then read link */
static int lt8618_hpd_output_dc_high(void)
{
	u8 v;
	unsigned int i;

	/* Kernel lt8618_get_hpd_status: only enable I2C on 0x80ee */
	if (lt8618_wr(0x80ee, 0x01))
		return 0;
	mdelay(5);
	for (i = 0; i < 20; i++) {
		if (lt8618_rd(LT8618_REG_LINK_STATUS, &v))
			return 0;
		if (v & (1U << LT8618_LINK_OUT_DC_POS))
			return 1;
		mdelay(10);
	}
	return 0;
}

/*
 * One DDC burst: kernel lt8618_ddc_fifo_fetch() — EDID segment @ (block*128+offset).
 * @len must be <= 32; @offset_within_block 0..127 within the 128-byte EDID block.
 */
static int lt8618_ddc_fifo_fetch_edid(u8 block, u8 *dst, u8 len, u8 offset_within_block)
{
	u8 ddc_cfg[5];
	u8 ddc_status;
	unsigned int i, retry;
	u32 pos = (u32)offset_within_block + 128U * (u32)block;

	if (len > LT8618_FIFO_MAX_LENGTH || pos + len > 512U)
		return -EOVERFLOW;

	ddc_cfg[0] = 0x0a;
	ddc_cfg[1] = 0xc9;
	ddc_cfg[2] = LT8618_EDID_DDC_ADDR;
	ddc_cfg[3] = (u8)pos;
	ddc_cfg[4] = len;

	for (i = 0; i < ARRAY_SIZE(ddc_cfg); i++) {
		if (lt8618_wr(LT8618_REG_DDC_BASE + i, ddc_cfg[i]))
			return -EIO;
	}

	if (lt8618_wr(LT8618_REG_DDC_CMD, 0x36) ||
	    lt8618_wr(LT8618_REG_DDC_CMD, 0x34) ||
	    lt8618_wr(LT8618_REG_DDC_CMD, 0x37))
		return -EIO;

	ddc_status = 0;
	for (retry = 0; retry < 50; retry++) {
		mdelay(1);
		if (lt8618_rd(LT8618_REG_DDC_STATUS, &ddc_status))
			return -EIO;
		if (ddc_status & (1U << LT8618_REG_DDC_BUS_DONE_POS))
			break;
	}
	if (!(ddc_status & (1U << LT8618_REG_DDC_BUS_DONE_POS)))
		return -ETIMEDOUT;

	for (i = 0; i < len; i++) {
		u8 b;

		if (lt8618_rd(LT8618_REG_FIFO_CONTENT, &b))
			return -EIO;
		dst[i] = b;
	}
	return 0;
}

ssize_t x5_lt8618_edid_read(u8 *buf, size_t len)
{
	unsigned int chunk;
	int ret;

	if (!buf || len < 128)
		return -EINVAL;

	ret = x5_lt8618_probe_of();
	if (ret)
		return ret;

	if (!lt8618_hpd_output_dc_high()) {
		DISP_LOG_WARN("[LT8618][EDID] HPD/output DC low, DDC not attempted\n");
		return -ENODEV;
	}

	for (chunk = 0; chunk < 128; chunk += LT8618_FIFO_MAX_LENGTH) {
		ret = lt8618_ddc_fifo_fetch_edid(0, buf + chunk, LT8618_FIFO_MAX_LENGTH,
						 (u8)chunk);
		if (ret) {
			DISP_LOG_WARN("[LT8618][EDID] block0 @%u failed (%d)\n", chunk, ret);
			return ret;
		}
	}

	if (edid_check_checksum(buf)) {
		DISP_LOG_WARN("[LT8618][EDID] block0 checksum invalid\n");
		return -EIO;
	}

	if (!buf[126] || len < 256)
		return 128;

	for (chunk = 0; chunk < 128; chunk += LT8618_FIFO_MAX_LENGTH) {
		ret = lt8618_ddc_fifo_fetch_edid(1, buf + 128 + chunk,
						 LT8618_FIFO_MAX_LENGTH, (u8)chunk);
		if (ret) {
			DISP_LOG_WARN("[LT8618][EDID] block1 @%u failed (%d), using 128B\n",
				      chunk, ret);
			return 128;
		}
	}

	if (edid_check_checksum(buf + 128)) {
		DISP_LOG_WARN("[LT8618][EDID] block1 checksum invalid, using 128B\n");
		return 128;
	}

	return 256;
}

static void lt8618_reset_hw(void)
{
	if (!g_have_reset)
		return;

	dm_gpio_set_value(&g_reset, 1); /* assert reset (active-low → pin LOW) */
	mdelay(20);
	dm_gpio_set_value(&g_reset, 0); /* deassert reset (pin HIGH, chip runs) */
	mdelay(20);
}

static void lt8618_apply_seq(const u16 *regs, const u8 *vals, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		lt8618_wr(regs[i], vals[i]);
}

static const u16 sw_reset_r[] = { 0x8011, 0x8013, 0x8013 };
static const u8 sw_reset_v[] = { 0x00, 0xf1, 0xf9 };

/*
 * TTL input analog config - matches kernel lt8618_input_analog_seq[].
 * Note: 0x814e is NOT set in kernel (commented as "for U2" only).
 * Freq meter: kernel uses 0x6978 (27000 cycles), not 0x77ec.
 */
static const u16 in_analog_r[] = { 0x8102, 0x810a, 0x8115, 0x821b, 0x821c };
static const u8 in_analog_v[] = { 0x66, 0x06, 0x06, 0x69, 0x78 };

/*
 * Video output analog config - matches kernel lt8618_output_analog_seq[].
 * Only 4 registers; extra PLL regs are set in pll_cfg.
 */
static const u16 out_analog_r[] = { 0x8123, 0x8124, 0x8126, 0x8129 };
static const u8 out_analog_v[] = { 0x40, 0x62, 0x55, 0x04 };

static const u16 pll_cfg_r[] = { 0x82de, 0x82de, 0x8016, 0x8018,
				 0x8018, 0x8016 };
static const u8 pll_cfg_v[] = { 0x00, 0xc0, 0xf1, 0xdc, 0xfc, 0xf3 };

/* From kernel lt8618_pll_range_timing[0][lv][i], regs 0x8125 / 0x812c / 0x812d */
static const u8 pll_range_vals[3][3] = {
	{ 0x00, 0x9e, 0xaa },
	{ 0x00, 0x9e, 0x99 },
	{ 0x00, 0x9e, 0x88 },
};

/* Same register block as kernel lt8618_video_input_param() */
static void lt8618_video_input_param(void)
{
	lt8618_wr(LT8618_REG_INPUT_VIDEO_TYPE,
		  (u8)(RGB_BT1120 * 0x10 + 0x80));
	lt8618_wr(LT8618_REG_INPUT_VIDEO_SYNC_GEN,
		  (u8)((RGB_BT1120 & 0x3) * 0x40 + 0x07));
	lt8618_wr(LT8618_REG_INPUT_DATA_LANE_SEQ,
		  (u8)(DATA_LANE_SEQ_RBG * 0x10));
	lt8618_wr(LT8618_REG_INPUT_SIGNAL_SAMPLE_TYPE,
		  (u8)(SDR_CLK * 0x40));
	lt8618_wr(LT8618_REG_INPUT_SRC_SELECT, 0x00);
	lt8618_wr(LT8618_REG_VIDEO_CHECK_SELECT, 0x42);
	lt8618_wr(LT8618_REG_EMBEDDED_SYNC_MODE_INPUT_ENABLE, 0x08);
}

static void lt8618_video_input_cfg(void)
{
	u8 a, t, lane;

	lt8618_apply_seq(in_analog_r, in_analog_v, ARRAY_SIZE(in_analog_r));
	lt8618_video_input_param();
	lt8618_rd(0x8102, &a);
	lt8618_rd(0x800a, &t);
	lt8618_rd(0x8245, &lane);
	DISP_LOG_INFO("[LT8618] video_input_cfg: rd 8102=0x%02x 800a=0x%02x 8245=0x%02x (lt8618_input_analog_seq + BT1120/RBG)\n",
		      a, t, lane);
}

static void lt8618_hdmi_output_mode_hdmi(void)
{
	lt8618_wr(0x82d6, 0x0e);
}

static void lt8618_hdmi_csc(void)
{
	lt8618_wr(0x82b9, 0x00);
}

static void lt8618_audio_stub(void)
{
	lt8618_wr(0x82d6, 0x8e);
	lt8618_wr(0x82d7, 0x00);
	lt8618_wr(0x8406, 0x0c);
	lt8618_wr(0x8407, 0x10);
	lt8618_wr(0x840f, 0x2b);
	lt8618_wr(0x8434, 0xd4);
	lt8618_wr(0x8435, 0);
	lt8618_wr(0x8436, 0);
	lt8618_wr(0x8437, (u8)(6144 & 0xff));
	lt8618_wr(0x843c, 0x21);
}

static void lt8618_hdmi_config(void)
{
	lt8618_hdmi_csc();
	lt8618_hdmi_output_mode_hdmi();
	lt8618_audio_stub();
}

static void lt8618_video_output_analog(void)
{
	lt8618_apply_seq(out_analog_r, out_analog_v, ARRAY_SIZE(out_analog_r));
}

static void lt8618_video_output_pll_range(unsigned long pclk_hz)
{
	unsigned int lv = (unsigned int)(pclk_hz / 50000000UL);

	if (lv > 2)
		lv = 2;

	lt8618_wr(0x8125, pll_range_vals[lv][0]);
	lt8618_wr(0x812c, pll_range_vals[lv][1]);
	lt8618_wr(0x812d, pll_range_vals[lv][2]);
}

/*
 * PLL configuration - matches kernel lt8618_video_output_pll_cfg()
 * SDR_CLK path for U3 version.
 */
static void lt8618_video_output_pll_cfg(void)
{
	u32 i;
	u8 lock, cali_val, cali_done, val;

	/* pll_cfg initial: { 0x814d=0x09, 0x8127=0x60, 0x8128=0x88 } */
	lt8618_wr(0x814d, 0x09);
	lt8618_wr(0x8127, 0x60);
	lt8618_wr(0x8128, 0x88);

	/* sw_en_txpll_cal_en: clear BIT(1) of 0x812b */
	lt8618_rd(0x812b, &val);
	lt8618_wr(0x812b, val & 0xfd);

	/* sw_en_txpll_iband_set: clear BIT(0) of 0x812e */
	lt8618_rd(0x812e, &val);
	lt8618_wr(0x812e, val & 0xfe);

	/* txpll _sw_rst_n sequence */
	lt8618_apply_seq(pll_cfg_r, pll_cfg_v, ARRAY_SIZE(pll_cfg_r));

	/* U3 SDR_CLK: clk_type=0 -> 0x812a = 0x00, then 0x20 */
	if (g_chip_ver == LT8618_VER_U3) {
		lt8618_wr(0x812a, 0x00);
		lt8618_wr(0x812a, 0x20);
	}

	for (i = 0; i < 5; i++) {
		mdelay(100);
		/* PLL lock logic reset */
		lt8618_wr(0x8016, 0xe3);
		lt8618_wr(0x8016, 0xf3);
		lt8618_rd(0x8215, &lock);
		lt8618_rd(0x82ea, &cali_val);
		lt8618_rd(0x82eb, &cali_done);
		lock &= 0x80;
		cali_done &= 0x80;
		DISP_LOG_INFO("LT8618 PLL try %u: lock=0x%02x cali=0x%02x done=0x%02x\n",
			      i, lock, cali_val, cali_done);
		if (lock && cali_done && cali_val != 0xff) {
			DISP_LOG_INFO("LT8618: TXPLL Locked\n");
			break;
		}
		/* Retry: txpll sw_rst_n */
		lt8618_wr(0x8016, 0xf1);
		lt8618_wr(0x8018, 0xdc);
		lt8618_wr(0x8018, 0xfc);
		lt8618_wr(0x8016, 0xf3);
	}
	if (i == 5)
		DISP_LOG_WARN("LT8618 PLL lock failed after 5 attempts\n");
}

static void lt8618_avi_and_vsif(u8 vic, u8 avi_pb1, u8 avi_pb2)
{
	u8 avi_pb0;

	avi_pb0 = ((avi_pb1 + avi_pb2 + vic) <= 0x6f) ?
		  (0x6f - avi_pb1 - avi_pb2 - vic) :
		  (0x16f - avi_pb1 - avi_pb2 - vic);

	lt8618_wr(0x8443, avi_pb0);
	lt8618_wr(0x8444, avi_pb1);
	lt8618_wr(0x8445, avi_pb2);
	lt8618_wr(0x8447, vic);

	if (vic == 0x5f) {
		lt8618_wr(0x843d, 0x2a);
		lt8618_wr(0x8474, 0x81);
		lt8618_wr(0x8475, 0x01);
		lt8618_wr(0x8476, 0x05);
		lt8618_wr(0x8477, 0x49);
		lt8618_wr(0x8478, 0x03);
		lt8618_wr(0x8479, 0x0c);
		lt8618_wr(0x847a, 0x00);
		lt8618_wr(0x847b, 0x20);
		lt8618_wr(0x847c, 0x01);
	} else {
		lt8618_wr(0x843d, 0x0a);
	}
}

/*
 * drm_mode.h sync bits — low nibble of struct display_timing.flags on X5 matches
 * kernel drm_display_mode flags bits 0..3 (see x5_output_hdmi display_timing_sync_to_drm_low_nibble).
 * HSYNC_LOW|VSYNC_LOW encodes as numeric 0x5 same as DRM PHSYNC|PVSYNC; HSYNC_HIGH|HIGH -> 0xa.
 */
#define DRM_MODE_FLAG_PHSYNC	1U
#define DRM_MODE_FLAG_NHSYNC	2U
#define DRM_MODE_FLAG_PVSYNC	4U
#define DRM_MODE_FLAG_NVSYNC	8U

/*
 * modetest -M vs-drm -a -c — "modes:" lines under each connector
 * ==============================================================
 * Example:
 *   #0 800x480 60.00 800 844 932 1056 480 483 489 535 33900 flags: phsync, pvsync; type: preferred, driver
 *        |   |    |    |   |   |   |    |   |   |   |   |
 *        |   |    |    |   |   |   |    vdisp |   |   clock_kHz (last number before "flags:")
 *        |   |    refresh   hdisp hss hse htot      vss vse vtot
 *
 * Convert to struct display_timing (.typ), same fields as panel_jc050hd134.c:
 *   pixelclock.typ    = clock_khz * 1000
 *   hactive.typ       = hdisp
 *   hfront_porch.typ  = hss - hdisp
 *   hsync_len.typ     = hse - hss
 *   hback_porch.typ   = htot - hse
 *   vactive.typ       = vdisp
 *   vfront_porch.typ  = vss - vdisp
 *   vsync_len.typ     = vse - vss
 *   vback_porch.typ   = vtot - vse
 *
 * Polarity (same mapping as x5_output_hdmi.c display_timing_sync_to_drm_low_nibble):
 *   "phsync" -> DISPLAY_FLAGS_HSYNC_HIGH ; "nhsync" -> DISPLAY_FLAGS_HSYNC_LOW
 *   "pvsync" -> DISPLAY_FLAGS_VSYNC_HIGH ; "nvsync" -> DISPLAY_FLAGS_VSYNC_LOW
 *
 * More examples (paste from modetest and call x5_lt8618_timing_from_modetest_crtc):
 *   1920x1080 @ 60: 1920 2008 2052 2200 1080 1084 1089 1125 148500  phsync pvsync
 *   1280x720  @ 60: 1280 1390 1430 1650 720 725 730 750 74250      (often nhsync nvsync on HDMI)
 *   3840x2160 @ 30: 3840 4000 4104 4400 2160 2168 2178 2250 297000 phsync pvsync
 */
void x5_lt8618_timing_from_modetest_crtc(struct display_timing *t,
					 u32 clock_khz,
					 u32 hdisp, u32 hss, u32 hse, u32 htot,
					 u32 vdisp, u32 vss, u32 vse, u32 vtot,
					 bool phsync, bool pvsync)
{
	if (!t)
		return;

	memset(t, 0, sizeof(*t));
	t->pixelclock.typ = clock_khz * 1000U;
	t->hactive.typ = hdisp;
	t->hfront_porch.typ = hss - hdisp;
	t->hsync_len.typ = hse - hss;
	t->hback_porch.typ = htot - hse;
	t->vactive.typ = vdisp;
	t->vfront_porch.typ = vss - vdisp;
	t->vsync_len.typ = vse - vss;
	t->vback_porch.typ = vtot - vse;

	t->flags = (enum display_flags)0;
	if (phsync)
		t->flags |= DISPLAY_FLAGS_HSYNC_HIGH;
	else
		t->flags |= DISPLAY_FLAGS_HSYNC_LOW;
	if (pvsync)
		t->flags |= DISPLAY_FLAGS_VSYNC_HIGH;
	else
		t->flags |= DISPLAY_FLAGS_VSYNC_LOW;
}

static int lt8618_drm_match_cea_mode_vic(const struct display_timing *t)
{
	u32 hdisplay = t->hactive.typ;
	u32 hfp = t->hfront_porch.typ;
	u32 hsw = t->hsync_len.typ;
	u32 hbp = t->hback_porch.typ;
	u32 vdisplay = t->vactive.typ;
	u32 vfp = t->vfront_porch.typ;
	u32 vsw = t->vsync_len.typ;
	u32 vbp = t->vback_porch.typ;
	u32 hss = hdisplay + hfp;
	u32 hse = hss + hsw;
	u32 htot = hse + hbp;
	u32 vss = vdisplay + vfp;
	u32 vse = vss + vsw;
	u32 vtot = vse + vbp;
	u32 clock_khz = t->pixelclock.typ / 1000U;
	u32 drm_f = t->flags & 0xfU;

	/* CEA-861 VIC 16 — drm_edid.c edid_cea_modes[] */
	if (clock_khz != 148500U || hdisplay != 1920U || vdisplay != 1080U)
		return 0;
	if (hss != 2008U || hse != 2052U || htot != 2200U)
		return 0;
	if (vss != 1084U || vse != 1089U || vtot != 1125U)
		return 0;
	/* Modetest often prints phsync,pvsync (drm 0x5); kernel EDID path may use NHSYNC|NVSYNC (0xa). */
	if (drm_f != (DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) &&
	    drm_f != (DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC))
		return 0;
	return 16;
}

static void lt8618_timing_hw_regs_uboot(const char *where)
{
	u8 in_t[LT8618_INPUT_VIDEO_TIMING_LEN];
	u8 fm[LT8618_FREQ_METER2_LEN];
	u8 v45, v47, v48, v4f, v50, v51;
	char buf[80];
	unsigned int i;
	size_t pos = 0;

	if (!g_chip)
		return;

	lt8618_rd(LT8618_REG_INPUT_DATA_LANE_SEQ, &v45);
	lt8618_rd(LT8618_REG_INPUT_VIDEO_SYNC_GEN, &v47);
	lt8618_rd(LT8618_REG_EMBEDDED_SYNC_MODE_INPUT_ENABLE, &v48);
	lt8618_rd(LT8618_REG_INPUT_SIGNAL_SAMPLE_TYPE, &v4f);
	lt8618_rd(LT8618_REG_INPUT_SRC_SELECT, &v50);
	lt8618_rd(LT8618_REG_VIDEO_CHECK_SELECT, &v51);
	DISP_LOG_INFO("[LT8618_TIMING] %s: in_path 8245=%02x 8247=%02x 8248=%02x 824f=%02x 8250=%02x 8251=%02x\n",
		      where, v45, v47, v48, v4f, v50, v51);

	for (i = 0; i < LT8618_INPUT_VIDEO_TIMING_LEN; i++) {
		if (lt8618_rd(LT8618_REG_INPUT_VIDEO_TIMING_BASE + i, &in_t[i]))
			in_t[i] = 0xff;
	}
	for (i = 0; i < LT8618_INPUT_VIDEO_TIMING_LEN && pos < sizeof(buf) - 4; i++)
		pos += snprintf(buf + pos, sizeof(buf) - pos, "%02x ", in_t[i]);
	DISP_LOG_INFO("[LT8618_TIMING] %s: in_hw_timing 0x8270..0x8281 (%u B): %s\n",
		      where, LT8618_INPUT_VIDEO_TIMING_LEN, buf);

	for (i = 0; i < LT8618_FREQ_METER2_LEN; i++) {
		if (lt8618_rd(LT8618_REG_FREQ_METER2_BASE + i, &fm[i]))
			fm[i] = 0xff;
	}
	DISP_LOG_INFO("[LT8618_TIMING] %s: freq_meter 0x821d..0x821f: %02x %02x %02x\n",
		      where, fm[0], fm[1], fm[2]);
}

/*
 * Decode 0x8220..0x823d 30-byte pack (same packing as kernel lt8618_video_output_timing).
 */
static void lt8618_log_out_timing_pack_decode(const char *where, const u8 rb[30])
{
	u32 timing[15];
	unsigned int i;

	for (i = 0; i < 15; i++)
		timing[i] = ((u32)rb[i * 2] << 8) | rb[i * 2 + 1];

	DISP_LOG_INFO("[LT8618_TIMING] %s: out_pack_decode hact=%u hfp=%u hsw=%u rsv=%u,%u,%u htot=%u | "
		      "hact_b=%u hfp_b=%u hbp=%u hsw_b=%u | vact=%u vfp=%u vbp=%u vsw=%u\n",
		      where,
		      timing[0], timing[1], timing[2], timing[3], timing[4], timing[5], timing[6],
		      timing[7], timing[8], timing[9], timing[10],
		      timing[11], timing[12], timing[13], timing[14]);
}

/*
 * Lontium internal pattern block 0x82a3..0x82b0 (14 B), same packing as
 * lontium-lt8618-kernel-pattern.c test_pattern() / vendor comments:
 *   a3,a4 BE: hsync_len + hback_porch
 *   a5:       vsync_len + vback_porch  (one byte; must be <= 255)
 *   a6,a7 BE: hactive
 *   a8,a9 BE: vactive
 *   aa,ab BE: htotal
 *   ac,ad BE: vtotal
 *   ae,af BE: hsync_len
 *   b0:       vsync_len
 *
 * Used only with internal pattern (0x824f/0x8250 + 0x8326/0x832d). Normal HDMI
 * mode programs 0x8220..0x823d via lt8618_video_output_timing() instead.
 */
static void lt8618_pattern_regs_pack(const struct display_timing *t, u8 regs[14])
{
	u32 hsw = t->hsync_len.typ;
	u32 hbp = t->hback_porch.typ;
	u32 vsw = t->vsync_len.typ;
	u32 vbp = t->vback_porch.typ;
	u32 hact = t->hactive.typ;
	u32 vact = t->vactive.typ;
	u32 htot = hact + t->hfront_porch.typ + hsw + hbp;
	u32 vtot = vact + t->vfront_porch.typ + vsw + vbp;
	u32 hs_hbp = hsw + hbp;
	u32 vs_vbp = vsw + vbp;

	regs[0] = (hs_hbp >> 8) & 0xff;
	regs[1] = hs_hbp & 0xff;
	regs[2] = vs_vbp & 0xff;
	regs[3] = (hact >> 8) & 0xff;
	regs[4] = hact & 0xff;
	regs[5] = (vact >> 8) & 0xff;
	regs[6] = vact & 0xff;
	regs[7] = (htot >> 8) & 0xff;
	regs[8] = htot & 0xff;
	regs[9] = (vtot >> 8) & 0xff;
	regs[10] = vtot & 0xff;
	regs[11] = (hsw >> 8) & 0xff;
	regs[12] = hsw & 0xff;
	regs[13] = vsw & 0xff;
}

static int lt8618_pattern_timing_fits(const struct display_timing *t)
{
	u32 hs_hbp = t->hsync_len.typ + t->hback_porch.typ;
	u32 vs_vbp = t->vsync_len.typ + t->vback_porch.typ;

	if (hs_hbp > 65535U) {
		DISP_LOG_WARN("LT8618 pattern: hs+hbp=%u overflow (>65535)\n", hs_hbp);
		return 0;
	}
	if (vs_vbp > 255U) {
		DISP_LOG_WARN("LT8618 pattern: vs+vbp=%u overflow (>255, register a5 is 8-bit)\n",
			      vs_vbp);
		return 0;
	}
	return 1;
}

static u8 lt8618_pattern_pll_812d_for_pc(unsigned long pixelclock_hz)
{
	unsigned long khz = pixelclock_hz / 1000UL;

	if (khz < 50000UL)
		return 0xaa;
	if (khz < 100000UL)
		return 0x99;
	return 0x88;
}

static void lt8618_fill_default_pattern_timing(struct display_timing *t)
{
	/*
	 * CEA 1920x1080@60 — numbers copied from modetest line:
	 *   1920x1080 60.00 1920 2008 2052 2200 1080 1084 1089 1125 148500 flags: phsync, pvsync
	 */
	x5_lt8618_timing_from_modetest_crtc(t, 148500U, 1920U, 2008U, 2052U, 2200U,
					    1080U, 1084U, 1089U, 1125U, true, true);
}

void x5_lt8618_get_default_timing(struct display_timing *t)
{
	lt8618_fill_default_pattern_timing(t);
}

/*
 * After internal pattern enable: pattern block 0x82a3..0x82b0 (+2 extra reads),
 * HDMI path 0x8220..0x823d, pattern clock / clk sel / AVI — then in_path +
 * in_hw_timing + freq_meter (kernel style).
 */
static void lt8618_pattern_timing_dump_uboot(const char *where,
					     const struct display_timing *expect)
{
	u8 rb_pat[16];
	u8 rb_out[30];
	u8 pc26, pc2d, csel4f, csel50;
	u8 avi0, avi1, avi2, vic;
	u16 hs_hbp, hact, vact, ht, vt, hsw;
	u8 vs_vbp, vsw;
	unsigned int i;
	char lpat[64];
	char lout[120];
	size_t pos;
	u8 exp[14];

	if (!g_chip)
		return;

	for (i = 0; i < sizeof(rb_pat); i++) {
		if (lt8618_rd(0x82a3 + i, &rb_pat[i]))
			rb_pat[i] = 0xff;
	}
	hs_hbp = ((u16)rb_pat[0] << 8) | rb_pat[1];
	vs_vbp = rb_pat[2];
	hact = ((u16)rb_pat[3] << 8) | rb_pat[4];
	vact = ((u16)rb_pat[5] << 8) | rb_pat[6];
	ht = ((u16)rb_pat[7] << 8) | rb_pat[8];
	vt = ((u16)rb_pat[9] << 8) | rb_pat[10];
	hsw = ((u16)rb_pat[11] << 8) | rb_pat[12];
	vsw = rb_pat[13];

	if (expect) {
		u32 xht = expect->hactive.typ + expect->hfront_porch.typ +
			  expect->hsync_len.typ + expect->hback_porch.typ;
		u32 xvt = expect->vactive.typ + expect->vfront_porch.typ +
			  expect->vsync_len.typ + expect->vback_porch.typ;

		DISP_LOG_INFO("[LT8618_TIMING] %s: pattern expect %ux%u @%lu Hz htot=%u vtot=%u\n",
			      where, expect->hactive.typ, expect->vactive.typ,
			      (unsigned long)expect->pixelclock.typ, xht, xvt);
		lt8618_pattern_regs_pack(expect, exp);
		DISP_LOG_INFO("[LT8618_TIMING] %s: pattern packed (14 B expect): "
			      "hs+hbp=%u vs+vbp=%u hact=%u vact=%u htot=%u vtot=%u hs=%u vs=%u\n",
			      where,
			      (unsigned int)(expect->hsync_len.typ + expect->hback_porch.typ),
			      (unsigned int)(expect->vsync_len.typ + expect->vback_porch.typ),
			      expect->hactive.typ, expect->vactive.typ, xht, xvt,
			      expect->hsync_len.typ, expect->vsync_len.typ);
	}

	DISP_LOG_INFO("[LT8618_TIMING] %s: pattern_regs 0x82a3..0x82b0 decode (vendor): "
		      "hs+hbp=%u vs+vbp=%u hact=%u vact=%u htot=%u vtot=%u hs=%u vs=%u\n",
		      where, (unsigned int)hs_hbp, (unsigned int)vs_vbp,
		      (unsigned int)hact, (unsigned int)vact, (unsigned int)ht, (unsigned int)vt,
		      (unsigned int)hsw, (unsigned int)vsw);

	for (i = 0, pos = 0; i < 14 && pos < sizeof(lpat) - 4; i++)
		pos += snprintf(lpat + pos, sizeof(lpat) - pos, "%02x ", rb_pat[i]);
	DISP_LOG_INFO("[LT8618_TIMING] %s: pattern_regs 0x82a3..0x82b0 (14 B): %s\n",
		      where, lpat);
	if (expect) {
		DISP_LOG_INFO("[LT8618_TIMING] %s: pattern_regs expect 14 B: "
			      "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			      where, exp[0], exp[1], exp[2], exp[3], exp[4], exp[5], exp[6],
			      exp[7], exp[8], exp[9], exp[10], exp[11], exp[12], exp[13]);
	}
	DISP_LOG_INFO("[LT8618_TIMING] %s: pattern_regs 0x82b1..0x82b2 (extra): %02x %02x\n",
		      where, rb_pat[14], rb_pat[15]);

	for (i = 0; i < sizeof(rb_out); i++) {
		if (lt8618_rd(0x8220 + i, &rb_out[i]))
			rb_out[i] = 0xff;
	}
	for (i = 0, pos = 0; i < sizeof(rb_out) && pos < sizeof(lout) - 4; i++)
		pos += snprintf(lout + pos, sizeof(lout) - pos, "%02x ", rb_out[i]);
	DISP_LOG_INFO("[LT8618_TIMING] %s: out_timing_pack 0x8220..0x823d (%zu B): %s\n",
		      where, sizeof(rb_out), lout);
	lt8618_log_out_timing_pack_decode(where, rb_out);

	lt8618_rd(0x8326, &pc26);
	lt8618_rd(0x832d, &pc2d);
	lt8618_rd(0x824f, &csel4f);
	lt8618_rd(0x8250, &csel50);
	DISP_LOG_INFO("[LT8618_TIMING] %s: pattern_clk 0x8326=0x%02x 0x832d=0x%02x clk_sel 0x824f=0x%02x 0x8250=0x%02x\n",
		      where, pc26, pc2d, csel4f, csel50);

	lt8618_rd(0x8443, &avi0);
	lt8618_rd(0x8444, &avi1);
	lt8618_rd(0x8445, &avi2);
	lt8618_rd(0x8447, &vic);
	DISP_LOG_INFO("[LT8618_TIMING] %s: AVI PB0=0x%02x PB1=0x%02x PB2=0x%02x VIC=0x%02x (kernel BT1120 path often PB1=0x30 YUV422)\n",
		      where, avi0, avi1, avi2, vic);

	lt8618_timing_hw_regs_uboot(where);
}

/* Match kernel lt8618_timing_dump() — fields + 0x8220..0x823d readback */
static void lt8618_timing_dump_uboot(const char *where, const struct display_timing *t,
				     const u32 timing[15])
{
	u8 rb[30];
	unsigned int i;
	char buf[240];
	size_t pos = 0;
	u32 vtot = t->vactive.typ + t->vfront_porch.typ + t->vsync_len.typ +
		   t->vback_porch.typ;

	DISP_LOG_INFO("[LT8618_TIMING] %s: timing %ux%u dotclock_khz=%u pixclk_hz=%u htot=%u vtot=%u flags=0x%x\n",
		      where, t->hactive.typ, t->vactive.typ,
		      (unsigned int)(t->pixelclock.typ / 1000U), t->pixelclock.typ,
		      timing[6], vtot, t->flags);

	DISP_LOG_INFO("[LT8618_TIMING] %s: fields hact=%u hfp=%u hsw=%u rsv=%u,%u,%u htot=%u | hact_b=%u hfp_b=%u hbp=%u hsw_b=%u | vact=%u vfp=%u vbp=%u vsw=%u\n",
		      where,
		      timing[0], timing[1], timing[2], timing[3], timing[4], timing[5],
		      timing[6], timing[7], timing[8], timing[9], timing[10],
		      timing[11], timing[12], timing[13], timing[14]);

	for (i = 0; i < 30; i++) {
		if (lt8618_rd(0x8220 + i, &rb[i]))
			rb[i] = 0xff;
	}
	for (i = 0; i < 30 && pos < sizeof(buf) - 4; i++)
		pos += snprintf(buf + pos, sizeof(buf) - pos, "%02x ", rb[i]);
	DISP_LOG_INFO("[LT8618_TIMING] %s: out_timing_pack 0x8220..0x823d (30 B): %s\n",
		      where, buf);

	lt8618_timing_hw_regs_uboot(where);
}

/*
 * HDMI output timing for normal (BT1120 pixel-in -> TMDS) path: 15x u16 DRM-style
 * fields packed MSB-first into 0x8220..0x823d. Matches kernel
 * lt8618_video_output_timing() — not the internal pattern block at 0x82a3..0xb0
 * (vendor layout, only valid with internal pattern clock / test_pattern).
 */
static void lt8618_video_output_timing(const struct display_timing *t)
{
	u32 hact = t->hactive.typ;
	u32 hfp = t->hfront_porch.typ;
	u32 hsw = t->hsync_len.typ;
	u32 hbp = t->hback_porch.typ;
	u32 vact = t->vactive.typ;
	u32 vfp = t->vfront_porch.typ;
	u32 vsw = t->vsync_len.typ;
	u32 vbp = t->vback_porch.typ;
	u32 htot = hact + hfp + hsw + hbp;
	u32 timing[15];
	u8 pack[30];
	unsigned int i;

	/* Same field order as kernel lt8618_video_output_timing() */
	timing[0] = hact;
	timing[1] = hfp;
	timing[2] = hsw;
	timing[3] = 0;
	timing[4] = 0;
	timing[5] = 0;
	timing[6] = htot;
	timing[7] = hact;
	timing[8] = hfp;
	timing[9] = hbp;
	timing[10] = hsw;
	timing[11] = vact;
	timing[12] = vfp;
	timing[13] = vbp;
	timing[14] = vsw;

	for (i = 0; i < ARRAY_SIZE(pack); i++)
		pack[i] = (i % 2) ? (timing[i / 2] & 0xff) :
				    ((timing[i / 2] >> 8) & 0xff);

	for (i = 0; i < ARRAY_SIZE(pack); i++)
		lt8618_wr(0x8220 + i, pack[i]);

	lt8618_timing_dump_uboot("video_output_timing", t, timing);
}

static void lt8618_afe_high(void)
{
	lt8618_wr(0x8130, 0xea);
	lt8618_wr(0x8131, 0x44);
	lt8618_wr(0x8132, 0x4a);
	lt8618_wr(0x8133, 0x0b);
	lt8618_wr(0x8134, 0x00);
	lt8618_wr(0x8135, 0x00);
	lt8618_wr(0x8136, 0x00);
	lt8618_wr(0x8137, 0x44);
	lt8618_wr(0x813f, 0x0f);
	lt8618_wr(0x8140, 0xb0);
	lt8618_wr(0x8141, 0x68);
	lt8618_wr(0x8142, 0x68);
	lt8618_wr(0x8143, 0x68);
	lt8618_wr(0x8144, 0x0a);
}

/*
 * Parallel input sampling phase — port of kernel lt8618_phase_config().
 * Scans 0x8127 0x60..0x69, toggles 0x814d, reads 0x8150 to pick best window.
 */
static int lt8618_phase_config(void)
{
	u8 temp;
	unsigned int read_val;
	u8 OK_CNT = 0x00;
	u8 OK_CNT_1 = 0x00;
	u8 OK_CNT_2 = 0x00;
	u8 OK_CNT_3 = 0x00;
	u8 Jump_CNT = 0x00;
	u8 Jump_Num = 0x00;
	u8 Jump_Num_1 = 0x00;
	u8 Jump_Num_2 = 0x00;
	u8 Jump_Num_3 = 0x00;
	int temp0_ok = 0;
	int temp9_ok = 0;
	int b_OK = 0;
	u8 rb;

	lt8618_wr(0x8013, 0xf1);
	mdelay(5);
	lt8618_wr(0x8013, 0xf9);
	mdelay(10);

	for (temp = 0; temp < 0x0a; temp++) {
		lt8618_wr(0x8127, (u8)(0x60 + temp));

		if (USE_DDR_CLK) {
			lt8618_wr(0x814d, 0x05);
			mdelay(5);
			lt8618_wr(0x814d, 0x0d);
		} else {
			lt8618_wr(0x814d, 0x01);
			mdelay(5);
			lt8618_wr(0x814d, 0x09);
		}
		mdelay(10);
		if (lt8618_rd(0x8150, &rb))
			read_val = 1;
		else
			read_val = (unsigned int)(rb & 0x01);

		if (read_val == 0) {
			OK_CNT++;

			if (b_OK == 0) {
				b_OK = 1;
				Jump_CNT++;

				if (Jump_CNT == 1) {
					Jump_Num_1 = temp;
				} else if (Jump_CNT == 3) {
					Jump_Num_2 = temp;
				} else if (Jump_CNT == 5) {
					Jump_Num_3 = temp;
				}
			}

			if (Jump_CNT == 1) {
				OK_CNT_1++;
			} else if (Jump_CNT == 3) {
				OK_CNT_2++;
			} else if (Jump_CNT == 5) {
				OK_CNT_3++;
			}

			if (temp == 0)
				temp0_ok = 1;
			if (temp == 9) {
				Jump_CNT++;
				temp9_ok = 1;
			}
		} else {
			if (b_OK) {
				b_OK = 0;
				Jump_CNT++;
			}
		}
	}

	if (Jump_CNT == 0 || Jump_CNT > 6) {
		DISP_LOG_WARN("LT8618: phase_config cali fail (Jump_CNT=%u)\n",
			      (unsigned int)Jump_CNT);
		return 0;
	}

	if (temp9_ok == 1 && temp0_ok == 1) {
		if (Jump_CNT == 6) {
			OK_CNT_3 = OK_CNT_3 + OK_CNT_1;
			OK_CNT_1 = 0;
		} else if (Jump_CNT == 4) {
			OK_CNT_2 = OK_CNT_2 + OK_CNT_1;
			OK_CNT_1 = 0;
		}
	}
	if (Jump_CNT >= 2) {
		if (OK_CNT_1 >= OK_CNT_2) {
			if (OK_CNT_1 >= OK_CNT_3) {
				OK_CNT = OK_CNT_1;
				Jump_Num = Jump_Num_1;
			} else {
				OK_CNT = OK_CNT_3;
				Jump_Num = Jump_Num_3;
			}
		} else {
			if (OK_CNT_2 >= OK_CNT_3) {
				OK_CNT = OK_CNT_2;
				Jump_Num = Jump_Num_2;
			} else {
				OK_CNT = OK_CNT_3;
				Jump_Num = Jump_Num_3;
			}
		}
	}

	if (USE_DDR_CLK)
		lt8618_wr(0x814d, 0x0d);
	else
		lt8618_wr(0x814d, 0x09);

	if (Jump_CNT == 2 || Jump_CNT == 4 || Jump_CNT == 6) {
		lt8618_wr(0x8127, (u8)(0x60 + (Jump_Num + (OK_CNT / 2)) % 0x0a));
	} else if (OK_CNT >= 0x09) {
		lt8618_wr(0x8127, 0x65);
	}

	DISP_LOG_INFO("LT8618: phase_config done (Jump_CNT=%u)\n",
		      (unsigned int)Jump_CNT);
	return 1;
}

/* Same unlock + chip version read as init_for_pattern (no global state). */
static int lt8618_i2c_read_chip_version(struct udevice *chip, u8 ver[3])
{
	u8 unlock = 0x01;
	int ret;

	ret = dm_i2c_write(chip, LT8618_REG_ENABLE, &unlock, 1);
	if (ret)
		return ret;
	ver[0] = ver[1] = ver[2] = 0;
	ret = dm_i2c_read(chip, LT8618_REG_CHIP_VERSION_BASE + 0, &ver[0], 1);
	if (ret)
		return ret;
	ret = dm_i2c_read(chip, LT8618_REG_CHIP_VERSION_BASE + 1, &ver[1], 1);
	if (ret)
		return ret;
	return dm_i2c_read(chip, LT8618_REG_CHIP_VERSION_BASE + 2, &ver[2], 1);
}

static bool lt8618_chip_version_is_expected(const u8 ver[3])
{
	return ver[0] == 0x17 && ver[1] == 0x02;
}

static void lt8618_chip_present_reset_pulse(struct gpio_desc *rst)
{
	dm_gpio_set_value(rst, 1);
	mdelay(20);
	dm_gpio_set_value(rst, 0);
	mdelay(20);
}

int x5_lt8618_chip_present(void)
{
	ofnode node, busnode;
	struct udevice *bus, *chip;
	struct gpio_desc rst;
	u32 addr;
	u8 ver[3];
	int ret, gpio_ret;

	node = ofnode_by_compatible(ofnode_null(), "lontium,lt8618");
	if (!ofnode_valid(node) || !ofnode_is_enabled(node))
		return -ENODEV;

	busnode = ofnode_get_parent(node);
	if (!ofnode_valid(busnode))
		return -ENODEV;

	ret = uclass_get_device_by_ofnode(UCLASS_I2C, busnode, &bus);
	if (ret)
		return ret;
	/* Auto-init / EDID runs before some buses are activated — match probe_of timing. */
	if (!device_active(bus)) {
		ret = device_probe(bus);
		if (ret)
			return ret;
	}

	addr = ofnode_read_u32_default(node, "reg", 0x3b);
	ret = i2c_get_chip(bus, addr, 1, &chip);
	if (ret)
		return ret;

	/*
	 * Match x5_lt8618_init_chip: lt8618_reset_hw() before 0x80ee. Probing 0x80ee
	 * first while the chip is held in reset yields I2C errors even though TX init
	 * later works after reset.
	 */
	memset(&rst, 0, sizeof(rst));
	gpio_ret = gpio_request_by_name_nodev(node, "reset-gpios", 0, &rst,
					      GPIOD_IS_OUT);
	if (!gpio_ret) {
		lt8618_chip_present_reset_pulse(&rst);
		gpio_free_list_nodev(&rst, 1);
	}

	ret = lt8618_i2c_read_chip_version(chip, ver);
	/* Quiet bus / pull-ups often read 0x00 — not a valid LT8618 signature. */
	if (!ret && ver[0] == 0 && ver[1] == 0 && ver[2] == 0)
		return -ENODEV;
	if (!ret && lt8618_chip_version_is_expected(ver))
		return 0;

	LT8618_LOG_DEBUG("chip_present: ID ret=%d ver=%02x %02x %02x had_reset_gpio=%d\n",
			 ret, ver[0], ver[1], ver[2], !gpio_ret);
	return -ENODEV;
}

int x5_lt8618_probe_of(void)
{
	ofnode node, busnode;
	struct udevice *bus;
	u32 addr;
	int ret;

	if (g_probed)
		return g_chip ? 0 : -ENODEV;

	node = ofnode_by_compatible(ofnode_null(), "lontium,lt8618");
	if (!ofnode_valid(node) || !ofnode_is_enabled(node)) {
		DISP_LOG_WARN("LT8618: no enabled lontium,lt8618 node\n");
		return -ENODEV;
	}

	busnode = ofnode_get_parent(node);
	if (!ofnode_valid(busnode)) {
		DISP_LOG_WARN("LT8618: no parent I2C bus\n");
		return -ENODEV;
	}

	ret = uclass_get_device_by_ofnode(UCLASS_I2C, busnode, &bus);
	if (ret) {
		DISP_LOG_WARN("LT8618: I2C bus probe failed (%d)\n", ret);
		return ret;
	}
	if (!device_active(bus)) {
		ret = device_probe(bus);
		if (ret) {
			DISP_LOG_WARN("LT8618: I2C bus device_probe failed (%d)\n", ret);
			return ret;
		}
	}

	addr = ofnode_read_u32_default(node, "reg", 0x3b);
	ret = i2c_get_chip(bus, addr, 1, &g_chip);
	if (ret) {
		DISP_LOG_WARN("LT8618: i2c_get_chip 0x%x failed (%d)\n", addr,
			      ret);
		g_chip = NULL;
		return ret;
	}

	ret = gpio_request_by_name_nodev(node, "reset-gpios", 0, &g_reset,
					 GPIOD_IS_OUT);
	if (ret) {
		g_have_reset = false;
		DISP_LOG_WARN("LT8618: reset-gpios optional (%d)\n", ret);
	} else {
		g_have_reset = true;
	}

	g_probed = true;
	return 0;
}

int x5_lt8618_sink_connected(void)
{
	int ret;

	ret = x5_lt8618_probe_of();
	if (ret)
		return ret;

	/* Match init_chip: release reset before 0x80ee / link status (see chip_present comment). */
	lt8618_reset_hw();

	return lt8618_hpd_output_dc_high() ? 1 : 0;
}

int x5_lt8618_init_chip(void)
{
	u8 ver[3];
	int ret;

	ret = x5_lt8618_probe_of();
	if (ret)
		return ret;

	lt8618_reset_hw();

	lt8618_wr(0x80ee, 0x01);

	ver[0] = ver[1] = ver[2] = 0;
	lt8618_rd(LT8618_REG_CHIP_VERSION_BASE + 0, &ver[0]);
	lt8618_rd(LT8618_REG_CHIP_VERSION_BASE + 1, &ver[1]);
	lt8618_rd(LT8618_REG_CHIP_VERSION_BASE + 2, &ver[2]);
	if ((ver[0] == 0 && ver[1] == 0 && ver[2] == 0) ||
	    ver[0] != 0x17 || ver[1] != 0x02) {
		DISP_LOG_WARN("LT8618: invalid chip version %02x %02x %02x — no LT8618 on I2C (check board / DT)\n",
			      ver[0], ver[1], ver[2]);
		return -EIO;
	}
	DISP_LOG_INFO("LT8618 version %02x %02x %02x\n", ver[0], ver[1],
		      ver[2]);

	lt8618_wr(LT8618_REG_ENABLE, ENABLE_REG_BANK);

	lt8618_apply_seq(sw_reset_r, sw_reset_v, ARRAY_SIZE(sw_reset_r));

	lt8618_video_input_cfg();
	lt8618_hdmi_config();

	return 0;
}

/*
 * Minimal init for pattern mode - matches reference test_pattern() exactly:
 *   LT8618SX_Chip_ID() + LT8618SX_RST_PD_Init() + LT8618SX_TTL_Input_Analog()
 *
 * Does NOT call video_input_param() or hdmi_config() - those configure
 * BT1120 input mode which can interfere with internal pattern generation.
 */
int x5_lt8618_init_for_pattern(void)
{
	u8 ver[3];
	int ret;

	ret = x5_lt8618_probe_of();
	if (ret)
		return ret;

	/* Reference: LT8618SX_Chip_ID() */
	lt8618_wr(0x80ee, 0x01);

	ver[0] = ver[1] = ver[2] = 0;
	lt8618_rd(0x8000, &ver[0]);
	lt8618_rd(0x8001, &ver[1]);
	lt8618_rd(0x8002, &ver[2]);
	DISP_LOG_INFO("LT8618 Chip ID: %02x %02x %02x\n",
		      ver[0], ver[1], ver[2]);

	if (ver[0] != 0x17 || ver[1] != 0x02) {
		DISP_LOG_ERROR("LT8618: Invalid chip ID! I2C communication problem?\n");
		return -EIO;
	}

	/* Reference: LT8618SX_RST_PD_Init() */
	lt8618_wr(0x8011, 0x00);
	lt8618_wr(0x8013, 0xf1);
	lt8618_wr(0x8013, 0xf9);

	/* Reference: LT8618SX_TTL_Input_Analog() - exact same values */
	lt8618_wr(0x8102, 0x66);
	lt8618_wr(0x810a, 0x06);
	lt8618_wr(0x8115, 0x06);
	lt8618_wr(0x814e, 0x00);
	lt8618_wr(0x821b, 0x77);
	lt8618_wr(0x821c, 0xec);

	DISP_LOG_INFO("LT8618: Pattern init done (ChipID+RST+TTL_Analog)\n");
	return 0;
}

/*
 * Matches kernel lt8618_bridge_mode_set():
 *   input_cfg -> output_cfg(analog + pll_range + pll_cfg) -> csc -> AVI -> timing
 * AVI: LT8618SXB_AVI_setting() uses VIC_Num=0x5F, PB1=0x30, PB2=0x19 (+ VSIF when VIC==95).
 */
int x5_lt8618_apply_mode(const struct display_timing *t)
{
	int cea_vic;
	u32 htot, vtot, refresh_hz;
	unsigned long long px;

	if (!g_chip)
		return -ENODEV;

	htot = t->hactive.typ + t->hfront_porch.typ + t->hsync_len.typ +
	       t->hback_porch.typ;
	vtot = t->vactive.typ + t->vfront_porch.typ + t->vsync_len.typ +
	       t->vback_porch.typ;
	px = t->pixelclock.typ;
	refresh_hz = (htot && vtot) ?
		     (u32)(px / ((unsigned long long)htot * vtot)) : 0U;

	X5_DISP_LOG_INFO("LT8618 bridge_mode_set: START %ux%u@%u dotclock_khz=%u\n",
			 t->hactive.typ, t->vactive.typ, refresh_hz,
			 t->pixelclock.typ / 1000U);
	DISP_LOG_INFO("[LT8618] bridge_mode_set: START mode=%ux%u@%u\n",
		      t->hactive.typ, t->vactive.typ, refresh_hz);
	cea_vic = lt8618_drm_match_cea_mode_vic(t);
	DISP_LOG_INFO("[LT8618] bridge_mode_set: VIC=%d\n", cea_vic);

	DISP_LOG_INFO("[LT8618] bridge_mode_set: video_input_cfg\n");
	lt8618_video_input_cfg();

	DISP_LOG_INFO("[LT8618] bridge_mode_set: video_output_cfg\n");
	DISP_LOG_INFO("[LT8618] video_output_cfg: START\n");
	DISP_LOG_INFO("[LT8618] video_output_cfg: analog\n");
	lt8618_video_output_analog();
	DISP_LOG_INFO("[LT8618] video_output_cfg: pll_range\n");
	lt8618_video_output_pll_range(t->pixelclock.typ);
	DISP_LOG_INFO("[LT8618] video_output_cfg: pll_cfg\n");
	lt8618_video_output_pll_cfg();
	DISP_LOG_INFO("[LT8618] video_output_cfg: DONE\n");

	DISP_LOG_INFO("[LT8618] bridge_mode_set: hdmi_csc\n");
	lt8618_hdmi_csc();

	DISP_LOG_INFO("[LT8618] bridge_mode_set: AVI_setting\n");
	lt8618_avi_and_vsif(0x5f, 0x30, 0x19);

	DISP_LOG_INFO("[LT8618] bridge_mode_set: video_output_timing\n");
	lt8618_video_output_timing(t);

	DISP_LOG_INFO("[LT8618] bridge_mode_set: DONE\n");
	X5_DISP_LOG_INFO("LT8618 bridge_mode_set: DONE\n");

	return 0;
}

int x5_lt8618_enable_phy(void)
{
	u8 val;
	int i, locked = 0;

	if (!g_chip)
		return -ENODEV;

	X5_DISP_LOG_INFO("LT8618 bridge_enable: START (after upstream BT1120/DC)\n");

	/* Kernel lt8618_bridge_enable: afe_high -> phase_config -> input_timing_check */
	lt8618_afe_high();
	DISP_LOG_INFO("[LT8618] bridge_enable: phase_config (kernel algorithm)\n");
	lt8618_phase_config();

	/* Optional status read (kernel does not poll here; useful for U-Boot log) */
	for (i = 0; i < 10; i++) {
		mdelay(10);
		lt8618_rd(0x8215, &val);
		if (val & 0x80) {
			locked = 1;
			break;
		}
	}

	if (locked) {
		DISP_LOG_INFO("LT8618: PLL locked (0x8215=0x%02x)\n", val);
	} else {
		DISP_LOG_WARN("LT8618: PLL lock failed after 100ms (0x8215=0x%02x)\n", val);
	}

	lt8618_timing_hw_regs_uboot("enable_phy");

	X5_DISP_LOG_INFO("LT8618 bridge_enable: DONE\n");

	return 0;
}

void x5_lt8618_disable_tx(void)
{
	if (!g_chip)
		return;
	X5_DISP_LOG_INFO("LT8618 bridge_disable: START\n");
	lt8618_wr(0x8130, 0x00);
	X5_DISP_LOG_INFO("LT8618 bridge_disable: DONE\n");
}

/*
 * Detect input clock frequency from BT1120.
 * LT8618 has internal frequency counter at 0x821d/821e/821f.
 * Returns detected clock in Hz, or 0 on error.
 */
unsigned long x5_lt8618_clock_detect(void)
{
	u8 r821d, r821e, r821f, r821b, r821c, r814e;
	unsigned long dclk;
	int ret;

	if (!g_chip) {
		DISP_LOG_WARN("LT8618: no chip for clock detect\n");
		return 0;
	}

	/* Debug: Check frequency meter config before detection */
	lt8618_rd(0x814e, &r814e);
	lt8618_rd(0x821b, &r821b);
	lt8618_rd(0x821c, &r821c);
	DISP_LOG_DEBUG("LT8618: Freq meter cfg: 0x814e=0x%02x, 0x821b=0x%02x, 0x821c=0x%02x\n",
		       r814e, r821b, r821c);

	/* Start clock detection */
	ret = lt8618_wr(0x8217, 0x80);
	if (ret) {
		DISP_LOG_WARN("LT8618: clock detect start failed\n");
		return 0;
	}

	/* Wait for detection (500ms as per reference code) */
	mdelay(500);

	/* Read detected clock value */
	ret = lt8618_rd(0x821d, &r821d);
	if (ret) {
		DISP_LOG_WARN("LT8618: freq meter read 0x821d failed (%d)\n", ret);
		return 0;
	}
	ret = lt8618_rd(0x821e, &r821e);
	if (ret) {
		DISP_LOG_WARN("LT8618: freq meter read 0x821e failed (%d)\n", ret);
		return 0;
	}
	ret = lt8618_rd(0x821f, &r821f);
	if (ret) {
		DISP_LOG_WARN("LT8618: freq meter read 0x821f failed (%d)\n", ret);
		return 0;
	}

	/* Same as kernel lt8618_video_input_timing_check(): 24-bit count, unit kHz */
	dclk = ((unsigned long)(r821d & 0x0f) << 16) | ((unsigned long)r821e << 8) | r821f;
	dclk *= 1000UL;

	DISP_LOG_INFO("LT8618: Freq meter raw 0x821d..f = %02x %02x %02x -> ~%lu kHz -> %lu Hz\n",
		      r821d, r821e, r821f,
		      (((unsigned long)(r821d & 0x0f) << 16) | ((unsigned long)r821e << 8) | r821f),
		      dclk);

	if (dclk < 25000000UL || dclk > 340000000UL)
		DISP_LOG_WARN("LT8618: pixel clock %lu Hz outside expected 25..340 MHz (BT1120 path)\n",
			      dclk);

	return dclk;
}

void x5_lt8618_log_parallel_input_summary(const char *tag)
{
	u8 typ, lane, syncgen, embed, sample, src;

	if (!g_chip) {
		DISP_LOG_INFO("LT8618 [%s]: (no chip)\n", tag ? tag : "?");
		return;
	}

	lt8618_rd(LT8618_REG_INPUT_VIDEO_TYPE, &typ);
	lt8618_rd(LT8618_REG_INPUT_DATA_LANE_SEQ, &lane);
	lt8618_rd(LT8618_REG_INPUT_VIDEO_SYNC_GEN, &syncgen);
	lt8618_rd(LT8618_REG_EMBEDDED_SYNC_MODE_INPUT_ENABLE, &embed);
	lt8618_rd(LT8618_REG_INPUT_SIGNAL_SAMPLE_TYPE, &sample);
	lt8618_rd(LT8618_REG_INPUT_SRC_SELECT, &src);

	DISP_LOG_INFO(
		"LT8618 [%s]: par_in 0x800a=0x%02x 0x8245=0x%02x 0x8247=0x%02x 0x8248=0x%02x 0x824f=0x%02x 0x8250=0x%02x (expect type=%u RBG=%u)\n",
		tag ? tag : "?", typ, lane, syncgen, embed, sample, src,
		(unsigned int)RGB_BT1120, (unsigned int)DATA_LANE_SEQ_RBG);
}

/*
 * Dump all LT8618 registers for U-Boot vs kernel comparison.
 * Grouped by function: chip ID, input, PLL, video timing, HDMI output.
 */
void x5_lt8618_dump_regs(void)
{
	u8 val;
	int ret;
	unsigned int i;

	if (!g_chip) {
		LT8618_LOG_WARN("no chip\n");
		return;
	}

	/* Skip I2C spam when LT8618 debug logging is off (LT8618_LOG_DEBUG is no-op). */
	if (X5_LOG_LEVEL_LT8618 < X5_LVL_DEBUG)
		return;

	LT8618_LOG_DEBUG("\n==================== LT8618 Full Register Dump ====================\n");

	/* --- Page 0x80: System/Chip ID --- */
	LT8618_LOG_DEBUG("\n[Page 0x80 - System]\n");
	lt8618_rd(0x8000, &val); LT8618_LOG_DEBUG("  Chip ID0   (0x8000) = 0x%02x\n", val);
	lt8618_rd(0x8001, &val); LT8618_LOG_DEBUG("  Chip ID1   (0x8001) = 0x%02x\n", val);
	lt8618_rd(0x8002, &val); LT8618_LOG_DEBUG("  Chip ID2   (0x8002) = 0x%02x\n", val);
	lt8618_rd(0x800a, &val); LT8618_LOG_DEBUG("  Video Type (0x800a) = 0x%02x\n", val);
	lt8618_rd(0x80ee, &val); LT8618_LOG_DEBUG("  I2C Enable (0x80ee) = 0x%02x\n", val);

	/* --- Page 0x80: Reset/Power --- */
	lt8618_rd(0x8011, &val); LT8618_LOG_DEBUG("  SW Reset   (0x8011) = 0x%02x\n", val);
	lt8618_rd(0x8013, &val); LT8618_LOG_DEBUG("  Power Ctrl (0x8013) = 0x%02x\n", val);
	lt8618_rd(0x8016, &val); LT8618_LOG_DEBUG("  MPLL Ctrl  (0x8016) = 0x%02x\n", val);
	lt8618_rd(0x8018, &val); LT8618_LOG_DEBUG("  SSC Ctrl   (0x8018) = 0x%02x\n", val);

	/* --- Page 0x81: TTL Input Analog --- */
	LT8618_LOG_DEBUG("\n[Page 0x81 - TTL Input Analog]\n");
	lt8618_rd(0x8102, &val); LT8618_LOG_DEBUG("  TTL CFG    (0x8102) = 0x%02x\n", val);
	lt8618_rd(0x810a, &val); LT8618_LOG_DEBUG("  TTL CFG2   (0x810a) = 0x%02x\n", val);
	lt8618_rd(0x8115, &val); LT8618_LOG_DEBUG("  TTL CFG3   (0x8115) = 0x%02x\n", val);
	lt8618_rd(0x814e, &val); LT8618_LOG_DEBUG("  TTL CFG4   (0x814e) = 0x%02x\n", val);

	/* --- Page 0x81: PLL Configuration --- */
	LT8618_LOG_DEBUG("\n[Page 0x81 - PLL Config]\n");
	lt8618_rd(0x8123, &val); LT8618_LOG_DEBUG("  MPLL 0x8123 = 0x%02x\n", val);
	lt8618_rd(0x8124, &val); LT8618_LOG_DEBUG("  MPLL 0x8124 = 0x%02x\n", val);
	lt8618_rd(0x8125, &val); LT8618_LOG_DEBUG("  MPLL 0x8125 = 0x%02x\n", val);
	lt8618_rd(0x8126, &val); LT8618_LOG_DEBUG("  MPLL 0x8126 = 0x%02x\n", val);
	lt8618_rd(0x8127, &val); LT8618_LOG_DEBUG("  MPLL 0x8127 = 0x%02x\n", val);
	lt8618_rd(0x8128, &val); LT8618_LOG_DEBUG("  MPLL 0x8128 = 0x%02x\n", val);
	lt8618_rd(0x8129, &val); LT8618_LOG_DEBUG("  MPLL 0x8129 = 0x%02x\n", val);
	lt8618_rd(0x812a, &val); LT8618_LOG_DEBUG("  MPLL 0x812a = 0x%02x\n", val);
	lt8618_rd(0x812b, &val); LT8618_LOG_DEBUG("  MPLL 0x812b = 0x%02x\n", val);
	lt8618_rd(0x812c, &val); LT8618_LOG_DEBUG("  MPLL 0x812c = 0x%02x\n", val);
	lt8618_rd(0x812d, &val); LT8618_LOG_DEBUG("  MPLL 0x812d = 0x%02x\n", val);
	lt8618_rd(0x814d, &val); LT8618_LOG_DEBUG("  MPLL 0x814d = 0x%02x\n", val);

	/* --- Page 0x81: HDMI PHY --- */
	LT8618_LOG_DEBUG("\n[Page 0x81 - HDMI PHY]\n");
	lt8618_rd(0x8130, &val); LT8618_LOG_DEBUG("  HDMI PHY0  (0x8130) = 0x%02x\n", val);
	lt8618_rd(0x8131, &val); LT8618_LOG_DEBUG("  HDMI PHY1  (0x8131) = 0x%02x\n", val);
	lt8618_rd(0x8132, &val); LT8618_LOG_DEBUG("  HDMI PHY2  (0x8132) = 0x%02x\n", val);
	lt8618_rd(0x8133, &val); LT8618_LOG_DEBUG("  HDMI PHY3  (0x8133) = 0x%02x\n", val);
	lt8618_rd(0x8134, &val); LT8618_LOG_DEBUG("  HDMI PHY4  (0x8134) = 0x%02x\n", val);
	lt8618_rd(0x8135, &val); LT8618_LOG_DEBUG("  HDMI PHY5  (0x8135) = 0x%02x\n", val);
	lt8618_rd(0x8136, &val); LT8618_LOG_DEBUG("  HDMI PHY6  (0x8136) = 0x%02x\n", val);
	lt8618_rd(0x8137, &val); LT8618_LOG_DEBUG("  HDMI PHY7  (0x8137) = 0x%02x\n", val);
	lt8618_rd(0x813f, &val); LT8618_LOG_DEBUG("  HDMI PHYf  (0x813f) = 0x%02x\n", val);
	lt8618_rd(0x8140, &val); LT8618_LOG_DEBUG("  HDMI PHY40 (0x8140) = 0x%02x\n", val);
	lt8618_rd(0x8141, &val); LT8618_LOG_DEBUG("  HDMI PHY41 (0x8141) = 0x%02x\n", val);
	lt8618_rd(0x8142, &val); LT8618_LOG_DEBUG("  HDMI PHY42 (0x8142) = 0x%02x\n", val);
	lt8618_rd(0x8143, &val); LT8618_LOG_DEBUG("  HDMI PHY43 (0x8143) = 0x%02x\n", val);
	lt8618_rd(0x8144, &val); LT8618_LOG_DEBUG("  HDMI PHY44 (0x8144) = 0x%02x\n", val);

	/* --- Page 0x82: Video Input/PLL Status --- */
	LT8618_LOG_DEBUG("\n[Page 0x82 - Video/PLL Status]\n");
	lt8618_rd(0x8210, &val); LT8618_LOG_DEBUG("  INT CFG    (0x8210) = 0x%02x\n", val);
	lt8618_rd(0x8215, &val); LT8618_LOG_DEBUG("  PLL Lock   (0x8215) = 0x%02x [locked=%d]\n", val, !!(val & 0x80));
	lt8618_rd(0x8217, &val); LT8618_LOG_DEBUG("  DCLK Start (0x8217) = 0x%02x\n", val);
	lt8618_rd(0x821b, &val); LT8618_LOG_DEBUG("  Freq Cfg1  (0x821b) = 0x%02x\n", val);
	lt8618_rd(0x821c, &val); LT8618_LOG_DEBUG("  Freq Cfg2  (0x821c) = 0x%02x\n", val);
	lt8618_rd(0x821d, &val); LT8618_LOG_DEBUG("  DCLK_H     (0x821d) = 0x%02x\n", val);
	lt8618_rd(0x821e, &val); LT8618_LOG_DEBUG("  DCLK_M     (0x821e) = 0x%02x\n", val);
	lt8618_rd(0x821f, &val); LT8618_LOG_DEBUG("  DCLK_L     (0x821f) = 0x%02x\n", val);

	/* --- Page 0x82: Input Data Lane/Format --- */
	lt8618_rd(0x8245, &val); LT8618_LOG_DEBUG("  Data Lane  (0x8245) = 0x%02x\n", val);
	lt8618_rd(0x8247, &val); LT8618_LOG_DEBUG("  Video Sync (0x8247) = 0x%02x\n", val);
	lt8618_rd(0x8248, &val); LT8618_LOG_DEBUG("  Embed Sync (0x8248) = 0x%02x\n", val);
	lt8618_rd(0x824f, &val); LT8618_LOG_DEBUG("  Sample Type(0x824f) = 0x%02x\n", val);
	lt8618_rd(0x8250, &val); LT8618_LOG_DEBUG("  Input SRC  (0x8250) = 0x%02x\n", val);
	lt8618_rd(0x8251, &val); LT8618_LOG_DEBUG("  Video Check(0x8251) = 0x%02x\n", val);

	/* --- Page 0x82: HPD/Connection --- */
	ret = lt8618_rd(0x825e, &val);
	if (ret == 0)
		LT8618_LOG_DEBUG("  HPD Status (0x825e) = 0x%02x [connected=%d]\n",
				 val, !!(val & 0x01));

	/* --- Page 0x82: Video Output Timing (30 bytes) --- */
	LT8618_LOG_DEBUG("\n[Page 0x82 - Video Output Timing (0x8220-0x823d)]\n");
	for (i = 0; i < 30; i++) {
		lt8618_rd(0x8220 + i, &val);
		LT8618_LOG_DEBUG("  0x%04x = 0x%02x ", 0x8220 + i, val);
		if ((i + 1) % 5 == 0)
			LT8618_LOG_DEBUG("\n");
	}
	if (i % 5 != 0)
		LT8618_LOG_DEBUG("\n");

	/* --- Page 0x82: AVI InfoFrame --- */
	LT8618_LOG_DEBUG("\n[Page 0x82 - AVI InfoFrame]\n");
	lt8618_rd(0x82b9, &val); LT8618_LOG_DEBUG("  CSC CFG    (0x82b9) = 0x%02x\n", val);
	lt8618_rd(0x82d6, &val); LT8618_LOG_DEBUG("  HDMI Mode  (0x82d6) = 0x%02x\n", val);

	/* --- Page 0x82: PLL Calibration --- */
	LT8618_LOG_DEBUG("\n[Page 0x82 - PLL Calibration]\n");
	lt8618_rd(0x82de, &val); LT8618_LOG_DEBUG("  PLL Reset  (0x82de) = 0x%02x\n", val);
	lt8618_rd(0x82ea, &val); LT8618_LOG_DEBUG("  Cali Value (0x82ea) = 0x%02x\n", val);
	lt8618_rd(0x82eb, &val); LT8618_LOG_DEBUG("  Cali Done  (0x82eb) = 0x%02x [done=%d]\n", val, !!(val & 0x80));

	/* --- Page 0x84: HDMI InfoFrame/Audio --- */
	LT8618_LOG_DEBUG("\n[Page 0x84 - HDMI Config]\n");
	lt8618_rd(0x843d, &val); LT8618_LOG_DEBUG("  InfoFrame  (0x843d) = 0x%02x\n", val);
	lt8618_rd(0x8443, &val); LT8618_LOG_DEBUG("  AVI PB0    (0x8443) = 0x%02x\n", val);
	lt8618_rd(0x8444, &val); LT8618_LOG_DEBUG("  AVI PB1    (0x8444) = 0x%02x\n", val);
	lt8618_rd(0x8445, &val); LT8618_LOG_DEBUG("  AVI PB2    (0x8445) = 0x%02x\n", val);
	lt8618_rd(0x8447, &val); LT8618_LOG_DEBUG("  VIC        (0x8447) = 0x%02x\n", val);

	LT8618_LOG_DEBUG("\n=================================================================\n");
	LT8618_LOG_DEBUG("To compare in kernel, use i2cget or devmem-equivalent reads:\n");
	LT8618_LOG_DEBUG("  i2cget -y <bus> <addr> <page> ; i2cget -y <bus> <addr> <reg>\n");
	LT8618_LOG_DEBUG("=================================================================\n\n");
}

/*
 * Clock detection - exact copy of reference LT8618SX_CLK_Det().
 * Triggers freq meter, waits 500ms, reads result.
 */
// static unsigned long lt8618_clk_det_for_pattern(void)
// {
// 	u8 r_d, r_e, r_f;
// 	unsigned long dclk;

// 	lt8618_wr(0x8217, 0x80);
// 	mdelay(500);

// 	lt8618_rd(0x821d, &r_d);
// 	lt8618_rd(0x821e, &r_e);
// 	lt8618_rd(0x821f, &r_f);

// 	dclk = ((r_d & 0x0f) << 8) + r_e;
// 	dclk = (dclk << 8) + r_f;

// 	DISP_LOG_INFO("LT8618: CLK_Det dclk = %lu\n", dclk);
// 	return dclk;
// }

/*
 * Full pattern enable - matches reference test_pattern() EXACTLY.
 *
 * Must be called after x5_lt8618_init_for_pattern() which does:
 *   Chip_ID + RST_PD_Init + TTL_Input_Analog
 *
 * This function does:
 *   LT8618SX_PLL (with CLK_Det + PLL_Version_U3)
 *   + pattern regs + pixel clk + digital reset + clk select + AVI + PHY + enable
 */
int x5_lt8618_enable_pattern(const struct display_timing *t)
{
	u8 lock, cali_val, cali_done, read_val;
	u8 pattern_regs[14];
	u8 pll_812d;
	struct display_timing def;
	const struct display_timing *tm;
	int i;

	if (!g_chip)
		return -ENODEV;

	if (!t) {
		lt8618_fill_default_pattern_timing(&def);
		tm = &def;
	} else {
		tm = t;
	}

	if (!lt8618_pattern_timing_fits(tm))
		return -EINVAL;

	lt8618_pattern_regs_pack(tm, pattern_regs);
	pll_812d = lt8618_pattern_pll_812d_for_pc(tm->pixelclock.typ);

	DISP_LOG_INFO("LT8618: === Enabling internal test pattern %ux%u (PLL 0x812d=0x%02x) ===\n",
		      tm->hactive.typ, tm->vactive.typ, pll_812d);

	/*
	 * Step 1: LT8618SX_PLL() -> LT8618SX_PLL_Version_U3()
	 * Exact register sequence from reference lontium-lt8618-kernel.c
	 */

	// /* 1a: Clock detection (reference: LT8618SX_CLK_Det) */
	// dclk = lt8618_clk_det_for_pattern();

	// /* 1b: Output analog + PLL range based on dclk
	//  * Video_Input_Mode == Input_BT1120_16BIT, USE_DDRCLK == 0
	//  */
	// lt8618_wr(0x8123, 0x40);
	// lt8618_wr(0x8124, 0x62);
	// lt8618_wr(0x8125, 0x00);
	// lt8618_wr(0x812c, 0x9e);

	// if (dclk < 50000) {
	// 	lt8618_wr(0x812d, 0xaa);
	// 	DISP_LOG_INFO("LT8618: PLL LOW (dclk=%lu)\n", dclk);
	// } else if (dclk < 100000) {
	// 	lt8618_wr(0x812d, 0x99);
	// 	DISP_LOG_INFO("LT8618: PLL MID (dclk=%lu)\n", dclk);
	// } else {
	// 	lt8618_wr(0x812d, 0x88);
	// 	DISP_LOG_INFO("LT8618: PLL HIGH (dclk=%lu)\n", dclk);
	// }

	/* 1a: Pattern mode - skip BT1120 clock detection, use internal generator */
	DISP_LOG_INFO("LT8618: Pattern mode - using internal clock\n");

	/* 1b: Output analog + PLL range from pixel clock (kHz thresholds like reference CLK_Det) */
	lt8618_wr(0x8123, 0x40);
	lt8618_wr(0x8124, 0x62);
	lt8618_wr(0x8125, 0x00);
	lt8618_wr(0x812c, 0x9e);
	lt8618_wr(0x812d, pll_812d);

	lt8618_wr(0x8126, 0x55);
	lt8618_wr(0x8127, 0x66);
	lt8618_wr(0x8128, 0x88);
	lt8618_wr(0x8129, 0x04);

	/* 1c: Clear calibration enable bits */
	lt8618_rd(0x812b, &read_val);
	lt8618_wr(0x812b, read_val & 0xfd);
	lt8618_rd(0x812e, &read_val);
	lt8618_wr(0x812e, read_val & 0xfe);

	/* 1d: PLL reset sequence */
	lt8618_wr(0x82de, 0x00);
	lt8618_wr(0x82de, 0xc0);
	lt8618_wr(0x8016, 0xf1);
	lt8618_wr(0x8018, 0xdc);
	lt8618_wr(0x8018, 0xfc);
	lt8618_wr(0x8016, 0xf3);

	/* 1e: SDR_CLK path (USE_DDRCLK==0) */
	lt8618_wr(0x8127, 0x66);
	lt8618_wr(0x812a, 0x00);
	lt8618_wr(0x812a, 0x20);

	/* 1f: PLL lock loop - 5 attempts, 10ms each (matching reference) */
	for (i = 0; i < 5; i++) {
		mdelay(120);
		lt8618_wr(0x8016, 0xe3);
		lt8618_wr(0x8016, 0xf3);

		lt8618_rd(0x8215, &lock);
		lock &= 0x80;
		lt8618_rd(0x82ea, &cali_val);
		lt8618_rd(0x82eb, &cali_done);
		cali_done &= 0x80;

		DISP_LOG_INFO("LT8618: PLL[%d] lock=0x%02x cali=0x%02x done=0x%02x\n",
			      i, lock, cali_val, cali_done);

		if (lock && cali_done && cali_val != 0xff) {
			DISP_LOG_INFO("LT8618: TXPLL Locked!\n");
			lt8618_wr(0x812a, 0x00);
			lt8618_wr(0x812a, 0x20);
			break;
		}
		lt8618_wr(0x8016, 0xf1);
		lt8618_wr(0x8018, 0xdc);
		lt8618_wr(0x8018, 0xfc);
		lt8618_wr(0x8016, 0xf3);
	}
	if (i == 5)
		DISP_LOG_WARN("LT8618: TXPLL Unlock after 5 attempts\n");

	/* Step 2: Pattern timing 0x82a3..0x82b0 (14 B, vendor layout) */
	for (i = 0; i < (int)sizeof(pattern_regs); i++)
		lt8618_wr(0x82a3 + i, pattern_regs[i]);

	/* Step 3: Pattern pixel clock */
	lt8618_wr(0x832d, 0x50);
	lt8618_wr(0x8326, 0xb7);

	/* Step 4: Reset digital logic */
	lt8618_wr(0x8011, 0x5a);
	lt8618_wr(0x8011, 0xfa);

	/* Step 5: Clock source select - internal pattern clock */
	lt8618_wr(0x824f, 0x80);
	lt8618_wr(0x8250, 0x20);

	/* Step 6: AVI InfoFrame (same as reference) */
	lt8618_wr(0x8443, 0x26);
	lt8618_wr(0x8444, 0x10);
	lt8618_wr(0x8445, 0x21);
	lt8618_wr(0x8447, 0x10);

	/* Step 7: HDMI PHY - LT8618SX_HDMI_TX_Phy() */
	lt8618_wr(0x8130, 0xea);
	lt8618_wr(0x8131, 0x44);
	lt8618_wr(0x8132, 0x4a);
	lt8618_wr(0x8133, 0x0b);
	lt8618_wr(0x8134, 0x00);
	lt8618_wr(0x8135, 0x00);
	lt8618_wr(0x8136, 0x00);
	lt8618_wr(0x8137, 0x44);
	lt8618_wr(0x813f, 0x0f);
	lt8618_wr(0x8140, 0xb0);
	lt8618_wr(0x8141, 0xa0);
	lt8618_wr(0x8142, 0xa0);
	lt8618_wr(0x8143, 0xa0);
	lt8618_wr(0x8144, 0x0a);
	/* Phase Calibration - critical for HDMI signal quality */
	for (u8 phase_val = 0; phase_val < 128; phase_val += 16) {
		lt8618_wr(0x8127, phase_val);
		mdelay(66);
	}

	/* Timing verification */
	mdelay(50);

	/* Step 8: HDMI output enable - LT8618SX_HDMI_Out_Enable(1) */
	lt8618_wr(0x8130, 0xea);

	mdelay(100);

	/* Verify */
	lt8618_rd(0x8215, &lock);
	lt8618_rd(0x825e, &read_val);
	DISP_LOG_INFO("LT8618: Pattern done. PLL=0x%02x[lock=%d] HPD=0x%02x[cable=%d]\n",
		      lock, !!(lock & 0x80), read_val, !!(read_val & 0x01));

	lt8618_pattern_timing_dump_uboot("pattern_done", tm);

	return 0;
}
