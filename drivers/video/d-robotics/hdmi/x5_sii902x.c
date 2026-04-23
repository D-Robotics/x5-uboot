// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 *
 * U-Boot port of kernel/drivers/gpu/drm/bridge/sii902x.c (video path only).
 * BT1120 YUV422 embedded-sync parallel -> SII902x -> HDMI/DVI TMDS.
 */

#include <common.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <dm/ofnode.h>
#include <i2c.h>
#include <fdtdec.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <panel.h>
#include <asm/gpio.h>
#include <edid.h>

#include "x5_sii902x.h"
#include <hb_display_log.h>

/* --- Register map (match kernel sii902x.c) --- */
#define SII902X_TPI_VIDEO_DATA			0x00
#define SII902X_TPI_PIXEL_REPETITION		0x08
#define SII902X_TPI_AVI_PIXEL_REP_BUS_24BIT	BIT(5)
#define SII902X_TPI_AVI_PIXEL_REP_NONE		0
#define SII902X_TPI_CLK_RATIO_1X		(1 << 6)

#define SII902X_TPI_AVI_IN_FORMAT		0x09
#define SII902X_TPI_AVI_INPUT_RANGE_AUTO	(0 << 2)
#define SII902X_TPI_AVI_INPUT_COLORSPACE_YUV422	(2 << 0)
#define SII902X_TPI_AVI_INPUT_COLORSPACE_RGB	(0 << 0)

#define SII902X_TPI_AVI_OUT_FORMAT		0x0a
#define SII902X_TPI_AVI_OUTPUT_COLORSPACE_RGB	(0 << 0)

/*
 * TPI offset 0x0b = YC input mode; kernel sii902x.c sets BIT(0) only.
 * Do not use 0x0b as the value — that was the register address mistaken for data.
 */
#define SII902X_TPI_AVI_CHANNEL_DATA_SWAP	BIT(0)

#define SII902X_SYS_CTRL_DATA			0x1a
#define SII902X_SYS_CTRL_PWR_DWN		BIT(4)
#define SII902X_SYS_CTRL_OUTPUT_MODE		BIT(0)
#define SII902X_SYS_CTRL_OUTPUT_HDMI		1
#define SII902X_SYS_CTRL_OUTPUT_DVI		0

#define SII902X_REG_CHIPID(n)			(0x1b + (n))

#define SII902X_PWR_STATE_CTRL			0x1e
#define SII902X_AVI_POWER_STATE_MSK		0x3u
#define SII902X_AVI_POWER_STATE_D(l)		((u8)((l) & SII902X_AVI_POWER_STATE_MSK))

#define SII902X_TPI_AVI_INFOFRAME		0x0c

#define SII902X_TPI_SYNC_GENERATION_CTRL	0x60
#define SII902X_TPI_EMBEDDED_SYNC		BIT(7)

#define SII902X_TPI_EMBEDDED_SYNC_EXTRACTION	0x63
#define SII902X_TPI_EMBEDDED_SYNC_ENBALED	BIT(6)

#define SII902X_REG_TPI_RQB			0xc7

#define SII902X_INT_STATUS			0x3d
#define SII902X_PLUGGED_STATUS			BIT(2)

/* HDMI AVI pack (linux/hdmi.h + drivers/video/hdmi.c) */
#define HDMI_INFOFRAME_TYPE_AVI			0x82
#define HDMI_INFOFRAME_HEADER_SIZE		4
#define HDMI_AVI_INFOFRAME_SIZE			13

#define HDMI_COLORSPACE_RGB			0
#define HDMI_SCAN_MODE_UNDERSCAN			2
#define HDMI_COLORIMETRY_NONE			0
#define HDMI_PICTURE_ASPECT_NONE		0
#define HDMI_PICTURE_ASPECT_4_3			1
#define HDMI_PICTURE_ASPECT_16_9		2
#define HDMI_ACTIVE_ASPECT_PICTURE		8
#define HDMI_CONTENT_TYPE_GRAPHICS		0

#define SII902X_MAX_DOTCLOCK_KHZ		165000U

/* Single bridge instance; U-Boot display init is not re-entrant. */
static struct udevice *g_chip;
/* From DT reset-gpios; held from probe until exit (no remove in this port). */
static struct gpio_desc g_reset;
static bool g_have_reset;
static bool g_probed;
/* From DT sil,tmds-mode; drives HDMI vs DVI TPI setting. */
static bool g_sink_is_hdmi = true;

static int sii902x_wr8(u8 reg, u8 val)
{
	if (!g_chip)
		return -ENODEV;
	return dm_i2c_write(g_chip, reg, &val, 1);
}

static int sii902x_rd8(u8 reg, u8 *val)
{
	if (!g_chip || !val)
		return -ENODEV;
	return dm_i2c_read(g_chip, reg, val, 1);
}

static int sii902x_wr_buf(u8 reg, const u8 *buf, uint len)
{
	if (!g_chip || !buf)
		return -ENODEV;
	if (len == 0 || len > 32)
		return -EINVAL;
	return dm_i2c_write(g_chip, reg, (u8 *)buf, len);
}

static int sii902x_upd8(u8 reg, u8 mask, u8 val)
{
	u8 v;
	int ret = sii902x_rd8(reg, &v);

	if (ret)
		return ret;
	v = (u8)((v & ~mask) | (val & mask));
	return sii902x_wr8(reg, v);
}

static void sii902x_reset_hw(void)
{
	if (!g_have_reset)
		return;

	dm_gpio_set_value(&g_reset, 1);
	udelay(200);
	dm_gpio_set_value(&g_reset, 0);
}

static u8 hdmi_infoframe_checksum(const u8 *ptr, size_t size)
{
	u8 csum = 0;
	size_t i;

	for (i = 0; i < size; i++)
		csum += ptr[i];
	return (u8)(256 - csum);
}

static void hdmi_infoframe_set_checksum(void *buffer, size_t size)
{
	u8 *ptr = buffer;

	ptr[3] = hdmi_infoframe_checksum(buffer, size);
}

struct sii902x_avi_fields {
	u8 type;
	u8 version;
	u8 length;
	u8 colorspace;
	u8 scan_mode;
	u8 colorimetry;
	u8 picture_aspect;
	u8 active_aspect;
	u8 itc;
	u8 extended_colorimetry;
	u8 quantization_range;
	u8 nups;
	u8 video_code;
	u8 ycc_quantization_range;
	u8 content_type;
	u8 pixel_repeat;
};

/* Minimal hdmi_avi_infoframe_pack_only() from kernel/drivers/video/hdmi.c */
static int sii902x_avi_infoframe_pack(const struct sii902x_avi_fields *frame,
				      u8 *buffer, size_t size)
{
	u8 *ptr = buffer;
	size_t length;
	u8 *payload;

	if (!frame || !buffer)
		return -EINVAL;
	length = HDMI_INFOFRAME_HEADER_SIZE + frame->length;
	if (size < length)
		return -ENOSPC;

	memset(buffer, 0, size);
	ptr[0] = frame->type;
	ptr[1] = frame->version;
	ptr[2] = frame->length;
	ptr[3] = 0;

	payload = ptr + HDMI_INFOFRAME_HEADER_SIZE;

	payload[0] = (u8)(((frame->colorspace & 0x3) << 5) | (frame->scan_mode & 0x3));
	if (frame->active_aspect & 0xf)
		payload[0] |= (1U << 4);

	payload[1] = (u8)(((frame->colorimetry & 0x3) << 6) |
			  ((frame->picture_aspect & 0x3) << 4) |
			  (frame->active_aspect & 0xf));

	payload[2] = (u8)(((frame->extended_colorimetry & 0x7) << 4) |
			  ((frame->quantization_range & 0x3) << 2) |
			  (frame->nups & 0x3));
	if (frame->itc)
		payload[2] |= (1U << 7);

	payload[3] = (u8)(frame->video_code & 0x7f);

	payload[4] = (u8)(((frame->ycc_quantization_range & 0x3) << 6) |
			  ((frame->content_type & 0x3) << 4) |
			  (frame->pixel_repeat & 0xf));

	hdmi_infoframe_set_checksum(buffer, length);
	return (int)length;
}

/* CEA VIC for common modes; accept +/- sync like x5_output_hdmi CEA fills */
static u8 sii902x_match_cea_vic(const struct display_timing *t)
{
	u32 hact = t->hactive.typ;
	u32 hfp = t->hfront_porch.typ;
	u32 hsw = t->hsync_len.typ;
	u32 hbp = t->hback_porch.typ;
	u32 vact = t->vactive.typ;
	u32 vfp = t->vfront_porch.typ;
	u32 vsw = t->vsync_len.typ;
	u32 vbp = t->vback_porch.typ;
	u32 hss = hact + hfp;
	u32 hse = hss + hsw;
	u32 htot = hse + hbp;
	u32 vss = vact + vfp;
	u32 vse = vss + vsw;
	u32 vtot = vse + vbp;
	u32 clock_khz = t->pixelclock.typ / 1000U;

	/* 1280x720 @ 60 — VIC 4 */
	if (clock_khz == 74250U && hact == 1280U && vact == 720U &&
	    htot == 1650U && vtot == 750U &&
	    hss == 1390U && hse == 1430U && vss == 725U && vse == 730U)
		return 4;

	/* 1920x1080 @ 60 — VIC 16 (CEA table; sync may be +/+ or -/- in DRM) */
	if (clock_khz == 148500U && hact == 1920U && vact == 1080U &&
	    htot == 2200U && vtot == 1125U &&
	    hss == 2008U && hse == 2052U && vss == 1084U && vse == 1089U)
		return 16;

	return 0;
}

static void sii902x_read_tmds_mode_from_dt(ofnode node)
{
	const char *s;

	g_sink_is_hdmi = true;
	s = ofnode_read_string(node, "sil,tmds-mode");
	if (!s)
		return;
	if (!strcmp(s, "dvi"))
		g_sink_is_hdmi = false;
	else if (!strcmp(s, "hdmi"))
		g_sink_is_hdmi = true;
	else
		DISP_LOG_WARN("SII902x: unknown sil,tmds-mode \"%s\", using hdmi\n", s);
}

int x5_sii902x_chip_present(void)
{
	ofnode node, busnode;
	struct udevice *bus, *chip;
	struct gpio_desc rst;
	u32 addr;
	u8 rq = 0;
	u8 id;
	int ret, gpio_ret;

	node = ofnode_by_compatible(ofnode_null(), "sil,sii9022");
	if (!ofnode_valid(node) || !ofnode_is_enabled(node))
		return -ENODEV;

	busnode = ofnode_get_parent(node);
	if (!ofnode_valid(busnode) || !ofnode_is_enabled(busnode))
		return -ENODEV;

	ret = uclass_get_device_by_ofnode(UCLASS_I2C, busnode, &bus);
	if (ret)
		return ret;
	if (!device_active(bus)) {
		ret = device_probe(bus);
		if (ret)
			return ret;
	}

	addr = ofnode_read_u32_default(node, "reg", 0x39);
	ret = i2c_get_chip(bus, addr, 1, &chip);
	if (ret)
		return ret;

	/* Pulse reset for ID read only; long-lived GPIO is acquired in x5_sii902x_probe_of(). */
	memset(&rst, 0, sizeof(rst));
	gpio_ret = gpio_request_by_name_nodev(node, "reset-gpios", 0, &rst,
					      GPIOD_IS_OUT);
	if (!gpio_ret) {
		dm_gpio_set_value(&rst, 1);
		udelay(200);
		dm_gpio_set_value(&rst, 0);
		udelay(200);
		gpio_free_list_nodev(&rst, 1);
	}

	ret = dm_i2c_write(chip, SII902X_REG_TPI_RQB, &rq, 1);
	if (ret)
		return ret;

	ret = dm_i2c_read(chip, SII902X_REG_CHIPID(0), &id, 1);
	if (ret)
		return ret;

	if (id != 0xb0)
		return -ENODEV;

	return 0;
}

int x5_sii902x_probe_of(void)
{
	ofnode node, busnode;
	struct udevice *bus;
	u32 addr;
	int ret;

	if (g_probed)
		return g_chip ? 0 : -ENODEV;

	node = ofnode_by_compatible(ofnode_null(), "sil,sii9022");
	if (!ofnode_valid(node) || !ofnode_is_enabled(node)) {
		DISP_LOG_WARN("SII902x: no enabled sil,sii9022 node\n");
		return -ENODEV;
	}

	sii902x_read_tmds_mode_from_dt(node);

	busnode = ofnode_get_parent(node);
	if (!ofnode_valid(busnode)) {
		DISP_LOG_WARN("SII902x: no parent I2C bus\n");
		return -ENODEV;
	}
	if (!ofnode_is_enabled(busnode)) {
		DISP_LOG_WARN("SII902x: parent I2C not enabled\n");
		return -ENODEV;
	}

	ret = uclass_get_device_by_ofnode(UCLASS_I2C, busnode, &bus);
	if (ret) {
		DISP_LOG_WARN("SII902x: I2C bus probe failed (%d)\n", ret);
		return ret;
	}
	if (!device_active(bus)) {
		ret = device_probe(bus);
		if (ret) {
			DISP_LOG_WARN("SII902x: I2C bus device_probe failed (%d)\n", ret);
			return ret;
		}
	}

	addr = ofnode_read_u32_default(node, "reg", 0x39);
	ret = i2c_get_chip(bus, addr, 1, &g_chip);
	if (ret) {
		DISP_LOG_WARN("SII902x: i2c_get_chip 0x%x failed (%d)\n", addr, ret);
		g_chip = NULL;
		return ret;
	}

	ret = gpio_request_by_name_nodev(node, "reset-gpios", 0, &g_reset,
					 GPIOD_IS_OUT);
	if (ret) {
		g_have_reset = false;
		DISP_LOG_WARN("SII902x: reset-gpios optional (%d)\n", ret);
	} else {
		g_have_reset = true;
	}

	g_probed = true;
	return 0;
}

int x5_sii902x_sink_connected(void)
{
	u8 status;
	int ret;

	ret = x5_sii902x_probe_of();
	if (ret)
		return ret;

	sii902x_reset_hw();
	ret = sii902x_wr8(SII902X_REG_TPI_RQB, 0);
	if (ret)
		return ret;
	ret = sii902x_rd8(SII902X_INT_STATUS, &status);
	if (ret)
		return ret;

	return (status & SII902X_PLUGGED_STATUS) ? 1 : 0;
}

int x5_sii902x_init_chip(void)
{
	u8 chipid[4];
	int ret, i;

	ret = x5_sii902x_probe_of();
	if (ret)
		return ret;

	sii902x_reset_hw();

	ret = sii902x_wr8(SII902X_REG_TPI_RQB, 0);
	if (ret)
		return ret;

	for (i = 0; i < 4; i++) {
		ret = sii902x_rd8(SII902X_REG_CHIPID(0) + (u8)i, &chipid[i]);
		if (ret) {
			DISP_LOG_ERROR("SII902x: chip id read failed %d\n", ret);
			return ret;
		}
	}

	if (chipid[0] != 0xb0) {
		DISP_LOG_ERROR("SII902x: bad chip id 0x%02x (expect 0xb0)\n",
			       chipid[0]);
		return -EINVAL;
	}

	DISP_LOG_INFO("SII902x: chip id %02x %02x %02x %02x, tmds=%s\n",
		      chipid[0], chipid[1], chipid[2], chipid[3],
		      g_sink_is_hdmi ? "hdmi" : "dvi");

	return 0;
}

int x5_sii902x_apply_mode(const struct display_timing *t)
{
	u8 vbuf[12];
	u8 sbuf[10];
	u8 avibuf[HDMI_INFOFRAME_HEADER_SIZE + HDMI_AVI_INFOFRAME_SIZE];
	u16 pixel_clock_10khz;
	u32 clock_khz;
	u32 hdisplay, hsync_start, hsync_end, htotal;
	u32 vdisplay, vsync_start, vsync_end, vtotal;
	u32 hfp, hsync, vfp, vsync;
	u8 output_bit;
	int ret, avi_len;
	u8 vic;
	u8 pic_aspect;
	struct sii902x_avi_fields af;

	if (!g_chip || !t)
		return -ENODEV;

	clock_khz = t->pixelclock.typ / 1000U;
	if (clock_khz > SII902X_MAX_DOTCLOCK_KHZ) {
		DISP_LOG_ERROR("SII902x: dot clock %u kHz > %u (kernel limit)\n",
			       clock_khz, SII902X_MAX_DOTCLOCK_KHZ);
		return -EINVAL;
	}

	hdisplay = t->hactive.typ;
	hfp = t->hfront_porch.typ;
	hsync = t->hsync_len.typ;
	hsync_start = hdisplay + hfp;
	hsync_end = hsync_start + hsync;
	htotal = hsync_end + t->hback_porch.typ;

	vdisplay = t->vactive.typ;
	vfp = t->vfront_porch.typ;
	vsync = t->vsync_len.typ;
	vsync_start = vdisplay + vfp;
	vsync_end = vsync_start + vsync;
	vtotal = vsync_end + t->vback_porch.typ;

	pixel_clock_10khz = (u16)(clock_khz / 10U);

	output_bit = g_sink_is_hdmi ? SII902X_SYS_CTRL_OUTPUT_HDMI :
				    SII902X_SYS_CTRL_OUTPUT_DVI;

	memset(vbuf, 0, sizeof(vbuf));
	vbuf[0] = (u8)(pixel_clock_10khz & 0xff);
	vbuf[1] = (u8)(pixel_clock_10khz >> 8);
	vbuf[2] = (u8)(t->pixelclock.typ /
		       ((unsigned long long)htotal * vtotal));
	vbuf[3] = 0;
	vbuf[4] = (u8)(hdisplay & 0xff);
	vbuf[5] = (u8)(hdisplay >> 8);
	vbuf[6] = (u8)(vdisplay & 0xff);
	vbuf[7] = (u8)(vdisplay >> 8);
	vbuf[8] = (u8)(SII902X_TPI_CLK_RATIO_1X | SII902X_TPI_AVI_PIXEL_REP_NONE |
		       SII902X_TPI_AVI_PIXEL_REP_BUS_24BIT);
	/* Match kernel sii902x_bridge_mode_set: BT1120 path is YUV422 */
	vbuf[9] = (u8)(SII902X_TPI_AVI_INPUT_RANGE_AUTO |
		       SII902X_TPI_AVI_INPUT_COLORSPACE_YUV422);
	vbuf[10] = SII902X_TPI_AVI_OUTPUT_COLORSPACE_RGB;
	vbuf[11] = SII902X_TPI_AVI_CHANNEL_DATA_SWAP;

	ret = sii902x_upd8(SII902X_SYS_CTRL_DATA, SII902X_SYS_CTRL_OUTPUT_MODE,
			   output_bit);
	if (ret)
		return ret;

	ret = sii902x_wr_buf(SII902X_TPI_VIDEO_DATA, vbuf, 12);
	if (ret)
		return ret;

	memset(sbuf, 0, sizeof(sbuf));
	sbuf[0] = SII902X_TPI_EMBEDDED_SYNC;
	hfp = hsync_start - hdisplay;
	sbuf[2] = (u8)(hfp & 0xff);
	sbuf[3] = (u8)((sbuf[3] & ~0x03) | ((hfp >> 8) & 0x03));
	hsync = hsync_end - hsync_start;
	sbuf[6] = (u8)(hsync & 0xff);
	sbuf[7] = (u8)((sbuf[7] & ~0x03) | ((hsync >> 8) & 0x03));
	sbuf[8] = (u8)(vsync_start - vdisplay);
	sbuf[9] = (u8)(vsync_end - vsync_start);

	ret = sii902x_wr_buf(SII902X_TPI_SYNC_GENERATION_CTRL, sbuf, 10);
	if (ret)
		return ret;

	vic = sii902x_match_cea_vic(t);
	if (vic == 4 || vic == 16)
		pic_aspect = HDMI_PICTURE_ASPECT_16_9;
	else if (hdisplay * 9U / 16U == vdisplay || hdisplay * 3U / 4U == vdisplay)
		pic_aspect = HDMI_PICTURE_ASPECT_16_9;
	else
		pic_aspect = HDMI_PICTURE_ASPECT_4_3;

	memset(&af, 0, sizeof(af));
	af.type = HDMI_INFOFRAME_TYPE_AVI;
	af.version = 2;
	af.length = HDMI_AVI_INFOFRAME_SIZE;
	af.colorspace = HDMI_COLORSPACE_RGB;
	af.scan_mode = HDMI_SCAN_MODE_UNDERSCAN;
	af.colorimetry = HDMI_COLORIMETRY_NONE;
	af.picture_aspect = pic_aspect;
	af.active_aspect = HDMI_ACTIVE_ASPECT_PICTURE;
	af.video_code = vic;
	af.content_type = HDMI_CONTENT_TYPE_GRAPHICS;

	avi_len = sii902x_avi_infoframe_pack(&af, avibuf, sizeof(avibuf));

	if (avi_len < 0)
		return avi_len;

	/* kernel: bulk_write @ AVI from buf + HEADER_SIZE - 1, len AVI_SIZE + 1 */
	ret = sii902x_wr_buf(SII902X_TPI_AVI_INFOFRAME,
			     avibuf + HDMI_INFOFRAME_HEADER_SIZE - 1,
			     HDMI_AVI_INFOFRAME_SIZE + 1);
	if (ret)
		return ret;

	ret = sii902x_wr8(SII902X_TPI_EMBEDDED_SYNC_EXTRACTION,
			  SII902X_TPI_EMBEDDED_SYNC_ENBALED);
	if (ret)
		DISP_LOG_ERROR("SII902x: embedded sync enable failed (%d)\n", ret);
	return ret;
}

int x5_sii902x_enable_output(void)
{
	int ret;

	if (!g_chip)
		return -ENODEV;

	ret = sii902x_upd8(SII902X_PWR_STATE_CTRL, SII902X_AVI_POWER_STATE_MSK,
			   SII902X_AVI_POWER_STATE_D(0));
	if (ret)
		return ret;
	ret = sii902x_upd8(SII902X_SYS_CTRL_DATA, SII902X_SYS_CTRL_PWR_DWN, 0);
	return ret;
}

void x5_sii902x_disable_output(void)
{
	int ret;

	if (!g_chip)
		return;
	ret = sii902x_upd8(SII902X_SYS_CTRL_DATA, SII902X_SYS_CTRL_PWR_DWN,
			   SII902X_SYS_CTRL_PWR_DWN);
	if (ret)
		DISP_LOG_WARN("SII902x: power-down update failed (%d)\n", ret);
}

ssize_t x5_sii902x_edid_read(u8 *buf, size_t len)
{
	int ret;

	if (!buf || len < 128)
		return -EINVAL;

	ret = x5_sii902x_probe_of();
	if (ret)
		return ret;

	/*
	 * EDID over HDMI DDC is not a plain I2C slave at 0x50 on the same adapter as the
	 * SII9022 chip. The kernel driver exposes it via an i2c_mux child and
	 * sii902x_i2c_bypass_select(): TPI SYS_CTRL DDC_BUS_REQ / DDC_BUS_GRTD must be
	 * toggled before DDC traffic reaches the sink. Reading 0x50 on the parent bus
	 * therefore fails spuriously and is not a reliable test of monitor health.
	 * Return -EOPNOTSUPP so hdmi_try_edid_timing() uses x5_sii902x_get_default_timing().
	 */
	return -EOPNOTSUPP;
}

static void sii902x_timing_from_modetest_crtc(struct display_timing *t,
					      u32 clock_khz,
					      u32 hdisp, u32 hss, u32 hse, u32 htot,
					      u32 vdisp, u32 vss, u32 vse, u32 vtot,
					      bool phsync, bool pvsync)
{
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

void x5_sii902x_get_default_timing(struct display_timing *t)
{
	/* CEA 1920x1080@60 product default */
	sii902x_timing_from_modetest_crtc(t, 148500U, 1920U, 2008U, 2052U, 2200U,
					  1080U, 1084U, 1089U, 1125U, true, true);
}
