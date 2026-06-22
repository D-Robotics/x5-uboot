// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * HDMI output via VeriSilicon BT1120 + LT8618 or Silicon Image SII902x.
 * When both bridges are built in, only DT-enabled nodes are candidates: I2C
 * chip ID is checked on LT8618 first, then SII902x; first match wins.
 */

#include <common.h>
#include <dm/device.h>
#include <dm/ofnode.h>
#include <dm/pinctrl.h>
#include <dm/uclass.h>
#include <env.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <fdtdec.h>
#include <edid.h>

#include "x5_display_output.h"
#include "bridge/x5_syscon.h"
#include "clock/x5_crm.h"
#include "hdmi/x5_bt1120.h"
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618)
#include "hdmi/x5_lt8618.h"
#endif
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
#include "hdmi/x5_sii902x.h"
#endif
#include <hb_display.h>
#include <hb_display_log.h>

#if CONFIG_IS_ENABLED(VIDEO_X5_HDMI) && \
	!CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618) && \
	!CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
#error "VIDEO_X5_HDMI requires X5_HDMI_BRIDGE_LT8618 and/or X5_HDMI_BRIDGE_SII9022"
#endif

/*
 * Reasonable pixel-clock envelope for env override (Hz); delays before downstream bridges (ms).
 */
#define X5_HDMI_PCLK_ENV_MIN_HZ		50000000UL
#define X5_HDMI_PCLK_ENV_MAX_HZ		340000000UL
#define X5_HDMI_DELAY_SII902_MS			50
#define X5_HDMI_DELAY_LT8618_SETTLE_MS	300

static ofnode hdmi_bt1120_node(void);

enum x5_hdmi_bridge_type {
	X5_HDMI_BRIDGE_LT8618,
	X5_HDMI_BRIDGE_SII9022,
	X5_HDMI_BRIDGE_NONE,
};

#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618) && CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
/* Cached after first LT8618 vs SII902x selection; single-thread display bring-up only. */
static bool g_hdmi_bridge_kind_valid;
static enum x5_hdmi_bridge_type g_hdmi_bridge_kind_val;
#endif

static enum x5_hdmi_bridge_type hdmi_bridge_kind(void)
{
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618) && CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
	enum x5_hdmi_bridge_type k;
	const char *src;
	ofnode lt = ofnode_by_compatible(ofnode_null(), "lontium,lt8618");
	ofnode si = ofnode_by_compatible(ofnode_null(), "sil,sii9022");
	bool lt_dt = ofnode_valid(lt) && ofnode_is_enabled(lt);
	bool si_dt = ofnode_valid(si) && ofnode_is_enabled(si);

	if (g_hdmi_bridge_kind_valid)
		return g_hdmi_bridge_kind_val;

	if (lt_dt && !x5_lt8618_chip_present()) {
		k = X5_HDMI_BRIDGE_LT8618;
		src = "I2C LT8618 (DT node enabled)";
	} else if (si_dt && !x5_sii902x_chip_present()) {
		k = X5_HDMI_BRIDGE_SII9022;
		src = "I2C SII902x (DT node enabled)";
	} else {
		DISP_LOG_WARN("HDMI bridge: no chip ID on enabled DT nodes — HDMI path disabled (try DSI)\n");
		k = X5_HDMI_BRIDGE_NONE;
		src = "none";
	}
	g_hdmi_bridge_kind_val = k;
	g_hdmi_bridge_kind_valid = true;
	if (k != X5_HDMI_BRIDGE_NONE)
		DISP_LOG_INFO("HDMI bridge: active transmitter %s (%s)\n",
			      k == X5_HDMI_BRIDGE_LT8618 ? "LT8618" : "SII902x", src);
	return k;
#elif CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618)
	return X5_HDMI_BRIDGE_LT8618;
#else
	return X5_HDMI_BRIDGE_SII9022;
#endif
}

static int hdmi_bridge_probe(void)
{
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618) && CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
	if (hdmi_bridge_kind() == X5_HDMI_BRIDGE_NONE)
		return -ENODEV;
#endif
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
	if (hdmi_bridge_kind() == X5_HDMI_BRIDGE_SII9022)
		return x5_sii902x_probe_of();
#endif
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618)
	return x5_lt8618_probe_of();
#else
	return -ENODEV;
#endif
}

/*
 * drm/modes.h sync polarity low nibble (U-Boot has no DRM includes).
 * PHSYNC/NHSYNC/PVSYNC/NVSYNC match kernel drm_display_mode.flags bits 0..3.
 */
#define DRM_SYNC_PHSYNC	BIT(0)
#define DRM_SYNC_NHSYNC	BIT(1)
#define DRM_SYNC_PVSYNC	BIT(2)
#define DRM_SYNC_NVSYNC	BIT(3)

/*
 * Map struct display_timing.flags sync bits (include/video/display_timing.h / fdtdec
 * enum display_flags) onto the same layout as kernel drm_display_mode.flags bits 0..3.
 *
 * Kernel reference (vs_bt1120 / BT1120 bridge):
 *   - bt1120_set_online_configs() uses mode->flags with DRM_MODE_FLAG_PHSYNC/PVSYNC tests
 *     (see vs_bt1120.c).
 *   - bt1120_bridge_mode_set() passes crtc->state->mode into that path, not adjusted_mode
 *     (bt1120_bridge.c).
 *   - drm_atomic_helper_commit_modeset_disables() calls drm_bridge_chain_mode_set(bridge,
 *     &crtc_state->mode, &crtc_state->adjusted_mode) before enables.
 *   - bt1120_bridge_atomic_pre_enable() only patches adjusted_mode->flags (positive sync
 *     comment there); it does not call bt1120_set_online_configs again.
 *
 * DC8000 in U-Boot still uses struct display_timing with DISPLAY_FLAGS_*; only this
 * HDMI/BT1120 boundary must pass drm-style low nibble into x5_bt1120_set_online_mode().
 *
 * CEA HDMI modes in Linux drm use NHSYNC|NVSYNC (drm low nibble 0xa); bt1120_bridge_mode_set
 * passes that into bt1120_set_online_configs — see modetest dmesg flags=0xa, CTL=0x09.
 * Do not force PHSYNC|PVSYNC here; that mismatched kernel and set CTL=0x69.
 */
static u32 display_timing_sync_to_drm_low_nibble(u32 timing_flags)
{
	u32 drm = 0;

	if (timing_flags & DISPLAY_FLAGS_HSYNC_HIGH)
		drm |= DRM_SYNC_PHSYNC;
	else if (timing_flags & DISPLAY_FLAGS_HSYNC_LOW)
		drm |= DRM_SYNC_NHSYNC;
	if (timing_flags & DISPLAY_FLAGS_VSYNC_HIGH)
		drm |= DRM_SYNC_PVSYNC;
	else if (timing_flags & DISPLAY_FLAGS_VSYNC_LOW)
		drm |= DRM_SYNC_NVSYNC;
	return drm;
}

static void __iomem *hdmi_bt1120_base; /* BT1120 MMIO base from hdmi_bt1120_init */
static struct display_timing hdmi_timing; /* Active mode from hdmi_get_timing */

static ofnode hdmi_bt1120_node(void)
{
	return ofnode_by_compatible(ofnode_null(), "verisilicon,bt1120");
}

/* DRM/modetest -c column layout: hdisp hss hse htot / vdisp vss vse vtot, clock kHz, refresh Hz */
static void hdmi_format_modetest_sync_flags(char *buf, size_t len,
					    const struct display_timing *t)
{
	const char *hs, *vs;
	const enum display_flags f = t->flags;

	if (f & DISPLAY_FLAGS_HSYNC_HIGH)
		hs = "phsync";
	else if (f & DISPLAY_FLAGS_HSYNC_LOW)
		hs = "nhsync";
	else
		hs = "hsync-unspec";

	if (f & DISPLAY_FLAGS_VSYNC_HIGH)
		vs = "pvsync";
	else if (f & DISPLAY_FLAGS_VSYNC_LOW)
		vs = "nvsync";
	else
		vs = "vsync-unspec";

	if (f & DISPLAY_FLAGS_INTERLACED)
		snprintf(buf, len, "interlace, %s, %s", hs, vs);
	else
		snprintf(buf, len, "%s, %s", hs, vs);
}

/*
 * One DRM-style mode line from EDID-derived display_timing, for comparing with e.g.
 * modetest -M vs-drm -a -c (same column order: hdisp hss hse htot / vdisp vss vse vtot, kHz, flags).
 */
static void hdmi_log_timing_for_compare(const struct display_timing *t)
{
	unsigned int ha = t->hactive.typ;
	unsigned int hf = t->hfront_porch.typ;
	unsigned int hs = t->hsync_len.typ;
	unsigned int hb = t->hback_porch.typ;
	unsigned int va = t->vactive.typ;
	unsigned int vf = t->vfront_porch.typ;
	unsigned int vs = t->vsync_len.typ;
	unsigned int vb = t->vback_porch.typ;
	unsigned int hss = ha + hf;
	unsigned int hse = hss + hs;
	unsigned int htot = hse + hb;
	unsigned int vss = va + vf;
	unsigned int vse = vss + vs;
	unsigned int vtot = vse + vb;
	unsigned int pclk = t->pixelclock.typ;
	unsigned int pclk_khz = pclk / 1000U;
	unsigned int refresh_centi;
	u64 denom;
	char flagbuf[80];

	hdmi_format_modetest_sync_flags(flagbuf, sizeof(flagbuf), t);

	/*
	 * Vertical refresh is computed as pclk / (htot * vtot). Any of htot, vtot, or pclk
	 * being zero makes that division undefined in C and the printed line meaningless.
	 * EDID detailed timings should not produce this; the check is a guard if the struct
	 * is ever partial or corrupted.
	 */
	if (!htot || !vtot || !pclk) {
		DISP_LOG_INFO("HDMI: EDID timing incomplete %ux%u pclk=%u Hz (skip DRM mode line)\n",
			      ha, va, pclk);
		return;
	}

	denom = (u64)htot * (u64)vtot;
	refresh_centi = (unsigned int)(((u64)pclk * 100ULL + denom / 2) / denom);

	DISP_LOG_INFO("HDMI: EDID timing (compare: modetest -M vs-drm -a -c)\n");
	DISP_LOG_INFO("       index name  refresh (Hz) hdisp hss hse htot vdisp vss vse vtot\n");
	DISP_LOG_INFO("  #0 %ux%u %u.%02u %u %u %u %u %u %u %u %u %u flags: %s; type: EDID, U-Boot\n",
		      ha, va, refresh_centi / 100U, refresh_centi % 100U,
		      ha, hss, hse, htot, va, vss, vse, vtot, pclk_khz,
		      flagbuf);
}

/* CEA presets: EDID decode may match these; keep BT1120 polarity aligned with table path. */
static void hdmi_timing_bt1120_polarity_fixup(struct display_timing *t)
{
	if (t->hactive.typ == 1920 && t->vactive.typ == 1080 &&
	    t->pixelclock.typ == 148500000U)
		t->flags = DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_VSYNC_LOW;
	else if (t->hactive.typ == 3840 && t->vactive.typ == 2160 &&
		 t->pixelclock.typ == 297000000U)
		t->flags = DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_VSYNC_LOW;
}

/*
 * EDID parse uses common/edid.c — VIDEO_X5_HDMI selects I2C_EDID so this always links.
 */
static int hdmi_try_edid_timing(struct display_timing *t)
{
	u8 raw[256];
	ssize_t n;
	int bits;

	if (!x5_hdmi_edid_enabled())
		return -ENOENT;

	switch (hdmi_bridge_kind()) {
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618)
	case X5_HDMI_BRIDGE_LT8618:
		n = x5_lt8618_edid_read(raw, sizeof(raw));
		break;
#endif
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
	case X5_HDMI_BRIDGE_SII9022:
		n = x5_sii902x_edid_read(raw, sizeof(raw));
		break;
#endif
	case X5_HDMI_BRIDGE_NONE:
		return -ENODEV;
	default:
		return -ENODEV;
	}

	if (n < 0)
		return (int)n;
	if (n < 128)
		return -EIO;

	if (edid_get_first_valid_timing(raw, (int)n, t, &bits)) {
		DISP_LOG_WARN("HDMI: EDID read OK but no usable detailed timing\n");
		return -EINVAL;
	}

	hdmi_timing_bt1120_polarity_fixup(t);
	hdmi_log_timing_for_compare(t);
	return 0;
}

static int hdmi_bridge_default_timing(struct display_timing *t)
{
	enum x5_hdmi_bridge_type kind = hdmi_bridge_kind();

	switch (kind) {
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618)
	case X5_HDMI_BRIDGE_LT8618:
		x5_lt8618_get_default_timing(t);
		break;
#endif
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
	case X5_HDMI_BRIDGE_SII9022:
		x5_sii902x_get_default_timing(t);
		break;
#endif
	default:
		return -ENODEV;
	}

	t->hdmi_monitor = true;
	hdmi_timing_bt1120_polarity_fixup(t);
	DISP_LOG_INFO("HDMI: using bridge default timing %ux%u\n",
		      t->hactive.typ, t->vactive.typ);
	return 0;
}

int x5_hdmi_pattern_boot_requested(void)
{
	const char *e = env_get("x5_hdmi_pattern");

#ifdef CONFIG_VIDEO_X5_HDMI_PATTERN_BOOT
	if (e && (!strcmp(e, "0") || !strcmp(e, "n") || !strcmp(e, "N") ||
		  !strcmp(e, "no") || !strcmp(e, "NO") || !strcmp(e, "off") ||
		  !strcmp(e, "OFF")))
		return 0;
	return 1;
#else
	if (!e)
		return 0;
	return !strcmp(e, "1") || !strcmp(e, "y") || !strcmp(e, "Y") ||
	       !strcmp(e, "yes") || !strcmp(e, "YES");
#endif
}

int x5_hdmi_enable_pattern_boot(const struct display_timing *t)
{
	int ret;

	if (!t) {
		DISP_LOG_ERROR("HDMI: pattern-boot needs display_timing\n");
		return -EINVAL;
	}

#if !CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618)
	DISP_LOG_ERROR("HDMI: pattern-boot requires LT8618 (not compiled)\n");
	return -ENODEV;
#else
	if (hdmi_bridge_kind() != X5_HDMI_BRIDGE_LT8618) {
		DISP_LOG_ERROR("HDMI: pattern-boot requires LT8618 (SII902x not supported)\n");
		return -ENODEV;
	}

	DISP_LOG_INFO("HDMI: pattern-boot — LT8618 color bars %ux%u @%u Hz (DC8000/BT1120 skipped)\n",
		      t->hactive.typ, t->vactive.typ, t->pixelclock.typ);
	ret = x5_lt8618_init_for_pattern();
	if (ret) {
		DISP_LOG_ERROR("HDMI: pattern init failed (%d)\n", ret);
		return ret;
	}
	ret = x5_lt8618_enable_pattern(t);
	if (ret) {
		DISP_LOG_ERROR("HDMI: pattern enable failed (%d)\n", ret);
		return ret;
	}
	DISP_LOG_INFO("HDMI: If you see color bars, LT8618 TMDS/timing is likely OK.\n");
	return 0;
#endif
}

static int hdmi_detect(void)
{
	ofnode bt = hdmi_bt1120_node();

	if (!ofnode_valid(bt) || !ofnode_is_enabled(bt))
		return -ENODEV;

	/*
	 * hdmi_bridge_probe() -> hdmi_bridge_kind() runs I2C chip_present on LS I2C4
	 * (LT8618) and/or DSP I2C7 (SII902x). DSP I2C PCLK is gated by default; LSIO
	 * I2C4 is often already usable. Enable both before detection (idempotent).
	 */
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022) && CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618)
	x5_crm_i2c4_pclk_enable();
	x5_crm_dsp_i2c_pclk_enable();
#elif CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
	x5_crm_dsp_i2c_pclk_enable();
#elif CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618)
	x5_crm_i2c4_pclk_enable();
#endif

	if (hdmi_bridge_probe())
		return -ENODEV;

	switch (hdmi_bridge_kind()) {
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618)
	case X5_HDMI_BRIDGE_LT8618: {
		int c = x5_lt8618_sink_connected();

		if (c <= 0) {
			if (c < 0)
				DISP_LOG_WARN("HDMI: [LT8618] sink detect failed (%d) — DSI fallback\n",
					      c);
			else
				DISP_LOG_WARN("HDMI: [LT8618] no sink (HPD low) — DSI fallback\n");
			return -ENODEV;
		}
		break;
	}
#endif
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
	case X5_HDMI_BRIDGE_SII9022: {
		int c = x5_sii902x_sink_connected();

		if (c <= 0) {
			if (c < 0)
				DISP_LOG_WARN("HDMI: [SII902x] sink detect failed (%d) — DSI fallback\n",
					      c);
			else
				DISP_LOG_WARN("HDMI: [SII902x] not plugged — DSI fallback\n");
			return -ENODEV;
		}
		break;
	}
#endif
	default:
		break;
	}

	return 0;
}

static int hdmi_get_timing(struct display_timing *t)
{
	int err;

	if (!ofnode_valid(hdmi_bt1120_node()))
		return -ENODEV;

	err = hdmi_try_edid_timing(t);
	if (err) {
		err = hdmi_bridge_default_timing(t);
		if (err)
			return err;
	}

	/*
	 * Optional pixel-clock override for bring-up (Hz). CEA 1080p60 is 148500000.
	 * Same h/v totals => line rate and refresh scale with PCLK; TMDS bitrate changes.
	 * May snap to same actual rate if CRM divider grid cannot hit the request.
	 * Example: setenv x5_hdmi_pixelclock_hz 151000000; saveenv; reset
	 * Clear with: setenv x5_hdmi_pixelclock_hz; saveenv
	 */
	{
		ulong ov = env_get_ulong("x5_hdmi_pixelclock_hz", 10, 0);
		u32 orig = t->pixelclock.typ;

		if (ov) {
			if (ov >= X5_HDMI_PCLK_ENV_MIN_HZ && ov <= X5_HDMI_PCLK_ENV_MAX_HZ) {
				if ((u32)ov != orig) {
					X5_DISP_LOG_INFO("x5_hdmi_pixelclock_hz=%lu (default was %u Hz)\n",
							 ov, orig);
					t->pixelclock.typ = (u32)ov;
				}
			} else {
				DISP_LOG_WARN("x5_hdmi_pixelclock_hz=%lu out of 50..340 MHz, ignored\n",
					      ov);
			}
		}
	}

	hdmi_timing = *t;
	return 0;
}

static int hdmi_clk_init(unsigned long pixel_clock)
{
	(void)pixel_clock;
	return x5_crm_bt1120_clk_init(pixel_clock);
}

static int hdmi_reset(void)
{
	int ret;

	ret = x5_crm_common_reset();
	if (ret)
		return ret;
	return x5_crm_bt1120_reset();
}

static int hdmi_bridge_init(struct display_timing *timing)
{
#if CONFIG_IS_ENABLED(PINCTRL)
	struct udevice *bdev;
	int err;

	err = uclass_get_device_by_ofnode(UCLASS_NOP, hdmi_bt1120_node(), &bdev);
	if (!err) {
		err = pinctrl_select_state(bdev, "default");
		if (err)
			X5_DISP_LOG_INFO("BT1120 pinctrl default: %d (check disp_iomuxc)\n",
					 err);
		else
			X5_DISP_LOG_INFO("BT1120: pinctrl default applied (17 pins, bt1120datagrp)\n");
	} else {
		X5_DISP_LOG_INFO("BT1120 DM device missing (%d), skipping pinctrl\n", err);
	}
#endif
	return x5_syscon_bt1120_init(timing);
}

/*
 * Step 6: BT1120 initialization
 * Only get register base and disable BT1120 (prepare stage)
 */
static int hdmi_bt1120_init(struct display_timing *timing)
{
	phys_addr_t reg_base;
	ofnode bt = hdmi_bt1120_node();

	(void)timing;

	if (!ofnode_valid(bt))
		return -ENODEV;

	reg_base = ofnode_get_addr_size_index(bt, 0, NULL);
	if (reg_base == FDT_ADDR_T_NONE)
		reg_base = X5_BT1120_REG_BASE_DEFAULT;

	hdmi_bt1120_base = (void __iomem *)(uintptr_t)reg_base;
	BT1120_LOG_INFO("prep mmio=0x%llx, output held off\n",
			(unsigned long long)reg_base);
	x5_bt1120_disable(hdmi_bt1120_base);

	return 0;
}

/*
 * tx_init — called BEFORE DC8000 enable (kernel bridge_mode_set equivalent).
 *
 * Mirrors the kernel lt8618_probe() + lt8618_bridge_mode_set() sequence:
 *   1. LT8618 chip init (reset, IIC, input cfg, HDMI cfg)
 *   2. Video output config (PLL range, analog, CSC, AVI, timing)
 *
 * After this, LT8618 is fully configured and waiting for a valid pixel
 * clock from BT1120 to lock its PLL.
 */
static int hdmi_tx_init(struct display_timing *timing)
{
	int ret;

#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618) && CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
	if (hdmi_bridge_kind() == X5_HDMI_BRIDGE_NONE)
		return -ENODEV;
#endif
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
	if (hdmi_bridge_kind() == X5_HDMI_BRIDGE_SII9022) {
		DISP_LOG_INFO("HDMI: [tx_init 1/2] SII902x chip init...\n");
		ret = x5_sii902x_init_chip();
		if (ret) {
			DISP_LOG_ERROR("HDMI: SII902x init_chip failed (%d)\n", ret);
			return ret;
		}
		DISP_LOG_INFO("HDMI: [tx_init 2/2] SII902x mode_set...\n");
		ret = x5_sii902x_apply_mode(timing);
		if (ret) {
			DISP_LOG_ERROR("HDMI: SII902x apply_mode failed (%d)\n", ret);
			return ret;
		}
		DISP_LOG_INFO("HDMI: TX init done (SII902x), waiting for DC8000 pixels\n");
		return 0;
	}
#endif
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618)
	/* === Kernel probe equivalent === */
	DISP_LOG_INFO("HDMI: [tx_init 1/2] LT8618 chip init (probe)...\n");
	ret = x5_lt8618_init_chip();
	if (ret) {
		DISP_LOG_ERROR("HDMI: LT8618 init_chip failed (%d)\n", ret);
		return ret;
	}

	/* === Kernel bridge_mode_set equivalent === */
	DISP_LOG_INFO("HDMI: [tx_init 2/2] LT8618 output config (mode_set)...\n");
	ret = x5_lt8618_apply_mode(timing);
	if (ret) {
		DISP_LOG_ERROR("HDMI: LT8618 apply_mode failed (%d)\n", ret);
		return ret;
	}

	DISP_LOG_INFO("HDMI: TX init done, waiting for DC8000 pixels\n");
	x5_lt8618_log_parallel_input_summary("post_mode_set");
	return 0;
#else
	return -ENODEV;
#endif
}

/*
 * enable — called AFTER DC8000 enable (kernel bridge_enable equivalent).
 *
 * DC8000 is already producing pixels. Sequence:
 *   1. BT1120 online mode (CSC RGB→YUV422 + data output)
 *   2. Wait for BT1120 clock to stabilise
 *   3. LT8618 PLL lock + AFE high + phase calibration
 *
 * This matches the kernel flow where bridge_enable() runs after CRTC enable
 * and performs the final PLL lock + PHY bring-up.
 */
static int hdmi_enable(void)
{
	int ret;
	unsigned long dclk;
	u32 htot, vtot, refresh_hz;
	unsigned long long px;

#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618) && CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
	if (hdmi_bridge_kind() == X5_HDMI_BRIDGE_NONE)
		return -ENODEV;
#endif
#if CONFIG_IS_ENABLED(PINCTRL)
	/*
	 * Re-apply BT1120 pad/mux state after display reset/clocks (belt-and-suspenders
	 * vs kernel, where pinctrl stays from probe; compare dump with Linux).
	 */
	{
		struct udevice *bdev;
		int perr;

		perr = uclass_get_device_by_ofnode(UCLASS_NOP, hdmi_bt1120_node(), &bdev);
		if (!perr) {
			perr = pinctrl_select_state(bdev, "default");
			if (perr)
				X5_DISP_LOG_INFO("BT1120 pinctrl re-apply (hdmi_enable): %d\n",
						 perr);
			else
				X5_DISP_LOG_INFO("BT1120: pinctrl default re-applied (hdmi_enable)\n");
		}
	}
#endif
	x5_disp_iomux_dump_one_line("uboot-hdmi-enable-pre-bt1120");

	htot = hdmi_timing.hactive.typ + hdmi_timing.hfront_porch.typ +
	       hdmi_timing.hsync_len.typ + hdmi_timing.hback_porch.typ;
	vtot = hdmi_timing.vactive.typ + hdmi_timing.vfront_porch.typ +
	       hdmi_timing.vsync_len.typ + hdmi_timing.vback_porch.typ;
	px = hdmi_timing.pixelclock.typ;
	refresh_hz = htot && vtot ? (u32)(px / ((unsigned long long)htot * vtot)) : 0U;

	{
		u32 drm_nib = display_timing_sync_to_drm_low_nibble(hdmi_timing.flags);

		X5_DISP_LOG_INFO("bt1120_bridge atomic_pre_enable: crtc=crtc-dc8000 %ux%u@%u dotclock_khz=%u htot=%u vtot=%u display_timing.flags=0x%x drm_mode_low_nibble=0x%x (BT1120 uses this like kernel mode_set)\n",
				 hdmi_timing.hactive.typ, hdmi_timing.vactive.typ, refresh_hz,
				 hdmi_timing.pixelclock.typ / 1000U, htot, vtot,
				 (unsigned int)hdmi_timing.flags, (unsigned int)drm_nib);
	}

	/* Step 1: BT1120 starts converting DC8000 RGB to BT1120 data */
	DISP_LOG_INFO("HDMI: [enable 1/3] BT1120 online mode...\n");
	{
		struct display_timing bt1120_mode = hdmi_timing;
		u32 drm_sync = display_timing_sync_to_drm_low_nibble(hdmi_timing.flags);

		bt1120_mode.flags = (hdmi_timing.flags & 0xFFFFFFF0U) | drm_sync;
		ret = x5_bt1120_set_online_mode(hdmi_bt1120_base, &bt1120_mode);
		if (ret) {
			DISP_LOG_ERROR("HDMI: BT1120 enable failed\n");
			return ret;
		}
		x5_bt1120_log_timing_regs(hdmi_bt1120_base, "after_set_online", &bt1120_mode);
	}
	x5_disp_iomux_dump_one_line("uboot-hdmi-post-bt1120");

#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
	if (hdmi_bridge_kind() == X5_HDMI_BRIDGE_SII9022) {
		DISP_LOG_INFO("HDMI: [enable 2/2] SII902x output enable...\n");
		mdelay(X5_HDMI_DELAY_SII902_MS);
		if (hdmi_bt1120_base)
			x5_bt1120_log_irq_status(hdmi_bt1120_base,
						 "hdmi_post_bt1120_delay");
		ret = x5_sii902x_enable_output();
		if (ret) {
			DISP_LOG_ERROR("HDMI: SII902x enable failed (%d)\n", ret);
			return ret;
		}
		DISP_LOG_INFO("HDMI: Enable complete (SII902x)\n");
		return 0;
	}
#endif
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618)
	x5_lt8618_log_parallel_input_summary("post_bt1120");

	/* Step 2: Verify LT8618 sees the pixel clock from BT1120 */
	DISP_LOG_INFO("HDMI: [enable 2/3] Clock detection...\n");
	mdelay(X5_HDMI_DELAY_LT8618_SETTLE_MS); /* BT1120 settle before LT8618 detect */
	if (hdmi_bt1120_base)
		x5_bt1120_log_irq_status(hdmi_bt1120_base, "hdmi_post_bt1120_delay");
	dclk = x5_lt8618_clock_detect();
	if (dclk == 0) {
		DISP_LOG_WARN("HDMI: Clock detection failed, continuing\n");
	} else if (dclk >= 120000000UL && dclk <= 170000000UL) {
		DISP_LOG_INFO("HDMI: Input clock OK (%lu Hz, typical 1080p)\n", dclk);
	} else if (dclk < 20000000UL || dclk > 300000000UL) {
		DISP_LOG_WARN("HDMI: Clock %lu Hz looks invalid (expect rough 25..300 MHz for BT1120)\n",
			      dclk);
	} else {
		/* e.g. ~46 MHz for 720p-class timing — not 1080p, not an error */
		DISP_LOG_DEBUG("HDMI: Clock %lu Hz (non-1080p timing, OK)\n", dclk);
	}

	/* Step 3: PLL lock + HDMI PHY enable + phase calibration */
	DISP_LOG_INFO("HDMI: [enable 3/3] PHY enable (PLL + AFE + phase)...\n");
	ret = x5_lt8618_enable_phy();
	if (ret) {
		DISP_LOG_ERROR("HDMI: PHY enable failed (%d)\n", ret);
		return ret;
	}

	/* LT8618 full reg dump: no-op unless X5_LOG_LEVEL_LT8618 >= X5_LVL_DEBUG */
	x5_lt8618_dump_regs();
	DISP_LOG_INFO("HDMI: Enable complete\n");
	return 0;
#else
	return -ENODEV;
#endif
}

static int hdmi_disable(void)
{
	X5_DISP_LOG_INFO("bt1120_bridge atomic_disable: is_online=1 parent=hdmi-tx\n");

#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022) && CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618)
	if (hdmi_bridge_kind() == X5_HDMI_BRIDGE_SII9022)
		x5_sii902x_disable_output();
	else if (hdmi_bridge_kind() == X5_HDMI_BRIDGE_LT8618)
		x5_lt8618_disable_tx();
#elif CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
	x5_sii902x_disable_output();
#elif CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618)
	x5_lt8618_disable_tx();
#endif
	if (hdmi_bt1120_base)
		x5_bt1120_disable(hdmi_bt1120_base);
	x5_syscon_bt1120_disable();
	return 0;
}

static void hdmi_clk_disable(void)
{
	x5_crm_bt1120_clk_disable();
}

static int hdmi_get_backlight_config(struct x5_backlight_config *config)
{
	(void)config;
	return -ENODEV;
}

static char hdmi_info_buf[48]; /* get_info() string (single consumer, init-time) */

static const char *hdmi_get_info(void)
{
#if CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022) && CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_LT8618)
	if (hdmi_bridge_kind() == X5_HDMI_BRIDGE_SII9022) {
		snprintf(hdmi_info_buf, sizeof(hdmi_info_buf),
			 "BT1120+SII902x %ux%u",
			 hdmi_timing.hactive.typ, hdmi_timing.vactive.typ);
	} else if (hdmi_bridge_kind() == X5_HDMI_BRIDGE_LT8618) {
		snprintf(hdmi_info_buf, sizeof(hdmi_info_buf),
			 "BT1120+LT8618 HDMI %ux%u",
			 hdmi_timing.hactive.typ, hdmi_timing.vactive.typ);
	} else {
		snprintf(hdmi_info_buf, sizeof(hdmi_info_buf),
			 "BT1120+HDMI (no bridge) %ux%u",
			 hdmi_timing.hactive.typ, hdmi_timing.vactive.typ);
	}
#elif CONFIG_IS_ENABLED(X5_HDMI_BRIDGE_SII9022)
	snprintf(hdmi_info_buf, sizeof(hdmi_info_buf),
		 "BT1120+SII902x %ux%u",
		 hdmi_timing.hactive.typ, hdmi_timing.vactive.typ);
#else
	snprintf(hdmi_info_buf, sizeof(hdmi_info_buf),
		 "BT1120+LT8618 HDMI %ux%u",
		 hdmi_timing.hactive.typ, hdmi_timing.vactive.typ);
#endif
	return hdmi_info_buf;
}

const struct x5_display_output_ops x5_hdmi_output_ops = {
	.name			= "HDMI(BT1120)",
	.type			= X5_OUTPUT_BT1120_HDMI,
	.detect			= hdmi_detect,
	.get_timing		= hdmi_get_timing,
	.clk_init		= hdmi_clk_init,
	.reset			= hdmi_reset,
	.bridge_init	= hdmi_bridge_init,
	.hw_init		= hdmi_bt1120_init,
	.tx_init		= hdmi_tx_init,
	.enable			= hdmi_enable,
	.disable		= hdmi_disable,
	.clk_disable	= hdmi_clk_disable,
	.get_backlight_config	= hdmi_get_backlight_config,
	.get_info		= hdmi_get_info,
};
