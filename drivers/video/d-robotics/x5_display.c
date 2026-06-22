// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 * Modified by: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * X5 Display Platform Driver
 *
 * This is the top-level orchestrator for the X5 display subsystem.
 * It manages shared resources (DC8000, CRM clocks, framebuffer) and
 * delegates output-specific work (DSI / BT1120-HDMI) through the
 * x5_display_output_ops abstraction defined in x5_display_output.h.
 *
 * Resources allocated in probe(), hardware init via x5_display_hw_init() command.
 */

#include <common.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <dm/ofnode.h>
#include <video.h>
#include <panel.h>
#include <mipi_dsi.h>
#include <asm/io.h>
#include <fdtdec.h>
#include <linux/delay.h>
#include <linux/sizes.h>
#include <bmp_layout.h>
#include <splash.h>
#include <malloc.h>
#include <asm/global_data.h>
#include <mapmem.h>
#include <blk.h>
#include <part.h>
#include <env.h>
#include <linux/string.h>
#if CONFIG_IS_ENABLED(VIDEO_X5_BOOT_UI) && CONFIG_IS_ENABLED(CONSOLE_MUX) && \
	CONFIG_IS_ENABLED(CONSOLE_RECORD)
#include <stdio_dev.h>
#include <membuff.h>
#endif
#include "x5_display_output.h"
#include "dc8000/x5_dc8000.h"
#include "clock/x5_crm.h"
#include "backlight/x5_backlight.h"
#include "common/x5_display_debug.h"
#include <hb_display_log.h>
#include "bridge/x5_syscon.h"

DECLARE_GLOBAL_DATA_PTR;

/*
 * DC8000 plane is A8R8G8B8 (premultiplied blend). Clears and vidconsole must
 * use alpha=0xff or undefined chroma shows through (often green).
 */
#define X5_FB_OPAQUE_BLACK 0xff000000u

/* ARGB8888 / VIDEO_BPP32: bytes per pixel for size and stride math. */
#define X5_PIXEL_SIZE_ARGB32		4U
/* Default FB reservation when DT omits framebuffer-size (4K ARGB, see x5.dtsi). */
#define X5_FB_SIZE_DEFAULT		SZ_32M
/* Default LPWM backlight when get_backlight_config() fails (EVB-style path). */
#define X5_DEFAULT_BL_PWM_ID		1
#define X5_DEFAULT_BL_PWM_CHANNEL	1
#define X5_DEFAULT_BL_DIV_RATIO		24U
#define X5_DEFAULT_BL_PERIOD_TICKS	999U
#define X5_DEFAULT_BL_DUTY_TICKS	750U
/* Max logo partition read: header and body must not exceed (BMP can lie). */
#define X5_LOGO_FILE_MAX_BYTES		SZ_32M

struct x5_display_priv {
	struct display_timing timing;
	u32 fb_base;
	u32 fb_size;
	bool resources_ready;
	bool hw_initialized;
	const struct x5_display_output_ops *output;
};

static struct x5_display_priv *x5_display_get_priv(struct udevice *dev)
{
	return dev_get_priv(dev);
}

/* First UCLASS_VIDEO instance (X5 is single-head in practice). */
static struct x5_display_priv *x5_display_priv_get(void)
{
	struct udevice *dev;
	int ret;

	ret = uclass_first_device_err(UCLASS_VIDEO, &dev);
	if (ret || !dev)
		return NULL;
	return dev_get_priv(dev);
}

static int x5_display_first_video_dev(struct udevice **devp)
{
	return uclass_first_device_err(UCLASS_VIDEO, devp);
}

static struct udevice *x5_display_video_dev(void)
{
	struct udevice *dev;
	int ret;

	ret = uclass_first_device_err(UCLASS_VIDEO, &dev);
	if (ret || !dev)
		return NULL;
	return dev;
}

static void x5_fb_fill_u32(void *fb, size_t nbytes, u32 pix)
{
	u32 *p = fb;
	size_t i, n = nbytes / sizeof(u32);

	for (i = 0; i < n; i++)
		p[i] = pix;
}

/*
 * When /chosen requests HDMI but HDMI detect fails (no bridge / no sink),
 * try DSI unless the user disabled it: setenv x5_hdmi_fb_lcd off; saveenv
 * Default (unset or on): allow DSI fallback.
 */
static bool x5_chosen_hdmi_dsi_fallback_enabled(void)
{
	const char *s = env_get("x5_hdmi_fb_lcd");

	if (!s || !s[0])
		return true;
	if (!strcasecmp(s, "off") || !strcmp(s, "0"))
		return false;
	return true;
}

#if CONFIG_IS_ENABLED(VIDEO_X5_HDMI)
/*
 * HDMI runtime env defaults (RAM env; printenv shows values after boot).
 * x5_hdmi_fb_lcd: on = allow DSI fallback when chosen hdmi detect fails; off = no fallback.
 * x5_hdmi_edid: on = read EDID when available (fallback to bridge default on failure); off = skip EDID.
 */
static void x5_display_hdmi_env_defaults(void)
{
	const char *v;

	v = env_get("x5_hdmi_fb_lcd");
	if (!v || !*v)
		env_set("x5_hdmi_fb_lcd", "on");

	v = env_get("x5_hdmi_edid");
	if (!v || !*v)
		env_set("x5_hdmi_edid", "on");
}

bool x5_hdmi_edid_enabled(void)
{
	const char *s = env_get("x5_hdmi_edid");

	if (!s || !*s)
		return true;
	if (!strcasecmp(s, "off") || !strcmp(s, "0"))
		return false;
	return true;
}
#endif

/*
 * Detect which output path is active by scanning Device-Tree nodes.
 *
 * /chosen x5,display-output = "hdmi" | "dsi" forces a specific path.
 * If chosen is "hdmi" but HDMI detect fails, DSI is tried when built in
 * and x5_hdmi_fb_lcd is not "off".
 * Without an explicit choice, auto-detect: DSI first, then HDMI.
 */
static const struct x5_display_output_ops *x5_detect_output(void)
{
	const char *choice;
	ofnode chosen = ofnode_path("/chosen");

	choice = NULL;
	if (ofnode_valid(chosen))
		choice = ofnode_read_string(chosen, "x5,display-output");

	if (choice) {
#ifdef CONFIG_VIDEO_X5_HDMI
		if (!strcmp(choice, "hdmi")) {
			if (x5_hdmi_output_ops.detect &&
			    x5_hdmi_output_ops.detect() == 0)
				return &x5_hdmi_output_ops;
#if CONFIG_IS_ENABLED(VIDEO_X5_PANEL_DSI)
			if (x5_chosen_hdmi_dsi_fallback_enabled()) {
				DISP_LOG_WARN("chosen hdmi: HDMI detect failed, falling back to DSI\n");
				if (x5_dsi_output_ops.detect &&
				    x5_dsi_output_ops.detect() == 0)
					return &x5_dsi_output_ops;
			} else {
				DISP_LOG_WARN("chosen hdmi: HDMI detect failed; DSI fallback disabled (x5_hdmi_fb_lcd=off)\n");
			}
#endif
			DISP_LOG_WARN("chosen hdmi but detect failed\n");
			return NULL;
		}
#endif
		if (!strcmp(choice, "dsi")) {
			if (x5_dsi_output_ops.detect &&
			    x5_dsi_output_ops.detect() == 0)
				return &x5_dsi_output_ops;
			DISP_LOG_WARN("chosen dsi but detect failed\n");
			return NULL;
		}
		DISP_LOG_WARN("unknown display-output '%s'\n", choice);
		return NULL;
	}

	/* No explicit choice — auto-detect: DSI first, then HDMI */
	if (x5_dsi_output_ops.detect && x5_dsi_output_ops.detect() == 0)
		return &x5_dsi_output_ops;

#ifdef CONFIG_VIDEO_X5_HDMI
	if (x5_hdmi_output_ops.detect && x5_hdmi_output_ops.detect() == 0)
		return &x5_hdmi_output_ops;
#endif

	DISP_LOG_WARN("No display output detected\n");
	return NULL;
}

static int x5_display_setup_default_timing(struct x5_display_priv *priv,
					   struct display_timing *timing)
{
	int ret;

	if (!timing)
		return -EINVAL;
	if (!priv->output || !priv->output->get_timing)
		return -ENODEV;

	ret = priv->output->get_timing(timing);
	if (ret)
		return ret;
	if (!timing->hactive.typ || !timing->vactive.typ) {
		DISP_LOG_ERROR("Invalid timing %ux%u\n",
			       timing->hactive.typ, timing->vactive.typ);
		return -EINVAL;
	}
	return 0;
}

int x5_display_show_logo_auto(int x, int y, bool clear);

/* Initialize display hardware (manual call via command) */
int x5_display_hw_init(void)
{
	int ret;
	struct udevice *dev;
	struct x5_display_priv *priv;
	const struct x5_display_output_ops *out;

	ret = x5_display_first_video_dev(&dev);
	if (ret || !dev) {
		DISP_LOG_ERROR("Resources not ready, no video device (%d)\n", ret);
		return ret ? ret : -ENODEV;
	}
	priv = x5_display_get_priv(dev);
	if (!priv) {
		DISP_LOG_ERROR("Video private data missing\n");
		return -EINVAL;
	}
	out = priv->output;

	/*
	 * DM_FLAG_PRE_RELOC probe may activate the device before relocation
	 * without running the full x5_display_probe(); remove and re-probe.
	 */
	if (!priv->resources_ready) {
		if (dev_get_flags(dev) & DM_FLAG_ACTIVATED) {
			ret = device_remove(dev, DM_REMOVE_NORMAL);
			if (ret) {
				DISP_LOG_ERROR("Video device remove failed (%d)\n", ret);
				return ret;
			}
		}
		ret = device_probe(dev);
		if (ret) {
			DISP_LOG_ERROR("Video device probe failed (%d)\n", ret);
			return ret;
		}
		priv = x5_display_get_priv(dev);
		out = priv->output;
	}

	if (!priv->resources_ready || !out)
	{
		DISP_LOG_ERROR("Resources not ready, run probe first\n");
		return -ENODEV;
	}

	if (priv->hw_initialized)
	{
		DISP_LOG_DEBUG("Already initialized\n");
		return 0;
	}

	DISP_LOG_INFO("Starting hardware initialization (output: %s, %ux%u @%u Hz, flags=0x%x)...\n",
		      out->name,
		      priv->timing.hactive.typ, priv->timing.vactive.typ,
		      priv->timing.pixelclock.typ, priv->timing.flags);

	/* === Step 1: Shared CRM common clocks (PLL, NOC, DC8000, IOMMU) === */
	DISP_LOG_INFO("  [1/7] Initializing common CRM clocks...\n");
	ret = x5_crm_display_common_init(priv->timing.pixelclock.typ);
	if (ret)
	{
		DISP_LOG_ERROR("Common CRM clock init failed (ret=%d)\n", ret);
		return ret;
	}

	/* === Step 2: Output-specific clocks === */
	DISP_LOG_INFO("  [2/7] Initializing %s clocks...\n", out->name);
	if (out->clk_init) {
		ret = out->clk_init(priv->timing.pixelclock.typ);
		if (ret) {
			DISP_LOG_ERROR("%s clock init failed (ret=%d)\n",
				       out->name, ret);
			return ret;
		}
	}

	/*
	 * Reset: each output implements the CRM sequence that matches the
	 * legacy single-path driver (see X5 Display / CRM spec).
	 *
	 * DSI  — x5_crm_display_reset(): DC8000 + DSI-TX asserted/released
	 *        together (required for stable pipe; splitting caused kernel
	 *        handoff / logo issues).
	 * HDMI — x5_crm_common_reset() + x5_crm_bt1120_reset() in hdmi_reset().
	 */
	/* === Step 3: Reset display subsystem === */
	DISP_LOG_INFO("  [3/7] Resetting display subsystem...\n");
	if (out->reset) {
		ret = out->reset();
		if (ret) {
			DISP_LOG_ERROR("%s reset failed (ret=%d)\n",
				       out->name, ret);
			return ret;
		}
	}

	x5_crm_dump_display_clocks();

	/* === Step 4: Output-specific SYSCON bridge routing === */
	DISP_LOG_INFO("  [4/7] Configuring SYSCON bridge for %s...\n", out->name);
	if (out->bridge_init) {
		ret = out->bridge_init(&priv->timing);
		if (ret) {
			DISP_LOG_ERROR("SYSCON bridge init failed (ret=%d)\n", ret);
			return ret;
		}
	}

	/* === Step 5: Output-specific preparation (BT1120/DSI get base, disable) === */
	DISP_LOG_INFO("  [5/7] Preparing %s transmitter...\n", out->name);
	if (out->hw_init) {
		ret = out->hw_init(&priv->timing);
		if (ret) {
			DISP_LOG_ERROR("%s preparation failed (ret=%d)\n",
				       out->name, ret);
			return ret;
		}
	}

	/*
	 * Steps 6-7: DC8000 init / framebuffer / enable are deferred to
	 * x5_display_enable() to guarantee the strict sequence:
	 *   DC8000 init -> framebuffer -> DC8000 enable -> BT1120 -> LT8618
	 */
#ifdef CONFIG_VIDEO_X5_HDMI
	if (out->type == X5_OUTPUT_BT1120_HDMI && x5_hdmi_pattern_boot_requested()) {
		DISP_LOG_INFO("  [6-7] Deferred: LT8618 HDMI color-bar pattern (no DC/BT1120)\n");
		DISP_LOG_INFO("        (disable: x5_hdmi_pattern=0  or turn off CONFIG_VIDEO_X5_HDMI_PATTERN_BOOT)\n");
	} else
#endif
	{
		DISP_LOG_INFO("  [6-7] DC8000 & output enable deferred to enable phase\n");
		DISP_LOG_INFO("        (will run: DC8000 -> FB -> BT1120 -> LT8618)\n");
	}

	priv->hw_initialized = true;

	{
		u32 fb_used = priv->timing.hactive.typ *
			      priv->timing.vactive.typ * X5_PIXEL_SIZE_ARGB32;
		u32 fb_used_mb = fb_used / (1024 * 1024);
		u32 fb_used_kb = (fb_used % (1024 * 1024)) / 1024;
		const char *info_str = (out->get_info) ? out->get_info() : out->name;

		DISP_DUMP_LOG_INFO("\n");
		DISP_DUMP_LOG_INFO("=== Display Configuration ===\n");
		DISP_DUMP_LOG_INFO("Output:      %s\n", info_str);
		DISP_DUMP_LOG_INFO("Resolution:  %dx%d\n",
			   priv->timing.hactive.typ,
			   priv->timing.vactive.typ);
		DISP_DUMP_LOG_INFO("Pixel Clock: %d.%02d MHz\n",
			   priv->timing.pixelclock.typ / 1000000,
			   (priv->timing.pixelclock.typ % 1000000) / 10000);
		DISP_DUMP_LOG_INFO("H-Timing:    active=%d, fp=%d, sync=%d, bp=%d (total=%d)\n",
			   priv->timing.hactive.typ,
			   priv->timing.hfront_porch.typ,
			   priv->timing.hsync_len.typ,
			   priv->timing.hback_porch.typ,
			   priv->timing.hactive.typ + priv->timing.hfront_porch.typ +
			   priv->timing.hsync_len.typ + priv->timing.hback_porch.typ);
		DISP_DUMP_LOG_INFO("V-Timing:    active=%d, fp=%d, sync=%d, bp=%d (total=%d)\n",
			   priv->timing.vactive.typ,
			   priv->timing.vfront_porch.typ,
			   priv->timing.vsync_len.typ,
			   priv->timing.vback_porch.typ,
			   priv->timing.vactive.typ + priv->timing.vfront_porch.typ +
			   priv->timing.vsync_len.typ + priv->timing.vback_porch.typ);
		DISP_DUMP_LOG_INFO("Framebuffer: 0x%08x\n", priv->fb_base);
		DISP_DUMP_LOG_INFO("  Reserved:  %u bytes (%u MB)\n",
			   priv->fb_size, priv->fb_size / (1024 * 1024));
		DISP_DUMP_LOG_INFO("  Used:      %u bytes (%u.%03u MB, ARGB8888)\n",
			   fb_used, fb_used_mb, fb_used_kb);
		DISP_DUMP_LOG_INFO("==============================\n");
		DISP_DUMP_LOG_INFO("\n");
	}

	/* === Step 7: Output-specific backlight === */
	DISP_LOG_INFO("  [7/7] Initializing backlight...\n");
	{
		struct x5_backlight_config bl_cfg;

		ret = out->get_backlight_config ?
		      out->get_backlight_config(&bl_cfg) : -ENODEV;
		DISP_LOG_DEBUG("backlight get_backlight_config ret=%d (%s)\n",
			       ret, ret == -ENODEV ? "skip LPWM; BL may be I2C/kernel-only" :
			       ret ? "fail->default LPWM" : "ok->LPWM init");
		if (ret == -ENODEV) {
			DISP_LOG_INFO("Output %s has no backlight, skipping\n",
				      out->name);
		} else if (ret) {
			DISP_LOG_WARN("Backlight config failed, using default\n");
			bl_cfg.pwm_type = X5_PWM_TYPE_LPWM;
			bl_cfg.pwm_base = LPWM1_BASE;
			bl_cfg.pwm_id = X5_DEFAULT_BL_PWM_ID;
			bl_cfg.channel = X5_DEFAULT_BL_PWM_CHANNEL;
			bl_cfg.period_ns = 0;
			bl_cfg.duty_ns = 0;
			bl_cfg.div_ratio = X5_DEFAULT_BL_DIV_RATIO;
			bl_cfg.period = X5_DEFAULT_BL_PERIOD_TICKS;
			bl_cfg.duty = X5_DEFAULT_BL_DUTY_TICKS;
			bl_cfg.enable_gpio.gpio_num = -1;
			bl_cfg.enable_gpio.active_high = 1;
			ret = 0;
		}

		if (ret == 0) {
			if (bl_cfg.pwm_type == X5_PWM_TYPE_LPWM) {
				ret = x5_crm_lpwm_clk_init(bl_cfg.pwm_id);
				if (ret)
					DISP_LOG_WARN("LPWM%d clock init failed (ret=%d)\n",
						      bl_cfg.pwm_id, ret);
			}

			ret = x5_backlight_init(&bl_cfg);
			if (ret) {
				DISP_LOG_WARN("Backlight init failed (ret=%d)\n", ret);
			}
#if !CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
			else {
				x5_backlight_enable();
				DISP_LOG_DEBUG("backlight x5_backlight_enable() called\n");
			}
#else
			else {
				DISP_LOG_DEBUG("backlight enable skipped (X5_SEAMLESS_DISPLAY)\n");
			}
#endif
		}
	}

	return 0;
}

#if CONFIG_IS_ENABLED(VIDEO_X5_BOOT_UI)
bool x5_boot_ui_is_fbcon(void)
{
	const char *s = env_get("x5_boot_ui");

	return s && !strcmp(s, "console");
}
#endif

/*
 * Enable video output.
 *
 * The sequence mirrors the kernel DRM atomic commit path:
 *
 *   1. DC8000 init + framebuffer       (CRTC mode_set)
 *   2. tx_init  — output-specific      (bridge mode_set, BEFORE CRTC enable)
 *        HDMI: LT8618 chip init + PLL/output config
 *        DSI:  NULL (no-op)
 *   3. DC8000 enable                   (CRTC enable — pixels start flowing)
 *   4. enable  — output-specific       (bridge enable, AFTER CRTC enable)
 *        HDMI: BT1120 online + LT8618 PLL lock + AFE + phase calibration
 *        DSI:  panel init + DSI host video mode
 *
 * CONFIG_VIDEO_X5_BOOT_UI + x5_boot_ui=console: replay CONSOLE_RECORD once.
 */
#if CONFIG_IS_ENABLED(VIDEO_X5_BOOT_UI) && CONFIG_IS_ENABLED(CONSOLE_MUX) && \
	CONFIG_IS_ENABLED(CONSOLE_RECORD)
static void x5_display_replay_console_record_to_vidconsole(void)
{
	struct stdio_dev *vc;
	struct membuff mb;
	char buf[128];
	int len;
	struct udevice *dev;

	if (!(gd->flags & GD_FLG_DEVINIT) || !(gd->flags & GD_FLG_RECORD) ||
	    !gd->console_out.start)
		return;

	if (x5_display_first_video_dev(&dev) || !dev)
		return;

	vc = stdio_get_by_name("vidconsole");
	if (!vc || !vc->puts)
		return;

	/* Drop stale ANSI from early boot (e.g. green background attrs). */
	video_set_default_colors(dev, false);
	vc->puts(vc, "\033[0m");

	mb = gd->console_out;
	while ((len = membuff_get(&mb, buf, sizeof(buf) - 1)) > 0) {
		buf[len] = '\0';
		vc->puts(vc, buf);
	}
}
#endif

int x5_display_enable(void)
{
	int ret;
	struct x5_display_priv *priv = x5_display_priv_get();
	const struct x5_display_output_ops *out;

	if (!priv)
		return -ENODEV;
	out = priv->output;

	if (!priv->hw_initialized || !out) {
		DISP_LOG_ERROR("Hardware not initialized\n");
		return -ENODEV;
	}

	DISP_LOG_INFO("Enabling video output (%s, %ux%u @%u Hz, flags=0x%x)...\n",
		      out->name,
		      priv->timing.hactive.typ, priv->timing.vactive.typ,
		      priv->timing.pixelclock.typ, priv->timing.flags);

#ifdef CONFIG_VIDEO_X5_HDMI
	if (out->type == X5_OUTPUT_BT1120_HDMI && x5_hdmi_pattern_boot_requested()) {
		DISP_LOG_INFO("  [Pattern] LT8618 internal test pattern (skipping steps 1–4)\n");
		ret = x5_hdmi_enable_pattern_boot(&priv->timing);
		if (ret)
			DISP_LOG_ERROR("LT8618 pattern boot failed (%d)\n", ret);
		else
			DISP_LOG_INFO("Video output enabled (HDMI pattern)\n");
		return ret;
	}
#endif

	/* Step 1: DC8000 timing + framebuffer configuration */
	DISP_LOG_INFO("  [Enable 1/4] Initializing DC8000 + framebuffer...\n");
	ret = x5_dc8000_init(&priv->timing);
	if (ret) {
		DISP_LOG_ERROR("DC8000 init failed (ret=%d)\n", ret);
		return ret;
	}
	ret = x5_dc8000_set_framebuffer(priv->fb_base, DC8000_FORMAT_ARGB8888);
	if (ret) {
		DISP_LOG_ERROR("DC8000 framebuffer config failed (ret=%d)\n", ret);
		return ret;
	}

	/*
	 * Step 2: Transmitter early init (before DC8000 outputs pixels).
	 *
	 * Kernel equivalent: bridge_mode_set() is called before CRTC enable.
	 * For HDMI this initialises LT8618 (chip reset, input cfg, HDMI cfg,
	 * PLL range, output timing, AVI) so it is ready to lock once the
	 * pixel clock appears.
	 */
	if (out->tx_init) {
		DISP_LOG_INFO("  [Enable 2/4] TX early init (%s)...\n",
			      out->name);
		ret = out->tx_init(&priv->timing);
		if (ret) {
			DISP_LOG_ERROR("TX init failed (ret=%d)\n", ret);
			return ret;
		}
	}

	/* Step 3: DC8000 enable — pixels start flowing */
	DISP_LOG_INFO("  [Enable 3/4] DC8000 enable (pixels active)...\n");
	x5_dc8000_enable();

	{
		const struct display_timing *tm = &priv->timing;
		u32 htot = tm->hactive.typ + tm->hfront_porch.typ +
			   tm->hsync_len.typ + tm->hback_porch.typ;
		u32 vtot = tm->vactive.typ + tm->vfront_porch.typ +
			   tm->vsync_len.typ + tm->vback_porch.typ;
		unsigned long long px2 = tm->pixelclock.typ;
		u32 refresh_hz = htot && vtot ?
			(u32)(px2 / ((unsigned long long)htot * vtot)) : 0U;

		X5_DISP_LOG_INFO("dc_crtc_commit: crtc=crtc-dc8000 enable=1 active=1 mode_changed=1 "
				 "%ux%u@%u dotclock_khz=%u htot=%u vtot=%u planes_mask=0x0/0x1\n",
				 tm->hactive.typ, tm->vactive.typ, refresh_hz,
				 tm->pixelclock.typ / 1000U, htot, vtot);
	}

	/*
	 * Step 4: Output enable (after DC8000 produces pixel data).
	 *
	 * Kernel equivalent: bridge_enable() runs after CRTC enable.
	 * For HDMI: BT1120 online → LT8618 PLL lock + AFE + phase calib.
	 * For DSI:  panel init sequence → DSI host video mode.
	 */
	if (out->enable) {
		DISP_LOG_INFO("  [Enable 4/4] Output enable (%s)...\n",
			      out->name);
		ret = out->enable();
		if (ret) {
			DISP_LOG_ERROR("Output enable failed (ret=%d)\n", ret);
			return ret;
		}
	}

	/*
	 * --- Boot UI extras (orthogonal to DSI/HDMI) ---
	 * Logo timing: x5_display_boot_board_late() when VIDEO_X5_BOOT_UI + logo mode.
	 * CONSOLE_RECORD replay: VIDEO_X5_BOOT_UI + x5_boot_ui=console (runtime).
	 */
#if CONFIG_IS_ENABLED(VIDEO_X5_BOOT_UI) && CONFIG_IS_ENABLED(CONSOLE_MUX) && \
	CONFIG_IS_ENABLED(CONSOLE_RECORD)
	if (x5_boot_ui_is_fbcon())
		x5_display_replay_console_record_to_vidconsole();
#endif
	DISP_LOG_INFO("Video output enabled\n");
	return 0;
}

#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
static void x5_display_boot_try_seamless_env(bool display_ready)
{
	const char *seamless_env;

	if (!display_ready)
		return;

	seamless_env = env_get("seamless_display");
	if (seamless_env && !strcmp(seamless_env, "0")) {
		SEAMLESS_LOG_DEBUG("disabled by env\n");
	} else if (!seamless_env || strcmp(seamless_env, "1")) {
		if (env_set("seamless_display", "1") == 0)
			SEAMLESS_LOG_DEBUG("set runtime env to 1\n");
		else
			SEAMLESS_LOG_WARNING("failed to set runtime env\n");
	}
}
#endif /* CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY) */

/*
 * board_late_init() delegates here so board code stays free of display policy.
 * When CONFIG_VIDEO_X5_BOOT_UI is off, stdio stays serial-only (no auto video).
 * When on, env x5_boot_ui selects logo vs framebuffer console (console).
 * If x5_boot_ui is missing or empty, default is logo (set here for a stable cmdline).
 * When CONFIG_VIDEO_X5_HDMI, x5_display_hdmi_env_defaults() sets x5_hdmi_fb_lcd
 * and x5_hdmi_edid to on/off when unset (RAM env; printenv shows values).
 */
void x5_display_boot_board_late(void)
{
	bool fbcon;
	const char *ui;

	if (!(gd->flags & GD_FLG_DEVINIT))
		return;

#if CONFIG_IS_ENABLED(VIDEO_X5_HDMI)
	x5_display_hdmi_env_defaults();
#endif

#if !CONFIG_IS_ENABLED(VIDEO_X5_BOOT_UI)
	env_set("stdin", "serial");
	env_set("stdout", "serial");
	env_set("stderr", "serial");
	return;
#else
	ui = env_get("x5_boot_ui");
	if (!ui || !*ui) {
		env_set("x5_boot_ui", "logo");
		ui = "logo";
	}

	fbcon = x5_boot_ui_is_fbcon();

	env_set("stdin", "serial");
	if (fbcon) {
		env_set("stdout", "serial,vidconsole");
		env_set("stderr", "serial,vidconsole");
	} else {
		env_set("stdout", "serial");
		env_set("stderr", "serial");
	}

	if (!fbcon) {
		bool display_ready;

		DISP_LOG_INFO("Auto-initializing display (logo path, x5_boot_ui=%s)...\n",
			      ui);
		display_ready = false;
		if (x5_display_hw_init() == 0) {
#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
			{
				struct x5_display_priv *boot_priv = x5_display_priv_get();
				bool dsi_path = boot_priv && boot_priv->output &&
						boot_priv->output->type == X5_OUTPUT_DSI;

				/*
				 * HDMI: logo before enable so TMDS can show it as soon as
				 * enable() completes. DSI: enable first (DC8000 + panel),
				 * then blit — avoids pre-enable video_bmp_sync issues.
				 */
				if (dsi_path) {
					if (x5_display_enable() == 0) {
						int blen;

						display_ready = true;
						x5_display_show_logo_auto(BMP_ALIGN_CENTER,
									  BMP_ALIGN_CENTER, true);
						blen = x5_backlight_enable();
						DISP_LOG_DEBUG("post-enable x5_backlight_enable() ret=%d\n",
							       blen);
					}
				} else {
					x5_display_show_logo_auto(BMP_ALIGN_CENTER,
								  BMP_ALIGN_CENTER, true);
					SEAMLESS_LOG_DEBUG("logo prepared before video enable\n");
					if (x5_display_enable() == 0) {
						int blen;

						display_ready = true;
						blen = x5_backlight_enable();
						DISP_LOG_DEBUG("post-enable x5_backlight_enable() ret=%d\n",
							       blen);
					}
				}
			}
#else
			if (x5_display_enable() == 0) {
				if (x5_display_show_logo_auto(BMP_ALIGN_CENTER, BMP_ALIGN_CENTER,
							     true) == 0)
					DISP_LOG_INFO("Display logo shown\n");
			}
#endif
		}
#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
		x5_display_boot_try_seamless_env(display_ready);
#endif
	} else {
		bool display_ready = false;

		DISP_LOG_INFO("Auto-initializing display (framebuffer console path, x5_boot_ui=%s)...\n",
			      ui);
		if (x5_display_hw_init() == 0) {
			if (x5_display_enable() == 0) {
				int blen;

				display_ready = true;
				blen = x5_backlight_enable();
				DISP_LOG_DEBUG("post-enable x5_backlight_enable() ret=%d\n", blen);
				if (blen != 0)
					SEAMLESS_LOG_DEBUG("backlight not available (OK for HDMI)\n");
			}
		}
#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
		x5_display_boot_try_seamless_env(display_ready);
#endif
	}
#endif /* VIDEO_X5_BOOT_UI */
}

static bool x5_logo_partition_present(void)
{
	struct blk_desc *dev_desc;
	struct disk_partition part_info;

	dev_desc = blk_get_devnum_by_typename("mmc", 0);
	if (!dev_desc)
		return false;

	return part_get_info_by_name(dev_desc, "logo", &part_info) >= 0;
}

/*
 * Boot logo policy: uncompressed 24/32bpp BMP, bottom-up, dimensions must fit
 * the active display mode (no cropping). Logs details on failure.
 */
static int x5_logo_bmp_check(const void *data, ulong buf_len, u32 panel_w, u32 panel_h)
{
	const struct bmp_image *bmp = data;
	u32 file_size, width, height, compression, data_offset;
	u16 bit_count, planes;
	ulong min_size;

	if (!data || buf_len < sizeof(bmp->header))
		return -EINVAL;

	if (bmp->header.signature[0] != 'B' || bmp->header.signature[1] != 'M') {
		DISP_LOG_ERROR("Logo BMP: invalid signature\n");
		return -EINVAL;
	}

	file_size = le32_to_cpu(bmp->header.file_size);
	if (file_size < sizeof(struct bmp_header) || file_size > buf_len) {
		DISP_LOG_ERROR("Logo BMP: invalid file_size %u (buf %lu)\n",
			       file_size, buf_len);
		return -EINVAL;
	}

	compression = le32_to_cpu(bmp->header.compression);
	if (compression != BMP_BI_RGB) {
		DISP_LOG_ERROR("Logo BMP: need uncompressed BI_RGB\n");
		return -EINVAL;
	}

	bit_count = le16_to_cpu(bmp->header.bit_count);
	if (bit_count != 24 && bit_count != 32) {
		DISP_LOG_ERROR("Logo BMP: need 24 or 32 bpp (got %u)\n", bit_count);
		return -EINVAL;
	}

	planes = le16_to_cpu(bmp->header.planes);
	if (planes != 1) {
		DISP_LOG_ERROR("Logo BMP: planes must be 1\n");
		return -EINVAL;
	}

	width = le32_to_cpu(bmp->header.width);
	height = le32_to_cpu(bmp->header.height);
	if ((s32)height < 0) {
		DISP_LOG_ERROR("Logo BMP: top-down BMP not supported\n");
		return -EINVAL;
	}
	if (!width || !height) {
		DISP_LOG_ERROR("Logo BMP: invalid dimensions\n");
		return -EINVAL;
	}
	if (width > panel_w || height > panel_h) {
		DISP_LOG_ERROR("Logo BMP: %ux%u exceeds panel %ux%u\n",
			       width, height, panel_w, panel_h);
		return -EINVAL;
	}

	data_offset = le32_to_cpu(bmp->header.data_offset);
	min_size = (ulong)data_offset + (ulong)width * height * (bit_count / 8U);
	if (min_size > file_size) {
		DISP_LOG_ERROR("Logo BMP: pixel data exceeds file_size\n");
		return -EINVAL;
	}

	return 0;
}

static void x5_logo_align_axis(int *axis, unsigned long panel_size,
			       unsigned long picture_size)
{
	long panel_picture_delta = panel_size - picture_size;
	long axis_alignment;

	if (*axis == BMP_ALIGN_CENTER)
		axis_alignment = panel_picture_delta / 2;
	else if (*axis < 0)
		axis_alignment = panel_picture_delta + *axis + 1;
	else
		return;

	*axis = max(0, (int)axis_alignment);
}

static int x5_display_ensure_uc_fb(struct udevice *dev, struct video_priv *uc_priv)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	unsigned int stride;

	/*
	 * video_post_probe() only sets line_length when zero; after
	 * DM_FLAG_PRE_RELOC remove+re-probe it can stay stale (e.g. 800-wide)
	 * while xsize matches the active panel (720). Logo blit uses line_length
	 * for row stride — mismatch corrupts memory below the FB mapping.
	 */
	stride = uc_priv->xsize * VNBYTES(uc_priv->bpix);
	if (!stride || !uc_priv->ysize)
		return -EINVAL;
	uc_priv->line_length = stride;
	uc_priv->fb_size = (unsigned long)stride * uc_priv->ysize;

	if (uc_priv->fb)
		return 0;
	if (!plat->base)
		return -EINVAL;

	uc_priv->fb = map_sysmem(plat->base, plat->size);

	return uc_priv->fb ? 0 : -EINVAL;
}

/*
 * Logo-only BMP blit: CPU writes uc_priv->fb, no video_sync() (safe before
 * DC8000/DSI enable). Matches video_bmp_display 24/32bpp paths; bmp_src is a
 * CPU pointer (malloc buffer or embedded __splash_u_boot_logo_begin).
 */
static int x5_logo_blit_bmp(struct udevice *dev, const void *bmp_src, int x, int y)
{
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	const struct bmp_image *bmp = bmp_src;
	uchar *fb, *start, *bmap;
	unsigned long width, height;
	u16 bmp_bpix, bpix;
	int i, j, ret;

	ret = x5_display_ensure_uc_fb(dev, uc_priv);
	if (ret)
		return ret;

	if (!bmp || bmp->header.signature[0] != 'B' ||
	    bmp->header.signature[1] != 'M')
		return -EINVAL;

	width = le32_to_cpu(bmp->header.width);
	height = le32_to_cpu(bmp->header.height);
	if ((s32)height < 0)
		height = (unsigned long)(-(s32)height);

	bmp_bpix = le16_to_cpu(bmp->header.bit_count);
	bpix = VNBITS(uc_priv->bpix);

	if (bmp_bpix != 24 && bmp_bpix != 32)
		return -EINVAL;

	if (bpix != bmp_bpix &&
	    !(bmp_bpix == 24 && bpix == 32))
		return -EPERM;

	x5_logo_align_axis(&x, uc_priv->xsize, width);
	x5_logo_align_axis(&y, uc_priv->ysize, height);

	if (x + width > uc_priv->xsize)
		width = uc_priv->xsize - x;
	if (y + height > uc_priv->ysize)
		height = uc_priv->ysize - y;

	bmap = (uchar *)bmp + le32_to_cpu(bmp->header.data_offset);
	start = (uchar *)(uc_priv->fb +
			  (y + height) * uc_priv->line_length + x * bpix / 8);
	fb = start - uc_priv->line_length;

	switch (bmp_bpix) {
	case 24:
		if (!IS_ENABLED(CONFIG_BMP_24BPP))
			return -EINVAL;
		for (i = 0; i < height; ++i) {
			ulong row_bytes = (width * 3U + 3U) & ~3U;

			for (j = 0; j < width; j++) {
				*fb++ = *bmap++;
				*fb++ = *bmap++;
				*fb++ = *bmap++;
				*fb++ = 0xff;
			}
			fb -= uc_priv->line_length + width * (bpix / 8);
			bmap += row_bytes - width * 3U;
		}
		break;
	case 32:
		if (!IS_ENABLED(CONFIG_BMP_32BPP))
			return -EINVAL;
		for (i = 0; i < height; ++i) {
			ulong row_bytes = (width * 4U + 3U) & ~3U;

			for (j = 0; j < width; j++) {
				*fb++ = *bmap++;
				*fb++ = *bmap++;
				*fb++ = *bmap++;
				*fb++ = *bmap++;
			}
			fb -= uc_priv->line_length + width * (bpix / 8);
			bmap += row_bytes - width * 4U;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* Load BMP logo from "logo" partition. Returns buffer (caller must free) or NULL. */
static void *x5_display_load_logo_from_partition(void)
{
	struct blk_desc *dev_desc;
	struct disk_partition part_info;
	void *logo_buf = NULL;
	lbaint_t blk_count;
	ulong logo_size;
	int ret;

	dev_desc = blk_get_devnum_by_typename("mmc", 0);
	if (!dev_desc) {
		DISP_LOG_DEBUG("MMC device 0 not found\n");
		return NULL;
	}

	ret = part_get_info_by_name(dev_desc, "logo", &part_info);
	if (ret < 0) {
		DISP_LOG_DEBUG("Logo partition not found\n");
		return NULL;
	}

	DISP_LOG_INFO("Found logo partition: start=0x%lx, size=0x%lx blocks\n",
		      (ulong)part_info.start, (ulong)part_info.size);

	logo_buf = malloc(part_info.blksz);
	if (!logo_buf) {
		DISP_LOG_ERROR("Failed to allocate %lu bytes for BMP header\n",
			       (ulong)part_info.blksz);
		return NULL;
	}

	blk_count = blk_dread(dev_desc, part_info.start, 1, logo_buf);
	if (blk_count != 1) {
		DISP_LOG_ERROR("Failed to read BMP header\n");
		free(logo_buf);
		return NULL;
	}

	struct bmp_image *bmp = (struct bmp_image *)logo_buf;
	if (bmp->header.signature[0] != 'B' || bmp->header.signature[1] != 'M') {
		DISP_LOG_ERROR("Invalid BMP signature in logo partition\n");
		free(logo_buf);
		return NULL;
	}

	logo_size = le32_to_cpu(bmp->header.file_size);
	{
		ulong part_bytes = (ulong)part_info.size * (ulong)part_info.blksz;
		ulong max_logo = part_bytes;

		if (max_logo > X5_LOGO_FILE_MAX_BYTES)
			max_logo = X5_LOGO_FILE_MAX_BYTES;

		if (logo_size == 0 || logo_size > max_logo) {
			DISP_LOG_ERROR("Invalid logo BMP size %lu (max %lu, partition %lu)\n",
				       logo_size, max_logo, part_bytes);
			free(logo_buf);
			return NULL;
		}
	}

	/* Reallocate for block-aligned read (blk_dread always fills whole blocks). */
	free(logo_buf);
	{
		lbaint_t blocks_to_read =
			(logo_size + part_info.blksz - 1) / part_info.blksz;
		ulong read_bytes = (ulong)blocks_to_read * (ulong)part_info.blksz;

		logo_buf = malloc(read_bytes);
		if (!logo_buf) {
			DISP_LOG_ERROR("Failed to allocate %lu bytes for logo\n",
				       read_bytes);
			return NULL;
		}

		blk_count = blk_dread(dev_desc, part_info.start, blocks_to_read,
				      logo_buf);
		if (blk_count != blocks_to_read) {
			DISP_LOG_ERROR("Failed to read logo partition (read %lu/%lu blocks)\n",
				       (ulong)blk_count, (ulong)blocks_to_read);
			free(logo_buf);
			return NULL;
		}
	}

	bmp = (struct bmp_image *)logo_buf;
	int width = (int)le32_to_cpu(bmp->header.width);
	int height = (int)le32_to_cpu(bmp->header.height);
	if (height < 0)
		height = -height;

	DISP_LOG_INFO("Logo loaded from partition (%dx%d, %lu bytes)\n",
		      width, height, logo_size);

	return logo_buf;
}

/* Show logo (tries partition first, then built-in fallback) */
int x5_display_show_logo_auto(int x, int y, bool clear)
{
	struct udevice *dev;
	struct x5_display_priv *priv;
	struct video_priv *uc_priv;
	ulong fb_start, fb_end;
	ulong fb_used_size;
	void *logo_data = NULL;
	ulong logo_len = 0;
	bool from_partition = false;
	bool part_present;
	int ret;

	priv = x5_display_priv_get();
	if (!priv || !priv->hw_initialized) {
		DISP_LOG_ERROR("Hardware not initialized\n");
		return -ENODEV;
	}

	dev = x5_display_video_dev();
	if (!dev) {
		DISP_LOG_ERROR("Device not ready\n");
		return -ENODEV;
	}

	uc_priv = dev_get_uclass_priv(dev);

	ret = x5_display_ensure_uc_fb(dev, uc_priv);
	if (ret) {
		DISP_LOG_ERROR("Framebuffer not mapped (%d)\n", ret);
		return ret;
	}

	fb_used_size = uc_priv->xsize * uc_priv->ysize * VNBYTES(uc_priv->bpix);

	if (clear && uc_priv->fb) {
		fb_start = (ulong)uc_priv->fb;
		fb_end = fb_start + fb_used_size;
		invalidate_dcache_range(fb_start, fb_end);
		x5_fb_fill_u32(uc_priv->fb, fb_used_size, X5_FB_OPAQUE_BLACK);
	}

	part_present = x5_logo_partition_present();
	logo_data = x5_display_load_logo_from_partition();
	if (logo_data) {
		from_partition = true;
		logo_len = le32_to_cpu(((struct bmp_image *)logo_data)->header.file_size);
		if (x5_logo_bmp_check(logo_data, logo_len, uc_priv->xsize,
				      uc_priv->ysize) < 0) {
			DISP_LOG_ERROR("Logo partition data is unsuitable\n");
			free(logo_data);
			return -EINVAL;
		}
		DISP_LOG_INFO("Using logo from partition\n");
	} else if (part_present) {
		DISP_LOG_ERROR("Logo partition data is unsuitable\n");
		return -EINVAL;
	}
#ifdef CONFIG_VIDEO_LOGO
	else {
		extern u8 __splash_u_boot_logo_begin[];
		const struct bmp_image *builtin =
			(const struct bmp_image *)__splash_u_boot_logo_begin;

		logo_data = __splash_u_boot_logo_begin;
		logo_len = le32_to_cpu(builtin->header.file_size);
		if (x5_logo_bmp_check(logo_data, logo_len, uc_priv->xsize,
				      uc_priv->ysize) < 0)
			return -EINVAL;
		DISP_LOG_INFO("Using built-in logo (partition not available)\n");
	}
#else
	else {
		DISP_LOG_ERROR("No logo available (partition not found and CONFIG_VIDEO_LOGO not enabled)\n");
		return -ENOENT;
	}
#endif

	DISP_LOG_INFO("Showing logo at (%d, %d)...\n", x, y);

	ret = x5_logo_blit_bmp(dev, logo_data, x, y);
	if (ret) {
		DISP_LOG_ERROR("Logo display failed (ret=%d)\n", ret);
		if (from_partition && logo_data)
			free(logo_data);
		return ret;
	}

	if (from_partition && logo_data)
		free(logo_data);

	if (uc_priv && uc_priv->fb) {
		fb_start = (ulong)uc_priv->fb;
		fb_end = fb_start + fb_used_size;
		flush_dcache_range(fb_start, fb_end);
	}

	DISP_LOG_INFO("Logo displayed\n");
	return 0;
}

#ifdef CONFIG_VIDEO_LOGO
/*
 * Show built-in U-Boot logo
 * This uses the logo embedded via CONFIG_VIDEO_LOGO
 *
 * Position options:
 *   0 = center (default)
 *   1 = top-left
 *   2 = top-right
 *   3 = bottom-left
 *   4 = bottom-right
 *   or use x5_display_show_logo_xy() for custom coordinates
 */
extern u8 __splash_u_boot_logo_begin[];

int x5_display_show_logo_xy(int x, int y, bool clear)
{
	struct udevice *dev;
	struct x5_display_priv *priv;
	struct video_priv *uc_priv;
	ulong fb_start, fb_end;
	ulong fb_used_size;
	int ret;

	priv = x5_display_priv_get();
	if (!priv || !priv->hw_initialized)
	{
		DISP_LOG_ERROR("Hardware not initialized\n");
		return -ENODEV;
	}

	dev = x5_display_video_dev();
	if (!dev)
	{
		DISP_LOG_ERROR("Device not ready\n");
		return -ENODEV;
	}

	uc_priv = dev_get_uclass_priv(dev);

	ret = x5_display_ensure_uc_fb(dev, uc_priv);
	if (ret) {
		DISP_LOG_ERROR("Framebuffer not mapped (%d)\n", ret);
		return ret;
	}

	fb_used_size = uc_priv->xsize * uc_priv->ysize * VNBYTES(uc_priv->bpix);

	if (clear && uc_priv->fb) {
		fb_start = (ulong)uc_priv->fb;
		fb_end = fb_start + fb_used_size;
		invalidate_dcache_range(fb_start, fb_end);
		x5_fb_fill_u32(uc_priv->fb, fb_used_size, X5_FB_OPAQUE_BLACK);
	}

	{
		const struct bmp_image *builtin =
			(const struct bmp_image *)__splash_u_boot_logo_begin;
		ulong logo_len = le32_to_cpu(builtin->header.file_size);

		if (x5_logo_bmp_check(builtin, logo_len, uc_priv->xsize,
				      uc_priv->ysize) < 0)
			return -EINVAL;
	}

	DISP_LOG_INFO("Showing logo at (%d, %d)...\n", x, y);

	ret = x5_logo_blit_bmp(dev, __splash_u_boot_logo_begin, x, y);
	if (ret) {
		DISP_LOG_ERROR("Logo display failed (ret=%d)\n", ret);
		return ret;
	}

	/* Manually flush cache since video_sync relies on flush_dcache flag */
	if (uc_priv && uc_priv->fb) {
		fb_start = (ulong)uc_priv->fb;
		fb_end = fb_start + fb_used_size;
		flush_dcache_range(fb_start, fb_end);
	}

	DISP_LOG_INFO("Logo displayed\n");
	return 0;
}

int x5_display_show_logo(int position, bool clear)
{
	struct udevice *dev;
	struct video_priv *uc_priv;
	struct bmp_image *bmp;
	int x, y;
	int logo_w, logo_h;
	int screen_w, screen_h;

	dev = x5_display_video_dev();
	if (!dev)
	{
		DISP_LOG_ERROR("Device not ready\n");
		return -ENODEV;
	}

	uc_priv = dev_get_uclass_priv(dev);
	screen_w = uc_priv->xsize;
	screen_h = uc_priv->ysize;

	bmp = (struct bmp_image *)__splash_u_boot_logo_begin;
	logo_w = bmp->header.width;
	logo_h = bmp->header.height < 0 ? -bmp->header.height : bmp->header.height;

	switch (position) {
	case 1: /* top-left */
		x = 0;
		y = 0;
		break;
	case 2: /* top-right */
		x = screen_w - logo_w;
		y = 0;
		break;
	case 3: /* bottom-left */
		x = 0;
		y = screen_h - logo_h;
		break;
	case 4: /* bottom-right */
		x = screen_w - logo_w;
		y = screen_h - logo_h;
		break;
	case 0: /* center */
	default:
		x = BMP_ALIGN_CENTER;
		y = BMP_ALIGN_CENTER;
		break;
	}

	return x5_display_show_logo_xy(x, y, clear);
}
#endif /* CONFIG_VIDEO_LOGO */

/*
 * Disable video output
 */
int x5_display_disable(void)
{
	struct x5_display_priv *priv = x5_display_priv_get();
	const struct x5_display_output_ops *out;

	if (!priv || !priv->hw_initialized)
		return 0;

	out = priv->output;

	DISP_LOG_INFO("Disabling video output...\n");

	x5_backlight_disable();

	/* Output-specific: disable transmitter + SYSCON bridge */
	if (out && out->disable)
		out->disable();

	/* Shared: DC8000 */
	x5_dc8000_disable();

	/* Output-specific: gate output clocks */
	if (out && out->clk_disable)
		out->clk_disable();

	/* Shared: gate common clocks */
	x5_crm_common_clk_disable();

	priv->hw_initialized = false;

	return 0;
}

/*
 * Get display status
 */
void x5_display_status(void)
{
	struct x5_display_priv *priv = x5_display_priv_get();
	const struct x5_display_output_ops *out;

	DISP_DUMP_LOG_INFO("=== X5 Display Status ===\n");
	if (!priv) {
		DISP_DUMP_LOG_INFO("Output:          none\n");
		DISP_DUMP_LOG_INFO("Resources ready: NO\n");
		DISP_DUMP_LOG_INFO("HW initialized:  NO\n");
		DISP_DUMP_LOG_INFO("=========================\n");
		return;
	}

	out = priv->output;
	DISP_DUMP_LOG_INFO("Output:          %s\n", out ? out->name : "none");
	DISP_DUMP_LOG_INFO("Resources ready: %s\n", priv->resources_ready ? "YES" : "NO");
	DISP_DUMP_LOG_INFO("HW initialized:  %s\n", priv->hw_initialized ? "YES" : "NO");

	if (priv->resources_ready)
	{
		DISP_DUMP_LOG_INFO("Resolution:      %dx%d\n", priv->timing.hactive.typ,
			   priv->timing.vactive.typ);
		DISP_DUMP_LOG_INFO("Pixel clock:     %d Hz\n", priv->timing.pixelclock.typ);
		DISP_DUMP_LOG_INFO("Framebuffer:     0x%08x (size: %u)\n",
			   priv->fb_base, priv->fb_size);
	}
	DISP_DUMP_LOG_INFO("=========================\n");
}

/*
 * Fill framebuffer with solid color (for testing)
 */
int x5_display_fill_color(u32 color)
{
	volatile u32 *fb;
	u32 pixels;
	u32 i;
	u32 fb_used_size;
	ulong fb_start, fb_end;
	struct x5_display_priv *priv = x5_display_priv_get();

	if (!priv || !priv->resources_ready || !priv->fb_base)
	{
		DISP_LOG_ERROR("Resources not ready or framebuffer not configured\n");
		return -ENODEV;
	}

	fb_used_size = priv->timing.hactive.typ * priv->timing.vactive.typ *
		       X5_PIXEL_SIZE_ARGB32;

	fb = (volatile u32 *)(ulong)priv->fb_base;
	pixels = fb_used_size / X5_PIXEL_SIZE_ARGB32;

	fb_start = priv->fb_base;
	fb_end = fb_start + fb_used_size;

	invalidate_dcache_range(fb_start, fb_end);

	DISP_LOG_INFO("Filling framebuffer with color 0x%08x (%u pixels)\n",
		   color, pixels);

	for (i = 0; i < pixels; i++)
		fb[i] = color;

	/* Memory barrier to ensure all writes complete */
	__asm__ __volatile__("dsb sy" : : : "memory");

	/* Flush cache so DC8000 DMA can see the data */
	flush_dcache_range(fb_start, fb_end);

	__asm__ __volatile__("dsb sy" : : : "memory");

	return 0;
}

/*
 * Probe - allocate resources only, no hardware init
 * video_post_probe() will set fb, clear screen, and create console
 */
static int x5_display_probe(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	struct x5_display_priv *priv = x5_display_get_priv(dev);
	int ret;

	DISP_LOG_INFO("Probing (resource allocation only)\n");

	/* Before relocation, nothing to do */
	if (!(gd->flags & GD_FLG_RELOC))
		return 0;

	/* Detect active output from DT */
	priv->output = x5_detect_output();
	if (!priv->output) {
		DISP_LOG_ERROR("No display output available\n");
		return -ENODEV;
	}
	DISP_LOG_INFO("Output detected: %s\n", priv->output->name);

	ret = x5_display_setup_default_timing(priv, &priv->timing);
	if (ret) {
		DISP_LOG_ERROR("Output timing setup failed (%d)\n", ret);
		return ret;
	}

	/* Get framebuffer address from plat (set in bind / video_reserve) */
	priv->fb_base = plat->base;
	priv->fb_size = plat->size;

	if (!priv->fb_base)
	{
		DISP_LOG_ERROR("No framebuffer allocated!\n");
		return -ENOMEM;
	}

	uc_priv->xsize = priv->timing.hactive.typ;
	uc_priv->ysize = priv->timing.vactive.typ;
	uc_priv->bpix = VIDEO_BPP32;
	uc_priv->format = VIDEO_X8R8G8B8;
	uc_priv->rot = 0;
	uc_priv->line_length = uc_priv->xsize * VNBYTES(uc_priv->bpix);
	uc_priv->fb_size = (unsigned long)uc_priv->line_length * uc_priv->ysize;

	/*
	 * vidconsole ends with video_sync(); generic video-uclass only calls
	 * flush_dcache_range when flush_dcache is true, so DC8000 DMA sees CPU writes.
	 */
	video_set_flush_dcache(dev, true);

	priv->resources_ready = true;
	priv->hw_initialized = false;

	DISP_LOG_INFO("Resources allocated\n");
	DISP_LOG_INFO("  Framebuffer: 0x%08x (size: %u bytes)\n",
		   priv->fb_base, priv->fb_size);
	DISP_LOG_INFO("  Resolution:  %dx%d\n", uc_priv->xsize, uc_priv->ysize);
	DISP_LOG_INFO("  Use 'x5disp init' to initialize hardware\n");
	DISP_LOG_DEBUG("probe() done, returning to framework\n");

	return 0;
}

static int x5_display_bind(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	const void *blob = gd->fdt_blob;
	int node = dev_of_offset(dev);
	u32 fb_addr;
	u32 fb_size;

	DISP_LOG_DEBUG("bind()\n");

	if (!blob)
	{
		DISP_LOG_ERROR("fdt_blob is NULL!\n");
		goto fallback;
	}

	fb_addr = fdtdec_get_uint(blob, node, "framebuffer-base", 0);
	fb_size = fdtdec_get_uint(blob, node, "framebuffer-size", 0);

	if (fb_addr != 0 && fb_size != 0)
	{
		plat->base = fb_addr;
		plat->size = fb_size;
		/* Hide logo since display hardware is not initialized yet */
		plat->hide_logo = true;
		DISP_LOG_INFO("Framebuffer at 0x%08x (size: 0x%x)\n",
			   fb_addr, fb_size);
		return 0;
	}

fallback:
	/*
	 * Request default FB size; plat->base stays 0 here — video_reserve() /
	 * alloc_fb() fills plat->base before probe (see drivers/video/video-uclass.c).
	 */
	DISP_LOG_INFO("No framebuffer in DT, requesting from framework\n");
	plat->size = X5_FB_SIZE_DEFAULT;
	plat->hide_logo = true;

	return 0;
}

static int x5_display_remove(struct udevice *dev)
{
	struct x5_display_priv *priv = x5_display_get_priv(dev);

#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
	const char *seamless = env_get("seamless_display");

	if (seamless && !strcmp(seamless, "1")) {
		if (priv && priv->output &&
		    priv->output->type == X5_OUTPUT_BT1120_HDMI) {
			DISP_LOG_INFO("Seamless: clearing BT1120 pin mux before kernel\n");
			x5_syscon_bt1120_clear_pinmux();
		}
		if (priv)
			priv->resources_ready = false;
		DISP_LOG_DEBUG("Seamless display active, skip hardware disable in remove\n");
		return 0;
	}
#endif

	x5_display_disable();
	if (priv)
		priv->resources_ready = false;
	DISP_LOG_INFO("Removed\n");
	return 0;
}

static const struct udevice_id x5_display_ids[] = {
	{.compatible = "horizon,dc8000"},
	{.compatible = "horizon,x5-display"},
	{}};

U_BOOT_DRIVER(x5_display) = {
	.name = "x5_display",
	.id = UCLASS_VIDEO,
	.of_match = x5_display_ids,
	.bind = x5_display_bind,
	.probe = x5_display_probe,
	.remove = x5_display_remove,
	.priv_auto = sizeof(struct x5_display_priv),
	.flags = DM_FLAG_PRE_RELOC,
};
