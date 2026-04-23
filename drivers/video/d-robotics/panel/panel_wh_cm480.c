// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 * Author: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * WH-CM480 DSI Panel — U-Boot (compatible: "wh-cm480")
 *
 * Doc: repo WH-CM480_LCD驱动说明.md — kernel binding matches
 * drivers/gpu/drm/panel/panel-wh-cm480.c (800×480, 1 lane, RGB888, video mode).
 *
 * Multiple timings: prefer DTS "d-robotics,wh-cm480-timing" = "<name>";
 * if unset or empty, legacy "timing-name" is used.  Names must match
 * wh_cm480_modes[].name exactly.
 *
 * Backlight (WH-CM480_LCD驱动说明.md): Waveshare 4.3" uses I2C
 * hobot_bl_controller (kernel drivers/video/backlight/hb_bl.c), not LPWM.
 *
 * Optional DT on the wh-cm480 panel node (RDK / Waveshare):
 *   d-robotics,hb-bl-i2c = phandle to I2C bus (e.g. &i2c3)
 *   d-robotics,hb-bl-addr = <0x45> (7-bit address, default 0x45)
 *   d-robotics,hb-bl-default-brightness = <200> (1–255, default 200)
 * When d-robotics,hb-bl-i2c is set, get_backlight_config() returns -ENODEV so
 * the generic LPWM path is skipped; panel init runs the same I2C sequence as hb_bl.
 * For EVB-style LPWM only, omit d-robotics,hb-bl-i2c and set backlight = <&dsi_backlight>.
 *
 * Unified slot DT (no hb-bl phandle): if board enables /i2c@340e0000 (hobot-x5 &i2c3), this
 * driver still runs hb_bl @0x45 and skips LPWM — matches Waveshare WH without editing the slot.
 */

#include <common.h>
#include <errno.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <fdtdec.h>
#include <mipi_dsi.h>
#include <dm.h>
#include <dm/ofnode.h>
#include <i2c.h>
#include <asm/gpio.h>
#include "x5_panel.h"
#include "../dsi/x5_dsi_host.h"
#include "../backlight/x5_backlight.h"
#include <hb_display_log.h>

struct dsi_cmd {
	u8  cmd;
	u8  data[128];
	u8  len;
	u32 delay_ms;
};

struct panel_mode {
	const char             *name;
	struct display_timing   timing;
	const struct dsi_cmd   *init_seq;
	int                     init_seq_len;
	unsigned int            dsi_lanes;
};

#define WH_CM480_FORMAT	MIPI_DSI_FMT_RGB888

/* ======================================================================
 * Mode 0 — Kernel / Waveshare 4.3" DSI (WH-CM480_LCD驱动说明.md §4)
 *
 * Source: kernel drivers/gpu/drm/panel/panel-wh-cm480.c :: wh_cm480_mode
 *   .clock = 29070   (kHz)  → pixelclock 29070000 Hz
 *   h: 800 + 90 + 2 + 58 = 950
 *   v: 480 + 7 + 2 + 21 = 510
 *   ~59.98 Hz
 * lanes = 1, MIPI_DSI_MODE_VIDEO | VIDEO_SYNC_PULSE (handled in DSI host)
 * ======================================================================
 */
static const struct panel_mode wh_cm480_modes[] = {
	[0] = {
		.name = "800x480@60",
		.timing = {
			.pixelclock.typ    = 29070000,
			.hactive.typ       = 800,
			.hfront_porch.typ  = 90,
			.hback_porch.typ   = 58,
			.hsync_len.typ     = 2,
			.vactive.typ       = 480,
			.vfront_porch.typ  = 7,
			.vback_porch.typ   = 21,
			.vsync_len.typ     = 2,
			.flags = DISPLAY_FLAGS_HSYNC_LOW |
				 DISPLAY_FLAGS_VSYNC_LOW,
		},
		.init_seq     = NULL,
		.init_seq_len = 0,
		.dsi_lanes    = 1,
	},
	/* Mode 1 — Legacy strip / product verification (600×1280, 2 lanes) */
	[1] = {
		.name = "600x1280@56",
		.timing = {
			.pixelclock.typ    = 66720000,
			.hactive.typ       = 600,
			.hfront_porch.typ  = 110,
			.hback_porch.typ   = 110,
			.hsync_len.typ     = 92,
			.vactive.typ       = 1280,
			.vfront_porch.typ  = 13,
			.vback_porch.typ   = 13,
			.vsync_len.typ     = 6,
			.flags = DISPLAY_FLAGS_HSYNC_LOW |
				 DISPLAY_FLAGS_VSYNC_LOW,
		},
		.init_seq     = NULL,
		.init_seq_len = 0,
		.dsi_lanes    = 2,
	},
};

/*
 * To add a mode: optional modeN_init_seq[], append jc050hd134-style entry with
 * unique .name; set d-robotics,wh-cm480-timing (or timing-name) in DTS to that name.
 */

#define NUM_MODES	ARRAY_SIZE(wh_cm480_modes)
#define DEFAULT_MODE	0

static const struct panel_mode *active_mode;

static ofnode wh_cm480_ofnode(void)
{
	ofnode n;

	n = x5_panel_slot_ofnode();
	if (ofnode_valid(n))
		return n;
	return ofnode_by_compatible(ofnode_null(), "wh-cm480");
}

/* Waveshare hb_bl on Hobot X5 hobot-x5.dtsi: i2c3 reg 0x340e0000 (see dtc path). */
#define WH_CM480_HB_BL_DEFAULT_I2C_PATH	"/i2c@340e0000"

static ofnode wh_cm480_hb_bl_default_i2c_bus(void)
{
	ofnode bus = ofnode_path(WH_CM480_HB_BL_DEFAULT_I2C_PATH);

	if (!ofnode_valid(bus))
		return ofnode_null();
	if (!ofnode_is_enabled(bus))
		return ofnode_null();
	return bus;
}

/* Hobot I2C backlight (hb_bl.c): same wire protocol, panel-local bring-up. */
static struct udevice *wh_hb_bl_chip;
static bool wh_hb_bl_done;

static int wh_cm480_hb_bl_send_raw(struct udevice *chip, const u8 *buf, int len)
{
	int ret = dm_i2c_write(chip, 0, buf, len);

	if (ret < 0)
		return ret;
	mdelay(20);
	return 0;
}

static int wh_cm480_hb_bl_try_enable(void)
{
	ofnode panel = wh_cm480_ofnode();
	struct udevice *bus;
	ofnode bus_node;
	u32 ph, addr, bright;
	u8 b, recv;
	/* Match kernel hb_bl controller_screen_power_up(): "8501" not "8500". */
	u8 p85[] = { 0x85, 0x01 };
	u8 p81[] = { 0x81, 0x04 };
	u8 p86_on[] = { 0x86, 0x01 };
	u8 setb[2];
	int ret;

	if (wh_hb_bl_done)
		return 0;

	if (!ofnode_valid(panel))
		return -ENODEV;

	if (ofnode_read_u32_index(panel, "d-robotics,hb-bl-i2c", 0, &ph) == 0) {
		bus_node = ofnode_get_by_phandle(ph);
	} else {
		bus_node = wh_cm480_hb_bl_default_i2c_bus();
		if (!ofnode_valid(bus_node))
			return 0;
	}

	if (!ofnode_valid(bus_node)) {
		PANEL_LOG_WARN("WH-CM480 hb_bl: invalid I2C phandle\n");
		return -EINVAL;
	}

	addr = ofnode_read_u32_default(panel, "d-robotics,hb-bl-addr", 0x45);
	bright = ofnode_read_u32_default(panel, "d-robotics,hb-bl-default-brightness",
					 200);
	if (bright < 1)
		bright = 1;
	if (bright > 255)
		bright = 255;

	ret = uclass_get_device_by_ofnode(UCLASS_I2C, bus_node, &bus);
	if (ret) {
		PANEL_LOG_ERROR("WH-CM480 hb_bl: I2C bus probe failed (%d)\n", ret);
		return ret;
	}

	ret = i2c_get_chip(bus, addr, 0, &wh_hb_bl_chip);
	if (ret) {
		PANEL_LOG_ERROR("WH-CM480 hb_bl: i2c_get_chip 0x%02x failed (%d)\n",
				addr, ret);
		return ret;
	}

	b = 0x80;
	ret = wh_cm480_hb_bl_send_raw(wh_hb_bl_chip, &b, 1);
	if (ret < 0)
		goto err;

	ret = dm_i2c_read(wh_hb_bl_chip, 0, &recv, 1);
	if (ret < 0)
		goto err;
	mdelay(20);

	if (recv != 0xde && recv != 0xc3) {
		PANEL_LOG_WARN("WH-CM480 hb_bl: probe read 0x%02x (expect 0xDE/0xC3)\n",
			       recv);
		ret = -EIO;
		goto err;
	}

	ret = wh_cm480_hb_bl_send_raw(wh_hb_bl_chip, p85, 2);
	if (ret < 0)
		goto err;
	ret = wh_cm480_hb_bl_send_raw(wh_hb_bl_chip, p81, 2);
	if (ret < 0)
		goto err;
	ret = wh_cm480_hb_bl_send_raw(wh_hb_bl_chip, p86_on, 2);
	if (ret < 0)
		goto err;

	setb[0] = 0x86;
	setb[1] = (u8)bright;
	ret = dm_i2c_write(wh_hb_bl_chip, 0, setb, 2);
	if (ret < 0)
		goto err;

	wh_hb_bl_done = true;
	PANEL_LOG_INFO("WH-CM480 hb_bl: I2C backlight on (addr=0x%02x bright=%u)\n",
		       addr, bright);
	return 0;

err:
	PANEL_LOG_ERROR("WH-CM480 hb_bl: enable failed (%d)\n", ret);
	wh_hb_bl_chip = NULL;
	return ret;
}

static const char *wh_cm480_timing_string_from_dt(ofnode node)
{
	const char *s;

	if (!ofnode_valid(node))
		return NULL;
	s = ofnode_read_string(node, "d-robotics,wh-cm480-timing");
	if (s && s[0])
		return s;
	return ofnode_read_string(node, "timing-name");
}

static void wh_cm480_select_timing(void)
{
	const char *mode_name = NULL;
	ofnode node;
	int i;

	if (active_mode)
		return;

	node = wh_cm480_ofnode();
	if (ofnode_valid(node))
		mode_name = wh_cm480_timing_string_from_dt(node);

	if (mode_name) {
		for (i = 0; i < NUM_MODES; i++) {
			if (!strcmp(mode_name, wh_cm480_modes[i].name)) {
				active_mode = &wh_cm480_modes[i];
				PANEL_LOG_INFO("WH-CM480: mode '%s' %ux%u lanes=%u (DTS)\n",
					       active_mode->name,
					       active_mode->timing.hactive.typ,
					       active_mode->timing.vactive.typ,
					       active_mode->dsi_lanes);
				return;
			}
		}
		PANEL_LOG_WARN("WH-CM480: DTS mode '%s' unknown; using default '%s' (%ux%u, %u lane(s))\n",
			       mode_name, wh_cm480_modes[DEFAULT_MODE].name,
			       (unsigned int)wh_cm480_modes[DEFAULT_MODE].timing.hactive.typ,
			       (unsigned int)wh_cm480_modes[DEFAULT_MODE].timing.vactive.typ,
			       wh_cm480_modes[DEFAULT_MODE].dsi_lanes);
		if (strstr(mode_name, "720") && strstr(mode_name, "1280"))
			PANEL_LOG_WARN(
				"WH-CM480: DT mode '%s' is not a WH mode; using %s. "
				"Set d-robotics,wh-cm480-timing on the slot (e.g. 800x480@60). "
				"I2C hb_bl uses /i2c@340e0000 when the slot omits d-robotics,hb-bl-i2c and that bus is enabled.\n",
				mode_name, wh_cm480_modes[DEFAULT_MODE].name);
	}

	active_mode = &wh_cm480_modes[DEFAULT_MODE];
	PANEL_LOG_INFO("WH-CM480: default mode '%s' %ux%u lanes=%u\n",
		       active_mode->name,
		       active_mode->timing.hactive.typ,
		       active_mode->timing.vactive.typ,
		       active_mode->dsi_lanes);
}

static struct gpio_desc panel_reset;
static bool panel_reset_requested;

static void wh_cm480_reset_gpio_request(ofnode panel_node)
{
	int ret;

	if (panel_reset_requested)
		return;
	panel_reset_requested = true;

	if (!ofnode_valid(panel_node))
		return;

	ret = gpio_request_by_name_nodev(panel_node, "reset-gpios", 0,
					 &panel_reset, GPIOD_IS_OUT);
	if (ret)
		PANEL_LOG_WARN("WH-CM480: reset-gpios optional (%d)\n", ret);
}

static int wh_cm480_reset(void)
{
	ofnode node = wh_cm480_ofnode();

	PANEL_LOG_DEBUG("WH-CM480: reset\n");

	wh_cm480_reset_gpio_request(node);

	if (dm_gpio_is_valid(&panel_reset)) {
		dm_gpio_set_value(&panel_reset, 1);
		mdelay(2);
		dm_gpio_set_value(&panel_reset, 0);
		mdelay(2);
		dm_gpio_set_value(&panel_reset, 1);
		mdelay(25);
	}

	return 0;
}

static int wh_cm480_send_init_seq(const struct dsi_cmd *seq, int len)
{
	int i, ret;
	u8 buf[130];

	if (!seq || !len)
		return 0;

	for (i = 0; i < len; i++) {
		const struct dsi_cmd *cmd = &seq[i];

		if (cmd->len == 0) {
			ret = x5_dsi_dcs_write_0(cmd->cmd);
		} else if (cmd->len == 1) {
			ret = x5_dsi_dcs_write_1(cmd->cmd, cmd->data[0]);
		} else {
			buf[0] = cmd->cmd;
			memcpy(&buf[1], cmd->data, cmd->len);
			ret = x5_dsi_dcs_write_buffer(buf, cmd->len + 1);
		}

		if (ret) {
			PANEL_LOG_ERROR("WH-CM480: cmd 0x%02x failed (%d)\n",
					cmd->cmd, ret);
			return ret;
		}

		if (cmd->delay_ms)
			mdelay(cmd->delay_ms);
	}
	return 0;
}

static int wh_cm480_init(void)
{
	int ret;

	wh_cm480_select_timing();

	if (!active_mode->init_seq_len) {
		PANEL_LOG_INFO("WH-CM480: no DSI init sequence for mode '%s'\n",
			       active_mode->name);
		wh_cm480_hb_bl_try_enable();
		return 0;
	}

	ret = wh_cm480_send_init_seq(active_mode->init_seq,
				     active_mode->init_seq_len);
	if (ret)
		return ret;

	PANEL_LOG_INFO("WH-CM480: init done\n");
	wh_cm480_hb_bl_try_enable();
	return 0;
}

static int wh_cm480_get_timing(struct display_timing *timing)
{
	wh_cm480_select_timing();
	*timing = active_mode->timing;
	return 0;
}

static int wh_cm480_get_lanes(void)
{
	wh_cm480_select_timing();
	return (int)active_mode->dsi_lanes;
}

static enum mipi_dsi_pixel_format wh_cm480_get_format(void)
{
	return WH_CM480_FORMAT;
}

static const char *wh_cm480_get_name(void)
{
	return "wh-cm480";
}

static int wh_cm480_resolve_pwm(ofnode panel,
				struct ofnode_phandle_args *pwm_args)
{
	ofnode bl_node;
	u32 ph;
	int ret;

	ret = ofnode_read_u32_index(panel, "backlight", 0, &ph);
	if (!ret) {
		bl_node = ofnode_get_by_phandle(ph);
		if (ofnode_valid(bl_node)) {
			ret = ofnode_parse_phandle_with_args(bl_node, "pwms",
							     "#pwm-cells", 0,
							     0, pwm_args);
			if (!ret)
				return 0;
		}
	}

	ret = ofnode_parse_phandle_with_args(panel, "backlight-pwm",
					     "#pwm-cells", 0, 0, pwm_args);
	if (!ret)
		return 0;

	return ofnode_parse_phandle_with_args(panel, "pwms", "#pwm-cells",
					      0, 0, pwm_args);
}

static int wh_cm480_get_backlight_config(struct x5_backlight_config *config)
{
	ofnode panel = wh_cm480_ofnode();
	struct ofnode_phandle_args pwm;
	struct udevice *pwm_dev;
	fdt_addr_t addr;
	u32 tickv[3];
	u32 ph;
	int ret;

	if (!ofnode_valid(panel))
		return -ENODEV;

	/* Hobot I2C backlight path: never use LPWM (see file header). */
	if (ofnode_read_u32_index(panel, "d-robotics,hb-bl-i2c", 0, &ph) == 0)
		return -ENODEV;
	if (ofnode_valid(wh_cm480_hb_bl_default_i2c_bus()))
		return -ENODEV;

	/* No DT backlight: skip LPWM (Linux may use I2C hb_bl only). */
	if (ofnode_read_u32_index(panel, "backlight", 0, &ph) != 0)
		return -ENODEV;

	ret = wh_cm480_resolve_pwm(panel, &pwm);
	if (ret)
		return ret;

	ret = uclass_get_device_by_ofnode(UCLASS_PWM, pwm.node, &pwm_dev);
	if (ret)
		return ret;

	addr = dev_read_addr(pwm_dev);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	config->pwm_type = X5_PWM_TYPE_LPWM;
	config->pwm_base = addr;

	if ((addr & 0xffff0000UL) == 0x34100000UL)
		config->pwm_id = 0;
	else if ((addr & 0xffff0000UL) == 0x34110000UL)
		config->pwm_id = 1;
	else
		return -EINVAL;

	config->channel = pwm.args_count > 0 ? (int)pwm.args[0] : 0;
	config->period_ns = pwm.args_count > 1 ? pwm.args[1] : 0;
	config->duty_ns = 0;

	if (ofnode_read_u32_array(panel, "x5,lpwm-params", tickv, 3) == 0) {
		config->div_ratio = tickv[0];
		config->period = tickv[1];
		config->duty = tickv[2];
	} else {
		config->div_ratio = 24;
		config->period = 999;
		config->duty = 750;
	}

	config->enable_gpio.gpio_num = -1;
	config->enable_gpio.active_high = 1;

	return 0;
}

static const struct x5_panel_ops wh_cm480_ops = {
	.reset = wh_cm480_reset,
	.init = wh_cm480_init,
	.get_timing = wh_cm480_get_timing,
	.get_lanes = wh_cm480_get_lanes,
	.get_format = wh_cm480_get_format,
	.get_name = wh_cm480_get_name,
	.get_backlight_config = wh_cm480_get_backlight_config,
};

const struct x5_panel panel_wh_cm480 = {
	.compatible = "wh-cm480",
	.name       = "WH-CM480",
	.ops        = &wh_cm480_ops,
};
