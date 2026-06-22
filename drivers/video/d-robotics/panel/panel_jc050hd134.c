// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 * Author: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * JC-050HD134 DSI Panel Driver
 *
 * Panel: JC-050HD134 (Himax HX8394-F IC)
 * Interface: MIPI DSI, 4 lanes, RGB888
 * Supported modes: 720x1280@60Hz
 *
 * Board hooks (device tree, same spirit as kernel x5-evb.dtsi):
 *   reset-gpios, pinctrl-* for reset pin mux
 *   backlight -> pwm-backlight node with pwms = <&lpwmN ch period_ns>
 *   x5,lpwm-params = <div period duty> for the U-Boot LPWM tick backlight path
 *
 * Architecture: Each panel mode bundles its timing and DSI init sequence
 * together, because DSI panels require matched timing/init pairs.
 * Mode selection: prefer DTS "d-robotics,jc050-timing" (slot may also carry
 * wh-cm480-only properties without affecting JC).  If unset or empty, falls
 * back to legacy "timing-name".  No match → DEFAULT_MODE.
 */

#include <common.h>
#include <errno.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <fdtdec.h>
#include <mipi_dsi.h>
#include <dm.h>
#include <asm/gpio.h>
#include "x5_panel.h"
#include "../dsi/x5_dsi_host.h"
#include "../backlight/x5_backlight.h"
#include <hb_display_log.h>

/* Must fit longest payload in mode tables (vendor long writes). */
#define DSI_CMD_DATA_MAX 128

/* ──────────────────────────────────────────────────────────────────────
 * DSI command descriptor — shared by all modes' init sequences
 * ────────────────────────────────────────────────────────────────────── */
struct dsi_cmd {
	uint8_t  cmd;
	uint8_t  data[DSI_CMD_DATA_MAX];
	uint8_t  len;
	uint32_t delay_ms;
};

/* ──────────────────────────────────────────────────────────────────────
 * Panel mode descriptor — each entry is a {timing + init_seq} pair
 * ────────────────────────────────────────────────────────────────────── */
struct panel_mode {
	const char                *name;
	struct display_timing      timing;
	const struct dsi_cmd      *init_seq;
	int                        init_seq_len;
};

/* ──────────────────────────────────────────────────────────────────────
 * Panel-level constants (shared across all modes)
 * ────────────────────────────────────────────────────────────────────── */
#define JC050HD134_LANES	4
#define JC050HD134_FORMAT	MIPI_DSI_FMT_RGB888

/* Defaults for LPWM tick backlight when x5,lpwm-params is omitted */
#define JC050HD134_LPWM_DIV_RATIO_DEFAULT  24U
#define JC050HD134_LPWM_PERIOD_TICKS_DEFAULT 999U
#define JC050HD134_LPWM_DUTY_TICKS_DEFAULT   750U

/* ======================================================================
 * Mode 0: 720x1280 @ 60 Hz  (default, from kernel panel-jc-050hd134.c)
 *
 * Pixel clock : 65 MHz
 * H-total     : 720 + 20 + 32 + 20 = 792
 * V-total     : 1280 + 20 + 4 + 20 = 1324
 * Refresh     : 65000000 / (792 * 1324) ≈ 62.0 Hz
 * ====================================================================== */
static const struct dsi_cmd mode0_init_seq[] = {
	{ 0xB9, {0xF1, 0x12, 0x83}, 3, 0 },
	{ 0xBA, {0x33, 0x81, 0x05, 0xF9, 0x0E, 0x0E, 0x20, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x00, 0x00, 0x44, 0x25, 0x00, 0x91, 0x0A, 0x00, 0x00, 0x02,
		 0x4F, 0xD1, 0x00, 0x00, 0x37}, 27, 0 },
	{ 0xB8, {0x26}, 1, 0 },
	{ 0xBF, {0x02, 0x10, 0x00}, 3, 0 },
	{ 0xB3, {0x07, 0x0B, 0x1E, 0x1E, 0x03, 0xFF, 0x00, 0x00, 0x00, 0x00}, 10, 0 },
	{ 0xC0, {0x73, 0x73, 0x50, 0x50, 0x00, 0x00, 0x08, 0x70, 0x00}, 9, 0 },
	{ 0xBC, {0x46}, 1, 0 },
	{ 0xCC, {0x0B}, 1, 0 },
	{ 0xB4, {0x80}, 1, 0 },
	{ 0xB2, {0xC8, 0x12, 0xA0}, 3, 0 },
	{ 0xE3, {0x07, 0x07, 0x0B, 0x0B, 0x03, 0x0B, 0x00, 0x00, 0x00, 0x00,
		 0xFF, 0x80, 0xC0, 0x10}, 14, 0 },
	{ 0xC1, {0x53, 0x00, 0x32, 0x32, 0x77, 0xF1, 0xFF, 0xFF, 0xCC, 0xCC,
		 0x77, 0x77}, 12, 0 },
	{ 0xB5, {0x09, 0x09}, 2, 0 },
	{ 0xB6, {0xB7, 0xB7}, 2, 0 },
	{ 0xE9, {0xC2, 0x10, 0x0A, 0x00, 0x00, 0x81, 0x80, 0x12, 0x30, 0x00,
		 0x37, 0x86, 0x81, 0x80, 0x37, 0x18, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xBA, 0x46, 0x02, 0x08, 0x28,
		 0x88, 0x88, 0x88, 0x88, 0x88, 0xF8, 0xBA, 0x57, 0x13, 0x18, 0x38, 0x88,
		 0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x00, 0x00, 0x00}, 63, 0 },
	{ 0xEA, {0x07, 0x12, 0x01, 0x01, 0x02, 0x3C, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x8F, 0xBA, 0x31, 0x75, 0x38, 0x18, 0x88, 0x88, 0x88, 0x88,
		 0x88, 0x8F, 0xBA, 0x20, 0x64, 0x28, 0x08, 0x88, 0x88, 0x88, 0x88, 0x88,
		 0x23, 0x10, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x00}, 63, 0 },
	{ 0xE0, {0x00, 0x02, 0x04, 0x1A, 0x23, 0x3F, 0x2C, 0x28, 0x05, 0x09,
		 0x0B, 0x10, 0x11, 0x10, 0x12, 0x12, 0x19, 0x00, 0x02, 0x04, 0x1A, 0x23,
		 0x3F, 0x2C, 0x28, 0x05, 0x09, 0x0B, 0x10, 0x11, 0x10, 0x12, 0x12, 0x19}, 34, 0 },
	{ MIPI_DCS_EXIT_SLEEP_MODE, {}, 0, 250 },
	{ MIPI_DCS_SET_DISPLAY_ON, {}, 0, 50 },
};

/*
 * To add a new mode (e.g. 720x1280@50Hz), copy the block above as
 * "mode1_init_seq[]" with the vendor-supplied init code for that
 * refresh rate, then append a new entry to jc050hd134_modes[].
 */

/* ──────────────────────────────────────────────────────────────────────
 * Mode table — all supported {timing, init_seq} pairs
 * ────────────────────────────────────────────────────────────────────── */
static const struct panel_mode jc050hd134_modes[] = {
	[0] = {
		.name = "720x1280@60",
		.timing = {
			.pixelclock.typ    = 65000000,
			.hactive.typ       = 720,
			.hfront_porch.typ  = 20,
			.hback_porch.typ   = 20,
			.hsync_len.typ     = 32,
			.vactive.typ       = 1280,
			.vfront_porch.typ  = 20,
			.vback_porch.typ   = 20,
			.vsync_len.typ     = 4,
			.flags = DISPLAY_FLAGS_HSYNC_LOW |
				 DISPLAY_FLAGS_VSYNC_LOW,
		},
		.init_seq     = mode0_init_seq,
		.init_seq_len = ARRAY_SIZE(mode0_init_seq),
	},
};

#define NUM_MODES	ARRAY_SIZE(jc050hd134_modes)
#define DEFAULT_MODE	0

/*
 * Module globals: one JC050HD134 instance per U-Boot boot path (sequential
 * x5_panel ops; no concurrent access). active_mode is chosen once; reset GPIO
 * request is idempotent via panel_reset_requested.
 */
static const struct panel_mode *active_mode;

static ofnode jc050hd134_ofnode(void)
{
	ofnode n;

	n = x5_panel_slot_ofnode();
	if (ofnode_valid(n))
		return n;
	return ofnode_by_compatible(ofnode_null(), "jc-050hd134");
}

static const char *jc050hd134_timing_string_from_dt(ofnode node)
{
	const char *s;

	if (!ofnode_valid(node))
		return NULL;
	s = ofnode_read_string(node, "d-robotics,jc050-timing");
	if (s && s[0])
		return s;
	return ofnode_read_string(node, "timing-name");
}

static void jc050hd134_select_timing(void)
{
	const char *mode_name = NULL;
	ofnode node;
	int i;

	if (active_mode)
		return;

	node = jc050hd134_ofnode();
	if (ofnode_valid(node))
		mode_name = jc050hd134_timing_string_from_dt(node);

	if (mode_name) {
		for (i = 0; i < NUM_MODES; i++) {
			if (!strcmp(mode_name, jc050hd134_modes[i].name)) {
				active_mode = &jc050hd134_modes[i];
				PANEL_LOG_INFO("JC050HD134: mode '%s' selected from DTS\n",
					       mode_name);
				return;
			}
		}
		PANEL_LOG_WARN("JC050HD134: DTS mode '%s' not found, using default\n",
			       mode_name);
	}

	active_mode = &jc050hd134_modes[DEFAULT_MODE];
	PANEL_LOG_INFO("JC050HD134: using default mode '%s'\n",
		       active_mode->name);
}

/* Same pattern as uboot/drivers/video/d-robotics/hdmi/x5_lt8618.c */
static struct gpio_desc panel_reset;
static bool panel_reset_requested;

static void jc050hd134_reset_gpio_request(ofnode panel_node)
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
		PANEL_LOG_WARN("JC050HD134: reset-gpios optional (%d)\n", ret);
}

static int jc050hd134_reset(void)
{
	ofnode node = jc050hd134_ofnode();

	PANEL_LOG_DEBUG("JC050HD134: Executing reset sequence\n");

	jc050hd134_reset_gpio_request(node);

	if (dm_gpio_is_valid(&panel_reset)) {
		/* EVB: GPIO_ACTIVE_HIGH reset line (kernel x5-evb.dtsi) */
		dm_gpio_set_value(&panel_reset, 1);
		mdelay(2);
		dm_gpio_set_value(&panel_reset, 0);
		mdelay(2);
		dm_gpio_set_value(&panel_reset, 1);
		mdelay(25);
	}

	PANEL_LOG_DEBUG("JC050HD134: Reset complete\n");
	return 0;
}

static int jc050hd134_send_init_seq(const struct dsi_cmd *seq, int len)
{
	int i, ret;
	uint8_t buf[DSI_CMD_DATA_MAX + 2];

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
			PANEL_LOG_ERROR("JC050HD134: cmd 0x%02x failed (%d)\n",
					cmd->cmd, ret);
			return ret;
		}

		if (cmd->delay_ms)
			mdelay(cmd->delay_ms);
	}
	return 0;
}

static int jc050hd134_init(void)
{
	int ret;

	jc050hd134_select_timing();

	PANEL_LOG_INFO("JC050HD134: Sending init sequence for mode '%s' (%d cmds)\n",
		       active_mode->name, active_mode->init_seq_len);

	ret = jc050hd134_send_init_seq(active_mode->init_seq,
				       active_mode->init_seq_len);
	if (ret)
		return ret;

	PANEL_LOG_INFO("JC050HD134: Init complete\n");
	return 0;
}

static int jc050hd134_get_timing(struct display_timing *timing)
{
	jc050hd134_select_timing();
	*timing = active_mode->timing;
	return 0;
}

static int jc050hd134_get_lanes(void)
{
	return JC050HD134_LANES;
}

static enum mipi_dsi_pixel_format jc050hd134_get_format(void)
{
	return JC050HD134_FORMAT;
}

static const char *jc050hd134_get_name(void)
{
	return "jc-050hd134";
}

/*
 * Resolve PWM for x5_backlight: prefer panel -> backlight -> pwms (Linux),
 * else panel "backlight-pwm" / "pwms" with #pwm-cells on the LPWM provider.
 */
static int jc050hd134_resolve_pwm(ofnode panel,
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

static int jc050hd134_get_backlight_config(struct x5_backlight_config *config)
{
	ofnode panel = jc050hd134_ofnode();
	struct ofnode_phandle_args pwm;
	struct udevice *pwm_dev;
	fdt_addr_t addr;
	u32 tickv[3];
	int ret;

	if (!ofnode_valid(panel))
		return -ENODEV;

	ret = jc050hd134_resolve_pwm(panel, &pwm);
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

	if (((unsigned long)addr & LPWM_MMIO_MASK) == LPWM0_BASE)
		config->pwm_id = 0;
	else if (((unsigned long)addr & LPWM_MMIO_MASK) == LPWM1_BASE)
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
		/* Sensible default if property omitted */
		config->div_ratio = JC050HD134_LPWM_DIV_RATIO_DEFAULT;
		config->period = JC050HD134_LPWM_PERIOD_TICKS_DEFAULT;
		config->duty = JC050HD134_LPWM_DUTY_TICKS_DEFAULT;
	}

	config->enable_gpio.gpio_num = -1;
	config->enable_gpio.active_high = 1;

	return 0;
}

/* ──────────────────────────────────────────────────────────────────────
 * Registration
 * ────────────────────────────────────────────────────────────────────── */
static const struct x5_panel_ops jc050hd134_ops = {
	.reset              = jc050hd134_reset,
	.init               = jc050hd134_init,
	.get_timing         = jc050hd134_get_timing,
	.get_lanes          = jc050hd134_get_lanes,
	.get_format         = jc050hd134_get_format,
	.get_name           = jc050hd134_get_name,
	.get_backlight_config = jc050hd134_get_backlight_config,
};

const struct x5_panel panel_jc050hd134 = {
	.compatible = "jc-050hd134",
	.name       = "JC-050HD134 (720x1280)",
	.ops        = &jc050hd134_ops,
};
