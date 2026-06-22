// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 *
 * ST77031 Panel Driver (600x1280, 2 lanes)
 * For RDK Module Board
 *
 * Hardware Configuration:
 * - Reset GPIO: AON GPIO pin 1
 * - Backlight: PWM2 channel 1
 *
 * Init sequence from customer's panel-st77031.c driver
 *
 * Mode label (single timing in U-Boot; DT for alignment with jc050 / wh-cm480):
 *   d-robotics,st77031-timing = "600x1280@56"; optional; legacy timing-name fallback.
 * Unknown non-empty string: WARN, still use built-in timing.
 */

#include <common.h>
#include <dm.h>
#include <dm/ofnode.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <asm/io.h>
#include <fdtdec.h>
#include <mipi_dsi.h>
#include "x5_panel.h"
#include "../dsi/x5_dsi_host.h"
#include "../backlight/x5_backlight.h"
#include <hb_display_log.h>

/* GPIO Configuration */
#define GPIO_BASE           0x34040000  /* AON GPIO */
#define GPIO_DR_OFFSET      0x00
#define GPIO_DDR_OFFSET     0x04
#define RESET_PIN           1

/* Pinmux Configuration
 * AON GPIO1 uses INVALID_PINMUX in kernel DTS, meaning it's a dedicated GPIO pin
 * that doesn't need pinmux configuration. We still set it for safety.
 */
#define IOMUX_BASE          0x34050000  /* AON IOMUX */
#define PINMUX_OFFSET       0x00        /* AON_PINMUX_0 */
#define PINMUX_SHIFT        2           /* Bits[3:2] for GPIO1 */
#define PINMUX_VALUE        0x0         /* MUX_ALT0 for GPIO */
#define PINMUX_NEEDED       0           /* Set to 0 if pinmux not needed */

/* Delays */
#define DELAY_SLEEP_OUT_MS  250
#define DELAY_DISPLAY_ON_MS 50

/* Panel Specification */
#define PANEL_WIDTH         600
#define PANEL_HEIGHT        1280
#define PANEL_LANES         2
#define PANEL_CLOCK         66720000  /* 66.72 MHz */

/* Panel Timing */
#define PANEL_HSYNC_LEN     92
#define PANEL_HBP           110
#define PANEL_HFP           110
#define PANEL_VSYNC_LEN     6
#define PANEL_VBP           13
#define PANEL_VFP           13

/* Display Timing (from customer's x5-rdk.dtsi) */
static const struct display_timing st77031_timing = {
	.pixelclock.typ = PANEL_CLOCK,
	.hactive.typ = PANEL_WIDTH,
	.hfront_porch.typ = PANEL_HFP,
	.hback_porch.typ = PANEL_HBP,
	.hsync_len.typ = PANEL_HSYNC_LEN,
	.vactive.typ = PANEL_HEIGHT,
	.vfront_porch.typ = PANEL_VFP,
	.vback_porch.typ = PANEL_VBP,
	.vsync_len.typ = PANEL_VSYNC_LEN,
	.flags = DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_VSYNC_LOW,
};

/* Must match d-robotics,st77031-timing on unified slot (same label as WH 600x1280 mode). */
#define ST77031_TIMING_DT_NAME	"600x1280@56"

/* Init Sequence (from customer's panel-st77031.c) */
static const struct {
	u8 cmd;
	u8 data[128];
	u8 len;
	u32 delay_ms;
} init_sequence[] = {
	{ 0xB9, {0xF1, 0x12, 0x83}, 3, 0 },
	{ 0xB1, {0x00, 0x00, 0x00, 0xDA, 0x80}, 5, 0 },
	{ 0xB2, {0xC8, 0x04, 0x30}, 3, 0 },
	{ 0xB3, {0x10, 0x10, 0x28, 0x28, 0x03, 0xFF, 0x00, 0x00, 0x00, 0x00}, 10, 0 },
	{ 0xB4, {0x80}, 1, 0 },
	{ 0xB5, {0x0A, 0x0A}, 2, 0 },
	{ 0xB6, {0x9D, 0x9D}, 2, 0 },
	{ 0xB8, {0x26, 0x22, 0xF0, 0x13}, 4, 0 },
	{ 0xBA, {0x31, 0x81, 0x05, 0xF9, 0x0E, 0x0E, 0x20, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x00, 0x00, 0x44, 0x25, 0x00, 0x90, 0x0A, 0x00,
		 0x00, 0x01, 0x4F, 0x01, 0x00, 0x00, 0x37}, 27, 0 },
	{ 0xBC, {0x47}, 1, 0 },
	{ 0xBF, {0x02, 0x11, 0x00}, 3, 0 },
	{ 0xC0, {0x73, 0x73, 0x50, 0x50, 0x00, 0x00, 0x12, 0x70, 0x00}, 9, 0 },
	{ 0xC1, {0x57, 0x00, 0x32, 0x32, 0x77, 0xE1, 0xFF, 0xFF, 0xCC, 0xCC,
		 0x77, 0x77}, 12, 0 },
	{ 0xC6, {0x82, 0x00, 0xBF, 0xFF, 0x00, 0xFF}, 6, 0 },
	{ 0xC7, {0xB8, 0x00, 0x0A, 0x10, 0x01, 0x09}, 6, 0 },
	{ 0xC8, {0x10, 0x40, 0x1E, 0x02}, 4, 0 },
	{ 0xCC, {0x0B}, 1, 0 },
	{ 0xE0, {0x00, 0x00, 0x00, 0x1B, 0x25, 0x3F, 0x24, 0x1C, 0x05, 0x0A,
		 0x0C, 0x0E, 0x10, 0x0E, 0x11, 0x11, 0x17, 0x00, 0x00, 0x00,
		 0x1B, 0x25, 0x3F, 0x24, 0x1C, 0x05, 0x0A, 0x0C, 0x0E, 0x10,
		 0x0E, 0x11, 0x11, 0x17}, 34, 0 },
	{ 0xE3, {0x07, 0x07, 0x0B, 0x0B, 0x0B, 0x0B, 0x00, 0x00, 0x00, 0x00,
		 0xFF, 0x00, 0xC0, 0x10}, 14, 0 },
	{ 0xE9, {0xC8, 0x10, 0x07, 0x05, 0x02, 0x80, 0x81, 0x12, 0x31, 0x23,
		 0x4F, 0x86, 0x80, 0x81, 0x47, 0x16, 0x00, 0x00, 0x05, 0x00,
		 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x48, 0x18,
		 0xFA, 0xB3, 0x17, 0x58, 0x88, 0x88, 0x88, 0x88, 0x88, 0x48,
		 0x08, 0xFA, 0xB2, 0x06, 0x48, 0x88, 0x88, 0x88, 0x88, 0x88,
		 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x00}, 63, 0 },
	{ 0xEA, {0x96, 0x12, 0x01, 0x01, 0x02, 0x96, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x4F, 0x08, 0x8A, 0xB4, 0x60, 0x28, 0x88, 0x88,
		 0x88, 0x88, 0x88, 0x4F, 0x18, 0x8A, 0xB5, 0x71, 0x38, 0x88,
		 0x88, 0x88, 0x88, 0x88, 0x23, 0x00, 0x00, 0x00, 0x6A, 0x00,
		 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x00, 0x00, 0x40, 0x80, 0x81, 0x00, 0x00, 0x00,
		 0x00}, 63, 0 },
	{ 0xEF, {0xFF, 0xFF, 0x01}, 3, 0 },
	{ MIPI_DCS_EXIT_SLEEP_MODE, {}, 0, DELAY_SLEEP_OUT_MS }, /* Sleep Out */
	{ MIPI_DCS_SET_DISPLAY_ON, {}, 0, DELAY_DISPLAY_ON_MS },  /* Display On */
};

/*
 * GPIO Helper Functions
 */
static void gpio_set_value(u32 value)
{
	u32 val = readl(GPIO_BASE + GPIO_DR_OFFSET);
	if (value)
		val |= (1 << RESET_PIN);
	else
		val &= ~(1 << RESET_PIN);
	writel(val, GPIO_BASE + GPIO_DR_OFFSET);
}

static void gpio_init(void)
{
	u32 val;

#if PINMUX_NEEDED
	/* Configure pinmux (only if needed) */
	val = readl(IOMUX_BASE + PINMUX_OFFSET);
	val &= ~(0x3 << PINMUX_SHIFT);
	val |= (PINMUX_VALUE << PINMUX_SHIFT);
	writel(val, IOMUX_BASE + PINMUX_OFFSET);
	PANEL_LOG_DEBUG("ST77031: Pinmux configured\n");
#else
	PANEL_LOG_DEBUG("ST77031: GPIO%d is dedicated, no pinmux needed\n", RESET_PIN);
#endif

	/* Set as output */
	val = readl(GPIO_BASE + GPIO_DDR_OFFSET);
	val |= (1 << RESET_PIN);
	writel(val, GPIO_BASE + GPIO_DDR_OFFSET);

	PANEL_LOG_DEBUG("ST77031: GPIO%d configured as output\n", RESET_PIN);
}

/*
 * Panel Operations Implementation
 */
static int st77031_reset(void)
{
	PANEL_LOG_DEBUG("ST77031: Executing reset sequence\n");

	gpio_init();

	/* Reset sequence: HIGH -> LOW -> HIGH (matching customer timing) */
	gpio_set_value(1);
	mdelay(100);
	gpio_set_value(0);
	mdelay(120);
	gpio_set_value(1);
	mdelay(150);

	PANEL_LOG_DEBUG("ST77031: Reset complete\n");
	return 0;
}

static ofnode st77031_panel_ofnode(void)
{
	ofnode n;

	n = x5_panel_slot_ofnode();
	if (ofnode_valid(n))
		return n;
	n = ofnode_by_compatible(ofnode_null(), "d-robotics,st77031");
	if (ofnode_valid(n))
		return n;
	return ofnode_by_compatible(ofnode_null(), "st77031");
}

static const char *st77031_timing_string_from_dt(ofnode node)
{
	const char *s;

	if (!ofnode_valid(node))
		return NULL;
	s = ofnode_read_string(node, "d-robotics,st77031-timing");
	if (s && s[0])
		return s;
	return ofnode_read_string(node, "timing-name");
}

static void st77031_check_timing_from_dt(void)
{
	static bool checked;
	const char *mode_name;
	ofnode node;

	if (checked)
		return;
	checked = true;

	node = st77031_panel_ofnode();
	mode_name = st77031_timing_string_from_dt(node);
	if (!mode_name || !mode_name[0])
		return;
	if (strcmp(mode_name, ST77031_TIMING_DT_NAME))
		PANEL_LOG_WARN("ST77031: DTS mode '%s' unknown; using built-in '%s'\n",
			       mode_name, ST77031_TIMING_DT_NAME);
}

static int st77031_init(void)
{
	int i, ret;
	u8 buf[130];

	st77031_check_timing_from_dt();

	PANEL_LOG_INFO("ST77031: Sending %zu init commands\n",
		       ARRAY_SIZE(init_sequence));

	for (i = 0; i < ARRAY_SIZE(init_sequence); i++) {
		const typeof(init_sequence[0]) *cmd = &init_sequence[i];

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
			PANEL_LOG_ERROR("ST77031: Failed cmd 0x%02x (ret=%d)\n",
			                cmd->cmd, ret);
			return ret;
		}

		if (cmd->delay_ms)
			mdelay(cmd->delay_ms);
	}

	PANEL_LOG_INFO("ST77031: Init complete\n");
	return 0;
}

static int st77031_get_timing(struct display_timing *timing)
{
	if (!timing)
		return -EINVAL;

	*timing = st77031_timing;
	return 0;
}

static int st77031_get_lanes(void)
{
	return PANEL_LANES;
}

static enum mipi_dsi_pixel_format st77031_get_format(void)
{
	return MIPI_DSI_FMT_RGB888;
}

static const char *st77031_get_name(void)
{
	return "st77031";
}

/*
 * Optional panel property x5,pwm-duty-ns — backlight duty in nanoseconds.
 * When unset, use defaults that match historical U-Boot / kernel pwm-backlight.
 */
static u32 st77031_pwm_duty_ns_default(void)
{
#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
	return 750733;
#else
	return 750000;
#endif
}

static u32 st77031_pwm_duty_ns_from_dt(void)
{
	ofnode node;
	u32 duty;

	node = st77031_panel_ofnode();
	if (ofnode_valid(node) && !ofnode_read_u32(node, "x5,pwm-duty-ns", &duty))
		return duty;

	return st77031_pwm_duty_ns_default();
}

static int st77031_get_backlight_config(struct x5_backlight_config *config)
{
	if (!config)
		return -EINVAL;

	/* PWM2 channel 1 configuration (from customer's x5-rdk.dtsi)
	 * Customer DTS: pwms = <&pwm2 1 1000000>
	 *
	 * This uses standard PWM2, not LPWM!
	 * PWM2 base address: 0x34160000 (from kernel x5.dtsi)
	 * Period: 1000000ns = 1ms = 1kHz
	 * Duty: optional x5,pwm-duty-ns on panel node (else seamless defaults).
	 */
	config->pwm_type = X5_PWM_TYPE_STD;
	config->pwm_base = PWM2_BASE;
	config->pwm_id = 2;
	config->channel = 1;
	config->period_ns = 1000000;  /* 1ms period = 1kHz */
	config->duty_ns = st77031_pwm_duty_ns_from_dt();

	/* LPWM fields not used for standard PWM */
	config->div_ratio = 0;
	config->period = 0;
	config->duty = 0;

	config->enable_gpio.gpio_num = -1;  /* No enable GPIO */
	config->enable_gpio.active_high = 1;

	PANEL_LOG_DEBUG("ST77031: Using PWM2 CH1 for backlight\n");

	return 0;
}

/*
 * Panel Operations Structure
 */
static const struct x5_panel_ops st77031_ops = {
	.reset = st77031_reset,
	.init = st77031_init,
	.get_timing = st77031_get_timing,
	.get_lanes = st77031_get_lanes,
	.get_format = st77031_get_format,
	.get_name = st77031_get_name,
	.get_backlight_config = st77031_get_backlight_config,
};

/*
 * Panel Registration
 */
const struct x5_panel panel_st77031 = {
	.compatible = "st77031",
	.name       = "ST77031 (600x1280)",
	.ops        = &st77031_ops,
};
