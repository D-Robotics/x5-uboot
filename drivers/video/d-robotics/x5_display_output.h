/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * X5 Display Output Interface Abstraction
 *
 * This header defines a generic output interface for the X5 display
 * subsystem.  DC8000Nano is the shared display controller; the actual
 * output path (MIPI-DSI panel or BT1120-to-HDMI) is selected at
 * runtime by scanning Device-Tree nodes.
 *
 * Hardware paths (from X5 Display Subsystem Spec V1.5):
 *
 *   DDR -> DC8000Nano -> SYSCON MUX ─┬─ dc2dsi_en=1  -> DSI-TX -> DPHY -> Panel
 *                                     └─ dc2bt1120_en=1 -> BT1120 (CSC) -> Ext HDMI TX
 *
 * Each output type implements this ops table.  x5_display.c calls
 * through the ops without knowing which output is active.
 */

#ifndef __X5_DISPLAY_OUTPUT_H__
#define __X5_DISPLAY_OUTPUT_H__

#include <linux/types.h>
#include <linux/errno.h>
#include <fdtdec.h>

struct x5_backlight_config;

enum x5_output_type {
	X5_OUTPUT_DSI = 0,
	X5_OUTPUT_BT1120_HDMI,
};

/**
 * struct x5_display_output_ops - abstraction for one display output path
 *
 * All callbacks are optional; a NULL pointer is treated as "not
 * supported / no-op" by the caller in x5_display.c.
 */
struct x5_display_output_ops {
	const char *name;
	enum x5_output_type type;

	/**
	 * detect - check Device-Tree for this output
	 *
	 * DSI:  look for an enabled mipi_dsi / dsi_panel node
	 * HDMI: look for an enabled BT1120 / HDMI TX node
	 *
	 * Return: 0 if this output is present and enabled, negative errno otherwise
	 */
	int (*detect)(void);

	/**
	 * get_timing - retrieve display timing
	 *
	 * DSI:  delegates to panel driver (x5_panel_get_timing)
	 * HDMI: EDID when available; else active bridge default (CEA 1080p)
	 */
	int (*get_timing)(struct display_timing *timing);

	/**
	 * clk_init - enable output-specific clocks
	 *
	 * DSI:  CLK_EN_DISP_DSI_PCLK, CLK_EN_DPHY_CFG, DSI TXESC clock gen
	 * HDMI: CLK_EN_BT1120_PCLK, CLK_EN_BT1120_ACLK, BT1120 pixel/axi clock gen
	 *
	 * @pixel_clock: requested pixel clock rate in Hz (for clock tree calc)
	 */
	int (*clk_init)(unsigned long pixel_clock);

	/**
	 * reset - assert then de-assert output-specific resets
	 *
	 * DSI:  RST_DSI_TX
	 * HDMI: RST_BT1120
	 */
	int (*reset)(void);

	/**
	 * bridge_init - configure SYSCON routing for this output
	 *
	 * DSI:  set dc2dsi_en  (DISP_DC2CSI_DSI_EN, offset 0x10, bit 1)
	 * HDMI: crtc-bt1120 clears dc2bt1120_en; pin mux + pads (see
	 * vs_x5_syscon_bridge bt1120_data[] for crtc-bt1120)
	 *
	 * Note: spec DISP_SUB_002 allows both bits to be set simultaneously
	 * for dual-screen scenarios.
	 */
	int (*bridge_init)(struct display_timing *timing);

	/**
	 * hw_init - prepare output transmitter (Step 6: BT1120/DSI)
	 *
	 * Called during hw_init phase, only does preparation
	 * (get register base, disable module). Actual init is in enable().
	 *
	 * DSI:  get DSI base, disable
	 * HDMI: get BT1120 base, disable
	 */
	int (*hw_init)(struct display_timing *timing);

	/**
	 * tx_init - transmitter early init (called BEFORE DC8000 enable)
	 *
	 * Mirrors kernel bridge_mode_set(): configure the external TX chip
	 * so it is ready to lock onto the pixel clock once DC8000 starts.
	 *
	 * HDMI: LT8618 chip probe + sw_init + PLL/output config
	 * DSI:  NULL (not needed, panel init happens in enable)
	 */
	int (*tx_init)(struct display_timing *timing);

	/**
	 * enable - start video stream (called AFTER DC8000 enable)
	 *
	 * Mirrors kernel bridge_enable(): DC8000 is already producing
	 * pixels, BT1120/DSI host can receive data.
	 *
	 * HDMI: BT1120 online mode -> LT8618 PLL lock + AFE + phase calib
	 * DSI:  panel init sequence -> DSI host video mode enable
	 */
	int (*enable)(void);

	/**
	 * disable - stop the video stream and power-down the output
	 */
	int (*disable)(void);

	/**
	 * clk_disable - gate output-specific clocks (reverse of clk_init)
	 */
	void (*clk_disable)(void);

	/**
	 * get_backlight_config - fill backlight configuration
	 *
	 * HDMI outputs should return -ENODEV (no backlight).
	 */
	int (*get_backlight_config)(struct x5_backlight_config *config);

	/**
	 * get_info - return a human-readable description for log messages
	 *
	 * Example: "MIPI DSI (4 lanes, RGB888)" or "BT1120 -> HDMI (1080p@60)"
	 */
	const char *(*get_info)(void);
};

/* --- Output implementations (conditionally compiled) --- */

extern const struct x5_display_output_ops x5_dsi_output_ops;

#ifdef CONFIG_VIDEO_X5_HDMI
extern const struct x5_display_output_ops x5_hdmi_output_ops;

/**
 * x5_hdmi_pattern_boot_requested - Use LT8618 internal pattern instead of DC/BT1120
 *
 * Non-zero if CONFIG_VIDEO_X5_HDMI_PATTERN_BOOT (override off: env x5_hdmi_pattern=0),
 * or env x5_hdmi_pattern is 1/y/yes (when Kconfig option is disabled).
 */
int x5_hdmi_pattern_boot_requested(void);

/**
 * Run LT8618 pattern init + enable using @t (same timing as display pipeline;
 * must not be NULL).
 */
int x5_hdmi_enable_pattern_boot(const struct display_timing *t);
#endif

#endif /* __X5_DISPLAY_OUTPUT_H__ */
