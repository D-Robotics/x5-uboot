/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 * Modified by: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * X5 Panel Interface - Callback-based Architecture
 *
 * DESIGN:
 * -------
 * Each panel implements a set of callbacks:
 * - reset()      : Hardware reset sequence
 * - init()       : Send DSI init commands
 * - get_timing() : Return display timing
 *
 * Panel selection (U-Boot):
 * - Preferred: single DT child under DSI with leading compatible
 *   X5_PANEL_SLOT_COMPAT ("d-robotics,x5-dsi-panel"); GPIO/timing read from
 *   that node.  Optional env x5_lcd_panel (e.g. jc050, wh-cm480) picks driver
 *   without changing FDT at runtime.  On the slot, use per-driver timing strings:
 *   d-robotics,jc050-timing (JC050), d-robotics,wh-cm480-timing (WH-CM480),
 *   d-robotics,st77031-timing (ST77031); legacy timing-name is a fallback if a
 *   driver-specific string is absent.
 *   WH-CM480: optional d-robotics,hb-bl-i2c on the slot (other panel drivers ignore it);
 *   if omitted but board enables /i2c@340e0000, hb_bl @0x45 is still used automatically
 *   (see panel_wh_cm480.c).
 *   When env is unset, menuconfig
 *   "Default LCD panel" applies, then optional property x5,default-panel on
 *   the slot.
 * - Legacy: DT nodes whose first compatible is jc-050hd134 / wh-cm480 / st77031
 *   still match the first loop in x5_panel_core.c when slot/env/default miss.
 */

#ifndef __X5_PANEL_H__
#define __X5_PANEL_H__

#include <dm/ofnode.h>
#include <linux/types.h>
#include <linux/errno.h>
#include "fdtdec.h"
#include "mipi_dsi.h"
#include "../backlight/x5_backlight.h"

/*
 * Panel Operations - Callback Interface
 * Each panel must implement these functions
 */
struct x5_panel_ops {
	int (*reset)(void);
	int (*init)(void);
	int (*get_timing)(struct display_timing *timing);
	int (*get_lanes)(void);
	enum mipi_dsi_pixel_format (*get_format)(void);
	const char *(*get_name)(void);
	int (*get_backlight_config)(struct x5_backlight_config *config);
};

/*
 * Panel Registration
 * Each panel file should provide this structure.
 * 'compatible' must match the DT panel node's compatible string.
 */
struct x5_panel {
	const char *compatible;
	const char *name;
	const struct x5_panel_ops *ops;
};

/** DT "slot" compatible — first string on &dsi_panel; shared by all panel drivers */
#define X5_PANEL_SLOT_COMPAT	"d-robotics,x5-dsi-panel"

/**
 * x5_panel_slot_ofnode - Panel DT node (GPIO, per-driver timing props, backlight, …).
 * Uses compatible X5_PANEL_SLOT_COMPAT; if absent (legacy DT), returns invalid.
 */
ofnode x5_panel_slot_ofnode(void);

/*
 * Core Panel API - Called by display driver
 */

/**
 * x5_panel_init_sequence - Initialize active panel
 *
 * This calls the active panel's reset() and init() callbacks.
 * Panel is selected at compile time via panel_config.h
 *
 * Returns: 0 on success, negative on error
 */
int x5_panel_init_sequence(void);

/**
 * x5_panel_get_timing - Get active panel timing
 * @timing: Pointer to timing structure to fill
 *
 * Returns: 0 on success, negative on error
 */
int x5_panel_get_timing(struct display_timing *timing);

/**
 * x5_panel_get_lanes - Get active panel DSI lanes
 *
 * Returns: Number of lanes
 */
int x5_panel_get_lanes(void);

/**
 * x5_panel_get_format - Get active panel pixel format
 *
 * Returns: Pixel format
 */
enum mipi_dsi_pixel_format x5_panel_get_format(void);

/**
 * x5_panel_get_name - Get active panel name
 *
 * Returns: Panel name string
 */
const char *x5_panel_get_name(void);

/**
 * x5_panel_get_backlight_config - Get active panel backlight config
 * @config: Pointer to backlight config structure to fill
 *
 * Returns: 0 on success, negative on error
 */
int x5_panel_get_backlight_config(struct x5_backlight_config *config);

#endif /* __X5_PANEL_H__ */
