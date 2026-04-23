/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 *
 * X5 Display Driver Header
 */

#ifndef __X5_DISPLAY_H__
#define __X5_DISPLAY_H__

#include <linux/types.h>

/*
 * Board late_init entry: stdio routing + optional boot UI (logo vs framebuffer console).
 * See CONFIG_VIDEO_X5_BOOT_UI and env x5_boot_ui (unset → set to logo; console → FB console).
 */
#ifdef CONFIG_X5_SEAMLESS_DISPLAY
void x5_display_boot_board_late(void);
#endif

#ifdef CONFIG_VIDEO_X5_BOOT_UI
bool x5_boot_ui_is_fbcon(void);
#endif

#ifdef CONFIG_VIDEO_X5_HDMI
/*
 * x5_hdmi_edid: on = read EDID (bridge default on failure); off = skip EDID.
 * Defaults to on in x5_display_boot_board_late() when unset (see x5_hdmi_fb_lcd).
 */
bool x5_hdmi_edid_enabled(void);
#endif

/* Initialize display hardware (call after probe) */
int x5_display_hw_init(void);

/* Enable video output */
int x5_display_enable(void);

/* Disable video output */
int x5_display_disable(void);

/* Print display status */
void x5_display_status(void);

/* Fill framebuffer with solid color (ARGB32) */
int x5_display_fill_color(u32 color);

/*
 * Show logo (tries partition first, then built-in)
 * x, y: coordinates (use BMP_ALIGN_CENTER for centering)
 */
int x5_display_show_logo_auto(int x, int y, bool clear);

#ifdef CONFIG_VIDEO_LOGO
/* Show built-in logo (0=center, 1=TL, 2=TR, 3=BL, 4=BR) */
int x5_display_show_logo(int position, bool clear);

/* Show built-in logo at custom coordinates */
int x5_display_show_logo_xy(int x, int y, bool clear);
#endif

#endif /* __X5_DISPLAY_H__ */
