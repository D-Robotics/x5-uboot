/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 * Modified by: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * X5 Display Subsystem Logging Framework
 *
 * Compile-time per-module log level control.
 * Each module has an independent level that defaults to X5_LOG_LEVEL_DEFAULT.
 *
 * To enable debug for a single module, add before #include or in CFLAGS:
 *   #define X5_LOG_LEVEL_PANEL  X5_LVL_DEBUG
 *   #define X5_LOG_LEVEL_BT1120 X5_LVL_DEBUG
 *   #define X5_LOG_LEVEL_LT8618 X5_LVL_DEBUG   (LT8618 full reg dump: x5_lt8618_dump_regs)
 *   #define X5_LOG_LEVEL_SYSCON X5_LVL_DEBUG
 *
 * [X5_DISP] — optional cross-layer trace (kernel pr_info("[X5_DISP] ...") parity).
 *   Default follows X5_LOG_LEVEL_DEFAULT. Mute: #define X5_LOG_LEVEL_X5_DISP X5_LVL_NONE
 *
 * To enable debug for all modules at once:
 *   #define X5_LOG_LEVEL_DEFAULT  X5_LVL_DEBUG
 *
 * Levels:
 *   X5_LVL_NONE  (0) - No output
 *   X5_LVL_ERROR (1) - Only errors
 *   X5_LVL_WARN  (2) - Errors + warnings (default)
 *   X5_LVL_INFO  (3) - Errors + warnings + info
 *   X5_LVL_DEBUG (4) - All messages
 */

#ifndef __X5_DISPLAY_LOG_H__
#define __X5_DISPLAY_LOG_H__

/* Log levels */
#define X5_LVL_NONE	0
#define X5_LVL_ERROR	1
#define X5_LVL_WARN	2
#define X5_LVL_INFO	3
#define X5_LVL_DEBUG	4

/* Per-module compile-time level (override before #include) */
#ifndef X5_LOG_LEVEL_DEFAULT
#define X5_LOG_LEVEL_DEFAULT	X5_LVL_WARN
#endif

/*
 * Optional per-module verbosity (compile-time)
 * --------------------------------------------
 * Every symbol X5_LOG_LEVEL_<MODULE> defaults to X5_LOG_LEVEL_DEFAULT (above)
 * unless you define it earlier (before #include "x5_display_log.h") or pass
 * -DX5_LOG_LEVEL_<MODULE>=<level> on the compiler command line.
 *
 * Levels: X5_LVL_NONE | ERROR | WARN | INFO | DEBUG (numeric 0..4).
 *
 * MIPI + seamless bring-up: remove the leading "//" on any line in the block
 * below to pin that module to INFO for the entire build (same effect as -D).
 *
 * Whole-subsystem flood: #define X5_LOG_LEVEL_DEFAULT X5_LVL_DEBUG before include.
 */
// #define X5_LOG_LEVEL_CRM               X5_LVL_INFO
// #define X5_LOG_LEVEL_DSI               X5_LVL_INFO
// #define X5_LOG_LEVEL_DPHY              X5_LVL_INFO
// #define X5_LOG_LEVEL_DC8000            X5_LVL_INFO
// #define X5_LOG_LEVEL_SEAMLESS_DISPLAY  X5_LVL_INFO
// #define X5_LOG_LEVEL_PANEL             X5_LVL_INFO
// #define X5_LOG_LEVEL_DISPLAY           X5_LVL_INFO
// #define X5_LOG_LEVEL_LT8618            X5_LVL_INFO
// #define X5_LOG_LEVEL_BT1120            X5_LVL_INFO
// #define X5_LOG_LEVEL_SYSCON            X5_LVL_INFO
// #define X5_LOG_LEVEL_BL                X5_LVL_INFO
// #define X5_LOG_LEVEL_DISPLAY_DUMP      X5_LVL_INFO
// #define X5_LOG_LEVEL_X5_DISP           X5_LVL_INFO


#ifndef X5_LOG_LEVEL_CRM
#define X5_LOG_LEVEL_CRM	X5_LOG_LEVEL_DEFAULT
#endif

#ifndef X5_LOG_LEVEL_DC8000
#define X5_LOG_LEVEL_DC8000	X5_LOG_LEVEL_DEFAULT
#endif

#ifndef X5_LOG_LEVEL_DSI
#define X5_LOG_LEVEL_DSI	X5_LOG_LEVEL_DEFAULT
#endif

#ifndef X5_LOG_LEVEL_DPHY
#define X5_LOG_LEVEL_DPHY	X5_LOG_LEVEL_DEFAULT
#endif

#ifndef X5_LOG_LEVEL_SYSCON
#define X5_LOG_LEVEL_SYSCON	X5_LOG_LEVEL_DEFAULT
#endif

#ifndef X5_LOG_LEVEL_X5_DISP
#define X5_LOG_LEVEL_X5_DISP	X5_LOG_LEVEL_DEFAULT
#endif

#ifndef X5_LOG_LEVEL_BT1120
#define X5_LOG_LEVEL_BT1120	X5_LOG_LEVEL_DEFAULT
#endif

#ifndef X5_LOG_LEVEL_LT8618
#define X5_LOG_LEVEL_LT8618	X5_LOG_LEVEL_DEFAULT
#endif

#ifndef X5_LOG_LEVEL_PANEL
#define X5_LOG_LEVEL_PANEL	X5_LOG_LEVEL_DEFAULT
#endif

#ifndef X5_LOG_LEVEL_BL
#define X5_LOG_LEVEL_BL		X5_LOG_LEVEL_DEFAULT
#endif

#ifndef X5_LOG_LEVEL_DISPLAY
#define X5_LOG_LEVEL_DISPLAY	X5_LOG_LEVEL_DEFAULT
#endif

/* Display dump log , used for dump display configuration in x5_display.c, default level is INFO*/
#ifndef X5_LOG_LEVEL_DISPLAY_DUMP
#define X5_LOG_LEVEL_DISPLAY_DUMP	X5_LOG_LEVEL_DEFAULT
#endif

#ifndef X5_LOG_LEVEL_SEAMLESS_DISPLAY
#define X5_LOG_LEVEL_SEAMLESS_DISPLAY	X5_LOG_LEVEL_DEFAULT
#endif

/* Conditional printf — compiler eliminates dead branches at -O1+ */
#define _X5_LOG(mod_level, threshold, fmt, ...) do {	\
	if ((mod_level) >= (threshold))			\
		printf(fmt, ##__VA_ARGS__);		\
} while (0)

/* CRM (Clock and Reset Manager) */
#define CRM_LOG_ERROR(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_CRM, X5_LVL_ERROR, "CRM: ERROR - " fmt, ##__VA_ARGS__)
#define CRM_LOG_WARN(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_CRM, X5_LVL_WARN, "CRM: WARNING - " fmt, ##__VA_ARGS__)
#define CRM_LOG_INFO(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_CRM, X5_LVL_INFO, "CRM: " fmt, ##__VA_ARGS__)
#define CRM_LOG_DEBUG(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_CRM, X5_LVL_DEBUG, "CRM: " fmt, ##__VA_ARGS__)

/* DC8000 (Display Controller) */
#define DC8000_LOG_ERROR(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DC8000, X5_LVL_ERROR, "DC8000: ERROR - " fmt, ##__VA_ARGS__)
#define DC8000_LOG_WARN(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DC8000, X5_LVL_WARN, "DC8000: WARNING - " fmt, ##__VA_ARGS__)
#define DC8000_LOG_INFO(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DC8000, X5_LVL_INFO, "DC8000: " fmt, ##__VA_ARGS__)
#define DC8000_LOG_DEBUG(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DC8000, X5_LVL_DEBUG, "DC8000: " fmt, ##__VA_ARGS__)

/* DSI Host */
#define DSI_LOG_ERROR(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DSI, X5_LVL_ERROR, "DSI: ERROR - " fmt, ##__VA_ARGS__)
#define DSI_LOG_WARN(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DSI, X5_LVL_WARN, "DSI: WARNING - " fmt, ##__VA_ARGS__)
#define DSI_LOG_INFO(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DSI, X5_LVL_INFO, "DSI: " fmt, ##__VA_ARGS__)
#define DSI_LOG_DEBUG(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DSI, X5_LVL_DEBUG, "DSI: " fmt, ##__VA_ARGS__)

/* DPHY (D-PHY) */
#define DPHY_LOG_ERROR(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DPHY, X5_LVL_ERROR, "DPHY: ERROR - " fmt, ##__VA_ARGS__)
#define DPHY_LOG_WARN(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DPHY, X5_LVL_WARN, "DPHY: WARNING - " fmt, ##__VA_ARGS__)
#define DPHY_LOG_INFO(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DPHY, X5_LVL_INFO, "DPHY: " fmt, ##__VA_ARGS__)
#define DPHY_LOG_DEBUG(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DPHY, X5_LVL_DEBUG, "DPHY: " fmt, ##__VA_ARGS__)

/* SYSCON (System Control Bridge) */
#define SYSCON_LOG_ERROR(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_SYSCON, X5_LVL_ERROR, "SYSCON: ERROR - " fmt, ##__VA_ARGS__)
#define SYSCON_LOG_WARN(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_SYSCON, X5_LVL_WARN, "SYSCON: WARNING - " fmt, ##__VA_ARGS__)
#define SYSCON_LOG_INFO(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_SYSCON, X5_LVL_INFO, "SYSCON: " fmt, ##__VA_ARGS__)
#define SYSCON_LOG_DEBUG(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_SYSCON, X5_LVL_DEBUG, "SYSCON: " fmt, ##__VA_ARGS__)

/* BT1120 (VeriSilicon parallel output / HDMI bridge input) */
#define BT1120_LOG_ERROR(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_BT1120, X5_LVL_ERROR, "BT1120: ERROR - " fmt, ##__VA_ARGS__)
#define BT1120_LOG_WARN(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_BT1120, X5_LVL_WARN, "BT1120: WARNING - " fmt, ##__VA_ARGS__)
#define BT1120_LOG_INFO(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_BT1120, X5_LVL_INFO, "BT1120: " fmt, ##__VA_ARGS__)
#define BT1120_LOG_DEBUG(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_BT1120, X5_LVL_DEBUG, "BT1120: " fmt, ##__VA_ARGS__)

/* LT8618 (BT1120 -> HDMI bridge, x5_lt8618.c) */
#define LT8618_LOG_ERROR(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_LT8618, X5_LVL_ERROR, "LT8618: ERROR - " fmt, ##__VA_ARGS__)
#define LT8618_LOG_WARN(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_LT8618, X5_LVL_WARN, "LT8618: WARNING - " fmt, ##__VA_ARGS__)
#define LT8618_LOG_INFO(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_LT8618, X5_LVL_INFO, "LT8618: " fmt, ##__VA_ARGS__)
#define LT8618_LOG_DEBUG(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_LT8618, X5_LVL_DEBUG, "LT8618: " fmt, ##__VA_ARGS__)

/* Panel */
#define PANEL_LOG_ERROR(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_PANEL, X5_LVL_ERROR, "Panel: ERROR - " fmt, ##__VA_ARGS__)
#define PANEL_LOG_WARN(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_PANEL, X5_LVL_WARN, "Panel: WARNING - " fmt, ##__VA_ARGS__)
#define PANEL_LOG_INFO(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_PANEL, X5_LVL_INFO, "Panel: " fmt, ##__VA_ARGS__)
#define PANEL_LOG_DEBUG(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_PANEL, X5_LVL_DEBUG, "Panel: " fmt, ##__VA_ARGS__)

/* Backlight */
#define BL_LOG_ERROR(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_BL, X5_LVL_ERROR, "Backlight: ERROR - " fmt, ##__VA_ARGS__)
#define BL_LOG_WARN(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_BL, X5_LVL_WARN, "Backlight: WARNING - " fmt, ##__VA_ARGS__)
#define BL_LOG_INFO(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_BL, X5_LVL_INFO, "Backlight: " fmt, ##__VA_ARGS__)
#define BL_LOG_DEBUG(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_BL, X5_LVL_DEBUG, "Backlight: " fmt, ##__VA_ARGS__)

/* Display Framework */
#define DISP_LOG_ERROR(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DISPLAY, X5_LVL_ERROR, "X5 Display: ERROR - " fmt, ##__VA_ARGS__)
#define DISP_LOG_WARN(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DISPLAY, X5_LVL_WARN, "X5 Display: WARNING - " fmt, ##__VA_ARGS__)
#define DISP_LOG_INFO(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DISPLAY, X5_LVL_INFO, "X5 Display: " fmt, ##__VA_ARGS__)
#define DISP_LOG_DEBUG(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DISPLAY, X5_LVL_DEBUG, "X5 Display: " fmt, ##__VA_ARGS__)

/* [X5_DISP] — kernel dmesg parity (vs_bt1120 / bt1120_bridge / LT8618 / syscon summary) */
#define X5_DISP_LOG_ERROR(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_X5_DISP, X5_LVL_ERROR, "[X5_DISP] ERROR: " fmt, ##__VA_ARGS__)
#define X5_DISP_LOG_WARN(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_X5_DISP, X5_LVL_WARN, "[X5_DISP] WARNING: " fmt, ##__VA_ARGS__)
#define X5_DISP_LOG_INFO(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_X5_DISP, X5_LVL_INFO, "[X5_DISP] " fmt, ##__VA_ARGS__)
#define X5_DISP_LOG_DEBUG(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_X5_DISP, X5_LVL_DEBUG, "[X5_DISP] " fmt, ##__VA_ARGS__)

/* Display Framework Dump No fmt param*/
#define DISP_DUMP_LOG_ERROR(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DISPLAY_DUMP, X5_LVL_ERROR, fmt, ##__VA_ARGS__)
#define DISP_DUMP_LOG_WARN(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DISPLAY_DUMP, X5_LVL_WARN, fmt, ##__VA_ARGS__)
#define DISP_DUMP_LOG_INFO(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DISPLAY_DUMP, X5_LVL_INFO, fmt, ##__VA_ARGS__)
#define DISP_DUMP_LOG_DEBUG(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_DISPLAY_DUMP, X5_LVL_DEBUG, fmt, ##__VA_ARGS__)

/* Seamless Display */
#define SEAMLESS_LOG_ERROR(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_SEAMLESS_DISPLAY, X5_LVL_ERROR, "seamless_display: ERROR - " fmt, ##__VA_ARGS__)
#define SEAMLESS_LOG_WARN(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_SEAMLESS_DISPLAY, X5_LVL_WARN, "seamless_display: WARNING - " fmt, ##__VA_ARGS__)
#define SEAMLESS_LOG_WARNING SEAMLESS_LOG_WARN
#define SEAMLESS_LOG_INFO(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_SEAMLESS_DISPLAY, X5_LVL_INFO, "seamless_display: " fmt, ##__VA_ARGS__)
#define SEAMLESS_LOG_DEBUG(fmt, ...) \
	_X5_LOG(X5_LOG_LEVEL_SEAMLESS_DISPLAY, X5_LVL_DEBUG, "seamless_display: " fmt, ##__VA_ARGS__)
#endif /* __X5_DISPLAY_LOG_H__ */
