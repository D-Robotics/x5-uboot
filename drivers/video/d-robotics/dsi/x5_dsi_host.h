/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 *
 * X5 DSI Host Driver Header
 */

#ifndef __X5_DSI_HOST_H__
#define __X5_DSI_HOST_H__

#include <fdtdec.h>

/* DSI pixel formats (from mipi_dsi.h) */
#ifndef MIPI_DSI_FMT_RGB888
#define MIPI_DSI_FMT_RGB888 0
#define MIPI_DSI_FMT_RGB666 1
#define MIPI_DSI_FMT_RGB666_PACKED 2
#define MIPI_DSI_FMT_RGB565 3
#endif

/**
 * x5_dsi_host_init - Initialize DSI Host
 *
 * @timing: Display timing parameters
 * @lanes: Number of data lanes (1-4), 0 for default (4)
 * @format: Pixel format (MIPI_DSI_FMT_xxx), 0 for default (RGB888)
 *
 * Returns: 0 on success, negative error code on failure
 */
int x5_dsi_host_init(struct display_timing *timing, int lanes, int format);

/**
 * x5_dsi_host_enable - Enable video mode output
 *
 * Call this after panel initialization is complete
 */
void x5_dsi_host_enable(void);

/**
 * x5_dsi_host_disable - Disable DSI Host
 */
void x5_dsi_host_disable(void);

/**
 * x5_dsi_dcs_write_0 - Send DCS command with no parameter
 *
 * @cmd: DCS command
 *
 * Returns: 0 on success, negative error code on failure
 */
int x5_dsi_dcs_write_0(u8 cmd);

/**
 * x5_dsi_dcs_write_1 - Send DCS command with 1 parameter
 *
 * @cmd: DCS command
 * @param: Parameter value
 *
 * Returns: 0 on success, negative error code on failure
 */
int x5_dsi_dcs_write_1(u8 cmd, u8 param);

/**
 * x5_dsi_dcs_write_buffer - Send DCS long write command
 *
 * @data: Data buffer (first byte is command)
 * @len: Data length
 *
 * Returns: 0 on success, negative error code on failure
 */
int x5_dsi_dcs_write_buffer(const u8 *data, size_t len);

/* Common DCS commands */
#define MIPI_DCS_NOP 0x00
#define MIPI_DCS_SOFT_RESET 0x01
#define MIPI_DCS_GET_DISPLAY_ID 0x04
#define MIPI_DCS_GET_RED_CHANNEL 0x06
#define MIPI_DCS_GET_GREEN_CHANNEL 0x07
#define MIPI_DCS_GET_BLUE_CHANNEL 0x08
#define MIPI_DCS_GET_DISPLAY_STATUS 0x09
#define MIPI_DCS_GET_POWER_MODE 0x0A
#define MIPI_DCS_GET_ADDRESS_MODE 0x0B
#define MIPI_DCS_GET_PIXEL_FORMAT 0x0C
#define MIPI_DCS_GET_DISPLAY_MODE 0x0D
#define MIPI_DCS_GET_SIGNAL_MODE 0x0E
#define MIPI_DCS_GET_DIAGNOSTIC_RESULT 0x0F
#define MIPI_DCS_ENTER_SLEEP_MODE 0x10
#define MIPI_DCS_EXIT_SLEEP_MODE 0x11
#define MIPI_DCS_ENTER_PARTIAL_MODE 0x12
#define MIPI_DCS_ENTER_NORMAL_MODE 0x13
#define MIPI_DCS_EXIT_INVERT_MODE 0x20
#define MIPI_DCS_ENTER_INVERT_MODE 0x21
#define MIPI_DCS_SET_GAMMA_CURVE 0x26
#define MIPI_DCS_SET_DISPLAY_OFF 0x28
#define MIPI_DCS_SET_DISPLAY_ON 0x29
#define MIPI_DCS_SET_COLUMN_ADDRESS 0x2A
#define MIPI_DCS_SET_PAGE_ADDRESS 0x2B
#define MIPI_DCS_WRITE_MEMORY_START 0x2C
#define MIPI_DCS_WRITE_LUT 0x2D
#define MIPI_DCS_READ_MEMORY_START 0x2E
#define MIPI_DCS_SET_PARTIAL_ROWS 0x30
#define MIPI_DCS_SET_PARTIAL_COLUMNS 0x31
#define MIPI_DCS_SET_SCROLL_AREA 0x33
#define MIPI_DCS_SET_TEAR_OFF 0x34
#define MIPI_DCS_SET_TEAR_ON 0x35
#define MIPI_DCS_SET_ADDRESS_MODE 0x36
#define MIPI_DCS_SET_SCROLL_START 0x37
#define MIPI_DCS_EXIT_IDLE_MODE 0x38
#define MIPI_DCS_ENTER_IDLE_MODE 0x39
#define MIPI_DCS_SET_PIXEL_FORMAT 0x3A
#define MIPI_DCS_WRITE_MEMORY_CONTINUE 0x3C
#define MIPI_DCS_READ_MEMORY_CONTINUE 0x3E
#define MIPI_DCS_SET_TEAR_SCANLINE 0x44
#define MIPI_DCS_GET_SCANLINE 0x45
#define MIPI_DCS_SET_DISPLAY_BRIGHTNESS 0x51
#define MIPI_DCS_GET_DISPLAY_BRIGHTNESS 0x52
#define MIPI_DCS_WRITE_CONTROL_DISPLAY 0x53
#define MIPI_DCS_GET_CONTROL_DISPLAY 0x54
#define MIPI_DCS_WRITE_POWER_SAVE 0x55
#define MIPI_DCS_GET_POWER_SAVE 0x56
#define MIPI_DCS_SET_CABC_MIN_BRIGHTNESS 0x5E
#define MIPI_DCS_GET_CABC_MIN_BRIGHTNESS 0x5F
#define MIPI_DCS_READ_DDB_START 0xA1
#define MIPI_DCS_READ_DDB_CONTINUE 0xA8

#endif /* __X5_DSI_HOST_H__ */
