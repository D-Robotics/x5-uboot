/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * Lontium LT8618 BT1120/YUV parallel -> HDMI (subset of kernel lontium-lt8618.c).
 */

#ifndef __X5_LT8618_H__
#define __X5_LT8618_H__

#include <dm/ofnode.h>
#include <fdtdec.h>
#include <linux/types.h>

int x5_lt8618_probe_of(void);
/**
 * x5_lt8618_chip_present - I2C signature only; does not touch global @c g_chip / @c g_probed.
 * Used to pick LT8618 vs SII902x when both drivers are built in.
 */
int x5_lt8618_chip_present(void);
/**
 * x5_lt8618_sink_connected - HPD / link DC after reset + I2C enable (0x80ee), like EDID path.
 * @return 1 if a sink appears connected, 0 if not, negative errno on probe/I2C failure.
 */
int x5_lt8618_sink_connected(void);
int x5_lt8618_init_chip(void);
/** Product default when EDID is skipped or unavailable (CEA 1920x1080@60). */
void x5_lt8618_get_default_timing(struct display_timing *t);
int x5_lt8618_apply_mode(const struct display_timing *t);
int x5_lt8618_enable_phy(void);
void x5_lt8618_disable_tx(void);

/* Clock detection for BT1120 input timing validation */
unsigned long x5_lt8618_clock_detect(void);
void x5_lt8618_dump_regs(void);
void x5_lt8618_log_parallel_input_summary(const char *tag);

/* Test pattern (color bars) for debugging */
int x5_lt8618_init_for_pattern(void);
int x5_lt8618_enable_pattern(const struct display_timing *t);

/*
 * Build struct display_timing from one line of `modetest -M vs-drm -a -c` (modes list).
 * Columns: hdisp hss hse htot vdisp vss vse vtot clock_khz [flags: phsync/nhsync ...]
 * @clock_khz: value before "flags:" (modetest prints pixel clock in kHz).
 * @phsync: true if mode lists "phsync", false if "nhsync".
 * @pvsync: true if "pvsync", false if "nvsync".
 */
void x5_lt8618_timing_from_modetest_crtc(struct display_timing *t,
					 u32 clock_khz,
					 u32 hdisp, u32 hss, u32 hse, u32 htot,
					 u32 vdisp, u32 vss, u32 vse, u32 vtot,
					 bool phsync, bool pvsync);

/*
 * Read EDID over LT8618 DDC FIFO (kernel lontium-lt8618.c lt8618_ddc_fifo_fetch).
 * Requires HPD. Enables I2C path (0x80ee) like kernel connector_detect.
 * Returns 128, or 256 if extension block valid, or negative errno.
 */
ssize_t x5_lt8618_edid_read(u8 *buf, size_t len);

#endif /* __X5_LT8618_H__ */
