/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 *
 * U-Boot helper for Silicon Image SII902x (kernel drm bridge sii902x.c).
 */

#ifndef __X5_SII902X_H
#define __X5_SII902X_H

#include <linux/types.h>

struct display_timing;

int x5_sii902x_probe_of(void);
/**
 * x5_sii902x_chip_present - I2C CHIPID read only (optional reset-gpios); no globals.
 * Preconditions: enabled sil,sii9022 in DT; hdmi_detect() has enabled I2C bus clocks.
 */
int x5_sii902x_chip_present(void);
/**
 * x5_sii902x_sink_connected - INT_STATUS (0x3d) PLUGGED bit, same as kernel sii902x_detect().
 * @return 1 if plugged, 0 if not, negative errno on probe/I2C failure.
 */
int x5_sii902x_sink_connected(void);
int x5_sii902x_init_chip(void);
/** Product default when EDID is skipped or unavailable (CEA 1920x1080@60). */
void x5_sii902x_get_default_timing(struct display_timing *t);
int x5_sii902x_apply_mode(const struct display_timing *t);
int x5_sii902x_enable_output(void);
void x5_sii902x_disable_output(void);

/**
 * EDID is not implemented: kernel needs i2c_mux + DDC bus request on the bridge.
 * Returns -EOPNOTSUPP so callers use x5_sii902x_get_default_timing().
 */
ssize_t x5_sii902x_edid_read(u8 *buf, size_t len);

#endif /* __X5_SII902X_H */
