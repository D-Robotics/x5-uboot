/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 */

#ifndef __X5_DC8000_H__
#define __X5_DC8000_H__

#include <linux/types.h>
#include <fdtdec.h>

/* Framebuffer format definitions (values from register spec) */
#define DC8000_FORMAT_ARGB8888 0x4
#define DC8000_FORMAT_RGB565 0x3

struct dc8000_config
{
	u32 fb_addr;
	u32 fb_width;
	u32 fb_height;
	u32 fb_format;
	u32 fb_stride;
};

/* DC8000 API */
int x5_dc8000_init(struct display_timing *timing);
void x5_dc8000_setup_output(void);
int x5_dc8000_set_framebuffer(u32 fb_addr, u32 format);
void x5_dc8000_enable(void);
void x5_dc8000_disable(void);
struct dc8000_config *x5_dc8000_get_config(void);

#endif /* __X5_DC8000_H__ */
