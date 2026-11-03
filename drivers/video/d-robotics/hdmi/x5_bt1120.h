/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * VeriSilicon BT1120 online-mode programming for X5 (from kernel vs_bt1120.c).
 */

#ifndef __X5_BT1120_H__
#define __X5_BT1120_H__

#include <asm/io.h>
#include <fdtdec.h>

struct display_timing;

#define X5_BT1120_REG_BASE_DEFAULT 0x3e010000UL

void x5_bt1120_disable(void __iomem *base);
int x5_bt1120_set_online_mode(void __iomem *base, const struct display_timing *t);
void x5_bt1120_log_timing_regs(void __iomem *base, const char *where,
			       const struct display_timing *t);
void x5_bt1120_log_irq_status(void __iomem *base, const char *where);
void x5_bt1120_dump_all_regs(void __iomem *bt1120_base);

/* Online-capture health probe + re-arm (see x5_bt1120.c). */
u32 x5_bt1120_irq_status(void __iomem *base);
void x5_bt1120_irq_clear(void __iomem *base, u32 mask);
bool x5_bt1120_capture_is_bad(void __iomem *base);
void x5_bt1120_capture_rearm(void __iomem *base);
u32 x5_bt1120_capture_rearm_vblank(void __iomem *base, u32 vactive_lines);

#endif /* __X5_BT1120_H__ */
