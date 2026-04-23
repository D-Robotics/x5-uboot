/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 */

#ifndef __X5_SYSCON_H__
#define __X5_SYSCON_H__

#include <fdtdec.h>

/**
 * @brief Enable SYSCON routing from DC8000 to the DSI controller.
 * @param timing Active mode timing; must not be NULL (reserved for future polarity).
 * @return 0 on success, or negative errno (e.g. -EINVAL if @a timing is NULL).
 */
int x5_syscon_bridge_init(struct display_timing *timing);

/**
 * @brief Disable DC8000-to-DSI routing; restore SYSCON to DC2CSI-only (IDI path).
 * @return None.
 */
void x5_syscon_bridge_disable(void);

/**
 * @brief Enable SYSCON routing from DC8000 DPI to BT1120 (HDMI pipeline).
 * @param timing Active mode timing; must not be NULL (reserved for future use).
 * @return 0 on success, or negative errno (e.g. -EINVAL if @a timing is NULL).
 */
int x5_syscon_bt1120_init(struct display_timing *timing);

/**
 * @brief Clear BT1120 pin mux control registers (PIN_MUX0/PIN_MUX1) to zero.
 * @return None.
 */
void x5_syscon_bt1120_clear_pinmux(void);

/**
 * @brief Tear down BT1120 path: clear pin mux and drop DC2BT1120_EN.
 * @return None.
 */
void x5_syscon_bt1120_disable(void);

/**
 * @brief Log one line of display SYSCON/BT1120-related registers for debug bring-up.
 * @param tag Short label printed in the log line (may be NULL; shown as "?").
 * @return None.
 */
void x5_disp_iomux_dump_one_line(const char *tag);

#endif /* __X5_SYSCON_H__ */
