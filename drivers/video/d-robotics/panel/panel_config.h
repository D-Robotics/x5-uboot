/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 * Modified by: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * Panel registration table
 *
 * All compiled-in panels are listed here.  The core selects the
 * active panel at runtime by matching the DT "compatible" string.
 */

#ifndef __PANEL_CONFIG_H__
#define __PANEL_CONFIG_H__

#include "x5_panel.h"

#ifdef CONFIG_X5_PANEL_JC050HD134
extern const struct x5_panel panel_jc050hd134;
#endif

#ifdef CONFIG_X5_PANEL_WH_CM480
extern const struct x5_panel panel_wh_cm480;
#endif

#ifdef CONFIG_X5_PANEL_ST77031
extern const struct x5_panel panel_st77031;
#endif

/*
 * panel_table[] — built by x5_panel_core.c from the conditionally
 * compiled entries above.  Terminated by a NULL-compatible sentinel.
 */

#endif /* __PANEL_CONFIG_H__ */
