// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * X5 DSI Output Implementation
 *
 * Wraps the existing DSI-specific subsystems (panel, DSI host, SYSCON
 * bridge, backlight) behind the generic x5_display_output_ops interface
 * so that x5_display.c can drive DSI panels without hard-coding the
 * output path.
 */

#include <common.h>
#include <dm/ofnode.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <fdtdec.h>

#include "x5_display_output.h"
#include "panel/x5_panel.h"
#include "dsi/x5_dsi_host.h"
#include "bridge/x5_syscon.h"
#include "backlight/x5_backlight.h"
#include "clock/x5_crm.h"
#include <hb_display_log.h>

/* ------------------------------------------------------------------ */
/* detect: check DT for an enabled MIPI-DSI host node                 */
/* ------------------------------------------------------------------ */
static int dsi_detect(void)
{
	ofnode node;

	node = ofnode_by_compatible(ofnode_null(), "verisilicon,dw-mipi-dsi");
	if (ofnode_valid(node) && ofnode_is_enabled(node))
		return 0;

	return -ENODEV;
}

/* ------------------------------------------------------------------ */
/* get_timing: delegate to panel layer                                */
/* ------------------------------------------------------------------ */
static int dsi_get_timing(struct display_timing *timing)
{
	if (!timing)
		return -EINVAL;

	return x5_panel_get_timing(timing);
}

/* ------------------------------------------------------------------ */
/* clk_init: enable DSI-specific clocks (APB, DPHY cfg, TXESC)       */
/* ------------------------------------------------------------------ */
static int dsi_clk_init(unsigned long pixel_clock)
{
	return x5_crm_dsi_clk_init(pixel_clock);
}

/* ------------------------------------------------------------------ */
/* reset: DSI-TX-specific reset                                       */
/* ------------------------------------------------------------------ */
static int dsi_reset(void)
{
	/*
	 * Must match historical x5_display_hw_init(): one combined pulse of
	 * RST_DC8000_* and RST_DSI_TX.  Do not use split common_reset +
	 * dsi_reset — that changes reset timing vs. kernel / seamless expect.
	 */
	// return x5_crm_dsi_reset();
	return x5_crm_display_reset();
}

/* ------------------------------------------------------------------ */
/* bridge_init: configure SYSCON to route DC8000 -> DSI-TX            */
/* ------------------------------------------------------------------ */
static int dsi_bridge_init(struct display_timing *timing)
{
	if (!timing)
		return -EINVAL;

	return x5_syscon_bridge_init(timing);
}

/* ------------------------------------------------------------------ */
/* hw_init: initialise DSI host and DPHY                              */
/* ------------------------------------------------------------------ */
static int dsi_hw_init(struct display_timing *timing)
{
	if (!timing)
		return -EINVAL;

	return x5_dsi_host_init(timing,
				x5_panel_get_lanes(),
				x5_panel_get_format());
}

/* ------------------------------------------------------------------ */
/* enable: send panel init sequence + switch to DSI video mode        */
/* ------------------------------------------------------------------ */
static int dsi_enable(void)
{
	int ret;

	ret = x5_panel_init_sequence();
	DISP_LOG_DEBUG("DSI x5_panel_init_sequence ret=%d\n", ret);
	if (ret)
		/* Prefix "X5 Display: WARNING" comes from DISP_LOG_WARN (x5_display_log.h) */
		DISP_LOG_WARN("Panel init failed (ret=%d), continuing\n", ret);

	x5_dsi_host_enable();
	DISP_LOG_DEBUG("DSI x5_dsi_host_enable done\n");
	return 0;
}

/* ------------------------------------------------------------------ */
/* disable: tear down DSI host + SYSCON bridge                        */
/* ------------------------------------------------------------------ */
static int dsi_disable(void)
{
	x5_dsi_host_disable();
	x5_syscon_bridge_disable();
	return 0;
}

/* ------------------------------------------------------------------ */
/* clk_disable: gate DSI-specific clocks                              */
/* ------------------------------------------------------------------ */
static void dsi_clk_disable(void)
{
	x5_crm_dsi_clk_disable();
}

/* ------------------------------------------------------------------ */
/* get_backlight_config: delegate to panel layer                      */
/* ------------------------------------------------------------------ */
static int dsi_get_backlight_config(struct x5_backlight_config *config)
{
	if (!config)
		return -EINVAL;

	return x5_panel_get_backlight_config(config);
}

/* ------------------------------------------------------------------ */
/* get_info: human-readable description                               */
/* U-Boot display path is single-threaded; static buffer is not contested. */
/* ------------------------------------------------------------------ */
static const char *dsi_get_info(void)
{
	static char info_buf[64];

	snprintf(info_buf, sizeof(info_buf),
		 "MIPI DSI (%d lanes, RGB888)", x5_panel_get_lanes());
	return info_buf;
}

/* ================================================================== */
/* Exported ops table                                                  */
/* ================================================================== */
const struct x5_display_output_ops x5_dsi_output_ops = {
	.name                = "MIPI-DSI",
	.type                = X5_OUTPUT_DSI,
	.detect              = dsi_detect,
	.get_timing          = dsi_get_timing,
	.clk_init            = dsi_clk_init,
	.reset               = dsi_reset,
	.bridge_init         = dsi_bridge_init,
	.hw_init             = dsi_hw_init,
	.enable              = dsi_enable,
	.disable             = dsi_disable,
	.clk_disable         = dsi_clk_disable,
	.get_backlight_config = dsi_get_backlight_config,
	.get_info            = dsi_get_info,
};
