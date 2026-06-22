// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 * Modified by: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * X5 Panel Core — runtime selection
 *
 * Order: env x5_lcd_panel → Kconfig default (menuconfig) when env unset →
 * DT x5,default-panel on slot → legacy DT compatible match → first compiled driver.
 * DT slot: leading compatible X5_PANEL_SLOT_COMPAT; panel drivers read GPIO /
 * timing from x5_panel_slot_ofnode().  setenv x5_lcd_panel jc050; saveenv
 */

#include <common.h>
#include <dm/ofnode.h>
#include <env.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kconfig.h>
#include "x5_panel.h"
#include "panel_config.h"
#include <hb_display_log.h>

static const struct x5_panel *const panel_table[] = {
#ifdef CONFIG_X5_PANEL_JC050HD134
	&panel_jc050hd134,
#endif
#ifdef CONFIG_X5_PANEL_WH_CM480
	&panel_wh_cm480,
#endif
#ifdef CONFIG_X5_PANEL_ST77031
	&panel_st77031,
#endif
	NULL
};

static const struct x5_panel *active_panel;

static ofnode slot_cached;
static u8 slot_cached_valid;

ofnode x5_panel_slot_ofnode(void)
{
	if (slot_cached_valid)
		return slot_cached;

	slot_cached = ofnode_by_compatible(ofnode_null(), X5_PANEL_SLOT_COMPAT);
	slot_cached_valid = 1;
	return slot_cached;
}

/* Map env / x5,default-panel tokens to panel_table[].compatible */
static const char *x5_panel_alias_to_compat(const char *token)
{
	static const struct {
		const char *alias;
		const char *compat;
	} map[] = {
		{ "jc-050hd134",	"jc-050hd134" },
		{ "jc050",		"jc-050hd134" },
		{ "wh-cm480",		"wh-cm480" },
		{ "wh480",		"wh-cm480" },
		{ "st77031",		"st77031" },
	};
	size_t i;

	if (!token || !token[0])
		return NULL;

	for (i = 0; i < ARRAY_SIZE(map); i++) {
		if (!strcasecmp(token, map[i].alias))
			return map[i].compat;
	}
	return token;
}

/* menuconfig "Default LCD panel when env x5_lcd_panel is unset" */
static const char *x5_panel_kconfig_default_token(void)
{
#if IS_ENABLED(CONFIG_X5_DEFAULT_LCD_JC050)
	return "jc050";
#elif IS_ENABLED(CONFIG_X5_DEFAULT_LCD_WH480)
	return "wh480";
#elif IS_ENABLED(CONFIG_X5_DEFAULT_LCD_ST77031)
	return "st77031";
#else
	return NULL;
#endif
}

static const struct x5_panel *x5_panel_by_compat(const char *compat)
{
	int i;

	if (!compat || !compat[0])
		return NULL;

	for (i = 0; panel_table[i]; i++) {
		if (!strcasecmp(panel_table[i]->compatible, compat))
			return panel_table[i];
	}
	return NULL;
}

static const struct x5_panel *x5_panel_detect(void)
{
	const struct x5_panel *p;
	ofnode slot;
	const char *env, *compat, *def;
	int i;
	ofnode node;

	if (active_panel)
		return active_panel;

	slot = x5_panel_slot_ofnode();

	/* 1) env x5_lcd_panel (saveenv + reset; no FDT change) */
	env = env_get("x5_lcd_panel");
	if (env && env[0]) {
		compat = x5_panel_alias_to_compat(env);
		p = x5_panel_by_compat(compat);
		if (p) {
			active_panel = p;
			PANEL_LOG_INFO("Panel from env x5_lcd_panel=%s -> %s\n",
				       env, active_panel->name);
			return active_panel;
		}
		PANEL_LOG_WARN("x5_lcd_panel=\"%s\" unknown or not built-in; ignored\n",
			       env);
	} else {
		/* 2) Kconfig default: only when env unset or empty */
		def = x5_panel_kconfig_default_token();
		if (def && def[0]) {
			compat = x5_panel_alias_to_compat(def);
			p = x5_panel_by_compat(compat);
			if (p) {
				active_panel = p;
				PANEL_LOG_INFO("Panel from Kconfig default %s -> %s\n",
					       def, active_panel->name);
				return active_panel;
			}
			PANEL_LOG_WARN("Kconfig default \"%s\" not built-in; ignored\n",
				       def);
		}
	}

	/* 3) DT x5,default-panel on slot */
	if (ofnode_valid(slot)) {
		def = ofnode_read_string(slot, "x5,default-panel");
		if (def && def[0]) {
			compat = x5_panel_alias_to_compat(def);
			p = x5_panel_by_compat(compat);
			if (p) {
				active_panel = p;
				PANEL_LOG_INFO("Panel from DT x5,default-panel=%s -> %s\n",
					       def, active_panel->name);
				return active_panel;
			}
			PANEL_LOG_WARN("x5,default-panel=\"%s\" unknown or not built-in\n",
				       def);
		}
	}

	/* 4) Legacy: first enabled node matching a built-in compatible */
	for (i = 0; panel_table[i]; i++) {
		node = ofnode_by_compatible(ofnode_null(),
					    panel_table[i]->compatible);
		if (ofnode_valid(node) && ofnode_is_enabled(node)) {
			active_panel = panel_table[i];
			PANEL_LOG_INFO("Matched DT panel: %s (compatible=%s)\n",
				       active_panel->name,
				       active_panel->compatible);
			return active_panel;
		}
	}

	if (ofnode_valid(slot)) {
		PANEL_LOG_WARN("No panel matched (slot " X5_PANEL_SLOT_COMPAT
			       " present); check x5_lcd_panel / menuconfig default / x5,default-panel\n");
	}

	/* 5) First compiled driver */
	if (panel_table[0]) {
		active_panel = panel_table[0];
		PANEL_LOG_WARN("LCD: fallback panel driver %s (compatible=%s); "
			       "no env/DT/slot match - check x5_lcd_panel / x5,default-panel / Kconfig\n",
			       active_panel->name, active_panel->compatible);
		return active_panel;
	}

	PANEL_LOG_ERROR("No panel drivers compiled in\n");
	return NULL;
}

static inline const struct x5_panel *get_active_panel(void)
{
	return x5_panel_detect();
}

int x5_panel_init_sequence(void)
{
	const struct x5_panel *panel = get_active_panel();
	int ret;

	if (!panel || !panel->ops) {
		PANEL_LOG_ERROR("No active panel configured\n");
		return -ENODEV;
	}

	PANEL_LOG_WARN("LCD: initializing panel driver %s (compatible=%s)\n",
		       panel->name, panel->compatible);

	if (panel->ops->reset) {
		ret = panel->ops->reset();
		if (ret) {
			PANEL_LOG_ERROR("Panel reset failed (ret=%d)\n", ret);
			return ret;
		}
	}

	if (panel->ops->init) {
		ret = panel->ops->init();
		if (ret) {
			PANEL_LOG_ERROR("Panel init failed (ret=%d)\n", ret);
			return ret;
		}
	}

	return 0;
}

int x5_panel_get_timing(struct display_timing *timing)
{
	const struct x5_panel *panel = get_active_panel();

	if (!timing)
		return -EINVAL;

	if (!panel || !panel->ops || !panel->ops->get_timing) {
		PANEL_LOG_ERROR("No timing callback\n");
		return -ENODEV;
	}

	return panel->ops->get_timing(timing);
}

int x5_panel_get_lanes(void)
{
	const struct x5_panel *panel = get_active_panel();

	if (!panel || !panel->ops || !panel->ops->get_lanes) {
		PANEL_LOG_ERROR("No get_lanes callback\n");
		return -ENODEV;
	}

	return panel->ops->get_lanes();
}

enum mipi_dsi_pixel_format x5_panel_get_format(void)
{
	const struct x5_panel *panel = get_active_panel();

	if (!panel || !panel->ops || !panel->ops->get_format) {
		PANEL_LOG_ERROR("No get_format callback\n");
		return MIPI_DSI_FMT_RGB888;
	}

	return panel->ops->get_format();
}

const char *x5_panel_get_name(void)
{
	const struct x5_panel *panel = get_active_panel();

	if (!panel || !panel->ops || !panel->ops->get_name)
		return "unknown";

	return panel->ops->get_name();
}

int x5_panel_get_backlight_config(struct x5_backlight_config *config)
{
	const struct x5_panel *panel = get_active_panel();

	if (!config)
		return -EINVAL;

	if (!panel || !panel->ops) {
		PANEL_LOG_ERROR("No active panel\n");
		return -ENODEV;
	}

	if (!panel->ops->get_backlight_config) {
		PANEL_LOG_DEBUG("Panel %s has no backlight config\n", panel->name);
		return -ENODEV;
	}

	return panel->ops->get_backlight_config(config);
}
