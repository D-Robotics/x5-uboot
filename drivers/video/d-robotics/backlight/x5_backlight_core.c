// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 *
 * X5 Backlight Core - routes operations to LPWM/PWM drivers
 *
 * Global state (g_bl_config, g_bl_driver, g_bl_initialized):
 * U-Boot runs this driver on a single-threaded boot path with no concurrent
 * calls into these APIs from another context, so the globals are not subject
 * to races. This matches common U-Boot practice for early display/backlight.
 */

#include <common.h>
#include <asm/gpio.h>
#include "x5_backlight.h"
#include <hb_display_log.h>

/* Global state — see file comment above */
static struct x5_backlight_config g_bl_config;
static const struct x5_backlight_driver *g_bl_driver;
static int g_bl_initialized;

/* Driver table */
static const struct x5_backlight_driver *backlight_drivers[] = {
	&x5_backlight_lpwm_driver,
	&x5_backlight_pwm_driver,
	NULL
};

/*
 * Find driver by PWM type
 */
static const struct x5_backlight_driver *find_driver(enum x5_pwm_type type)
{
	int i;

	for (i = 0; backlight_drivers[i]; i++) {
		if (backlight_drivers[i]->type == type)
			return backlight_drivers[i];
	}

	return NULL;
}

/*
 * Drive backlight enable GPIO using legacy GPIO numbers (see x5_backlight_gpio).
 * Level is logical "enable backlight": combined with active_high/active_low.
 */
static int backlight_config_gpio(const struct x5_backlight_gpio *gpio, int enable)
{
	unsigned int offset = (unsigned int)gpio->gpio_num;
	int level;

	if (gpio->gpio_num < 0)
		return 0; /* No GPIO configured */

	/* Map logical enable to pin level (handles active-high vs active-low). */
	level = enable ? (gpio->active_high ? 1 : 0) : (gpio->active_high ? 0 : 1);
	if (gpio_direction_output(offset, level)) {
		BL_LOG_ERROR("GPIO %d direction/output failed (enable=%d, level=%d)\n",
			     gpio->gpio_num, enable, level);
		return -EIO;
	}

	BL_LOG_DEBUG("GPIO %d %s (active %s, level %d)\n",
		     gpio->gpio_num,
		     enable ? "on" : "off",
		     gpio->active_high ? "high" : "low",
		     level);

	return 0;
}

/*
 * Initialize backlight with given configuration
 */
int x5_backlight_init(const struct x5_backlight_config *config)
{
	int ret;

	if (!config) {
		BL_LOG_ERROR("Invalid config\n");
		return -EINVAL;
	}

	g_bl_driver = find_driver(config->pwm_type);
	if (!g_bl_driver) {
		BL_LOG_ERROR("No driver found for PWM type %d\n", config->pwm_type);
		return -ENODEV;
	}

	memset(&g_bl_config, 0, sizeof(g_bl_config));
	memcpy(&g_bl_config, config, sizeof(g_bl_config));

	BL_LOG_INFO("Using driver: %s (PWM type %d)\n", g_bl_driver->name,
		    (int)config->pwm_type);

	if (g_bl_config.enable_gpio.gpio_num >= 0 &&
	    gpio_request((unsigned int)g_bl_config.enable_gpio.gpio_num, "x5_bl_en"))
		BL_LOG_WARN("gpio_request enable GPIO %d failed (BL may still work)\n",
			    g_bl_config.enable_gpio.gpio_num);

	ret = g_bl_driver->ops->init(config);
	if (ret) {
		BL_LOG_ERROR("Driver init failed: PWM type %d, driver %s, ret=%d\n",
			     config->pwm_type, g_bl_driver->name, ret);
		return ret;
	}

	g_bl_initialized = 1;
	return 0;
}

/*
 * Enable backlight output
 */
int x5_backlight_enable(void)
{
	int ret;

	if (!g_bl_initialized) {
		/* HDMI (and outputs without a panel BL) never call x5_backlight_init; enable is optional. */
		BL_LOG_DEBUG("Not initialized\n");
		return -ENODEV;
	}

	/* Enable GPIO first (if configured) */
	ret = backlight_config_gpio(&g_bl_config.enable_gpio, 1);
	if (ret)
		return ret;

	ret = g_bl_driver->ops->enable(&g_bl_config);
	if (ret) {
		BL_LOG_ERROR("Driver enable failed: PWM type %d, ret=%d\n",
			     (int)g_bl_config.pwm_type, ret);
		return ret;
	}

	return 0;
}

/*
 * Disable backlight output
 */
int x5_backlight_disable(void)
{
	int ret;

	if (!g_bl_initialized)
		return -ENODEV;

	ret = g_bl_driver->ops->disable(&g_bl_config);
	if (ret) {
		BL_LOG_ERROR("Driver disable failed: PWM type %d, ret=%d\n",
			     (int)g_bl_config.pwm_type, ret);
		return ret;
	}

	/* Disable GPIO (if configured) */
	ret = backlight_config_gpio(&g_bl_config.enable_gpio, 0);
	if (ret)
		return ret;

	return 0;
}

/*
 * Set backlight brightness (0-100%)
 */
int x5_backlight_set_brightness(int percent)
{
	int ret;

	if (!g_bl_initialized)
		return -ENODEV;

	if (percent < 0)
		percent = 0;
	if (percent > 100)
		percent = 100;

	ret = g_bl_driver->ops->set_brightness(&g_bl_config, percent);
	if (ret) {
		BL_LOG_ERROR("Driver set_brightness failed: PWM type %d, percent %d, ret=%d\n",
			     (int)g_bl_config.pwm_type, percent, ret);
		return ret;
	}

	return 0;
}

/*
 * Dump backlight registers for debugging
 */
void x5_backlight_dump(void)
{
	if (!g_bl_initialized) {
		BL_LOG_DEBUG("Not initialized\n");
		return;
	}

	g_bl_driver->ops->dump(&g_bl_config);

	if (g_bl_config.enable_gpio.gpio_num >= 0) {
		BL_LOG_DEBUG("Enable GPIO: %d (active %s)\n",
			   g_bl_config.enable_gpio.gpio_num,
			   g_bl_config.enable_gpio.active_high ? "high" : "low");
	} else {
		BL_LOG_DEBUG("Enable GPIO: not configured\n");
	}
}
