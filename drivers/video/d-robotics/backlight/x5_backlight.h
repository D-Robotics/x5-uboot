/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 *
 * X5 LPWM Backlight Driver Header
 */

#ifndef __X5_BACKLIGHT_H__
#define __X5_BACKLIGHT_H__

#include <linux/types.h>

/*
 * LSIO pinmux / drive-strength block (same reg as &lsio_iomuxc in arch/arm/dts/x5.dtsi).
 * Used by LPWM/PWM backlight paths to match kernel pin state during seamless handoff.
 */
#define X5_LSIO_IOMUXC_BASE	0x34180000u

/*
 * PWM Type
 */
enum x5_pwm_type {
	X5_PWM_TYPE_LPWM = 0,  /* Low Power PWM (LPWM0-LPWM1) */
	X5_PWM_TYPE_STD = 1,   /* Standard PWM (PWM0-PWM3) */
};

/*
 * LPWM Base Addresses (arch/arm/dts/x5.dtsi lpwm0 / lpwm1)
 */
#define LPWM0_BASE		0x34100000
#define LPWM1_BASE		0x34110000
#define LPWM_MMIO_MASK		0xffff0000u

/*
 * Standard PWM Base Addresses (from kernel x5.dtsi)
 */
#define PWM2_BASE 0x34160000

/* GPIO for backlight enable (set gpio_num to -1 if unused) */
struct x5_backlight_gpio
{
	int gpio_num;	 /* GPIO number, -1 if not used */
	int active_high; /* 1 = active high, 0 = active low */
};

/* Backlight configuration (supports both LPWM and standard PWM) */
struct x5_backlight_config
{
	/* PWM type and configuration */
	enum x5_pwm_type pwm_type; /* PWM type: LPWM or standard PWM */
	unsigned long pwm_base;    /* PWM base address */
	int pwm_id;                /* PWM instance ID */
	int channel;               /* PWM channel */

	/* Timing configuration */
	u32 period_ns;             /* PWM period in nanoseconds (for standard PWM) */
	u32 duty_ns;               /* PWM duty in nanoseconds (for standard PWM) */

	/* LPWM-specific configuration (ignored for standard PWM) */
	u32 div_ratio;             /* Clock divider (0-1023) */
	u32 period;                /* PWM period (ticks) */
	u32 duty;                  /* PWM duty (ticks, <= period) */

	/* Optional backlight enable GPIO */
	struct x5_backlight_gpio enable_gpio;
};

/* Backlight operations (implemented by each PWM type) */
struct x5_backlight_ops {
	int (*init)(const struct x5_backlight_config *config);
	int (*enable)(const struct x5_backlight_config *config);
	int (*disable)(const struct x5_backlight_config *config);
	int (*set_brightness)(const struct x5_backlight_config *config, int percent);
	void (*dump)(const struct x5_backlight_config *config);
};

/*
 * Backlight driver structure
 */
struct x5_backlight_driver {
	const char *name;
	enum x5_pwm_type type;
	const struct x5_backlight_ops *ops;
};

/* API functions */
int x5_backlight_init(const struct x5_backlight_config *config);
int x5_backlight_enable(void);
int x5_backlight_disable(void);
int x5_backlight_set_brightness(int percent);  /* 0-100% */
void x5_backlight_dump(void);

/*
 * Driver registration (implemented by each driver)
 */
extern const struct x5_backlight_driver x5_backlight_lpwm_driver;
extern const struct x5_backlight_driver x5_backlight_pwm_driver;

#endif /* __X5_BACKLIGHT_H__ */
