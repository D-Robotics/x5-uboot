// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 *
 * Hobot X5 LPWM (Low Power PWM) Driver
 *
 * Register layout from kernel: drivers/media/platform/horizon/camsys/lpwm/
 * Clock configuration from kernel: drivers/clk/hobot/clk-x5.c
 */

#include <common.h>
#include <div64.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <errno.h>
#include <pwm.h>
#include <asm/io.h>
#include <linux/delay.h>
#include "../video/d-robotics/clock/x5_crm.h"

/* LPWM Register Offsets (from kernel driver) */
#define LPWM_GLB_CFG		0x0000
#define LPWM_SW_TRIG		0x0004
#define LPWM_RST		0x0008
#define LPWM_0_CFG0		0x0010
#define LPWM_0_CFG1		0x0014
#define LPWM_0_CFG2		0x0018
#define LPWM_1_CFG0		0x001C
#define LPWM_1_CFG1		0x0020
#define LPWM_1_CFG2		0x0024
#define LPWM_2_CFG0		0x0028
#define LPWM_2_CFG1		0x002C
#define LPWM_2_CFG2		0x0030
#define LPWM_3_CFG0		0x0034
#define LPWM_3_CFG1		0x0038
#define LPWM_3_CFG2		0x003C

/* Channel register offset: 12 bytes per channel */
#define LPWM_CORE_REG_OFFSET	12

/* LPWM_GLB_CFG bits */
#define LPWM_GLB_CFG_EN(ch)	BIT(ch)
#define LPWM_GLB_CFG_INT_EN	BIT(4)
#define LPWM_GLB_CFG_MODE_SEL	BIT(5)
#define LPWM_GLB_CFG_TRIG_SRC_SHIFT	8
#define LPWM_GLB_CFG_TRIG_SRC_MASK	(0xF << 8)
#define LPWM_GLB_CFG_DIV_RATIO_SHIFT	12
#define LPWM_GLB_CFG_DIV_RATIO_MASK	(0x3FF << 12)

/* LPWM_0_CFG1 bits */
#define LPWM_CFG1_PERIOD_MASK		0xFFFFF
#define LPWM_CFG1_DUTY_TIME_SHIFT	20
#define LPWM_CFG1_DUTY_TIME_MASK	0xFFF

/* LPWM_0_CFG0 bits */
#define LPWM_CFG0_OFFSET_MASK		0xFFFFF

/* LPWM supports 4 channels */
#define LPWM_NUM_CHANNELS	4

/* Clock configuration
 * Base clock: 24MHz
 * Default divider: 0x257 (599)
 * Effective clock: 24MHz / 600 = 40kHz = 25us per tick
 */
#define LPWM_BASE_CLK_FREQ	24000000UL
#define LPWM_DEFAULT_DIV	599
#define LPWM_CLK_MUL_FACTOR	1000  /* ns to us conversion */

struct hobot_lpwm_priv {
	void __iomem *base;
	u32 clk_div;  /* Clock divider */
	u32 lpwm_id;  /* LPWM instance ID (0 or 1) */
};

static int hobot_lpwm_set_invert(struct udevice *dev, uint channel, bool polarity)
{
	if (channel >= LPWM_NUM_CHANNELS)
		return -EINVAL;

	/* X5 LPWM doesn't support polarity inversion in hardware */
	debug("LPWM: %s: channel=%u, polarity=%u (not supported)\n",
	      __func__, channel, polarity);
	return polarity ? -ENOSYS : 0;
}

static int hobot_lpwm_set_config(struct udevice *dev, uint channel,
				 uint period_ns, uint duty_ns)
{
	struct hobot_lpwm_priv *priv = dev_get_priv(dev);
	u64 period_us, duty_us;
	u32 period_val, duty_val, cfg1_val;
	u32 cfg0_offset, cfg1_offset;

	if (channel >= LPWM_NUM_CHANNELS)
		return -EINVAL;

	debug("LPWM: %s: channel=%u, period_ns=%u, duty_ns=%u\n",
	      __func__, channel, period_ns, duty_ns);

	/* Convert ns to us (LPWM works in microseconds)
	 * With default divider 599, effective clock is 40kHz (25us per tick)
	 * But kernel uses 1us granularity, so we follow that
	 */
	period_us = (u64)period_ns;
	do_div(period_us, LPWM_CLK_MUL_FACTOR);
	period_val = (u32)period_us;

	duty_us = (u64)duty_ns;
	do_div(duty_us, LPWM_CLK_MUL_FACTOR);
	duty_val = (u32)duty_us;

	/* Ensure duty cycle doesn't exceed period */
	if (duty_val > period_val)
		duty_val = period_val;

	/* X5 IP requires period and duty -1 (from kernel driver) */
	if (period_val > 0)
		period_val -= 1;
	if (duty_val > 0)
		duty_val -= 1;

	/* Limit to hardware max values */
	period_val &= LPWM_CFG1_PERIOD_MASK;
	duty_val &= LPWM_CFG1_DUTY_TIME_MASK;

	debug("LPWM: %s: period_us=%u, duty_us=%u (after -1)\n",
	      __func__, period_val, duty_val);

	/* Calculate register offsets */
	cfg0_offset = LPWM_0_CFG0 + channel * LPWM_CORE_REG_OFFSET;
	cfg1_offset = LPWM_0_CFG1 + channel * LPWM_CORE_REG_OFFSET;

	/* CFG0: Set offset to 0 (no delay) */
	writel(0, priv->base + cfg0_offset);

	/* CFG1: duty_time[31:20] | period[19:0] */
	cfg1_val = (duty_val << LPWM_CFG1_DUTY_TIME_SHIFT) | period_val;
	writel(cfg1_val, priv->base + cfg1_offset);

	return 0;
}

static int hobot_lpwm_set_enable(struct udevice *dev, uint channel, bool enable)
{
	struct hobot_lpwm_priv *priv = dev_get_priv(dev);
	u32 glb_cfg;

	if (channel >= LPWM_NUM_CHANNELS)
		return -EINVAL;

	debug("LPWM: %s: channel=%u, enable=%u\n", __func__, channel, enable);

	glb_cfg = readl(priv->base + LPWM_GLB_CFG);
	if (enable) {
		glb_cfg |= LPWM_GLB_CFG_EN(channel);
		writel(glb_cfg, priv->base + LPWM_GLB_CFG);

		/* Software trigger (mode 0) */
		writel(1, priv->base + LPWM_SW_TRIG);
	} else {
		glb_cfg &= ~LPWM_GLB_CFG_EN(channel);
		writel(glb_cfg, priv->base + LPWM_GLB_CFG);
	}

	return 0;
}

static int hobot_lpwm_of_to_plat(struct udevice *dev)
{
	struct hobot_lpwm_priv *priv = dev_get_priv(dev);
	fdt_addr_t addr;

	addr = dev_read_addr(dev);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	priv->base = (void __iomem *)addr;

	/* Determine LPWM instance ID from base address
	 * LPWM0: 0x34100000
	 * LPWM1: 0x34110000
	 */
	if ((addr & 0xFFFF0000) == 0x34100000)
		priv->lpwm_id = 0;
	else if ((addr & 0xFFFF0000) == 0x34110000)
		priv->lpwm_id = 1;
	else {
		dev_err(dev, "LPWM: Unknown base address 0x%lx\n", (unsigned long)addr);
		priv->lpwm_id = 0;  /* Default to LPWM0 */
	}

	/* Read clock divider from DT, default to 599 */
	priv->clk_div = dev_read_u32_default(dev, "clock-divider", LPWM_DEFAULT_DIV);

	debug("LPWM: %s: base=%p, lpwm_id=%u, clk_div=%u\n",
	      __func__, priv->base, priv->lpwm_id, priv->clk_div);

	return 0;
}

static int hobot_lpwm_probe(struct udevice *dev)
{
	struct hobot_lpwm_priv *priv = dev_get_priv(dev);
	u32 glb_cfg;
	int ret;

	debug("LPWM: %s: Hobot LPWM%d probe\n", __func__, priv->lpwm_id);

	/* Enable clocks via CRM module */
	ret = x5_crm_lpwm_clk_init(priv->lpwm_id);
	if (ret) {
		dev_err(dev, "LPWM: Failed to enable clocks (lpwm_id=%u)\n",
			priv->lpwm_id);
		return ret;
	}

	/* Initialize global config register
	 * - Disable all channels
	 * - Set trigger mode to software (bit 5 = 0)
	 * - Set trigger source to software (bits 8-11 = 5)
	 * - Set clock divider (bits 12-21)
	 */
	glb_cfg = (5 << LPWM_GLB_CFG_TRIG_SRC_SHIFT) |
		  (priv->clk_div << LPWM_GLB_CFG_DIV_RATIO_SHIFT);
	writel(glb_cfg, priv->base + LPWM_GLB_CFG);

	/* Reset all channels */
	writel(1, priv->base + LPWM_RST);
	udelay(10);

	debug("LPWM: %s: LPWM%d initialized (GLB_CFG=0x%08x)\n",
	      __func__, priv->lpwm_id, glb_cfg);

	return 0;
}

static const struct pwm_ops hobot_lpwm_ops = {
	.set_invert	= hobot_lpwm_set_invert,
	.set_config	= hobot_lpwm_set_config,
	.set_enable	= hobot_lpwm_set_enable,
};

static const struct udevice_id hobot_lpwm_ids[] = {
	{ .compatible = "hobot,x5-lpwm" },
	{ .compatible = "d-robotics,x5-lpwm" },
	{ }
};

U_BOOT_DRIVER(hobot_lpwm) = {
	.name		= "hobot_lpwm",
	.id		= UCLASS_PWM,
	.of_match	= hobot_lpwm_ids,
	.ops		= &hobot_lpwm_ops,
	.of_to_plat	= hobot_lpwm_of_to_plat,
	.probe		= hobot_lpwm_probe,
	.priv_auto	= sizeof(struct hobot_lpwm_priv),
};
