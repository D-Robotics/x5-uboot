// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 *
 * D-Robotics Standard PWM Driver for U-Boot
 *
 * Ported from kernel driver: kernel/drivers/pwm/pwm-drobot.c
 *
 * This driver supports standard PWM controllers (PWM0-PWM3)
 * Each controller has 2 channels.
 *
 * Hardware:
 * - PWM0: 0x34090000
 * - PWM1: 0x340A0000
 * - PWM2: 0x340B0000
 * - PWM3: 0x340C0000
 */

#include <common.h>
#include <dm.h>
#include <pwm.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>

/* Register offsets */
#define PWM_MCR                 0x00
#define PWM_ISR                 0x04
#define PWM_CTR(x)              (0x10 + (0x20 * (x)))
#define PWM_CCR(x)              (0x14 + (0x20 * (x)))
#define PWM_PW16AR(x)           (0x18 + (0x20 * (x)))
#define PWM_PW8AR(x)            (0x1c + (0x20 * (x)))
#define PWM_PR(x)               (0x20 + (0x20 * (x)))
#define PWM_CR(x)               (0x24 + (0x20 * (x)))
#define PWM_SR(x)               (0x28 + (0x20 * (x)))

/* MCR bits */
#define PWM_CHANNEL0_EN         BIT(0)
#define PWM_CHANNEL1_EN         BIT(4)

/* CTR bits */
#define PWM_POLARITY            BIT(0)

/* CCR bits */
#define PWM_CLK_APB             (0x00 << 0)
#define PWM_CLK_PWM             (0x01 << 0)

/* Constants */
#define PWM_DUTY_FIFO_SIZE      64
#define PWM_FIFO_DEPTH          (PWM_DUTY_FIFO_SIZE / (sizeof(u16) * 8))

struct hobot_pwm_priv {
	void __iomem *base;
	u32 clk_rate;  /* APB clock rate */
};

/*
 * Calculate PWM parameters
 *
 * PWM frequency = clk_rate / (2^(div_reg+1) * prescale * (prd+1))
 *
 * @clk_rate: APB clock rate (Hz)
 * @period_ns: Desired period in nanoseconds
 * @div_reg: Output divider register value (0-7)
 * @prescale: Output prescaler value (1-256)
 * @prd: Output period register value (0-65535)
 */
static int hobot_pwm_calc_params(u32 clk_rate, u32 period_ns,
                                  u8 *div_reg, u16 *prescale, u16 *prd)
{
	u64 period_cycles;
	u32 prd_prescale;
	u8 div = 0;
	u16 pre;

	period_cycles = (u64)clk_rate * period_ns / 1000000000ULL;

	while (div <= 7) {
		prd_prescale = period_cycles >> (div + 1);
		if (prd_prescale <= 0xffff * 256)
			break;
		div++;
	}

	pre = prd_prescale / 0xffff;
	if (prd_prescale % 0xffff)
		pre++;

	if (pre > 256) {
		debug("PWM: prescale overflow, clamping to 256\n");
		pre = 256;
	}

	*div_reg = div;
	*prescale = pre;
	*prd = (prd_prescale / pre) - 1;

	return 0;
}

/*
 * Set PWM duty cycle
 */
static void hobot_pwm_set_duty(struct hobot_pwm_priv *priv, uint channel,
                                u32 duty_ns, u8 div_reg, u16 prescale)
{
	u64 duty_cycles;
	u16 duty;
	int i;

	duty_cycles = (u64)priv->clk_rate * duty_ns / 1000000000ULL;

	duty = (duty_cycles >> (div_reg + 1)) / prescale;

	if (duty == 0)
		debug("PWM: Warning - duty is 0, no waveform output\n");

	for (i = 0; i < PWM_FIFO_DEPTH; i++)
		writel(duty, priv->base + PWM_PW16AR(channel));
}

/*
 * Enable PWM channel
 */
static void hobot_pwm_enable(struct hobot_pwm_priv *priv, uint channel)
{
	u32 val;

	val = readl(priv->base + PWM_MCR);
	if (channel == 0)
		val |= PWM_CHANNEL0_EN;
	else
		val |= PWM_CHANNEL1_EN;
	writel(val, priv->base + PWM_MCR);
}

/*
 * Disable PWM channel
 */
static void hobot_pwm_disable(struct hobot_pwm_priv *priv, uint channel)
{
	u32 val;

	val = readl(priv->base + PWM_MCR);
	if (channel == 0)
		val &= ~PWM_CHANNEL0_EN;
	else
		val &= ~PWM_CHANNEL1_EN;
	writel(val, priv->base + PWM_MCR);
}

/*
 * Set PWM configuration
 */
static int hobot_pwm_set_config(struct udevice *dev, uint channel,
                                 uint period_ns, uint duty_ns)
{
	struct hobot_pwm_priv *priv = dev_get_priv(dev);
	u8 div_reg;
	u16 prescale, prd;
	u32 val;
	int ret;

	if (channel >= 2) {
		debug("PWM: Invalid channel %d (max 1)\n", channel);
		return -EINVAL;
	}

	ret = hobot_pwm_calc_params(priv->clk_rate, period_ns,
	                             &div_reg, &prescale, &prd);
	if (ret)
		return ret;

	val = PWM_CLK_APB | (div_reg << 4) | ((prescale - 1) << 8);
	writel(val, priv->base + PWM_CCR(channel));

	writel(prd, priv->base + PWM_PR(channel));

	if (duty_ns)
		hobot_pwm_set_duty(priv, channel, duty_ns, div_reg, prescale);

	debug("PWM%d CH%d: period=%uns, duty=%uns (div=%d, pre=%d, prd=%d)\n",
	      dev->seq_, channel, period_ns, duty_ns, div_reg, prescale, prd);

	return 0;
}

/*
 * Set PWM enable state
 */
static int hobot_pwm_set_enable(struct udevice *dev, uint channel, bool enable)
{
	struct hobot_pwm_priv *priv = dev_get_priv(dev);

	if (channel >= 2) {
		debug("PWM: Invalid channel %d (max 1)\n", channel);
		return -EINVAL;
	}

	if (enable)
		hobot_pwm_enable(priv, channel);
	else
		hobot_pwm_disable(priv, channel);

	return 0;
}

/*
 * Set PWM polarity
 */
static int hobot_pwm_set_invert(struct udevice *dev, uint channel, bool polarity)
{
	struct hobot_pwm_priv *priv = dev_get_priv(dev);
	u32 val;

	if (channel >= 2) {
		debug("PWM: Invalid channel %d (max 1)\n", channel);
		return -EINVAL;
	}

	val = readl(priv->base + PWM_CTR(channel));
	if (polarity)
		val |= PWM_POLARITY;
	else
		val &= ~PWM_POLARITY;
	writel(val, priv->base + PWM_CTR(channel));

	return 0;
}

static const struct pwm_ops hobot_pwm_ops = {
	.set_config = hobot_pwm_set_config,
	.set_enable = hobot_pwm_set_enable,
	.set_invert = hobot_pwm_set_invert,
};

static int hobot_pwm_probe(struct udevice *dev)
{
	struct hobot_pwm_priv *priv = dev_get_priv(dev);

	priv->base = dev_read_addr_ptr(dev);
	if (!priv->base) {
		debug("PWM: Failed to get base address\n");
		return -EINVAL;
	}

	/* Get clock rate from device tree or use default */
	priv->clk_rate = dev_read_u32_default(dev, "clock-frequency", 24000000);

	debug("PWM: Initialized at 0x%p, clock=%u Hz\n",
	      priv->base, priv->clk_rate);

	return 0;
}

static const struct udevice_id hobot_pwm_ids[] = {
	{ .compatible = "d-robotics,pwm" },
	{ }
};

U_BOOT_DRIVER(hobot_pwm) = {
	.name = "hobot_pwm",
	.id = UCLASS_PWM,
	.of_match = hobot_pwm_ids,
	.ops = &hobot_pwm_ops,
	.probe = hobot_pwm_probe,
	.priv_auto = sizeof(struct hobot_pwm_priv),
};
