// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 *
 * X5 Standard PWM Backlight Driver (ref: kernel pwm-drobot.c)
 */

#include <common.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <dt-bindings/pinctrl/horizon-pinfunc.h>
#include <dt-bindings/pinctrl/horizon-lsio-pinfunc.h>
#include "x5_backlight.h"
#include <hb_display_log.h>
#include "../clock/x5_crm.h"

#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
#define X5_BL_SEAMLESS_DBG(_fmt, ...)                                          \
	BL_LOG_DEBUG("seamless: " _fmt "\n", ##__VA_ARGS__)
#else
#define X5_BL_SEAMLESS_DBG(_fmt, ...) do { } while (0)
#endif

/*
 * Standard PWM Register Offsets
 */
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
#define PWM_CCR_DIV_SHIFT       4
#define PWM_CCR_PRE_SHIFT       8

/* Constants */
#define PWM_FIFO_DEPTH          4
/* Period register is 16-bit; prescale up to 256 (see pwm_calc_params). */
#define PWM_PRD_MAX             0xffffU
#define PWM_PRESCALE_MAX        256U
#define PWM_PRD_PRESCALE_MAX    ((u32)PWM_PRD_MAX * PWM_PRESCALE_MAX)

#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
#define LSIO_IOMUX_BASE         X5_LSIO_IOMUXC_BASE
#define PWM2_DS_SHIFT_SCL       9
#define PWM2_DS_SHIFT_SDA       1
#define PWM2_DS_MASK(_shift)    (0xFU << (_shift))
#define PWM2_POWER_SEL_BIT      BIT(31)

/* Sync PWM2 pinmux/pinconf with the DTS defaults. */
static void pwm2_apply_kernel_pin_cfg(void)
{
	void __iomem *iomux = (void __iomem *)LSIO_IOMUX_BASE;
	u32 mux_before, mux_after;
	u32 cfg_before, cfg_after;

	mux_before = readl(iomux + LSIO_PINMUX_2);
	cfg_before = readl(iomux + LSIO_PINCTRL_2);

	mux_after = mux_before;
	mux_after &= ~((0x3U << BIT_OFFSET16) | (0x3U << BIT_OFFSET18));
	mux_after |= (MUX_ALT3 << BIT_OFFSET16) | (MUX_ALT3 << BIT_OFFSET18);

	cfg_after = cfg_before;
	cfg_after &= ~(PWM2_DS_MASK(PWM2_DS_SHIFT_SCL) |
		       PWM2_DS_MASK(PWM2_DS_SHIFT_SDA));
	cfg_after |= (1U << PWM2_DS_SHIFT_SCL) | (1U << PWM2_DS_SHIFT_SDA);
	cfg_after &= ~PWM2_POWER_SEL_BIT;

	if (mux_after != mux_before)
		writel(mux_after, iomux + LSIO_PINMUX_2);
	if (cfg_after != cfg_before)
		writel(cfg_after, iomux + LSIO_PINCTRL_2);

	X5_BL_SEAMLESS_DBG(
		"pwm2 pincfg mux2 0x%08x->0x%08x pinctrl2 0x%08x->0x%08x",
		mux_before, readl(iomux + LSIO_PINMUX_2),
		cfg_before, readl(iomux + LSIO_PINCTRL_2));
}

static void pwm_log_snapshot(void __iomem *base, int ch, const char *tag)
{
	u32 mcr, ctr, ccr, pr, pw16;

	mcr = readl(base + PWM_MCR);
	ctr = readl(base + PWM_CTR(ch));
	ccr = readl(base + PWM_CCR(ch));
	pr = readl(base + PWM_PR(ch));
	pw16 = readl(base + PWM_PW16AR(ch));

	X5_BL_SEAMLESS_DBG("%s ch%d MCR=0x%08x CTR=0x%08x CCR=0x%08x PR=0x%08x PW16=0x%08x",
			   tag, ch, mcr, ctr, ccr, pr, pw16);
}
#endif

/* PWM pclk comes from top_apb_clk. */
static u32 pwm_get_apb_clock(void)
{
	return TOP_APB_CLK_RATE;  /* 200 MHz from x5_crm.h */
}

/*
 * Calculate PWM parameters
 * PWM frequency = clk_rate / (2^(div_reg+1) * prescale * (prd+1))
 */
static int pwm_calc_params(u32 period_ns, u8 *div_reg, u16 *prescale, u16 *prd)
{
	u64 period_cycles;
	u32 prd_prescale;
	u8 div = 0;
	u16 pre;
	u32 apb_clk;

	apb_clk = pwm_get_apb_clock();

	period_cycles = (u64)apb_clk * period_ns / 1000000000ULL;

	while (div <= 7) {
		prd_prescale = period_cycles >> (div + 1);
		if (prd_prescale <= PWM_PRD_PRESCALE_MAX)
			break;
		div++;
	}

	pre = prd_prescale / PWM_PRD_MAX;
	if (prd_prescale % PWM_PRD_MAX)
		pre++;

	if (pre > PWM_PRESCALE_MAX) {
		BL_LOG_DEBUG("PWM: prescale overflow, clamping to %u\n",
			     PWM_PRESCALE_MAX);
		pre = PWM_PRESCALE_MAX;
	}

	*div_reg = div;
	*prescale = pre;
	*prd = (prd_prescale / pre) - 1;

	return 0;
}

/*
 * Set PWM duty cycle
 */
static void pwm_set_duty(void __iomem *base, uint32_t channel,
                         u32 duty_ns, u8 div_reg, u16 prescale)
{
	u64 duty_cycles;
	u16 duty;
	int i;
	u32 apb_clk;

	apb_clk = pwm_get_apb_clock();

	duty_cycles = (u64)apb_clk * duty_ns / 1000000000ULL;

	duty = (duty_cycles >> (div_reg + 1)) / prescale;

	if (duty == 0)
		BL_LOG_DEBUG("PWM: Warning - duty is 0, no waveform output\n");

	for (i = 0; i < PWM_FIFO_DEPTH; i++)
		writel(duty, base + PWM_PW16AR(channel));

	X5_BL_SEAMLESS_DBG("set_duty ch%u duty_ns=%u duty_reg=%u div=%u pre=%u",
			   channel, duty_ns, duty, div_reg, prescale);
}

/*
 * Initialize standard PWM
 */
static int pwm_init(const struct x5_backlight_config *config)
{
	void __iomem *base;
	u8 div_reg;
	u16 prescale, prd;
	u32 val;
	int ch, ret;

	if (!config || !config->pwm_base)
		return -EINVAL;

	base = (void __iomem *)config->pwm_base;
	ch = config->channel;

	if (ch >= 2) {
		BL_LOG_ERROR("PWM: Invalid channel %d (max 1)\n", ch);
		return -EINVAL;
	}

	BL_LOG_INFO("Initializing PWM%d channel %d\n",
		   config->pwm_id, ch);
	BL_LOG_DEBUG("period=%uns, duty=%uns\n",
		   config->period_ns, config->duty_ns);
	X5_BL_SEAMLESS_DBG("init cfg pwm_id=%d ch=%d base=0x%08lx period_ns=%u duty_ns=%u",
			   config->pwm_id, ch, config->pwm_base,
			   config->period_ns, config->duty_ns);

#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
	if (config->pwm_id == 2)
		pwm2_apply_kernel_pin_cfg();
#endif

	ret = pwm_calc_params(config->period_ns, &div_reg, &prescale, &prd);
	if (ret)
		return ret;

	val = PWM_CLK_APB | (div_reg << PWM_CCR_DIV_SHIFT) |
	      ((prescale - 1) << PWM_CCR_PRE_SHIFT);
	writel(val, base + PWM_CCR(ch));

	writel(prd, base + PWM_PR(ch));

	if (config->duty_ns)
		pwm_set_duty(base, ch, config->duty_ns, div_reg, prescale);

	BL_LOG_INFO("Initialized (div=%d, pre=%d, prd=%d)\n",
		   div_reg, prescale, prd);

#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
	pwm_log_snapshot(base, ch, "after_init");
#endif

	return 0;
}

/*
 * Enable PWM output
 */
static int pwm_enable(const struct x5_backlight_config *config)
{
	void __iomem *base;
	u32 val;
	int ch;

	if (!config || !config->pwm_base)
		return -EINVAL;

	base = (void __iomem *)config->pwm_base;
	ch = config->channel;

#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
	pwm_log_snapshot(base, ch, "before_enable");
#endif

	val = readl(base + PWM_MCR);
	if (ch == 0)
		val |= PWM_CHANNEL0_EN;
	else
		val |= PWM_CHANNEL1_EN;
	writel(val, base + PWM_MCR);

	BL_LOG_INFO("Enabled (MCR=0x%08x)\n", readl(base + PWM_MCR));

#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
	pwm_log_snapshot(base, ch, "after_enable");
#endif

	return 0;
}

/*
 * Disable PWM output
 */
static int pwm_disable(const struct x5_backlight_config *config)
{
	void __iomem *base;
	u32 val;
	int ch;

	if (!config || !config->pwm_base)
		return -EINVAL;

	base = (void __iomem *)config->pwm_base;
	ch = config->channel;

	val = readl(base + PWM_MCR);
	if (ch == 0)
		val &= ~PWM_CHANNEL0_EN;
	else
		val &= ~PWM_CHANNEL1_EN;
	writel(val, base + PWM_MCR);

	BL_LOG_INFO("Disabled\n");

	return 0;
}

/*
 * Set PWM brightness
 */
static int pwm_set_brightness(const struct x5_backlight_config *config, int percent)
{
	void __iomem *base;
	u32 duty_ns;
	u8 div_reg;
	u16 prescale, prd;
	int ch;
	int ret;

	if (!config || !config->pwm_base)
		return -EINVAL;

	if (percent < 0 || percent > 100)
		return -EINVAL;

	base = (void __iomem *)config->pwm_base;
	ch = config->channel;

	duty_ns = config->period_ns * percent / 100;

	/* Recalculate parameters (to get div_reg and prescale) */
	ret = pwm_calc_params(config->period_ns, &div_reg, &prescale, &prd);
	if (ret)
		return ret;

	/* Update duty cycle only */
	pwm_set_duty(base, ch, duty_ns, div_reg, prescale);

	BL_LOG_INFO("Brightness set to %d%% (duty=%uns)\n", percent, duty_ns);

	return 0;
}

/*
 * Dump PWM registers
 */
static void pwm_dump(const struct x5_backlight_config *config)
{
	void __iomem *base;
	int ch;
	int i;
	u32 mcr, ccr, pr, ctr;

	if (!config || !config->pwm_base)
		return;

	base = (void __iomem *)config->pwm_base;
	ch = config->channel;

	BL_LOG_DEBUG("\n=== Backlight PWM%d Registers (0x%08lx) ===\n",
		   config->pwm_id, config->pwm_base);

	mcr = readl(base + PWM_MCR);
	BL_LOG_DEBUG("MCR (0x00): 0x%08x\n", mcr);
	BL_LOG_DEBUG("  CH0_EN: %u\n", (mcr & PWM_CHANNEL0_EN) ? 1 : 0);
	BL_LOG_DEBUG("  CH1_EN: %u\n", (mcr & PWM_CHANNEL1_EN) ? 1 : 0);

	BL_LOG_DEBUG("ISR (0x04): 0x%08x\n", readl(base + PWM_ISR));

	for (i = 0; i < 2; i++) {
		ctr = readl(base + PWM_CTR(i));
		ccr = readl(base + PWM_CCR(i));
		pr = readl(base + PWM_PR(i));

		BL_LOG_DEBUG("\nChannel %d:\n", i);
		BL_LOG_DEBUG("  CTR (0x%02x): 0x%08x (polarity=%u)\n",
			   0x10 + i * 0x20, ctr,
			   (unsigned int)(ctr & PWM_POLARITY));
		BL_LOG_DEBUG("  CCR (0x%02x): 0x%08x (clk=%u, div=%u, pre=%u)\n",
			   0x14 + i * 0x20, ccr,
			   ccr & 0x1, (ccr >> PWM_CCR_DIV_SHIFT) & 0xF,
			   (ccr >> PWM_CCR_PRE_SHIFT) & 0xFF);
		BL_LOG_DEBUG("  PR  (0x%02x): 0x%08x (period=%u)\n",
			   0x20 + i * 0x20, pr, pr);
		BL_LOG_DEBUG("  CR  (0x%02x): 0x%08x\n",
			   0x24 + i * 0x20, readl(base + PWM_CR(i)));
		BL_LOG_DEBUG("  SR  (0x%02x): 0x%08x\n",
			   0x28 + i * 0x20, readl(base + PWM_SR(i)));
	}

	BL_LOG_DEBUG("==========================================\n\n");
}

/*
 * Standard PWM operations
 */
static const struct x5_backlight_ops pwm_ops = {
	.init = pwm_init,
	.enable = pwm_enable,
	.disable = pwm_disable,
	.set_brightness = pwm_set_brightness,
	.dump = pwm_dump,
};

/*
 * Standard PWM driver registration
 */
const struct x5_backlight_driver x5_backlight_pwm_driver = {
	.name = "pwm",
	.type = X5_PWM_TYPE_STD,
	.ops = &pwm_ops,
};
