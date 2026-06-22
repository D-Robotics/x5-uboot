// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 *
 * X5 LPWM Backlight Driver (matches kernel driver behavior)
 */

#include <common.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <dt-bindings/pinctrl/horizon-pinfunc.h>
#include <dt-bindings/pinctrl/horizon-lsio-pinfunc.h>
#include "x5_backlight.h"
#include <hb_display_log.h>

#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
#define X5_BL_SEAMLESS_DBG(_fmt, ...)                                          \
	BL_LOG_DEBUG("seamless: " _fmt "\n", ##__VA_ARGS__)
#else
#define X5_BL_SEAMLESS_DBG(_fmt, ...) do { } while (0)
#endif

/* LSIO pinmux block (see X5_LSIO_IOMUXC_BASE in x5_backlight.h). */
#define LSIO_SYSCON_BASE ((uintptr_t)X5_LSIO_IOMUXC_BASE)
#define LPWM1_CH0_PINMUX_SHIFT 24
#define LPWM1_CH1_PINMUX_SHIFT 26
#define LPWM1_CH2_PINMUX_SHIFT 28
#define LPWM1_CH3_PINMUX_SHIFT 30

#define LPWM_PINMUX_MASK 0x3U
#define LPWM_PINMUX_ALT2 0x2U
#define LPWM_DS_MASK(_shift) (0xFU << (_shift))
#define LPWM_DS_SHIFT_CH0 1
#define LPWM_DS_SHIFT_CH1 9
#define LPWM_DS_SHIFT_CH2 17
#define LPWM_DS_SHIFT_CH3 25

#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
/* LPWM pinmux settings shared with kernel DTS. */
struct lpwm_pin_cfg {
	u32 pinmux_reg;
	u32 pinctrl_reg;
	u32 pinmux_shift;
	u32 ds_shift;
};
#endif

/*
 * LPWM Register Offsets (from kernel hobot_lpwm_hw_reg.c)
 */
#define LPWM_GLB_CFG 0x0000
#define LPWM_SW_TRIG 0x0004
#define LPWM_RST 0x0008
#define LPWM_CH_CFG0(ch) (0x0010 + (ch) * 12)
#define LPWM_CH_CFG1(ch) (0x0014 + (ch) * 12)
#define LPWM_CH_CFG2(ch) (0x0018 + (ch) * 12)

/*
 * LPWM_GLB_CFG bits (from kernel)
 *   bit 0-3: channel 0-3 enable
 *   bit 4: interrupt enable
 *   bit 5: mode select (0=software trigger, 1=external trigger)
 *   bit 8-11: trigger source select
 *   bit 12-21: div_ratio (10 bits)
 */
#define GLB_CFG_CH_EN(ch) BIT(ch)
#define GLB_CFG_INT_EN BIT(4)
#define GLB_CFG_MODE_SEL BIT(5)
#define GLB_CFG_TRIG_SRC_SHIFT 8
#define GLB_CFG_TRIG_SRC_MASK (0xF << 8)
#define GLB_CFG_DIV_SHIFT 12
#define GLB_CFG_DIV_MASK (0x3FF << 12)

/*
 * LPWM_CH_CFG1 bits
 *   bit 0-19: period (20 bits)
 *   bit 20-31: duty_time (12 bits)
 */
#define CFG1_PERIOD_MASK 0xFFFFF
#define CFG1_DUTY_SHIFT 20

#if !CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
static void lpwm_config_pinmux(int lpwm_id, int channel)
{
	void __iomem *pinmux_reg;
	u32 val, shift;

	if (lpwm_id != 1)
		return;

	switch (channel) {
	case 0:
		shift = LPWM1_CH0_PINMUX_SHIFT;
		break;
	case 1:
		shift = LPWM1_CH1_PINMUX_SHIFT;
		break;
	case 2:
		shift = LPWM1_CH2_PINMUX_SHIFT;
		break;
	case 3:
		shift = LPWM1_CH3_PINMUX_SHIFT;
		break;
	default:
		return;
	}

	pinmux_reg = (void __iomem *)(LSIO_SYSCON_BASE + LSIO_PINMUX_1);
	val = readl(pinmux_reg);
	val &= ~(LPWM_PINMUX_MASK << shift);
	val |= (LPWM_PINMUX_ALT2 << shift);
	writel(val, pinmux_reg);

	BL_LOG_DEBUG("Pinmux LPWM%d_CH%d configured (LSIO_PINMUX_1=0x%08x)\n",
		     lpwm_id, channel, readl(pinmux_reg));
}
#else
static int lpwm_get_pin_cfg(int lpwm_id, int channel, struct lpwm_pin_cfg *cfg)
{
	if (!cfg || channel < 0 || channel > 3)
		return -EINVAL;

	switch (lpwm_id) {
	case 0:
		cfg->pinmux_reg = LSIO_PINMUX_2;
		cfg->pinctrl_reg = LSIO_PINCTRL_5;
		cfg->pinmux_shift = BIT_OFFSET8 + channel * 2;
		break;
	case 1:
		cfg->pinmux_reg = LSIO_PINMUX_1;
		cfg->pinctrl_reg = LSIO_PINCTRL_7;
		cfg->pinmux_shift = BIT_OFFSET24 + channel * 2;
		break;
	default:
		return -EINVAL;
	}

	switch (channel) {
	case 0:
		cfg->ds_shift = LPWM_DS_SHIFT_CH0;
		break;
	case 1:
		cfg->ds_shift = LPWM_DS_SHIFT_CH1;
		break;
	case 2:
		cfg->ds_shift = LPWM_DS_SHIFT_CH2;
		break;
	case 3:
		cfg->ds_shift = LPWM_DS_SHIFT_CH3;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* Sync LPWM pinmux/drive-strength with the DTS defaults. */
static void lpwm_apply_kernel_pin_cfg(int lpwm_id, int channel)
{
	struct lpwm_pin_cfg cfg;
	void __iomem *iomux;
	void __iomem *mux_reg;
	void __iomem *pinctrl_reg;
	u32 mux_before, mux_after;
	u32 cfg_before, cfg_after;

	if (lpwm_get_pin_cfg(lpwm_id, channel, &cfg))
		return;

	iomux = (void __iomem *)(uintptr_t)LSIO_SYSCON_BASE;
	mux_reg = iomux + cfg.pinmux_reg;
	pinctrl_reg = iomux + cfg.pinctrl_reg;

	mux_before = readl(mux_reg);
	cfg_before = readl(pinctrl_reg);

	mux_after = mux_before;
	mux_after &= ~(LPWM_PINMUX_MASK << cfg.pinmux_shift);
	mux_after |= (MUX_ALT2 << cfg.pinmux_shift);

	cfg_after = cfg_before;
	cfg_after &= ~LPWM_DS_MASK(cfg.ds_shift);
	cfg_after |= (1U << cfg.ds_shift);

	if (mux_after != mux_before)
		writel(mux_after, mux_reg);
	if (cfg_after != cfg_before)
		writel(cfg_after, pinctrl_reg);

	X5_BL_SEAMLESS_DBG("lpwm%d ch%d pincfg mux+0x%x 0x%08x->0x%08x pinctrl+0x%x 0x%08x->0x%08x",
			   lpwm_id, channel, cfg.pinmux_reg, mux_before,
			   readl(mux_reg), cfg.pinctrl_reg, cfg_before,
			   readl(pinctrl_reg));
}
#endif

/*
 * Initialize LPWM
 */
static int lpwm_init(const struct x5_backlight_config *config)
{
	void __iomem *base;
	u32 glb_cfg, cfg1;
	int ch;

	if (!config || !config->pwm_base)
		return -EINVAL;

	base = (void __iomem *)config->pwm_base;
	ch = config->channel;

	BL_LOG_INFO("Initializing LPWM%d channel %d\n",
		   config->pwm_id, ch);
	BL_LOG_DEBUG("period=%u, duty=%u, div=%u\n",
		   config->period, config->duty, config->div_ratio);

#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
	lpwm_apply_kernel_pin_cfg(config->pwm_id, ch);
#else
	lpwm_config_pinmux(config->pwm_id, ch);
#endif

	writel(1, base + LPWM_RST);
	udelay(10);
	writel(0, base + LPWM_RST);

	/* Configure CFG2 (threshold=0, adjust_step=0) */
	writel(0, base + LPWM_CH_CFG2(ch));

	/* Configure GLB_CFG
	 * - div_ratio from config
	 * - mode_sel = 0 (software trigger)
	 * - trigger_source = 0 (matching kernel dump)
	 * - no channels enabled yet
	 */
	glb_cfg = (config->div_ratio << GLB_CFG_DIV_SHIFT) & GLB_CFG_DIV_MASK;
	writel(glb_cfg, base + LPWM_GLB_CFG);

	/* Configure CFG0 (offset = 0) */
	writel(0, base + LPWM_CH_CFG0(ch));

	cfg1 = (config->duty << CFG1_DUTY_SHIFT) | (config->period & CFG1_PERIOD_MASK);
	writel(cfg1, base + LPWM_CH_CFG1(ch));

	BL_LOG_INFO("Initialized (GLB_CFG=0x%08x, CFG1=0x%08x)\n",
		   readl(base + LPWM_GLB_CFG), readl(base + LPWM_CH_CFG1(ch)));

	return 0;
}

/*
 * Enable LPWM output
 */
static int lpwm_enable(const struct x5_backlight_config *config)
{
	void __iomem *base;
	u32 glb_cfg;
	int ch;

	if (!config || !config->pwm_base)
		return -EINVAL;

	base = (void __iomem *)config->pwm_base;
	ch = config->channel;

	glb_cfg = readl(base + LPWM_GLB_CFG);
	glb_cfg |= GLB_CFG_CH_EN(ch);
	writel(glb_cfg, base + LPWM_GLB_CFG);

	/* Software trigger (required for mode_sel=0) */
	writel(1, base + LPWM_SW_TRIG);

	BL_LOG_INFO("Enabled (GLB_CFG=0x%08x)\n", readl(base + LPWM_GLB_CFG));

	return 0;
}

/*
 * Disable LPWM output
 */
static int lpwm_disable(const struct x5_backlight_config *config)
{
	void __iomem *base;
	u32 glb_cfg;
	int ch;

	if (!config || !config->pwm_base)
		return -EINVAL;

	base = (void __iomem *)config->pwm_base;
	ch = config->channel;

	glb_cfg = readl(base + LPWM_GLB_CFG);
	glb_cfg &= ~GLB_CFG_CH_EN(ch);
	writel(glb_cfg, base + LPWM_GLB_CFG);

	BL_LOG_INFO("Disabled\n");

	return 0;
}

/*
 * Set LPWM brightness
 */
static int lpwm_set_brightness(const struct x5_backlight_config *config, int percent)
{
	void __iomem *base;
	u32 cfg1, duty;
	int ch;

	if (!config || !config->pwm_base)
		return -EINVAL;

	base = (void __iomem *)config->pwm_base;
	ch = config->channel;

	duty = config->period * percent / 100;

	cfg1 = (duty << CFG1_DUTY_SHIFT) | (config->period & CFG1_PERIOD_MASK);
	writel(cfg1, base + LPWM_CH_CFG1(ch));

	writel(1, base + LPWM_SW_TRIG);

	BL_LOG_INFO("Brightness set to %d%% (duty=%u)\n", percent, duty);

	return 0;
}

/*
 * Dump LPWM registers
 */
static void lpwm_dump(const struct x5_backlight_config *config)
{
	void __iomem *base;
	int i;
	u32 glb_cfg, pinmux, pinctrl;
	u32 pinmux_reg, pinctrl_reg, pinmux_base_shift;

	if (!config || !config->pwm_base)
		return;

	base = (void __iomem *)config->pwm_base;
	pinmux_reg = config->pwm_id == 0 ? LSIO_PINMUX_2 : LSIO_PINMUX_1;
	pinctrl_reg = config->pwm_id == 0 ? LSIO_PINCTRL_5 : LSIO_PINCTRL_7;
	pinmux_base_shift = config->pwm_id == 0 ? BIT_OFFSET8 : BIT_OFFSET24;

	BL_LOG_DEBUG("\n=== Backlight LPWM%d Registers (0x%08lx) ===\n",
		   config->pwm_id, config->pwm_base);

	pinmux = readl((void __iomem *)(uintptr_t)(LSIO_SYSCON_BASE + pinmux_reg));
	pinctrl = readl((void __iomem *)(uintptr_t)(LSIO_SYSCON_BASE + pinctrl_reg));
	BL_LOG_DEBUG("PINMUX(reg+0x%x @0x%08lx): 0x%08x\n",
		   pinmux_reg, (unsigned long)(LSIO_SYSCON_BASE + pinmux_reg), pinmux);
	BL_LOG_DEBUG("PINCTRL(reg+0x%x @0x%08lx): 0x%08x\n",
		   pinctrl_reg, (unsigned long)(LSIO_SYSCON_BASE + pinctrl_reg), pinctrl);
	for (i = 0; i < 4; i++) {
		u32 mux_val = (pinmux >> (pinmux_base_shift + i * 2)) & LPWM_PINMUX_MASK;
		u32 ds_shift = LPWM_DS_SHIFT_CH0 + i * 8;
		u32 ds_val = (pinctrl >> ds_shift) & 0xF;

		BL_LOG_DEBUG("  CH%d mux=%u%s ds=%u\n", i, mux_val,
			   mux_val == MUX_ALT2 ? "(LPWM)" : "", ds_val);
	}

	glb_cfg = readl(base + LPWM_GLB_CFG);
	BL_LOG_DEBUG("GLB_CFG  (0x00): 0x%08x\n", glb_cfg);
	BL_LOG_DEBUG("  div_ratio: %u\n", (glb_cfg >> GLB_CFG_DIV_SHIFT) & 0x3FF);
	BL_LOG_DEBUG("  trig_src:  %u\n", (glb_cfg >> GLB_CFG_TRIG_SRC_SHIFT) & 0xF);
	BL_LOG_DEBUG("  mode_sel:  %u (%s)\n", (glb_cfg >> 5) & 1,
		   (glb_cfg & GLB_CFG_MODE_SEL) ? "external" : "software");
	for (i = 0; i < 4; i++)
		BL_LOG_DEBUG("  ch%d_en:    %u\n", i, (glb_cfg >> i) & 1);

	BL_LOG_DEBUG("SW_TRIG  (0x04): 0x%08x\n", readl(base + LPWM_SW_TRIG));
	BL_LOG_DEBUG("RST      (0x08): 0x%08x\n", readl(base + LPWM_RST));

	for (i = 0; i < 4; i++) {
		u32 cfg1 = readl(base + LPWM_CH_CFG1(i));
		BL_LOG_DEBUG("CH%d_CFG0 (0x%02x): 0x%08x (offset)\n", i, 0x10 + i * 12,
			   readl(base + LPWM_CH_CFG0(i)));
		BL_LOG_DEBUG("CH%d_CFG1 (0x%02x): 0x%08x (period=%u, duty=%u)\n",
			   i, 0x14 + i * 12, cfg1,
			   cfg1 & CFG1_PERIOD_MASK, cfg1 >> CFG1_DUTY_SHIFT);
		BL_LOG_DEBUG("CH%d_CFG2 (0x%02x): 0x%08x\n", i, 0x18 + i * 12,
			   readl(base + LPWM_CH_CFG2(i)));
	}

	BL_LOG_DEBUG("==========================================\n\n");
}

/*
 * LPWM operations
 */
static const struct x5_backlight_ops lpwm_ops = {
	.init = lpwm_init,
	.enable = lpwm_enable,
	.disable = lpwm_disable,
	.set_brightness = lpwm_set_brightness,
	.dump = lpwm_dump,
};

/*
 * LPWM driver registration
 */
const struct x5_backlight_driver x5_backlight_lpwm_driver = {
	.name = "lpwm",
	.type = X5_PWM_TYPE_LPWM,
	.ops = &lpwm_ops,
};
