// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 * Modified by: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * X5 CRM (Clock and Reset Manager) for Display Subsystem
 *
 * This module configures display-related clocks only.
 * It does NOT modify non-display clocks or system-wide settings.
 */

#include <common.h>
#include <div64.h>
#include <errno.h>
#include <asm/io.h>
#include <linux/delay.h>
#include "x5_crm.h"
#include <hb_display_log.h>

/* Maximum divider values */
#define GEN_PRE_DIV_MAX 8
#define GEN_POST_DIV_MAX 64
#define GEN_DIV_MAX (GEN_PRE_DIV_MAX * GEN_POST_DIV_MAX)

/*
 * Last DISP_PLL output frequency (Hz) programmed for the display path.
 * Written once during x5_crm_display_*_init / BT1120 init before pixel/AXI
 * clock generators read it. Range: entries from pll_rate_table / pll_calc_params
 * (roughly tens of MHz .. ~1.8 GHz). U-Boot runs single-threaded pre-boot;
 * no concurrent writers — do not use from IRQ or secondary CPUs without locking.
 */
static unsigned long g_disp_pll_rate;

/* API sanity limits (Hz): pll_rate_table max ~1.8GHz; pixel clocks stay << 1GHz. */
#define X5_CRM_PIXEL_CLOCK_MAX_HZ 1000000000UL
#define X5_CRM_DISP_PLL_MAX_HZ    2000000000UL

/*
 * PLL Rate Entry
 */
struct pll_rate_entry
{
	unsigned long rate;
	u16 prediv;
	u16 mint;
	u16 mfrac;
	u16 postdivp;
	u16 divvcop;
	u16 postdivr;
	u16 divvcor;
};

/* PLL constraints (from kernel clk-snps-pll.h) */
#define PLL_REF_CLK_HZ 24000000UL /* 24 MHz reference clock */
#define PLL_MINT_MIN 16
#define PLL_MINT_MAX 625
#define PLL_VCO_MIN_HZ 2500000000ULL /* 2.5 GHz (MINVCO1) */
#define PLL_VCO_MAX_HZ 3750000000ULL /* 3.75 GHz (MAXVCO1, lowfreq mode) */
/* pll_set_regs Fvco bands vs PLL_LOWFREQ_MASK (clk-snps): above MAXVCO1 use mid band */
#define PLL_VCO_HIGHFREQ_THRESHOLD 4500000000ULL

/* Valid divvco values */
static const u16 divvco_values[] = {2, 4, 8, 16, 32, 64};

/*
 * Pre-defined PLL rate table from kernel (clk-x5.c)
 * Format: { rate, prediv, mint, mfrac, postdivp, divvcop, postdivr, divvcor }
 */
static const struct pll_rate_entry pll_rate_table[] = {
	{1800000000, 1, 0x96, 0x0, 1, 2, 1, 2},
	{1631872000, 1, 0x87, 0xFD45, 1, 2, 1, 2},
	{1622250000, 1, 0x87, 0x3000, 1, 2, 3, 2},
	{1620000000, 1, 0x87, 0, 1, 2, 3, 2},
	{1500000000, 1, 0x7D, 0, 1, 2, 2, 2},
	{1483500000, 1, 0x7B, 0xA000, 1, 2, 1, 2},
	{1347750000, 1, 0x70, 0x5000, 1, 2, 1, 2},
	{1200000000, 1, 0xC8, 0, 1, 4, 1, 4},
	{1050000000, 1, 0xAF, 0, 1, 4, 1, 4},
	{996000000, 1, 0xA6, 0, 1, 4, 2, 4},
	{339937500, 1, 0x71, 0x5000, 4, 2, 2, 2},
	{312375000, 1, 0x9C, 0x3000, 6, 2, 2, 2},
	{297000000, 1, 0xC6, 0, 2, 8, 2, 8},
	{251750000, 1, 0x7D, 0xE000, 6, 2, 2, 2},
	{65000000, 1, 0x82, 0, 6, 8, 6, 8},
};

/*
 * Convert divvco value to register value
 */
static u32 pll_get_divvco_reg(unsigned int value)
{
	switch (value)
	{
	case 2:
		return 0x0;
	case 4:
		return 0x8;
	case 8:
		return 0xc;
	case 16:
		return 0xd;
	case 32:
		return 0xe;
	case 64:
		return 0xf;
	default:
		return 0xff;
	}
}

/*
 * Look up PLL parameters from pre-defined rate table
 */
static bool pll_lookup_rate_table(unsigned long target_rate, struct pll_rate_entry *entry)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pll_rate_table); i++)
	{
		if (pll_rate_table[i].rate == target_rate)
		{
			memcpy(entry, &pll_rate_table[i], sizeof(*entry));
			return true;
		}
	}
	return false;
}

/*
 * Dynamically calculate PLL parameters for any target frequency
 */
static bool pll_calc_params(unsigned long target_rate, struct pll_rate_entry *entry)
{
	unsigned int postdivp, divvcop_idx;
	unsigned long best_error = ULONG_MAX;
	bool found = false;

	/* First try pre-defined table */
	if (pll_lookup_rate_table(target_rate, entry))
	{
		CRM_LOG_DEBUG("Found PLL rate in table: %lu Hz\n", target_rate);
		return true;
	}

	CRM_LOG_DEBUG("Calculating PLL params for %lu Hz\n", target_rate);

	memset(entry, 0, sizeof(*entry));
	entry->prediv = 1;
	entry->postdivr = 1;
	entry->divvcor = 2;

	for (postdivp = 1; postdivp <= 8; postdivp++)
	{
		for (divvcop_idx = 0; divvcop_idx < ARRAY_SIZE(divvco_values); divvcop_idx++)
		{
			unsigned int divvcop = divvco_values[divvcop_idx];
			unsigned long long mint_calc;
			unsigned int mint;
			unsigned long long vco;
			unsigned long actual_rate;
			unsigned long error;

			mint_calc = (unsigned long long)target_rate * entry->prediv * postdivp * divvcop;
			mint_calc = DIV_ROUND_CLOSEST_ULL(mint_calc, PLL_REF_CLK_HZ);
			mint = (unsigned int)mint_calc;

			if (mint < PLL_MINT_MIN || mint > PLL_MINT_MAX)
				continue;

			vco = (unsigned long long)PLL_REF_CLK_HZ * mint / entry->prediv;
			if (vco < PLL_VCO_MIN_HZ || vco > PLL_VCO_MAX_HZ)
				continue;

			actual_rate = vco / postdivp / divvcop;

			error = (actual_rate > target_rate) ? (actual_rate - target_rate) : (target_rate - actual_rate);

			if (error < best_error)
			{
				best_error = error;
				entry->rate = actual_rate;
				entry->mint = mint;
				entry->postdivp = postdivp;
				entry->divvcop = divvcop;
				entry->divvcor = divvcop;
				entry->postdivr = postdivp;
				found = true;

				if (error == 0)
					return true;
			}
		}
	}

	return found;
}

/*
 * Wait for PLL to lock
 */
static int pll_wait_lock(ulong cfg_reg)
{
	int i;
	u32 val;

	for (i = 0; i < PLL_LOCK_CHECK_COUNT; i++)
	{
		val = readl((void __iomem *)(ulong)(cfg_reg + PLL_LOCK_OFFSET));
		if (val & PLL_LOCK)
			return 0;
		udelay(PLL_LOCK_DELAY_US);
	}

	/* Only print error on timeout */
	CRM_LOG_ERROR("PLL lock timeout\n");
	return -ETIMEDOUT;
}

/*
 * Configure PLL registers
 */
static int pll_set_regs(ulong cfg_reg, ulong internal_reg,
						struct pll_rate_entry *rate_entry)
{
	u32 cfg_val, fbdiv_val, postdiv_val;
	u64 fvco;
	int ret, retry;

	fvco = (u64)PLL_REF_CLK_HZ * rate_entry->mint;
	if (rate_entry->mfrac)
		fvco += ((u64)PLL_REF_CLK_HZ * rate_entry->mfrac) >> 16;

	cfg_val = PLL_CTRL_CFG0;
	if (fvco <= PLL_VCO_MAX_HZ)
		cfg_val |= PLL_LOWFREQ_MASK;
	else if (fvco <= PLL_VCO_HIGHFREQ_THRESHOLD)
		cfg_val &= ~PLL_LOWFREQ_MASK;
	else
	{
		cfg_val &= ~PLL_VCOMODE_MASK;
		cfg_val |= PLL_LOWFREQ_MASK;
	}
	cfg_val &= ~PLL_PREDIV_MASK;
	cfg_val |= SET_PLL_PREDIV(rate_entry->prediv - 1);

	fbdiv_val = SET_PLL_MINT(rate_entry->mint - 16);
	fbdiv_val |= (rate_entry->mfrac & PLL_MFRAC_MASK);

	postdiv_val = PLL_EN;
	postdiv_val |= SET_PLL_DIVVCOP(pll_get_divvco_reg(rate_entry->divvcop));
	postdiv_val |= SET_PLL_POSTDIVP(rate_entry->postdivp - 1);
	postdiv_val |= SET_PLL_DIVVCOR(pll_get_divvco_reg(rate_entry->divvcor));
	postdiv_val |= SET_PLL_POSTDIVR(rate_entry->postdivr - 1);

	for (retry = 0; retry < PLL_LOCK_RETRY; retry++)
	{
		writel(readl((void __iomem *)(ulong)(cfg_reg + PLL_POSTDIV_OFFSET)) &
				   ~(PLL_P_OUT | PLL_R_OUT),
			   (void __iomem *)(ulong)(cfg_reg + PLL_POSTDIV_OFFSET));

		writel(PLL_CTRL_DEFAULT, (void __iomem *)(ulong)cfg_reg);
		writel(fbdiv_val, (void __iomem *)(ulong)(cfg_reg + PLL_FBDIV_OFFSET));
		writel(cfg_val, (void __iomem *)(ulong)cfg_reg);

		if (rate_entry->mfrac)
			writel(PLL_ANAREG6_EN, (void __iomem *)(ulong)(internal_reg + PLL_ANAREG6));

		writel(postdiv_val, (void __iomem *)(ulong)(cfg_reg + PLL_POSTDIV_OFFSET));
		writel(fbdiv_val | PLL_FBDIV_LOAD,
			   (void __iomem *)(ulong)(cfg_reg + PLL_FBDIV_OFFSET));

		if (rate_entry->mfrac)
			writel(PLL_FRAC_EN, (void __iomem *)(ulong)(internal_reg + PLL_SCFRAC_CNT));

		writel(cfg_val | PLL_PWRON, (void __iomem *)(ulong)cfg_reg);

		ret = pll_wait_lock(cfg_reg);
		if (ret == 0)
			break;
	}

	if (ret)
	{
		CRM_LOG_ERROR("PLL failed to lock after %d retries\n", PLL_LOCK_RETRY);
		return ret;
	}

	writel(readl((void __iomem *)(ulong)(cfg_reg + PLL_POSTDIV_OFFSET)) | PLL_P_OUT | PLL_R_OUT,
		   (void __iomem *)(ulong)(cfg_reg + PLL_POSTDIV_OFFSET));

	return 0;
}

/*
 * Find a PLL rate from the pre-defined table that is an exact integer
 * multiple of the requested pixel clock.  Prefer the smallest multiplier
 * (closest PLL rate) to minimise jitter and power.
 *
 * Kernel clk-x5.c expects DISP_PLL at a table-defined rate (default
 * 297 MHz).  If we set the PLL to 297 MHz, the kernel can later derive
 * 27 MHz (its bt1120_pixel_clk default) with divider 11, which avoids
 * CLK_SET_RATE_PARENT reprogramming the PLL and the resulting bus stall
 * that garbles the UART.
 */
static unsigned long find_best_pll_rate(unsigned long pixel_clock)
{
	int i;
	unsigned long best_rate = 0;
	unsigned int best_div = GEN_DIV_MAX + 1;

	for (i = 0; i < ARRAY_SIZE(pll_rate_table); i++) {
		unsigned long rate = pll_rate_table[i].rate;
		unsigned int div;

		if (rate < pixel_clock)
			continue;
		div = rate / pixel_clock;
		if (div > GEN_DIV_MAX || div == 0)
			continue;
		if (div * pixel_clock != rate)
			continue;
		if (div < best_div) {
			best_rate = rate;
			best_div = div;
		}
	}

	return best_rate ? best_rate : pixel_clock;
}

/*
 * Initialize DISP_PLL
 */
int x5_crm_disp_pll_init(unsigned long rate)
{
	struct pll_rate_entry entry;
	int ret;

	if (rate == 0 || rate > X5_CRM_DISP_PLL_MAX_HZ) {
		CRM_LOG_ERROR("invalid DISP_PLL rate %lu\n", rate);
		return -EINVAL;
	}

	if (!pll_calc_params(rate, &entry))
	{
		CRM_LOG_ERROR("Cannot calculate PLL params for %lu Hz\n", rate);
		return -EINVAL;
	}

	CRM_LOG_INFO("Configuring DISP_PLL: %lu Hz\n", entry.rate);

	ret = pll_set_regs(DISP_PLL_CFG, DISP_PLL_INTERNAL, &entry);
	if (ret) {
		CRM_LOG_ERROR("pll_set_regs failed: %d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * Dump PLL status for debugging
 */
void x5_crm_dump_pll_status(void)
{
	u32 cfg, postdiv, lock;

	CRM_LOG_DEBUG("\n=== PLL Status ===\n");

	cfg = readl((void __iomem *)DISP_PLL_CFG);
	postdiv = readl((void __iomem *)(DISP_PLL_CFG + PLL_POSTDIV_OFFSET));
	lock = readl((void __iomem *)(DISP_PLL_CFG + PLL_LOCK_OFFSET));
	CRM_LOG_DEBUG("DISP_PLL: cfg=0x%08x, postdiv=0x%08x, lock=%s\n",
		   cfg, postdiv, (lock & PLL_LOCK) ? "YES" : "NO");

	CRM_LOG_DEBUG("==================\n\n");
}

/*
 * Calculate divider table from total divider value
 */
static bool calc_div_table(unsigned int total_div, u8 *prediv, u8 *postdiv)
{
	unsigned int i, j;

	if (total_div == 0)
		return false;

	if (total_div <= GEN_PRE_DIV_MAX)
	{
		*prediv = total_div - 1;
		*postdiv = 0;
		return true;
	}

	if (total_div >= GEN_DIV_MAX)
	{
		*prediv = GEN_PRE_DIV_MAX - 1;
		*postdiv = GEN_POST_DIV_MAX - 1;
		return true;
	}

	for (i = 1; i <= GEN_PRE_DIV_MAX; i++)
	{
		for (j = 1; j <= GEN_POST_DIV_MAX; j++)
		{
			if (i * j == total_div)
			{
				*prediv = i - 1;
				*postdiv = j - 1;
				return true;
			}
		}
	}

	for (i = 1; i <= GEN_PRE_DIV_MAX; i++)
	{
		for (j = 1; j <= GEN_POST_DIV_MAX; j++)
		{
			if (i * j >= total_div)
			{
				*prediv = i - 1;
				*postdiv = j - 1;
				return true;
			}
		}
	}

	return false;
}

/*
 * Configure a clock generator
 * On success returns programmed rate in Hz; on error returns a negative errno.
 */
static long clk_gen_set_rate(ulong reg_addr, unsigned long parent_rate,
									  unsigned long target_rate, u8 mux)
{
	u32 val;
	u8 prediv, postdiv;
	unsigned int div;
	unsigned long actual_rate;

	if (target_rate == 0 || parent_rate == 0)
		return -EINVAL;

	div = DIV_ROUND_CLOSEST(parent_rate, target_rate);
	if (div == 0)
		div = 1;

	if (!calc_div_table(div, &prediv, &postdiv))
	{
		CRM_LOG_ERROR("Failed to calculate divider for rate %lu\n", target_rate);
		return -EIO;
	}

	actual_rate = parent_rate / ((prediv + 1) * (postdiv + 1));

	val = readl((void __iomem *)(ulong)reg_addr);
	val &= ~(GEN_MUX_MASK | GEN_PRE_DIV_MASK | GEN_POST_DIV_MASK);
	val |= SET_GEN_MUX(mux);
	val |= SET_GEN_PREDIV(prediv);
	val |= SET_GEN_POSTDIV(postdiv);

	writel(val, (void __iomem *)(ulong)reg_addr);

	return (long)actual_rate;
}

/*
 * Enable a clock generator
 */
static void clk_gen_enable(ulong reg_addr)
{
	u32 val = readl((void __iomem *)(ulong)reg_addr);
	val |= GEN_EN;
	writel(val, (void __iomem *)(ulong)reg_addr);
}

/*
 * Disable a clock generator
 */
static void clk_gen_disable(ulong reg_addr)
{
	u32 val = readl((void __iomem *)(ulong)reg_addr);
	val &= ~GEN_EN;
	writel(val, (void __iomem *)(ulong)reg_addr);
}

/*
 * Enable display APB/AXI gate clocks
 * Only enables the clocks we need, preserves other bits
 */
static void enable_display_apb_clocks(void)
{
	u32 val = readl((void __iomem *)HPS_MIX_CLK_ENB);

	val |= CLK_EN_DISP_DSI_PCLK; /* DSI APB clock */
	val |= CLK_EN_DC8000_PCLK;	 /* DC8000 APB clock */
	val |= CLK_EN_DC8000_ACLK;	 /* DC8000 AXI clock gate */
	val |= CLK_EN_DPHY_CFG;		 /* DPHY config clock (24MHz) */
	val |= CLK_EN_DISP_SIF_ACLK; /* Display SIF AXI clock gate */
	val |= CLK_EN_DISP_SIF_PCLK; /* Display SIF APB clock */

	writel(val, (void __iomem *)HPS_MIX_CLK_ENB);
}

/*
 * Disable display APB/AXI gate clocks
 */
static void disable_display_apb_clocks(void)
{
	u32 val = readl((void __iomem *)HPS_MIX_CLK_ENB);

	val &= ~CLK_EN_DISP_DSI_PCLK;
	val &= ~CLK_EN_DC8000_PCLK;
	val &= ~CLK_EN_DC8000_ACLK;
	val &= ~CLK_EN_DPHY_CFG;
	val &= ~CLK_EN_DISP_SIF_ACLK;
	val &= ~CLK_EN_DISP_SIF_PCLK;

	writel(val, (void __iomem *)HPS_MIX_CLK_ENB);
}

/*
 * Release NOC idle for display modules
 * Only releases display-related bits, preserves others
 */
static int release_display_noc_idle(void)
{
	u32 val, status;
	int timeout = NOC_IDLE_TIMEOUT_US / NOC_IDLE_POLL_US;
	u32 display_mask = NOC_IDLE_DC8000 | NOC_IDLE_DISP_SIF;

	/* Clear idle request for display modules */
	val = readl((void __iomem *)NOC_IDLE_REQ_REG0);
	val &= ~display_mask;
	writel(val, (void __iomem *)NOC_IDLE_REQ_REG0);

	/* Wait for idle status to clear */
	while (timeout > 0)
	{
		status = readl((void __iomem *)NOC_IDLE_STATUS_REG0);
		if ((status & display_mask) == 0)
			return 0;
		udelay(NOC_IDLE_POLL_US);
		timeout--;
	}

	CRM_LOG_ERROR("NOC idle release timeout\n");
	return -ETIMEDOUT;
}

/*
 * Configure DC8000 AXI clock
 */
static long configure_dc8000_axi_clock(unsigned long target_rate)
{
	u32 reg_addr = X5_CRM_HPS_CLK_GEN + DC8000_ACLK_OFF;
	long actual_rate;

	actual_rate = clk_gen_set_rate(reg_addr, CPU_PLL_P_RATE, target_rate, MUX_CPU_PLL_P);
	if (actual_rate < 0)
		return actual_rate;

	clk_gen_enable(reg_addr);

	CRM_LOG_DEBUG("DC8000 AXI clock: %ld Hz\n", actual_rate);
	return actual_rate;
}

/*
 * Configure DC8000 pixel clock
 * Derives pixel_clock from DISP_PLL_P via integer divider.
 */
static long configure_dc8000_pixel_clock(unsigned long pixel_clock)
{
	u32 reg_addr = X5_CRM_HPS_CLK_GEN + DC8000_PIXEL_CLK_OFF;
	long actual_rate;

	actual_rate = clk_gen_set_rate(reg_addr, g_disp_pll_rate, pixel_clock,
				       DISP_MUX_DISP_PLL_P);
	if (actual_rate < 0)
		return actual_rate;

	clk_gen_enable(reg_addr);

	CRM_LOG_DEBUG("DC8000 pixel clock: %ld Hz (PLL %lu Hz)\n",
		      actual_rate, g_disp_pll_rate);
	return actual_rate;
}

/*
 * Configure Display SIF AXI clock
 */
static long configure_disp_sif_axi_clock(unsigned long target_rate)
{
	u32 reg_addr = X5_CRM_HPS_CLK_GEN + DISP_SIF_ACLK_OFF;
	long actual_rate;

	actual_rate = clk_gen_set_rate(reg_addr, CPU_PLL_P_RATE, target_rate, MUX_CPU_PLL_P);
	if (actual_rate < 0)
		return actual_rate;

	clk_gen_enable(reg_addr);

	CRM_LOG_DEBUG("DISP SIF AXI clock: %ld Hz\n", actual_rate);
	return actual_rate;
}

/*
 * Configure DSI TX Escape clock
 */
static long configure_dsi_txesc_clock(unsigned long target_rate)
{
	u32 reg_addr = X5_CRM_HPS_CLK_GEN + DSI_TXESC_CLK_OFF;
	long actual_rate;

	actual_rate = clk_gen_set_rate(reg_addr, CPU_PLL_P_RATE, target_rate, MUX_CPU_PLL_P);
	if (actual_rate < 0)
		return actual_rate;

	clk_gen_enable(reg_addr);

	CRM_LOG_DEBUG("DSI TXESC clock: %ld Hz\n", actual_rate);
	return actual_rate;
}

#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
/* Keep cpu_pll_r/top_apb_clk aligned with the kernel handoff state. */
static void x5_crm_align_cpu_pll_r_postdiv(void)
{
	u32 pll_val, gen_val;
	u32 current_postdivr_reg;
	u32 old_prediv, old_postdiv, old_total_div;
	long actual_apb_rate;

	pll_val = readl((void __iomem *)CPU_PLL_POSTDIV);
	current_postdivr_reg = (pll_val & PLL_POSTDIVR_MASK) >> PLL_POSTDIVR_SHIFT;

	if (current_postdivr_reg == X5_SEAMLESS_CPU_PLL_R_POSTDIVR) {
		CRM_LOG_DEBUG("cpu_pll_r postdiv already 0x%x\n",
			      current_postdivr_reg);

		gen_val = readl((void __iomem *)(X5_CRM_HPS_CLK_GEN + TOP_APB_CLK_OFF));
		old_prediv = ((gen_val & GEN_PRE_DIV_MASK) >> GEN_PRE_DIV_SHIFT) + 1;
		old_postdiv = ((gen_val & GEN_POST_DIV_MASK) >> GEN_POST_DIV_SHIFT) + 1;
		old_total_div = old_prediv * old_postdiv;

		CRM_LOG_DEBUG("apb_gen=0x%08x (prediv=%u postdiv=%u total=%u)\n",
			      gen_val, old_prediv, old_postdiv, old_total_div);

		if (old_total_div != 1) {
			CRM_LOG_DEBUG("generator divider stale (%u), fix to 1\n",
				      old_total_div);
			actual_apb_rate = clk_gen_set_rate(
				X5_CRM_HPS_CLK_GEN + TOP_APB_CLK_OFF,
				TOP_APB_CLK_RATE, TOP_APB_CLK_RATE,
				MUX_CPU_PLL_R);
			gen_val = readl((void __iomem *)(X5_CRM_HPS_CLK_GEN + TOP_APB_CLK_OFF));
			CRM_LOG_DEBUG("apb_gen after fix=0x%08x, rate=%ld Hz\n",
				      gen_val, actual_apb_rate);
		}
		return;
	}

	CRM_LOG_DEBUG("cpu_pll_r postdiv: 0x%x -> 0x%x\n",
		      current_postdivr_reg, X5_SEAMLESS_CPU_PLL_R_POSTDIVR);

	pll_val &= ~PLL_POSTDIVR_MASK;
	pll_val |= SET_PLL_POSTDIVR(X5_SEAMLESS_CPU_PLL_R_POSTDIVR);
	writel(pll_val, (void __iomem *)CPU_PLL_POSTDIV);
	__asm__ __volatile__("dsb sy" : : : "memory");

	actual_apb_rate = clk_gen_set_rate(X5_CRM_HPS_CLK_GEN + TOP_APB_CLK_OFF,
					   TOP_APB_CLK_RATE, TOP_APB_CLK_RATE,
					   MUX_CPU_PLL_R);

	gen_val = readl((void __iomem *)(X5_CRM_HPS_CLK_GEN + TOP_APB_CLK_OFF));
	CRM_LOG_DEBUG("top_apb_clk: actual=%ld apb_gen=0x%08x\n",
		      actual_apb_rate, gen_val);
}
#endif

/*
 * Initialize display clocks
 *
 * Sequence:
 *   1. Configure DISP_PLL to pixel_clock
 *   2. Release NOC idle for display modules
 *   3. Enable APB gate clocks
 *   4. Configure and enable AXI clocks
 *   5. Configure and enable pixel clock
 *   6. Configure DC IOMMU
 */
int x5_crm_display_clk_init(unsigned long pixel_clock)
{
	int ret;

	CRM_LOG_INFO("Initializing display clocks (pixel_clock=%lu Hz)\n", pixel_clock);

	if (pixel_clock == 0 || pixel_clock > X5_CRM_PIXEL_CLOCK_MAX_HZ) {
		CRM_LOG_ERROR("invalid pixel_clock %lu\n", pixel_clock);
		return -EINVAL;
	}

	/* Keep legacy clock flow unchanged when seamless handoff is disabled. */
#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
	x5_crm_align_cpu_pll_r_postdiv();
#endif

	/* Configure DISP_PLL to a table-defined rate */
	g_disp_pll_rate = find_best_pll_rate(pixel_clock);
	ret = x5_crm_disp_pll_init(g_disp_pll_rate);
	if (ret)
	{
		CRM_LOG_ERROR("Failed to configure DISP_PLL (ret=%d)\n", ret);
		return ret;
	}

	/* Release NOC idle for display modules */
	ret = release_display_noc_idle();
	if (ret)
	{
		CRM_LOG_WARN("NOC idle release failed\n");
		/* Continue anyway, might still work */
	}

	/* Enable APB gate clocks */
	enable_display_apb_clocks();
	udelay(10);

	/* Configure and enable AXI clocks */
	if (configure_dc8000_axi_clock(DC8000_ACLK_RATE) < 0)
		return -EIO;

	if (configure_disp_sif_axi_clock(DISP_SIF_ACLK_RATE) < 0)
		return -EIO;

	if (configure_dsi_txesc_clock(DSI_TXESC_CLK_RATE) < 0)
		return -EIO;

	/* Configure and enable pixel clock */
	if (configure_dc8000_pixel_clock(pixel_clock) < 0)
		return -EIO;

	/* Configure DC IOMMU */
	x5_crm_dc_iommu_enable_linear();

	udelay(100);

	CRM_LOG_INFO("Display clocks initialized\n");
	return 0;
}

/*
 * Reset display subsystem
 */
int x5_crm_display_reset(void)
{
	u32 val;

	CRM_LOG_INFO("Resetting display subsystem\n");

	/* Assert resets */
	val = readl((void __iomem *)HPS_MIX_SW_RST);
	val |= (RST_DSI_TX | RST_DC8000_2 | RST_DC8000_3);
	writel(val, (void __iomem *)HPS_MIX_SW_RST);

	udelay(100);

	/* De-assert resets */
	val &= ~(RST_DSI_TX | RST_DC8000_2 | RST_DC8000_3);
	writel(val, (void __iomem *)HPS_MIX_SW_RST);

	udelay(100);

	CRM_LOG_INFO("Display reset complete\n");
	return 0;
}

/*
 * Disable display clocks (backward-compatible: disables shared + DSI)
 */
void x5_crm_display_clk_disable(void)
{
	/* Disable clock generators */
	clk_gen_disable(X5_CRM_HPS_CLK_GEN + DC8000_PIXEL_CLK_OFF);
	clk_gen_disable(X5_CRM_HPS_CLK_GEN + DC8000_ACLK_OFF);
	clk_gen_disable(X5_CRM_HPS_CLK_GEN + DISP_SIF_ACLK_OFF);
	clk_gen_disable(X5_CRM_HPS_CLK_GEN + DSI_TXESC_CLK_OFF);

	/* Disable APB gate clocks */
	disable_display_apb_clocks();

	CRM_LOG_INFO("Display clocks disabled\n");
}

/* ================================================================== */
/* Split CRM API — shared vs. output-specific                         */
/* ================================================================== */

static void enable_common_apb_clocks(void)
{
	u32 val = readl((void __iomem *)HPS_MIX_CLK_ENB);

	val |= CLK_EN_DC8000_PCLK;
	val |= CLK_EN_DC8000_ACLK;
	val |= CLK_EN_DISP_SIF_ACLK;
	val |= CLK_EN_DISP_SIF_PCLK;

	writel(val, (void __iomem *)HPS_MIX_CLK_ENB);
}

static void enable_dsi_apb_clocks(void)
{
	u32 val = readl((void __iomem *)HPS_MIX_CLK_ENB);

	val |= CLK_EN_DISP_DSI_PCLK;
	val |= CLK_EN_DPHY_CFG;

	writel(val, (void __iomem *)HPS_MIX_CLK_ENB);
}

static void disable_common_apb_clocks(void)
{
	u32 val = readl((void __iomem *)HPS_MIX_CLK_ENB);

	val &= ~CLK_EN_DC8000_PCLK;
	val &= ~CLK_EN_DC8000_ACLK;
	val &= ~CLK_EN_DISP_SIF_ACLK;
	val &= ~CLK_EN_DISP_SIF_PCLK;

	writel(val, (void __iomem *)HPS_MIX_CLK_ENB);
}

static void disable_dsi_apb_clocks(void)
{
	u32 val = readl((void __iomem *)HPS_MIX_CLK_ENB);

	val &= ~CLK_EN_DISP_DSI_PCLK;
	val &= ~CLK_EN_DPHY_CFG;

	writel(val, (void __iomem *)HPS_MIX_CLK_ENB);
}

/*
 * x5_crm_display_common_init - shared clock init (PLL, NOC, DC8000, IOMMU)
 */
int x5_crm_display_common_init(unsigned long pixel_clock)
{
	int ret;

	CRM_LOG_INFO("Common clock init (pixel_clock=%lu Hz)\n", pixel_clock);

	if (pixel_clock == 0 || pixel_clock > X5_CRM_PIXEL_CLOCK_MAX_HZ) {
		CRM_LOG_ERROR("invalid pixel_clock %lu\n", pixel_clock);
		return -EINVAL;
	}

#if CONFIG_IS_ENABLED(X5_SEAMLESS_DISPLAY)
	x5_crm_align_cpu_pll_r_postdiv();
#endif

	g_disp_pll_rate = find_best_pll_rate(pixel_clock);
	CRM_LOG_INFO("DISP_PLL target: %lu Hz (pixel %lu Hz, div %lu)\n",
		     g_disp_pll_rate, pixel_clock,
		     g_disp_pll_rate / pixel_clock);

	ret = x5_crm_disp_pll_init(g_disp_pll_rate);
	if (ret) {
		CRM_LOG_ERROR("Failed to configure DISP_PLL (ret=%d)\n", ret);
		return ret;
	}

	ret = release_display_noc_idle();
	if (ret)
		CRM_LOG_WARN("NOC idle release failed\n");

	enable_common_apb_clocks();
	udelay(10);

	if (configure_dc8000_axi_clock(DC8000_ACLK_RATE) < 0)
		return -EIO;

	if (configure_disp_sif_axi_clock(DISP_SIF_ACLK_RATE) < 0)
		return -EIO;

	if (configure_dc8000_pixel_clock(pixel_clock) < 0)
		return -EIO;

	x5_crm_dc_iommu_enable_linear();

	udelay(100);
	CRM_LOG_INFO("Common clocks initialized\n");
	return 0;
}

/*
 * x5_crm_dsi_clk_init - DSI-specific clock init (APB, DPHY cfg, TXESC)
 */
int x5_crm_dsi_clk_init(unsigned long pixel_clock)
{
	CRM_LOG_INFO("DSI clock init\n");

	enable_dsi_apb_clocks();
	udelay(10);

	if (configure_dsi_txesc_clock(DSI_TXESC_CLK_RATE) < 0)
		return -EIO;

	CRM_LOG_INFO("DSI clocks initialized\n");
	return 0;
}

/*
 * x5_crm_common_reset - shared reset (DC8000 only)
 */
int x5_crm_common_reset(void)
{
	u32 val;

	CRM_LOG_INFO("Resetting DC8000\n");

	val = readl((void __iomem *)HPS_MIX_SW_RST);
	val |= (RST_DC8000_2 | RST_DC8000_3);
	writel(val, (void __iomem *)HPS_MIX_SW_RST);
	udelay(100);

	val &= ~(RST_DC8000_2 | RST_DC8000_3);
	writel(val, (void __iomem *)HPS_MIX_SW_RST);
	udelay(100);

	return 0;
}

/*
 * x5_crm_dsi_reset - DSI TX reset
 */
int x5_crm_dsi_reset(void)
{
	u32 val;

	CRM_LOG_INFO("Resetting DSI TX\n");

	val = readl((void __iomem *)HPS_MIX_SW_RST);
	val |= RST_DSI_TX;
	writel(val, (void __iomem *)HPS_MIX_SW_RST);
	udelay(100);

	val &= ~RST_DSI_TX;
	writel(val, (void __iomem *)HPS_MIX_SW_RST);
	udelay(100);

	return 0;
}

/*
 * x5_crm_common_clk_disable - disable shared clocks only
 */
void x5_crm_common_clk_disable(void)
{
	clk_gen_disable(X5_CRM_HPS_CLK_GEN + DC8000_PIXEL_CLK_OFF);
	clk_gen_disable(X5_CRM_HPS_CLK_GEN + DC8000_ACLK_OFF);
	clk_gen_disable(X5_CRM_HPS_CLK_GEN + DISP_SIF_ACLK_OFF);
	disable_common_apb_clocks();
	CRM_LOG_INFO("Common clocks disabled\n");
}

/*
 * x5_crm_dsi_clk_disable - disable DSI-specific clocks only
 */
void x5_crm_dsi_clk_disable(void)
{
	clk_gen_disable(X5_CRM_HPS_CLK_GEN + DSI_TXESC_CLK_OFF);
	disable_dsi_apb_clocks();
	CRM_LOG_INFO("DSI clocks disabled\n");
}

/* ------------------------------------------------------------------ */
/* BT1120 clock / reset (HDMI via parallel BT1120 -> LT8618)         */
/* Note: HPS_CLK_GEN offset 0x500 is bt1120_aclk in kernel clk-x5.c; */
/* DSI TXESC in this tree uses the same offset — only one path active. */
/* ------------------------------------------------------------------ */

static void enable_bt1120_apb_clocks(void)
{
	u32 val = readl((void __iomem *)HPS_MIX_CLK_ENB);

	val |= CLK_EN_BT1120_PCLK;
	val |= CLK_EN_BT1120_ACLK;

	writel(val, (void __iomem *)HPS_MIX_CLK_ENB);
}

static void disable_bt1120_apb_clocks(void)
{
	u32 val = readl((void __iomem *)HPS_MIX_CLK_ENB);

	val &= ~CLK_EN_BT1120_PCLK;
	val &= ~CLK_EN_BT1120_ACLK;

	writel(val, (void __iomem *)HPS_MIX_CLK_ENB);
}

static long configure_bt1120_pixel_clock(unsigned long pixel_clock)
{
	u32 reg_addr = X5_CRM_HPS_CLK_GEN + BT1120_PIXEL_CLK_OFF;
	long actual_rate;

	actual_rate = clk_gen_set_rate(reg_addr, g_disp_pll_rate, pixel_clock,
				       DISP_MUX_DISP_PLL_P);
	if (actual_rate < 0)
		return actual_rate;

	clk_gen_enable(reg_addr);

	CRM_LOG_DEBUG("BT1120 pixel clock: %ld Hz (PLL %lu Hz)\n",
		      actual_rate, g_disp_pll_rate);
	return actual_rate;
}

static long configure_bt1120_axi_clock(unsigned long target_rate)
{
	u32 reg_addr = X5_CRM_HPS_CLK_GEN + BT1120_ACLK_OFF;
	long actual_rate;

	actual_rate = clk_gen_set_rate(reg_addr, CPU_PLL_P_RATE, target_rate,
				       MUX_CPU_PLL_P);
	if (actual_rate < 0)
		return actual_rate;

	clk_gen_enable(reg_addr);

	CRM_LOG_DEBUG("BT1120 AXI clock: %ld Hz\n", actual_rate);
	return actual_rate;
}

void x5_crm_i2c4_pclk_enable(void)
{
	u32 val = readl((void __iomem *)LSIO_0_CLK_ENB);
	val |= CLK_EN_I2C4_PCLK;
	writel(val, (void __iomem *)LSIO_0_CLK_ENB);
	udelay(10);
}

void x5_crm_dsp_i2c_pclk_enable(void)
{
	u32 val;

	val = readl((void __iomem *)DSP_CLK_ENB);
	val |= CLK_EN_DSP_I2C_PCLK;
	writel(val, (void __iomem *)DSP_CLK_ENB);
	udelay(10);

	val = readl((void __iomem *)DSP_SW_RST_REG);
	val &= ~RST_DSP_I2C;
	writel(val, (void __iomem *)DSP_SW_RST_REG);
	udelay(10);
}

int x5_crm_bt1120_clk_init(unsigned long pixel_clock)
{
	int ret;

	CRM_LOG_INFO("BT1120 clock init (pixel_clock=%lu Hz)\n", pixel_clock);

	if (pixel_clock == 0 || pixel_clock > X5_CRM_PIXEL_CLOCK_MAX_HZ) {
		CRM_LOG_ERROR("invalid pixel_clock %lu\n", pixel_clock);
		return -EINVAL;
	}

	ret = x5_crm_noc_idle_release(NOC_IDLE_BT1120);
	if (ret)
		CRM_LOG_WARN("BT1120 NOC idle release failed (ret=%d)\n", ret);

	enable_bt1120_apb_clocks();
	udelay(10);

	if (configure_bt1120_axi_clock(DC8000_ACLK_RATE) < 0)
		return -EIO;

	if (configure_bt1120_pixel_clock(pixel_clock) < 0)
		return -EIO;

	CRM_LOG_INFO("BT1120 clocks initialized\n");
	return 0;
}

int x5_crm_bt1120_reset(void)
{
	u32 val;

	CRM_LOG_INFO("Resetting BT1120\n");

	val = readl((void __iomem *)HPS_MIX_SW_RST);
	val |= RST_BT1120;
	writel(val, (void __iomem *)HPS_MIX_SW_RST);
	udelay(100);

	val &= ~RST_BT1120;
	writel(val, (void __iomem *)HPS_MIX_SW_RST);
	udelay(100);

	return 0;
}

void x5_crm_bt1120_clk_disable(void)
{
	clk_gen_disable(X5_CRM_HPS_CLK_GEN + BT1120_PIXEL_CLK_OFF);
	clk_gen_disable(X5_CRM_HPS_CLK_GEN + BT1120_ACLK_OFF);
	disable_bt1120_apb_clocks();
	CRM_LOG_INFO("BT1120 clocks disabled\n");
}

/*
 * Initialize LPWM clock
 */
int x5_crm_lpwm_clk_init(int lpwm_id)
{
	u32 gen_offset;
	u32 pclk_bit;
	u32 val;
	long actual_rate;

	if (lpwm_id == 0)
	{
		gen_offset = LPWM0_CLK_OFF;
		pclk_bit = CLK_EN_LPWM0_PCLK;
	}
	else if (lpwm_id == 1)
	{
		gen_offset = LPWM1_CLK_OFF;
		pclk_bit = CLK_EN_LPWM1_PCLK;
	}
	else
	{
		CRM_LOG_ERROR("Invalid LPWM ID %d\n", lpwm_id);
		return -EINVAL;
	}

	CRM_LOG_INFO("Initializing LPWM%d clocks\n", lpwm_id);

	/* Configure clock generator: 24MHz from OSC */
	actual_rate = clk_gen_set_rate(X5_CRM_HPS_CLK_GEN + gen_offset,
								   OSC_RATE, LPWM_CLK_RATE, MUX_OSC);
	if (actual_rate < 0)
	{
		CRM_LOG_ERROR("Failed to configure LPWM%d clock (ret=%ld)\n", lpwm_id,
			      actual_rate);
		return (int)actual_rate;
	}

	clk_gen_enable(X5_CRM_HPS_CLK_GEN + gen_offset);

	/* Enable APB clock */
	val = readl((void __iomem *)LSIO_0_CLK_ENB);
	val |= pclk_bit;
	writel(val, (void __iomem *)LSIO_0_CLK_ENB);

	CRM_LOG_INFO("LPWM%d clocks: %lu Hz\n", lpwm_id, (unsigned long)actual_rate);
	return 0;
}

/*
 * Disable LPWM clock
 */
void x5_crm_lpwm_clk_disable(int lpwm_id)
{
	u32 gen_offset;
	u32 pclk_bit;
	u32 val;

	if (lpwm_id == 0)
	{
		gen_offset = LPWM0_CLK_OFF;
		pclk_bit = CLK_EN_LPWM0_PCLK;
	}
	else if (lpwm_id == 1)
	{
		gen_offset = LPWM1_CLK_OFF;
		pclk_bit = CLK_EN_LPWM1_PCLK;
	}
	else
	{
		return;
	}

	clk_gen_disable(X5_CRM_HPS_CLK_GEN + gen_offset);

	val = readl((void __iomem *)LSIO_0_CLK_ENB);
	val &= ~pclk_bit;
	writel(val, (void __iomem *)LSIO_0_CLK_ENB);

	CRM_LOG_INFO("LPWM%d clocks disabled\n", lpwm_id);
}

/*
 * Dump display clock configuration for debugging
 */
void x5_crm_dump_display_clocks(void)
{
	u32 val;
	u8 mux, prediv, postdiv;
	int enabled;

	CRM_LOG_DEBUG("\n=== Display Clock Configuration ===\n");

	/* HPS_MIX_CLK_ENB */
	val = readl((void __iomem *)HPS_MIX_CLK_ENB);
	CRM_LOG_DEBUG("HPS_MIX_CLK_ENB: 0x%08x\n", val);
	CRM_LOG_DEBUG("  DSI_PCLK: %s, DC8000_PCLK: %s, DC8000_ACLK: %s\n",
		   (val & CLK_EN_DISP_DSI_PCLK) ? "ON" : "OFF",
		   (val & CLK_EN_DC8000_PCLK) ? "ON" : "OFF",
		   (val & CLK_EN_DC8000_ACLK) ? "ON" : "OFF");
	CRM_LOG_DEBUG("  DPHY_CFG: %s, SIF_ACLK: %s, SIF_PCLK: %s\n",
		   (val & CLK_EN_DPHY_CFG) ? "ON" : "OFF",
		   (val & CLK_EN_DISP_SIF_ACLK) ? "ON" : "OFF",
		   (val & CLK_EN_DISP_SIF_PCLK) ? "ON" : "OFF");

	/* DC8000 Pixel Clock */
	val = readl((void __iomem *)(X5_CRM_HPS_CLK_GEN + DC8000_PIXEL_CLK_OFF));
	enabled = (val & GEN_EN) ? 1 : 0;
	mux = (val & GEN_MUX_MASK) >> GEN_MUX_SHIFT;
	prediv = ((val & GEN_PRE_DIV_MASK) >> GEN_PRE_DIV_SHIFT) + 1;
	postdiv = ((val & GEN_POST_DIV_MASK) >> GEN_POST_DIV_SHIFT) + 1;
	CRM_LOG_DEBUG("DC8000_PIXEL_CLK: 0x%08x (EN=%d, MUX=%d, DIV=%d)\n",
		   val, enabled, mux, prediv * postdiv);

	/* DC8000 AXI Clock */
	val = readl((void __iomem *)(X5_CRM_HPS_CLK_GEN + DC8000_ACLK_OFF));
	enabled = (val & GEN_EN) ? 1 : 0;
	mux = (val & GEN_MUX_MASK) >> GEN_MUX_SHIFT;
	prediv = ((val & GEN_PRE_DIV_MASK) >> GEN_PRE_DIV_SHIFT) + 1;
	postdiv = ((val & GEN_POST_DIV_MASK) >> GEN_POST_DIV_SHIFT) + 1;
	CRM_LOG_DEBUG("DC8000_ACLK: 0x%08x (EN=%d, MUX=%d, DIV=%d)\n",
		   val, enabled, mux, prediv * postdiv);

	/* DISP SIF AXI Clock */
	val = readl((void __iomem *)(X5_CRM_HPS_CLK_GEN + DISP_SIF_ACLK_OFF));
	enabled = (val & GEN_EN) ? 1 : 0;
	mux = (val & GEN_MUX_MASK) >> GEN_MUX_SHIFT;
	prediv = ((val & GEN_PRE_DIV_MASK) >> GEN_PRE_DIV_SHIFT) + 1;
	postdiv = ((val & GEN_POST_DIV_MASK) >> GEN_POST_DIV_SHIFT) + 1;
	CRM_LOG_DEBUG("DISP_SIF_ACLK: 0x%08x (EN=%d, MUX=%d, DIV=%d)\n",
		   val, enabled, mux, prediv * postdiv);

	/* DSI TX Escape Clock */
	val = readl((void __iomem *)(X5_CRM_HPS_CLK_GEN + DSI_TXESC_CLK_OFF));
	enabled = (val & GEN_EN) ? 1 : 0;
	mux = (val & GEN_MUX_MASK) >> GEN_MUX_SHIFT;
	prediv = ((val & GEN_PRE_DIV_MASK) >> GEN_PRE_DIV_SHIFT) + 1;
	postdiv = ((val & GEN_POST_DIV_MASK) >> GEN_POST_DIV_SHIFT) + 1;
	CRM_LOG_DEBUG("DSI_TXESC_CLK: 0x%08x (EN=%d, MUX=%d, DIV=%d)\n",
		   val, enabled, mux, prediv * postdiv);

	CRM_LOG_DEBUG("===================================\n\n");
}

/*
 * NOC Idle Control Functions
 */
int x5_crm_noc_idle_release(u32 mask)
{
	u32 val, status;
	int timeout = NOC_IDLE_TIMEOUT_US / NOC_IDLE_POLL_US;

	val = readl((void __iomem *)NOC_IDLE_REQ_REG0);
	val &= ~mask;
	writel(val, (void __iomem *)NOC_IDLE_REQ_REG0);

	while (timeout > 0)
	{
		status = readl((void __iomem *)NOC_IDLE_STATUS_REG0);
		if ((status & mask) == 0)
			return 0;
		udelay(NOC_IDLE_POLL_US);
		timeout--;
	}

	return -ETIMEDOUT;
}

int x5_crm_noc_idle_request(u32 mask)
{
	u32 val, status;
	int timeout = NOC_IDLE_TIMEOUT_US / NOC_IDLE_POLL_US;

	val = readl((void __iomem *)NOC_IDLE_REQ_REG0);
	val |= mask;
	writel(val, (void __iomem *)NOC_IDLE_REQ_REG0);

	while (timeout > 0)
	{
		status = readl((void __iomem *)NOC_IDLE_STATUS_REG0);
		if ((status & mask) == mask)
			return 0;
		udelay(NOC_IDLE_POLL_US);
		timeout--;
	}

	return -ETIMEDOUT;
}

void x5_crm_dump_noc_idle(void)
{
	u32 req, status;

	req = readl((void __iomem *)NOC_IDLE_REQ_REG0);
	status = readl((void __iomem *)NOC_IDLE_STATUS_REG0);

	CRM_LOG_DEBUG("\n=== NOC Idle Status ===\n");
	CRM_LOG_DEBUG("REQ: 0x%08x, STATUS: 0x%08x\n", req, status);
	CRM_LOG_DEBUG("DC8000: req=%d, status=%d\n",
		   (req & NOC_IDLE_DC8000) ? 1 : 0,
		   (status & NOC_IDLE_DC8000) ? 1 : 0);
	CRM_LOG_DEBUG("DISP_SIF: req=%d, status=%d\n",
		   (req & NOC_IDLE_DISP_SIF) ? 1 : 0,
		   (status & NOC_IDLE_DISP_SIF) ? 1 : 0);
	CRM_LOG_DEBUG("=======================\n\n");
}

/*
 * DC IOMMU Control Functions
 */
void x5_crm_dc_iommu_disable(void)
{
	writel(0, (void __iomem *)DC_IOMMU_CTRL);
}

/*
 * Enable DC IOMMU with linear (identity) mapping
 * Maps address bits [31:28] directly (0->0, 1->1, ..., F->F)
 */
void x5_crm_dc_iommu_enable_linear(void)
{
	/* Linear mapping: each 256MB region maps to itself */
	writel(0x03020100, (void __iomem *)DC_IOMMU_MAP_CTRL0); /* 0-3 */
	writel(0x07060504, (void __iomem *)DC_IOMMU_MAP_CTRL1); /* 4-7 */
	writel(0x0B0A0908, (void __iomem *)DC_IOMMU_MAP_CTRL2); /* 8-11 */
	writel(0x0F0E0D0C, (void __iomem *)DC_IOMMU_MAP_CTRL3); /* 12-15 */

	/* Enable IOMMU */
	writel(DC_IOMMU_CTRL_ENABLE, (void __iomem *)DC_IOMMU_CTRL);
}

void x5_crm_dump_dc_iommu(void)
{
	u32 ctrl, map0, map1, map2, map3;

	ctrl = readl((void __iomem *)DC_IOMMU_CTRL);
	map0 = readl((void __iomem *)DC_IOMMU_MAP_CTRL0);
	map1 = readl((void __iomem *)DC_IOMMU_MAP_CTRL1);
	map2 = readl((void __iomem *)DC_IOMMU_MAP_CTRL2);
	map3 = readl((void __iomem *)DC_IOMMU_MAP_CTRL3);

	CRM_LOG_DEBUG("\n=== DC IOMMU Status ===\n");
	CRM_LOG_DEBUG("CTRL: 0x%08x (%s)\n", ctrl,
		   (ctrl & DC_IOMMU_CTRL_ENABLE) ? "ENABLED" : "DISABLED");
	CRM_LOG_DEBUG("MAP0: 0x%08x, MAP1: 0x%08x\n", map0, map1);
	CRM_LOG_DEBUG("MAP2: 0x%08x, MAP3: 0x%08x\n", map2, map3);
	CRM_LOG_DEBUG("=======================\n\n");
}
