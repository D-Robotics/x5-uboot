// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 *
 * X5 Synopsys MIPI DPHY TX Driver
 *
 * Ported from kernel: drivers/phy/verisilicon/phy-snps-mipi-dphy.c
 */

#include <common.h>
#include <dm.h>
#include <div64.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <hb_display_log.h>

/* Constants */
#define KHZ 1000UL
#define MHZ (1000 * 1000UL)
#define PSEC_PER_SEC 1000000000000ULL

/*
 * PLL Reference Clock: 24 MHz (from OSC via dphy_cfg_clk)
 *
 * From kernel device tree (x5.dtsi):
 *   dphy0: phy@3e0a0028 {
 *       clocks = <&hpsclks X5_DISP_DPHY_CFG_CLK>, <&hpsclks X5_DISP_DPHY_CFG_CLK>;
 *       clock-names = "ref-clk", "cfg-clk";
 *   }
 *
 * X5_DISP_DPHY_CFG_CLK is a gate clock from "osc" (24MHz)
 */
#define PLL_REF_CLK_KHZ 24000

/* PLL constraints */
#define MAX_DM 625
#define MIN_DM 40
#define MAX_DN 16
#define MIN_DN 1
#define MAX_VCO_KHZ (1250 * KHZ)
#define MIN_VCO_KHZ (320 * KHZ)
#define MAX_LINK_RATE (1250 * MHZ)
#define MIN_LINK_RATE (40 * MHZ)

/* Delays (test-interface bit-bang; PLL settle — see x5_dphy_configure comment) */
#define DPHY_TEST_DELAY_US      10
#define DPHY_PLL_STABILIZE_MS   50

/* Register Offsets */
#define DISP_DPHY_CFG_0 0x04
#define DISP_DPHY_PLL_CFG 0x08
#define DISP_PHY_TST_CTRL0 0x24
#define DISP_PHY_TST_CTRL1 0x28

/* Test Interface Bits */
#define TESTCLR_BIT BIT(0)
#define TESTCLK_BIT BIT(1)
#define TESTEN_BIT BIT(16)
#define TESTDIN_MASK GENMASK(7, 0)
#define TESTDOUT_MASK GENMASK(15, 8)
#define TESTDOUT_SHIFT 8

/* PLL Test Codes */
#define PLL_PROP_CHARGE_PUMP_CTRL 0x0E
#define PLL_INT_CHARGE_PUMP_CTRL 0x0F
#define PLL_VCO_CTRL 0x12
#define PLL_GMP_CTRL 0x13
#define PLL_PHASE_ERR_CTRL 0x14
#define PLL_LOCKING_FILTER 0x15
#define PLL_UNLOCKING_FILTER 0x16
#define PLL_INPUT_DIVIDER_RATIO 0x17
#define PLL_LOOP_DIVIDER_RATIO 0x18
#define PLL_INPUT_LOOP_DIVIDER_RATIO_CTRL 0x19
#define PLL_CHARGE_PUMP_BIAS_CTRL 0x1C
#define PLL_LOCK_DETECTOR_MODE_SEL 0x1D
#define PLL_ANALOG_PROG_CTRL 0x1F
#define SLEW_RATE_FSM_OVERRIDE_CTRL 0xA0
#define SLEW_RATE_DDL_LOOP_CONF_CTRL 0xA3
#define LP_RX_BIAS_CTRL 0x4A

/* HS Timing Test Codes (from kernel) */
#define HS_TX_CLK_TLP_CODE 0x60
#define HS_TX_CLK_PREPARE_CODE 0x61
#define HS_TX_CLK_ZERO_CODE 0x62
#define HS_TX_CLK_TRAIL_CODE 0x63
#define HS_TX_CLK_EXIT_CODE 0x64
#define HS_TX_CLK_POST_CODE 0x65
#define HS_TX_DAT_TLP_CODE 0x70
#define HS_TX_DAT_PREPARE_CODE 0x71
#define HS_TX_DAT_ZERO_CODE 0x72
#define HS_TX_DAT_TRAIL_CODE 0x73
#define HS_TX_DAT_EXIT_CODE 0x74

#define diff_abs(a, b) (((a) > (b)) ? ((a) - (b)) : ((b) - (a)))

/* HS Frequency Range Table (from kernel) */
struct hsfreq_range
{
	unsigned int min_khz;
	unsigned int max_khz;
	u8 value;
};

static const struct hsfreq_range hsfreqrange_table[] = {
	{80000, 97125, 0x00},
	{80000, 107625, 0x10},
	{83125, 118125, 0x20},
	{92625, 128625, 0x30},
	{102125, 139125, 0x01},
	{111625, 149625, 0x11},
	{121125, 160125, 0x21},
	{130625, 170625, 0x31},
	{140125, 181125, 0x02},
	{149625, 191625, 0x12},
	{159125, 202125, 0x22},
	{168625, 212625, 0x32},
	{182875, 228375, 0x03},
	{197125, 244125, 0x13},
	{211375, 259875, 0x23},
	{225625, 275625, 0x33},
	{249375, 301875, 0x04},
	{273125, 328125, 0x14},
	{296875, 354375, 0x25},
	{320625, 380625, 0x35},
	{368125, 433125, 0x05},
	{415625, 485625, 0x16},
	{463125, 538125, 0x26},
	{510625, 590625, 0x37},
	{558125, 643125, 0x07},
	{605625, 695625, 0x18},
	{653125, 748125, 0x28},
	{700625, 800625, 0x39},
	{748125, 853125, 0x09},
	{795625, 905625, 0x19},
	{843125, 958125, 0x29},
	{890625, 1010625, 0x3A},
	{938125, 1063125, 0x0A},
	{985625, 1115625, 0x1A},
	{1033125, 1168125, 0x2A},
	{1080625, 1220625, 0x3B},
	{1128125, 1273125, 0x0B},
	{1175625, 1325625, 0x1B},
	{1223125, 1378125, 0x2B},
	{1270625, 1430625, 0x3C},
	{1318125, 1483125, 0x0C},
	{1365625, 1535625, 0x1C},
	{1413125, 1588125, 0x2C},
	{1460625, 1640625, 0x3D},
	{1508125, 1693125, 0x0D},
	{1555625, 1745625, 0x1D},
	{1603125, 1798125, 0x2E},
	{1650625, 1850625, 0x3E},
	{1698125, 1903125, 0x0E},
	{1745625, 1955625, 0x1E},
	{1793125, 2008125, 0x2F},
	{1840625, 2060625, 0x3F},
	{1888125, 2113125, 0x0F},
	{1935625, 2165625, 0x40},
	{1983125, 2218125, 0x41},
	{2030625, 2270625, 0x42},
	{2078125, 2323125, 0x43},
	{2125625, 2375625, 0x44},
	{2173125, 2428125, 0x45},
	{2220625, 2480625, 0x46},
	{2268125, 2500000, 0x47},
};

/* PLL Info structure */
struct pll_info
{
	unsigned int input; /* Input clock in KHz */
	unsigned int fout;	/* Output clock in KHz */
	unsigned int dm;	/* Loop divider */
	unsigned int dn;	/* Input divider */
};

/* DPHY private data */
struct x5_dphy_priv
{
	void __iomem *base;
	unsigned int ref_clk_khz;
	unsigned int hs_clk_rate; /* HS clock rate in Hz */
	unsigned int lane_mbps;	  /* Lane rate in Mbps */
};

/*
 * Single static DPHY context: U-Boot uses one MIPI-DPHY instance for the display
 * path during sequential boot; there is no second controller or concurrent access.
 */
static struct x5_dphy_priv g_dphy_priv;

/*
 * Register access helpers
 */
static inline void dphy_write(struct x5_dphy_priv *priv, u32 reg, u32 val)
{
	writel(val, priv->base + reg);
}

static inline u32 dphy_read(struct x5_dphy_priv *priv, u32 reg)
{
	return readl(priv->base + reg);
}

/*
 * Test interface write sequence (from kernel)
 */
static void dphy_test_write(struct x5_dphy_priv *priv, u8 code, u8 data)
{
	/* Set test code with TESTEN high */
	dphy_write(priv, DISP_PHY_TST_CTRL1, TESTEN_BIT | code);
	/* Clock pulse */
	dphy_write(priv, DISP_PHY_TST_CTRL0, TESTCLK_BIT);
	dphy_write(priv, DISP_PHY_TST_CTRL0, 0);
	/* Write test data */
	dphy_write(priv, DISP_PHY_TST_CTRL1, data);
	/* Clock pulse */
	dphy_write(priv, DISP_PHY_TST_CTRL0, TESTCLK_BIT);
	dphy_write(priv, DISP_PHY_TST_CTRL0, 0);
	udelay(DPHY_TEST_DELAY_US);
}

/*
 * Test interface write with two data bytes
 */
static void dphy_test_write2(struct x5_dphy_priv *priv, u8 code, u8 data0, u8 data1)
{
	dphy_test_write(priv, code, data0);
	/* Write second byte */
	dphy_write(priv, DISP_PHY_TST_CTRL1, data1);
	dphy_write(priv, DISP_PHY_TST_CTRL0, TESTCLK_BIT);
	dphy_write(priv, DISP_PHY_TST_CTRL0, 0);
}

/*
 * Reset test interface
 */
static void dphy_test_reset(struct x5_dphy_priv *priv)
{
	dphy_write(priv, DISP_PHY_TST_CTRL0, TESTCLR_BIT);
	dphy_write(priv, DISP_PHY_TST_CTRL0, 0);
}

/*
 * Get HS frequency range value from table
 */
static u8 get_hsfreqrange(unsigned int fout_khz)
{
	unsigned int hs_rate_khz = fout_khz * 2; /* DDR, so double */
	int i;

	for (i = 0; i < ARRAY_SIZE(hsfreqrange_table); i++)
	{
		if (hs_rate_khz > hsfreqrange_table[i].min_khz &&
			hs_rate_khz <= hsfreqrange_table[i].max_khz)
			return hsfreqrange_table[i].value;
	}

	/* Default to first entry */
	return hsfreqrange_table[0].value;
}

/*
 * Get config clock frequency range
 * Formula: (ref_clk_mhz - 17) * 4
 */
static u8 get_cfgfreqrange(unsigned int ref_clk_khz)
{
	unsigned int ref_clk_mhz = ref_clk_khz / 1000;

	if (ref_clk_mhz < 17)
		return 0;

	return (ref_clk_mhz - 17) * 4;
}

/*
 * Get VCO control value based on output frequency
 */
static u8 get_vco_ctrl(unsigned int fout_khz)
{
	if (fout_khz > 1100 * KHZ)
		return 0x01;
	else if (fout_khz > 630 * KHZ)
		return 0x03;
	else if (fout_khz > 420 * KHZ)
		return 0x07;
	else if (fout_khz > 320 * KHZ)
		return 0x0F;
	else if (fout_khz > 210 * KHZ)
		return 0x17;
	else if (fout_khz > 160 * KHZ)
		return 0x1F;
	else if (fout_khz > 105 * KHZ)
		return 0x27;
	else if (fout_khz > 80 * KHZ)
		return 0x2F;
	else if (fout_khz > 52500)
		return 0x37;
	else
		return 0x3F;
}

/*
 * Find best PLL parameters (dm, dn) for target frequency
 * From kernel: find_best_rate()
 */
static void find_best_pll_params(struct pll_info *info)
{
	unsigned int dp, dm, dn;
	unsigned long long vco, diff, target;

	/* Determine output divider based on fout range */
	if (info->fout > 320 * KHZ)
		dp = 1;
	else if (info->fout > 160 * KHZ)
		dp = 2;
	else if (info->fout > 80 * KHZ)
		dp = 4;
	else
		dp = 8;

	vco = (unsigned long long)dp * info->fout;
	diff = vco;
	info->dm = MIN_DM;
	info->dn = MIN_DN;

	/* Search for best dm/dn combination */
	for (dn = MIN_DN; dn <= MAX_DN; dn++)
	{
		for (dm = MAX_DM; dm >= MIN_DM; dm--)
		{
			target = (unsigned long long)dm * info->input;
			do_div(target, dn);

			if (diff_abs(target, vco) < diff)
			{
				diff = diff_abs(target, vco);
				info->dm = dm;
				info->dn = dn;
			}

			if (diff == 0)
				break;
		}
		if (diff == 0)
			break;
	}

	/* Recalculate actual fout */
	vco = (unsigned long long)info->dm * info->input;
	do_div(vco, info->dn);
	info->fout = vco / dp;

	/* Hardware expects dm-2 and dn-1 */
	info->dm -= 2;
	info->dn -= 1;

	DPHY_LOG_DEBUG("PLL: input=%u KHz, fout=%u KHz, dm=%u, dn=%u\n",
		  info->input, info->fout, info->dm + 2, info->dn + 1);
}

/*
 * Get lane byte clock cycles from picosecond
 * From kernel: ps_to_lbcc()
 */
static inline unsigned int ps_to_lbcc(unsigned long hs_clk_rate, unsigned int ps)
{
	unsigned long long ui, lbcc;

	/* ui_ps = ceil(1e12 / HS-CLK) */
	ui = PSEC_PER_SEC;
	do_div(ui, hs_clk_rate);
	if (ui == 0)
		ui = 1;

	/* lbcc = ps / 8 / ui */
	lbcc = ps / 8;
	do_div(lbcc, ui);

	return (unsigned int)lbcc;
}

/*
 * Configure DPHY HS Timing
 * From kernel: config_pll() with dbg_write_en=1
 *
 * This configures the HS timing parameters using test codes 0x60-0x74.
 * These are critical for proper MIPI DSI operation.
 *
 * IMPORTANT: The timing values MUST match kernel's phy-core-mipi-dphy.c exactly!
 */
static void dphy_config_hs_timing(struct x5_dphy_priv *priv, unsigned long hs_clk_rate)
{
	unsigned int cnt;
	u8 val;
	unsigned long long ui;

	/*
	 * Calculate UI (Unit Interval) in picoseconds
	 * UI = 1 / hs_clk_rate (in seconds), converted to picoseconds
	 * From kernel: ui = ALIGN(PSEC_PER_SEC, hs_clk_rate) / hs_clk_rate
	 */
	ui = PSEC_PER_SEC;
	do_div(ui, hs_clk_rate);

	/*
	 * MIPI D-PHY timing parameters (from kernel phy-core-mipi-dphy.c)
	 * All values in picoseconds
	 *
	 * These MUST match the kernel's phy_mipi_dphy_get_default_config() exactly!
	 */
	unsigned int lpx = 50000;		  /* 50ns - T_LPX >= 50ns */
	unsigned int clk_prepare = 38000; /* 38ns - T_CLK-PREPARE */
	unsigned int clk_zero = 262000;	  /* 262ns - T_CLK-ZERO */
	unsigned int clk_trail = 60000;	  /* 60ns - T_CLK-TRAIL >= 60ns */
	/* T_CLK-POST = 60ns + 52*UI */
	unsigned int clk_post = 60000 + 52 * ui;
	/* T_HS-PREPARE = 40ns + 4*UI */
	unsigned int hs_prepare = 40000 + 4 * ui;
	/* T_HS-ZERO = 105ns + 6*UI */
	unsigned int hs_zero = 105000 + 6 * ui;
	/*
	 * T_HS-TRAIL = max(n*8*UI, 60ns + n*4*UI) where n=4 for reverse HS mode
	 * From kernel: cfg->hs_trail = max(4 * 8 * ui, 60000 + 4 * 4 * ui);
	 * = max(32*UI, 60ns + 16*UI)
	 */
	unsigned int hs_trail_opt1 = 4 * 8 * ui;		 /* 32*UI */
	unsigned int hs_trail_opt2 = 60000 + 4 * 4 * ui; /* 60ns + 16*UI */
	unsigned int hs_trail = (hs_trail_opt1 > hs_trail_opt2) ? hs_trail_opt1 : hs_trail_opt2;
	unsigned int hs_exit = 100000; /* 100ns - T_HS-EXIT >= 100ns */

	DPHY_LOG_DEBUG("Configuring HS timing for hs_clk_rate=%lu Hz\n", hs_clk_rate);

	/* HS clock TLP (code 0x60) */
	cnt = ps_to_lbcc(hs_clk_rate, lpx);
	DPHY_LOG_DEBUG("HS_TX_CLK_TLP: lpx=%u ps, cnt=%u\n", lpx, cnt);
	dphy_test_write(priv, HS_TX_CLK_TLP_CODE, cnt);

	/* HS clock PREPARE (code 0x61) */
	cnt = ps_to_lbcc(hs_clk_rate, clk_prepare);
	val = BIT(6) | cnt;
	DPHY_LOG_DEBUG("HS_TX_CLK_PREPARE: clk_prepare=%u ps, cnt=%u, val=0x%02x\n",
		  clk_prepare, cnt, val);
	dphy_test_write(priv, HS_TX_CLK_PREPARE_CODE, val);

	/* HS clock ZERO (code 0x62) */
	cnt = ps_to_lbcc(hs_clk_rate, clk_zero);
	val = BIT(7) | cnt;
	DPHY_LOG_DEBUG("HS_TX_CLK_ZERO: clk_zero=%u ps, cnt=%u, val=0x%02x\n",
		  clk_zero, cnt, val);
	dphy_test_write(priv, HS_TX_CLK_ZERO_CODE, val);

	/* HS clock TRAIL (code 0x63) */
	cnt = ps_to_lbcc(hs_clk_rate, clk_trail);
	val = BIT(6) | cnt;
	DPHY_LOG_DEBUG("HS_TX_CLK_TRAIL: clk_trail=%u ps, cnt=%u, val=0x%02x\n",
		  clk_trail, cnt, val);
	dphy_test_write(priv, HS_TX_CLK_TRAIL_CODE, val);

	/* HS clock EXIT (code 0x64) */
	cnt = ps_to_lbcc(hs_clk_rate, hs_exit);
	val = BIT(7) | BIT(6) | cnt;
	DPHY_LOG_DEBUG("HS_TX_CLK_EXIT: hs_exit=%u ps, cnt=%u, val=0x%02x\n",
		  hs_exit, cnt, val);
	dphy_test_write(priv, HS_TX_CLK_EXIT_CODE, val);

	/* HS clock POST (code 0x65) */
	cnt = ps_to_lbcc(hs_clk_rate, clk_post);
	val = BIT(6) | cnt;
	DPHY_LOG_DEBUG("HS_TX_CLK_POST: clk_post=%u ps, cnt=%u, val=0x%02x\n",
		  clk_post, cnt, val);
	dphy_test_write(priv, HS_TX_CLK_POST_CODE, val);

	/* HS data TLP (code 0x70) */
	cnt = ps_to_lbcc(hs_clk_rate, lpx);
	DPHY_LOG_DEBUG("HS_TX_DAT_TLP: lpx=%u ps, cnt=%u\n", lpx, cnt);
	dphy_test_write(priv, HS_TX_DAT_TLP_CODE, cnt);

	/* HS data PREPARE (code 0x71) */
	cnt = ps_to_lbcc(hs_clk_rate, hs_prepare);
	val = BIT(6) | cnt;
	DPHY_LOG_DEBUG("HS_TX_DAT_PREPARE: hs_prepare=%u ps, cnt=%u, val=0x%02x\n",
		  hs_prepare, cnt, val);
	dphy_test_write(priv, HS_TX_DAT_PREPARE_CODE, val);

	/* HS data ZERO (code 0x72) */
	cnt = ps_to_lbcc(hs_clk_rate, hs_zero);
	val = BIT(7) | cnt;
	DPHY_LOG_DEBUG("HS_TX_DAT_ZERO: hs_zero=%u ps, cnt=%u, val=0x%02x\n",
		  hs_zero, cnt, val);
	dphy_test_write(priv, HS_TX_DAT_ZERO_CODE, val);

	/* HS data TRAIL (code 0x73) */
	cnt = ps_to_lbcc(hs_clk_rate, hs_trail);
	val = BIT(6) | cnt;
	DPHY_LOG_DEBUG("HS_TX_DAT_TRAIL: hs_trail=%u ps, cnt=%u, val=0x%02x\n",
		  hs_trail, cnt, val);
	dphy_test_write(priv, HS_TX_DAT_TRAIL_CODE, val);

	/* HS data EXIT (code 0x74) */
	cnt = ps_to_lbcc(hs_clk_rate, hs_exit);
	val = BIT(7) | BIT(6) | cnt;
	DPHY_LOG_DEBUG("HS_TX_DAT_EXIT: hs_exit=%u ps, cnt=%u, val=0x%02x\n",
		  hs_exit, cnt, val);
	dphy_test_write(priv, HS_TX_DAT_EXIT_CODE, val);

	DPHY_LOG_DEBUG("HS timing configured\n");
}

/*
 * Configure DPHY PLL
 * From kernel: config_pll()
 */
static void dphy_config_pll(struct x5_dphy_priv *priv, struct pll_info *info)
{
	u8 hsfreqrange, cfgfreqrange;
	u32 cfg0_val;

	/* Reset test interface */
	dphy_test_reset(priv);

	/* Get frequency range values */
	hsfreqrange = get_hsfreqrange(info->fout);
	cfgfreqrange = get_cfgfreqrange(priv->ref_clk_khz);

	cfg0_val = (hsfreqrange << 8) | cfgfreqrange;
	DPHY_LOG_DEBUG("hsfreqrange=0x%02x, cfgfreqrange=0x%02x, cfg0=0x%04x\n",
		  hsfreqrange, cfgfreqrange, cfg0_val);

	/* Write DPHY_CFG_0: hsfreqrange[15:8] | cfgfreqrange[7:0] */
	dphy_write(priv, DISP_DPHY_CFG_0, cfg0_val);

	/* Slew rate configuration */
	dphy_test_write(priv, SLEW_RATE_DDL_LOOP_CONF_CTRL, 0x00);
	dphy_test_write(priv, SLEW_RATE_FSM_OVERRIDE_CTRL, 0x02);

	/* PLL analog programming */
	dphy_test_write(priv, PLL_ANALOG_PROG_CTRL, 0x01);
	dphy_test_write(priv, LP_RX_BIAS_CTRL, 0x40);

	/* PLL configuration register */
	dphy_write(priv, DISP_DPHY_PLL_CFG, 0x11);

	/* PLL phase error and locking filters */
	dphy_test_write2(priv, PLL_PHASE_ERR_CTRL, 0x02, 0x80);
	dphy_test_write(priv, PLL_LOCKING_FILTER, 0x60);
	dphy_test_write(priv, PLL_UNLOCKING_FILTER, 0x03);
	dphy_test_write(priv, PLL_LOCK_DETECTOR_MODE_SEL, 0x02);

	/* Input/Loop divider ratio control */
	dphy_test_write(priv, PLL_INPUT_LOOP_DIVIDER_RATIO_CTRL, BIT(5) | BIT(4));

	/* Loop divider (dm) - split into two writes */
	dphy_test_write2(priv, PLL_LOOP_DIVIDER_RATIO,
					 BIT(7) | ((info->dm & GENMASK(9, 5)) >> 5),
					 info->dm & GENMASK(4, 0));

	/* Input divider (dn) */
	dphy_test_write(priv, PLL_INPUT_DIVIDER_RATIO, info->dn & GENMASK(3, 0));

	/* VCO control */
	dphy_test_write(priv, PLL_VCO_CTRL, get_vco_ctrl(info->fout));

	/* Charge pump configuration */
	dphy_test_write(priv, PLL_CHARGE_PUMP_BIAS_CTRL, 0x10);
	dphy_test_write(priv, PLL_GMP_CTRL, 0x00);
	dphy_test_write(priv, PLL_INT_CHARGE_PUMP_CTRL, 0x00);

	/* Proportional charge pump control - depends on frequency */
	dphy_test_write(priv, PLL_PROP_CHARGE_PUMP_CTRL,
					(info->fout > 1150 * KHZ) ? 0x0E : 0x0D);

	/*
	 * Configure HS timing (matching kernel with dbg_write_en=1)
	 * This is critical for proper MIPI DSI operation!
	 */
	dphy_config_hs_timing(priv, info->fout * KHZ * 2); /* fout is half of hs_clk_rate */

	/*
	 * Configure additional DPHY registers to match kernel values
	 * These registers are at offset 0x2c and 0x30 from DPHY base
	 * Kernel values: 0x12, Uboot default: 0x08
	 * These appear to be timing-related registers
	 */
	writel(0x12, priv->base + 0x2c); /* DPHY_0x54 in absolute address */
	writel(0x12, priv->base + 0x30); /* DPHY_0x58 in absolute address */
	DPHY_LOG_DEBUG("Set reg 0x2c=0x12, 0x30=0x12 (matching kernel)\n");

	DPHY_LOG_DEBUG("PLL configured for %u KHz output\n", info->fout);
}

/*
 * Calculate HS clock rate from display timing
 *
 * Formula: hs_clk_rate = pixel_clock * bpp / lanes
 *
 * @pixel_clock: Pixel clock in Hz
 * @bpp: Bits per pixel (24 for RGB888, 18 for RGB666, 16 for RGB565)
 * @lanes: Number of data lanes (1-4)
 *
 * Returns: HS clock rate in Hz
 */
unsigned long x5_dphy_calc_hs_clk_rate(unsigned long pixel_clock,
									   int bpp, int lanes)
{
	unsigned long hs_clk_rate;

	if (lanes == 0)
		lanes = 4; /* Default to 4 lanes */

	hs_clk_rate = pixel_clock * bpp / lanes;

	DPHY_LOG_DEBUG("pixel_clock=%lu Hz, bpp=%d, lanes=%d -> hs_clk_rate=%lu Hz\n",
		  pixel_clock, bpp, lanes, hs_clk_rate);

	return hs_clk_rate;
}

/*
 * Initialize and configure DPHY
 *
 * @base: DPHY register base address
 * @hs_clk_rate: Target HS clock rate in Hz (calculated from timing)
 *
 * Returns: 0 on success, negative error code on failure
 */
int x5_dphy_configure(void __iomem *base, unsigned long hs_clk_rate)
{
	struct x5_dphy_priv *priv = &g_dphy_priv;
	struct pll_info info;

	if (!base)
	{
		DPHY_LOG_ERROR("Invalid base address\n");
		return -EINVAL;
	}

	if (hs_clk_rate < MIN_LINK_RATE || hs_clk_rate > MAX_LINK_RATE * 2)
	{
		DPHY_LOG_ERROR("HS clock rate %lu Hz out of range\n", hs_clk_rate);
		return -EINVAL;
	}

	priv->base = base;
	priv->ref_clk_khz = PLL_REF_CLK_KHZ;
	priv->hs_clk_rate = hs_clk_rate;
	priv->lane_mbps = hs_clk_rate / 1000000;

	DPHY_LOG_INFO("Configuring for HS clock rate %lu Hz (%u Mbps per lane)\n",
		   hs_clk_rate, priv->lane_mbps);

	/* Calculate PLL parameters */
	info.input = priv->ref_clk_khz;
	info.fout = hs_clk_rate / KHZ / 2; /* PLL output is half of HS rate (DDR) */

	find_best_pll_params(&info);

	/* Configure PLL */
	dphy_config_pll(priv, &info);

	/*
	 * Wait for PLL to stabilize
	 * Note: PLL lock status is checked by DSI Host via DSI_PHY_STATUS register
	 * DPHY itself doesn't have a direct lock status register
	 * Typical lock time is ~10-50ms, we use 50ms to be safe
	 */
	mdelay(DPHY_PLL_STABILIZE_MS);

	/* Store actual lane rate */
	priv->lane_mbps = info.fout * 2 / 1000; /* Convert back to Mbps */

	DPHY_LOG_DEBUG("Configured, actual lane rate = %u Mbps\n", priv->lane_mbps);

	return 0;
}

/*
 * Get configured lane rate in Mbps
 */
unsigned int x5_dphy_get_lane_mbps(void)
{
	return g_dphy_priv.lane_mbps;
}

/*
 * Dump DPHY registers for debugging
 */
void x5_dphy_dump_regs(void)
{
	struct x5_dphy_priv *priv = &g_dphy_priv;
	u32 val;

	if (!priv->base)
	{
		DPHY_LOG_DEBUG("Not initialized\n");
		return;
	}

	DPHY_LOG_DEBUG("\n=== DPHY Register Dump (Base: 0x%p) ===\n", priv->base);

	/* DPHY_CFG_0 @ offset 0x04 */
	val = dphy_read(priv, DISP_DPHY_CFG_0);
	DPHY_LOG_DEBUG("DPHY_CFG_0 (0x04): 0x%08x\n", val);
	DPHY_LOG_DEBUG("  hsfreqrange: 0x%02x, cfgfreqrange: 0x%02x\n",
		   (val >> 8) & 0xFF, val & 0xFF);

	/* DPHY_PLL_CFG @ offset 0x08 */
	val = dphy_read(priv, DISP_DPHY_PLL_CFG);
	DPHY_LOG_DEBUG("DPHY_PLL_CFG (0x08): 0x%08x\n", val);

	/* PLL_STS_0 @ offset 0x14 (0x3c - 0x28 = 0x14) */
	val = readl(priv->base + 0x14);
	DPHY_LOG_DEBUG("DPHY_PLL_STS_0 (0x14): 0x%08x\n", val);

	/* PLL_STS_1 @ offset 0x18 (0x40 - 0x28 = 0x18) */
	val = readl(priv->base + 0x18);
	DPHY_LOG_DEBUG("DPHY_PLL_STS_1 (0x18): 0x%08x\n", val);

	/* TST_CTRL0 @ offset 0x24 */
	val = dphy_read(priv, DISP_PHY_TST_CTRL0);
	DPHY_LOG_DEBUG("PHY_TST_CTRL0 (0x24): 0x%08x\n", val);

	/* TST_CTRL1 @ offset 0x28 */
	val = dphy_read(priv, DISP_PHY_TST_CTRL1);
	DPHY_LOG_DEBUG("PHY_TST_CTRL1 (0x28): 0x%08x\n", val);

	DPHY_LOG_DEBUG("==========================================\n\n");
}

/*
 * Simple initialization with timing parameters
 *
 * @base: DPHY register base address
 * @pixel_clock: Pixel clock in Hz
 * @bpp: Bits per pixel
 * @lanes: Number of data lanes
 *
 * Returns: 0 on success, negative error code on failure
 */
int x5_dphy_init(void __iomem *base, unsigned long pixel_clock,
				 int bpp, int lanes)
{
	unsigned long hs_clk_rate;

	hs_clk_rate = x5_dphy_calc_hs_clk_rate(pixel_clock, bpp, lanes);

	return x5_dphy_configure(base, hs_clk_rate);
}
