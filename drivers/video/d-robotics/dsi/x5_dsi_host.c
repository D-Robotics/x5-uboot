// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 *
 * X5 DSI Host Driver
 *
 * This is a wrapper that integrates:
 * - U-Boot's generic dw_mipi_dsi driver
 * - X5 DPHY driver
 * - Panel communication
 *
 * State is held in a single struct x5_dsi_host instance: U-Boot drives one DSI
 * path during sequential boot bring-up (no parallel callers). This module is not
 * bound as a DM driver, so there is no separate udevice priv; use
 * x5_dsi_host_get() as the single accessor instead of scattering references to
 * the backing storage.
 */

#include <common.h>
#include <dm.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <video.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <fdtdec.h>
#include "x5_dsi_host.h"
#include "x5_snps_dphy.h"
#include <hb_display_log.h>

/* DSI Host Register Base (must match mipi_dsi0@3e060000 in arch/arm/dts/x5.dtsi) */
#define X5_DSI_BASE 0x3e060000

/* DSI Register Offsets (Synopsys DesignWare) - Corrected per user spec */
#define DSI_VERSION 0x00
#define DSI_PWR_UP 0x04
#define DSI_CLKMGR_CFG 0x08
#define DSI_DPI_VCID 0x0C
#define DSI_DPI_COLOR_CODING 0x10
#define DSI_DPI_CFG_POL 0x14
#define DSI_DPI_LP_CMD_TIM 0x18
#define DSI_DBI_VCID 0x1C
#define DSI_DBI_CFG 0x20
#define DSI_DBI_PARTITIONING_EN 0x24
#define DSI_DBI_CMDSIZE 0x28
#define DSI_PCKHDL_CFG 0x2C
#define DSI_GEN_VCID 0x30
#define DSI_MODE_CFG 0x34
#define DSI_VID_MODE_CFG 0x38
#define DSI_VID_PKT_SIZE 0x3C
#define DSI_VID_NUM_CHUNKS 0x40
#define DSI_VID_NULL_SIZE 0x44
#define DSI_VID_HSA_TIME 0x48
#define DSI_VID_HBP_TIME 0x4C
#define DSI_VID_HLINE_TIME 0x50
#define DSI_VID_VSA_LINES 0x54
#define DSI_VID_VBP_LINES 0x58
#define DSI_VID_VFP_LINES 0x5C
#define DSI_VID_VACTIVE_LINES 0x60
#define DSI_EDPI_CMD_SIZE 0x64
#define DSI_CMD_MODE_CFG 0x68
#define DSI_GEN_HDR 0x6C
#define DSI_GEN_PLD_DATA 0x70
#define DSI_CMD_PKT_STATUS 0x74
#define DSI_TO_CNT_CFG 0x78
#define DSI_HS_RD_TO_CNT 0x7C
#define DSI_LP_RD_TO_CNT 0x80
#define DSI_HS_WR_TO_CNT 0x84
#define DSI_LP_WR_TO_CNT 0x88
#define DSI_BTA_TO_CNT 0x8C
#define DSI_SDF_3D 0x90
#define DSI_LPCLK_CTRL 0x94
#define DSI_PHY_TMR_LPCLK_CFG 0x98
#define DSI_PHY_TMR_CFG 0x9C
#define DSI_PHY_RSTZ 0xA0 /* CRITICAL: Corrected from 0xA8 */
#define DSI_PHY_IF_CFG 0xA4
#define DSI_PHY_ULPS_CTRL 0xA8
#define DSI_PHY_TX_TRIGGERS 0xAC
#define DSI_PHY_STATUS 0xB0 /* CRITICAL: Corrected from 0xB4 */
#define DSI_INT_ST0 0xBC
#define DSI_INT_ST1 0xC0
#define DSI_INT_MSK0 0xC4
#define DSI_INT_MSK1 0xC8
#define DSI_PHY_TMR_RD_CFG 0xF4 /* Note: This is at 0xF4, not 0xA0 */

/* DSI_PWR_UP bits */
#define RESET 0
#define POWERUP BIT(0)

/* DSI_MODE_CFG bits */
#define ENABLE_VIDEO_MODE 0
#define ENABLE_CMD_MODE BIT(0)

/* DSI_VID_MODE_CFG bits */
#define VID_MODE_TYPE_BURST 0x02
#define VID_MODE_TYPE_NON_BURST_SYNC_EVENTS 0x01
#define VID_MODE_TYPE_NON_BURST_SYNC_PULSES 0x00
#define ENABLE_LOW_POWER (0x3F << 8)

/* DSI_LPCLK_CTRL bits */
#define PHY_TXREQUESTCLKHS BIT(0)
#define AUTO_CLKLANE_CTRL BIT(1)

/* DSI_PHY_RSTZ bits */
#define PHY_ENFORCEPLL BIT(3)
#define PHY_ENABLECLK BIT(2)
#define PHY_UNRSTZ BIT(1)
#define PHY_UNSHUTDOWNZ BIT(0)

/* DSI_PHY_STATUS bits */
#define PHY_LOCK BIT(0)
#define PHY_STOP_STATE_CLK_LANE BIT(2)

/* DSI_CMD_PKT_STATUS bits */
#define GEN_CMD_EMPTY BIT(0)
#define GEN_CMD_FULL BIT(1)
#define GEN_PLD_W_EMPTY BIT(2)
#define GEN_PLD_W_FULL BIT(3)
#define GEN_PLD_R_EMPTY BIT(4)
#define GEN_RD_CMD_BUSY BIT(6)

/* DPI Color Coding */
#define DPI_COLOR_CODING_16BIT_1 0x0
#define DPI_COLOR_CODING_16BIT_2 0x1
#define DPI_COLOR_CODING_16BIT_3 0x2
#define DPI_COLOR_CODING_18BIT_1 0x3
#define DPI_COLOR_CODING_18BIT_2 0x4
#define DPI_COLOR_CODING_24BIT 0x5
#define LOOSELY18_EN BIT(8)

/* Command mode LP settings */
#define CMD_MODE_ALL_LP (BIT(24) | BIT(19) | BIT(18) | BIT(17) | BIT(16) | \
						 BIT(14) | BIT(13) | BIT(12) | BIT(11) | BIT(10) | \
						 BIT(9) | BIT(8))

/* Timeout values */
#define PHY_STATUS_TIMEOUT_US 10000
#define CMD_PKT_STATUS_TIMEOUT_US 20000

/* DSI_CLKMGR_CFG: TO_CLK_DIVISION (bits 15:8), TX_ESC_CLK_DIVISION (7:0) — kernel match */
#define DSI_CLKMGR_TO_CLK_DIVISION	100
#define DSI_CLKMGR_TO_DIV_SHIFT		8
#define DSI_ESC_CLK_TARGET_MAX_MHZ	20
#define DSI_CLKMGR_TX_ESC_DIV_MIN	3

/* DPI polarity / LP command timing (DSI_DPI_CFG_POL, DSI_DPI_LP_CMD_TIM) */
#define DPI_CFG_POL_VSYNC_ACTIVE_LOW	BIT(1)
#define DPI_CFG_POL_HSYNC_ACTIVE_LOW	BIT(2)
#define DPI_LP_CMD_TIM_HS_SHIFT		16
#define DPI_LP_CMD_TIM_HS_CLK		4
#define DPI_LP_CMD_TIM_LP_CLK		4

/* PHY timing register composites (match kernel dw_mipi_dsi) */
#define DSI_PHY_TMR_LPCLK_HI_SHIFT	16
#define DSI_PHY_TMR_LPCLK_HI_VAL	2
#define DSI_PHY_TMR_LPCLK_LO_VAL	17
#define DSI_PHY_TMR_CFG_HI_SHIFT	16
#define DSI_PHY_TMR_CFG_HI_VAL		3
#define DSI_PHY_TMR_CFG_LO_VAL		10
#define DSI_PHY_TMR_RD_TIMEOUT_VAL	0x00002710

/* DSI_PCKHDL_CFG: CRC_RX | ECC_RX | BTA enable */
#define PCKHDL_CRC_RX_EN		BIT(4)
#define PCKHDL_ECC_RX_EN		BIT(3)
#define PCKHDL_BTA_EN			BIT(2)

/* DSI_TO_CNT_CFG: HSTX_TO_CNT | LPRX_TO_CNT */
#define DSI_HSTX_TO_CNT			0x21C8
#define DSI_LPRX_TO_CNT			0x03E8
#define DSI_TO_CNT_HSTX_SHIFT		16

#define DSI_BTA_TO_CNT_VAL		0xd00

/* DSI_PHY_IF_CFG: PHY_STOP_WAIT_TIME (15:8), N_LANES (2:0) */
#define DSI_PHY_STOP_WAIT_TIME		0x20
#define DSI_PHY_STOP_WAIT_SHIFT		8

/* DSI Host private data */
struct x5_dsi_host
{
	void __iomem *base;
	void __iomem *dphy_base;
	unsigned int lanes;
	unsigned int channel;
	unsigned int format;
	unsigned int mode_flags;
	unsigned int lane_mbps;
	struct display_timing timing;
};

static struct x5_dsi_host g_dsi_host;

static inline struct x5_dsi_host *x5_dsi_host_get(void)
{
	return &g_dsi_host;
}

/*
 * Register access helpers
 */
static inline void dsi_write(struct x5_dsi_host *dsi, u32 reg, u32 val)
{
	writel(val, dsi->base + reg);
}

static inline u32 dsi_read(struct x5_dsi_host *dsi, u32 reg)
{
	return readl(dsi->base + reg);
}

/*
 * Wait for PHY lock
 */
static int dsi_wait_phy_lock(struct x5_dsi_host *dsi)
{
	u32 val;
	int timeout = PHY_STATUS_TIMEOUT_US;

	while (timeout > 0)
	{
		val = dsi_read(dsi, DSI_PHY_STATUS);
		if (val & PHY_LOCK)
			return 0;
		udelay(10);
		timeout -= 10;
	}

	DSI_LOG_ERROR("PHY lock timeout\n");
	return -ETIMEDOUT;
}

/*
 * Wait for command FIFO empty
 */
static int dsi_wait_cmd_fifo_empty(struct x5_dsi_host *dsi)
{
	u32 val;
	int timeout = CMD_PKT_STATUS_TIMEOUT_US;

	while (timeout > 0)
	{
		val = dsi_read(dsi, DSI_CMD_PKT_STATUS);
		if ((val & GEN_CMD_EMPTY) && (val & GEN_PLD_W_EMPTY))
			return 0;
		udelay(10);
		timeout -= 10;
	}

	DSI_LOG_ERROR("Command FIFO timeout\n");
	return -ETIMEDOUT;
}

/*
 * Get lane byte clock cycles for horizontal timing component
 */
static u32 dsi_get_hcomponent_lbcc(struct x5_dsi_host *dsi, u32 hcomponent)
{
	u32 lbcc;
	u32 pixel_clock_khz;

	if (!dsi->lane_mbps) {
		DSI_LOG_ERROR("dsi_get_hcomponent_lbcc: lane_mbps is 0\n");
		return 0;
	}

	pixel_clock_khz = dsi->timing.pixelclock.typ / 1000;
	if (!pixel_clock_khz) {
		DSI_LOG_ERROR("dsi_get_hcomponent_lbcc: pixel clock too small or zero\n");
		return 0;
	}

	lbcc = hcomponent * dsi->lane_mbps * 1000 / 8;
	lbcc = lbcc / pixel_clock_khz;

	return lbcc;
}

/*
 * Initialize DSI controller
 * Match kernel: DSI_CLKMGR_CFG = 0x00006403 = (100 << 8) | 3
 * TO_CLK_DIVISION = 100 (bits 15:8), TX_ESC_CLK_DIVISION = 3 (bits 7:0)
 */
static void dsi_init(struct x5_dsi_host *dsi)
{
	u32 esc_clk_division;

	/* Reset DSI */
	dsi_write(dsi, DSI_PWR_UP, RESET);

	/*
	 * Calculate escape clock division
	 * ESC clock should be < 20MHz
	 * ESC clock = lane_byte_clk / esc_clk_division
	 * lane_byte_clk = lane_mbps / 8
	 *
	 * For 390 Mbps: lane_byte_clk = 390/8 = 48.75 MHz
	 * esc_clk_division = 48.75 / 20 + 1 = 3.4 -> 3
	 *
	 * Match kernel: TO_CLK_DIVISION = 100, TX_ESC_CLK_DIVISION = 3
	 */
	esc_clk_division = (dsi->lane_mbps / 8) / DSI_ESC_CLK_TARGET_MAX_MHZ + 1;
	if (esc_clk_division < DSI_CLKMGR_TX_ESC_DIV_MIN)
		esc_clk_division = DSI_CLKMGR_TX_ESC_DIV_MIN;

	/* Match kernel value: (TO_DIV << 8) | esc_clk_division */
	dsi_write(dsi, DSI_CLKMGR_CFG,
		  (DSI_CLKMGR_TO_CLK_DIVISION << DSI_CLKMGR_TO_DIV_SHIFT) |
		  esc_clk_division);

	DSI_LOG_DEBUG("esc_clk_division=%u, CLKMGR_CFG=0x%08x\n",
		  esc_clk_division,
		  (DSI_CLKMGR_TO_CLK_DIVISION << DSI_CLKMGR_TO_DIV_SHIFT) |
		  esc_clk_division);
}

/*
 * Configure DPI interface
 */
static void dsi_dpi_config(struct x5_dsi_host *dsi)
{
	u32 color = DPI_COLOR_CODING_24BIT; /* Default RGB888 */
	u32 pol = 0;

	/* Set color coding based on format */
	switch (dsi->format)
	{
	case MIPI_DSI_FMT_RGB888:
		color = DPI_COLOR_CODING_24BIT;
		break;
	case MIPI_DSI_FMT_RGB666:
		color = DPI_COLOR_CODING_18BIT_2 | LOOSELY18_EN;
		break;
	case MIPI_DSI_FMT_RGB666_PACKED:
		color = DPI_COLOR_CODING_18BIT_1;
		break;
	case MIPI_DSI_FMT_RGB565:
		color = DPI_COLOR_CODING_16BIT_1;
		break;
	}

	/*
	 * Polarity configuration - match kernel value 0x06
	 * bit 1 = VSYNC_ACTIVE_LOW = 1
	 * bit 2 = HSYNC_ACTIVE_LOW = 1
	 * This matches the panel's negative sync polarity
	 */
	pol = DPI_CFG_POL_VSYNC_ACTIVE_LOW | DPI_CFG_POL_HSYNC_ACTIVE_LOW;

	dsi_write(dsi, DSI_DPI_VCID, dsi->channel);
	dsi_write(dsi, DSI_DPI_COLOR_CODING, color);
	dsi_write(dsi, DSI_DPI_CFG_POL, pol);
	dsi_write(dsi, DSI_DPI_LP_CMD_TIM,
		  (DPI_LP_CMD_TIM_HS_CLK << DPI_LP_CMD_TIM_HS_SHIFT) |
		  DPI_LP_CMD_TIM_LP_CLK);
}

/*
 * Configure video mode
 */
static void dsi_video_mode_config(struct x5_dsi_host *dsi)
{
	u32 val;

	/*
	 * Use burst mode with LP transitions enabled
	 * Kernel value: 0x3F00 = ENABLE_LOW_POWER (bits 13:8) + burst mode (bits 1:0 = 0)
	 * Note: bits 1:0 = 0 means non-burst sync pulses, not burst mode
	 * Let's match kernel exactly: 0x3F00
	 */
	val = ENABLE_LOW_POWER; /* 0x3F00 - no burst mode bits set */

	dsi_write(dsi, DSI_VID_MODE_CFG, val);
}

/*
 * Configure video packet
 */
static void dsi_video_packet_config(struct x5_dsi_host *dsi)
{
	dsi_write(dsi, DSI_VID_PKT_SIZE, dsi->timing.hactive.typ);
	/* Kernel uses 1 chunk, not 0 */
	dsi_write(dsi, DSI_VID_NUM_CHUNKS, 1);
	dsi_write(dsi, DSI_VID_NULL_SIZE, 0);
}

/*
 * Configure video timing
 */
static void dsi_timing_config(struct x5_dsi_host *dsi)
{
	u32 htotal, hsa, hbp, lbcc;

	htotal = dsi->timing.hactive.typ + dsi->timing.hfront_porch.typ +
			 dsi->timing.hback_porch.typ + dsi->timing.hsync_len.typ;
	hsa = dsi->timing.hsync_len.typ;
	hbp = dsi->timing.hback_porch.typ;

	/* Horizontal timing in lane byte clock cycles */
	lbcc = dsi_get_hcomponent_lbcc(dsi, htotal);
	dsi_write(dsi, DSI_VID_HLINE_TIME, lbcc);

	lbcc = dsi_get_hcomponent_lbcc(dsi, hsa);
	dsi_write(dsi, DSI_VID_HSA_TIME, lbcc);

	lbcc = dsi_get_hcomponent_lbcc(dsi, hbp);
	dsi_write(dsi, DSI_VID_HBP_TIME, lbcc);

	/* Vertical timing in lines */
	dsi_write(dsi, DSI_VID_VACTIVE_LINES, dsi->timing.vactive.typ);
	dsi_write(dsi, DSI_VID_VSA_LINES, dsi->timing.vsync_len.typ);
	dsi_write(dsi, DSI_VID_VFP_LINES, dsi->timing.vfront_porch.typ);
	dsi_write(dsi, DSI_VID_VBP_LINES, dsi->timing.vback_porch.typ);

	DSI_LOG_DEBUG("htotal=%u, hsa=%u, hbp=%u\n", htotal, hsa, hbp);
	DSI_LOG_DEBUG("vactive=%u, vsa=%u, vfp=%u, vbp=%u\n",
		  dsi->timing.vactive.typ, dsi->timing.vsync_len.typ,
		  dsi->timing.vfront_porch.typ, dsi->timing.vback_porch.typ);
}

/*
 * Configure PHY timing
 * Match kernel values:
 * DSI_PHY_TMR_LPCLK: 0x00020011 = (2 << 16) | 17
 * DSI_PHY_TMR_CFG: 0x0003000A = (3 << 16) | 10
 * DSI_PHY_TMR_RD_CFG: 0x00002710 = 10000 (read timeout)
 */
static void dsi_phy_timing_config(struct x5_dsi_host *dsi)
{
	dsi_write(dsi, DSI_PHY_TMR_LPCLK_CFG,
		  (DSI_PHY_TMR_LPCLK_HI_VAL << DSI_PHY_TMR_LPCLK_HI_SHIFT) |
		  DSI_PHY_TMR_LPCLK_LO_VAL);
	dsi_write(dsi, DSI_PHY_TMR_CFG,
		  (DSI_PHY_TMR_CFG_HI_VAL << DSI_PHY_TMR_CFG_HI_SHIFT) |
		  DSI_PHY_TMR_CFG_LO_VAL);
	/*
	 * Match kernel DSI_PHY_TMR_RD_CFG (max time for PHY read operations).
	 */
	dsi_write(dsi, DSI_PHY_TMR_RD_CFG, DSI_PHY_TMR_RD_TIMEOUT_VAL);
}

/*
 * Configure PHY interface
 * Match kernel: DSI_PHY_IF_CFG = 0x00002003 = (0x20 << 8) | 3
 * PHY_STOP_WAIT_TIME = 0x20 (32), N_LANES = 3 (4 lanes - 1)
 */
static void dsi_phy_interface_config(struct x5_dsi_host *dsi)
{
	dsi_write(dsi, DSI_PHY_IF_CFG,
		  (DSI_PHY_STOP_WAIT_TIME << DSI_PHY_STOP_WAIT_SHIFT) |
		  (dsi->lanes - 1));
}

/*
 * Initialize and enable DPHY
 *
 * CRITICAL: Following kernel sequence from dw_mipi_dsi_dphy_init():
 * 1. Put PHY in reset state BEFORE configuring PLL
 * 2. Configure DPHY PLL (includes 500ms delay)
 *
 * The PHY must be in reset state (DSI_PHY_RSTZ = 0) during PLL configuration!
 */
static int dsi_dphy_init(struct x5_dsi_host *dsi)
{
	int ret;
	int bpp;

	/* Get bits per pixel */
	switch (dsi->format)
	{
	case MIPI_DSI_FMT_RGB888:
		bpp = 24;
		break;
	case MIPI_DSI_FMT_RGB666:
	case MIPI_DSI_FMT_RGB666_PACKED:
		bpp = 18;
		break;
	case MIPI_DSI_FMT_RGB565:
		bpp = 16;
		break;
	default:
		bpp = 24;
	}

	/*
	 * CRITICAL: Put PHY in reset state BEFORE configuring PLL
	 * From kernel dw_mipi_dsi_dphy_init():
	 *   dw_write(dsi, DSI_PHY_RSTZ, PHY_DISFORCEPLL | PHY_DISABLECLK | PHY_RSTZ | PHY_SHUTDOWNZ);
	 * All these bits are 0, so write 0 to put PHY in reset state
	 */
	DSI_LOG_DEBUG("Putting PHY in reset state before PLL configuration\n");
	dsi_write(dsi, DSI_PHY_RSTZ, 0);
	udelay(10);

	/* Initialize DPHY with timing parameters (includes 500ms delay) */
	ret = x5_dphy_init(dsi->dphy_base, dsi->timing.pixelclock.typ,
					   bpp, dsi->lanes);
	if (ret)
	{
		DSI_LOG_ERROR("DPHY init failed\n");
		return ret;
	}

	/* Dump DPHY registers after configuration */
	x5_dphy_dump_regs();

	return 0;
}

/*
 * Enable DPHY
 *
 * From kernel dw_mipi_dsi_dphy_enable():
 * PHY configuration has been done in reset state, now enable it with a single write
 */
static int dsi_dphy_enable(struct x5_dsi_host *dsi)
{
	int ret;

	/*
	 * Enable PHY with a single write (matching kernel behavior)
	 * From kernel: dw_write(dsi, DSI_PHY_RSTZ, PHY_ENFORCEPLL | PHY_ENABLECLK | PHY_UNRSTZ | PHY_UNSHUTDOWNZ);
	 * = BIT(3) | BIT(2) | BIT(1) | BIT(0) = 0x0F
	 */
	DSI_LOG_DEBUG("Enabling PHY (DSI_PHY_RSTZ = 0x0F)\n");
	dsi_write(dsi, DSI_PHY_RSTZ, PHY_ENFORCEPLL | PHY_ENABLECLK | PHY_UNRSTZ | PHY_UNSHUTDOWNZ);

	/* Wait for PHY lock */
	ret = dsi_wait_phy_lock(dsi);
	if (ret)
		return ret;

	DSI_LOG_DEBUG("PHY locked, PHY_STATUS = 0x%08x\n", dsi_read(dsi, DSI_PHY_STATUS));

	return 0;
}

/*
 * Set DSI mode (video or command)
 *
 * Match kernel dw_mipi_dsi_set_mode():
 * 1. Reset DSI (PWR_UP = 0)
 * 2. Set mode
 * 3. Configure LPCLK_CTRL for video mode
 * 4. Power up DSI (PWR_UP = 1)
 */
static void dsi_set_mode(struct x5_dsi_host *dsi, int video_mode)
{
	/* Reset DSI first (kernel does this) */
	dsi_write(dsi, DSI_PWR_UP, RESET);

	if (video_mode)
	{
		dsi_write(dsi, DSI_MODE_CFG, ENABLE_VIDEO_MODE);
		/* Kernel uses PHY_TXREQUESTCLKHS only (0x01), not AUTO_CLKLANE_CTRL */
		dsi_write(dsi, DSI_LPCLK_CTRL, PHY_TXREQUESTCLKHS);
	}
	else
	{
		dsi_write(dsi, DSI_MODE_CFG, ENABLE_CMD_MODE);
		dsi_write(dsi, DSI_LPCLK_CTRL, 0);
	}

	/* Power up DSI */
	dsi_write(dsi, DSI_PWR_UP, POWERUP);
}

/*
 * Power up DSI
 */
static void dsi_power_up(struct x5_dsi_host *dsi)
{
	dsi_write(dsi, DSI_PWR_UP, POWERUP);
}

/*
 * Send DCS short write command (no parameter)
 */
int x5_dsi_dcs_write_0(u8 cmd)
{
	struct x5_dsi_host *dsi = x5_dsi_host_get();
	u32 val;
	int ret;

	/* Wait for FIFO */
	ret = dsi_wait_cmd_fifo_empty(dsi);
	if (ret)
		return ret;

	/* DCS short write, no parameter: data type = 0x05 */
	val = (cmd << 8) | 0x05;
	dsi_write(dsi, DSI_GEN_HDR, val);

	return dsi_wait_cmd_fifo_empty(dsi);
}

/*
 * Send DCS short write command (1 parameter)
 */
int x5_dsi_dcs_write_1(u8 cmd, u8 param)
{
	struct x5_dsi_host *dsi = x5_dsi_host_get();
	u32 val;
	int ret;

	/* Wait for FIFO */
	ret = dsi_wait_cmd_fifo_empty(dsi);
	if (ret)
		return ret;

	/* DCS short write, 1 parameter: data type = 0x15 */
	val = (param << 16) | (cmd << 8) | 0x15;
	dsi_write(dsi, DSI_GEN_HDR, val);

	return dsi_wait_cmd_fifo_empty(dsi);
}

/*
 * Send DCS long write command
 */
int x5_dsi_dcs_write_buffer(const u8 *data, size_t len)
{
	struct x5_dsi_host *dsi = x5_dsi_host_get();
	u32 val;
	int ret;
	size_t i;

	if (len == 0)
		return 0;
	if (!data)
		return -EINVAL;

	/* Wait for FIFO */
	ret = dsi_wait_cmd_fifo_empty(dsi);
	if (ret)
		return ret;

	/* Write payload data */
	for (i = 0; i < len; i += 4)
	{
		val = 0;
		if (i < len)
			val |= data[i];
		if (i + 1 < len)
			val |= data[i + 1] << 8;
		if (i + 2 < len)
			val |= data[i + 2] << 16;
		if (i + 3 < len)
			val |= data[i + 3] << 24;
		dsi_write(dsi, DSI_GEN_PLD_DATA, val);
	}

	/* DCS long write: data type = 0x39 */
	val = (len << 8) | 0x39;
	dsi_write(dsi, DSI_GEN_HDR, val);

	return dsi_wait_cmd_fifo_empty(dsi);
}

/*
 * Initialize DSI Host
 *
 * Following kernel dw_mipi_dsi_mode_set() sequence:
 * 1. dw_mipi_dsi_init() - Reset DSI, configure clkmgr
 * 2. dw_mipi_dsi_dpi_config() - DPI config
 * 3. dw_mipi_dsi_packet_handler_config() - Packet handler
 * 4. dw_mipi_dsi_video_mode_config() - Video mode
 * 5. dw_mipi_dsi_video_packet_config() - Video packet
 * 6. dw_mipi_dsi_command_mode_config() - Command mode (timeout config)
 * 7. dw_mipi_dsi_timing_config() - Timing
 * 8. dw_mipi_dsi_dphy_init() - PHY reset + PLL config
 * 9. dw_mipi_dsi_dphy_timing_config() - PHY timing
 * 10. dw_mipi_dsi_dphy_interface_config() - PHY interface
 * 11. Wait for 2 frames
 * 12. Set command mode
 * Then dw_mipi_dsi_dphy_enable() is called separately
 *
 * Note: In kernel, lane_link_rate is calculated in bridge_mode_fixup() before
 * mode_set(). We calculate it first from pixel clock and bpp.
 */
int x5_dsi_host_init(struct display_timing *timing, int lanes, int format)
{
	struct x5_dsi_host *dsi = x5_dsi_host_get();
	int ret;
	int bpp;

	DSI_LOG_INFO("Host: Initializing\n");

	if (!timing)
		return -EINVAL;

	/* Setup private data */
	dsi->base = (void __iomem *)X5_DSI_BASE;
	dsi->dphy_base = (void __iomem *)X5_DPHY_BASE;
	dsi->lanes = lanes ? lanes : 4;
	dsi->channel = 0;
	dsi->format = format ? format : MIPI_DSI_FMT_RGB888;
	memcpy(&dsi->timing, timing, sizeof(*timing));

	/* Calculate bpp */
	switch (dsi->format)
	{
	case MIPI_DSI_FMT_RGB888:
		bpp = 24;
		break;
	case MIPI_DSI_FMT_RGB666:
	case MIPI_DSI_FMT_RGB666_PACKED:
		bpp = 18;
		break;
	case MIPI_DSI_FMT_RGB565:
		bpp = 16;
		break;
	default:
		bpp = 24;
	}

	/*
	 * Pre-calculate lane_mbps before DSI init (like kernel bridge_mode_fixup)
	 * Formula: hs_clk_rate = pixel_clock * bpp / lanes
	 * lane_mbps = hs_clk_rate / 1000000
	 */
	dsi->lane_mbps = (unsigned int)((unsigned long long)dsi->timing.pixelclock.typ * bpp / dsi->lanes / 1000000);
	DSI_LOG_DEBUG("Pre-calculated lane_mbps = %u (pixel_clk=%u, bpp=%d, lanes=%d)\n",
		   dsi->lane_mbps, dsi->timing.pixelclock.typ, bpp, dsi->lanes);

	/* Read DSI version */
	DSI_LOG_DEBUG("Version = 0x%08x\n", dsi_read(dsi, DSI_VERSION));

	/* Step 1: Initialize DSI controller (reset + clkmgr) */
	dsi_init(dsi);

	/* Step 2: Configure DPI interface */
	dsi_dpi_config(dsi);

	/* Step 3: Packet handler config (from kernel) */
	dsi_write(dsi, DSI_PCKHDL_CFG,
		  PCKHDL_CRC_RX_EN | PCKHDL_ECC_RX_EN | PCKHDL_BTA_EN);

	/* Step 4: Configure video mode */
	dsi_video_mode_config(dsi);

	/* Step 5: Configure video packet */
	dsi_video_packet_config(dsi);

	/* Step 6: Configure command mode timeout (from kernel) */
	dsi_write(dsi, DSI_TO_CNT_CFG,
		  (DSI_HSTX_TO_CNT << DSI_TO_CNT_HSTX_SHIFT) | DSI_LPRX_TO_CNT);
	dsi_write(dsi, DSI_BTA_TO_CNT, DSI_BTA_TO_CNT_VAL);

	/* Step 7: Configure timing */
	dsi_timing_config(dsi);

	/* Step 8: Initialize DPHY (puts PHY in reset, configures PLL with 500ms delay) */
	ret = dsi_dphy_init(dsi);
	if (ret)
		return ret;

	/* Update lane_mbps with actual value from DPHY */
	dsi->lane_mbps = x5_dphy_get_lane_mbps();

	/* Step 9: Configure PHY timing */
	dsi_phy_timing_config(dsi);

	/* Step 10: Configure PHY interface */
	dsi_phy_interface_config(dsi);

	/* Step 11: Wait for 2 frames (at 60Hz, ~33ms)
	 * Reduced to 20ms for faster boot */
	DSI_LOG_DEBUG("Waiting for 2 frames (~20ms)\n");
	mdelay(20);

	/* Step 12: Enable DPHY */
	ret = dsi_dphy_enable(dsi);
	if (ret)
		return ret;

	/* Power up DSI */
	dsi_power_up(dsi);

	/* Start in command mode for panel init */
	dsi_set_mode(dsi, 0);

	/* Configure command mode for LP transmission */
	dsi_write(dsi, DSI_CMD_MODE_CFG, CMD_MODE_ALL_LP);

	DSI_LOG_INFO("Host: Initialized, lane_mbps=%u\n", dsi->lane_mbps);

	return 0;
}

/*
 * Enable video mode output
 *
 * CRITICAL: Must clear DSI_CMD_MODE_CFG when switching to video mode!
 * Kernel sets DSI_CMD_MODE_CFG = 0 in video mode.
 */
void x5_dsi_host_enable(void)
{
	struct x5_dsi_host *dsi = x5_dsi_host_get();

	DSI_LOG_INFO("Host: Enabling video mode\n");

	/*
	 * CRITICAL FIX: Clear CMD_MODE_CFG before switching to video mode
	 * Kernel value in video mode: 0x00000000
	 * This was causing the display to not work!
	 */
	dsi_write(dsi, DSI_CMD_MODE_CFG, 0);

	/* Switch to video mode */
	dsi_set_mode(dsi, 1);
}

/*
 * Disable DSI Host
 */
void x5_dsi_host_disable(void)
{
	struct x5_dsi_host *dsi = x5_dsi_host_get();

	/* Switch to command mode */
	dsi_set_mode(dsi, 0);

	/* Power down */
	dsi_write(dsi, DSI_PWR_UP, RESET);
	dsi_write(dsi, DSI_PHY_RSTZ, 0);
}
