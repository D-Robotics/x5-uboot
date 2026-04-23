/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 *
 * X5 Synopsys MIPI DPHY TX Driver Header
 */

#ifndef __X5_SNPS_DPHY_H__
#define __X5_SNPS_DPHY_H__

/* DPHY MMIO base — must match dphy0@3e0a0028 reg in arch/arm/dts/x5.dtsi */
#define X5_DPHY_BASE 0x3e0a0028

/* Bits per pixel for different formats */
#define MIPI_DSI_FMT_RGB888_BPP 24
#define MIPI_DSI_FMT_RGB666_BPP 18
#define MIPI_DSI_FMT_RGB565_BPP 16

/**
 * x5_dphy_calc_hs_clk_rate - Calculate HS clock rate from display parameters
 *
 * @pixel_clock: Pixel clock in Hz
 * @bpp: Bits per pixel (24 for RGB888, 18 for RGB666, 16 for RGB565)
 * @lanes: Number of data lanes (1-4)
 *
 * Returns: HS clock rate in Hz
 */
unsigned long x5_dphy_calc_hs_clk_rate(unsigned long pixel_clock,
									   int bpp, int lanes);

/**
 * x5_dphy_configure - Configure DPHY with specific HS clock rate
 *
 * @base: DPHY register base address
 * @hs_clk_rate: Target HS clock rate in Hz
 *
 * Returns: 0 on success, negative error code on failure
 */
int x5_dphy_configure(void __iomem *base, unsigned long hs_clk_rate);

/**
 * x5_dphy_init - Initialize DPHY with display timing parameters
 *
 * @base: DPHY register base address
 * @pixel_clock: Pixel clock in Hz
 * @bpp: Bits per pixel
 * @lanes: Number of data lanes
 *
 * Returns: 0 on success, negative error code on failure
 */
int x5_dphy_init(void __iomem *base, unsigned long pixel_clock,
				 int bpp, int lanes);

/**
 * x5_dphy_get_lane_mbps - Get configured lane rate
 *
 * Returns: Lane rate in Mbps
 */
unsigned int x5_dphy_get_lane_mbps(void);

/**
 * x5_dphy_dump_regs - Dump DPHY registers for debugging
 */
void x5_dphy_dump_regs(void);

#endif /* __X5_SNPS_DPHY_H__ */
