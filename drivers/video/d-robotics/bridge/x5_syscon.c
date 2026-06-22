// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 * Modified by: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * X5 SYSCON Bridge Driver - Routes DC8000 to DSI
 * Ported from: kernel/drivers/gpu/drm/bridge/verisilicon/vs_x5_syscon_bridge.c
 */

#include <common.h>
#include <errno.h>
#include <linux/types.h>
#include <asm/io.h>
#include <fdtdec.h>
#include "../dc8000/x5_dc8000_regs.h"
#include <hb_display_log.h>

/*
 * Offsets from disp_sys_con base 0x3e0a0000 (kernel disp_sys_con + disp_iomuxc).
 * DTS: reg = <0x3e0a0000 0x130> — 304 bytes. 0x54/0x58: BT1120 IORING; 0x98: MODE_SEL;
 * 0x9c/0xa0: PINMUX (see DISP_BT1120_PIN_MUX_CTRL* in x5_dc8000_regs.h).
 */
#define DISP_SYSCON_BT1120_CLK_IORING	0x54
#define DISP_SYSCON_BT1120_D0_IORING	0x58
#define DISP_SYSCON_BT1120_MS		0x98

/**
 * x5_syscon_bridge_init - Initialize DC to DSI bridge
 * @timing: Display timing (must not be NULL; reserved for future polarity)
 *
 * Configures the SYSCON registers to route DC8000 output to DSI controller.
 * Based on kernel's dsi_data configuration in vs_x5_syscon_bridge.c
 */
int x5_syscon_bridge_init(struct display_timing *timing)
{
	u32 val, before, after;

	if (!timing)
		return -EINVAL;

	/* Kernel vs_x5_syscon_bridge.c dsi_data[]: DISP_DC2CSI_DSI_EN @0x10, value DC2DSI_EN */
	before = syscon_readl(DISP_DC2CSI_DSI_EN);
	val = before | DC2DSI_EN;
	syscon_writel(val, DISP_DC2CSI_DSI_EN);
	after = syscon_readl(DISP_DC2CSI_DSI_EN);

	SYSCON_LOG_DEBUG("DC->DSI before=0x%08x after=0x%08x\n", before, after);
	X5_DISP_LOG_INFO("syscon_dsi mode_set: DISP_DC2CSI_DSI_EN@0x10=0x%08x "
			 "(kernel dsi_data DC2DSI_EN; disable rst DC2CSI_EN)\n",
			 after);

	return 0;
}

/**
 * x5_syscon_bridge_disable - Disable DC to DSI bridge
 *
 * Reset to DC2CSI_EN only (for IDI output)
 */
void x5_syscon_bridge_disable(void)
{
	u32 after;

	/* Kernel dsi_data rst_values: DC2CSI_EN only */
	syscon_writel(DC2CSI_EN, DISP_DC2CSI_DSI_EN);
	after = syscon_readl(DISP_DC2CSI_DSI_EN);
	SYSCON_LOG_DEBUG("DC->DSI disabled DISP_DC2CSI_DSI_EN=0x%08x\n", after);
	X5_DISP_LOG_INFO("syscon_dsi bridge_disable: DISP_DC2CSI_DSI_EN@0x10=0x%08x "
			 "(kernel dsi_data rst DC2CSI_EN)\n",
			 after);
}

/**
 * x5_syscon_bt1120_init - Route DC8000 DPI output to BT1120 (HDMI path)
 *
 * Matches kernel vs_x5_syscon_bridge.c bt1120_data[] for crtc-dc8000:
 * set DC2BT1120_EN so DC8000 pixels reach BT1120, plus pin mux + IO pads.
 *
 * The crtc-bt1120 table clears DC2BT1120_EN (BT1120 as CRTC, not DC-fed);
 * U-Boot always uses DC8000 -> BT1120 -> LT8618, so we must SET the bit.
 *
 * BT1120 pad mux + IORING + mode-select are applied by the Horizon disp pinctrl
 * driver (d-robotics,x5-disp-iomuxc) from pinctrl_bt1120_data, same as Linux.
 *
 * @timing must not be NULL (reserved for future use; matches bridge_init contract).
 */
int x5_syscon_bt1120_init(struct display_timing *timing)
{
	u32 val, rb14, rb9c, rba0;

	if (!timing)
		return -EINVAL;

	/*
	 * Kernel vs_x5_syscon_bridge.c bt1120_data[0] (crtc-dc8000):
	 *   update_bits 0x14 DC2BT1120_EN | 0x9c SELECT_BT1120_DATA | 0xa0 SELECT_BT1120_CLK
	 */
	val = syscon_readl(DISP_DC2BT1120_EN);
	val |= DC2BT1120_EN;
	syscon_writel(val, DISP_DC2BT1120_EN);

	syscon_writel(SELECT_BT1120_DATA, DISP_BT1120_PIN_MUX_CTRL0);
	syscon_writel(SELECT_BT1120_CLK, DISP_BT1120_PIN_MUX_CTRL1);

	rb14 = syscon_readl(DISP_DC2BT1120_EN);
	rb9c = syscon_readl(DISP_BT1120_PIN_MUX_CTRL0);
	rba0 = syscon_readl(DISP_BT1120_PIN_MUX_CTRL1);

	SYSCON_LOG_DEBUG("DC->BT1120 after program RB 0x14=0x%08x 0x9c=0x%08x 0xa0=0x%08x\n",
			 rb14, rb9c, rba0);
	X5_DISP_LOG_INFO("syscon_bt1120 mode_set crtc-dc8000: "
			 "DISP_DC2BT1120_EN@0x14=0x%08x DISP_BT1120_PIN_MUX0@0x9c=0x%08x "
			 "PIN_MUX1@0xa0=0x%08x (kernel bt1120_data[0])\n",
			 rb14, rb9c, rba0);

	return 0;
}

void x5_syscon_bt1120_clear_pinmux(void)
{
	syscon_writel(0, DISP_BT1120_PIN_MUX_CTRL0);
	syscon_writel(0, DISP_BT1120_PIN_MUX_CTRL1);
}

void x5_syscon_bt1120_disable(void)
{
	u32 val, rb14, rb9c, rba0;

	x5_syscon_bt1120_clear_pinmux();

	val = syscon_readl(DISP_DC2BT1120_EN);
	val &= ~DC2BT1120_EN;
	syscon_writel(val, DISP_DC2BT1120_EN);

	rb14 = syscon_readl(DISP_DC2BT1120_EN);
	rb9c = syscon_readl(DISP_BT1120_PIN_MUX_CTRL0);
	rba0 = syscon_readl(DISP_BT1120_PIN_MUX_CTRL1);

	SYSCON_LOG_DEBUG("DC->BT1120 disabled RB 0x14=0x%08x 0x9c=0x%08x 0xa0=0x%08x\n",
			 rb14, rb9c, rba0);
	X5_DISP_LOG_INFO("syscon_bt1120 bridge_disable: "
			 "DISP_DC2BT1120_EN@0x14=0x%08x PIN_MUX0@0x9c=0x%08x PIN_MUX1@0xa0=0x%08x "
			 "(kernel bt1120 rst_values 0)\n",
			 rb14, rb9c, rba0);
}

void x5_disp_iomux_dump_one_line(const char *tag)
{
	X5_DISP_LOG_INFO(
		"disp_iomuxc_dump(%s): DC2BT1120@0x14=0x%08x CLKIOR@0x54=0x%08x D0IOR@0x58=0x%08x MS@0x98=0x%08x MUX0@0x9c=0x%08x MUX1@0xa0=0x%08x\n",
		tag ? tag : "?",
		syscon_readl(DISP_DC2BT1120_EN),
		syscon_readl(DISP_SYSCON_BT1120_CLK_IORING),
		syscon_readl(DISP_SYSCON_BT1120_D0_IORING),
		syscon_readl(DISP_SYSCON_BT1120_MS),
		syscon_readl(DISP_BT1120_PIN_MUX_CTRL0),
		syscon_readl(DISP_BT1120_PIN_MUX_CTRL1));
}
