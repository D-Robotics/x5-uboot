// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 *
 * X5 Display Debug Utilities
 * Register addresses match kernel dc_8000_nano_reg.h
 */

#include <common.h>
#include <asm/io.h>
#include <linux/delay.h>

/*===========================================================================
 * Register Base Addresses
 *===========================================================================*/

/* CRM (Clock and Reset Manager) */
#define HPS_CRM_BASE 0x34210000
#define TOP_CLK_ENB 0x34210050
#define CPU_CLK_ENB 0x34210060
#define DDR_CLK_ENB 0x34210088
#define CAMERA_CLK_ENB 0x34210090
#define HPS_MIX_CLK_ENB 0x34210098
#define HPS_MIX_SW_RST 0x3421009c
#define HSIO_CLK_ENB 0x342100a0
#define LSIO_0_CLK_ENB 0x342100a8
#define LSIO_1_CLK_ENB 0x342100b0
#define X5_CRM_HPS_CLK_GEN 0x34211000

/* Clock Generator Offsets */
#define TOP_NOC_CLK_OFF 0x000
#define TOP_APB_CLK_OFF 0x0a0
#define BT1120_PIXEL_CLK_OFF 0x4a0
#define DC8000_PIXEL_CLK_OFF 0x4c0
#define DISP_SIF_ACLK_OFF 0x4e0
#define BT1120_ACLK_OFF 0x500
#define DC8000_ACLK_OFF 0x520
#define GC820_CLK_OFF 0x540
#define GC8000L_CLK_OFF 0x560
#define GC820_ACLK_OFF 0x580
#define GC8000L_ACLK_OFF 0x5a0
#define LPWM0_CLK_OFF 0x880
#define LPWM1_CLK_OFF 0x8a0

/* NOC Idle Registers */
#define NOC_IDLE_BASE 0x31032000
#define NOC_IDLE_REQ_REG0 0x31032000
#define NOC_IDLE_REQ_REG1 0x31032004
#define NOC_IDLE_STATUS_REG0 0x31032008
#define NOC_IDLE_STATUS_REG1 0x3103200c

/* DC IOMMU Registers */
#define DC_IOMMU_CTRL 0x3e0a0130
#define DC_IOMMU_MAP_CTRL0 0x3e0a0134
#define DC_IOMMU_MAP_CTRL1 0x3e0a0138
#define DC_IOMMU_MAP_CTRL2 0x3e0a013c
#define DC_IOMMU_MAP_CTRL3 0x3e0a0140

/* SYSCON Bridge */
#define X5_SYSCON_BASE 0x3e0a0000

/* DPHY */
#define X5_DPHY_BASE 0x3e0a0028

/* DSI Host */
#define X5_DSI_BASE 0x3e060000

/* DC8000 - registers start at offset 0x2000 */
#define X5_DC8000_BASE 0x3e000000
#define DC8000_REG_OFFSET 0x2000

/* Offsets from DC8000 nano register block (+0x2000) — scan/timing debug helpers */
#define DC_REG_HDISPLAY			0x100
#define DC_REG_VDISPLAY			0x110
#define DC_CURRENT_LOCATION		0x118
#define DC_REG_PANEL_CONFIG		0x0e8
#define DC_REG_PANEL_CONTROL		0x0ec
#define DC_REG_PANEL_STATE		0x0f8
#define DC_REG_DC_CONTROL		0x168
#define DC_REG_DEBUG_CNT_SELECT		0x170
#define DC_DEBUG_CNT_VALUE		0x174

/* LPWM */
#define X5_LPWM0_BASE 0x34100000
#define X5_LPWM1_BASE 0x34110000
#define LSIO_PINMUX_BASE 0x34180000

/*===========================================================================
 * Helper Macros
 *===========================================================================*/
#define dc_read(off) readl((void *)(X5_DC8000_BASE + DC8000_REG_OFFSET + (off)))
#define dc_write(off, val) writel((val), (void *)(X5_DC8000_BASE + DC8000_REG_OFFSET + (off)))
#define dsi_read(off) readl((void *)(X5_DSI_BASE + (off)))
#define dphy_read(off) readl((void *)(X5_DPHY_BASE + (off)))
#define crm_read(addr) readl((void *)(addr))

/* Called from x5_display_dump_all_status after static dump helpers */
static void x5_display_check_scan_running(void);

/*===========================================================================
 * Dump Functions - Match kernel dump_display_regs.sh exactly
 *===========================================================================*/

static void x5_display_dump_crm(void)
{
    printf("\n========================================================================\n");
    printf("  1. CRM Clock and Reset Registers (HPS CRM: 0x34210000)\n");
    printf("========================================================================\n");

    printf("\n--- 1.1 PLL Configuration ---\n");
    printf("# CPU_PLL (0x34210000)\n");
    printf("CPU_PLL_CFG      (0x34210000): 0x%08x\n", crm_read(0x34210000));
    printf("CPU_PLL_FBDIV    (0x34210004): 0x%08x\n", crm_read(0x34210004));
    printf("CPU_PLL_POSTDIV  (0x34210008): 0x%08x\n", crm_read(0x34210008));
    printf("CPU_PLL_LOCK     (0x3421000c): 0x%08x\n", crm_read(0x3421000c));
    printf("# SYS0_PLL (0x34210010)\n");
    printf("SYS0_PLL_CFG     (0x34210010): 0x%08x\n", crm_read(0x34210010));
    printf("SYS0_PLL_FBDIV   (0x34210014): 0x%08x\n", crm_read(0x34210014));
    printf("SYS0_PLL_POSTDIV (0x34210018): 0x%08x\n", crm_read(0x34210018));
    printf("SYS0_PLL_LOCK    (0x3421001c): 0x%08x\n", crm_read(0x3421001c));
    printf("# SYS1_PLL (0x34210020)\n");
    printf("SYS1_PLL_CFG     (0x34210020): 0x%08x\n", crm_read(0x34210020));
    printf("SYS1_PLL_FBDIV   (0x34210024): 0x%08x\n", crm_read(0x34210024));
    printf("SYS1_PLL_POSTDIV (0x34210028): 0x%08x\n", crm_read(0x34210028));
    printf("SYS1_PLL_LOCK    (0x3421002c): 0x%08x\n", crm_read(0x3421002c));
    printf("# DISP_PLL (0x34210030) - Used for DC8000 pixel clock\n");
    printf("DISP_PLL_CFG     (0x34210030): 0x%08x\n", crm_read(0x34210030));
    printf("DISP_PLL_FBDIV   (0x34210034): 0x%08x\n", crm_read(0x34210034));
    printf("DISP_PLL_POSTDIV (0x34210038): 0x%08x\n", crm_read(0x34210038));
    printf("DISP_PLL_LOCK    (0x3421003c): 0x%08x\n", crm_read(0x3421003c));
    printf("# PIXEL_PLL (0x34210040) - May be used for pixel clock\n");
    printf("PIXEL_PLL_CFG    (0x34210040): 0x%08x\n", crm_read(0x34210040));
    printf("PIXEL_PLL_FBDIV  (0x34210044): 0x%08x\n", crm_read(0x34210044));
    printf("PIXEL_PLL_POSTDIV(0x34210048): 0x%08x\n", crm_read(0x34210048));
    printf("PIXEL_PLL_LOCK   (0x3421004c): 0x%08x\n", crm_read(0x3421004c));

    printf("\n--- 1.2 Clock Enable Registers ---\n");
    printf("TOP_CLK_ENB      (0x34210050): 0x%08x\n", crm_read(TOP_CLK_ENB));
    printf("CPU_CLK_ENB      (0x34210060): 0x%08x\n", crm_read(CPU_CLK_ENB));
    printf("DDR_CLK_ENB      (0x34210088): 0x%08x\n", crm_read(DDR_CLK_ENB));
    printf("CAMERA_CLK_ENB   (0x34210090): 0x%08x\n", crm_read(CAMERA_CLK_ENB));
    printf("HPS_MIX_CLK_ENB  (0x34210098): 0x%08x\n", crm_read(HPS_MIX_CLK_ENB));
    printf("HPS_MIX_SW_RST   (0x3421009c): 0x%08x\n", crm_read(HPS_MIX_SW_RST));
    printf("HSIO_CLK_ENB     (0x342100a0): 0x%08x\n", crm_read(HSIO_CLK_ENB));
    printf("LSIO_0_CLK_ENB   (0x342100a8): 0x%08x\n", crm_read(LSIO_0_CLK_ENB));
    printf("LSIO_1_CLK_ENB   (0x342100b0): 0x%08x\n", crm_read(LSIO_1_CLK_ENB));

    printf("\n--- 1.3 Clock Generator Registers (HPS_CLK_GEN base: 0x34211000) ---\n");
    printf("# TOP_NOC_CLK (0x34211000)\n");
    printf("TOP_NOC_CLK      (0x34211000): 0x%08x\n", crm_read(X5_CRM_HPS_CLK_GEN + TOP_NOC_CLK_OFF));
    printf("# TOP_APB_CLK (0x342110a0) - Parent for all APB clocks\n");
    printf("TOP_APB_CLK      (0x342110a0): 0x%08x\n", crm_read(X5_CRM_HPS_CLK_GEN + TOP_APB_CLK_OFF));

    printf("\n--- 1.4 Display Clock Generators ---\n");
    printf("# BT1120_PIXEL_CLK (0x342114a0)\n");
    printf("BT1120_PIXEL_CLK (0x342114a0): 0x%08x\n", crm_read(X5_CRM_HPS_CLK_GEN + BT1120_PIXEL_CLK_OFF));
    printf("# DC8000_PIXEL_CLK (0x342114c0) - DC8000 pixel clock\n");
    printf("DC8000_PIXEL_CLK (0x342114c0): 0x%08x\n", crm_read(X5_CRM_HPS_CLK_GEN + DC8000_PIXEL_CLK_OFF));
    printf("# DISP_SIF_ACLK (0x342114e0)\n");
    printf("DISP_SIF_ACLK    (0x342114e0): 0x%08x\n", crm_read(X5_CRM_HPS_CLK_GEN + DISP_SIF_ACLK_OFF));
    printf("# BT1120_ACLK (0x34211500)\n");
    printf("BT1120_ACLK      (0x34211500): 0x%08x\n", crm_read(X5_CRM_HPS_CLK_GEN + BT1120_ACLK_OFF));
    printf("# DC8000_ACLK (0x34211520) - DC8000 AXI clock\n");
    printf("DC8000_ACLK      (0x34211520): 0x%08x\n", crm_read(X5_CRM_HPS_CLK_GEN + DC8000_ACLK_OFF));

    printf("\n--- 1.5 GPU Clock Generators ---\n");
    printf("GC820_CLK        (0x34211540): 0x%08x\n", crm_read(X5_CRM_HPS_CLK_GEN + GC820_CLK_OFF));
    printf("GC8000L_CLK      (0x34211560): 0x%08x\n", crm_read(X5_CRM_HPS_CLK_GEN + GC8000L_CLK_OFF));
    printf("GC820_ACLK       (0x34211580): 0x%08x\n", crm_read(X5_CRM_HPS_CLK_GEN + GC820_ACLK_OFF));
    printf("GC8000L_ACLK     (0x342115a0): 0x%08x\n", crm_read(X5_CRM_HPS_CLK_GEN + GC8000L_ACLK_OFF));

    printf("\n--- 1.6 LPWM Clock Generators ---\n");
    printf("LPWM0_CLK        (0x34211880): 0x%08x\n", crm_read(X5_CRM_HPS_CLK_GEN + LPWM0_CLK_OFF));
    printf("LPWM1_CLK        (0x342118a0): 0x%08x\n", crm_read(X5_CRM_HPS_CLK_GEN + LPWM1_CLK_OFF));
}

static void x5_display_dump_syscon(void)
{
    printf("\n========================================================================\n");
    printf("  2. NOC Idle Registers (0x31032000)\n");
    printf("========================================================================\n");
    printf("NOC_IDLE_REQ_REG0    (0x31032000): 0x%08x\n", crm_read(NOC_IDLE_REQ_REG0));
    printf("NOC_IDLE_REQ_REG1    (0x31032004): 0x%08x\n", crm_read(NOC_IDLE_REQ_REG1));
    printf("NOC_IDLE_STATUS_REG0 (0x31032008): 0x%08x\n", crm_read(NOC_IDLE_STATUS_REG0));
    printf("NOC_IDLE_STATUS_REG1 (0x3103200c): 0x%08x\n", crm_read(NOC_IDLE_STATUS_REG1));
    printf("# Display-related bits in REG0:\n");
    printf("#   bit 27: BT1120\n");
    printf("#   bit 28: DC8000\n");
    printf("#   bit 29: DISP_SIF\n");

    printf("\n========================================================================\n");
    printf("  3. DC IOMMU Registers (0x3e0a0130)\n");
    printf("========================================================================\n");
    printf("DC_IOMMU_CTRL        (0x3e0a0130): 0x%08x\n", crm_read(DC_IOMMU_CTRL));
    printf("DC_IOMMU_MAP_CTRL0   (0x3e0a0134): 0x%08x\n", crm_read(DC_IOMMU_MAP_CTRL0));
    printf("DC_IOMMU_MAP_CTRL1   (0x3e0a0138): 0x%08x\n", crm_read(DC_IOMMU_MAP_CTRL1));
    printf("DC_IOMMU_MAP_CTRL2   (0x3e0a013c): 0x%08x\n", crm_read(DC_IOMMU_MAP_CTRL2));
    printf("DC_IOMMU_MAP_CTRL3   (0x3e0a0140): 0x%08x\n", crm_read(DC_IOMMU_MAP_CTRL3));

    printf("\n========================================================================\n");
    printf("  4. SYSCON Bridge Registers (0x3e0a0000)\n");
    printf("========================================================================\n");
    printf("SYSCON_0x00          (0x3e0a0000): 0x%08x\n", crm_read(X5_SYSCON_BASE + 0x00));
    printf("SYSCON_0x04          (0x3e0a0004): 0x%08x\n", crm_read(X5_SYSCON_BASE + 0x04));
    printf("SYSCON_0x08          (0x3e0a0008): 0x%08x\n", crm_read(X5_SYSCON_BASE + 0x08));
    printf("SYSCON_0x0c          (0x3e0a000c): 0x%08x\n", crm_read(X5_SYSCON_BASE + 0x0c));
    printf("DC2CSI_DSI_EN        (0x3e0a0010): 0x%08x\n", crm_read(X5_SYSCON_BASE + 0x10));
    printf("SYSCON_0x14          (0x3e0a0014): 0x%08x\n", crm_read(X5_SYSCON_BASE + 0x14));
    printf("SYSCON_0x18          (0x3e0a0018): 0x%08x\n", crm_read(X5_SYSCON_BASE + 0x18));
    printf("SYSCON_0x1c          (0x3e0a001c): 0x%08x\n", crm_read(X5_SYSCON_BASE + 0x1c));
    printf("SYSCON_0x20          (0x3e0a0020): 0x%08x\n", crm_read(X5_SYSCON_BASE + 0x20));
    printf("SYSCON_0x24          (0x3e0a0024): 0x%08x\n", crm_read(X5_SYSCON_BASE + 0x24));
}

static void x5_display_dump_dphy(void)
{
    printf("\n========================================================================\n");
    printf("  5. DPHY Registers (base: 0x3e0a0028)\n");
    printf("========================================================================\n");
    printf("DPHY_BASE        (0x3e0a0028): 0x%08x\n", dphy_read(0x00));
    printf("DPHY_CFG_0       (0x3e0a002c): 0x%08x\n", dphy_read(0x04));
    printf("DPHY_PLL_CFG     (0x3e0a0030): 0x%08x\n", dphy_read(0x08));
    printf("DPHY_0x34        (0x3e0a0034): 0x%08x\n", dphy_read(0x0c));
    printf("DPHY_0x38        (0x3e0a0038): 0x%08x\n", dphy_read(0x10));
    printf("DPHY_PLL_STS_0   (0x3e0a003c): 0x%08x\n", dphy_read(0x14));
    printf("DPHY_PLL_STS_1   (0x3e0a0040): 0x%08x\n", dphy_read(0x18));
    printf("DPHY_0x44        (0x3e0a0044): 0x%08x\n", dphy_read(0x1c));
    printf("DPHY_0x48        (0x3e0a0048): 0x%08x\n", dphy_read(0x20));
    printf("PHY_TST_CTRL0    (0x3e0a004c): 0x%08x\n", dphy_read(0x24));
    printf("PHY_TST_CTRL1    (0x3e0a0050): 0x%08x\n", dphy_read(0x28));
    printf("DPHY_0x54        (0x3e0a0054): 0x%08x\n", dphy_read(0x2c));
    printf("DPHY_0x58        (0x3e0a0058): 0x%08x\n", dphy_read(0x30));
}

static void x5_display_dump_dsi(void)
{
    printf("\n========================================================================\n");
    printf("  6. DSI Host Registers (0x3e060000)\n");
    printf("========================================================================\n");
    printf("DSI_VERSION      (0x3e060000): 0x%08x\n", dsi_read(0x00));
    printf("DSI_PWR_UP       (0x3e060004): 0x%08x\n", dsi_read(0x04));
    printf("DSI_CLKMGR_CFG   (0x3e060008): 0x%08x\n", dsi_read(0x08));
    printf("DSI_DPI_VCID     (0x3e06000c): 0x%08x\n", dsi_read(0x0c));
    printf("DSI_DPI_COLOR    (0x3e060010): 0x%08x\n", dsi_read(0x10));
    printf("DSI_DPI_CFG_POL  (0x3e060014): 0x%08x\n", dsi_read(0x14));
    printf("DSI_DPI_LP_CMD   (0x3e060018): 0x%08x\n", dsi_read(0x18));
    printf("DSI_DBI_VCID     (0x3e06001c): 0x%08x\n", dsi_read(0x1c));
    printf("DSI_DBI_CFG      (0x3e060020): 0x%08x\n", dsi_read(0x20));
    printf("DSI_DBI_PART_EN  (0x3e060024): 0x%08x\n", dsi_read(0x24));
    printf("DSI_DBI_CMDSIZE  (0x3e060028): 0x%08x\n", dsi_read(0x28));
    printf("DSI_PCKHDL_CFG   (0x3e06002c): 0x%08x\n", dsi_read(0x2c));
    printf("DSI_GEN_VCID     (0x3e060030): 0x%08x\n", dsi_read(0x30));
    printf("DSI_MODE_CFG     (0x3e060034): 0x%08x\n", dsi_read(0x34));
    printf("DSI_VID_MODE_CFG (0x3e060038): 0x%08x\n", dsi_read(0x38));
    printf("DSI_VID_PKT_SIZE (0x3e06003c): 0x%08x\n", dsi_read(0x3c));
    printf("DSI_VID_NUM_CHUNKS(0x3e060040): 0x%08x\n", dsi_read(0x40));
    printf("DSI_VID_NULL_SIZE(0x3e060044): 0x%08x\n", dsi_read(0x44));
    printf("DSI_VID_HSA_TIME (0x3e060048): 0x%08x\n", dsi_read(0x48));
    printf("DSI_VID_HBP_TIME (0x3e06004c): 0x%08x\n", dsi_read(0x4c));
    printf("DSI_VID_HLINE    (0x3e060050): 0x%08x\n", dsi_read(0x50));
    printf("DSI_VID_VSA_LINES(0x3e060054): 0x%08x\n", dsi_read(0x54));
    printf("DSI_VID_VBP_LINES(0x3e060058): 0x%08x\n", dsi_read(0x58));
    printf("DSI_VID_VFP_LINES(0x3e06005c): 0x%08x\n", dsi_read(0x5c));
    printf("DSI_VID_VACT_LINES(0x3e060060): 0x%08x\n", dsi_read(0x60));
    printf("DSI_EDPI_CMD_SIZE(0x3e060064): 0x%08x\n", dsi_read(0x64));
    printf("DSI_CMD_MODE_CFG (0x3e060068): 0x%08x\n", dsi_read(0x68));
    printf("DSI_GEN_HDR      (0x3e06006c): 0x%08x\n", dsi_read(0x6c));
    printf("DSI_GEN_PLD_DATA (0x3e060070): 0x%08x\n", dsi_read(0x70));
    printf("DSI_CMD_PKT_STATUS(0x3e060074): 0x%08x\n", dsi_read(0x74));
    printf("DSI_TO_CNT_CFG   (0x3e060078): 0x%08x\n", dsi_read(0x78));
    printf("DSI_HS_RD_TO_CNT (0x3e06007c): 0x%08x\n", dsi_read(0x7c));
    printf("DSI_LP_RD_TO_CNT (0x3e060080): 0x%08x\n", dsi_read(0x80));
    printf("DSI_HS_WR_TO_CNT (0x3e060084): 0x%08x\n", dsi_read(0x84));
    printf("DSI_LP_WR_TO_CNT (0x3e060088): 0x%08x\n", dsi_read(0x88));
    printf("DSI_BTA_TO_CNT   (0x3e06008c): 0x%08x\n", dsi_read(0x8c));
    printf("DSI_SDF_3D       (0x3e060090): 0x%08x\n", dsi_read(0x90));
    printf("DSI_LPCLK_CTRL   (0x3e060094): 0x%08x\n", dsi_read(0x94));
    printf("DSI_PHY_TMR_LPCLK(0x3e060098): 0x%08x\n", dsi_read(0x98));
    printf("DSI_PHY_TMR_CFG  (0x3e06009c): 0x%08x\n", dsi_read(0x9c));
    printf("DSI_PHY_RSTZ     (0x3e0600a0): 0x%08x\n", dsi_read(0xa0));
    printf("DSI_PHY_IF_CFG   (0x3e0600a4): 0x%08x\n", dsi_read(0xa4));
    printf("DSI_PHY_ULPS_CTRL(0x3e0600a8): 0x%08x\n", dsi_read(0xa8));
    printf("DSI_PHY_TX_TRIG  (0x3e0600ac): 0x%08x\n", dsi_read(0xac));
    printf("DSI_PHY_STATUS   (0x3e0600b0): 0x%08x\n", dsi_read(0xb0));
    printf("DSI_0xb4         (0x3e0600b4): 0x%08x\n", dsi_read(0xb4));
    printf("DSI_0xb8         (0x3e0600b8): 0x%08x\n", dsi_read(0xb8));
    printf("DSI_INT_ST0      (0x3e0600bc): 0x%08x\n", dsi_read(0xbc));
    printf("DSI_INT_ST1      (0x3e0600c0): 0x%08x\n", dsi_read(0xc0));
    printf("DSI_INT_MSK0     (0x3e0600c4): 0x%08x\n", dsi_read(0xc4));
    printf("DSI_INT_MSK1     (0x3e0600c8): 0x%08x\n", dsi_read(0xc8));
    printf("DSI_PHY_TMR_RD   (0x3e0600f4): 0x%08x\n", dsi_read(0xf4));
}

static void x5_display_dump_dc8000(void)
{
    printf("\n========================================================================\n");
    printf("  7. DC8000 Nano Registers (base: 0x3e000000, offset: 0x2000)\n");
    printf("========================================================================\n");

    printf("\n--- 7.1 Chip Information ---\n");
    printf("DC_CHIP_REV      (0x3e002000): 0x%08x\n", dc_read(0x000));
    printf("DC_CHIP_DATE     (0x3e002004): 0x%08x\n", dc_read(0x004));
    printf("DC_CHIP_PATCH    (0x3e002008): 0x%08x\n", dc_read(0x008));
    printf("DC_PRODUCT_ID    (0x3e00200c): 0x%08x\n", dc_read(0x00c));
    printf("DC_CUSTOMER_ID   (0x3e002010): 0x%08x\n", dc_read(0x010));
    printf("DC_CHIP_ID       (0x3e002014): 0x%08x\n", dc_read(0x014));
    printf("DC_CHIP_TIME     (0x3e002018): 0x%08x\n", dc_read(0x018));
    printf("DC_CHIP_INFO     (0x3e00201c): 0x%08x\n", dc_read(0x01c));
    printf("DC_ECO_ID        (0x3e002020): 0x%08x\n", dc_read(0x020));

    printf("\n--- 7.2 Framebuffer Configuration ---\n");
    printf("FB_CONFIG        (0x3e002024): 0x%08x\n", dc_read(0x024));
    printf("FB_ADDRESS       (0x3e002028): 0x%08x\n", dc_read(0x028));
    printf("FB_STRIDE        (0x3e00202c): 0x%08x\n", dc_read(0x02c));
    printf("FB_ORIGIN        (0x3e002030): 0x%08x\n", dc_read(0x030));
    printf("TILE_IN_CFG      (0x3e002034): 0x%08x\n", dc_read(0x034));
    printf("TILE_UV_FB_ADR   (0x3e002038): 0x%08x\n", dc_read(0x038));
    printf("TILE_UV_FB_STR   (0x3e00203c): 0x%08x\n", dc_read(0x03c));

    printf("\n--- 7.3 Framebuffer Colors ---\n");
    printf("FB_BACKGROUND    (0x3e002058): 0x%08x\n", dc_read(0x058));
    printf("FB_COLOR_KEY     (0x3e00205c): 0x%08x\n", dc_read(0x05c));
    printf("FB_COLOR_KEY_HIGH(0x3e002060): 0x%08x\n", dc_read(0x060));
    printf("FB_CLEAR_VALUE   (0x3e002064): 0x%08x\n", dc_read(0x064));

    printf("\n--- 7.4 Video Layer ---\n");
    printf("VIDEO_TL         (0x3e002068): 0x%08x\n", dc_read(0x068));
    printf("FB_SIZE          (0x3e00206c): 0x%08x\n", dc_read(0x06c));
    printf("VIDEO_GLOBAL_ALPHA(0x3e002070): 0x%08x\n", dc_read(0x070));
    printf("BLEND_STACK_ORDER(0x3e002074): 0x%08x\n", dc_read(0x074));
    printf("VIDEO_BLEND_CFG  (0x3e002078): 0x%08x\n", dc_read(0x078));

    printf("\n--- 7.5 Overlay 0 ---\n");
    printf("OVERLAY_CONFIG   (0x3e00207c): 0x%08x\n", dc_read(0x07c));
    printf("OVERLAY_ADDRESS  (0x3e002080): 0x%08x\n", dc_read(0x080));
    printf("OVERLAY_STRIDE   (0x3e002084): 0x%08x\n", dc_read(0x084));
    printf("OVERLAY_TILE_CFG (0x3e002088): 0x%08x\n", dc_read(0x088));
    printf("OVERLAY_UV_ADR   (0x3e00208c): 0x%08x\n", dc_read(0x08c));
    printf("OVERLAY_UV_STR   (0x3e002090): 0x%08x\n", dc_read(0x090));
    printf("OVERLAY_TL       (0x3e002094): 0x%08x\n", dc_read(0x094));
    printf("OVERLAY_SIZE     (0x3e002098): 0x%08x\n", dc_read(0x098));
    printf("OVERLAY_COLOR_KEY(0x3e00209c): 0x%08x\n", dc_read(0x09c));
    printf("OVERLAY_CK_HIGH  (0x3e0020a0): 0x%08x\n", dc_read(0x0a0));
    printf("OVERLAY_BLEND_CFG(0x3e0020a4): 0x%08x\n", dc_read(0x0a4));
    printf("OVERLAY_ALPHA    (0x3e0020a8): 0x%08x\n", dc_read(0x0a8));
    printf("OVERLAY_CLEAR    (0x3e0020ac): 0x%08x\n", dc_read(0x0ac));

    printf("\n--- 7.6 Overlay 1 ---\n");
    printf("OVERLAY1_CONFIG  (0x3e0020b0): 0x%08x\n", dc_read(0x0b0));
    printf("OVERLAY1_ADDRESS (0x3e0020b4): 0x%08x\n", dc_read(0x0b4));
    printf("OVERLAY1_STRIDE  (0x3e0020b8): 0x%08x\n", dc_read(0x0b8));
    printf("OVERLAY1_TL      (0x3e0020bc): 0x%08x\n", dc_read(0x0bc));
    printf("OVERLAY1_SIZE    (0x3e0020c0): 0x%08x\n", dc_read(0x0c0));
    printf("OVERLAY1_CK      (0x3e0020c4): 0x%08x\n", dc_read(0x0c4));
    printf("OVERLAY1_CK_HIGH (0x3e0020c8): 0x%08x\n", dc_read(0x0c8));
    printf("OVERLAY1_BLEND   (0x3e0020cc): 0x%08x\n", dc_read(0x0cc));
    printf("OVERLAY1_ALPHA   (0x3e0020d0): 0x%08x\n", dc_read(0x0d0));
    printf("OVERLAY1_CLEAR   (0x3e0020d4): 0x%08x\n", dc_read(0x0d4));

    printf("\n--- 7.7 Panel Destination (Writeback) ---\n");
    printf("PANEL_DEST_ADDR  (0x3e0020d8): 0x%08x\n", dc_read(0x0d8));
    printf("DEST_STRIDE      (0x3e0020dc): 0x%08x\n", dc_read(0x0dc));

    printf("\n--- 7.8 Dither ---\n");
    printf("DITHER_TABLE_LOW (0x3e0020e0): 0x%08x\n", dc_read(0x0e0));
    printf("DITHER_TABLE_HIGH(0x3e0020e4): 0x%08x\n", dc_read(0x0e4));

    printf("\n--- 7.9 Panel Configuration ---\n");
    printf("PANEL_CONFIG     (0x3e0020e8): 0x%08x\n", dc_read(0x0e8));
    printf("PANEL_CONTROL    (0x3e0020ec): 0x%08x\n", dc_read(0x0ec));
    printf("PANEL_FUNCTION   (0x3e0020f0): 0x%08x\n", dc_read(0x0f0));
    printf("PANEL_WORKING    (0x3e0020f4): 0x%08x\n", dc_read(0x0f4));
    printf("PANEL_STATE      (0x3e0020f8): 0x%08x\n", dc_read(0x0f8));
    printf("PANEL_TIMING     (0x3e0020fc): 0x%08x\n", dc_read(0x0fc));

    printf("\n--- 7.10 Display Timing ---\n");
    printf("HDISPLAY         (0x3e002100): 0x%08x\n", dc_read(0x100));
    printf("HSYNC            (0x3e002104): 0x%08x\n", dc_read(0x104));
    printf("HCOUNTER1        (0x3e002108): 0x%08x\n", dc_read(0x108));
    printf("HCOUNTER2        (0x3e00210c): 0x%08x\n", dc_read(0x10c));
    printf("VDISPLAY         (0x3e002110): 0x%08x\n", dc_read(0x110));
    printf("VSYNC            (0x3e002114): 0x%08x\n", dc_read(0x114));
    printf("CURRENT_LOCATION (0x3e002118): 0x%08x\n", dc_read(DC_CURRENT_LOCATION));

    printf("\n--- 7.11 Gamma ---\n");
    printf("GAMMA_INDEX      (0x3e00211c): 0x%08x\n", dc_read(0x11c));
    printf("GAMMA_DATA       (0x3e002120): 0x%08x\n", dc_read(0x120));

    printf("\n--- 7.12 Cursor ---\n");
    printf("CURSOR_CONFIG    (0x3e002124): 0x%08x\n", dc_read(0x124));
    printf("CURSOR_ADDRESS   (0x3e002128): 0x%08x\n", dc_read(0x128));
    printf("CURSOR_LOCATION  (0x3e00212c): 0x%08x\n", dc_read(0x12c));
    printf("CURSOR_BACKGROUND(0x3e002130): 0x%08x\n", dc_read(0x130));
    printf("CURSOR_FOREGROUND(0x3e002134): 0x%08x\n", dc_read(0x134));

    printf("\n--- 7.13 Interrupt ---\n");
    printf("DISPLAY_INTR     (0x3e002138): 0x%08x\n", dc_read(0x138));
    printf("DISPLAY_INTR_EN  (0x3e00213c): 0x%08x\n", dc_read(0x13c));

    printf("\n--- 7.14 DBI/DPI Configuration ---\n");
    printf("DBI_CONFIG       (0x3e002140): 0x%08x\n", dc_read(0x140));
    printf("DBI_IF_RESET     (0x3e002144): 0x%08x\n", dc_read(0x144));
    printf("DBI_WR_CHAR1     (0x3e002148): 0x%08x\n", dc_read(0x148));
    printf("DBI_WR_CHAR2     (0x3e00214c): 0x%08x\n", dc_read(0x14c));
    printf("DBI_CMD          (0x3e002150): 0x%08x\n", dc_read(0x150));
    printf("DPI_CONFIG       (0x3e002154): 0x%08x\n", dc_read(0x154));
    printf("DBI_TYPEC_CFG    (0x3e002158): 0x%08x\n", dc_read(0x158));
    printf("DC_STATUS        (0x3e00215c): 0x%08x\n", dc_read(0x15c));

    printf("\n--- 7.15 Control ---\n");
    printf("SRC_CONFIG_ENDIAN(0x3e002160): 0x%08x\n", dc_read(0x160));
    printf("SOFT_RESET       (0x3e002164): 0x%08x\n", dc_read(0x164));
    printf("DC_CONTROL       (0x3e002168): 0x%08x\n", dc_read(0x168));
    printf("REG_TIMING_CTRL  (0x3e00216c): 0x%08x\n", dc_read(0x16c));
    printf("DEBUG_CNT_SELECT (0x3e002170): 0x%08x\n", dc_read(DC_REG_DEBUG_CNT_SELECT));
    printf("DEBUG_CNT_VALUE  (0x3e002174): 0x%08x\n", dc_read(DC_DEBUG_CNT_VALUE));

    printf("\n--- 7.16 Layer Clock Gate ---\n");
    printf("LAYER_CLOCK_GATE (0x3e0021a0): 0x%08x\n", dc_read(0x1a0));

    printf("\n--- 7.17 Debug Registers ---\n");
    /* Enable debug registers first (DC_CONTROL bit 3) */
    {
        u32 dc_ctrl = dc_read(DC_REG_DC_CONTROL);
        if (!(dc_ctrl & (1 << 3)))
        {
            printf("# Enabling debug registers (DC_CONTROL bit 3)...\n");
            dc_write(DC_REG_DC_CONTROL, dc_ctrl | (1 << 3));
            udelay(100); /* Wait for debug registers to be ready */
        }
        printf("DC_CONTROL       (0x3e002168): 0x%08x (DEBUG_REG=%d)\n",
               dc_read(DC_REG_DC_CONTROL), (dc_read(DC_REG_DC_CONTROL) >> 3) & 1);
    }
    /* Debug counter registers */
    printf("DEBUG_TOT_VIDEO_REQ   (0x3e0021a4): 0x%08x\n", dc_read(0x1a4));
    printf("DEBUG_LST_VIDEO_REQ   (0x3e0021a8): 0x%08x\n", dc_read(0x1a8));
    printf("DEBUG_TOT_VIDEO_RRB   (0x3e0021ac): 0x%08x\n", dc_read(0x1ac));
    printf("DEBUG_LST_VIDEO_RRB   (0x3e0021b0): 0x%08x\n", dc_read(0x1b0));
    printf("DEBUG_TOT_OVERLAY0_REQ(0x3e0021b4): 0x%08x\n", dc_read(0x1b4));
    printf("DEBUG_LST_OVERLAY0_REQ(0x3e0021b8): 0x%08x\n", dc_read(0x1b8));
    printf("DEBUG_TOT_OVERLAY0_RRB(0x3e0021bc): 0x%08x\n", dc_read(0x1bc));
    printf("DEBUG_LST_OVERLAY0_RRB(0x3e0021c0): 0x%08x\n", dc_read(0x1c0));
    printf("DEBUG_TOT_OVERLAY1_REQ(0x3e0021c4): 0x%08x\n", dc_read(0x1c4));
    printf("DEBUG_LST_OVERLAY1_REQ(0x3e0021c8): 0x%08x\n", dc_read(0x1c8));
    printf("DEBUG_TOT_OVERLAY1_RRB(0x3e0021cc): 0x%08x\n", dc_read(0x1cc));
    printf("DEBUG_LST_OVERLAY1_RRB(0x3e0021d0): 0x%08x\n", dc_read(0x1d0));
    printf("DEBUG_TOT_CURSOR_REQ  (0x3e0021d4): 0x%08x\n", dc_read(0x1d4));
    printf("DEBUG_LST_CURSOR_REQ  (0x3e0021d8): 0x%08x\n", dc_read(0x1d8));
    printf("DEBUG_TOT_CURSOR_RRB  (0x3e0021dc): 0x%08x\n", dc_read(0x1dc));
    printf("DEBUG_LST_CURSOR_RRB  (0x3e0021e0): 0x%08x\n", dc_read(0x1e0));
    printf("DEBUG_TOT_DC_REQ      (0x3e0021e4): 0x%08x\n", dc_read(0x1e4));
    printf("DEBUG_LST_DC_REQ      (0x3e0021e8): 0x%08x\n", dc_read(0x1e8));
    printf("DEBUG_TOT_DC_RRB      (0x3e0021ec): 0x%08x\n", dc_read(0x1ec));
    printf("DEBUG_LST_DC_RRB      (0x3e0021f0): 0x%08x\n", dc_read(0x1f0));
    printf("DEBUG_FRAME_MISFLAG   (0x3e0021f4): 0x%08x\n", dc_read(0x1f4));
}

static void x5_display_dump_lpwm(void)
{
    printf("\n========================================================================\n");
    printf("  8. LPWM Registers (Backlight)\n");
    printf("========================================================================\n");

    printf("\n--- 8.1 LSIO Pinmux ---\n");
    printf("LSIO_PINMUX_0    (0x34180078): 0x%08x\n", crm_read(LSIO_PINMUX_BASE + 0x78));
    printf("LSIO_PINMUX_1    (0x3418007c): 0x%08x\n", crm_read(LSIO_PINMUX_BASE + 0x7c));

    printf("\n--- 8.2 LPWM0 (0x34100000) ---\n");
    printf("LPWM0_GLB_CFG    (0x34100000): 0x%08x\n", crm_read(X5_LPWM0_BASE + 0x00));
    printf("LPWM0_SW_TRIG    (0x34100004): 0x%08x\n", crm_read(X5_LPWM0_BASE + 0x04));
    printf("LPWM0_RST        (0x34100008): 0x%08x\n", crm_read(X5_LPWM0_BASE + 0x08));
    printf("LPWM0_CH0_CFG0   (0x34100010): 0x%08x\n", crm_read(X5_LPWM0_BASE + 0x10));
    printf("LPWM0_CH0_CFG1   (0x34100014): 0x%08x\n", crm_read(X5_LPWM0_BASE + 0x14));
    printf("LPWM0_CH0_CFG2   (0x34100018): 0x%08x\n", crm_read(X5_LPWM0_BASE + 0x18));
    printf("LPWM0_CH1_CFG0   (0x3410001c): 0x%08x\n", crm_read(X5_LPWM0_BASE + 0x1c));
    printf("LPWM0_CH1_CFG1   (0x34100020): 0x%08x\n", crm_read(X5_LPWM0_BASE + 0x20));
    printf("LPWM0_CH1_CFG2   (0x34100024): 0x%08x\n", crm_read(X5_LPWM0_BASE + 0x24));
    printf("LPWM0_CH2_CFG0   (0x34100028): 0x%08x\n", crm_read(X5_LPWM0_BASE + 0x28));
    printf("LPWM0_CH2_CFG1   (0x3410002c): 0x%08x\n", crm_read(X5_LPWM0_BASE + 0x2c));
    printf("LPWM0_CH2_CFG2   (0x34100030): 0x%08x\n", crm_read(X5_LPWM0_BASE + 0x30));
    printf("LPWM0_CH3_CFG0   (0x34100034): 0x%08x\n", crm_read(X5_LPWM0_BASE + 0x34));
    printf("LPWM0_CH3_CFG1   (0x34100038): 0x%08x\n", crm_read(X5_LPWM0_BASE + 0x38));
    printf("LPWM0_CH3_CFG2   (0x3410003c): 0x%08x\n", crm_read(X5_LPWM0_BASE + 0x3c));

    printf("\n--- 8.3 LPWM1 (0x34110000) ---\n");
    printf("LPWM1_GLB_CFG    (0x34110000): 0x%08x\n", crm_read(X5_LPWM1_BASE + 0x00));
    printf("LPWM1_SW_TRIG    (0x34110004): 0x%08x\n", crm_read(X5_LPWM1_BASE + 0x04));
    printf("LPWM1_RST        (0x34110008): 0x%08x\n", crm_read(X5_LPWM1_BASE + 0x08));
    printf("LPWM1_CH0_CFG0   (0x34110010): 0x%08x\n", crm_read(X5_LPWM1_BASE + 0x10));
    printf("LPWM1_CH0_CFG1   (0x34110014): 0x%08x\n", crm_read(X5_LPWM1_BASE + 0x14));
    printf("LPWM1_CH0_CFG2   (0x34110018): 0x%08x\n", crm_read(X5_LPWM1_BASE + 0x18));
    printf("LPWM1_CH1_CFG0   (0x3411001c): 0x%08x\n", crm_read(X5_LPWM1_BASE + 0x1c));
    printf("LPWM1_CH1_CFG1   (0x34110020): 0x%08x\n", crm_read(X5_LPWM1_BASE + 0x20));
    printf("LPWM1_CH1_CFG2   (0x34110024): 0x%08x\n", crm_read(X5_LPWM1_BASE + 0x24));
    printf("LPWM1_CH2_CFG0   (0x34110028): 0x%08x\n", crm_read(X5_LPWM1_BASE + 0x28));
    printf("LPWM1_CH2_CFG1   (0x3411002c): 0x%08x\n", crm_read(X5_LPWM1_BASE + 0x2c));
    printf("LPWM1_CH2_CFG2   (0x34110030): 0x%08x\n", crm_read(X5_LPWM1_BASE + 0x30));
    printf("LPWM1_CH3_CFG0   (0x34110034): 0x%08x\n", crm_read(X5_LPWM1_BASE + 0x34));
    printf("LPWM1_CH3_CFG1   (0x34110038): 0x%08x\n", crm_read(X5_LPWM1_BASE + 0x38));
    printf("LPWM1_CH3_CFG2   (0x3411003c): 0x%08x\n", crm_read(X5_LPWM1_BASE + 0x3c));
}

/*===========================================================================
 * Main Dump Function - Dumps all registers matching kernel script
 *===========================================================================*/
void x5_display_dump_all_status(void)
{
    printf("\n");
    printf("========================================================================\n");
    printf("  X5 Display Subsystem Register Dump (U-Boot)\n");
    printf("  Format matches kernel dump_display_regs.sh for comparison\n");
    printf("========================================================================\n");

    x5_display_dump_crm();
    x5_display_dump_syscon();
    x5_display_dump_dphy();
    x5_display_dump_dsi();
    x5_display_dump_dc8000();
    x5_display_dump_lpwm();
    x5_display_check_scan_running();

    printf("\n========================================================================\n");
    printf("  Dump Complete\n");
    printf("========================================================================\n");
}

/*===========================================================================
 * Debug Function - Check if DC8000 scan counter is running
 *===========================================================================*/
static void x5_display_check_scan_running(void)
{
    u32 loc1, loc2, cnt1, cnt2;
    int i;
    volatile int delay;

    printf("\n=== DC8000 Scan Counter Check ===\n");
    printf("Reading CURRENT_LOCATION and DEBUG_CNT_VALUE multiple times...\n\n");

    for (i = 0; i < 5; i++)
    {
        loc1 = dc_read(DC_CURRENT_LOCATION);
        cnt1 = dc_read(DC_DEBUG_CNT_VALUE);
        /* Simple delay loop ~16ms at 60Hz */
        for (delay = 0; delay < 1000000; delay++)
            ;
        loc2 = dc_read(DC_CURRENT_LOCATION);
        cnt2 = dc_read(DC_DEBUG_CNT_VALUE);

        printf("Sample %d:\n", i + 1);
        /* CURRENT_LOCATION: X=bits[15:0], Y=bits[31:16] per DC8000 spec */
        printf("  CURRENT_LOCATION: 0x%08x -> 0x%08x (X=%d->%d, Y=%d->%d)\n",
               loc1, loc2,
               loc1 & 0xFFFF, loc2 & 0xFFFF,
               (loc1 >> 16) & 0xFFFF, (loc2 >> 16) & 0xFFFF);
        printf("  DEBUG_CNT_VALUE:  0x%08x -> 0x%08x (diff=%d)\n",
               cnt1, cnt2, cnt2 - cnt1);

        if (loc1 != loc2 || cnt1 != cnt2)
        {
            printf("  -> CHANGING (scan counter is running)\n");
        }
        else
        {
            printf("  -> STATIC (scan counter NOT running)\n");
        }
        printf("\n");
    }

    /* Check timing registers */
    printf("Timing configuration:\n");
    printf("  HDISPLAY: 0x%08x (htotal=%d, hactive=%d)\n",
           dc_read(DC_REG_HDISPLAY),
           (dc_read(DC_REG_HDISPLAY) >> 16) & 0xFFFF,
           dc_read(DC_REG_HDISPLAY) & 0xFFFF);
    printf("  VDISPLAY: 0x%08x (vtotal=%d, vactive=%d)\n",
           dc_read(DC_REG_VDISPLAY),
           (dc_read(DC_REG_VDISPLAY) >> 16) & 0xFFFF,
           dc_read(DC_REG_VDISPLAY) & 0xFFFF);
    printf("  PANEL_CONFIG: 0x%08x (CLOCK=%d, DE=%d)\n",
           dc_read(DC_REG_PANEL_CONFIG),
           (dc_read(DC_REG_PANEL_CONFIG) >> 8) & 1,
           dc_read(DC_REG_PANEL_CONFIG) & 1);
    printf("  PANEL_CONTROL: 0x%08x (VALID=%d)\n",
           dc_read(DC_REG_PANEL_CONTROL),
           dc_read(DC_REG_PANEL_CONTROL) & 1);
    printf("  PANEL_STATE: 0x%08x\n", dc_read(DC_REG_PANEL_STATE));

    /* Check if X/Y values are within timing range */
    {
        u32 htotal = (dc_read(DC_REG_HDISPLAY) >> 16) & 0xFFFF;
        u32 vtotal = (dc_read(DC_REG_VDISPLAY) >> 16) & 0xFFFF;
        u32 loc = dc_read(DC_CURRENT_LOCATION);
        /* CURRENT_LOCATION: X=bits[15:0], Y=bits[31:16] per DC8000 spec */
        u32 x = loc & 0xFFFF;
        u32 y = (loc >> 16) & 0xFFFF;

        printf("\n  Current X=%d (max=%d), Y=%d (max=%d)\n", x, htotal, y, vtotal);
        if (x < htotal && y < vtotal)
        {
            printf("  -> Position VALID (within timing range)\n");
        }
        else
        {
            printf("  -> Position INVALID (exceeds timing range!)\n");
            printf("     This means timing config was NOT loaded to working set!\n");
        }
    }
    printf("================================\n\n");
}
