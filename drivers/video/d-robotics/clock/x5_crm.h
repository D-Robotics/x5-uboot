/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 D-Robotics Co., Ltd.
 * Author: jiale01.luo <jiale01.luo@d-robotics.cc>
 * Modified by: fuhua.wang <fuhua.wang@d-robotics.cc>
 *
 * X5 CRM (Clock and Reset Manager) for Display Subsystem
 * Ported from kernel drivers/clk/hobot/clk-generator.c and clk-x5.c
 */

#ifndef __X5_CRM_H__
#define __X5_CRM_H__

/* CRM Base Addresses */
#define X5_CRM_HPS_BASE 0x34210000
#define X5_CRM_HPS_CLK_GEN 0x34211000

/* CPU_PLL Configuration (for aligning CPU_PLL_R output) */
#define CPU_PLL_CFG (X5_CRM_HPS_BASE + 0x00)     /* 0x34210000 */
#define CPU_PLL_POSTDIV (X5_CRM_HPS_BASE + 0x08) /* 0x34210008 */

/*
 * PLL Configuration Registers (from kernel clk-x5.c)
 * HPS CRM base: 0x34210000
 * Each PLL has: cfg_reg (base+offset), fbdiv (+0x4), postdiv (+0x8), lock (+0xC)
 *
 * PLL offsets from HPS CRM base:
 *   CPU_PLL:   0x00
 *   SYS0_PLL:  0x10
 *   SYS1_PLL:  0x20
 *   DISP_PLL:  0x30
 *   PIXEL_PLL: 0x40
 *
 * PLL internal registers (for fractional mode):
 *   DISP_PLL_INTERNAL:  0x2300
 *   PIXEL_PLL_INTERNAL: 0x2400
 */
#define DISP_PLL_CFG (X5_CRM_HPS_BASE + 0x30)         /* 0x34210030 */
#define PIXEL_PLL_CFG (X5_CRM_HPS_BASE + 0x40)        /* 0x34210040 */
#define DISP_PLL_INTERNAL (X5_CRM_HPS_BASE + 0x2300)  /* 0x34212300 */
#define PIXEL_PLL_INTERNAL (X5_CRM_HPS_BASE + 0x2400) /* 0x34212400 */

/* PLL Register Offsets */
#define PLL_FBDIV_OFFSET 0x4
#define PLL_POSTDIV_OFFSET 0x8
#define PLL_LOCK_OFFSET 0xC
#define PLL_ANAREG6 0x14
#define PLL_SCFRAC_CNT 0x38

/* PLL Register Bit Definitions */
#define PLL_LOWFREQ_SHIFT 4
#define PLL_VCOMODE_SHIFT 5
#define PLL_PREDIV_SHIFT 6
#define PLL_POSTDIVP_SHIFT 2
#define PLL_DIVVCOP_SHIFT 8
#define PLL_POSTDIVR_SHIFT 12
#define PLL_DIVVCOR_SHIFT 18
#define PLL_MINT_SHIFT 16

#define PLL_PREDIV_MASK GENMASK(10, PLL_PREDIV_SHIFT)
#define PLL_DIVVCOP_MASK GENMASK(11, PLL_DIVVCOP_SHIFT)
#define PLL_POSTDIVP_MASK GENMASK(7, PLL_POSTDIVP_SHIFT)
#define PLL_DIVVCOR_MASK GENMASK(21, PLL_DIVVCOR_SHIFT)
#define PLL_POSTDIVR_MASK GENMASK(17, PLL_POSTDIVR_SHIFT)
#define PLL_MINT_MASK GENMASK(25, PLL_MINT_SHIFT)
#define PLL_MFRAC_MASK GENMASK(15, 0)
#define PLL_FBDIV_LOAD BIT(26)
#define PLL_LOWFREQ_MASK BIT(4)
#define PLL_VCOMODE_MASK BIT(3)
#define PLL_FRAC_EN BIT(0)
#define PLL_ANAREG6_EN BIT(0)
#define PLL_CTRL_CFG0 0x3028
#define PLL_CTRL_DEFAULT 0x2838

#define SET_PLL_PREDIV(x) (((x) << PLL_PREDIV_SHIFT) & PLL_PREDIV_MASK)
#define SET_PLL_DIVVCOP(x) (((x) << PLL_DIVVCOP_SHIFT) & PLL_DIVVCOP_MASK)
#define SET_PLL_POSTDIVP(x) (((x) << PLL_POSTDIVP_SHIFT) & PLL_POSTDIVP_MASK)
#define SET_PLL_DIVVCOR(x) (((x) << PLL_DIVVCOR_SHIFT) & PLL_DIVVCOR_MASK)
#define SET_PLL_POSTDIVR(x) (((x) << PLL_POSTDIVR_SHIFT) & PLL_POSTDIVR_MASK)
#define SET_PLL_MINT(x) (((x) << PLL_MINT_SHIFT) & PLL_MINT_MASK)

/*
 * Seamless Linux handoff: cpu_pll_r POSTDIVR must match kernel clk-x5.c expectation
 * (see DIV path / top_apb alignment). Not read from DT today.
 */
#define X5_SEAMLESS_CPU_PLL_R_POSTDIVR	5

#define PLL_PWRON (BIT(0) | BIT(1))
#define PLL_EN (BIT(0) | BIT(1))
#define PLL_P_OUT BIT(22)
#define PLL_R_OUT BIT(23)
#define PLL_LOCK BIT(0)
#define PLL_LOCK_DELAY_US 1
#define PLL_LOCK_CHECK_COUNT 1000
#define PLL_LOCK_RETRY 3

/* Clock Enable Registers (Gate Type) */
#define HPS_MIX_CLK_ENB (X5_CRM_HPS_BASE + 0x98)
#define HPS_MIX_SW_RST (X5_CRM_HPS_BASE + 0x9C)
#define LSIO_0_CLK_ENB (X5_CRM_HPS_BASE + 0xA8)
/* DSP clock gates (kernel clk-x5.c DSP_CLK_ENB) */
#define DSP_CLK_ENB (X5_CRM_HPS_BASE + 0x10)
#define DSP_SW_RST_REG (X5_CRM_HPS_BASE + 0x14)

/* Clock Generator Offsets (from HPS_CLK_GEN base) */
#define TOP_APB_CLK_OFF 0x0A0      /* TOP APB clock generator */
#define BT1120_PIXEL_CLK_OFF 0x4A0 /* BT1120 pixel clock generator (CRM Spec) */
#define DC8000_PIXEL_CLK_OFF 0x4C0 /* DC8000 pixel clock generator */
#define DISP_SIF_ACLK_OFF 0x4E0    /* Display SIF AXI clock generator */
#define BT1120_ACLK_OFF 0x500      /* BT1120 AXI clock generator (CRM Spec) */
#define DSI_TXESC_CLK_OFF 0x500    /* DSI TX Escape clock generator */
#define DC8000_ACLK_OFF 0x520      /* DC8000 AXI clock generator */
#define LPWM0_CLK_OFF 0x880        /* LPWM0 clock generator */
#define LPWM1_CLK_OFF 0x8A0        /* LPWM1 clock generator */

/* Clock Enable Bits (HPS_MIX_CLK_ENB) - from CRM.log */
#define CLK_EN_DISP_CSI_PCLK BIT(0)
#define CLK_EN_DISP_DSI_PCLK BIT(1)
#define CLK_EN_DC8000_PCLK BIT(2) /* DC8000Nano PCLK/HCLK */
#define CLK_EN_DC8000_ACLK BIT(3) /* DC8000Nano ACLK */
#define CLK_EN_BT1120_PCLK BIT(4)
#define CLK_EN_BT1120_ACLK BIT(5)
#define CLK_EN_DPHY_CFG BIT(6)      /* DPHY config clock, parent: osc (24MHz) */
#define CLK_EN_DISP_SIF_ACLK BIT(7) /* DISP SIF AXI clock */
#define CLK_EN_DISP_SIF_PCLK BIT(8) /* DISP SIF APB clock */
#define CLK_EN_DISP_GPIO_PCLK BIT(9)

/* Clock Enable Bits (LSIO_0_CLK_ENB) */
#define CLK_EN_LPWM0_PCLK BIT(18) /* LPWM0 APB clock */
#define CLK_EN_LPWM1_PCLK BIT(19) /* LPWM1 APB clock */
#define CLK_EN_I2C4_PCLK BIT(26)  /* I2C4 APB clock (for LT8618 HDMI) */
#define CLK_EN_DSP_I2C_PCLK BIT(21) /* DSP I2C APB (I2C7, SII902x on RDK) */
#define RST_DSP_I2C BIT(19) /* DSP SW reset block: I2C (kernel reset-x5.c) */

/* Reset Bits (HPS_MIX_SW_RST) */
#define RST_DISP_CSI BIT(0)
#define RST_DSI_TX BIT(1)
#define RST_DC8000_2 BIT(2)
#define RST_DC8000_3 BIT(3)
#define RST_BT1120 BIT(4)

/*
 * NOC Idle Controller (from kernel idle.c)
 *
 * The NOC (Network on Chip) idle controller manages power isolation
 * for various IP blocks. When enabling a clock, the kernel clears
 * the corresponding idle request bit to allow the IP to access the bus.
 *
 * Base address: 0x31032000 (from device tree)
 */
#define NOC_IDLE_BASE 0x31032000
#define NOC_IDLE_REQ_REG0 (NOC_IDLE_BASE + 0x0000)
#define NOC_IDLE_STATUS_REG0 (NOC_IDLE_BASE + 0x0008)
#define NOC_IDLE_REQ_REG1 (NOC_IDLE_BASE + 0x0004)
#define NOC_IDLE_STATUS_REG1 (NOC_IDLE_BASE + 0x000C)

/* NOC Idle Masks for Display Subsystem (from kernel idle.c) */
#define NOC_IDLE_BT1120 BIT(27)   /* ISO_CG_BT1120 */
#define NOC_IDLE_DC8000 BIT(28)   /* ISO_CG_DC8000 */
#define NOC_IDLE_DISP_SIF BIT(29) /* ISO_CG_DISP_SIF */
#define NOC_IDLE_DISP_ALL (NOC_IDLE_BT1120 | NOC_IDLE_DC8000 | NOC_IDLE_DISP_SIF)

/* NOC Idle timeout */
#define NOC_IDLE_TIMEOUT_US 3000
#define NOC_IDLE_POLL_US 10

/*
 * DC IOMMU (Lite MMU) Configuration
 *
 * The DC8000 uses a simple address mapper (lite_mmu) that maps
 * 32-bit addresses by remapping the upper 4 bits (bits 31:28).
 *
 * Base address: 0x3e0a0130 (from device tree dc_iommu node)
 */
#define DC_IOMMU_BASE 0x3e0a0130
#define DC_IOMMU_CTRL (DC_IOMMU_BASE + 0x00)
#define DC_IOMMU_MAP_CTRL0 (DC_IOMMU_BASE + 0x04)
#define DC_IOMMU_MAP_CTRL1 (DC_IOMMU_BASE + 0x08)
#define DC_IOMMU_MAP_CTRL2 (DC_IOMMU_BASE + 0x0C)
#define DC_IOMMU_MAP_CTRL3 (DC_IOMMU_BASE + 0x10)

#define DC_IOMMU_CTRL_ENABLE BIT(0)

/*
 * Clock Generator Register Layout (from kernel clk-generator.c)
 *
 * bit 28:    GEN_EN (enable)
 * bit 24-26: MUX (parent select)
 * bit 16-18: pre_div (actual div = value + 1, max 8)
 * bit 0-5:   post_div (actual div = value + 1, max 64)
 *
 * Total divider = (pre_div + 1) * (post_div + 1)
 * Output rate = parent_rate / total_divider
 */
#define GEN_PRE_DIV_SHIFT 16
#define GEN_PRE_DIV_WIDTH 3
#define GEN_POST_DIV_SHIFT 0
#define GEN_POST_DIV_WIDTH 6
#define GEN_MUX_SHIFT 24
#define GEN_EN_SHIFT 28

#define GEN_PRE_DIV_MASK GENMASK(18, GEN_PRE_DIV_SHIFT)
#define GEN_POST_DIV_MASK GENMASK(5, GEN_POST_DIV_SHIFT)
#define GEN_MUX_MASK GENMASK(26, GEN_MUX_SHIFT)
#define GEN_EN BIT(28)

#define SET_GEN_PREDIV(x) (((x) << GEN_PRE_DIV_SHIFT) & GEN_PRE_DIV_MASK)
#define SET_GEN_POSTDIV(x) (((x) << GEN_POST_DIV_SHIFT) & GEN_POST_DIV_MASK)
#define SET_GEN_MUX(x) (((x) << GEN_MUX_SHIFT) & GEN_MUX_MASK)

/*
 * Parent Clock Sources for soc_gen_src_sels (AXI clocks)
 * From kernel clk-x5.c: { "osc", "cpu_pll_p", "cpu_pll_r", "sys0_pll_p",
 *                         "sys0_pll_r", "sys1_pll_p", "sys1_pll_r", "pixel_pll_r" }
 */
#define MUX_OSC 0       /* 24 MHz */
#define MUX_CPU_PLL_P 1 /* 1200 MHz */
#define MUX_CPU_PLL_R 2
#define MUX_SYS0_PLL_P 3 /* 1500 MHz */
#define MUX_SYS0_PLL_R 4
#define MUX_SYS1_PLL_P 5 /* 996 MHz */
#define MUX_SYS1_PLL_R 6
#define MUX_PIXEL_PLL_R 7

/*
 * Parent Clock Sources for disp_gen_src_sels (Pixel clocks)
 * From kernel clk-x5.c: { "osc", "disp_pll_p", "pixel_pll_p", "disp_pll_r", "pixel_pll_r" }
 */
#define DISP_MUX_OSC 0         /* 24 MHz */
#define DISP_MUX_DISP_PLL_P 1  /* 297 MHz */
#define DISP_MUX_PIXEL_PLL_P 2 /* 297 MHz or 1800 MHz */
#define DISP_MUX_DISP_PLL_R 3
#define DISP_MUX_PIXEL_PLL_R 4

/* PLL Output Rates (from kernel soc_pll_rates0) */
#define OSC_RATE 24000000UL          /* 24 MHz */
#define CPU_PLL_P_RATE 1200000000UL  /* 1200 MHz */
#define SYS0_PLL_P_RATE 1500000000UL /* 1500 MHz */
#define SYS1_PLL_P_RATE 996000000UL  /* 996 MHz */
#define DISP_PLL_P_RATE 297000000UL  /* 297 MHz */
#define PIXEL_PLL_P_RATE 297000000UL /* 297 MHz (default) */

/* Default Display Clock Rates (from kernel soc_gen_rates) */
#define TOP_APB_CLK_RATE 200000000UL     /* 200 MHz (from CPU_PLL_R) */
#define DC8000_PIXEL_CLK_RATE 27000000UL /* 27 MHz */
#define DC8000_ACLK_RATE 600000000UL     /* 600 MHz */
#define DISP_SIF_ACLK_RATE 600000000UL   /* 600 MHz */
#define DSI_TXESC_CLK_RATE 600000000UL   /* 600 MHz (from CPU_PLL_P/2) */
#define LPWM_CLK_RATE 24000000UL         /* 24 MHz */

/**
 * @brief Legacy one-shot init: DISP PLL, NOC idle, APB/AXI/pixel clocks, DC IOMMU.
 * @param pixel_clock Target pixel frequency (Hz); must be valid for the panel path.
 * @retval 0 on success, negative errno on failure (e.g. -EINVAL, -EIO).
 */
int x5_crm_display_clk_init(unsigned long pixel_clock);

/**
 * @brief Assert/deassert display-related resets in CRM (DSI / DC8000 as implemented).
 * @retval 0 on success, negative errno on failure.
 */
int x5_crm_display_reset(void);

/**
 * @brief Gate off display CRM clocks configured by this driver (legacy path).
 * @retval None.
 */
void x5_crm_display_clk_disable(void);

/**
 * @brief Shared clock bring-up: DISP PLL, NOC, DC8000/SIF/common APB, pixel clock, IOMMU linear map.
 * @param pixel_clock Target pixel frequency (Hz).
 * @retval 0 on success, negative errno on failure.
 */
int x5_crm_display_common_init(unsigned long pixel_clock);

/**
 * @brief DSI-only clocks: DSI APB gates, DPHY cfg clock, DSI TX escape clock.
 * @param pixel_clock Used for logging/context consistency with common init (Hz).
 * @retval 0 on success, negative errno on failure.
 */
int x5_crm_dsi_clk_init(unsigned long pixel_clock);

/**
 * @brief Pulse reset lines for DC8000 only (shared with DSI/BT1120 orchestration).
 * @retval 0 on success, negative errno on failure.
 */
int x5_crm_common_reset(void);

/**
 * @brief Pulse DSI TX host reset (DSI path only).
 * @retval 0 on success, negative errno on failure.
 */
int x5_crm_dsi_reset(void);

/**
 * @brief Disable DC8000/SIF/common generators gated by display common init.
 * @retval None.
 */
void x5_crm_common_clk_disable(void);

/**
 * @brief Disable DSI TXESC and DSI-related APB/DPHY cfg gates.
 * @retval None.
 */
void x5_crm_dsi_clk_disable(void);

/**
 * @brief BT1120/HDMI path: NOC idle, APB gates, AXI and pixel generators from DISP PLL.
 * @param pixel_clock Target pixel frequency (Hz).
 * @retval 0 on success, negative errno on failure.
 */
int x5_crm_bt1120_clk_init(unsigned long pixel_clock);

/**
 * @brief Assert/deassert BT1120 block reset via CRM.
 * @retval 0 on success, negative errno on failure.
 */
int x5_crm_bt1120_reset(void);

/**
 * @brief Gate off BT1120 pixel/AXI generators and related APB clocks.
 * @retval None.
 */
void x5_crm_bt1120_clk_disable(void);

/**
 * @brief Enable LSIO I2C4 peripheral clock (e.g. LT8618 on HDMI board).
 * @retval None.
 */
void x5_crm_i2c4_pclk_enable(void);

/**
 * @brief Enable DSP I2C PCLK and release I2C reset (e.g. SII902x path).
 * @retval None.
 */
void x5_crm_dsp_i2c_pclk_enable(void);

/**
 * @brief Configure LPWM channel clock from OSC and enable generator/APB gate.
 * @param lpwm_id LPWM instance index (0 or 1).
 * @retval 0 on success, negative errno on failure.
 */
int x5_crm_lpwm_clk_init(int lpwm_id);

/**
 * @brief Disable LPWM generator and APB clock for the given instance.
 * @param lpwm_id LPWM instance index (0 or 1).
 * @retval None.
 */
void x5_crm_lpwm_clk_disable(int lpwm_id);

/**
 * @brief Log HPS clock generators and gates relevant to display (debug).
 * @retval None.
 */
void x5_crm_dump_display_clocks(void);

/**
 * @brief Program DISP PLL frequency via table/calculation and SYSCON registers.
 * @param rate Target PLL output (Hz); must be within driver-supported range.
 * @retval 0 on success, negative errno on failure (e.g. -EINVAL, lock timeout).
 */
int x5_crm_disp_pll_init(unsigned long rate);

/**
 * @brief Program PIXEL PLL (when used) to @a rate.
 * @param rate Target PLL output (Hz).
 * @retval 0 on success, negative errno on failure.
 */
int x5_crm_pixel_pll_init(unsigned long rate);

/**
 * @brief Print DISP/PIXEL PLL cfg/postdiv/lock status (debug).
 * @retval None.
 */
void x5_crm_dump_pll_status(void);

/**
 * @brief Clear NOC idle request bits for @a mask (display-related masters).
 * @param mask Bitmask of NOC idle slots (see NOC_IDLE_* in this header).
 * @retval 0 on success, negative errno on timeout or error.
 */
int x5_crm_noc_idle_release(unsigned int mask);

/**
 * @brief Set NOC idle request bits for @a mask.
 * @param mask Bitmask of NOC idle slots.
 * @retval 0 on success, negative errno on failure.
 */
int x5_crm_noc_idle_request(unsigned int mask);

/**
 * @brief Dump NOC idle request/status registers (debug).
 * @retval None.
 */
void x5_crm_dump_noc_idle(void);

/**
 * @brief Disable DC IOMMU mapping programmed by U-Boot.
 * @retval None.
 */
void x5_crm_dc_iommu_disable(void);

/**
 * @brief Enable DC IOMMU with linear/bypass-style mapping used by the display driver.
 * @retval None.
 */
void x5_crm_dc_iommu_enable_linear(void);

/**
 * @brief Dump DC IOMMU control/map registers (debug).
 * @retval None.
 */
void x5_crm_dump_dc_iommu(void);

#endif /* __X5_CRM_H__ */
