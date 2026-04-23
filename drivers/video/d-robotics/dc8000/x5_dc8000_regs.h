/* SPDX-License-Identifier: GPL-2.0+ */
/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2021-2025 Horizon Robotics, Inc.
 *                   All rights reserved.
 *************************************************************************
 * DC8000 / display SYSCON register offsets and helpers (U-Boot).
 * Ported from: kernel dc_8000_nano_reg.h / vs_x5_syscon_bridge.c
 */

#ifndef __X5_DC8000_REGS_H__
#define __X5_DC8000_REGS_H__

#include <linux/types.h>
#include <linux/bitops.h>
#include <asm/io.h>

/*
 * Fixed MMIO bases (X5 single display pipeline)
 *
 * Linux DRM obtains these via platform_ioremap_resource() from DT **reg**.
 * U-Boot bring-up uses the same physical addresses as documented in SoC TRM /
 * kernel DTS for this chip — there is only one DC8000 and one disp SYSCON
 * instance on X5; multi-instance drivers would store per-device __iomem * from
 * probe and pass them to x5_dc_readl()/x5_syscon_readl() instead of the
 * compatibility macros below.
 */
#define DC8000_REG_BASE 0x3e000000

/* Chip Information Registers */
#define GCREG_DC_CHIP_REV_Address 0x02000
#define GCREG_DC_CHIP_DATE_Address 0x02004
#define GCREG_DC_CHIP_PATCH_REV_Address 0x02008
#define GCREG_DC_PRODUCT_ID_Address 0x0200C

/* Framebuffer Configuration */
#define GCREG_FRAME_BUFFER_CONFIG_Address 0x02024
#define GCREG_FRAME_BUFFER_ADDRESS_Address 0x02028
#define GCREG_FRAME_BUFFER_STRIDE_Address 0x0202C
#define GCREG_FRAME_BUFFER_ORIGIN_Address 0x02030
#define GCREG_FRAME_BUFFER_SIZE_Address 0x0206C
#define GCREG_FRAME_BUFFER_BACKGROUND_Address 0x02058
#define GCREG_FRAME_BUFFER_COLOR_KEY_Address 0x0205C
#define GCREG_FRAME_BUFFER_CLEAR_VALUE_Address 0x02064

/* Tile Configuration */
#define GCREG_DC_TILE_IN_CFG_Address 0x02034
#define GCREG_DC_TILE_UV_FRAME_BUFFER_ADR_Address 0x02038
#define GCREG_DC_TILE_UV_FRAME_BUFFER_STR_Address 0x0203C

/* Video layer + compositor (see kernel dc_8000_nano_reg.h) */
#define GCREG_VIDEO_TL_Address 0x02068
#define GCREG_VIDEO_GLOBAL_ALPHA_Address 0x02070
#define GCREG_BLEND_STACK_ORDER_Address 0x02074
#define GCREG_VIDEO_ALPHA_BLEND_CONFIG_Address 0x02078
#define GCREG_OVERLAY_CONFIG_Address 0x0207C
#define GCREG_OVERLAY_CONFIG1_Address 0x020B0

/* Display Timing */
#define GCREG_HDISPLAY_Address 0x02100
#define GCREG_HSYNC_Address 0x02104
#define GCREG_VDISPLAY_Address 0x02110
#define GCREG_VSYNC_Address 0x02114
#define GCREG_CURRENT_LOCATION_Address 0x02118

/* Panel Configuration */
#define GCREG_PANEL_CONFIG_Address 0x020E8
#define GCREG_PANEL_CONTROL_Address 0x020EC	 /* Shadow register control */
#define GCREG_PANEL_FUNCTION_Address 0x020F0 /* Output enable, gamma, dither */
#define GCREG_PANEL_WORKING_Address 0x020F4
#define GCREG_PANEL_STATE_Address 0x020F8

/* Gamma (corrected to match kernel; was shifted +0x4) */
#define GCREG_GAMMA_INDEX_Address 0x0211C
#define GCREG_GAMMA_DATA_Address 0x02120

/* Dither (corrected: kernel 0x20e0/0x20e4, not cursor block) */
#define GCREG_DISPLAY_DITHER_TABLE_LOW_Address 0x020E0
#define GCREG_DISPLAY_DITHER_TABLE_HIGH_Address 0x020E4

/* Cursor (corrected to match kernel) */
#define GCREG_CURSOR_CONFIG_Address 0x02124
#define GCREG_CURSOR_ADDRESS_Address 0x02128
#define GCREG_CURSOR_LOCATION_Address 0x0212C
#define GCREG_CURSOR_BACKGROUND_Address 0x02130
#define GCREG_CURSOR_FOREGROUND_Address 0x02134

/*
 * DBI/DPI — addresses match kernel dc_8000_nano_reg.h:
 *   GCREG_CURSOR_FOREGROUND @ 0x02134, GCREG_DBI_CONFIG @ 0x02140 (distinct registers).
 * An older one-line note suggested equivalence with CURSOR_FOREGROUND; that was
 * misleading — programming DBI does not use the cursor foreground register.
 */
#define GCREG_DBI_CONFIG_Address 0x02140
#define GCREG_DPI_CONFIG_Address 0x02154

/* Soft Reset */
#define GCREG_SOFT_RESET_Address 0x02164
#define GCREG_SOFT_RESET_RESET_RESET BIT(0)

/* Debug Registers */
#define GCREG_DEBUG_CNT_SELECT_Address 0x02170
#define GCREG_DEBUG_CNT_VALUE_Address 0x02174

/*******************************************************************************
** Register Field Definitions
**
** Bit ranges use GCC extension syntax "high : low" (same as kernel VS_* macros).
** ISO C does not define multi-bit anonymous bit-field slices in macros; builds
** rely on GCC/Clang. VS_SET/VS_MASK expand these with __vsFIELDSTART/END.
*******************************************************************************/

/* GCREG_FRAME_BUFFER_CONFIG */
#define GCREG_FRAME_BUFFER_CONFIG_FORMAT 2 : 0
#define GCREG_FRAME_BUFFER_CONFIG_FORMAT_NONE 0x0
#define GCREG_FRAME_BUFFER_CONFIG_FORMAT_A4R4G4B4 0x1
#define GCREG_FRAME_BUFFER_CONFIG_FORMAT_A1R5G5B5 0x2
#define GCREG_FRAME_BUFFER_CONFIG_FORMAT_R5G6B5 0x3
#define GCREG_FRAME_BUFFER_CONFIG_FORMAT_A8R8G8B8 0x4

#define GCREG_FRAME_BUFFER_CONFIG_ENABLE 3 : 3
#define GCREG_FRAME_BUFFER_CONFIG_ENABLE_DISABLED 0x0
#define GCREG_FRAME_BUFFER_CONFIG_ENABLE_ENABLED 0x1

#define GCREG_FRAME_BUFFER_CONFIG_CLEAR_EN 5 : 5
#define GCREG_FRAME_BUFFER_CONFIG_SWIZZLE 18 : 17
#define GCREG_FRAME_BUFFER_CONFIG_UV_SWIZZLE 19 : 19

/* GCREG_HDISPLAY */
#define GCREG_HDISPLAY_DISPLAY_END 11 : 0
#define GCREG_HDISPLAY_TOTAL 27 : 16

/* GCREG_HSYNC - CORRECTED to match kernel dc_8000_nano_reg.h */
#define GCREG_HSYNC_START 12 : 0
#define GCREG_HSYNC_END 28 : 16
#define GCREG_HSYNC_PULSE 30 : 30
#define GCREG_HSYNC_POLARITY 31 : 31

/* GCREG_VDISPLAY */
#define GCREG_VDISPLAY_DISPLAY_END 11 : 0
#define GCREG_VDISPLAY_TOTAL 27 : 16

/* GCREG_VSYNC - CORRECTED to match kernel dc_8000_nano_reg.h */
#define GCREG_VSYNC_START 12 : 0
#define GCREG_VSYNC_END 28 : 16
#define GCREG_VSYNC_PULSE 30 : 30
#define GCREG_VSYNC_POLARITY 31 : 31

/* GCREG_PANEL_CONFIG */
#define GCREG_PANEL_CONFIG_DE 0 : 0
#define GCREG_PANEL_CONFIG_DE_POLARITY 4 : 4
#define GCREG_PANEL_CONFIG_DATA_POLARITY 5 : 5
#define GCREG_PANEL_CONFIG_CLOCK_POLARITY 6 : 6
#define GCREG_PANEL_CONFIG_CLOCK 8 : 8

/* GCREG_PANEL_CONTROL */
#define GCREG_PANEL_CONTROL_VALID 0 : 0

/* GCREG_PANEL_FUNCTION */
#define GCREG_PANEL_FUNCTION_OUTPUT 0 : 0
#define GCREG_PANEL_FUNCTION_GAMMA 1 : 1
#define GCREG_PANEL_FUNCTION_DITHER 2 : 2

/* GCREG_PANEL_WORKING */
#define GCREG_PANEL_WORKING_WORKING 0 : 0

/* GCREG_DBI_CONFIG */
#define GCREG_DBI_CONFIG_DBI_TYPE 1 : 0
#define GCREG_DBI_CONFIG_DBI_DATA_FORMAT 5 : 2
#define GCREG_DBI_CONFIG_BUS_OUTPUT_SEL 6 : 6
#define GCREG_DBI_CONFIG_BUS_OUTPUT_SEL_DPI 0x0
#define GCREG_DBI_CONFIG_BUS_OUTPUT_SEL_DBI 0x1

/* GCREG_DPI_CONFIG */
#define GCREG_DPI_CONFIG_DPI_DATA_FORMAT 2 : 0
#define GCREG_DPI_CONFIG_DPI_DATA_FORMAT_D16CFG1 0x0
#define GCREG_DPI_CONFIG_DPI_DATA_FORMAT_D16CFG2 0x1
#define GCREG_DPI_CONFIG_DPI_DATA_FORMAT_D16CFG3 0x2
#define GCREG_DPI_CONFIG_DPI_DATA_FORMAT_D18CFG1 0x3
#define GCREG_DPI_CONFIG_DPI_DATA_FORMAT_D18CFG2 0x4
#define GCREG_DPI_CONFIG_DPI_DATA_FORMAT_D24 0x5

/* Helper Macros (from kernel dc_8000_nano.h) */
#define __vsFIELDSTART(reg_field)       (0 ? reg_field)
#define __vsFIELDEND(reg_field)         (1 ? reg_field)

#define VS_SET(reg, field, value) \
	(((u32)(value)) << __vsFIELDSTART(reg##_##field))

#define VS_MASK(reg, field) \
	((u32)(GENMASK(__vsFIELDEND(reg##_##field), __vsFIELDSTART(reg##_##field))))

#define VS_SET_FIELD(data, reg, field, value) \
	(((data) & ~VS_MASK(reg, field)) | VS_SET(reg, field, (value)))

/* SYSCON Registers for DC-DSI Bridge (from vs_x5_syscon_bridge.c) */
#define X5_SYSCON_BASE 0x3e0a0000
#define DISP_DC2CSI_DSI_EN 0x10
#define DC2DSI_EN BIT(1)
#define DC2CSI_EN BIT(0)

/* DC -> BT1120 routing (vs_x5_syscon_bridge.c) */
#define DISP_DC2BT1120_EN	0x14
#define DC2BT1120_EN		BIT(0)
#define DISP_BT1120_PIN_MUX_CTRL0 0x9c
#define SELECT_BT1120_DATA	0x55555555
#define DISP_BT1120_PIN_MUX_CTRL1 0xa0
#define SELECT_BT1120_CLK	0x1

/**
 * x5_mmio_offset - byte offset from an iomapped base (standard char pointer math).
 */
static inline void __iomem *x5_mmio_offset(void __iomem *base, u32 byte_off)
{
	return (void __iomem *)((char __iomem *)base + byte_off);
}

static inline u32 x5_dc_readl(void __iomem *base, u32 offset)
{
	return readl(x5_mmio_offset(base, offset));
}

static inline void x5_dc_writel(u32 val, void __iomem *base, u32 offset)
{
	writel(val, x5_mmio_offset(base, offset));
}

static inline u32 x5_syscon_readl(void __iomem *base, u32 offset)
{
	return readl(x5_mmio_offset(base, offset));
}

static inline void x5_syscon_writel(u32 val, void __iomem *base, u32 offset)
{
	writel(val, x5_mmio_offset(base, offset));
}

/*
 * Default X5 single-mapping accessors (phys base); call x5_*_readl/writel with
 * a mapped base when sharing helpers across a future multi-device probe path.
 */
#define dc_readl(off) \
	x5_dc_readl((void __iomem *)(unsigned long)DC8000_REG_BASE, (off))
#define dc_writel(val, off) \
	x5_dc_writel((val), (void __iomem *)(unsigned long)DC8000_REG_BASE, (off))
#define syscon_readl(off) \
	x5_syscon_readl((void __iomem *)(unsigned long)X5_SYSCON_BASE, (off))
#define syscon_writel(val, off) \
	x5_syscon_writel((val), (void __iomem *)(unsigned long)X5_SYSCON_BASE, (off))

#endif /* __X5_DC8000_REGS_H__ */
