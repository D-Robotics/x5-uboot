// SPDX-License-Identifier: GPL-2.0
/*
 * Designware master SPI core controller driver
 *
 * Copyright (C) 2014 Stefan Roese <sr@denx.de>
 * Copyright (C) 2020 Sean Anderson <seanga2@gmail.com>
 *
 * Very loosely based on the Linux driver:
 * drivers/spi/spi-dw.c, which is:
 * Copyright (c) 2009, Intel Corporation.
 */

#define LOG_CATEGORY UCLASS_SPI
#include <common.h>
#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <errno.h>
#include <fdtdec.h>
#include <log.h>
#include <malloc.h>
#include <reset.h>
#include <spi.h>
#include <spi-mem.h>
#include <asm/io.h>
#include <asm-generic/gpio.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/compat.h>
#include <linux/iopoll.h>
#include <linux/sizes.h>

/* Register offsets */
#define DW_SPI_CTRLR0			0x00
#define DW_SPI_CTRLR1			0x04
#define DW_SPI_SSIENR			0x08
#define DW_SPI_MWCR			0x0c
#define DW_SPI_SER			0x10
#define DW_SPI_BAUDR			0x14
#define DW_SPI_TXFTLR			0x18
#define DW_SPI_RXFTLR			0x1c
#define DW_SPI_TXFLR			0x20
#define DW_SPI_RXFLR			0x24
#define DW_SPI_SR			0x28
#define DW_SPI_IMR			0x2c
#define DW_SPI_ISR			0x30
#define DW_SPI_RISR			0x34
#define DW_SPI_TXOICR			0x38
#define DW_SPI_RXOICR			0x3c
#define DW_SPI_RXUICR			0x40
#define DW_SPI_MSTICR			0x44
#define DW_SPI_ICR			0x48
#define DW_SPI_DMACR			0x4c
#define DW_SPI_DMATDLR			0x50
#define DW_SPI_DMARDLR			0x54
#define DW_SPI_IDR			0x58
#define DW_SPI_VERSION			0x5c
#define DW_SPI_DR			0x60
#define DW_SPI_RX_SAMPLE_DLY		0xf0
#define DW_SPI_ENHANCE_CTRLR0		0xf4

#define NSEC_PER_SEC	1000000000L

/* Bit fields in CTRLR0 */
/*
 * Only present when SSI_MAX_XFER_SIZE=16. This is the default, and the only
 * option before version 3.23a.
 */
#define CTRLR0_DFS_MASK			GENMASK(3, 0)

#define CTRLR0_FRF_MASK			GENMASK(5, 4)
#define CTRLR0_FRF_SPI			0x0
#define CTRLR0_FRF_SSP			0x1
#define CTRLR0_FRF_MICROWIRE		0x2
#define CTRLR0_FRF_RESV			0x3

#define CTRLR0_MODE_MASK		GENMASK(7, 6)
#define CTRLR0_MODE_SCPH		0x1
#define CTRLR0_MODE_SCPOL		0x2

#define CTRLR0_TMOD_MASK		GENMASK(9, 8)
#define	CTRLR0_TMOD_TR			0x0		/* xmit & recv */
#define CTRLR0_TMOD_TO			0x1		/* xmit only */
#define CTRLR0_TMOD_RO			0x2		/* recv only */
#define CTRLR0_TMOD_EPROMREAD		0x3		/* eeprom read mode */

#define CTRLR0_SLVOE_OFFSET		10
#define CTRLR0_SRL_OFFSET		11
#define CTRLR0_CFS_MASK			GENMASK(15, 12)

/* Only present when SSI_MAX_XFER_SIZE=32 */
#define CTRLR0_DFS_32_MASK		GENMASK(20, 16)

/* The next field is only present on versions after 4.00a */
#define CTRLR0_SPI_FRF_MASK		GENMASK(22, 21)
#define CTRLR0_SPI_FRF_BYTE		0x0
#define	CTRLR0_SPI_FRF_DUAL		0x1
#define	CTRLR0_SPI_FRF_QUAD		0x2

/* Bit fields in CTRLR0 based on DWC_ssi_databook.pdf v1.01a */
#define DWC_SSI_CTRLR0_DFS_MASK		GENMASK(4, 0)
#define DWC_SSI_CTRLR0_FRF_MASK		GENMASK(7, 6)
#define DWC_SSI_CTRLR0_MODE_MASK	GENMASK(9, 8)
#define DW_HSSI_CTRLR0_SCPHA		BIT(8)
#define DW_HSSI_CTRLR0_SCPOL		BIT(9)
#define DWC_SSI_CTRLR0_TMOD_MASK	GENMASK(11, 10)
#define DWC_SSI_CTRLR0_SRL_OFFSET	13
#define DWC_SSI_CTRLR0_SPI_FRF_MASK	GENMASK(23, 22)

/* Bit fields in SR, 7 bits */
#define SR_MASK				GENMASK(6, 0)	/* cover 7 bits */
#define SR_BUSY				BIT(0)
#define SR_TF_NOT_FULL			BIT(1)
#define SR_TF_EMPT			BIT(2)
#define SR_RF_NOT_EMPT			BIT(3)
#define SR_RF_FULL			BIT(4)
#define SR_TX_ERR			BIT(5)
#define SR_DCOL				BIT(6)

/* Bit fields in (R)ISR */

/* TX FIFO Empty */
#define ISR_TXEI			BIT(0)
/* TX FIFO Overflow */
#define ISR_TXOI			BIT(1)
/* RX FIFO Underflow */
#define ISR_RXUI			BIT(2)
/* RX FIFO Overflow */
#define ISR_RXOI			BIT(3)
/* RX FIFO Full */
#define ISR_RXFI			BIT(4)
/* Multi-master contention */
#define ISR_MSTI			BIT(5)
/* XIP Receive FIFO Overflow */
#define ISR_XRXOI			BIT(6)
/* TX FIFO Underflow */
#define ISR_TXUI			BIT(7)
/* AXI Error */
#define ISR_AXIE			BIT(8)
/* SPI TX Error */
#define ISR_SPITE			BIT(10)
/* SSI Done */
#define ISR_DONE			BIT(11)

/* Bit fields in SPI_CTRLR0 */

/*
 * Whether the instruction or address use the value of SPI_FRF or use
 * FRF_BYTE
 */
#define SPI_CTRLR0_TRANS_TYPE_MASK	GENMASK(1, 0)
#define SPI_CTRLR0_TRANS_TYPE_0	0x0
#define SPI_CTRLR0_TRANS_TYPE_1	0x1
#define SPI_CTRLR0_TRANS_TYPE_2	0x2
/* Address length in 4-bit units */
#define SPI_CTRLR0_ADDR_L_MASK		GENMASK(5, 2)
/* Enable mode bits after address in XIP mode */
#define SPI_CTRLR0_XIP_MD_BIT_EN	BIT(7)
/* Instruction length */
#define SPI_CTRLR0_INST_L_MASK		GENMASK(9, 8)
#define INST_L_0			0x0
#define INST_L_4			0x1
#define INST_L_8			0x2
#define INST_L_16			0x3
/* Number of "dummy" cycles */
#define SPI_CTRLR0_WAIT_CYCLES_MASK	GENMASK(15, 11)
/* Stretch the clock if the FIFO over/underflows */
#define SPI_CTRLR0_CLK_STRETCH_EN	BIT(30)

#define RX_TIMEOUT			1000		/* timeout in ms */

/* DW SPI capabilities */
#define DW_SPI_CAP_CS_OVERRIDE		BIT(0) /* Unimplemented */
#define DW_SPI_CAP_KEEMBAY_MST		BIT(1) /* Unimplemented */
#define DW_SPI_CAP_DWC_SSI		BIT(2)
#define DW_SPI_CAP_DFS32		BIT(3)
#define DW_SPI_CAP_ENHANCED		BIT(4)

#define do_read(type) do { \
	type *start = ((type *)rx) + idx; \
	type *end = start + count; \
	do { \
		*start++ = __raw_readl(dr); \
	} while (start < end); \
} while (0)

#define do_write(type) do { \
	type *start = ((type *)tx) + idx; \
	type *end = start + count; \
	do { \
		__raw_writel(*start++, dr); \
	} while (start < end); \
} while (0)

#ifdef CONFIG_TARGET_X5
/* Sunrise5 SoC reset registers */
#define X5_QSPI_NOC_IDLE_CTRL (0x31032004)
#define X5_QSPI_NOC_IDLE_CTRL_IDLE (BIT(6))
#define X5_QSPI_NOC_IDLE_STAT (0x3103200C)
#define X5_QSPI_NOC_IDLE_STAT_BIT (BIT(6))
#define X5_QSPI_IDLE_TIMEOUT_MS (10)

#define X5_QSPI_SW_RST_CTRL (0x342100A4)
#define X5_QSPI_SW_RST_CTRL_ASSERT (BIT(0))
#define X5_QSPI_SW_RST_DELAY_US (10)
#endif

struct dw_spi_plat {
	s32 frequency;		/* Default clock frequency, -1 for none */
	void __iomem *regs;
};

struct dw_spi_priv {
	struct clk clk;
	struct reset_ctl_bulk resets;
	struct gpio_desc cs_gpio;	/* External chip-select gpio */

	u32 (*update_cr0)(struct dw_spi_priv *priv);

	void __iomem *regs;
	unsigned long caps;
	unsigned long bus_clk_rate;
	unsigned int freq;		/* Default frequency */
	unsigned int mode;

	const void *tx;
	const void *tx_end;
	void *rx;
	void *rx_end;
	u32 fifo_len;			/* depth of the FIFO buffer */
	u32 max_xfer;			/* Maximum transfer size (in bits) */
	u32 cur_rx_sample_dly;
	u32 def_rx_sample_dly_ns;

	int bits_per_word;
	int len;
	int frames;			/* Number of frames in the transfer */
	u8 cs;				/* chip select pin */
	u8 tmode;			/* TR/TO/RO/EEPROM */
	u8 type;			/* SPI/SSP/MicroWire */
	u8 spi_frf;			/* BYTE/DUAL/QUAD/OCTAL */
};

static inline u32 dw_read(struct dw_spi_priv *priv, u32 offset)
{
	return __raw_readl(priv->regs + offset);
}

static inline void dw_write(struct dw_spi_priv *priv, u32 offset, u32 val)
{
	__raw_writel(val, priv->regs + offset);
}

static inline u32 dw_spi_update_cr0(struct dw_spi_priv *priv)
{
	u32 cr0;
	u32 current_cr0;
	u32 enabled;
	if (priv->caps & DW_SPI_CAP_DWC_SSI) {
		cr0 = FIELD_PREP(DWC_SSI_CTRLR0_DFS_MASK,
				 priv->bits_per_word - 1)
		    | FIELD_PREP(DWC_SSI_CTRLR0_FRF_MASK, priv->type)
		    | FIELD_PREP(DWC_SSI_CTRLR0_TMOD_MASK, priv->tmode)
		    | FIELD_PREP(DWC_SSI_CTRLR0_SPI_FRF_MASK, priv->spi_frf);
	} else {
		if (priv->caps & DW_SPI_CAP_DFS32)
			cr0 = FIELD_PREP(CTRLR0_DFS_32_MASK,
					 priv->bits_per_word - 1);
		else
			cr0 = FIELD_PREP(CTRLR0_DFS_MASK,
					 priv->bits_per_word - 1);

 		cr0 |= FIELD_PREP(CTRLR0_FRF_MASK, priv->type)
		    |  FIELD_PREP(CTRLR0_TMOD_MASK, priv->tmode)
		    |  FIELD_PREP(CTRLR0_SPI_FRF_MASK, priv->spi_frf);
	}

	if (priv->mode & SPI_CPOL)
		cr0 |= DW_HSSI_CTRLR0_SCPOL;
	if (priv->mode & SPI_CPHA)
		cr0 |= DW_HSSI_CTRLR0_SCPHA;

	current_cr0 = dw_read(priv, DW_SPI_CTRLR0);
	enabled = dw_read(priv, DW_SPI_SSIENR);
	enabled &= BIT(0);
	if ((cr0 & DW_HSSI_CTRLR0_SCPOL) != (current_cr0 & DW_HSSI_CTRLR0_SCPOL)){
		if (cr0 & DW_HSSI_CTRLR0_SCPOL)
			current_cr0 |= DW_HSSI_CTRLR0_SCPOL;
		else
			current_cr0 &= ~DW_HSSI_CTRLR0_SCPOL;

		if (enabled)
			dw_write(priv, DW_SPI_SSIENR, 0);

		dw_write(priv, DW_SPI_CTRLR0, current_cr0);

		if (enabled)
			dw_write(priv, DW_SPI_SSIENR, 1);
	}

	return cr0;
}

static inline u32 dw_spi_update_spi_cr0(const struct spi_mem_op *op)
{
	uint trans_type, wait_cycles;

	/* This assumes support_op has filtered invalid types */
	if (op->addr.buswidth == 1)
		trans_type = SPI_CTRLR0_TRANS_TYPE_0;
	else if (op->cmd.buswidth == 1)
		trans_type = SPI_CTRLR0_TRANS_TYPE_1;
	else
		trans_type = SPI_CTRLR0_TRANS_TYPE_2;

	if (op->dummy.buswidth)
		wait_cycles = op->dummy.nbytes * 8 / op->dummy.buswidth;
	else
		wait_cycles = 0;

    return FIELD_PREP(SPI_CTRLR0_TRANS_TYPE_MASK, trans_type) |
           FIELD_PREP(SPI_CTRLR0_ADDR_L_MASK, op->addr.nbytes * 2) |
           FIELD_PREP(SPI_CTRLR0_INST_L_MASK, INST_L_8) |
           FIELD_PREP(SPI_CTRLR0_WAIT_CYCLES_MASK, wait_cycles) |
		   SPI_CTRLR0_CLK_STRETCH_EN;
}

static u32 dw_spi_dw16_update_cr0(struct dw_spi_priv *priv)
{
	return FIELD_PREP(CTRLR0_DFS_MASK, priv->bits_per_word - 1)
	     | FIELD_PREP(CTRLR0_FRF_MASK, priv->type)
	     | FIELD_PREP(CTRLR0_MODE_MASK, priv->mode)
	     | FIELD_PREP(CTRLR0_TMOD_MASK, priv->tmode);
}

static u32 dw_spi_dw32_update_cr0(struct dw_spi_priv *priv)
{
	return FIELD_PREP(CTRLR0_DFS_32_MASK, priv->bits_per_word - 1)
	     | FIELD_PREP(CTRLR0_FRF_MASK, priv->type)
	     | FIELD_PREP(CTRLR0_MODE_MASK, priv->mode)
	     | FIELD_PREP(CTRLR0_TMOD_MASK, priv->tmode);
}

static u32 dw_spi_dwc_update_cr0(struct dw_spi_priv *priv)
{
	return FIELD_PREP(DWC_SSI_CTRLR0_DFS_MASK, priv->bits_per_word - 1)
	     | FIELD_PREP(DWC_SSI_CTRLR0_FRF_MASK, priv->type)
	     | FIELD_PREP(DWC_SSI_CTRLR0_MODE_MASK, priv->mode)
	     | FIELD_PREP(DWC_SSI_CTRLR0_TMOD_MASK, priv->tmode);
}

static int dw_spi_apb_init(struct udevice *bus, struct dw_spi_priv *priv)
{
	/* If we read zeros from DFS, then we need to use DFS_32 instead */
	dw_write(priv, DW_SPI_SSIENR, 0);
	dw_write(priv, DW_SPI_CTRLR0, 0xffffffff);
	if (FIELD_GET(CTRLR0_DFS_MASK, dw_read(priv, DW_SPI_CTRLR0))) {
		priv->max_xfer = 16;
		priv->update_cr0 = dw_spi_dw16_update_cr0;
	} else {
		priv->max_xfer = 32;
		priv->update_cr0 = dw_spi_dw32_update_cr0;
	}

	return 0;
}

static int dw_spi_apb_k210_init(struct udevice *bus, struct dw_spi_priv *priv)
{
	/*
	 * The Canaan Kendryte K210 SoC DW apb_ssi v4 spi controller is
	 * documented to have a 32 word deep TX and RX FIFO, which
	 * spi_hw_init() detects. However, when the RX FIFO is filled up to
	 * 32 entries (RXFLR = 32), an RX FIFO overrun error occurs. Avoid
	 * this problem by force setting fifo_len to 31.
	 */
	priv->fifo_len = 31;

	return dw_spi_apb_init(bus, priv);
}

#ifdef CONFIG_TARGET_X5
static int dw_ssi_hw_reset_x5(struct udevice *bus)
{
	int ret = 0;
	u32 reg_val;
	s32 idle_timeout = X5_QSPI_IDLE_TIMEOUT_MS;

	/* QSPI assert idle */
	reg_val = (readl(X5_QSPI_NOC_IDLE_CTRL) | X5_QSPI_NOC_IDLE_CTRL_IDLE);
	writel(reg_val, X5_QSPI_NOC_IDLE_CTRL);
	while (idle_timeout) {
		if (!(readl(X5_QSPI_NOC_IDLE_STAT) & X5_QSPI_NOC_IDLE_STAT_BIT)){
			idle_timeout--;
			mdelay(1);
		} else {
			break;
		}
	}

	if (idle_timeout <= 0) {
		dev_err(bus, "%s: QSPI request idle timeout!\n", __func__);
		ret = -1;
		goto exit;
	}

	/* Assert qspi sw rst, wait 10us and deassert */
	reg_val = (readl(X5_QSPI_SW_RST_CTRL) | X5_QSPI_SW_RST_CTRL_ASSERT);
	writel(reg_val, X5_QSPI_SW_RST_CTRL);
	udelay(X5_QSPI_SW_RST_DELAY_US);
	reg_val = (readl(X5_QSPI_SW_RST_CTRL) & ~X5_QSPI_SW_RST_CTRL_ASSERT);
	writel(reg_val, X5_QSPI_SW_RST_CTRL);

	/* QSPI deassert idle */
	idle_timeout = X5_QSPI_IDLE_TIMEOUT_MS;
	reg_val = (readl(X5_QSPI_NOC_IDLE_CTRL) & ~X5_QSPI_NOC_IDLE_CTRL_IDLE);
	writel(reg_val, X5_QSPI_NOC_IDLE_CTRL);
	while (idle_timeout) {
		if ((readl(X5_QSPI_NOC_IDLE_STAT) & X5_QSPI_NOC_IDLE_STAT_BIT)){
			idle_timeout--;
			mdelay(1);
		} else {
			break;
		}
	}

	if (idle_timeout <= 0) {
		dev_err(bus, "%s: QSPI release idle timeout!\n", __func__);
		ret = -1;
		goto exit;
	}

exit:
	return ret;
}
#endif

static int dw_spi_dwc_init(struct udevice *bus, struct dw_spi_priv *priv)
{
	priv->max_xfer = 32;
	priv->update_cr0 = dw_spi_dwc_update_cr0;
	priv->caps = DW_SPI_CAP_DWC_SSI;
#ifdef CONFIG_TARGET_X5
	if (dw_ssi_hw_reset_x5(bus)) {
		dev_err(bus, "QSPI hw reset failed!\n");
		return -1;
	}
#endif
	return 0;
}

static int dw_ssi_init(struct udevice *bus, struct dw_spi_priv *priv)
{
	u32 cr0;

	priv->max_xfer = 32;
	priv->update_cr0 = dw_spi_update_cr0;

	dw_write(priv, DW_SPI_SSIENR, 0);
#ifdef CONFIG_TARGET_X5
	if (dw_ssi_hw_reset_x5(bus)) {
		dev_err(bus, "QSPI hw reset failed!\n");
		return -1;
	}
#else
	dw_write(priv, DW_SPI_CTRLR0, 0xffffffff);
#endif
	cr0 = dw_read(priv, DW_SPI_CTRLR0);

	priv->caps = DW_SPI_CAP_DWC_SSI;

	/*
	 * DWC_SPI always has DFS_32. If we read zeros from DFS, then we need to
	 * use DFS_32 instead
	 */
	if (!FIELD_GET(CTRLR0_DFS_MASK, cr0))
		priv->caps |= DW_SPI_CAP_DFS32;

	/*
	 * If SPI_FRF exists that means we have DUAL, QUAD, or OCTAL. Since we
	 * can't differentiate, just set a general ENHANCED cap and let the
	 * slave decide what to use.
	 */
	if (FIELD_GET(DWC_SSI_CTRLR0_SPI_FRF_MASK, cr0))
		priv->caps |= DW_SPI_CAP_ENHANCED;

	dev_dbg(bus, "cr0:%#x, caps:%#lx\n", cr0, priv->caps);
	dw_write(priv, DW_SPI_SSIENR, 1);
	return 0;
}

static int request_gpio_cs(struct udevice *bus)
{
#if CONFIG_IS_ENABLED(DM_GPIO) && !defined(CONFIG_SPL_BUILD)
	struct dw_spi_priv *priv = dev_get_priv(bus);
	int ret;

	/* External chip select gpio line is optional */
	ret = gpio_request_by_name(bus, "cs-gpios", 0, &priv->cs_gpio,
				   GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	if (ret == -ENOENT)
		return 0;

	if (ret < 0) {
		dev_err(bus, "Couldn't request gpio! (error %d)\n", ret);
		return ret;
	}

	if (dm_gpio_is_valid(&priv->cs_gpio)) {
		dm_gpio_set_dir_flags(&priv->cs_gpio,
				      GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	}

	dev_dbg(bus, "Using external gpio for CS management\n");
#endif
	return 0;
}

static int dw_spi_of_to_plat(struct udevice *bus)
{
	struct dw_spi_plat *plat = dev_get_plat(bus);

	plat->regs = dev_read_addr_ptr(bus);
	if (!plat->regs)
		return -EINVAL;

	/* Use 500KHz as a suitable default */
	plat->frequency = dev_read_u32_default(bus, "spi-max-frequency",
					       500000);

	if (dev_read_bool(bus, "spi-slave"))
		return -EINVAL;

	dev_info(bus, "max-frequency=%d\n", plat->frequency);

	return request_gpio_cs(bus);
}

/* Restart the controller, disable all interrupts, clean rx fifo */
static void spi_hw_init(struct udevice *bus, struct dw_spi_priv *priv)
{
	dw_write(priv, DW_SPI_SSIENR, 0);
	dw_write(priv, DW_SPI_IMR, 0);
	dw_write(priv, DW_SPI_SSIENR, 1);

	/*
	 * Try to detect the FIFO depth if not set by interface driver,
	 * the depth could be from 2 to 256 from HW spec
	 */
	if (!priv->fifo_len) {
		u32 fifo;

		for (fifo = 1; fifo < 256; fifo++) {
			dw_write(priv, DW_SPI_TXFTLR, fifo);
			if (fifo != dw_read(priv, DW_SPI_TXFTLR))
				break;
		}

		priv->fifo_len = (fifo == 1) ? 0 : fifo;
		dw_write(priv, DW_SPI_TXFTLR, 0);
	}
	dw_write(priv, DW_SPI_RXFTLR, priv->fifo_len - 1);
}

/*
 * We define dw_spi_get_clk function as 'weak' as some targets
 * (like SOCFPGA_GEN5 and SOCFPGA_ARRIA10) don't use standard clock API
 * and implement dw_spi_get_clk their own way in their clock manager.
 */
__weak int dw_spi_get_clk(struct udevice *bus, ulong *rate)
{
	struct dw_spi_priv *priv = dev_get_priv(bus);
	int ret;

	ret = clk_get_by_index(bus, 0, &priv->clk);
	if (ret)
		return ret;

	ret = clk_enable(&priv->clk);
	if (ret && ret != -ENOSYS && ret != -ENOTSUPP)
		return ret;

	*rate = clk_get_rate(&priv->clk);
	if (!*rate)
		goto err_rate;

	dev_dbg(bus, "Got clock via device tree: %lu Hz\n", *rate);

	return 0;

err_rate:
	clk_disable(&priv->clk);
	clk_free(&priv->clk);

	return -EINVAL;
}

static int dw_spi_reset(struct udevice *bus)
{
	int ret;
	struct dw_spi_priv *priv = dev_get_priv(bus);

	ret = reset_get_bulk(bus, &priv->resets);
	if (ret) {
		/*
		 * Return 0 if error due to !CONFIG_DM_RESET and reset
		 * DT property is not present.
		 */
		if (ret == -ENOENT || ret == -ENOTSUPP)
			return 0;

		dev_warn(bus, "Couldn't find/assert reset device (error %d)\n",
			 ret);
		return ret;
	}

	ret = reset_deassert_bulk(&priv->resets);
	if (ret) {
		reset_release_bulk(&priv->resets);
		dev_err(bus, "Failed to de-assert reset for SPI (error %d)\n",
			ret);
		return ret;
	}

	return 0;
}

typedef int (*dw_spi_init_t)(struct udevice *bus, struct dw_spi_priv *priv);

static int dw_spi_probe(struct udevice *bus)
{
	dw_spi_init_t init = (dw_spi_init_t)dev_get_driver_data(bus);
	struct dw_spi_plat *plat = dev_get_plat(bus);
	struct dw_spi_priv *priv = dev_get_priv(bus);
	int ret;
	u32 version;

	priv->regs = plat->regs;
	priv->freq = plat->frequency;

	ret = dw_spi_get_clk(bus, &priv->bus_clk_rate);
	if (ret)
		return ret;

	ret = dw_spi_reset(bus);
	if (ret)
		return ret;

	if (!init)
		return -EINVAL;
	ret = init(bus, priv);
	if (ret)
		return ret;

	version = dw_read(priv, DW_SPI_VERSION);
	dev_dbg(bus, "ssi_version_id=%c.%c%c%c ssi_max_xfer_size=%u\n",
		version >> 24, version >> 16, version >> 8, version,
		priv->max_xfer);

	/* Currently only bits_per_word == 8 supported */
	priv->bits_per_word = 8;

	priv->tmode = 0; /* Tx & Rx */

	/* Get default rx sample delay */
	dev_read_u32(bus, "rx-sample-delay-ns", &priv->def_rx_sample_dly_ns);

	/* Basic HW init */
	spi_hw_init(bus, priv);

	return 0;
}

/**
 * dw_writer_enh() - Write data frames to the tx fifo
 * @priv: Driver private info
 * @tx: The tx buffer
 * @idx: The number of data frames already transmitted
 * @tx_frames: The number of data frames left to transmit
 * @rx_frames: The number of data frames left to receive (0 if only
 *             transmitting)
 * @frame_bytes: The number of bytes taken up by one data frame
 *
 * This function writes up to @tx_frames data frames using data from @tx[@idx].
 *
 * Return: The number of frames read
 */
static inline uint dw_writer_enh(struct dw_spi_priv *priv, const void *tx, uint idx,
		      uint tx_frames, uint rx_frames, uint frame_bytes)
{
	u32 tx_room = priv->fifo_len - dw_read(priv, DW_SPI_TXFLR);

	/*
	 * Another concern is about the tx/rx mismatch, we
	 * thought about using (priv->fifo_len - rxflr - txflr) as
	 * one maximum value for tx, but it doesn't cover the
	 * data which is out of tx/rx fifo and inside the
	 * shift registers. So a control from sw point of
	 * view is taken.
	 */
	u32 rxtx_gap = rx_frames - tx_frames;
	u32 count = min3(tx_frames, tx_room, (u32)(priv->fifo_len - rxtx_gap));
	u32 *dr = priv->regs + DW_SPI_DR;
	if (!count)
		return 0;

	switch (frame_bytes) {
	case 1:
		do_write(u8);
		break;
	case 2:
		do_write(u16);
		break;
	case 3:
	case 4:
	default:
		do_write(u32);
		break;
	}
	return count;
}

/**
 * dw_reader_enh() - Read data frames from the rx fifo
 * @priv: Driver private data
 * @rx: The rx buffer
 * @idx: The number of data frames already received
 * @frames: The number of data frames left to receive
 * @frame_bytes: The size of a data frame in bytes
 *
 * This function reads up to @frames data frames from @rx[@idx].
 *
 * Return: The number of frames read
 */
static inline uint dw_reader_enh(struct dw_spi_priv *priv, void *rx, uint idx, uint frames,
		      uint frame_bytes)
{
	u32 rx_lvl = dw_read(priv, DW_SPI_RXFLR);
	u32 count = min(frames, rx_lvl);
	u32 *dr = priv->regs + DW_SPI_DR;

	if (!count)
		return 0;

	switch (frame_bytes) {
	case 1:
		do_read(u8);
		break;
	case 2:
		do_read(u16);
		break;
	case 3:
	case 4:
	default:
		do_read(u32);
		break;
	}
	return count;
}

/**
 * poll_transfer_enh() - Transmit and receive data frames
 * @priv: Driver private data
 * @tx: The tx buffer. May be %NULL to only receive.
 * @rx: The rx buffer. May be %NULL to discard read data.
 * @frames: The number of data frames to transfer
 *
 * Transmit @tx, while recieving @rx.
 *
 * Return: The lesser of the number of frames transmitted or received.
 */
static uint poll_transfer_enh(struct dw_spi_priv *priv, const void *tx, void *rx,
			  uint frames)
{
	uint frame_bytes = priv->bits_per_word >> 3;
	uint tx_idx = 0;
	uint rx_idx = 0;
	uint tx_frames = tx ? frames : 0;
	uint rx_frames = rx ? frames : 0;

	while (tx_frames || rx_frames) {
		if (tx_frames) {
			uint tx_diff = dw_writer_enh(priv, tx, tx_idx, tx_frames,
						 rx_frames, frame_bytes);

			tx_idx += tx_diff;
			tx_frames -= tx_diff;
		}

		if (rx_frames) {
			uint rx_diff = dw_reader_enh(priv, rx, rx_idx, rx_frames,
						 frame_bytes);

			rx_idx += rx_diff;
			rx_frames -= rx_diff;
		}

		/*
		 * If we don't read/write fast enough, the transfer stops.
		 * Don't bother reading out what's left in the FIFO; it's
		 * garbage.
		 */
		if (dw_read(priv, DW_SPI_RISR) & (ISR_RXOI | ISR_TXUI))
			break;
	}
	return min(tx ? tx_idx : rx_idx, rx ? rx_idx : tx_idx);
}


/* Return the max entries we can fill into tx fifo */
static inline u32 tx_max(struct dw_spi_priv *priv)
{
	u32 tx_left, tx_room, rxtx_gap;

	tx_left = (priv->tx_end - priv->tx) / (priv->bits_per_word >> 3);
	tx_room = priv->fifo_len - dw_read(priv, DW_SPI_TXFLR);

	/*
	 * Another concern is about the tx/rx mismatch, we
	 * thought about using (priv->fifo_len - rxflr - txflr) as
	 * one maximum value for tx, but it doesn't cover the
	 * data which is out of tx/rx fifo and inside the
	 * shift registers. So a control from sw point of
	 * view is taken.
	 */
	rxtx_gap = ((priv->rx_end - priv->rx) - (priv->tx_end - priv->tx)) /
		(priv->bits_per_word >> 3);

	return min3(tx_left, tx_room, (u32)(priv->fifo_len - rxtx_gap));
}

/* Return the max entries we should read out of rx fifo */
static inline u32 rx_max(struct dw_spi_priv *priv)
{
	u32 rx_left = (priv->rx_end - priv->rx) / (priv->bits_per_word >> 3);

	return min_t(u32, rx_left, dw_read(priv, DW_SPI_RXFLR));
}

static void dw_writer(struct dw_spi_priv *priv)
{
	u32 max = tx_max(priv);
	u32 txw = 0xFFFFFFFF;

	while (max--) {
		/* Set the tx word if the transfer's original "tx" is not null */
		if (priv->tx_end - priv->len) {
			if (priv->bits_per_word == 8)
				txw = *(u8 *)(priv->tx);
			else
				txw = *(u16 *)(priv->tx);
		}
		dw_write(priv, DW_SPI_DR, txw);
		log_content("tx=0x%02x\n", txw);
		priv->tx += priv->bits_per_word >> 3;
	}
}

static void dw_reader(struct dw_spi_priv *priv)
{
	u32 max = rx_max(priv);
	u16 rxw;

	while (max--) {
		rxw = dw_read(priv, DW_SPI_DR);
		log_content("rx=0x%02x\n", rxw);

		/* Care about rx if the transfer's original "rx" is not null */
		if (priv->rx_end - priv->len) {
			if (priv->bits_per_word == 8)
				*(u8 *)(priv->rx) = rxw;
			else
				*(u16 *)(priv->rx) = rxw;
		}
		priv->rx += priv->bits_per_word >> 3;
	}
}

static int poll_transfer(struct dw_spi_priv *priv)
{
	do {
		dw_writer(priv);
		dw_reader(priv);
	} while (priv->rx_end > priv->rx);

	return 0;
}

/*
 * We define external_cs_manage function as 'weak' as some targets
 * (like MSCC Ocelot) don't control the external CS pin using a GPIO
 * controller. These SoCs use specific registers to control by
 * software the SPI pins (and especially the CS).
 */
__weak void external_cs_manage(struct udevice *dev, bool on)
{
#if CONFIG_IS_ENABLED(DM_GPIO) && !defined(CONFIG_SPL_BUILD)
	struct dw_spi_priv *priv = dev_get_priv(dev->parent);

	if (!dm_gpio_is_valid(&priv->cs_gpio))
		return;

	dm_gpio_set_value(&priv->cs_gpio, on ? 1 : 0);
#endif
}

static int dw_spi_xfer(struct udevice *dev, unsigned int bitlen,
		       const void *dout, void *din, unsigned long flags)
{
	struct udevice *bus = dev->parent;
	struct dw_spi_priv *priv = dev_get_priv(bus);
	const u8 *tx = dout;
	u8 *rx = din;
	int ret = 0;
	u32 cr0 = 0;
	u32 val;
	u32 cs;

	/* spi core configured to do 8 bit transfers */
	if (bitlen % 8) {
		dev_err(dev, "Non byte aligned SPI transfer.\n");
		return -1;
	}

	/* Start the transaction if necessary. */
	if (flags & SPI_XFER_BEGIN)
		external_cs_manage(dev, false);

	if (rx && tx)
		priv->tmode = CTRLR0_TMOD_TR;
	else if (rx)
		priv->tmode = CTRLR0_TMOD_RO;
	else
		/*
		 * In transmit only mode (CTRL0_TMOD_TO) input FIFO never gets
		 * any data which breaks our logic in poll_transfer() above.
		 */
		priv->tmode = CTRLR0_TMOD_TR;

	cr0 = priv->update_cr0(priv);

	priv->len = bitlen >> 3;

	priv->tx = (void *)tx;
	priv->tx_end = priv->tx + priv->len;
	priv->rx = rx;
	priv->rx_end = priv->rx + priv->len;

	/* Disable controller before writing control registers */
	dw_write(priv, DW_SPI_SSIENR, 0);

	dev_dbg(dev, "cr0=%08x rx=%p tx=%p len=%d [bytes]\n", cr0, rx, tx,
		priv->len);
	/* Reprogram cr0 only if changed */
	if (dw_read(priv, DW_SPI_CTRLR0) != cr0)
		dw_write(priv, DW_SPI_CTRLR0, cr0);

	/*
	 * Configure the desired SS (slave select 0...3) in the controller
	 * The DW SPI controller will activate and deactivate this CS
	 * automatically. So no cs_activate() etc is needed in this driver.
	 */
	cs = spi_chip_select(dev);
	dw_write(priv, DW_SPI_SER, 1 << cs);

	/* Enable controller after writing control registers */
	dw_write(priv, DW_SPI_SSIENR, 1);

	/* Start transfer in a polling loop */
	ret = poll_transfer(priv);

	/*
	 * Wait for current transmit operation to complete.
	 * Otherwise if some data still exists in Tx FIFO it can be
	 * silently flushed, i.e. dropped on disabling of the controller,
	 * which happens when writing 0 to DW_SPI_SSIENR which happens
	 * in the beginning of new transfer.
	 */
	if (readl_poll_timeout(priv->regs + DW_SPI_SR, val,
			       (val & SR_TF_EMPT) && !(val & SR_BUSY),
			       RX_TIMEOUT * 1000)) {
		ret = -ETIMEDOUT;
	}

	/* Stop the transaction if necessary */
	if (flags & SPI_XFER_END)
		external_cs_manage(dev, true);

	return ret;
}

/*
 * This function is necessary for reading SPI flash with the native CS
 * c.f. https://lkml.org/lkml/2015/12/23/132
 */
static int dw_spi_exec_op(struct spi_slave *slave, const struct spi_mem_op *op)
{
	bool read = op->data.dir == SPI_MEM_DATA_IN;
	int pos, i, ret = 0;
	struct udevice *bus = slave->dev->parent;
	struct dw_spi_priv *priv = dev_get_priv(bus);
	struct spi_mem_op *mut_op = (struct spi_mem_op *)op;
	u8 op_len = op->cmd.nbytes + op->addr.nbytes + op->dummy.nbytes;
	u8 op_buf[op_len];
	u32 cr0, spi_cr0, val;
	u32 rx_sample_dly;
	u32 rx_sample_dly_ns;

	/* Only bytes are supported for spi-mem transfers */
	if (priv->bits_per_word != 8)
		return -EINVAL;

	switch (op->data.buswidth) {
	case 0:
	case 1:
		priv->spi_frf = CTRLR0_SPI_FRF_BYTE;
		break;
	case 2:
		priv->spi_frf = CTRLR0_SPI_FRF_DUAL;
		break;
	case 4:
		priv->spi_frf = CTRLR0_SPI_FRF_QUAD;
		break;
	default:
		return -EINVAL;
	}

	if (read)
		if (priv->spi_frf == CTRLR0_SPI_FRF_BYTE)
			priv->tmode = CTRLR0_TMOD_EPROMREAD;
		else
			priv->tmode = CTRLR0_TMOD_RO;
	else
		if (priv->spi_frf == CTRLR0_SPI_FRF_BYTE)
			priv->tmode = CTRLR0_TMOD_TR;
		else
			priv->tmode = CTRLR0_TMOD_TO;

	cr0 = dw_spi_update_cr0(priv);
	spi_cr0 = dw_spi_update_spi_cr0(op);
	dev_dbg(bus, "cr0=%08x spi_cr0=%08x buf=%p len=%u [bytes]\n", cr0,
		spi_cr0, op->data.buf.in, op->data.nbytes);

	dw_write(priv, DW_SPI_SSIENR, 0);
	external_cs_manage(slave->dev, false);
	dw_write(priv, DW_SPI_SER, 1 << spi_chip_select(slave->dev));
	dw_write(priv, DW_SPI_CTRLR0, cr0);
	if (priv->spi_frf != CTRLR0_SPI_FRF_BYTE)
		dw_write(priv, DW_SPI_ENHANCE_CTRLR0, spi_cr0);
	dw_write(priv, DW_SPI_CTRLR1, op->data.nbytes - 1);
	if (priv->spi_frf == CTRLR0_SPI_FRF_QUAD && priv->tmode == CTRLR0_TMOD_TO) {
		val = dw_read(priv, DW_SPI_TXFTLR);
		val &= (unsigned int)0xffff;
		val |= 0x2 << 16;
		dw_write(priv, DW_SPI_TXFTLR, val);
	} else
		dw_write(priv, DW_SPI_TXFTLR, 0);
	/* Update RX sample delay if required */
	if (dev_read_u32(slave->dev,"rx-sample-delay-ns", &rx_sample_dly_ns) != 0)
			/* Use default controller value */
			rx_sample_dly_ns = priv->def_rx_sample_dly_ns;
	rx_sample_dly = DIV_ROUND_CLOSEST(rx_sample_dly_ns,
					NSEC_PER_SEC /
					priv->bus_clk_rate);

	if (priv->cur_rx_sample_dly != rx_sample_dly) {
		dw_write(priv, DW_SPI_RX_SAMPLE_DLY, rx_sample_dly);
		priv->cur_rx_sample_dly = rx_sample_dly;
	}
	dw_write(priv, DW_SPI_SSIENR, 1);

	/* Write out the instruction */
	if (priv->spi_frf == CTRLR0_SPI_FRF_BYTE) {
		/* From spi_mem_exec_op */
		pos = 0;
		op_buf[pos++] = op->cmd.opcode;
		if (op->addr.nbytes) {
			for (i = 0; i < op->addr.nbytes; i++)
				op_buf[pos + i] = op->addr.val >>
					(8 * (op->addr.nbytes - i - 1));

			pos += op->addr.nbytes;
		}
		memset(op_buf + pos, 0xff, op->dummy.nbytes);
		dw_writer_enh(priv, &op_buf, 0, op_len, 0, sizeof(u8));
	} else {
		writel(op->cmd.opcode, priv->regs + DW_SPI_DR);
		writel(op->addr.val, priv->regs + DW_SPI_DR);
	}

	/*
	 * XXX: The following are tight loops! Enabling debug messages may cause
	 * them to fail because we are not reading/writing the fifo fast enough.
	 */
	if (read)
		mut_op->data.nbytes = poll_transfer_enh(priv, NULL, op->data.buf.in,
						    op->data.nbytes);
	else
		mut_op->data.nbytes = poll_transfer_enh(priv, op->data.buf.out,
						    NULL, op->data.nbytes);
	/*
	 * Ensure the data (or the instruction for zero-data instructions) has
	 * been transmitted from the fifo/shift register before disabling the
	 * device.
	 */
	if (readl_poll_timeout(priv->regs + DW_SPI_SR, val,
			       (val & SR_TF_EMPT) && !(val & SR_BUSY),
			       RX_TIMEOUT * 1000)) {
		dev_dbg(bus, "timed out; sr=%x\n", dw_read(priv, DW_SPI_SR));
		ret = -ETIMEDOUT;
	}
	dw_write(priv, DW_SPI_SER, 0);
	external_cs_manage(slave->dev, true);

	dev_dbg(bus, "%u bytes xfered\n", op->data.nbytes);
	return ret;
}

/* The size of ctrl1 limits data transfers to 64K */
static int dw_spi_adjust_op_size(struct spi_slave *slave, struct spi_mem_op *op)
{
	op->data.nbytes = min(op->data.nbytes, (unsigned int)SZ_64K);

	return 0;
}

bool dw_spi_supports_op(struct spi_slave *slave, const struct spi_mem_op *op)
{
	struct dw_spi_priv *priv = dev_get_priv(slave->dev->parent);

	if (!spi_mem_default_supports_op(slave, op))
		return false;

	/*
	 * Everything before the data must fit in the fifo.
	 * In EEPROM mode we also need to fit the dummy.
	 */
	if (1 + op->addr.nbytes +
	    (op->data.buswidth == 1 ? op->dummy.nbytes : 0) > priv->fifo_len)
		return false;

	if (op->cmd.buswidth == 1 &&
	    (!op->addr.nbytes || op->addr.buswidth == 1))
		return true;

	if (op->cmd.buswidth == 1 &&
	    (!op->addr.nbytes || op->addr.buswidth == op->data.buswidth))
		return true;

	if (op->cmd.buswidth == op->data.buswidth &&
	    (!op->addr.nbytes || op->addr.buswidth == op->data.buswidth))
		return true;

	return false;
}

static const struct spi_controller_mem_ops dw_spi_mem_ops = {
	.exec_op = dw_spi_exec_op,
	.supports_op = dw_spi_supports_op,
	.adjust_op_size = dw_spi_adjust_op_size,
};

static int dw_spi_set_speed(struct udevice *bus, uint speed)
{
	struct dw_spi_plat *plat = dev_get_plat(bus);
	struct dw_spi_priv *priv = dev_get_priv(bus);
	u16 clk_div;

	if (speed > plat->frequency)
		speed = plat->frequency;

	/* Disable controller before writing control registers */
	dw_write(priv, DW_SPI_SSIENR, 0);

	/* clk_div doesn't support odd number */
	clk_div = priv->bus_clk_rate / speed;
	clk_div = (clk_div + 1) & 0xfffe;
	dw_write(priv, DW_SPI_BAUDR, clk_div);

	/* Enable controller after writing control registers */
	dw_write(priv, DW_SPI_SSIENR, 1);

	priv->freq = speed;
	dev_dbg(bus, "speed=%d clk_div=%d\n", priv->freq, clk_div);

	return 0;
}

static int dw_spi_set_mode(struct udevice *bus, uint mode)
{
	struct dw_spi_priv *priv = dev_get_priv(bus);

	if (!(priv->caps & DW_SPI_CAP_ENHANCED) &&
	    (mode & (SPI_RX_DUAL | SPI_TX_DUAL |
		     SPI_RX_QUAD | SPI_TX_QUAD |
		     SPI_RX_OCTAL | SPI_TX_OCTAL)))
		return -EINVAL;

	/*
	 * Can't set mode yet. Since this depends on if rx, tx, or
	 * rx & tx is requested. So we have to defer this to the
	 * real transfer function.
	 */
	priv->mode = mode;
	dev_dbg(bus, "mode=%d\n", priv->mode);

	return 0;
}

static int dw_spi_remove(struct udevice *bus)
{
	struct dw_spi_priv *priv = dev_get_priv(bus);
	int ret;

	ret = reset_release_bulk(&priv->resets);
	if (ret)
		return ret;

#if CONFIG_IS_ENABLED(CLK)
	ret = clk_disable(&priv->clk);
	if (ret)
		return ret;

	clk_free(&priv->clk);
	if (ret)
		return ret;
#endif
	return 0;
}

static const struct dm_spi_ops dw_spi_ops = {
	.xfer		= dw_spi_xfer,
	.mem_ops	= &dw_spi_mem_ops,
	.set_speed	= dw_spi_set_speed,
	.set_mode	= dw_spi_set_mode,
	/*
	 * cs_info is not needed, since we require all chip selects to be
	 * in the device tree explicitly
	 */
};

static const struct udevice_id dw_spi_ids[] = {
	/* Generic compatible strings */

	{ .compatible = "snps,dw-apb-ssi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "snps,dw-apb-ssi-3.20a", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "snps,dw-apb-ssi-3.22a", .data = (ulong)dw_spi_apb_init },
	/* First version with SSI_MAX_XFER_SIZE */
	{ .compatible = "snps,dw-apb-ssi-3.23a", .data = (ulong)dw_spi_apb_init },
	/* First version with Dual/Quad SPI; unused by this driver */
	{ .compatible = "snps,dw-apb-ssi-4.00a", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "snps,dw-apb-ssi-4.01", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "snps,dwc-ssi-1.01a", .data = (ulong)dw_spi_dwc_init },
	{ .compatible = "snps,dwc-ssi-2.00a", .data = (ulong)dw_ssi_init },

	/* Compatible strings for specific SoCs */

	/*
	 * Both the Cyclone V and Arria V share a device tree and have the same
	 * version of this device. This compatible string is used for those
	 * devices, and is not used for sofpgas in general.
	 */
	{ .compatible = "altr,socfpga-spi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "altr,socfpga-arria10-spi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "canaan,k210-spi", .data = (ulong)dw_spi_apb_k210_init},
	{ .compatible = "canaan,k210-ssi", .data = (ulong)dw_spi_dwc_init },
	{ .compatible = "intel,stratix10-spi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "intel,agilex-spi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "mscc,ocelot-spi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "mscc,jaguar2-spi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "snps,axs10x-spi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "snps,hsdk-spi", .data = (ulong)dw_spi_apb_init },
	{ }
};

U_BOOT_DRIVER(dw_spi) = {
	.name = "dw_spi",
	.id = UCLASS_SPI,
	.of_match = dw_spi_ids,
	.ops = &dw_spi_ops,
	.of_to_plat = dw_spi_of_to_plat,
	.plat_auto	= sizeof(struct dw_spi_plat),
	.priv_auto	= sizeof(struct dw_spi_priv),
	.probe = dw_spi_probe,
	.remove = dw_spi_remove,
};
