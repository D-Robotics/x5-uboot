// SPDX-License-Identifier: GPL-2.0+
/*
 * SDHCI driver for X5 board.
 *
 * Copyright (C) 2023 Verisilicon Inc.
 */

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <malloc.h>
#include <sdhci.h>
#include "mmc_private.h"
#include <linux/delay.h>

#define SDHC_MIN_FREQ	400000
/* DWC IP vendor area 1 pointer */
#define DWCMSHC_P_VENDOR_AREA1		0xe8
#define DWCMSHC_AREA1_MASK		GENMASK(11, 0)
/* Offset inside the  vendor area 1 */
#define DWCMSHC_HOST_CTRL3		0x8
#define NOC_IDLE_REQ_REG0 0x31032000
#define HSIO_SW_RST 0x342100A4
#define NOC_IDLE_STATUS_REG0 0x31032008

DECLARE_GLOBAL_DATA_PTR;

/* x5 phy */
#define DWCMSHC_EMMC_PHY_BASE		0x300

struct x5_sdhci_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

struct x5_sdhci_priv {
	struct sdhci_host *host;
	uint has_pad_init;
	u32 mshc_ctrl_addr;
	u8 mshc_ctrl_val;
};

/*
 * x5 Specific Operations in mmc_host_ops
 */
static inline uint32_t sd_emmc_setbits(u32 value,
				       u32 msb, u32 lsb,
				       u32 set)
{
	return (value & ~GENMASK(msb, lsb)) | (set << lsb);
}

static int x5_init_pad(struct sdhci_host *host)
{
	u32 phy;
	u16 ctrl = 0;

	phy = sdhci_readl(host, DWCMSHC_EMMC_PHY_BASE);
	phy = sd_emmc_setbits(phy, 23, 16, 0x89);
	sdhci_writel(host, phy, DWCMSHC_EMMC_PHY_BASE);

	/* CMDPAD */
	ctrl = sdhci_readw(host, DWCMSHC_EMMC_PHY_BASE + 0x4);
	ctrl = sd_emmc_setbits(ctrl, 12, 9, 2);
	ctrl = sd_emmc_setbits(ctrl,  8, 5, 2);
	ctrl = sd_emmc_setbits(ctrl,  4, 3, 1);
	ctrl = sd_emmc_setbits(ctrl,  2, 0, 1);
	sdhci_writew(host, ctrl, DWCMSHC_EMMC_PHY_BASE + 0x4);

	/* DATPAD */
	ctrl = sdhci_readw(host, DWCMSHC_EMMC_PHY_BASE + 0x6);
	ctrl = sd_emmc_setbits(ctrl, 12, 9, 2);
	ctrl = sd_emmc_setbits(ctrl,  8, 5, 2);
	ctrl = sd_emmc_setbits(ctrl,  4, 3, 1);
	ctrl = sd_emmc_setbits(ctrl,  2, 0, 1);
	sdhci_writew(host, ctrl, DWCMSHC_EMMC_PHY_BASE + 0x6);

	/* CLKPAD */
	ctrl = sdhci_readw(host,  DWCMSHC_EMMC_PHY_BASE + 0x8);
	ctrl = sd_emmc_setbits(ctrl, 12, 9, 2);
	ctrl = sd_emmc_setbits(ctrl,  8, 5, 2);
	ctrl = sd_emmc_setbits(ctrl,  4, 3, 0);
	ctrl = sd_emmc_setbits(ctrl,  2, 0, 1);
	sdhci_writew(host, ctrl, DWCMSHC_EMMC_PHY_BASE + 0x8);

	/* STBPAD */
	ctrl = sdhci_readw(host, DWCMSHC_EMMC_PHY_BASE + 0xa);
	ctrl = sd_emmc_setbits(ctrl, 12, 9, 2);
	ctrl = sd_emmc_setbits(ctrl,  8, 5, 2);
	ctrl = sd_emmc_setbits(ctrl,  4, 3, 2); /* pulldown */
	ctrl = sd_emmc_setbits(ctrl,  2, 0, 1);
	sdhci_writew(host, ctrl, DWCMSHC_EMMC_PHY_BASE + 0xa);

	/* RSTPAD */
	ctrl = sdhci_readw(host, DWCMSHC_EMMC_PHY_BASE + 0xc);
	ctrl = sd_emmc_setbits(ctrl, 12, 9, 2);
	ctrl = sd_emmc_setbits(ctrl,  8, 5, 2);
	ctrl = sd_emmc_setbits(ctrl,  4, 3, 1);
	ctrl = sd_emmc_setbits(ctrl,  2, 0, 1);
	sdhci_writew(host, ctrl, DWCMSHC_EMMC_PHY_BASE + 0xc);

	return 0;
}

static int x5_update_phy(struct sdhci_host *host)
{
	u32 phy;
	u8  byte = 0;

	/* release reset */
	phy = sdhci_readl(host, DWCMSHC_EMMC_PHY_BASE);
	phy = sd_emmc_setbits(phy, 0, 0, 0x1);
	sdhci_writel(host, phy, DWCMSHC_EMMC_PHY_BASE);

	x5_init_pad(host);

	/* SDCLKDL_CNFG */
	byte = sdhci_readb(host, DWCMSHC_EMMC_PHY_BASE + 0x1d);
	byte = sd_emmc_setbits(byte, 0, 0, 1);
	sdhci_writeb(host, byte, DWCMSHC_EMMC_PHY_BASE + 0x1d);

	return 0;
}

#define SDHCI_TUNING_LOOP_COUNT	200

static int x5_sdhci_execute_tuning(struct mmc *mmc, u8 opcode)
{
	struct mmc_cmd cmd;
	struct mmc_data data;
	u32 ctrl;
	struct sdhci_host *host;
	struct x5_sdhci_priv *priv = dev_get_priv(mmc->dev);
	char tuning_loop_counter = SDHCI_TUNING_LOOP_COUNT;

	host = priv->host;

	ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	ctrl |= SDHCI_CTRL_EXEC_TUNING;
	sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);

	sdhci_writel(host, SDHCI_INT_DATA_AVAIL, SDHCI_INT_ENABLE);
	sdhci_writel(host, SDHCI_INT_DATA_AVAIL, SDHCI_SIGNAL_ENABLE);

	do {
		cmd.cmdidx = opcode;
		cmd.resp_type = MMC_RSP_R1;
		cmd.cmdarg = 0;

		data.blocksize = 64;
		data.blocks = 1;
		data.flags = MMC_DATA_READ;

		if (tuning_loop_counter-- == 0)
			break;

		if (cmd.cmdidx == MMC_CMD_SEND_TUNING_BLOCK_HS200 &&
		    mmc->bus_width == 8)
			data.blocksize = 128;

		sdhci_writew(host, SDHCI_MAKE_BLKSZ(SDHCI_DEFAULT_BOUNDARY_ARG,
						    data.blocksize),
			     SDHCI_BLOCK_SIZE);
		sdhci_writew(host, data.blocks, SDHCI_BLOCK_COUNT);
		sdhci_writew(host, SDHCI_TRNS_READ, SDHCI_TRANSFER_MODE);

		mmc_send_cmd(mmc, &cmd, NULL);
		ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);

		if (cmd.cmdidx == MMC_CMD_SEND_TUNING_BLOCK)
			udelay(1);

	} while (ctrl & SDHCI_CTRL_EXEC_TUNING);

	if (tuning_loop_counter < 0) {
		ctrl &= ~SDHCI_CTRL_TUNED_CLK;
		sdhci_writel(host, ctrl, SDHCI_HOST_CONTROL2);
	}

	if (!(ctrl & SDHCI_CTRL_TUNED_CLK)) {
		printf("Tuning failed\n");
		return -1;
	}
	printf("Tuning OK\n");

	/* Enable only interrupts served by the SD controller */
	sdhci_writel(host, SDHCI_INT_DATA_MASK | SDHCI_INT_CMD_MASK,
		     SDHCI_INT_ENABLE);

	/* Mask all sdhci interrupt sources */
	sdhci_writel(host, 0x0, SDHCI_SIGNAL_ENABLE);

	return 0;
}

static void x5_sdhci_set_control_reg(struct sdhci_host *host)
{
	struct mmc *mmc = (struct mmc *)host->mmc;
	struct x5_sdhci_priv *priv = dev_get_priv(mmc->dev);

	if (priv->has_pad_init)
		return;

	x5_update_phy(host);
	priv->has_pad_init = 1;
}

int x5_sdhci_set_ios_post(struct sdhci_host *host)
{
	struct x5_sdhci_priv *priv = dev_get_priv(host->mmc->dev);
	sdhci_writeb(host, priv->mshc_ctrl_val,
				 priv->mshc_ctrl_addr + DWCMSHC_HOST_CTRL3);
	return 0;
}

const struct sdhci_ops x5_sdhci_ops = {
	.set_ios_post = &x5_sdhci_set_ios_post,
	.platform_execute_tuning	= &x5_sdhci_execute_tuning,
	.set_control_reg = &x5_sdhci_set_control_reg,
};

static int x5_soc_reset(struct udevice *dev)
{
	u32 val;
	bool do_socrst = false;
	u32 rst_bit;
	u32 noc_bit;
	u32 noc_bit_shift;

	if (dev_read_bool(dev, "emmc-socrst")) {
		rst_bit = BIT(2);
		noc_bit = BIT(12);
		noc_bit_shift = 12;

		do_socrst = true;
	} else if (dev_read_bool(dev, "sd-socrst")) {
		rst_bit = BIT(3);
		noc_bit = BIT(14);
		noc_bit_shift = 14;

		do_socrst = true;
	} else if (dev_read_bool(dev, "sdio-socrst")) {
		rst_bit = BIT(4);
		noc_bit = BIT(15);
		noc_bit_shift = 15;

		do_socrst = true;
	}

	if (do_socrst) {
		val = readl(NOC_IDLE_REQ_REG0);
		val |= noc_bit;
		writel(val, NOC_IDLE_REQ_REG0);
		do {
			val = readl(NOC_IDLE_STATUS_REG0);
			val = (val & noc_bit) >> noc_bit_shift;
		} while(!val);

		val = readl(HSIO_SW_RST);
		val |= rst_bit;
		writel(val, HSIO_SW_RST);
		val &= ~rst_bit;
		writel(val, HSIO_SW_RST);

		val = readl(NOC_IDLE_REQ_REG0);
		val &= (~noc_bit);
		writel(val, NOC_IDLE_REQ_REG0);
		do {
			val = readl(NOC_IDLE_STATUS_REG0);
			val = (val & noc_bit) >> noc_bit_shift;
		} while(val);
	}

	return 0;
}

static int x5_sdhci_probe(struct udevice *dev)
{
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct x5_sdhci_plat *plat = dev_get_plat(dev);
	struct x5_sdhci_priv *priv = dev_get_priv(dev);
	struct sdhci_host *host;
	struct clk clk;
	u32 bus_clk = 0;
	int ret = 0;

	host = priv->host;

	debug("sdio is at %s %d\n", __FILE__, __LINE__);
	x5_soc_reset(dev);
	ret = clk_get_by_index(dev, 0, &clk);
	if (ret) {
		debug("%s: Failed to get the clock\n", __func__);
		return ret;
	}

	bus_clk = clk_get_rate(&clk);
	if (!bus_clk) {
		debug("%s: Invalid clock value\n", __func__);
		return -EINVAL;
	}

	host->name = dev->name;
	host->ioaddr = (void *)devfdt_get_addr(dev);

	host->quirks = SDHCI_QUIRK_32BIT_DMA_ADDR
		| SDHCI_QUIRK_WAIT_SEND_CMD;

	host->mmc = &plat->mmc;
	host->mmc->dev = dev;
	host->mmc->priv = host;
	upriv->mmc = host->mmc;

	/* Set the card frequency */
	plat->cfg.f_max = dev_read_u32_default(dev, "max-frequency", 0);

	/* Parse MSHC_CTRL values */
	priv->mshc_ctrl_addr = sdhci_readl(host, DWCMSHC_P_VENDOR_AREA1) & DWCMSHC_AREA1_MASK;
	priv->mshc_ctrl_val = sdhci_readb(host, priv->mshc_ctrl_addr + DWCMSHC_HOST_CTRL3);
	if (dev_read_bool(dev, "dwcmshc,no-cmd-conflict-check")) {
		priv->mshc_ctrl_val &= ~(BIT(0));
	}
	if (dev_read_bool(dev, "dwcmshc,positive-edge-drive")) {
		priv->mshc_ctrl_val |= BIT(6);
	}
	if (dev_read_bool(dev, "dwcmshc,negative-edge-sample")) {
		priv->mshc_ctrl_val |= BIT(7);
	}

	ret = mmc_of_parse(dev, &plat->cfg);
	if (ret)
		return ret;

	ret = sdhci_setup_cfg(&plat->cfg, host, plat->cfg.f_max, SDHC_MIN_FREQ);

	if (ret)
		return ret;

	/* Set the IP input clock */
	host->max_clk = bus_clk;

	return sdhci_probe(dev);
}

static int x5_sdhci_ofdata_to_plat(struct udevice *dev)
{
	struct x5_sdhci_priv *priv = dev_get_priv(dev);

	priv->host = calloc(1, sizeof(struct sdhci_host));
	if (!priv->host)
		return -1;

	priv->host->ops = &x5_sdhci_ops;
	priv->host->ioaddr = (void *)dev_read_addr(dev);
	priv->has_pad_init = 1;

	if (IS_ERR(priv->host->ioaddr))
		return PTR_ERR(priv->host->ioaddr);

	if (fdt_get_property(gd->fdt_blob, dev_of_offset(dev), "phy-pads", NULL))
		priv->has_pad_init = 0;

	return 0;
}

static int x5_sdhci_bind(struct udevice *dev)
{
	struct x5_sdhci_plat *plat = dev_get_plat(dev);

	return sdhci_bind(dev, &plat->mmc, &plat->cfg);
}

static const struct udevice_id x5_sdhci_ids[] = {
	{ .compatible = "horizon,x5-sdhci" },
	{ }
};

U_BOOT_DRIVER(x5_sdhci_drv) = {
	.name		= "x5_sdhci",
	.id		= UCLASS_MMC,
	.of_match	= x5_sdhci_ids,
	.of_to_plat	= x5_sdhci_ofdata_to_plat,
	.ops		= &sdhci_ops,
	.bind		= x5_sdhci_bind,
	.probe		= x5_sdhci_probe,
	.priv_auto	= sizeof(struct sdhci_host),
	.plat_auto	= sizeof(struct x5_sdhci_plat),
};
