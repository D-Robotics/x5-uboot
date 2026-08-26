// SPDX-License-Identifier: GPL-2.0+
/*
 * SDHCI driver for X5 board.
 *
 * Copyright (C) 2023 Verisilicon Inc.
 */

#include <common.h>
#include <asm/gpio.h>
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
/* DWC IP vendor area 1 pointer */
#define DWCMSHC_P_VENDOR_AREA1		0xe8
#define DWCMSHC_AREA1_MASK		GENMASK(11, 0)
/* Offset inside the  vendor area 1 */
#define DWCMSHC_HOST_CTRL3		0x8

#define SD_CLOCK_GATE	BIT(16)
#define SDIO_CLOCK_GATE	BIT(21)
#define EMMC_CLOCK_GATE	BIT(11)

#define HSIO_CLK_EN_REG	0x342100A0
#define HSIO_IO_MODE_SELECT_REG	0x31040040
#define HSIO_SD_MODE_1V8	BIT(1)

#define X5_MST_DLL_DEFAULT	0x3AA40004
#define X5_SLV_DLL_DEFAULT	0x00808080

DECLARE_GLOBAL_DATA_PTR;

/* x5 phy */
#define DWCMSHC_EMMC_PHY_BASE		0x300

struct x5_sdhci_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

struct x5_sdhci_priv {
	struct sdhci_host *host;
	struct gpio_desc power_gpio;
	struct gpio_desc voltage_gpio;
	u32 toggle_interval_us;
	enum mmc_voltage signal_voltage;
	uint has_pad_init;
	bool is_sd;
	u32 mshc_ctrl_addr;
	u8 mshc_ctrl_val;
	u32 clock_gate;
	void *subaddr;
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

int x5_sdhci_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct x5_sdhci_priv *priv = dev_get_priv(host->mmc->dev);
	unsigned int div, clk = 0, timeout;
	u32 reg;

	/* Wait max 20 ms */
	timeout = 200;
	while (sdhci_readl(host, SDHCI_PRESENT_STATE) &
			   (SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT)) {
		if (timeout == 0) {
			printf("%s: Timeout to wait cmd & data inhibit\n",
			       __func__);
			return -EBUSY;
		}

		timeout--;
		udelay(100);
	}

	reg = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	reg &= ~SDHCI_CLOCK_CARD_EN;
	reg &= ~SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, reg, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		return 0;

	if (SDHCI_GET_VERSION(host) >= SDHCI_SPEC_300) {
		/*
		 * Check if the Host Controller supports Programmable Clock
		 * Mode.
		 */
		if (host->clk_mul) {
			for (div = 1; div <= 1024; div++) {
				if ((host->max_clk / div) <= clock)
					break;
			}

			/*
			 * Set Programmable Clock Mode in the Clock
			 * Control register.
			 */
			clk = SDHCI_PROG_CLOCK_MODE;
			div--;
		} else {
			/* Version 3.00 divisors must be a multiple of 2. */
			if (host->max_clk <= clock) {
				div = 1;
			} else {
				for (div = 2;
				     div < SDHCI_MAX_DIV_SPEC_300;
				     div += 2) {
					if ((host->max_clk / div) <= clock)
						break;
				}
			}
			div >>= 1;
		}
	} else {
		/* Version 2.00 divisors must be a power of 2. */
		for (div = 1; div < SDHCI_MAX_DIV_SPEC_200; div *= 2) {
			if ((host->max_clk / div) <= clock)
				break;
		}
		div >>= 1;
	}
	reg = readl(HSIO_CLK_EN_REG);
	writel(reg & (~priv->clock_gate), HSIO_CLK_EN_REG);
	clk |= (div & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT;
	clk |= ((div & SDHCI_DIV_HI_MASK) >> SDHCI_DIV_MASK_LEN)
		<< SDHCI_DIVIDER_HI_SHIFT;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	writel(reg | priv->clock_gate, HSIO_CLK_EN_REG);
	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	/* Wait max 20 ms */
	timeout = 20;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
		& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			printf("%s: Internal clock never stabilised.\n",
			       __func__);
			return -EBUSY;
		}
		timeout--;
		udelay(1000);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	/*
	 * The X5 clock gate can lose the vendor control latch.  Linux restores
	 * MSHC_CTRL after every card-clock enable; do the same here before the
	 * next command is sent.
	 */
	sdhci_writeb(host, priv->mshc_ctrl_val,
		     priv->mshc_ctrl_addr + DWCMSHC_HOST_CTRL3);
	return 0;
}

static int x5_sdhci_set_dll(struct sdhci_host *host, int degrees)
{
	u32 val;
	u16 clk;
	unsigned int timeout_us = 150000;
	int ret = 0;
	struct x5_sdhci_priv *priv = dev_get_priv(host->mmc->dev);

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk &= ~SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	val = readl(priv->subaddr + 0x4);
	val &= 0xFFFF80FF;
	val |= (degrees << 8);
	writel(val, priv->subaddr + 0x4);

	while (1) {
		val = readl(priv->subaddr + 0xC);
		if (((val & 0x7F00) >> 8) == degrees)
			break;
		if (timeout_us < 10) {
			printf("X5 SD: DLL phase %d never stabilised\n", degrees);
			ret = -ETIMEDOUT;
			break;
		}
		timeout_us -= 10;
		udelay(10);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	return ret;
}

static void x5_sdhci_set_pad_voltage(enum mmc_voltage voltage)
{
	u32 val = readl(HSIO_IO_MODE_SELECT_REG);

	if (voltage == MMC_SIGNAL_VOLTAGE_180)
		val |= HSIO_SD_MODE_1V8;
	else
		val &= ~HSIO_SD_MODE_1V8;

	writel(val, HSIO_IO_MODE_SELECT_REG);
}

#define X5_TUNING_MAX 128

static int x5_sdhci_send_status(struct mmc *mmc)
{
	struct mmc_cmd cmd = { 0 };

	cmd.cmdidx = MMC_CMD_SEND_STATUS;
	cmd.cmdarg = mmc->rca << 16;
	cmd.resp_type = MMC_RSP_R1;

	return mmc_send_cmd(mmc, &cmd, NULL);
}

/*
 * A failed CMD19 resets U-Boot's SDHCI data path, so a blind scan starting at
 * phase 0 can poison all later tuning samples.  First find the command-response
 * sampling window with the harmless, data-less CMD13.  Then issue CMD19 only
 * once at the centre of that window to validate the data path.
 */
static int x5_sdhci_execute_sd_tuning(struct mmc *mmc, u8 opcode)
{
	struct sdhci_host *host = mmc->priv;
	bool good[X5_TUNING_MAX] = { false };
	int best_start = -1;
	int best_len = 0;
	int good_count = 0;
	int phase;
	int ret = 0;
	int i;

	if (opcode != MMC_CMD_SEND_TUNING_BLOCK || mmc->bus_width != 4)
		return -EINVAL;

	for (phase = 0; phase < X5_TUNING_MAX; phase++) {
		ret = x5_sdhci_set_dll(host, phase);
		if (!ret)
			ret = x5_sdhci_send_status(mmc);
		good[phase] = !ret;
		if (good[phase])
			good_count++;
	}

	if (!good_count) {
		printf("X5 SD: no valid CMD13 DLL phase at %u Hz\n",
		       mmc->clock);
		return -EIO;
	}

	if (good_count == X5_TUNING_MAX) {
		best_start = 0;
		best_len = X5_TUNING_MAX;
	} else {
		for (i = 0; i < X5_TUNING_MAX; i++) {
			int len = 0;

			if (!good[i] || good[(i + X5_TUNING_MAX - 1) %
						 X5_TUNING_MAX])
				continue;

			while (len < X5_TUNING_MAX &&
			       good[(i + len) % X5_TUNING_MAX])
				len++;
			if (len > best_len) {
				best_start = i;
				best_len = len;
			}
		}
	}

	phase = (best_start + best_len / 2) % X5_TUNING_MAX;
	ret = x5_sdhci_set_dll(host, phase);
	if (!ret)
		ret = mmc_send_tuning(mmc, opcode, NULL);

	printf("X5 SD: CMD13 phase window %d-%d (%d/%d valid), "
	       "selected %d; CMD19 %s (%d)\n",
	       best_start, (best_start + best_len - 1) % X5_TUNING_MAX,
	       best_len, good_count, phase, ret ? "failed" : "passed", ret);
	return ret;
}

static int x5_sdhci_execute_dll_tuning(struct mmc *mmc, u8 opcode)
{
	struct sdhci_host *host = mmc->priv;
	int ret = 0;
	int first_error = 0;
	int i;
	bool v, prev_v = 0, first_v;
	struct range_t {
		int start;
		int end;	/* inclusive */
	};
	struct range_t *ranges;
	unsigned int range_count = 0;
	int longest_range_len = -1;
	int longest_range = -1;
	int middle_phase;

	ranges = valloc((X5_TUNING_MAX / 2 + 1) *sizeof(*ranges));
	if (!ranges)
		return -ENOMEM;

	printf("X5 MMC: DLL tuning CMD%u at %u Hz\n",
	       opcode, mmc->clock);

	/* Try each phase and extract good ranges */
	for (i = 0; i < X5_TUNING_MAX; i++) {
		ret = x5_sdhci_set_dll(host, i);
		if (!ret)
			ret = mmc_send_tuning(mmc, opcode, NULL);
		v = !ret;
		if (ret && !first_error)
			first_error = ret;
		if (!v)
			debug("tuning failed in degree:%d, error:%d\n", i, ret);

		if (i == 0)
			first_v = v;

		if ((!prev_v) && v) {
			range_count++;
			ranges[range_count - 1].start = i;
		}

		if (v) {
			ranges[range_count - 1].end = i;
		}
        else if (i < X5_TUNING_MAX - 2) {
			/*
			 * No need to check too close to an invalid
			 * one since testing bad phases is slow. Skip
			 * the adjacent phase but always test the last phase.
			 */
			i++;
		}

		prev_v = v;
	}

	if (range_count == 0) {
		printf("X5 SD: all sample phases bad (first error %d, "
		       "host-control2 0x%04x)\n",
		       first_error,
		       sdhci_readw(host, SDHCI_HOST_CONTROL2));
		ret = -EIO;
		goto free;
	}

	/* wrap around case, merge the end points */
	if ((range_count > 1) && first_v && v) {
		ranges[0].start = ranges[range_count - 1].start;
		range_count--;
	}

	/* Find the longest range */
	for (i = 0; i < range_count; i++) {
		int len = (ranges[i].end - ranges[i].start + 1);

		if (len < 0)
			len += X5_TUNING_MAX;

		if (longest_range_len < len) {
			longest_range_len = len;
			longest_range = i;
		}

		debug("Good sample phase range %d-%d (%d len)\n",
			ranges[i].start,
			ranges[i].end, len);
	}

	middle_phase = ranges[longest_range].start + longest_range_len / 2;
	middle_phase %= X5_TUNING_MAX;
	ret = x5_sdhci_set_dll(host, middle_phase);
	if (ret)
		goto free;

	printf("X5 SD: tuning range %d-%d (%d phases), selected %d\n",
	       ranges[longest_range].start, ranges[longest_range].end,
	       longest_range_len, middle_phase);

free:
	free(ranges);
	/* set retuning period to enable retuning*/
	return ret;
}

static int x5_sdhci_execute_tuning(struct mmc *mmc, u8 opcode)
{
	struct x5_sdhci_priv *priv = dev_get_priv(mmc->dev);

	if (priv->is_sd)
		return x5_sdhci_execute_sd_tuning(mmc, opcode);

	return x5_sdhci_execute_dll_tuning(mmc, opcode);
}

static void x5_sdhci_set_control_reg(struct sdhci_host *host)
{
	struct mmc *mmc = (struct mmc *)host->mmc;
	struct x5_sdhci_priv *priv = dev_get_priv(mmc->dev);
	int ret;

	if (!priv->has_pad_init) {
		x5_update_phy(host);
		priv->has_pad_init = 1;
	}

	/*
	 * The RDK X5 switches the SD I/O rail with HSIO_GPIO1_1 instead of a
	 * regulator.  The DT marks the GPIO active-low, so a logical active
	 * value selects the physical low level used for 1.8 V.
	 */
	if (priv->is_sd && dm_gpio_is_valid(&priv->voltage_gpio) &&
	    priv->signal_voltage != mmc->signal_voltage) {
		if (mmc->signal_voltage != MMC_SIGNAL_VOLTAGE_330 &&
		    mmc->signal_voltage != MMC_SIGNAL_VOLTAGE_180)
			return;

		ret = dm_gpio_set_value(&priv->voltage_gpio,
					mmc->signal_voltage ==
					MMC_SIGNAL_VOLTAGE_180);
		if (ret) {
			printf("X5 SD: signal-voltage GPIO failed: %d\n", ret);
			return;
		}

		priv->signal_voltage = mmc->signal_voltage;
		x5_sdhci_set_pad_voltage(mmc->signal_voltage);
		printf("X5 SD: signal voltage %s\n",
		       mmc->signal_voltage == MMC_SIGNAL_VOLTAGE_180 ?
		       "1.8 V" : "3.3 V");
	}

	/* Set the SDHCI voltage-select bit and the selected UHS timing. */
	if (priv->is_sd)
		sdhci_set_control_reg(host);
}

static int x5_sdhci_host_power_cycle(struct sdhci_host *host)
{
	struct x5_sdhci_priv *priv = dev_get_priv(host->mmc->dev);
	int ret;

	if (!priv->is_sd || !dm_gpio_is_valid(&priv->power_gpio))
		return 0;

	ret = dm_gpio_set_value(&priv->power_gpio, 0);
	if (ret)
		return ret;
	udelay(priv->toggle_interval_us);

	/* A new SD power-up must always begin with the I/O rail at 3.3 V. */
	if (dm_gpio_is_valid(&priv->voltage_gpio)) {
		ret = dm_gpio_set_value(&priv->voltage_gpio, 0);
		if (ret)
			return ret;
		priv->signal_voltage = MMC_SIGNAL_VOLTAGE_330;
		x5_sdhci_set_pad_voltage(MMC_SIGNAL_VOLTAGE_330);
		udelay(5000);
	}

	ret = dm_gpio_set_value(&priv->power_gpio, 1);
	if (ret)
		return ret;
	udelay(priv->toggle_interval_us);

	printf("X5 SD: TF power cycle complete at 3.3 V\n");
	return 0;
}

const struct sdhci_ops x5_sdhci_ops = {
	.platform_execute_tuning	= &x5_sdhci_execute_tuning,
	.set_control_reg = &x5_sdhci_set_control_reg,
	.platform_set_clock = &x5_sdhci_set_clock,
	.host_power_cycle = &x5_sdhci_host_power_cycle,
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
		udelay(10);
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
	priv->is_sd = dev_read_bool(dev, "sd-socrst");
	if (priv->is_sd && dev_read_bool(dev, "voltage-gpios")) {
		ret = gpio_request_by_name(dev, "voltage-gpios", 0,
					   &priv->voltage_gpio, GPIOD_IS_OUT);
		if (ret) {
			printf("X5 SD: cannot request signal-voltage GPIO: %d\n",
			       ret);
			return ret;
		}
		priv->signal_voltage = MMC_SIGNAL_VOLTAGE_330;
	}
	if (priv->is_sd && dev_read_bool(dev, "power-gpios")) {
		ret = gpio_request_by_name(dev, "power-gpios", 0,
					   &priv->power_gpio, GPIOD_IS_OUT);
		if (ret) {
			printf("X5 SD: cannot request TF power GPIO: %d\n", ret);
			return ret;
		}
		priv->toggle_interval_us = dev_read_u32_default(dev,
								 "toggle-interval-us",
								 20000);
	}

	debug("sdio is at %s %d\n", __FILE__, __LINE__);
	x5_soc_reset(dev);

	/* Match the post-reset DLL initialization used by the X5 Linux driver. */
	writel(X5_MST_DLL_DEFAULT, priv->subaddr);
	writel(X5_SLV_DLL_DEFAULT, priv->subaddr + 0x4);

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
	/*
	 * Do not inherit the reset value: the X5 Linux driver deliberately starts
	 * from zero, which disables the DWC command-conflict check.  Leaving bit 0
	 * set makes the first command at SDR104 fail with timeout/CRC errors.
	 */
	priv->mshc_ctrl_val = 0;
	if (dev_read_bool(dev, "dwcmshc,no-cmd-conflict-check")) {
		priv->mshc_ctrl_val &= ~(BIT(0));
	}
	if (dev_read_bool(dev, "dwcmshc,positive-edge-drive")) {
		priv->mshc_ctrl_val |= BIT(6);
	}
	if (dev_read_bool(dev, "dwcmshc,negative-edge-sample")) {
		priv->mshc_ctrl_val |= BIT(7);
	}
	sdhci_writeb(host, priv->mshc_ctrl_val,
				 priv->mshc_ctrl_addr + DWCMSHC_HOST_CTRL3);

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
	priv->host->ioaddr = (void *)dev_read_addr_index(dev, 0);
	priv->has_pad_init = 1;

	if (IS_ERR(priv->host->ioaddr))
		return PTR_ERR(priv->host->ioaddr);

	if (fdt_get_property(gd->fdt_blob, dev_of_offset(dev), "phy-pads", NULL))
		priv->has_pad_init = 0;

	if (dev_read_bool(dev, "emmc-socrst")) {
		priv->clock_gate = EMMC_CLOCK_GATE;
	} else if (dev_read_bool(dev, "sd-socrst")) {
		priv->clock_gate = SD_CLOCK_GATE;
	} else if (dev_read_bool(dev, "sdio-socrst")) {
		priv->clock_gate = SDIO_CLOCK_GATE;
	}

	priv->subaddr = (void *)dev_read_addr_index(dev, 1);

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
