// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 VeriSilicon Holdings Co., Ltd.
 */

#include <common.h>
#include <asm/io.h>
#include <spi.h>
#include <linux/bitops.h>

#define SYS_CS_BASE		0x0a23001c /* qspi1 */

#define CS_MASK			(0x0f << 4)
#define CS_HIGH			(0x03 << 6)
#define CS_LOW			(0x01 << 6)

void external_cs_manage(struct udevice *dev, bool enable)
{
	u32 cs = spi_chip_select(dev);
	u32 sw_mode = 0;

	if (cs < 4) {
		sw_mode = readl(SYS_CS_BASE) & ~(CS_MASK);

		if (!enable)
			sw_mode |= CS_LOW;
		else
			sw_mode |= CS_HIGH;

		writel(sw_mode, SYS_CS_BASE);
	}
}
