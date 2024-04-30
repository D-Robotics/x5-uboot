// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * Copyright (C) 2020, STMicroelectronics - All Rights Reserved
 */

#include <blk.h>
#include <dm.h>
#include <dfu.h>
#include <env.h>
#include <log.h>
#include <memalign.h>

#define DFU_ALT_BUF_LEN SZ_2K	/* if many partitions & mediums, maybe needs to enlarge it again. */

static void board_get_alt_info_mmc(struct udevice *dev, char *buf)
{
	struct disk_partition info;
	int p, len, devnum;
	bool first = true;
	const char *name;
	struct mmc *mmc;
	struct blk_desc *desc;

	mmc = mmc_get_mmc_dev(dev);
	if (!mmc)
		return;

	if (mmc_init(mmc))
		return;

	desc = mmc_get_blk_desc(mmc);
	if (!desc)
		return;

	name = blk_get_if_type_name(desc->if_type);
	devnum = desc->devnum;
	len = strlen(buf);

	if (buf[0] != '\0')
		len += snprintf(buf + len,
				DFU_ALT_BUF_LEN - len, "&");
	len += snprintf(buf + len, DFU_ALT_BUF_LEN - len,
			 "%s %d=", name, devnum);

	/* every interface's 1st alt always is disk.img, fixed to 2G. just enlarge it if not enough */
	len += snprintf(buf + len, DFU_ALT_BUF_LEN - len,
			"%s%d_disk.img raw 0x0 0x%lx;",
			name, devnum, SZ_2G/desc->blksz);

	/* each partition alt info */
	for (p = 1; p <= MAX_SEARCH_PARTITIONS; p++) {
		if (part_get_info(desc, p, &info))
			continue;
		if (!first)
			len += snprintf(buf + len, DFU_ALT_BUF_LEN - len, ";");
		first = false;
		len += snprintf(buf + len, DFU_ALT_BUF_LEN - len,
				"%s%d_%s part %d %d",
				name, devnum, info.name, devnum, p);
	}
}

void set_dfu_alt_info(char *interface, char *devstr)
{
	struct udevice *dev;

	ALLOC_CACHE_ALIGN_BUFFER(char, buf, DFU_ALT_BUF_LEN);

	if (env_get("dfu_alt_info"))
		return;

	memset(buf, 0, sizeof(buf));

	if (CONFIG_IS_ENABLED(MMC)) {
		if (!uclass_get_device(UCLASS_MMC, 0, &dev))
			board_get_alt_info_mmc(dev, buf);

		if (!uclass_get_device(UCLASS_MMC, 1, &dev))
			board_get_alt_info_mmc(dev, buf);
	}

	// TODO: jianghe.xu - ram, mtd & virt medium

	env_set("dfu_alt_info", buf);
	puts("DFU alt info setting: done\n");
}
