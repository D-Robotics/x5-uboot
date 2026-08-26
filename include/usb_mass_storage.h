/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2011 Samsung Electrnoics
 * Lukasz Majewski <l.majewski@samsung.com>
 */

#ifndef __USB_MASS_STORAGE_H__
#define __USB_MASS_STORAGE_H__

#define SECTOR_SIZE		0x200
#include <part.h>
#include <linux/usb/composite.h>

/* Wait at maximum 60 seconds for cable connection */
#define UMS_CABLE_READY_TIMEOUT	60

struct ums {
	int (*read_sector)(struct ums *ums_dev,
			   ulong start, lbaint_t blkcnt, void *buf);
	int (*write_sector)(struct ums *ums_dev,
			    ulong start, lbaint_t blkcnt, const void *buf);
	int (*flush)(struct ums *ums_dev);
	unsigned int start_sector;
	unsigned int num_sectors;
	const char *name;
	struct blk_desc block_dev;
#if CONFIG_IS_ENABLED(USB_MASS_STORAGE_WRITE_CACHE)
	void *write_cache;
	lbaint_t write_cache_start;
	lbaint_t write_cache_blocks;
#endif
};

int fsg_init(struct ums *ums_devs, int count, unsigned int controller_idx);
void fsg_cleanup(void);
int fsg_main_thread(void *);
int fsg_add(struct usb_configuration *c);
#endif /* __USB_MASS_STORAGE_H__ */
