// SPDX-License-Identifier: GPL-2.0+
/*
 * Zhaohui Shi <zhaohui.shi@horizon.ai>
 *
 * Copyright (c) 2022 Horizon , Inc
 */

#include <common.h>
#include <dm.h>
#include <hb_info.h>

struct btype_bus_plat {
	u32 base;
	u32 size;
	u32 target;
};

fdt_addr_t btype_bus_translate(struct udevice *dev, fdt_addr_t addr)
{
	struct btype_bus_plat *plat = dev_get_uclass_plat(dev);

	if (addr >= plat->base && addr < plat->base + plat->size)
		addr = (addr - plat->base) + plat->target;

	return addr;
}

static int btype_bus_post_bind(struct udevice *dev)
{
#if CONFIG_IS_ENABLED(OF_PLATDATA)
	return 0;
#else
	const char *btypdev;
	struct udevice *bdev;
	u32 board_id;
	u32 btypes[64];
	ofnode node;
	u32 cell[3];
	int size;
	int ret;
	int count;
	int i;

	ret = dev_read_u32_array(dev, "ranges", cell, ARRAY_SIZE(cell));
	if (!ret) {
		struct btype_bus_plat *plat = dev_get_uclass_plat(dev);

		plat->base = cell[0];
		plat->target = cell[1];
		plat->size = cell[2];
	}
	btypdev = dev_read_string(dev, "btype-dev");
	if (btypdev) {
		node = ofnode_path(btypdev);
		device_get_global_by_ofnode(node, &bdev);
		(void) bdev;
	}
	size = dev_read_size(dev, "supported-btype");
	if (size % sizeof(u32) == 0) {

		count = size / sizeof(u32);
		if (count > ARRAY_SIZE(btypes)) {
			return -EINVAL;
		}

		ret = hb_board_id_get(&board_id);
		if (ret)
			return -EINVAL;
		ret = dev_read_u32_array(dev, "supported-btype", btypes, count);
		if (ret < 0)
			return -EINVAL;

		for ( i = 0; i < count; i++) {
			debug("%s: [%s] supported-btype[%d] = %x\n",
				__func__, dev->name, i, btypes[i]);
			if (board_id == btypes[i]) {
				return dm_scan_fdt_dev(dev);
			}
		}

		return 0;
	}
	return dm_scan_fdt_dev(dev);
#endif
}

UCLASS_DRIVER(btype_bus) = {
	.id		= UCLASS_SIMPLE_BUS,
	.name		= "btype_bus",
	.post_bind	= btype_bus_post_bind,
	.per_device_plat_auto = sizeof(struct btype_bus_plat),
};

static const struct udevice_id generic_btype_bus_ids[] = {
	{ .compatible = "btype-bus" },
	{ }
};

U_BOOT_DRIVER(btype_bus_drv) = {
	.name	= "board_type_btype_bus",
	.id	= UCLASS_SIMPLE_BUS,
	.of_match = generic_btype_bus_ids,
};
