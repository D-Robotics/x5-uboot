// SPDX-License-Identifier: GPL-2.0+
/*
 * Bind the BT1120 SoC node for DM pinctrl (kernel uses devm_pinctrl_get on this node).
 */
#include <dm.h>
#include <dm/device.h>

static int bt1120_soc_probe(struct udevice *dev)
{
	(void)dev;
	return 0;
}

static const struct udevice_id bt1120_soc_ids[] = {
	{ .compatible = "verisilicon,bt1120" },
	{ }
};

U_BOOT_DRIVER(bt1120_soc) = {
	.name		= "bt1120_soc",
	.id		= UCLASS_NOP,
	.of_match	= bt1120_soc_ids,
	.probe		= bt1120_soc_probe,
};
