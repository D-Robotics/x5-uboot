// SPDX-License-Identifier: GPL-2.0
/*
 * horizon pinmux core definitions
 * Copyright (C) 2022 D-Robotics Holdings Co., Ltd.
 */

#include <common.h>
#include <dm.h>
#include <malloc.h>
#include <mapmem.h>
#include <log.h>
#include <dm/pinctrl.h>
#include <dm/devres.h>
#include <regmap.h>
#include <syscon.h>
#include <fdtdec.h>
#include <linux/bitops.h>
#include <linux/libfdt.h>
#include <asm/global_data.h>
#include <dm/device_compat.h>
#include <linux/io.h>
#include <linux/ioport.h>

#include "common.h"

#define HORIZON_PIN_SIZE 20

#define HORIZON_PULL_UP	  1
#define HORIZON_PULL_DOWN 0

#define INVALID_PINMUX    0x0

static void horizon_pin_drv_str_set(struct horizon_pinctrl_priv *priv, unsigned int pin,
				    unsigned int arg, const struct horizon_pinctrl_priv *info)
{
	void __iomem *drv_str_regs;
	unsigned int val, reg_domain, ds_bits_offset = 0;
	int i;

	/* We have two regs(marked as domain) for
	 * horizon pins schmitter trigger setting
	 */
	for (i = 0; i < info->npins; i++) {
		if (info->pins[i].pin_id == pin) {
			reg_domain     = info->pins[i].reg_domain;
			ds_bits_offset = info->pins[i].ds_bits_offset;
			break;
		}
	}

	if (i >= info->npins)
		return;

	drv_str_regs = priv->base + reg_domain;
	val = readl(drv_str_regs);
	val &= ~(PIN_DS_BIT_MASK << ds_bits_offset);
	val |= arg << ds_bits_offset;
	writel(val, drv_str_regs);
	dev_dbg(priv->dev, "write drv str reg_domain: 0x%x val:0x%x\n", reg_domain, val);

}

static void horizon_pin_schmit_en(struct horizon_pinctrl_priv *priv, unsigned int pin,
				  unsigned int flags, const struct horizon_pinctrl_priv *info)
{
	void __iomem *schmit_tg_regs;
	unsigned int val, reg_domain, st_bits_offset = 0;
	int i;

	/* We have two regs(marked as domain) for
	 * horizon pins schmitter trigger setting
	 */
	for (i = 0; i < info->npins; i++) {
		if (info->pins[i].pin_id == pin) {
			reg_domain     = info->pins[i].reg_domain;
			st_bits_offset = info->pins[i].st_bits_offset;
			break;
		}
	}

	if (i >= info->npins)
		return;

	schmit_tg_regs = priv->base + reg_domain;
	val	       = readl(schmit_tg_regs);

	if (flags)
		val |= st_bits_offset;
	else
		val &= ~st_bits_offset;

	writel(val, schmit_tg_regs);
	dev_dbg(priv->dev, "write schmit en reg_domain: 0x%x val:0x%x\n", reg_domain, val);

}

static void horizon_pin_pull_en(struct horizon_pinctrl_priv *priv, unsigned int pin, unsigned int flags,
				const struct horizon_pinctrl_priv *info)
{
	void __iomem *pull_en_regs;
	void __iomem *pull_select_regs;
	void __iomem *pull_up_regs;
	void __iomem *pull_down_regs;
	unsigned int val, reg_domain, pe_bits_offset, ps_bits_offset, pu_bits_offset,
		pd_bits_offset = 0;
	int i;

	for (i = 0; i < info->npins; i++) {
		if (info->pins[i].pin_id == pin) {
			reg_domain     = info->pins[i].reg_domain;
			pe_bits_offset = info->pins[i].pe_bits_offset;
			ps_bits_offset = info->pins[i].ps_bits_offset;
			pu_bits_offset = info->pins[i].pu_bits_offset;
			pd_bits_offset = info->pins[i].pd_bits_offset;
			break;
		}
	}

	if (i >= info->npins)
		return;

	/* We have two regs(marked as domain) for horizon pins pull en setting */
	if (pe_bits_offset != INVALID_PULL_BIT) {
		pull_en_regs	 = priv->base + reg_domain;
		pull_select_regs = priv->base + reg_domain;
		val		 = readl(pull_en_regs);
		val |= pe_bits_offset;
		writel(val, pull_en_regs);

		val = readl(pull_select_regs);

		if (flags)
			val |= ps_bits_offset;
		else
			val &= ~ps_bits_offset;

		writel(val, pull_select_regs);
		dev_dbg(priv->dev, "write pull en reg_domain: 0x%x val:0x%x\n", reg_domain, val);
		dev_dbg(priv->dev, "write pull select reg_domain: 0x%x val:0x%x\n", reg_domain,
			val);
	} else {
		pull_up_regs = priv->base + reg_domain;
		val	     = readl(pull_up_regs);
		if (flags)
			val |= pu_bits_offset;
		else
			val &= ~pu_bits_offset;
		writel(val, pull_up_regs);

		pull_down_regs = priv->base + reg_domain;
		val	       = readl(pull_down_regs);
		if (flags)
			val &= ~pd_bits_offset;
		else
			val |= pd_bits_offset;

		writel(val, pull_down_regs);
		dev_dbg(priv->dev, "write pull down reg_domain: 0x%x val:0x%x\n", reg_domain, val);
		dev_dbg(priv->dev, "write pull up reg_domain: 0x%x val:0x%x\n", reg_domain, val);
	}
}

static void horizon_pin_pull_disable(struct horizon_pinctrl_priv *priv, unsigned int pin,
				     const struct horizon_pinctrl_priv *info)
{
	void __iomem *pull_en_regs;
	void __iomem *pull_up_regs;
	void __iomem *pull_down_regs;
	unsigned int val, reg_domain, pe_bits_offset, pu_bits_offset, pd_bits_offset = 0;
	int i;
	if (!info || !info->pins || !info->npins) {
		dev_err(priv->dev, "wrong pinctrl info\n");
	}
	for (i = 0; i < info->npins; i++) {
		if (info->pins[i].pin_id == pin) {
			reg_domain     = info->pins[i].reg_domain;
			pe_bits_offset = info->pins[i].pe_bits_offset;
			pu_bits_offset = info->pins[i].pu_bits_offset;
			pd_bits_offset = info->pins[i].pd_bits_offset;
			break;
		}
	}

	if (i >= info->npins)
		return;
	if (pe_bits_offset != INVALID_PULL_BIT) {
		pull_en_regs = priv->base + reg_domain;
		val	     = readl(pull_en_regs);
		val &= ~pe_bits_offset;
		writel(val, pull_en_regs);
		dev_dbg(priv->dev, "write pull en reg_domain: 0x%x val:0x%x\n", reg_domain, val);
	} else {
		pull_up_regs = priv->base + reg_domain;
		val	     = readl(pull_up_regs);
		val &= ~pu_bits_offset;
		writel(val, pull_up_regs);

		pull_down_regs = priv->base + reg_domain;
		val	       = readl(pull_down_regs);
		val &= ~pd_bits_offset;
		writel(val, pull_down_regs);
		dev_dbg(priv->dev, "write pull up en reg_domain: 0x%x val:0x%x\n", reg_domain,
			val);
		dev_dbg(priv->dev, "write pull down en reg_domain: 0x%x val:0x%x\n", reg_domain,
			val);
	}
}

static void horizon_pin_power_source(struct horizon_pinctrl_priv *priv, unsigned int pin,
				  unsigned int flags, const struct horizon_pinctrl_priv *info)
{
	void __iomem *mode_select_regs;
	unsigned int ms_reg_domain, ms_bits_offset = 0;
	int i;
	u32 val;

	for (i = 0; i < info->npins; i++) {
		if (info->pins[i].pin_id == pin) {
			ms_reg_domain  = info->pins[i].ms_reg_domain;
			ms_bits_offset = info->pins[i].ms_bits_offset;
			break;
		}
	}

	if (i >= info->npins)
		return;

	if (ms_bits_offset == INVALID_MS_BIT)
		return;

	if (priv->mscon)
		mode_select_regs = priv->mscon;
	else
		mode_select_regs = priv->base + ms_reg_domain;

	val = readl(mode_select_regs);

	if (flags == HORIZON_IO_PAD_VOLTAGE_IP_CTRL)
		val |= MS_BIT_CTRL;
	else{
		if (flags / 2)
			val &= ~MS_BIT_CTRL;
		if (flags % 2)
			val |= ms_bits_offset;
		else
			val &= ~ms_bits_offset;
	}

	writel(val, mode_select_regs);
	dev_dbg(priv->dev, "val:%d, mode_select_regs:%p\n", val, mode_select_regs);
}


int horizon_pinconf_set(struct udevice *dev, unsigned int pin,  unsigned int param,
			unsigned int arg, const struct horizon_pinctrl_priv *info)
{
	struct horizon_pinctrl_priv *priv = dev_get_priv(dev);
	int ret = 0;

	dev_dbg(priv->dev, "horizon pinconf pin- %d, param=%d, arg=%d\n", pin, param, arg);

	switch (param) {
	case PIN_CONFIG_POWER_SOURCE:
		horizon_pin_power_source(priv, pin, arg, info);
		break;
	case PIN_CONFIG_BIAS_DISABLE:
		horizon_pin_pull_disable(priv, pin, info);
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		/* horizon do not support those configs */
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		horizon_pin_pull_en(priv, pin, HORIZON_PULL_UP, info);
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		horizon_pin_pull_en(priv, pin, HORIZON_PULL_DOWN, info);
		break;
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		horizon_pin_schmit_en(priv, pin, arg, info);
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		horizon_pin_drv_str_set(priv, pin, arg, info);
		break;
	default:
		dev_err(priv->dev, "horizon pinconf pin %d param:%d not support\n", pin,
			param);
		return -ENOTSUPP;
	}

	return ret;
}

static const struct pinconf_param horizon_conf_params[] = {
	{ "bias-disable", PIN_CONFIG_BIAS_DISABLE, 0 },
	{ "bias-pull-up", PIN_CONFIG_BIAS_PULL_UP, 1 },
	{ "bias-pull-down", PIN_CONFIG_BIAS_PULL_DOWN, 1 },
	{ "drive-open-drain", PIN_CONFIG_DRIVE_OPEN_DRAIN, 0 },
	{ "drive-strength", PIN_CONFIG_DRIVE_STRENGTH, 0 },
	{ "input-disable", PIN_CONFIG_INPUT_ENABLE, 0 },
	{ "input-enable", PIN_CONFIG_INPUT_ENABLE, 1 },
	{ "input-schmitt-disable", PIN_CONFIG_INPUT_SCHMITT_ENABLE, 0 },
	{ "input-schmitt-enable", PIN_CONFIG_INPUT_SCHMITT_ENABLE, 1 },
	{ "output-high", PIN_CONFIG_OUTPUT, 1, },
	{ "output-low", PIN_CONFIG_OUTPUT, 0, },
	{ "power-source", PIN_CONFIG_POWER_SOURCE, 0 },
};

static int horizon_pinconf_prop_name_to_param(const char *property,
					       u32 *default_value)
{
	const struct pinconf_param *p, *end;

	p = horizon_conf_params;
	end = p + sizeof(horizon_conf_params) / sizeof(struct pinconf_param);

	/* See if this pctldev supports this parameter */
	for (; p < end; p++) {
		if (!strcmp(property, p->property)) {
			*default_value = p->default_value;
			return p->param;
		}
	}

	*default_value = 0;
	return -EPERM;
}

int horizon_pinctrl_set_state(struct udevice *dev, struct udevice *config, const struct horizon_pinctrl_priv *info)
{
	struct horizon_pinctrl_priv *priv = dev_get_priv(dev);
	const u32 *pin_data;
	int npins, size, pin_size;
	unsigned int mux_reg_offset, mux_reg_bit, mux_mod, pin_id, conf, default_val;
	unsigned int arg, ret, val = 0;
	const char *prop_name;
	const void *value;
	int i, j = 0;
	int property_offset, pcfg_node;
	const void *blob = gd->fdt_blob;
	int prop_len, param;
	ofnode node;

	dev_dbg(dev, "%s: %s\n", __func__, config->name);

	pin_size = HORIZON_PIN_SIZE;

	pin_data = dev_read_prop(config, "horizon,pins", &size);

	if (!size || size % pin_size) {
		dev_err(dev, "Invalid horizon,pins property in node %s\n",
			config->name);
		return -EINVAL;
	}

	npins = size / pin_size;

	for (i = 0; i < npins; i++) {
		pin_id = fdt32_to_cpu(pin_data[j++]);
		mux_reg_offset = fdt32_to_cpu(pin_data[j++]);
		mux_reg_bit = fdt32_to_cpu(pin_data[j++]);
		mux_mod = fdt32_to_cpu(pin_data[j++]);
		conf = fdt32_to_cpu(pin_data[j++]);

		dev_dbg(dev, "pin_id %d, mux_reg_offset 0x%x, "
		"mux_reg_bit 0x%x, mux_mod 0x%x\n",
		pin_id, mux_reg_offset, mux_reg_bit, mux_mod);


		/* Set Mux */
		if (mux_reg_offset == INVALID_PINMUX) {
			dev_dbg(dev, "Pin(%d) does not support mux function\n",
				pin_id);
			return 0;
		}

		val = readl(priv->base + mux_reg_offset);
		val &= ~(MUX_ALT3<< mux_reg_bit);
		val |= mux_mod << mux_reg_bit;
		writel(val, priv->base + mux_reg_offset);

		dev_dbg(priv->dev, "Write: mux_reg_bit %x mux_mod:%x val 0x%x\n", mux_reg_bit,
			mux_mod, val);
		dev_dbg(priv->dev, "mux_reg_offset:%x Pinctrl set pin %d\n", mux_reg_offset, pin_id);

		/* Set config */
		node = ofnode_get_by_phandle(conf);
		if (!ofnode_valid(node))
			return -ENODEV;

		pcfg_node = ofnode_to_offset(node);
		fdt_for_each_property_offset(property_offset, blob, pcfg_node) {
			value = fdt_getprop_by_offset(blob, property_offset,
						      &prop_name, &prop_len);
			if (!value)
				return -ENOENT;
			param = horizon_pinconf_prop_name_to_param(prop_name,
							&default_val);
			if (param < 0)
				break;

			if (prop_len >= sizeof(fdt32_t))
				arg = fdt32_to_cpu(*(fdt32_t *)value);
			else
				arg = default_val;

		ret = horizon_pinconf_set(dev, pin_id, param, arg, info);
		}
	}

	return 0;
}

int horizon_pinctrl_probe(struct udevice *dev, const struct horizon_pinctrl_priv *info)
{
	struct horizon_pinctrl_priv *priv = dev_get_priv(dev);
	struct resource res;
	int ret;

	priv->dev = dev;

	priv->base = dev_remap_addr_index(dev, 0);
	if (!priv->base)
		return -ENOMEM;

	ret		 = dev_read_resource(dev, 1, &res);
	if(ret >= 0)
		priv->mscon = devm_ioremap(dev, res.start, resource_size(&res));

	priv->npins = info->npins;

	dev_info(dev, "Initialized D-Robotics pinctrl driver\n");
	return 0;
}