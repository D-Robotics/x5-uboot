/* SPDX-License-Identifier: GPL-2.0 */
/*
 * horizon pinmux core definitions
 * Copyright (C) 2022 D-Robotics Holdings Co., Ltd.
 */

#ifndef __DRIVERS_PINCTRL_HORIZON_H
#define __DRIVERS_PINCTRL_HORIZON_H

#include <linux/types.h>
#include <dt-bindings/pinctrl/horizon-pinfunc.h>

/* horizon pin description */
struct horizon_pin_desc {
	unsigned int pin_id;
	const char *name;
	unsigned int reg_domain;
	unsigned int ms_reg_domain;
	unsigned int ds_bits_offset;
	unsigned int pe_bits_offset;
	unsigned int ps_bits_offset;
	unsigned int pu_bits_offset;
	unsigned int pd_bits_offset;
	unsigned int st_bits_offset;
	unsigned int ms_bits_offset;
};

#ifndef _PIN
#define _PIN(id, nm, reg, ms_reg, ds, pe, ps, pu, pd, st, ms)                         \
	{                                                                             \
		.pin_id = id, .name = nm, .reg_domain = reg, .ms_reg_domain = ms_reg, \
		.ds_bits_offset = ds, .pe_bits_offset = pe, .ps_bits_offset = ps,     \
		.pu_bits_offset = pu, .pd_bits_offset = pd, .st_bits_offset = st,     \
		.ms_bits_offset = ms,                                                 \
	}
#endif

#define INVALID_PULL_BIT   BIT(9)
#define INVALID_MS_BIT	   BIT(10)
#define INVALID_REG_DOMAIN 0x0
#define MS_BIT_CTRL	   BIT(0)
#define PIN_DS_BIT_MASK GENMASK(3,0)

/**
 * @dev:   pointer back to containing device
 * @pctl:  pointer to pinctrl device
 * @pins:  pointer to pinctrl pin description device
 * @npins: number of pins in description device
 * @base:  address of the controller memory
 * @input_sel_base:  address of the insel controller memory
 * @pin_regs:  abstract of a horizon pin
 * @group_index:  index of a group
 * @mutex: mutex
 */
struct horizon_pinctrl_priv {
    struct udevice *dev;
	//struct pinctrl_dev *pctl;
	//struct gpio_chip gpio_chip;
	const struct horizon_pin_desc *pins;
	unsigned int npins;
	void __iomem *base;
	void __iomem **gpio_bank_base;
	void __iomem *mscon;
	unsigned int gpio_bank_num;
	//struct horizon_pin *pin_regs;
	//unsigned int group_index;
	//struct mutex mutex; /* mutex */
};

extern const struct pinmux_ops horizon_pmx_ops;
extern const struct dev_pm_ops horizon_pinctrl_pm_ops;

int horizon_pinconf_set(struct udevice *dev, unsigned int pin,  unsigned int param,
			unsigned int arg, const struct horizon_pinctrl_priv *info);
int horizon_pinctrl_probe(struct udevice *dev, const struct horizon_pinctrl_priv *info);
int horizon_pinctrl_set_state(struct udevice *dev, struct udevice *config,
			const struct horizon_pinctrl_priv *info);
int horizon_pinctrl_remove(struct udevice *dev);


#endif /* __DRIVERS_PINCTRL_HORIZON_H */
