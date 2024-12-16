/*
 *   Copyright 2024 D-Robotics, Inc.
 */
#include <common.h>
#include <command.h>
#include <init.h>
#include <fdt_support.h>
#include <asm/sections.h>
#include <linux/sizes.h>
#include <exports.h>
#include <env.h>
#include <lmb.h>
#include <search.h>
#include <dm/ofnode.h>
#include <rand.h>
#include <asm/io.h>
#include <asm/arch/hb_efuse.h>
#include <asm/arch/hb_strappin.h>
#include <asm/arch/hb_aon.h>
#include <stdio.h>
#include <stdlib.h>
#include <sata.h>
#include <stdarg.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/string.h>
#include <malloc.h>
#include <mapmem.h>
#include <asm/io.h>
#include <hb_utils.h>

#include "x5_set_pin.h"

extern int dtoverlay_debug_enabled;

static int x5_pin_get_from_40pin_id(int pin_40pin)
{
	int i;
	for (i = 0; i < PIN_MAX_NUMS; i++) {
		if (pin_info_array[i].pin_index_40pin == pin_40pin)
			return i;
	}
	return -1;
}

static void x5_set_pin_func(int pin, unsigned int val)
{
	uint32_t reg = 0;
	uint32_t mask = 0x3;

	reg = readl((void *)pin_info_array[pin].pin_iomux_base);
	val &= mask;
	reg = (reg & ~(mask << pin_info_array[pin].pin_iomux_offset)) | (val << pin_info_array[pin].pin_iomux_offset);
	writel(reg, (void *)pin_info_array[pin].pin_iomux_base);
}

static void x5_set_pin_dir(int pin, int dir)
{
	uint32_t reg = 0;

	if (dir) {
		/* set pin to ouput */
		reg = readl((void *)pin_info_array[pin].pin_gpio_base + gpio_dir);
		reg |= (1 << pin_info_array[pin].pin_gpio_number);
		writel(reg, (void *)pin_info_array[pin].pin_gpio_base + gpio_dir);
	} else {
		/* set pin to input */
		reg = readl((void *)pin_info_array[pin].pin_gpio_base + gpio_dir);
		reg &= ~(1 << pin_info_array[pin].pin_gpio_number);
		writel(reg, (void *)pin_info_array[pin].pin_gpio_base + gpio_dir);
	}
}

static void x5_set_pin_value(int pin, unsigned int val)
{
	uint32_t reg = 0;

	/* set pin output value*/
	reg = readl((void *)pin_info_array[pin].pin_gpio_base + gpio_data);
	if (val == 1) {
		reg |= (1 << pin_info_array[pin].pin_gpio_number);
	} else {
		reg &= ~(1 << pin_info_array[pin].pin_gpio_number);
	}

	writel(reg, (void *)pin_info_array[pin].pin_gpio_base + gpio_data);
}

static void x5_set_gpio_pull(int pin, enum pull_state state)
{
	uint32_t reg = 0;

	reg = readl((void *)pin_info_array[pin].pin_ctrl_base);

	switch (state) {
		case NO_PULL:
			if (pin_info_array[pin].pin_pd_offset == -1)
			{
				reg &= ~(1 << pin_info_array[pin].pin_pe_offset);

			}else
			{
				reg &= ~(1 << pin_info_array[pin].pin_pd_offset);
				reg &= ~(1 << pin_info_array[pin].pin_pu_offset);
			}
			break;
		case PULL_UP:
			if (pin_info_array[pin].pin_pd_offset == -1)
			{
				reg |= (1 << pin_info_array[pin].pin_pe_offset);
				reg |= (1 << pin_info_array[pin].pin_ps_offset);

			}else
			{
				reg &= ~(1 << pin_info_array[pin].pin_pd_offset);
				reg |= (1 << pin_info_array[pin].pin_pu_offset);
			}
			break;
		case PULL_DOWN:
			if (pin_info_array[pin].pin_pd_offset == -1)
			{
				reg |= (1 << pin_info_array[pin].pin_pe_offset);
				reg &= ~(1 << pin_info_array[pin].pin_ps_offset);

			}else
			{
				reg |= (1 << pin_info_array[pin].pin_pd_offset);
				reg &= ~(1 << pin_info_array[pin].pin_pu_offset);
			}
			break;
		default:
			break;
	}

	writel(reg, (void *)pin_info_array[pin].pin_ctrl_base);
}

static void x5_set_gpio_schmit(int pin, int val)
{
	uint32_t reg = 0;

	reg = readl((void *)pin_info_array[pin].pin_ctrl_base);
	if (val == 1) {
		reg |= (1 << pin_info_array[pin].pin_schmit_offset);
	} else {
		reg &= ~(1 << pin_info_array[pin].pin_schmit_offset);
	}
	writel(reg, (void *)pin_info_array[pin].pin_ctrl_base);
}

static void x5_set_gpio_drvstrength(int pin, int val)
{

	uint32_t reg = 0;
	uint32_t mask = 0xf;

	reg = readl((void *)pin_info_array[pin].pin_ctrl_base);
	val &= mask;
	reg = (reg & ~(mask << pin_info_array[pin].pin_drvstrength_offset)) | (val << pin_info_array[pin].pin_drvstrength_offset);
	writel(reg, (void *)pin_info_array[pin].pin_ctrl_base);
}

void dump_pin_info(int pin_40pin)
{
	int pin;
	pin = x5_pin_get_from_40pin_id(pin_40pin);

	if (pin > PIN_MAX_NUMS && pin < 0)
	{
		printf("pin %d wrong\n", pin);
		return;
	}

	printf("pin_index_40pin %d\n", pin_info_array[pin].pin_index_40pin);
	printf("pin_index_bcm %d\n", pin_info_array[pin].pin_index_bcm);
	printf("pin_iomux_base 0x%llx\n", pin_info_array[pin].pin_iomux_base);
	printf("pin_iomux_offset %d\n", pin_info_array[pin].pin_iomux_offset);
	printf("pin_ctrl_base 0x%llx\n", pin_info_array[pin].pin_ctrl_base);
	printf("[offset]: schmit %d drvstrength %d pd %d pu %d pe %d ps %d strong_pu %d\n", 
	pin_info_array[pin].pin_schmit_offset,
	pin_info_array[pin].pin_drvstrength_offset,
	pin_info_array[pin].pin_pd_offset,
	pin_info_array[pin].pin_pu_offset,
	pin_info_array[pin].pin_pe_offset,
	pin_info_array[pin].pin_ps_offset,
	pin_info_array[pin].pin_strong_pu_offset);
	printf("pin_gpio_base 0x%llx\n", pin_info_array[pin].pin_gpio_base);
	printf("pin_gpio_number %d\n", pin_info_array[pin].pin_gpio_number);
	printf("func [%s] [%s] [%s] [%s]\n", pin_info_array[pin].func_name0,pin_info_array[pin].func_name1,pin_info_array[pin].func_name2,pin_info_array[pin].func_name3);
	printf("pin_ctrl_base 0x%08x\n", readl((void *)pin_info_array[pin].pin_iomux_base));
	printf("pin_ctrl_base 0x%08x\n", readl((void *)pin_info_array[pin].pin_ctrl_base));
	printf("pin_gpio_data_base 0x%08x\n", readl((void *)pin_info_array[pin].pin_gpio_base + gpio_data));
	printf("pin_gpio__dir_base 0x%08x\n", readl((void *)pin_info_array[pin].pin_gpio_base + gpio_dir));
}

void x5_set_pin(int pin_40pin, char *cmd)
{
	int pin;
	pin = x5_pin_get_from_40pin_id(pin_40pin);

	if (strcmp(cmd, "f0") == 0) {
		x5_set_pin_func(pin, 0);
	} else if (strcmp(cmd, "f1") == 0) {
		x5_set_pin_func(pin, 1);
	} else if (strcmp(cmd, "f2") == 0) {
		x5_set_pin_func(pin, 2);
	} else if (strcmp(cmd, "f3") == 0) {
		x5_set_pin_func(pin, 3);
	} else if (strcmp(cmd, "ip") == 0) {
		x5_set_pin_dir(pin, 0);
	} else if (strcmp(cmd, "op") == 0) {
		x5_set_pin_dir(pin, 1);
	} else if (strcmp(cmd, "dh") == 0) {
		x5_set_pin_value(pin, 1);
	} else if (strcmp(cmd, "dl") == 0) {
		x5_set_pin_value(pin, 0);
	} else if (strcmp(cmd, "pu") == 0) {
		x5_set_gpio_pull(pin, PULL_UP);
	} else if (strcmp(cmd, "pd") == 0) {
		x5_set_gpio_pull(pin, PULL_DOWN);
	} else if (strcmp(cmd, "pn") == 0 || strcmp(cmd, "np") == 0) {
		x5_set_gpio_pull(pin, NO_PULL);
	} else if (strcmp(cmd, "se") == 0) {
		x5_set_gpio_schmit(pin, 1);
	} else if (strcmp(cmd, "sd") == 0) {
		x5_set_gpio_schmit(pin, 0);
	} if (strncmp(cmd, "ds", 2) == 0) { 
    	int strength = atoi(cmd + 2); 
		if (strength >= 0 && strength <= 15) {
			x5_set_gpio_drvstrength(pin, strength);
		} else {
			printf("x5_set_pin Invalid drive strength value: %s\n", cmd);
		}
	}	
}

int x5_set_pin_by_config(char *cfg_file, ulong file_addr)
{
	char *data, *dp, *lp, *ptr;
	loff_t size = 0;
	char line[128], filter_name[32];
	int length;
	int pin, start_pin, end_pin;
	int som_type = 0;
	char devname[32];

	// read gpio info from /boot/config.txt
	ptr = map_sysmem(file_addr, 0);
	if(hb_ext4_load(cfg_file, file_addr, &size) != 0)
	{
		printf("can't load config file(%s)\n", cfg_file);
		return 0;
	}
	if ((data = malloc(size + 1)) == NULL)
	{
		printf("can't malloc %lu bytes\n", (ulong)size + 1);
		return 0;
	}

	memcpy(data, ptr, size);
	data[size] = '\0';
	dp = data;

	memset(devname, 0, sizeof(devname));

	// parse gpio info
	while ((length = hb_getline(line, sizeof(line), &dp)) > 0) {

		if (length == -1) {
			break;
		}

		lp = line;

		/* skip leading white space */
		while (isblank(*lp))
			++lp;

		/* skip comment lines */
		if (*lp == '#' || *lp == '\0')
			continue;

		/* Skip items that do not match the filter */
        if (som_type != 0 && strncmp(lp, "[", 1) != 0 )
            continue;

		if (strncmp(lp, "gpio=", 5) == 0)
		{
			lp += strlen("gpio=");
			char *str = strdup(lp);
			char *token;

			// Split by '=' to get key and value
			token = strtok(str, "=");
			if (token) {
				char *key = token;
				char *value = strtok(NULL, "=");

				char *dash = strchr(key, '-');
				if (dash) {
					// Handle ranges
					start_pin = atoi(key);
					end_pin = atoi(dash + 1);
				}
				else {
					start_pin = end_pin = atoi(key);
				}

				if(dtoverlay_debug_enabled != 0)
				{
					printf("After gpio init\n");
					for (pin = start_pin; pin <= end_pin; pin++) {
						dump_pin_info(pin);
					}
				}

				char *rangeToken = strtok(value, ",");
				while (rangeToken) {
					// set pin
					for (pin = start_pin; pin <= end_pin; pin++) {
						x5_set_pin(pin, strim(rangeToken));
					}
					rangeToken = strtok(NULL, ",");
				}

				if(dtoverlay_debug_enabled != 0)
				{
					printf("Before gpio init\n");
					for (pin = start_pin; pin <= end_pin; pin++) {
						dump_pin_info(pin);
					}
				}

			}

			free(str);
		}
		else if (strncmp(lp, "[", 1) == 0)
        {

            hb_extract_filter_name(lp, filter_name);
            hb_dev_name_get(devname);
            if (strcmp("all", filter_name) == 0 || strcmp("devname", filter_name) == 0)
                som_type = 0;
            else
                som_type = 1;
			printf("%s %s %d filter_name %s devname %s som_type %d\n",__FILE__, __FUNCTION__, __LINE__,filter_name,devname,som_type);
        }
	}

	free(data);
	return 0;
}

static int do_set_pin_by_config(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	char *cfg_file;
	ulong file_addr;

	if (argc < 2)
		return CMD_RET_USAGE;

	cfg_file = argv[1];

	file_addr = simple_strtoul(argv[2], NULL, 16);

	return x5_set_pin_by_config(cfg_file, file_addr);
}

static char set_pin_help_text[] =
	"setpin <config file> <config addr>  - set pin stat by /boot/config.txt\n";

U_BOOT_CMD(
	setpin, 255, 0, do_set_pin_by_config,
	"set pin by config", set_pin_help_text);

