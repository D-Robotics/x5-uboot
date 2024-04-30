// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2023 Horizon Robotics Co., Ltd
 */

#include <common.h>
#include <asm/io.h>
#include <dm/device.h>
#include <dm/ofnode.h>
#include <env.h>
#include <asm/arch/hb_aon.h>

static char hardware_name[64] = {0};
static char ethact_name[64] = {0};
static char net_eth0_ip[64] = {0};
static char board_name[64] = {0};
static char pmic_type[64] = {0};
static uint32_t  hb_board_id = 0;

char *hb_hardware_name_get(void)
{
	if (strlen(hardware_name) == 0)
		pr_err("hardware_name is empty now!\n");
	return hardware_name;
}

int32_t hb_hardware_name_set(const char *name)
{
	return  !strlcpy(hardware_name, name, sizeof(hardware_name));
}

char *hb_board_name_get(void)
{
	if (strlen(board_name) == 0)
		pr_err("board_name is empty now!\n");
	return board_name;
}

int32_t hb_board_name_set(const char *name)
{
	return !strlcpy(board_name, name, sizeof(board_name));
}

char *hb_ethact_name_get(void)
{
	if (strlen(ethact_name) == 0)
		pr_err("ethact_name is empty now!\n");
	return ethact_name;
}

int32_t hb_ethact_name_set(const char *name)
{
	return !strlcpy(ethact_name, name, sizeof(ethact_name));
}

static void hb_set_pmic_type_reg(int pmic_type)
{
	int32_t val = 0;

	val = readl(AON_STATUS_REG1);
	val &= ~(1 << AON_PMIC_TYPE_OFFSET);
	val |= (pmic_type << AON_PMIC_TYPE_OFFSET);
	writel(val, AON_STATUS_REG1);
}

int32_t hb_pmic_type_set(const char *name)
{
	if ((strncmp(name, "dual-pmic", strlen("dual-pmic") + 1)) == 0) {
		hb_set_pmic_type_reg(0);
	} else if ((strncmp(name, "single-pmic", strlen("single-pmic") + 1)) == 0) {
		hb_set_pmic_type_reg(1);
	} else {
		printf("%s pmic type is not support\n", name);
	}
	return !strlcpy(pmic_type, name, sizeof(pmic_type));
}

char *hb_pmic_type_get(void)
{
	if (strlen(pmic_type) == 0)
		pr_err("pmic type is empty now!\n");
	return pmic_type;
}

int32_t hb_board_id_get(uint32_t *board_id)
{
	if (hb_board_id == 0) {
		pr_err("board id is not set\n");
		return -1;
	}
	*board_id = hb_board_id;
	return 0;
}

int32_t hb_board_id_set(uint32_t board_id)
{
	hb_board_id = board_id;
	debug("hb_board_id=0x%x\n", hb_board_id);
	return 0;
}

char *hb_eth0_ip_get(void)
{
	if (strlen(net_eth0_ip) == 0)
		pr_err("net_eth0_ip is empty now!\n");
	return net_eth0_ip;
}

int hb_eth0_ip_set(const char *eth0_ip)
{
	return !strlcpy(net_eth0_ip, eth0_ip, sizeof(net_eth0_ip));
}

int hb_device_init(void)
{
	ofnode node;
	struct udevice *dev;
	const char *name;
	ofnode chosen_node;
	int ret;
	int len;
	int i;

	chosen_node = ofnode_path("/chosen");
	if (!ofnode_valid(chosen_node)) {
		return -EINVAL;
	}

	len = ofnode_read_string_count(chosen_node, "initial-devices");
	if (len < 0)
		return -EINVAL;

	for (i = 0; i < len; i++) {
		ret = ofnode_read_string_index(chosen_node, "initial-devices",
					i, &name);
		if (ret) {
			return -EINVAL;
		}
		node = ofnode_path(name);
		// we are here to call device's probe
		device_get_global_by_ofnode(node, &dev);
	}

	return 0;
}

