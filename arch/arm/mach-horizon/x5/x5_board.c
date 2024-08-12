// SPDX-License-Identifier: GPL-2.0+
/*
 * Common board-specific code
 *
 * Copyright (c) 2023 Horizon Robotics Co., Ltd
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
#include <asm/arch/hb_strappin.h>
#include <asm/arch/hb_aon.h>
#include <hb_info.h>
#include <wdt.h>

#ifdef CONFIG_LAST_STAGE_INIT

#define RECOVERY_MODE 0x010000

enum dr_reset_reason {
	COLD_BOOT = 0,
	WATCHDOG,
	REBOOT_CMD,
	PANIC,
	UBOOT_RESET,
	UNKNOWN,
};

static char *dr_reason[16] = {"COLD_BOOT", "WATCHDOG", "REBOOT_CMD",
                            "PANIC", "UBOOT_RESET", "UNKNOWN"};

static int dr_get_reset_reason(void)
{
	static int32_t value = 0;
	int32_t update_reason = 0;

	value = readl(AON_STATUS_REG1);
	update_reason = value;
	value = AON_RESET_REASON_VALUE(value);
	update_reason &= ~(AON_RESET_REASON_MASK << AON_RESET_REASON_OFFSET);
	update_reason |= AON_RESET_REASON_WATCHDOG;
	writel(update_reason, AON_STATUS_REG1);
	env_set("reset_reason", dr_reason[value]);

	return value;
}

int chip_last_stage_init(void)
{
	int32_t boot_action = 0;
	int32_t clear_mode = 0;
	char * const default_envs[] = {
		ENV_AB_SELECT_CMD,
		ENV_AVB_VERIFY_BOOT_CMD,
		ENV_ENTER_SAFETY_CMD,
	};
	env_set_default_vars(ARRAY_SIZE(default_envs), default_envs, H_FORCE);
	boot_action = readl(AON_STATUS_REG1);

	clear_mode =  boot_action & (~(AON_BOOT_ACTION_MASK << AON_BOOT_ACTION_OFFSET));

	boot_action = AON_BOOT_ACTION_VALUE(boot_action);
	switch (boot_action) {
		case BOOT_DEVICE_UART:
			env_set("bootdelay", "-1");
			printf("boot action: UART\n");
		break;
		case BOOT_DEVICE_USB3:
			printf("boot action: FASTBOOT USB3.0\n");
			env_set("preboot", "fastboot 1");
		break;
		case BOOT_DEVICE_USB2:
			printf("boot action: FASTBOOT USB2.0\n");
			env_set("preboot", "fastboot 0");
		break;
		case BOOT_RECOVERY:
			printf("boot action: entry recovery mode\n");
			env_set("recovery_mode", "yes");
		break;
	}
	writel(clear_mode, AON_STATUS_REG1);
	return 0;
}

static void set_panic_action(void)
{
	int32_t panic_action = 0;
	int32_t clear_mode = 0;
	int32_t reset_reason_v = 0;

	reset_reason_v = dr_get_reset_reason();
	if (reset_reason_v != PANIC)
		return;

	panic_action = readl(AON_STATUS_REG1);
	clear_mode =  panic_action & (~(AON_PANIC_ACTION_MASK << AON_PANIC_ACTION_OFFSET));
	panic_action = AON_PANIC_ACTION_VALUE(panic_action);

	switch (panic_action) {
		case PANIC_UBOOT_ONCE:
			printf("panic action: ubootonce\n");
			env_set("bootdelay", "-1");
			break;
		case PANIC_UDUMP_FASTBOOT:
			printf("panic action: enter fastboot ramdump mode\n");
			printf("please use \"fastboot oem ramdump\" command in pc\n");
			env_set("preboot", "fastboot 0");
			break;
	}
	writel(clear_mode, AON_STATUS_REG1);
}


static void set_bootdev(void)
{
	int boot_dev = 0;
	char *dev_str = "mmc";
	char *index_str = "0";
	char *boot_mode ="emmc";

	boot_dev = readl(BOOT_STRAP_PIN_REG + BOOT_MODE_SHIFT) & BOOT_MODE_MASK;
	switch (boot_dev) {
		case PLAT_HORIZON_BOOT_SRC_EMMC:
			dev_str = "mmc";
			index_str = "0";
			boot_mode = "emmc";
			break;
		case PLAT_HORIZON_BOOT_SRC_SD:
			dev_str = "mmc";
			index_str = "1";
			boot_mode = "sd";
			break;
		case PLAT_HORIZON_BOOT_SRC_QSPI_NOR:
			dev_str = "mtd";
			index_str = "0";
			boot_mode = "nor";
			break;
		case PLAT_HORIZON_BOOT_SRC_QSPI_NAND:
			dev_str = "mtd";
			index_str = "0";
			boot_mode = "nand";
			break;
		default:
			break;
	}
	env_set("dev_name", dev_str);
	env_set("dev_index", index_str);
	env_set("boot_device", boot_mode);
}

static char *hb_bootmedium_for_udev(void)
{
	char *boot_mode = env_get("boot_device");
	if (boot_mode == NULL) {
		return "MMC";
	} else if (strncmp(boot_mode, "emmc", strlen(boot_mode) + 1) == 0) {
		return "MMC";
	} else if(strncmp(boot_mode, "sd", strlen(boot_mode) + 1) == 0) {
		return "SD";
	} else if(strncmp(boot_mode, "nand", strlen(boot_mode) + 1) == 0) {
		return "nand";
	} else if(strncmp(boot_mode, "nor", strlen(boot_mode) + 1) == 0) {
		return "nor";
	} else {
		printf("boot device is invalid, set udev boot_mode to MMC");
		return "MMC";
	}
}

static void board_env_setup(void)
{
	u32 board_id;
	char hex_board_id[9];
	char *recovery_mode = env_get("recovery_mode");

	env_set("bootcmd",
		"run ab_select_cmd;"
		"run avb_boot;");
	set_bootdev();

	hb_board_id_get(&board_id);

	if ((recovery_mode == NULL) ||
		(strncmp(recovery_mode, "yes", strlen(recovery_mode) + 1) != 0)) {
		snprintf(hex_board_id, sizeof(hex_board_id), "0x%04X", board_id);
	} else {
		board_id |= RECOVERY_MODE;
		snprintf(hex_board_id, sizeof(hex_board_id), "0x%06X", board_id);
	}

	env_set("hb_board_id", hex_board_id);
}

int last_stage_init(void)
{
	chip_last_stage_init();
	set_panic_action();
	board_env_setup();
	return 0;
}
#endif

#if defined(CONFIG_BOARD_EARLY_INIT_R)
int board_early_init_r(void)
{
#ifdef CONFIG_DROBOT_DISABLE_WDT
	printf("stop watchdog\n");
	wdt_stop_all();
#endif
	return 0;
}
#endif

static char *hb_bootmode(void)
{
	char *recovery_mode = env_get("recovery_mode");

	if ((recovery_mode == NULL) ||
		(strncmp(recovery_mode, "yes", strlen(recovery_mode) + 1) != 0)) {
		return "normal";
	} else {
		return "recovery";
	}
}

static void hb_get_rootfs(void)
{
	char rootfs_args[256] = {0};
	char *recovery_mode = env_get("recovery_mode");

	if ((recovery_mode == NULL) ||
		(strncmp(recovery_mode, "yes", strlen(recovery_mode) + 1) != 0)) {
		snprintf(rootfs_args, sizeof(rootfs_args), "root=%s ro rootwait",
				 env_get("system_part"));
	} else {
		snprintf(rootfs_args, sizeof(rootfs_args),
				"root=/dev/ram0 rdinit=/init rw rootwait");
	}

	env_set("rootfs_args", rootfs_args);
}

uint32_t hb_get_uart_baud(void)
{
	int baud_stap = 0;
	baud_stap = (readl(BOOT_STRAP_PIN_REG) & BOOT_UART_BPS_MASK) >> BOOT_UART_BPS_SHIFT;
	if (baud_stap)
		return 921600;
    return 115200;
}

void board_bootargs_setup(void)
{
	int ret;
	char boot_args[2048] = { 0 }, slot_suffix[3] = { '_', 'a', '\0'};
	char console_args[64] = { 0 };
	char mtd_args[512] = {0};
	char *cmdline = env_get("bootargs");
	uint32_t uart_baud = hb_get_uart_baud();
	uint32_t len;
	char *console, *slot;
	char *dev_name = env_get("dev_name");

	slot = env_get("bootslot");
	if (slot == NULL || strlen(slot) > 1) {
		pr_warn("skip set hobotboot.slot_suffix in bootargs\n");
	} else {
		slot_suffix[1] = slot[0];
	}

	console = env_get("console");
	if (console == NULL || (strncmp(console, "tty", 3) != 0 &&
		strcmp(console, "null") != 0)) {
		snprintf(console_args, sizeof(console_args),
			"ttyS0,%dn8", uart_baud);
	} else {
		if (strchr(console, ',')) {
			len = strchr(console, ',') - console;
			strncpy(console_args, console, len);
			snprintf(console_args+len, sizeof(console_args)-len,
				",%dn8", uart_baud);
		} else {
			snprintf(console_args, sizeof(console_args),
				"%s,%dn8", console, uart_baud);
		}
	}
	hb_get_rootfs();
	snprintf(boot_args, sizeof(boot_args),
		"console=%s "
		"%s "
		"hobotboot.slot_suffix=%s "
		"hobotboot.reason=%s "
		"hobotboot.medium=%s "
		"hobotboot.mode=%s "
		"pmic_type=%s "
		" %s"
		" %s",
		console_args,
		env_get("rootfs_args"),
		slot_suffix,
		env_get("reset_reason"),
		hb_bootmedium_for_udev(),
		hb_bootmode(),
		hb_pmic_type_get(),
		X5_DEFAULT_BOOTARGS,
		cmdline ? cmdline : "");

	if (! strncmp(dev_name, "mtd", 3)) {
		snprintf(mtd_args, sizeof(mtd_args),
			" ubi.mtd=system "
			"mtdparts=%s",
			env_get("mtdparts"));
		strncat(boot_args, mtd_args, strlen(mtd_args));
	}

	debug("os cmdline: %s\n", boot_args);

	ret = env_set("bootargs", boot_args);
	if (ret) {
		pr_err("%s setup bootargs environment failed, ret:%d\n",
		       __func__, ret);
	}
}

void board_lmb_reserve(struct lmb *lmb)
{
	/* add this region for dtb */
	lmb_add(lmb, X5_DTB_LMB_START, X5_DTB_LMB_SIZE);
	//lmb_dump_all_force(lmb);
}
