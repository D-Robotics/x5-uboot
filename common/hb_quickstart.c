// SPDX-License-Identifier: GPL-2.0+
/*
 * CONFIG_QUICKSTART — hb_quickstart.c
 * Copyright(C) 2026, D-Robotics Co., Ltd.
 *
 * Sections (top to bottom):
 *   1) Linear env overrides (env_get / env_set / env_print)
 *   2) Minimal ft_board_setup when OF_BOARD_SETUP
 *   3) Board hook forward declarations
 *   4) GPT/partition helpers and hb_quickstart_boot()
 *   5) hb_quickstart_init_sequence_r[] and hb_quickstart_board_init_r()
 */

#include <common.h>
#include <asm/global_data.h>

DECLARE_GLOBAL_DATA_PTR;

#include <command.h>
#include <env.h>
#include <serial.h>
#include <cli.h>
#include <init.h>
#include <hang.h>
#include <cpu_func.h>
#include <relocate.h>
#include <initcall.h>
#include <android_bootloader_message.h>
#include <android_ab.h>
#include <div64.h>
#include <event.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/libfdt.h>
#include <part.h>
#include <asm/arch/hb_aon.h>
#include <asm/arch-x5/hb_efuse.h>
#include <env_internal.h>
#include <stdio.h>
#include <string.h>
#include <hb_quickstart.h>

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
#include <fdt_support.h>
#endif
int env_print(char *name, int flag)
{
	char *p = hb_quickstart_env_data;

	(void)name;
	(void)flag;

	while (*p) {
		printf("  %s\n", p);
		p += strlen(p) + 1;
	}
	return 0;
}

int env_set(const char *name, const char *value)
{
	char *pos;
	int old_len;

	if (!name || !*name)
		return -1;
	if (strchr(name, '='))
		return -1;

	pos = hb_quickstart_env_data;
	while (*pos) {
		if (!strncmp(pos, name, strlen(name)) && pos[strlen(name)] == '=') {
			old_len = strlen(pos) + 1;
			memmove(pos, pos + old_len,
				ENV_SIZE - (pos - hb_quickstart_env_data) - old_len);
			break;
		}
		pos += strlen(pos) + 1;
	}

	if (!value || !*value)
		return 0;

	if (strlen(hb_quickstart_env_data) + strlen(name) + 1 + strlen(value) + 1 >=
	    ENV_SIZE)
		return -1;

	sprintf(pos, "%s=%s", name, value);
	return 0;
}

char *env_get(const char *name)
{
	char *pos;

	if (!name || !*name)
		return NULL;

	pos = hb_quickstart_env_data;
	while (*pos) {
		if (!strncmp(pos, name, strlen(name)) && pos[strlen(name)] == '=')
			return pos + strlen(name) + 1;
		pos += strlen(pos) + 1;
	}

	return NULL;
}

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
int ft_board_setup(void *blob, struct bd_info *bd)
{
	(void)bd;

#ifdef CONFIG_CMD_SEND_ID
	hb_fdt_set_board_info(blob);
#endif
	hb_setup_ion_size(blob);
	fdt_rm_by_env(blob);
	return 0;
}
#endif

struct quickstart_part_scan {
	bool has_userdata;
	bool has_system_slot;
	bool has_system;
	bool has_boot_slot;
	bool has_boot;
	bool need_extend;
	/* GPT index for root= from the single part_get_info scan (1..N). */
	int partnum_system_slot;
	int partnum_system;
	struct disk_partition system_slot;
	struct disk_partition system;
	struct disk_partition boot_slot;
	struct disk_partition boot;
};

static void quickstart_scan_partitions(struct blk_desc *desc,
		const char *slot_suffix,struct quickstart_part_scan *scan)
{
	struct disk_partition info;
	char system_slot_name[20];
	char boot_slot_name[20];
	lbaint_t max_end = 0;
	lbaint_t target_end;
	lbaint_t end;
	int i;

	memset(scan, 0, sizeof(*scan));
	scan->partnum_system_slot = -1;
	scan->partnum_system = -1;
	if (!desc || !desc->lba)
		return;

	snprintf(system_slot_name, sizeof(system_slot_name), "system%s",
		 slot_suffix);
	snprintf(boot_slot_name, sizeof(boot_slot_name), "boot%s",
		 slot_suffix);

	for (i = 1; i <= MAX_SEARCH_PARTITIONS; i++) {
		if (part_get_info(desc, i, &info))
			continue;

		if (!strcmp((const char *)info.name, "userdata")) {
			scan->has_userdata = true;
			target_end = desc->lba - 33;
			if (info.start + info.size < target_end)
				scan->need_extend = true;
		} else if (!strcmp((const char *)info.name, system_slot_name)) {
			scan->has_system_slot = true;
			memcpy(&scan->system_slot, &info, sizeof(info));
			scan->partnum_system_slot = i;
		} else if (!strcmp((const char *)info.name, "system")) {
			scan->has_system = true;
			memcpy(&scan->system, &info, sizeof(info));
			scan->partnum_system = i;
		} else if (!strcmp((const char *)info.name, boot_slot_name)) {
			scan->has_boot_slot = true;
			memcpy(&scan->boot_slot, &info, sizeof(info));
		} else if (!strcmp((const char *)info.name, "boot")) {
			scan->has_boot = true;
			memcpy(&scan->boot, &info, sizeof(info));
		}

		end = info.start + info.size;
		if (end > max_end)
			max_end = end;
	}

	if (!scan->has_userdata && max_end)
		scan->need_extend = max_end < (desc->lba - 33);
}

static bool quickstart_need_gpt_extend(const struct quickstart_part_scan *scan)
{
	return scan && scan->need_extend;
}

static int quickstart_select_system_part(struct blk_desc *desc,
		const char *slot_suffix,
		struct quickstart_part_scan *scan,
		char *system_part,
		size_t system_part_size)
{
	int part_num = -1;

	(void)desc;

	if (scan->has_system_slot)
		part_num = scan->partnum_system_slot;
	else if (scan->has_system)
		part_num = scan->partnum_system;

	if (part_num < 1) {
		printf("quickstart: no GPT partition system%s or system\n",
		       slot_suffix);
		return -ENOENT;
	}
	snprintf(system_part, system_part_size, "/dev/mmcblk0p%d", part_num);
	return 0;
}

static int quickstart_select_boot_part(struct blk_desc *desc,
		const char *slot_suffix,
		const struct quickstart_part_scan *scan,
		char *partition,
		size_t partition_size,
		struct disk_partition *boot_part_info)
{
	(void)desc;

	if (scan->has_boot_slot) {
		snprintf(partition, partition_size, "boot%s", slot_suffix);
		memcpy(boot_part_info, &scan->boot_slot, sizeof(*boot_part_info));
		return 0;
	}
	if (scan->has_boot) {
		snprintf(partition, partition_size, "boot");
		memcpy(boot_part_info, &scan->boot, sizeof(*boot_part_info));
		return 0;
	}
	printf("quickstart: no boot partition boot%s/boot\n",
	       slot_suffix);
	return -ENOENT;
}

int hb_quickstart_boot(void)
{
	int32_t ret;
	int32_t boot_action = 0;
	int32_t aon_status = 0;
	int32_t clear_mode = 0;
	u32 efuse_buf[4] = {0};
	char hex_socuid[32];
	char partition[20] = {0};
	uint64_t selected_part_size = 0;
	char slot_suffix[3] = { '_', 'a', '\0'};
	int bootdev = 0;
	char *bootintf = NULL;
	char boot_args[2048] = { 0 };
	char console_args[64] = { 0 };
	char system_part[64] = {0};
	char buffer[128];
	struct blk_desc *mmc_desc;
	struct disk_partition boot_part_info;
	struct quickstart_part_scan part_scan;
	uint32_t uart_baud = hb_get_uart_baud();
	const uint32_t hb_board_id = CONFIG_QUICKSTART_BOARD_ID;
	ulong kernel_addr = CONFIG_QUICKSTART_KERNEL_ADDR;
	uint64_t part_size = 0;
	lbaint_t read_blks;
	lbaint_t head_blks;
	lbaint_t left_blks;
	ulong left_done;
	ulong blks_done;
	size_t fit_size;
	int abort_ms;

	if (!env_get("serial#") && hb_get_socuid(efuse_buf) == 0) {
		snprintf(hex_socuid, sizeof(hex_socuid), "%08x%08x%08x%08x",
			 efuse_buf[3], efuse_buf[2], efuse_buf[1], efuse_buf[0]);
		hex_socuid[31] = '\0';
		env_set("serial#", hex_socuid);
	}

	aon_status = readl(AON_STATUS_REG1);
	clear_mode = aon_status &
		     (~(AON_BOOT_ACTION_MASK << AON_BOOT_ACTION_OFFSET));
	boot_action = AON_BOOT_ACTION_VALUE(aon_status);
	switch (boot_action) {
	case BOOT_DEVICE_USB2:
		writel(clear_mode, AON_STATUS_REG1);
		run_command_list("fastboot 0", -1, 0);
		break;
	}

	slot_suffix[1] = AON_AB_SLOT_VALUE(aon_status);
	slot_suffix[1] = BOOT_SLOT_NAME(slot_suffix[1]);

	if (strncmp(hb_bootmedium_for_udev(), "MMC", sizeof("MMC")) != 0) {
		printf("Unsupported boot media, please use eMMC ...\n");
		do_reset(NULL, 0, 0, NULL);
	}
	bootintf = "mmc";
	bootdev = 0;

	for (abort_ms = CONFIG_QUICKSTART_ABORT_DELAY_MS; abort_ms-- > 0;) {
		if (tstc()) {
			if (serial_getc() == 0x20) /* Space */
				cli_loop();
		}
		udelay(1000);
	}

	snprintf(console_args, sizeof(console_args), "ttyS0,%dn8",
		 uart_baud);
	mmc_desc = blk_get_dev(bootintf, bootdev);
	if (!mmc_desc) {
		printf("quickstart: no %s device %d\n", bootintf, bootdev);
		do_reset(NULL, 0, 0, NULL);
	}
	part_init(mmc_desc);
	quickstart_scan_partitions(mmc_desc, slot_suffix, &part_scan);
	if (quickstart_need_gpt_extend(&part_scan)) {
		snprintf(buffer, sizeof(buffer), "gpt extend %s %d", bootintf,
			 bootdev);
		ret = run_command(buffer, 0);
		if (ret) {
			printf("gpt extend failed (%d)\n", ret);
		} else {
			/* GPT changed, reload cached partition type/info. */
			part_init(mmc_desc);
			quickstart_scan_partitions(mmc_desc, slot_suffix,
						   &part_scan);
		}
	}

	if (quickstart_select_system_part(mmc_desc, slot_suffix, &part_scan,
					  system_part, sizeof(system_part)))
		do_reset(NULL, 0, 0, NULL);

	if (quickstart_select_boot_part(mmc_desc, slot_suffix, &part_scan, partition,
					sizeof(partition), &boot_part_info))
		do_reset(NULL, 0, 0, NULL);

	selected_part_size = (uint64_t)boot_part_info.blksz *
			     (uint64_t)boot_part_info.size;
	part_size = selected_part_size;

	if (!boot_part_info.blksz) {
		printf("quickstart: boot partition block size 0\n");
		do_reset(NULL, 0, 0, NULL);
	}

	/*
	 * Read only FIT header first, then shrink read size to actual FIT total.
	 * This avoids always reading the whole boot partition.
	 */
	head_blks = (lbaint_t)DIV_ROUND_UP_ULL(4096, (u64)boot_part_info.blksz);
	if (head_blks > boot_part_info.size)
		head_blks = boot_part_info.size;
	blks_done = blk_dread(mmc_desc, boot_part_info.start, head_blks,(void *)kernel_addr);
	if (blks_done != (ulong)head_blks) {
		printf("quickstart: hdr read %s got %lu want %lu sectors\n",
		       partition, blks_done, (ulong)head_blks);
		do_reset(NULL, 0, 0, NULL);
	}
	if (fdt_check_header((const void *)kernel_addr)) {
		printf("quickstart: not a valid FIT/FDT at 0x%lx\n", kernel_addr);
		do_reset(NULL, 0, 0, NULL);
	}
	fit_size = fdt_totalsize((const void *)kernel_addr);
	if (fit_size) {
		/*
		 * Always read up to min(partition, fdt_totalsize), never
		 * truncate the FIT.
		 */
		if ((uint64_t)fit_size > selected_part_size) {
			printf("quickstart: FIT size %zu > partition %llu bytes\n",
			       fit_size, (unsigned long long)selected_part_size);
			do_reset(NULL, 0, 0, NULL);
		}
		part_size = (uint64_t)fit_size;
	}

	read_blks = (lbaint_t)DIV_ROUND_UP_ULL(part_size,
					       (u64)boot_part_info.blksz);
	if (read_blks > boot_part_info.size)
		read_blks = boot_part_info.size;

	if (read_blks > head_blks) {
		left_blks = read_blks - head_blks;
		left_done = blk_dread(mmc_desc, boot_part_info.start + head_blks,
						left_blks,
						(void *)(kernel_addr +
						head_blks * boot_part_info.blksz));
		if (left_done != (ulong)left_blks) {
			printf("quickstart: blk_dread %s got %lu want %lu sectors\n",
			       partition, left_done, (ulong)left_blks);
			do_reset(NULL, 0, 0, NULL);
		}
	}

	snprintf(boot_args, sizeof(boot_args),
		 "console=%s "
		 "root=%s ro rootwait "
		 "hobotboot.slot_suffix=%s "
		 "hobotboot.ab_switch_reason=%s "
		 "hobotboot.medium=%s "
		 "hobotboot.mode=%s "
		 "loglevel=3 panic=1 ",
		 console_args, system_part, slot_suffix,
		 hb_get_ab_switch_reason(), hb_bootmedium_for_udev(),
		 hb_bootmode());
	env_set("bootargs", boot_args);
	/* Skip FIT hash verification in quickstart path for latency. */
	env_set("verify", "n");

	snprintf(buffer, sizeof(buffer), "bootm 0x%lx#boardid-0x%04X",
		 kernel_addr, hb_board_id);
	ret = run_command(buffer, 0);
	if (ret) {
		printf("bootm failed: %d cmd=%s\n", ret, buffer);
		do_reset(NULL, 0, 0, NULL);
	}
	return 0;
}

init_fnc_t hb_quickstart_init_sequence_r[] = {
	initr_trace,
	initr_reloc,
	event_init,
#if defined(CONFIG_ARM) || defined(CONFIG_RISCV)
	initr_caches,
#endif
	initr_reloc_global_data,
	initr_malloc,
#ifdef CONFIG_DM
	initr_dm,
#endif
#ifdef CONFIG_MMC
	initr_mmc,
#endif
	hb_quickstart_boot,
	NULL,
};

void hb_quickstart_board_init_r(gd_t *new_gd, ulong dest_addr)
{

	if (CONFIG_IS_ENABLED(X86_64) && !IS_ENABLED(CONFIG_EFI_APP))
		arch_setup_gd(new_gd);
#if !defined(CONFIG_X86) && !defined(CONFIG_ARM) && !defined(CONFIG_ARM64)
	gd = new_gd;
#endif
	gd->flags &= ~GD_FLG_LOG_READY;

	if (IS_ENABLED(CONFIG_NEEDS_MANUAL_RELOC)) {
		for (int i = 0; hb_quickstart_init_sequence_r[i]; i++)
			MANUAL_RELOC(hb_quickstart_init_sequence_r[i]);
	}
	if (initcall_run_list(hb_quickstart_init_sequence_r))
		hang();
	hang();
}
