/*
 * avb_helper.c --- avb utils
 *
 * Copyright (C) 2021, Schspa, all rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#define DEBUG
#include <command.h>
#include <cli.h>
#include <avb_verify.h>
#include <env.h>
#include <linux/string.h>
#include <asm/arch/hb_efuse.h>
#include <asm/arch/hb_board.h>

extern struct AvbOps *avb_ops;

const char *golden_partition_list[] = {
	"system",
	"app",
};

static int gpt_extend_last_part(char *bootintf, char *bootdev)
{
	char gpt_extend[512] = {0};

	snprintf(gpt_extend, sizeof(gpt_extend), "gpt extend %s %s", bootintf, bootdev);

	if (0 == (strncmp(bootintf, "mmc", sizeof("mmc")))) {
		run_command(gpt_extend, 0);
	}

	return 0;
}

static int32_t hb_check_secure(void)
{
	if (is_secure_boot() == 1)
		return 1;
	if (env_get_yesno("force_secboot") == 1)
		return 1;
	else
		return 0;
}

static int dr_get_partition_dev(char *part, char *slot_suffix, char *dev_path, int32_t sys_part_name_len)
{
	char buffer[128];
	char *part_num_env = "part_num_temp";
	char *bootintf = NULL;
	int32_t ret = 0;
	int32_t part_num = 0;
	int32_t dev_index = 0;

	bootintf = env_get("dev_name");
	dev_index = hextoul(env_get("dev_index"), NULL);

	if (strcmp(bootintf, "mtd")) {
		snprintf(buffer, sizeof(buffer), "part number %s %d %s%s %s",
			bootintf,
			dev_index,
			part,
			slot_suffix,
			part_num_env);
		ret = run_command(buffer, 0);
		if (ret) {
			snprintf(buffer, sizeof(buffer), "part number %s %d %s %s",
						bootintf,
						dev_index,
						part,
						part_num_env);
			ret = run_command(buffer, 0);

			if (ret) {
				printf("execute %s failed\n", buffer);
				do_reset(NULL, 0, 0, NULL);
			}
		}

		part_num = dectoul(env_get(part_num_env), NULL);

		snprintf(dev_path, sys_part_name_len, "/dev/mmcblk%dp%d",
				dev_index, part_num);
	} else {
		snprintf(dev_path, sys_part_name_len, "ubi0:system rootfstype=ubifs");
	}

	if (! strcmp(part, "system"))
		env_set("system_part", dev_path);

	return 0;
}

static int replace_dm_part(char *system_part)
{
	char *cmdline = NULL;
	char * new_ret = NULL;

	cmdline = env_get("bootargs");

	if (strstr(cmdline, "dm-verity") || strstr(cmdline, "dm-crypt")) {
		new_ret = avb_replace(cmdline, "$(SYSTEM_PART)", system_part);
		if (new_ret == NULL) {
			printf("replace syetem failed\n");
			return -1;
		}
		env_set("system_part", "/dev/dm-0");
	} else {
		printf("System partition verification is required during secure boot\n");
		return -1;
	}

	env_set("bootargs", new_ret);
	return 0;
}

static int32_t hb_non_secure_boot(char *bootintf, char *bootdev, char *slot_suffix)
{
	int32_t ret = 0;
	char buffer[128] = {0};
	char partition[20] = {0};
	char system_part[64] = {0};
	uint64_t part_size = 0;
	size_t read_size = 0;
	void *kernel_addr = NULL;
	void *board_id = NULL;

	snprintf(buffer, sizeof(buffer), "avb init %s %s", bootintf, bootdev);
	ret = run_command(buffer, 0);
	if (ret) {
		panic("run avb init failed\n");
	}
	kernel_addr = (void *)env_get_ulong("kernel_addr", 16, 0);
	if (kernel_addr == 0) {
		printf("env kernel_addr is not set\n");
		do_reset(NULL, 0, 0, NULL);
	}
	snprintf(partition, sizeof(partition), "boot%s", slot_suffix);
	ret = avb_ops->get_size_of_partition(avb_ops, partition, &part_size);
	if (ret == AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION) {
		/* no boot_x partition , fallback to boot */
		snprintf(partition, sizeof(partition), "boot");
		debug("Try loading the kernel from the \'boot\' partition\n");
		ret = avb_ops->get_size_of_partition(avb_ops, partition, &part_size);
		if (ret) {
			/* shouldn't going to here. */
			printf("Can't get size of partition %s\n", partition);
			do_reset(NULL, 0, 0, NULL);
		}
	}
	ret = avb_ops->read_from_partition(avb_ops, partition, 0, part_size, kernel_addr, &read_size);
	if (ret) {
		printf("Can't read from boot partition, ret=%d\n", ret);
		do_reset(NULL, 0, 0, NULL);
	}
	dr_get_partition_dev("system", slot_suffix, system_part, sizeof(system_part));
	env_set("system_part", system_part);
	board_bootargs_setup();
	memset(buffer, 0, sizeof(buffer));
	board_id = (void *)env_get("hb_board_id");
	if (board_id == NULL) {
		snprintf(buffer, sizeof(buffer), "bootm ${kernel_addr}");
	} else {
		snprintf(buffer, sizeof(buffer), "bootm ${kernel_addr}#boardid-${hb_board_id}");
	}
	ret = run_command(buffer, 0);	if (ret) {
		printf("Boot Failed with status %d\n", ret);
		run_command("run $enter_safety_cmd", 0);
	}
	return 0;
}

static int32_t hb_avb_verify_boot(char *bootintf, char *bootdev, char *slot_suffix)
{
	char buffer[128] = {0};
	int32_t ret = 0;
	char partition[20] = {0};
	uint64_t part_size = 0;
	char ab_corrupt_cmd[128] = {0};
	bool out_is_unlocked = 0;
	char system_part[64] = {0};
	void *kernel_addr = NULL;
	void *board_id = NULL;
	bool ab_exist = true;

	snprintf(ab_corrupt_cmd, sizeof(ab_corrupt_cmd),
			"ab_corrupt $bootslot mtd %s#misc", bootdev);
	snprintf(buffer, sizeof(buffer), "avb init %s %s", bootintf, bootdev);
	ret = run_command(buffer, 0);
	if (ret) {
		panic("run avb init failed\n");
	}

	ret = avb_ops->read_is_device_unlocked(avb_ops, &out_is_unlocked);
	if (ret != AVB_IO_RESULT_OK) {
		panic("get device status failed\n");
	}
	/* TODO: get unlocked status from optee */
	out_is_unlocked = 0;
	snprintf(partition, sizeof(partition), "boot%s", slot_suffix);
	ret = avb_ops->get_size_of_partition(avb_ops, partition, &part_size);
	if (ret == AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION) {
		/* no boot_x partition , fallback to boot */
		ab_exist = false;
		snprintf(partition, sizeof(partition), "boot");
		ret = avb_ops->get_size_of_partition(avb_ops, partition, &part_size);
		if (ret) {
			/* shouldn't going to here. */
			panic("Can't find boot partition\n");
		}
	}
	if (ab_exist) {
		snprintf(buffer, sizeof(buffer), "avb verify %s", slot_suffix);
	} else {
		snprintf(buffer, sizeof(buffer), "avb verify");
	}
	ret = run_command(buffer, 0);
	if (ret && (!out_is_unlocked)) {
		printf("avb verify failed with status %d\n", ret);
		ret = run_command(ab_corrupt_cmd, 0);
		do_reset(NULL, 0, 0, NULL);
	}

	dr_get_partition_dev("system", slot_suffix, system_part, sizeof(system_part));
	ret = replace_dm_part(system_part);
	if (ret && (!out_is_unlocked)) {
		do_reset(NULL, 0, 0, NULL);
	}

	board_bootargs_setup();
	kernel_addr = (void *)env_get_ulong("kernel_addr", 16, 0);
	memset(buffer, 0, sizeof(buffer));
	board_id = (void *)env_get("hb_board_id");
	if (board_id == NULL) {
		snprintf(buffer, sizeof(buffer), "bootm ${kernel_addr}");
	} else {
		snprintf(buffer, sizeof(buffer), "bootm ${kernel_addr}#boardid-${hb_board_id}");
	}
	ret = run_command(buffer, 0);
	if (ret) {
		printf("Boot Failed with status %d\n", ret);
		run_command("run $enter_safety_cmd", 0);
	}

	return 0;
}

static int do_hb_avb_verify_boot(struct cmd_tbl *cmdtp, int flag,
			int argc, char * const argv[])
{
	char *slot_suffix = "";
	char *bootdev = NULL;
	char *bootintf = NULL;

	if (argc != 4) {
		return CMD_RET_USAGE;
	}
	bootintf = argv[1];
	bootdev = argv[2];
	slot_suffix = argv[3];
	debug("bootinterface: %s, bootdev:%s, slot_suffix = %s\n", bootintf, bootdev, slot_suffix);

	/* on first startup, gpt extends the last partition */
	gpt_extend_last_part(bootintf, bootdev);
	if (hb_check_secure()) {
		debug("start to verify boot\n");
		hb_avb_verify_boot(bootintf, bootdev, slot_suffix);

	} else {
		debug("skip verify boot partition\n");
		hb_non_secure_boot(bootintf, bootdev, slot_suffix);
	}
    return 0;
}

U_BOOT_CMD(hb_avb_helper, 4, 0, do_hb_avb_verify_boot,
	"Do verify according partition type", "hb_avb_helper <slotsuffix>");
