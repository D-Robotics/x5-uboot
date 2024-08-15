// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (C) 2017 The Android Open Source Project
 */
#include <android_bootloader_message.h>
#include <common.h>
#include <android_ab.h>
#include <command.h>
#include <env.h>
#include <part.h>
#include <mtd.h>
#include <linux/mtd/mtd.h>
//#include <hb_info.h>

static int do_ab_select_mtd(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[]);
static int do_ab_select(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	int ret;
	struct blk_desc *dev_desc;
	struct disk_partition part_info;
	char slot[2];

	if (argc != 4)
		return CMD_RET_USAGE;
	if (!strncmp(argv[2], "mtd", 3)) {
		return do_ab_select_mtd(cmdtp, flag, argc, argv);
	}
	/* Lookup the "misc" partition from argv[2] and argv[3] */
	if (part_get_info_by_dev_and_name_or_num(argv[2], argv[3],
						 &dev_desc, &part_info,
						 false) < 0) {
		return CMD_RET_FAILURE;
	}

	ret = ab_select_slot(dev_desc, &part_info);
	if (ret < 0) {
		printf("Android boot failed, error %d.\n", ret);
		return CMD_RET_FAILURE;
	}

	/* Android standard slot names are 'a', 'b', ... */
	slot[0] = BOOT_SLOT_NAME(ret);
	slot[1] = '\0';
	env_set(argv[1], slot);
	printf("ANDROID: Booting slot: %s\n", slot);
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(ab_select, 4, 0, do_ab_select,
	   "Select the slot used to boot from and register the boot attempt.",
	   "<slot_var_name> <interface> <dev[:part|#part_name]>\n"
	   "    - Load the slot metadata from the partition 'part' on\n"
	   "      device type 'interface' instance 'dev' and store the active\n"
	   "      slot in the 'slot_var_name' variable. This also updates the\n"
	   "      Android slot metadata with a boot attempt, which can cause\n"
	   "      successive calls to this function to return a different result\n"
	   "      if the returned slot runs out of boot attempts.\n"
	   "    - If 'part_name' is passed, preceded with a # instead of :, the\n"
	   "      partition name whose label is 'part_name' will be looked up in\n"
	   "      the partition table. This is commonly the \"misc\" partition.\n"
);
static struct mtd_info *mtd_get_partition_level(
	struct mtd_info *mtd, const char *partition, int level)
{
	struct mtd_info *part;
	struct mtd_info *out;

	list_for_each_entry(part, &mtd->partitions, node) {
		if (!strcmp(part->name, partition))
			return part;

		out = mtd_get_partition_level(part, partition, level + 1);
		if (out)
			return out;
	}

	return NULL;
}

static struct mtd_info *mtd_get_partition(struct mtd_info **mtd,
					const char *partition)
{
	if (!mtd) {
		printf("Failed to get mtd info\n");
		return NULL;
	}

	if (!(*mtd)) {
		*mtd = get_mtd_device(NULL, 0);
		if (IS_ERR_OR_NULL(*mtd)) {
			*mtd = NULL;
			printf("Failed to get mtd device\n");
			return NULL;
		}

	}

	return mtd_get_partition_level(*mtd, partition, 1);
}

static int ab_control_create_from_mtd(struct mtd_info *mtd,
				struct bootloader_message_ab **abc,
				const char *abc_partition)
{
	struct mtd_info *part = mtd_get_partition(&mtd, abc_partition);
	void *buffer;
	int ret;
	size_t out_num_read;

	if (!part)
		return -ENOENT;

	buffer = malloc(sizeof(**abc));
	if (!buffer) {
		return -ENOMEM;
	}

	*abc = buffer;
	ret = mtd_read(part, 0, sizeof(**abc), &out_num_read, buffer);
	if (ret || out_num_read < sizeof(*abc))
		return -EIO;

	return 0;
}

static int ab_control_store_to_mtd(struct mtd_info *mtd,
				struct bootloader_message_ab *abc,
                const char *abc_partition)
{
	struct mtd_info *part = mtd_get_partition(&mtd, abc_partition);
	struct erase_info erase_op = {};
	int ret = 0;
	size_t out_num_write;

	if (!part)
		return -ENOENT;

	erase_op.mtd = mtd;
	erase_op.addr = ALIGN_DOWN(part->offset, mtd->erasesize);
	erase_op.len = ALIGN(part->size, mtd->erasesize);
	erase_op.scrub = false;
	while (erase_op.len) {
		ret = mtd_erase(mtd, &erase_op);

		/* Abort if its not a bad block error */
		if (ret)
			break;

		/* Skip bad block and continue behind it */
		erase_op.len -= mtd->erasesize;
		erase_op.addr = erase_op.addr + mtd->erasesize;
	}

	if (ret) {
		printf("erase failed with status %d\n", ret);
		return ret;
	}

	ret = mtd_write(part, 0, sizeof(*abc), &out_num_write, (void *)abc);
	if (ret || out_num_write < sizeof(*abc))
		return -EIO;

	return 0;
}

static int do_ab_select_mtd(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	int ret;
	struct mtd_info *mtd;
	struct bootloader_message_ab *bm_ab;
	struct bootloader_control *abc = NULL;
	const char *part_str;
	const char *dev_part_str;
	char *ep;
	int dev_num;
	int slot;

	dev_part_str = argv[3];
	part_str = strchr(dev_part_str, '#');
	if (!part_str || part_str == dev_part_str)
		return CMD_RET_USAGE;

	dev_num = (int)simple_strtoul(dev_part_str, &ep, 16);
	if (ep != part_str) {
		/* Not all the first part before the # was parsed. */
		return CMD_RET_USAGE;
	}

	part_str++;

	mtd = get_mtd_device(NULL, dev_num);
	if (IS_ERR_OR_NULL(mtd)) {
		pr_err("MTD device not found, ret %ld\n", PTR_ERR(mtd));
		return -ENODEV;
	}

	ret = ab_control_create_from_mtd(mtd, &bm_ab, part_str);
	if (ret) {
		printf("Failed to get ab control from mtd partition: %s "
			"with status: %d\n", part_str, ret);
		ret = CMD_RET_FAILURE;
		goto free_mtd;
	}

	abc = (void *)&bm_ab->slot_suffix;
	slot = get_slot_from_abc(abc);
	printf("slot is :%c\n", slot + 'a');
	ret = CMD_RET_SUCCESS;

free_mtd:
	put_mtd_device(mtd);
	return ret;
}

static int do_ab_corrupt_mtd(struct cmd_tbl *cmdtp, int flag, int argc,
			char * const argv[])
{
	int ret;
	struct mtd_info *mtd;
	struct bootloader_message_ab *bm_ab;
	struct bootloader_control *abc = NULL;
	const char *part_str;
	const char *dev_part_str;
	char *ep;
	int dev_num;
	int slot;

	if (argc != 4)
		return CMD_RET_USAGE;

	/* Android standard slot names are 'a', 'b', ... */
	slot = argv[1][0] - BOOT_SLOT_NAME(0);
	if (argv[1][1] != '\0') {
		return CMD_RET_USAGE;
	}

	dev_part_str = argv[3];
	part_str = strchr(dev_part_str, '#');
	if (!part_str || part_str == dev_part_str)
		return CMD_RET_USAGE;

	dev_num = simple_strtoul(dev_part_str, &ep, 16);
	if (ep != part_str) {
		/* Not all the first part before the # was parsed. */
		return CMD_RET_USAGE;
	}

	part_str++;

	mtd = get_mtd_device(NULL, dev_num);
	if (IS_ERR_OR_NULL(mtd)) {
		pr_err("MTD device not found, ret %ld\n", PTR_ERR(mtd));
		return -ENODEV;
	}

	ret = ab_control_create_from_mtd(mtd, &bm_ab, part_str);
	if (ret) {
		printf("Failed to get ab control from mtd partition: %s "
			"with status: %d\n", part_str, ret);
		ret = CMD_RET_FAILURE;
		goto free_mtd;
	}

	abc = (void *)&bm_ab->slot_suffix;

	ret = ab_set_verity_corrupted_mem(abc, slot);
	if (ret < 0) {
		printf("Set corrupt failed, error %d.\n", ret);
		ret = CMD_RET_FAILURE;
		goto free_mtd;
	}

	ret = ab_control_store_to_mtd(mtd, bm_ab, part_str);
	if (ret < 0) {
		printf("Write to flash failed, error %d.\n", ret);
		ret = CMD_RET_FAILURE;
		goto free_mtd;
	}

	printf("Set slot %d as invalid.\n", slot);
	ret = CMD_RET_SUCCESS;

free_mtd:
	free(bm_ab);
	put_mtd_device(mtd);

	return ret;
}

static int do_ab_corrupt(struct cmd_tbl *cmdtp, int flag, int argc,
			char * const argv[])
{
    int ret;
    struct blk_desc *dev_desc;
    struct disk_partition part_info;
    int slot;

	if (argc != 4)
		return CMD_RET_USAGE;

	if (!strncmp(argv[2], "mtd", 3)) {
		return do_ab_corrupt_mtd(cmdtp, flag, argc, argv);
	}
        /* Lookup the "misc" partition from argv[2] and argv[3] */
	if (part_get_info_by_dev_and_name_or_num(argv[2], argv[3],
						 &dev_desc, &part_info, false) < 0) {
		return CMD_RET_FAILURE;
	}

	/* Android standard slot names are 'a', 'b', ... */
	slot = argv[1][0] - BOOT_SLOT_NAME(0);
	if (argv[1][1] != '\0') {
		return CMD_RET_FAILURE;
	}

	ret = ab_set_verity_corrupted(dev_desc, &part_info, slot);
	if (ret < 0) {
		printf("Android boot failed, error %d.\n", ret);
		return CMD_RET_FAILURE;
	}

	return CMD_RET_FAILURE;
}

U_BOOT_CMD(ab_corrupt, 4, 0, do_ab_corrupt,
	   "Set the slot to be corrupted.",
	   "<slot> <interface> <dev[:part|#part_name]>\n"
);