/*
 * memdump.c --- Description
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

#include <common.h>
#include <part.h>
#include <config.h>
#include <command.h>
#include <image.h>
#include <linux/ctype.h>
#include <asm/byteorder.h>
#include <linux/stat.h>
#include <malloc.h>
#include <fs.h>
#include <mtd.h>
#include <linux/mtd/mtd.h>
#include <asm/sections.h>
#include <memdump.h>

enum dump_type_t {
	DUMP_TYPE_NONE = 0,
	DUMP_TYPE_USERDATA = 1,
	DUMP_TYPE_SDCARD = 2
};

static struct {
	char *intf;
	int dev;
	enum dump_type_t dst_type;
	union {
		char *partition;
		struct {
			int part;
			char *directory;
		} ext4_desc;
	} dest;
} memdump_info;

static int do_memdump_init(struct cmd_tbl *cmdtp, int flag,
			int argc, char * const argv[])
{
	char *part_str = NULL;
	char *dup_str = NULL;

	if (argc != 4)
		return CMD_RET_USAGE;

	if (memdump_info.intf)
		free(memdump_info.intf);

	if (memdump_info.dst_type == DUMP_TYPE_USERDATA)
		free(memdump_info.dest.ext4_desc.directory);
	else if (memdump_info.dst_type == DUMP_TYPE_SDCARD)
		free(memdump_info.dest.partition);

	memdump_info.intf = strdup(argv[1]);
	part_str = strchr(argv[2], ':');
	if (part_str) {
		dup_str = strdup(argv[2]);
		dup_str[part_str - argv[2]] = 0;
		part_str++;
		memdump_info.dev = simple_strtoul(dup_str, NULL, 16);
		memdump_info.dest.ext4_desc.part = simple_strtoul(part_str, NULL, 16);
		memdump_info.dest.ext4_desc.directory = strdup(argv[3]);
		free(dup_str);
		memdump_info.dst_type = DUMP_TYPE_USERDATA;
	} else {
		memdump_info.dev = simple_strtoul(argv[2], NULL, 16);
		memdump_info.dest.partition = strdup(argv[3]);
		memdump_info.dst_type = DUMP_TYPE_SDCARD;
	}

    return CMD_RET_SUCCESS;
}

static int do_memdump_all(struct cmd_tbl *cmdtp, int flag,
		int argc, char * const argv[])
{
	int ret;

	ret = memdump_dump_blk(memdump_info.intf, memdump_info.dev, memdump_info.dest.partition);
	if (ret) {
		printf("memdump failed!\n");
		return CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}

#ifdef ENABLE_USERDATA_MEMDUMP
static int do_memdump_to_userdata(struct cmd_tbl *cmdtp, int flag,
		int argc, char * const argv[])
{
	int ret;
	char part_str[10] = {0};
	char *default_args[] = {"init", RAMDUMP_USERDATA_STORAGE,
							NULL, RAMDUMP_USERDATA_PATH};

	if (search_userdata_part(part_str, sizeof(part_str)))
		return CMD_RET_FAILURE;

	default_args[2] = part_str;
	if (memdump_info.dst_type != DUMP_TYPE_USERDATA) {
		do_memdump_init(cmdtp, flag, ARRAY_SIZE(default_args), default_args);
	}
	printf("intf %s,dev %d,part %d directory %s\n",memdump_info.intf, memdump_info.dev, memdump_info.dest.ext4_desc.part,memdump_info.dest.ext4_desc.directory);
	ret = memdump_dump_ext4(memdump_info.intf, memdump_info.dev,
							memdump_info.dest.ext4_desc.part, memdump_info.dest.ext4_desc.directory);
	if (ret) {
		printf("memdump to userdata failed!ret=%d\n",ret);
		return CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}
#endif

static struct cmd_tbl cmd_swinfo[] = {
	U_BOOT_CMD_MKENT(init, 4, 0, do_memdump_init, "", ""),
	U_BOOT_CMD_MKENT(dumpall, 2, 0, do_memdump_all, "", ""),
#ifdef ENABLE_USERDATA_MEMDUMP
	U_BOOT_CMD_MKENT(userdata, 1, 0, do_memdump_to_userdata, "", ""),
#endif
};

static inline void set_memdump_test_false(void)
{
	if (env_get_yesno("memdump_test") == 1)
		env_set("memdump_test","0");
}

static int do_memdump(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	struct cmd_tbl *cp;
	int ret;
	cp = find_cmd_tbl(argv[1], cmd_swinfo, ARRAY_SIZE(cmd_swinfo));

	/* Drop the mmc command */
	argc--;
	argv++;

	if (cp == NULL || argc > cp->maxargs) {
		ret = CMD_RET_USAGE;
		goto out;
	}
	if (flag == CMD_FLAG_REPEAT && !cmd_is_repeatable(cp)){
		ret = CMD_RET_SUCCESS;
		goto out;
	}
	ret = cp->cmd(cmdtp, flag, argc, argv);
out:
	set_memdump_test_false();
	return ret;
}

U_BOOT_CMD(
	memdump, 5, 0, do_memdump,
	"memdump system memory to flash",
	"init <interface> <dev:part> <directory(ext4)> - Initialize the interface's <dev:part> to dump memory\n"
	"memdump init <interface> <dev> <partition>/<directory(ext4)> - Initialize the interface's <dev>/<partition> to dump memory\n"
	"memdump dumpall - Dump all memory to the ramdump partition of the <dev>\n"
#ifdef ENABLE_USERDATA_MEMDUMP
	"memdump userdata - Dump all memory to /userdata/DDRCS*.bin of the dev <mmc 0>\n"
#endif
	);
