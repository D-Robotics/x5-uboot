/*
 * btype.c --- Hobot Board Type Command
 *
 * Copyright (C) 2023, ming.yu, all rights reserved.
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
#include <log.h>
#include <command.h>
#include <stdio.h>
#include <dm.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/string.h>
#include <mapmem.h>
#include <asm/io.h>
#include <asm/arch/hb_aon.h>
#include <dm/lists.h>
#include <dm/device-internal.h>
#include <dm/of_access.h>
#include <android_bootloader_message.h>
#include <android_ab.h>
#include <asm/arch/hb_board.h>
//#include <hb_info.h>

DECLARE_GLOBAL_DATA_PTR;
#define MAX_SLOT_NO   2

int get_boot_slot(void)
{
	int slot = -1;

    slot = AON_AB_SLOT_VALUE(readl(AON_STATUS_REG1));

	return slot;
}

static int do_ab_select(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	char slot[20];
	int slot_idx = 0;

	slot_idx = get_boot_slot();

	if (slot_idx < 0 || slot_idx > (MAX_SLOT_NO - 1)) {
		printf("AB select failed, error %d.\n", slot_idx);
		return CMD_RET_FAILURE;
	}

	slot[0] = BOOT_SLOT_NAME(slot_idx);
	slot[1] = '\0';
	if (argc >= 2)
		env_set(argv[1], slot);

	printf("ANDROID: Booting slot: %s\n", slot);

	return CMD_RET_SUCCESS;
}

static int do_update_bootargs(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
    board_bootargs_setup();
    return CMD_RET_SUCCESS;
}

static struct cmd_tbl cmd_btype[] = {
	U_BOOT_CMD_MKENT(ab_select, 3, 0, do_ab_select, "", ""),
	U_BOOT_CMD_MKENT(bootargs, 3, 0, do_update_bootargs, "", ""),
};

static int do_btype(struct cmd_tbl * cmdtp, int flag, int argc, char *const argv[])
{
	struct cmd_tbl *cp;

	cp = find_cmd_tbl(argv[1], cmd_btype, ARRAY_SIZE(cmd_btype));

	/* Drop the base command */
	argc--;
	argv++;

	if (cp == NULL || argc > cp->maxargs)
		return CMD_RET_USAGE;

	return cp->cmd(cmdtp, flag, argc, argv);
}

U_BOOT_CMD(
	btype, 4, 0, do_btype,
	"board type utility commands",
	"btype \n"
	" Example usage:\n"
	" btype ab_select <slot_var_name> - get ab slot to boot\n"
	);
