// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (C) 2016 The Android Open Source Project
 */

#include <common.h>
#include <env.h>
#include <fastboot.h>
#include <fastboot-internal.h>
#include <fs.h>
#include <part.h>
#include <version.h>
#include <fb_storage.h>

#include <asm/global_data.h>
DECLARE_GLOBAL_DATA_PTR;

static void getvar_version(char *var_parameter, char *response);
static void getvar_version_bootloader(char *var_parameter, char *response);
static void getvar_downloadsize(char *var_parameter, char *response);
static void getvar_maxfetchsize(char *var_parameter, char *response);
static void getvar_serialno(char *var_parameter, char *response);
static void getvar_version_baseband(char *var_parameter, char *response);
static void getvar_product(char *var_parameter, char *response);
static void getvar_platform(char *var_parameter, char *response);
static void getvar_current_slot(char *var_parameter, char *response);
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
static void getvar_has_slot(char *var_parameter, char *response);
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
static void getvar_partition_type(char *part_name, char *response);
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
static void getvar_partition_size(char *part_name, char *response);
static void getvar_block_size(char *part_name, char *response);
#endif
static void getvar_is_userspace(char *var_parameter, char *response);

static const struct {
	const char *variable;
	void (*dispatch)(char *var_parameter, char *response);
} getvar_dispatch[] = {
	{
		.variable = "version",
		.dispatch = getvar_version
	}, {
		.variable = "version-bootloader",
		.dispatch = getvar_version_bootloader
	}, {
		.variable = "downloadsize",
		.dispatch = getvar_downloadsize
	}, {
		.variable = "max-download-size",
		.dispatch = getvar_downloadsize
	}, {
		.variable = "max-fetch-size",
		.dispatch = getvar_maxfetchsize
	},{
		.variable = "serialno",
		.dispatch = getvar_serialno
	}, {
		.variable = "version-baseband",
		.dispatch = getvar_version_baseband
	}, {
		.variable = "product",
		.dispatch = getvar_product
	}, {
		.variable = "platform",
		.dispatch = getvar_platform
	}, {
		.variable = "current-slot",
		.dispatch = getvar_current_slot
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
	}, {
		.variable = "has-slot",
		.dispatch = getvar_has_slot
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
	}, {
		.variable = "partition-type",
		.dispatch = getvar_partition_type
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
	}, {
		.variable = "partition-size",
		.dispatch = getvar_partition_size
	}, {
		.variable = "block-size",
		.dispatch = getvar_block_size
#endif
	}, {
		.variable = "is-userspace",
		.dispatch = getvar_is_userspace
	}
};

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
/**
 * Get partition number and size for any storage type.
 *
 * Can be used to check if partition with specified name exists.
 *
 * If error occurs, this function guarantees to fill @p response with fail
 * string. @p response can be rewritten in caller, if needed.
 *
 * @param[in] part_name Info for which partition name to look for
 * @param[in,out] response Pointer to fastboot response buffer
 * @param[out] size If not NULL, will contain partition size
 * Return: Partition number or negative value on error
 */
static int getvar_get_part(const char *part_name, char *response,
				size_t *size)
{
	const struct fastboot_storage_ops *ops = get_current_storage_ops();

	if (!ops || !ops->get_part) {
		fastboot_fail("Operation not supported", response);
		return -ENOSYS;
	}

	return ops->get_part(part_name, size, response);
}
#endif

static void getvar_version(char *var_parameter, char *response)
{
	fastboot_okay(FASTBOOT_VERSION, response);
}

static void getvar_version_bootloader(char *var_parameter, char *response)
{
	fastboot_okay(U_BOOT_VERSION, response);
}

static void getvar_downloadsize(char *var_parameter, char *response)
{
	fastboot_response("OKAY", response, "0x%08x", fastboot_buf_size);
}

static void getvar_maxfetchsize(char *var_parameter, char *response)
{
	fastboot_response("DATA", response, "0x%08x", fastboot_buf_size);
}

static void getvar_serialno(char *var_parameter, char *response)
{
	const char *tmp = env_get("serial#");

	if (tmp)
		fastboot_okay(tmp, response);
	else
		fastboot_fail("Value not set", response);
}

static void getvar_version_baseband(char *var_parameter, char *response)
{
	fastboot_okay("N/A", response);
}

static void getvar_product(char *var_parameter, char *response)
{
	const char *board = env_get("board");

	if (board)
		fastboot_okay(board, response);
	else
		fastboot_fail("Board not set", response);
}

static void getvar_platform(char *var_parameter, char *response)
{
	const char *p = env_get("platform");

	if (p)
		fastboot_okay(p, response);
	else
		fastboot_fail("platform not set", response);
}

static void getvar_current_slot(char *var_parameter, char *response)
{
	/* A/B not implemented, for now always return "a" */
	fastboot_okay("a", response);
}

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
static void getvar_has_slot(char *part_name, char *response)
{
	char part_name_wslot[PART_NAME_LEN];
	size_t len;
	int r;

	struct fetch_info info;
	char cmd_copy[128];
	int ret;

	if (!part_name || part_name[0] == '\0')
		goto fail;

	/* Create working copy of command */
	strncpy(cmd_copy, part_name, sizeof(cmd_copy) - 1);
	cmd_copy[sizeof(cmd_copy) - 1] = '\0';

	/* Parse main fetch command */
	ret = fastboot_parse_fetch_cmd(cmd_copy, &info);
	if (ret) {
		fastboot_fail("cannot parse fetch command", response);
		return;
	}

	/* part_name_wslot = part_name + "_a" */
	len = strlcpy(part_name_wslot, info.part_name, PART_NAME_LEN - 3);
	if (len > PART_NAME_LEN - 3)
		goto fail;
	strcat(part_name_wslot, "_a");

	r = getvar_get_part(part_name_wslot, response, NULL);
	if (r >= 0) {
		fastboot_okay("yes", response); /* part exists and slotted */
		return;
	}

	r = getvar_get_part(info.part_name, response, NULL);
	if (r >= 0)
		fastboot_okay("no", response); /* part exists but not slotted */

	/* At this point response is filled with okay or fail string */
	return;

fail:
	fastboot_fail("invalid partition name", response);
}
#endif

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
static void getvar_partition_type(char *part_name, char *response)
{
	const struct fastboot_storage_ops *ops = get_current_storage_ops();

	if (ops && ops->get_part_type)
		ops->get_part_type(part_name, response);
}
#endif

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
static void getvar_partition_size(char *part_name, char *response)
{
	int ret;
	size_t size;

	struct fetch_info info;
	char cmd_copy[128];

	const struct fastboot_storage_ops *ops = get_current_storage_ops();

	/* Create working copy of command */
	strncpy(cmd_copy, part_name, sizeof(cmd_copy) - 1);
	cmd_copy[sizeof(cmd_copy) - 1] = '\0';

	/* Parse main fetch command */
	ret = fastboot_parse_fetch_cmd(cmd_copy, &info);
	if (ret) {
		fastboot_fail("cannot parse fetch command", response);
		return;
	}

	if (info.type == FETCH_PARTITION) {
		ret = getvar_get_part(info.part_name, response, &size);
		if (ret >= 0)
			fastboot_response("OKAY", response, "0x%016zx", size);
	} else if (info.type == FETCH_PART_RANGE) {
		fastboot_response("OKAY", response, "0x%016zx", info.size);
	} else if (info.type == FETCH_ADDR_PART) {
		if (ops && ops->get_fetch_size)
			ops->get_fetch_size(info.part_name, info.addr, response);
	} else if (info.type == FETCH_ADDR_RANGE) {
		fastboot_response("OKAY", response, "0x%016zx", info.size);
	} else if (info.type == FETCH_RAMDUMP) {
		printf("RAMDUMP, DRAM size: 0x%llx\n", gd->ram_size);
		fastboot_response("OKAY", response, "0x%016llx", gd->ram_size);
	} else if (info.type == FETCH_RAMDUMP_RANGE) {
		printf("RAMDUMP_RANGE, size: 0x%zx\n", info.size);
		fastboot_response("OKAY", response, "0x%016zx", info.size);
	}
}
#endif

static void getvar_is_userspace(char *var_parameter, char *response)
{
	fastboot_okay("no", response);
}

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
static void getvar_block_size(char *part_name, char *response)
{
	int ret = -1;
	size_t size = 0;

	const struct fastboot_storage_ops *ops = get_current_storage_ops();

	if (ops && ops->get_block_size)
		ret = ops->get_block_size(part_name, &size, response);

	if (ret >= 0)
		fastboot_response("OKAY", response, "0x%016zx", size);
}
#endif

/**
 * fastboot_getvar() - Writes variable indicated by cmd_parameter to response.
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 *
 * Look up cmd_parameter first as an environment variable of the form
 * fastboot.<cmd_parameter>, if that exists return use its value to set
 * response.
 *
 * Otherwise lookup the name of variable and execute the appropriate
 * function to return the requested value.
 */
void fastboot_getvar(char *cmd_parameter, char *response)
{
	if (!cmd_parameter) {
		fastboot_fail("missing var", response);
	} else {
#define FASTBOOT_ENV_PREFIX	"fastboot."
		int i;
		char *var_parameter = cmd_parameter;
		char envstr[FASTBOOT_RESPONSE_LEN];
		const char *s;

		snprintf(envstr, sizeof(envstr) - 1,
			 FASTBOOT_ENV_PREFIX "%s", cmd_parameter);
		s = env_get(envstr);
		if (s) {
			fastboot_response("OKAY", response, "%s", s);
			return;
		}

		strsep(&var_parameter, ":");
		for (i = 0; i < ARRAY_SIZE(getvar_dispatch); ++i) {
			if (!strcmp(getvar_dispatch[i].variable,
				    cmd_parameter)) {
				getvar_dispatch[i].dispatch(var_parameter,
							    response);
				return;
			}
		}
		pr_warn("WARNING: unknown variable: %s\n", cmd_parameter);
		fastboot_fail("Variable not implemented", response);
	}
}
