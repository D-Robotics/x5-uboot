// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (C) 2016 The Android Open Source Project
 */

#include <common.h>
#include <command.h>
#include <env.h>
#include <fastboot.h>
#include <fastboot-internal.h>
#include <fb_mmc.h>
#include <fb_nand.h>
#include <fb_spinand.h>
#include <fb_ram.h>
#include <part.h>
#include <stdlib.h>
#include <mapmem.h>

#include <asm/global_data.h>

DECLARE_GLOBAL_DATA_PTR;

#define EP_BUFFER_SIZE			4096


/**
 * image_size - final fastboot image size
 */
static u32 image_size;

/**
 * fastboot_bytes_received - number of bytes received in the current download
 */
static u32 fastboot_bytes_received;

/**
 * fastboot_bytes_send - number of bytes received in the current upload
 */
static u32 fastboot_bytes_send;

/**
 * fastboot_bytes_expected - number of bytes expected in the current download
 */
static u32 fastboot_bytes_expected;

static void okay(char *, char *);
static void getvar(char *, char *);
static void download(char *, char *);
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
static void flash(char *, char *);
static void erase(char *, char *);
#endif
static void reboot_bootloader(char *, char *);
static void reboot_fastbootd(char *, char *);
static void reboot_recovery(char *, char *);
#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_FORMAT)
static void oem_format(char *, char *);
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_PARTCONF)
static void oem_partconf(char *, char *);
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_BOOTBUS)
static void oem_bootbus(char *, char *);
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_RAMDUMP)
static void oem_ramdump(char *cmd_parameter, char *response);
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_SET_MEDIUM)
static void oem_set_medium(char *cmd_parameter, char *response);
#endif

#if CONFIG_IS_ENABLED(FASTBOOT_UUU_SUPPORT)
static void run_ucmd(char *, char *);
static void run_acmd(char *, char *);
#endif

static const struct {
	const char *command;
	void (*dispatch)(char *cmd_parameter, char *response);
} commands[FASTBOOT_COMMAND_COUNT] = {
	[FASTBOOT_COMMAND_GETVAR] = {
		.command = "getvar",
		.dispatch = getvar
	},
	[FASTBOOT_COMMAND_DOWNLOAD] = {
		.command = "download",
		.dispatch = download
	},
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
	[FASTBOOT_COMMAND_FLASH] =  {
		.command = "flash",
		.dispatch = flash
	},
	[FASTBOOT_COMMAND_ERASE] =  {
		.command = "erase",
		.dispatch = erase
	},
#endif
	[FASTBOOT_COMMAND_BOOT] =  {
		.command = "boot",
		.dispatch = okay
	},
	[FASTBOOT_COMMAND_CONTINUE] =  {
		.command = "continue",
		.dispatch = okay
	},
	[FASTBOOT_COMMAND_REBOOT] =  {
		.command = "reboot",
		.dispatch = okay
	},
	[FASTBOOT_COMMAND_REBOOT_BOOTLOADER] =  {
		.command = "reboot-bootloader",
		.dispatch = reboot_bootloader
	},
	[FASTBOOT_COMMAND_REBOOT_FASTBOOTD] =  {
		.command = "reboot-fastboot",
		.dispatch = reboot_fastbootd
	},
	[FASTBOOT_COMMAND_REBOOT_RECOVERY] =  {
		.command = "reboot-recovery",
		.dispatch = reboot_recovery
	},
	[FASTBOOT_COMMAND_SET_ACTIVE] =  {
		.command = "set_active",
		.dispatch = okay
	},
#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_FORMAT)
	[FASTBOOT_COMMAND_OEM_FORMAT] = {
		.command = "oem format",
		.dispatch = oem_format,
	},
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_PARTCONF)
	[FASTBOOT_COMMAND_OEM_PARTCONF] = {
		.command = "oem partconf",
		.dispatch = oem_partconf,
	},
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_BOOTBUS)
	[FASTBOOT_COMMAND_OEM_BOOTBUS] = {
		.command = "oem bootbus",
		.dispatch = oem_bootbus,
	},
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_RAMDUMP)
	[FASTBOOT_COMMAND_OEM_RAMDUMP] = {
		.command = "oem ramdump",
		.dispatch = oem_ramdump,
	},
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_SET_MEDIUM)
	[FASTBOOT_COMMAND_OEM_SET_MEDIUM] = {
		.command = "oem set_medium",
		.dispatch = oem_set_medium,
	},
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_UUU_SUPPORT)
	[FASTBOOT_COMMAND_UCMD] = {
		.command = "UCmd",
		.dispatch = run_ucmd,
	},
	[FASTBOOT_COMMAND_ACMD] = {
		.command = "ACmd",
		.dispatch = run_acmd,
	},
#endif
};

/**
 * fastboot_handle_command - Handle fastboot command
 *
 * @cmd_string: Pointer to command string
 * @response: Pointer to fastboot response buffer
 *
 * Return: Executed command, or -1 if not recognized
 */
int fastboot_handle_command(char *cmd_string, char *response)
{
	int i;
	char *cmd_parameter;

	cmd_parameter = cmd_string;
	strsep(&cmd_parameter, ":");

	for (i = 0; i < FASTBOOT_COMMAND_COUNT; i++) {
		if (!strcmp(commands[i].command, cmd_string)) {
			if (commands[i].dispatch) {
				commands[i].dispatch(cmd_parameter,
							response);
				return i;
			} else {
				break;
			}
		}
	}

	pr_err("command %s not recognized.\n", cmd_string);
	fastboot_fail("unrecognized command", response);
	return -1;
}

/**
 * okay() - Send bare OKAY response
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 *
 * Send a bare OKAY fastboot response. This is used where the command is
 * valid, but all the work is done after the response has been sent (e.g.
 * boot, reboot etc.)
 */
static void okay(char *cmd_parameter, char *response)
{
	fastboot_okay(NULL, response);
}

/**
 * getvar() - Read a config/version variable
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void getvar(char *cmd_parameter, char *response)
{
	fastboot_getvar(cmd_parameter, response);
}

/**
 * fastboot_download() - Start a download transfer from the client
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void download(char *cmd_parameter, char *response)
{
	char *tmp;

	if (!cmd_parameter) {
		fastboot_fail("Expected command parameter", response);
		return;
	}
	fastboot_bytes_received = 0;
	fastboot_bytes_expected = hextoul(cmd_parameter, &tmp);
	if (fastboot_bytes_expected == 0) {
		fastboot_fail("Expected nonzero image size", response);
		return;
	}
	/*
	 * Nothing to download yet. Response is of the form:
	 * [DATA|FAIL]$cmd_parameter
	 *
	 * where cmd_parameter is an 8 digit hexadecimal number
	 */
	if (fastboot_bytes_expected > fastboot_buf_size) {
		fastboot_fail(cmd_parameter, response);
	} else {
		printf("Starting download of %d bytes\n",
		       fastboot_bytes_expected);
		fastboot_response("DATA", response, "%s", cmd_parameter);
	}
}

/**
 * fastboot_download_remaining() - return bytes remaining in current transfer
 *
 * Return: Number of bytes left in the current download
 */
u32 fastboot_download_remaining(void)
{
	return fastboot_bytes_expected - fastboot_bytes_received;
}

/**
 * fastboot_data_download() - Copy image data to fastboot_buf_addr.
 *
 * @fastboot_data: Pointer to received fastboot data
 * @fastboot_data_len: Length of received fastboot data
 * @response: Pointer to fastboot response buffer
 *
 * Copies image data from fastboot_data to fastboot_buf_addr. Writes to
 * response. fastboot_bytes_received is updated to indicate the number
 * of bytes that have been transferred.
 *
 * On completion sets image_size and ${filesize} to the total size of the
 * downloaded image.
 */
void fastboot_data_download(const void *fastboot_data,
			    unsigned int fastboot_data_len,
			    char *response)
{
#define BYTES_PER_DOT	0x20000
	u32 pre_dot_num, now_dot_num;

	if (fastboot_data_len == 0 ||
	    (fastboot_bytes_received + fastboot_data_len) >
	    fastboot_bytes_expected) {
		fastboot_fail("Received invalid data length",
			      response);
		return;
	}
	/* Download data to fastboot_buf_addr */
	memcpy(fastboot_buf_addr + fastboot_bytes_received,
	       fastboot_data, fastboot_data_len);

	pre_dot_num = fastboot_bytes_received / BYTES_PER_DOT;
	fastboot_bytes_received += fastboot_data_len;
	now_dot_num = fastboot_bytes_received / BYTES_PER_DOT;

	if (pre_dot_num != now_dot_num) {
		putc('.');
		if (!(now_dot_num % 74))
			putc('\n');
	}
	*response = '\0';
}

/**
 * fastboot_download_complete() - Mark current transfer complete
 *
 * @response: Pointer to fastboot response buffer
 *
 * Set image_size and ${filesize} to the total size of the downloaded image.
 */
void fastboot_download_complete(char *response)
{
	/* Download complete. Respond with "OKAY" */
	fastboot_okay(NULL, response);
	printf("\ndownloading of %d bytes finished\n", fastboot_bytes_received);
	image_size = fastboot_bytes_received;
	env_set_hex("filesize", image_size);
	fastboot_bytes_expected = 0;
	fastboot_bytes_received = 0;
}

/**
 * fastboot_upload_complete() - Mark current transfer complete
 *
 * @response: Pointer to fastboot response buffer
 *
 * Mark current upload transfer complete, and reset global transfer info.
 */
void fastboot_upload_complete(char *response)
{
	/* Upload complete. Respond with "OKAY" */
	fastboot_okay(NULL, response);
	printf("\nuploading of %d bytes finished\n", fastboot_bytes_send);
	fastboot_bytes_expected = 0;
	fastboot_bytes_send = 0;
}

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
/**
 * flash() - write the downloaded image to the indicated partition.
 *
 * @cmd_parameter: Pointer to partition name
 * @response: Pointer to fastboot response buffer
 *
 * Writes the previously downloaded image to the partition indicated by
 * cmd_parameter. Writes to response.
 */
static void flash(char *cmd_parameter, char *response)
{
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
	if (fastboot_get_flash_type() == FLASH_TYPE_UNKNOWN ||
			fastboot_get_flash_type() == FLASH_TYPE_EMMC) {
		fastboot_mmc_flash_write(cmd_parameter, fastboot_buf_addr,
				image_size, response);
	}
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_NAND)
	if (fastboot_get_flash_type() == FLASH_TYPE_NAND)
		fastboot_nand_flash_write(cmd_parameter, fastboot_buf_addr,
				image_size, response);
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_SPINAND)
	if (fastboot_get_flash_type() == FLASH_TYPE_SPINAND)
		fastboot_spinand_flash_write(cmd_parameter, fastboot_buf_addr,
				image_size, response);
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_RAM)
	if (fastboot_get_flash_type() == FLASH_TYPE_RAM)
		fastboot_ram_flash_write(cmd_parameter, fastboot_buf_addr,
				image_size, response);
#endif
}

/**
 * erase() - erase the indicated partition.
 *
 * @cmd_parameter: Pointer to partition name
 * @response: Pointer to fastboot response buffer
 *
 * Erases the partition indicated by cmd_parameter (clear to 0x00s). Writes
 * to response.
 */
static void erase(char *cmd_parameter, char *response)
{
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
	if (fastboot_get_flash_type() == FLASH_TYPE_UNKNOWN ||
			fastboot_get_flash_type() == FLASH_TYPE_EMMC) {
		fastboot_mmc_erase(cmd_parameter, response);
	}
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_NAND)
	if (fastboot_get_flash_type() == FLASH_TYPE_NAND)
		fastboot_nand_erase(cmd_parameter, response);
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_SPINAND)
	if (fastboot_get_flash_type() == FLASH_TYPE_SPINAND)
		fastboot_spinand_erase(cmd_parameter, response);
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_RAM)
	if (fastboot_get_flash_type() == FLASH_TYPE_RAM)
		fastboot_ram_erase(cmd_parameter, response);
#endif
}
#endif

/**
 * fastboot_upload_remaining() - return bytes remaining in current transfer
 *
 * Return: Number of bytes left in the current upload
 */
u32 fastboot_upload_remaining(void)
{
	return fastboot_bytes_expected - fastboot_bytes_send;
}

/**
 * fastboot_data_upload() - Copy indicated data to in_ep->buf.
 *
 * @fastboot_data: Pointer to fastboot data need be sent
 * @fastboot_data_len: Length of fastboot data need be sent
 * @response: Pointer to fastboot response buffer
 *
 */
void fastboot_data_upload(void *fastboot_data,
			    unsigned int fastboot_data_len,
			    char *response)
{
#define BYTES_PER_DOT	0x20000
	u32 pre_dot_num, now_dot_num;
	void *dram_start_addr = (void *)PHYS_SDRAM_1;

	if (fastboot_data_len == 0 ||
	    (fastboot_bytes_send + fastboot_data_len) >
	    fastboot_bytes_expected) {
		fastboot_fail("Invalid data length for send",
			      response);
		return;
	}

	/* Upload data to fastboot_data */
	memcpy(fastboot_data, dram_start_addr + fastboot_bytes_send,
			fastboot_data_len);

	pre_dot_num = fastboot_bytes_send / BYTES_PER_DOT;
	fastboot_bytes_send += fastboot_data_len;
	now_dot_num = fastboot_bytes_send / BYTES_PER_DOT;

	if (pre_dot_num != now_dot_num) {
		putc('.');
		if (!(now_dot_num % 74))
			putc('\n');
	}
	*response = '\0';
}

#if CONFIG_IS_ENABLED(FASTBOOT_UUU_SUPPORT)
/**
 * run_ucmd() - Execute the UCmd command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void run_ucmd(char *cmd_parameter, char *response)
{
	if (!cmd_parameter) {
		pr_err("missing slot suffix\n");
		fastboot_fail("missing command", response);
		return;
	}

	if (run_command(cmd_parameter, 0))
		fastboot_fail("", response);
	else
		fastboot_okay(NULL, response);
}

static char g_a_cmd_buff[64];

void fastboot_acmd_complete(void)
{
	run_command(g_a_cmd_buff, 0);
}

/**
 * run_acmd() - Execute the ACmd command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void run_acmd(char *cmd_parameter, char *response)
{
	if (!cmd_parameter) {
		pr_err("missing slot suffix\n");
		fastboot_fail("missing command", response);
		return;
	}

	if (strlen(cmd_parameter) > sizeof(g_a_cmd_buff)) {
		pr_err("too long command\n");
		fastboot_fail("too long command", response);
		return;
	}

	strcpy(g_a_cmd_buff, cmd_parameter);
	fastboot_okay(NULL, response);
}
#endif

/**
 * reboot_bootloader() - Sets reboot bootloader flag.
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void reboot_bootloader(char *cmd_parameter, char *response)
{
	if (fastboot_set_reboot_flag(FASTBOOT_REBOOT_REASON_BOOTLOADER))
		fastboot_fail("Cannot set reboot flag", response);
	else
		fastboot_okay(NULL, response);
}

/**
 * reboot_fastbootd() - Sets reboot fastboot flag.
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void reboot_fastbootd(char *cmd_parameter, char *response)
{
	if (fastboot_set_reboot_flag(FASTBOOT_REBOOT_REASON_FASTBOOTD))
		fastboot_fail("Cannot set fastboot flag", response);
	else
		fastboot_okay(NULL, response);
}

/**
 * reboot_recovery() - Sets reboot recovery flag.
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void reboot_recovery(char *cmd_parameter, char *response)
{
	if (fastboot_set_reboot_flag(FASTBOOT_REBOOT_REASON_RECOVERY))
		fastboot_fail("Cannot set recovery flag", response);
	else
		fastboot_okay(NULL, response);
}

#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_FORMAT)
/**
 * oem_format() - Execute the OEM format command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void oem_format(char *cmd_parameter, char *response)
{
	char cmdbuf[32];
	u32 mmc_dev;

	mmc_dev = (fastboot_medium_devnum() < 0) ?
		CONFIG_FASTBOOT_FLASH_MMC_DEV : fastboot_medium_devnum();

	if (!env_get("partitions")) {
		fastboot_fail("partitions not set", response);
	} else {
		sprintf(cmdbuf, "gpt write mmc %x $partitions",
			mmc_dev);
		if (run_command(cmdbuf, 0))
			fastboot_fail("", response);
		else
			fastboot_okay(NULL, response);
	}
}
#endif

#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_PARTCONF)
/**
 * oem_partconf() - Execute the OEM partconf command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void oem_partconf(char *cmd_parameter, char *response)
{
	char cmdbuf[32];
	u32 mmc_dev;

	mmc_dev = (fastboot_medium_devnum() < 0) ?
		CONFIG_FASTBOOT_FLASH_MMC_DEV : fastboot_medium_devnum();

	if (!cmd_parameter) {
		fastboot_fail("Expected command parameter", response);
		return;
	}

	/* execute 'mmc partconfg' command with cmd_parameter arguments*/
	snprintf(cmdbuf, sizeof(cmdbuf), "mmc partconf %x %s 0",
		 mmc_dev, cmd_parameter);
	printf("Execute: %s\n", cmdbuf);
	if (run_command(cmdbuf, 0))
		fastboot_fail("Cannot set oem partconf", response);
	else
		fastboot_okay(NULL, response);
}
#endif

#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_BOOTBUS)
/**
 * oem_bootbus() - Execute the OEM bootbus command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void oem_bootbus(char *cmd_parameter, char *response)
{
	char cmdbuf[32];
	u32 mmc_dev;

	if (!cmd_parameter) {
		fastboot_fail("Expected command parameter", response);
		return;
	}

	mmc_dev = (fastboot_medium_devnum() < 0) ?
		CONFIG_FASTBOOT_FLASH_MMC_DEV : fastboot_medium_devnum();

	/* execute 'mmc bootbus' command with cmd_parameter arguments*/
	snprintf(cmdbuf, sizeof(cmdbuf), "mmc bootbus %x %s",
		 mmc_dev, cmd_parameter);
	printf("Execute: %s\n", cmdbuf);
	if (run_command(cmdbuf, 0))
		fastboot_fail("Cannot set oem bootbus", response);
	else
		fastboot_okay(NULL, response);
}
#endif

#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_RAMDUMP)
/**
 * oem_ramdump() - Execute the OEM ramdump command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void oem_ramdump(char *cmd_parameter, char *response)
{
	char *tmp;

	if (!cmd_parameter)
		fastboot_bytes_expected = gd->ram_size;
	else
		fastboot_bytes_expected = simple_strtoul(cmd_parameter, &tmp, 16);

	if (!fastboot_bytes_expected) {
		fastboot_fail("Expected nonzero image size", response);
		return;
	}

	fastboot_response("DATA", response, "%08x", fastboot_bytes_expected);
	fastboot_tx_write_more(response);

	fastboot_upload_ramdump();

	fastboot_none_resp(response);
}
#endif

#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_SET_MEDIUM)
/**
 * oem_set_medium() - Execute the OEM set_medium command
 * indicate the flash medium type and number
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void oem_set_medium(char *cmd_parameter, char *response)
{
	fb_flash_type flash_type;
	unsigned long medium_num;

	if (!cmd_parameter) {
		fastboot_fail("Expected command parameter", response);
		return;
	}

	if (strncmp(cmd_parameter, "mmc", 3) == 0) {
		flash_type = FLASH_TYPE_EMMC;
		if (strict_strtoul(cmd_parameter + 3, 10, &medium_num) < 0) {
			pr_err("please indicate the detail mmc devnum(eg. mmc0, mmc1)\n");
			fastboot_fail("please indicate the detail mmc devnum(eg. mmc0, mmc1)", response);
			return;
		}
	} else if (strncmp(cmd_parameter, "spinand", 7) == 0) {
		flash_type = FLASH_TYPE_SPINAND;
		if (strict_strtoul(cmd_parameter + 7, 10, &medium_num) < 0) {
			pr_err("please indicate the detail spinand devnum(eg. spinand0, spinand1)\n");
			fastboot_fail("please indicate the detail spinand devnum(eg. spinand0, spinand1)", response);
			return;
		}
	} else {
		pr_err("only suport indicate mmc & spinand medium\n");
		fastboot_fail("only suport indicate mmc & spinand medium", response);
		return;
	}

	fastboot_set_medium(flash_type, medium_num);

	printf("fastboot set medium to %s\n", cmd_parameter);
	fastboot_okay(NULL, response);
}

#endif
