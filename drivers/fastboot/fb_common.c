// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2008 - 2009
 * Windriver, <www.windriver.com>
 * Tom Rix <Tom.Rix@windriver.com>
 *
 * Copyright 2011 Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * Copyright 2014 Linaro, Ltd.
 * Rob Herring <robh@kernel.org>
 */

#include <bcb.h>
#include <common.h>
#include <command.h>
#include <env.h>
#include <fastboot.h>
#include <net/fastboot.h>

/**
 * fastboot_buf_addr - base address of the fastboot download buffer
 */
void *fastboot_buf_addr;

/**
 * fastboot_buf_size - size of the fastboot download buffer
 */
u32 fastboot_buf_size;

/**
 * selected_flash_type - user selected flash type (emmc/nand/spinand)
 */
fb_flash_type selected_flash_type = FLASH_TYPE_UNKNOWN;

/**
 * fastboot_medium_number - fastboot flash medium number info
 * >= 0: meaningful
 * < 0: no user input
 */
s32 fastboot_medium_number;

/**
 * fastboot_progress_callback - callback executed during long operations
 */
void (*fastboot_progress_callback)(const char *msg);

/**
 * fastboot_response() - Writes a response of the form "$tag$reason".
 *
 * @tag: The first part of the response
 * @response: Pointer to fastboot response buffer
 * @format: printf style format string
 */
void fastboot_response(const char *tag, char *response,
		       const char *format, ...)
{
	va_list args;

	strlcpy(response, tag, FASTBOOT_RESPONSE_LEN);
	if (format) {
		va_start(args, format);
		vsnprintf(response + strlen(response),
			  FASTBOOT_RESPONSE_LEN - strlen(response) - 1,
			  format, args);
		va_end(args);
	}
}

/**
 * fastboot_fail() - Write a FAIL response of the form "FAIL$reason".
 *
 * @reason: Pointer to returned reason string
 * @response: Pointer to fastboot response buffer
 */
void fastboot_fail(const char *reason, char *response)
{
	fastboot_response("FAIL", response, "%s", reason);
}

/**
 * fastboot_okay() - Write an OKAY response of the form "OKAY$reason".
 *
 * @reason: Pointer to returned reason string, or NULL to send a bare "OKAY"
 * @response: Pointer to fastboot response buffer
 */
void fastboot_okay(const char *reason, char *response)
{
	if (reason)
		fastboot_response("OKAY", response, "%s", reason);
	else
		fastboot_response("OKAY", response, NULL);
}

/**
 * fastboot_none() - Skip the common write operation, nothing output.
 *
 * @response: Pointer to fastboot response buffer
 */
void fastboot_none_resp(char *response)
{
	*response = 0;
}

/**
 * fastboot_set_reboot_flag() - Set flag to indicate reboot-bootloader
 *
 * Set flag which indicates that we should reboot into the bootloader
 * following the reboot that fastboot executes after this function.
 *
 * This function should be overridden in your board file with one
 * which sets whatever flag your board specific Android bootloader flow
 * requires in order to re-enter the bootloader.
 */
int __weak fastboot_set_reboot_flag(enum fastboot_reboot_reason reason)
{
#ifdef CONFIG_FASTBOOT_FLASH_MMC_DEV
	u32 mmc_dev;

	static const char * const boot_cmds[] = {
		[FASTBOOT_REBOOT_REASON_BOOTLOADER] = "bootonce-bootloader",
		[FASTBOOT_REBOOT_REASON_FASTBOOTD] = "boot-fastboot",
		[FASTBOOT_REBOOT_REASON_RECOVERY] = "boot-recovery"
	};

	if (reason >= FASTBOOT_REBOOT_REASONS_COUNT)
		return -EINVAL;

	mmc_dev = (fastboot_medium_devnum() < 0) ?
		CONFIG_FASTBOOT_FLASH_MMC_DEV : fastboot_medium_devnum();

	return bcb_write_reboot_reason(mmc_dev, "misc", boot_cmds[reason]);
#else
    return -EINVAL;
#endif
}

/**
 * fastboot_get_flash_type() - get user selected flash type
 *
 * Fastboot could support many flash types, such as mmc, nand and spinand.
 * This function is used to get user selected flash type.
 *
 * Which means fastboot could indicate flash to which kind of flash.
 */
int fastboot_get_flash_type(void)
{
	return selected_flash_type;
}


/**
 * fastboot_get_progress_callback() - Return progress callback
 *
 * Return: Pointer to function called during long operations
 */
void (*fastboot_get_progress_callback(void))(const char *)
{
	return fastboot_progress_callback;
}

/**
 * fastboot_boot() - Execute fastboot boot command
 *
 * If ${fastboot_bootcmd} is set, run that command to execute the boot
 * process, if that returns, then exit the fastboot server and return
 * control to the caller.
 *
 * Otherwise execute "bootm <fastboot_buf_addr>", if that fails, reset
 * the board.
 */
void fastboot_boot(void)
{
	char *s;

	s = env_get("fastboot_bootcmd");
	if (s) {
		run_command(s, CMD_FLAG_ENV);
	} else {
		static char boot_addr_start[20];
		static char *const bootm_args[] = {
			"bootm", boot_addr_start, NULL
		};

		snprintf(boot_addr_start, sizeof(boot_addr_start) - 1,
			 "0x%p", fastboot_buf_addr);
		printf("Booting kernel at %s...\n\n\n", boot_addr_start);

		do_bootm(NULL, 0, 2, bootm_args);

		/*
		 * This only happens if image is somehow faulty so we start
		 * over. We deliberately leave this policy to the invocation
		 * of fastbootcmd if that's what's being run
		 */
		do_reset(NULL, 0, 0, NULL);
	}
}

/**
 * fastboot_set_progress_callback() - set progress callback
 *
 * @progress: Pointer to progress callback
 *
 * Set a callback which is invoked periodically during long running operations
 * (flash and erase). This can be used (for example) by the UDP transport to
 * send INFO responses to keep the client alive whilst those commands are
 * executing.
 */
void fastboot_set_progress_callback(void (*progress)(const char *msg))
{
	fastboot_progress_callback = progress;
}

/*
 * fastboot_init() - initialise new fastboot protocol session
 *
 * @buf_addr: Pointer to download buffer, or NULL for default
 * @buf_size: Size of download buffer, or zero for default
 * @medium_devnum: Medium device number(eg. mmc 0 or 1)
 * @flash_type: User selected flash type, eg. mmc/nand/spinand/ram
 */
void fastboot_init(void *buf_addr, u32 buf_size, fb_flash_type flash_type,
		s32 medium_devnum)
{
	fastboot_buf_addr = buf_addr ? buf_addr :
				       (void *)CONFIG_FASTBOOT_BUF_ADDR;
	fastboot_buf_size = buf_size ? buf_size : CONFIG_FASTBOOT_BUF_SIZE;
	fastboot_set_progress_callback(NULL);

	fastboot_medium_number = medium_devnum;
	selected_flash_type = flash_type;
}

/*
 * fastboot_medium_devnum() - get fastboot medium devnum
 *
 * @void
 * Return: flash medium devnum. (eg. mmc 0 or 1)
 * >= 0: meaningful
 * < 0: no user input
 */
s32 fastboot_medium_devnum(void)
{
	return fastboot_medium_number;
}

/*
 * fastboot_set_medium() - set fastboot flash type and devnum
 *
 * @flash_type - the medium type to be flash
 * @medium_devnum - the medium number to be flash
 */
void fastboot_set_medium(fb_flash_type flash_type, unsigned long medium_devnum)
{
	selected_flash_type = flash_type;
	fastboot_medium_number = medium_devnum;
}
