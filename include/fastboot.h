/* SPDX-License-Identifier: GPL-2.0+ */
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
#ifndef _FASTBOOT_H_
#define _FASTBOOT_H_

#define FASTBOOT_VERSION	"0.4"

/* The 64 defined bytes plus \0 */
#define FASTBOOT_COMMAND_LEN	(64 + 1)
#define FASTBOOT_RESPONSE_LEN	(64 + 1)

/**
 * All known commands to fastboot
 */
enum {
	FASTBOOT_COMMAND_GETVAR = 0,
	FASTBOOT_COMMAND_DOWNLOAD,
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
	FASTBOOT_COMMAND_FLASH,
	FASTBOOT_COMMAND_ERASE,
#endif
	FASTBOOT_COMMAND_BOOT,
	FASTBOOT_COMMAND_CONTINUE,
	FASTBOOT_COMMAND_REBOOT,
	FASTBOOT_COMMAND_REBOOT_BOOTLOADER,
	FASTBOOT_COMMAND_REBOOT_FASTBOOTD,
	FASTBOOT_COMMAND_REBOOT_RECOVERY,
	FASTBOOT_COMMAND_SET_ACTIVE,
#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_FORMAT)
	FASTBOOT_COMMAND_OEM_FORMAT,
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_PARTCONF)
	FASTBOOT_COMMAND_OEM_PARTCONF,
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_BOOTBUS)
	FASTBOOT_COMMAND_OEM_BOOTBUS,
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_RAMDUMP)
	FASTBOOT_COMMAND_OEM_RAMDUMP,
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_SET_MEDIUM)
	FASTBOOT_COMMAND_OEM_SET_MEDIUM,
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_UUU_SUPPORT)
	FASTBOOT_COMMAND_ACMD,
	FASTBOOT_COMMAND_UCMD,
#endif

	FASTBOOT_COMMAND_COUNT
};

/**
 * flash type enums
 */
typedef enum {
	FLASH_TYPE_UNKNOWN = -1,
	FLASH_TYPE_EMMC,
	FLASH_TYPE_NAND,
	FLASH_TYPE_SPINAND,
	FLASH_TYPE_RAM,
	FLASH_TYPE_COUNT
} fb_flash_type;

/**
 * Reboot reasons
 */
enum fastboot_reboot_reason {
	FASTBOOT_REBOOT_REASON_BOOTLOADER,
	FASTBOOT_REBOOT_REASON_FASTBOOTD,
	FASTBOOT_REBOOT_REASON_RECOVERY,
	FASTBOOT_REBOOT_REASONS_COUNT
};

/**
 * fastboot_response() - Writes a response of the form "$tag$reason".
 *
 * @tag: The first part of the response
 * @response: Pointer to fastboot response buffer
 * @format: printf style format string
 */
void fastboot_response(const char *tag, char *response,
		       const char *format, ...)
	__attribute__ ((format (__printf__, 3, 4)));

/**
 * fastboot_fail() - Write a FAIL response of the form "FAIL$reason".
 *
 * @reason: Pointer to returned reason string
 * @response: Pointer to fastboot response buffer
 */
void fastboot_fail(const char *reason, char *response);

/**
 * fastboot_okay() - Write an OKAY response of the form "OKAY$reason".
 *
 * @reason: Pointer to returned reason string, or NULL to send a bare "OKAY"
 * @response: Pointer to fastboot response buffer
 */
void fastboot_okay(const char *reason, char *response);

/**
 * fastboot_none_resp() - Skip the common write operation, nothing output.
 *
 * @response: Pointer to fastboot response buffer
 */
void fastboot_none_resp(char *response);

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
int fastboot_set_reboot_flag(enum fastboot_reboot_reason reason);
/**
 * fastboot_get_flash_type() - get user selected flash type
 *
 * Fastboot could support many flash types, such as mmc, nand and spinand.
 * This function is used to get user selected flash type.
 *
 * Which means fastboot could indicate flash to which kind of flash.
 */
int fastboot_get_flash_type(void);

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
void fastboot_set_progress_callback(void (*progress)(const char *msg));

/*
 * fastboot_init() - initialise new fastboot protocol session
 *
 * @buf_addr: Pointer to download buffer, or NULL for default
 * @buf_size: Size of download buffer, or zero for default
 * @medium_devnum: Medium device number(eg. mmc 0 or 1)
 * @flash_type: User selected flash type, eg. mmc/nand/spinand/ram
 */
void fastboot_init(void *buf_addr, u32 buf_size, fb_flash_type flash_type,
		s32 medium_devnum);

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
void fastboot_boot(void);

/**
 * fastboot_handle_command() - Handle fastboot command
 *
 * @cmd_string: Pointer to command string
 * @response: Pointer to fastboot response buffer
 *
 * Return: Executed command, or -1 if not recognized
 */
int fastboot_handle_command(char *cmd_string, char *response);

/**
 * fastboot_download_remaining() - return bytes remaining in current transfer
 *
 * Return: Number of bytes left in the current download
 */
u32 fastboot_download_remaining(void);

/**
 * fastboot_upload_remaining() - return bytes remaining in current transfer
 *
 * Return: Number of bytes left in the current upload
 */
u32 fastboot_upload_remaining(void);

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
 */
void fastboot_data_download(const void *fastboot_data,
			    unsigned int fastboot_data_len, char *response);

/**
 * fastboot_data_upload() - Copy specify data to fastboot_data.
 *
 * @fastboot_data: Pointer to send fastboot data
 * @fastboot_data_len: Length of send fastboot data
 * @response: Pointer to fastboot response buffer
 *
 * Copies specify data(Only support ramdump currently) to fastboot_data.
 * Writes to response. fastboot_bytes_send is updated to indicate the number
 * of bytes that have been transferred.
 */
void fastboot_data_upload(void *fastboot_data,
			    unsigned int fastboot_data_len, char *response);
/**
 * fastboot_download_complete() - Mark current transfer complete
 *
 * @response: Pointer to fastboot response buffer
 *
 * Set image_size and ${filesize} to the total size of the downloaded image.
 */
void fastboot_download_complete(char *response);

/**
 * fastboot_tx_write() - fastboot tx write data
 *
 * @buffer: buffer to be send
 * @buffer_size: buffer size
 *
 * fastboot tx transfer data through usb bulk-ep1
 * (fastboot_func->in_ep)
 */
int fastboot_tx_write(const char *buffer, unsigned int buffer_size);

/**
 * fastboot_tx_write_more() - fastboot tx write more buffer
 *
 * @buffer: string buffer to more fastboot response
 *
 * fastboot_tx_write(_str) use const bulk-ep1 to transfer data,
 * fastboot_tx_write_more will alloc new urb/trb to transfer more
 * buffer.
 */
int fastboot_tx_write_more(const char *buffer);

/**
 * fastboot_upload_ramdump() - wrapper function for upload ramdump to host
 *
 * wrapper function to upload requested ram data to host pc,
 * used for oem ramdump feature..
 */
void fastboot_upload_ramdump(void);

/**
 * fastboot_upload_complete() - Mark current transfer complete
 *
 * @response: Pointer to fastboot response buffer
 *
 * Mark current upload transfer complete, and reset global transfer info.
 */
void fastboot_upload_complete(char *response);


#if CONFIG_IS_ENABLED(FASTBOOT_UUU_SUPPORT)
void fastboot_acmd_complete(void);
#endif

/*
 * fastboot_set_medium() - set fastboot flash type and devnum
 *
 * @flash_type - the medium type to be flash
 * @medium_devnum - the medium number to be flash
 */
void fastboot_set_medium(fb_flash_type flash_type, unsigned long medium_devnum);

/*
 * fastboot_medium_devnum() - get fastboot medium devnum
 *
 * @void
 * Return: flash medium devnum. (eg. mmc 0 or 1)
 * >= 0: meaningful
 * < 0: no user input
 */
s32 fastboot_medium_devnum(void);

#endif /* _FASTBOOT_H_ */
