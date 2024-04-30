/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2014 Broadcom Corporation.
 */

#ifndef FB_RAM_H_
#define FB_RAM_H_

struct ram_desc;

/**
 * fastboot_ram_flash_write() - Write(Actually just memcpy) image to RAM for fastboot
 *
 * @cmd: RAM address to write image to
 * @download_buffer: Pointer to image data
 * @download_bytes: Size of image data
 * @response: Pointer to fastboot response buffer
 */
void fastboot_ram_flash_write(const char *cmd, void *download_buffer,
			      u32 download_bytes, char *response);
/**
 * fastboot_ram_flash_erase() - Erase RAM for fastboot
 *                              Actualy do nothing for RAM medium.
 *
 * @cmd: RAM address to erase
 * @response: Pointer to fastboot response buffer
 */
void fastboot_ram_erase(const char *cmd, char *response);
#endif  /* FB_RAM_H_ */
