// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2014 Broadcom Corporation.
 */
#include <blk.h>
#include <errno.h>
#include <common.h>
#include <fastboot-internal.h>
#include <fastboot.h>
#include <image-sparse.h>
#include <div64.h>
#include <linux/compat.h>

#define FASTBOOT_MAX_BLK_WRITE 0x40000		// 256K blocks with 4bytes blk_size, totally 1M data
#define FB_RAM_BLKSZ 4

struct ram_desc {
	lbaint_t	blksz;
};

struct fb_ram_sparse {
	struct ram_desc *dev_desc;
};

/**
 * fb_ram_blk_write() - Write/erase RAM in chunks of FASTBOOT_MAX_BLK_WRITE
 *
 * @block_dev: Pointer to block device
 * @start: First block to write/erase
 * @blkcnt: Count of blocks
 * @buffer: Pointer to data buffer for write or NULL for erase
 */
static lbaint_t fb_ram_blk_write(struct ram_desc *ramdev,
			lbaint_t start, lbaint_t blkcnt, const void *buffer)
{
	lbaint_t blk = start;
	lbaint_t cur_blkcnt;
	lbaint_t blks = 0;
	int32_t i;

	if (!buffer)
		return -EINVAL;

	for (i = 0; i < blkcnt; i += FASTBOOT_MAX_BLK_WRITE) {
		cur_blkcnt = min((int)blkcnt - i, FASTBOOT_MAX_BLK_WRITE);

		if (fastboot_progress_callback)
			fastboot_progress_callback("writting");

		memcpy((void *)(blk * ramdev->blksz), buffer + (i * ramdev->blksz),
					cur_blkcnt * ramdev->blksz);

		blk += cur_blkcnt;
		blks += cur_blkcnt;
	}

	return blks;
}

static lbaint_t fb_ram_sparse_write(struct sparse_storage *info,
			lbaint_t blk, lbaint_t blkcnt, const void *buffer)
{
	struct fb_ram_sparse *sparse = info->priv;
	struct ram_desc *dev_desc = sparse->dev_desc;

	return fb_ram_blk_write(dev_desc, blk, blkcnt, buffer);
}

static lbaint_t fb_ram_sparse_reserve(struct sparse_storage *info,
		lbaint_t blk, lbaint_t blkcnt)
{
	return blkcnt;
}


/**
 * write_raw_image_to_addr - write raw image to addr
 */
static void write_raw_image_to_addr(struct ram_desc *dev_desc, uintptr_t addr,
		int blksz, void *buffer, u32 download_bytes, char *response)
{
	lbaint_t blkcnt;
	lbaint_t blks;
	lbaint_t start;

	if (!IS_ALIGNED(addr, 4)) {
		pr_err("flash address must be aligned with DWORD. addr(0x%lx)\n", addr);
		fastboot_fail("flash address must be aligned with DWORD.\n", response);
		return;
	}

	/* determine number of blocks to write */
	blkcnt = ((download_bytes + (blksz - 1)) & ~(blksz - 1));
	blkcnt = lldiv(blkcnt, blksz);
	start = addr / blksz;

	puts("Flashing Raw Image\n");

	blks = fb_ram_blk_write(dev_desc, start, blkcnt, buffer);
	if (blks != blkcnt) {
		pr_err("failed writing to ram 0x%lx\n", blkcnt * blksz);
		fastboot_fail("failed writing to ram", response);
		return;
	}

	printf("....... wrote " LBAFU " bytes to ram 0x%lx\n",
			blkcnt * blksz, addr);
	fastboot_okay(NULL, response);
}

/**
 * fastboot_ram_flash_write() - Write(Actually just memcpy) image to ram for fastboot
 *
 * @cmd: Destination RAM address to write image to
 * @download_buffer: Pointer to image data
 * @download_bytes: Size of image data
 * @response: Pointer to fastboot response buffer
 */
void fastboot_ram_flash_write(const char *cmd, void *download_buffer,
			      u32 download_bytes, char *response)
{
	struct ram_desc ramdev;
	uintptr_t dest_addr;

	if (strict_strtoul(cmd, 16, &dest_addr) < 0) {
		pr_err("invalid destination address: '%s'\n", cmd);
		return;
	}

	printf("fastboot ram flash dest_addr: 0x%lx\n", dest_addr);

	ramdev.blksz = FB_RAM_BLKSZ;
	if (is_sparse_image(download_buffer)) {
		struct fb_ram_sparse sparse_priv;
		struct sparse_storage sparse;
		int err;

		sparse_priv.dev_desc = &ramdev;
		sparse.blksz = FB_RAM_BLKSZ;		// ram medium, we just need dword alignment
		sparse.start = dest_addr;
		sparse.size = 0;

		sparse.write = fb_ram_sparse_write;
		sparse.reserve = fb_ram_sparse_reserve;
		sparse.mssg = fastboot_fail;

		printf("Flashing sparse image at offset " LBAFU "\n",
				sparse.start);

		sparse.priv = NULL;		// No need private data
		err = write_sparse_image(&sparse, cmd, download_buffer,
					response);

		if (!err)
			fastboot_okay(NULL, response);
	} else {
		write_raw_image_to_addr(&ramdev, dest_addr, FB_RAM_BLKSZ,
				download_buffer, download_bytes, response);
	}
}

/**
 * fastboot_ram_flash_erase() - Erase Ram for fastboot
 *
 * @cmd: Destination RAM address to erase    # actually do nonthing.
 * @response: Pointer to fastboot response buffer
 */
void fastboot_ram_erase(const char *cmd, char *response)
{
	// DO NOTHING... for ram medium

	printf("%s do nothing for cmd '%s' \n", __func__, cmd);
	fastboot_okay(NULL, response);

	return;
}
