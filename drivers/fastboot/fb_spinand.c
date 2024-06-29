// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2014 Broadcom Corporation.
 * Copyright 2015 Free Electrons.
 */

#include <config.h>
#include <common.h>

#include <fastboot.h>
#include <image-sparse.h>

#include <mtd.h>
#include <linux/mtd/mtd.h>
#include <jffs2/load_kernel.h>

/**
 * sparse_bad_blocks - stored bad blocks during sparse writting
 */
static int sparse_bad_blocks = 0;

static int _fb_spinand_erase_part(struct mtd_info *mtd,
		struct part_info *part);
static int _fb_spinand_erase_offset(struct mtd_info *mtd,
		u32 offset, size_t length);

struct fb_spinand_sparse {
	struct mtd_info		*mtd;
	struct part_info	*part;
};
#if 0
static uint mtd_len_to_pages(struct mtd_info *mtd, u64 len)
{
	do_div(len, mtd->writesize);

	return len;
}
#endif
static bool mtd_is_aligned_with_min_io_size(struct mtd_info *mtd, u64 size)
{
	return !do_div(size, mtd->writesize);
}

static bool mtd_is_aligned_with_block_size(struct mtd_info *mtd, u64 size)
{
	return !do_div(size, mtd->erasesize);
}

__weak int board_fastboot_erase_partition_setup(char *name)
{
	return 0;
}

__weak int board_fastboot_write_partition_setup(char *name)
{
	return 0;
}

static int fb_spinand_lookup(const char *partname,
			  struct mtd_info **mtd,
			  struct part_info **part,
			  char *response)
{
	struct mtd_device *dev;
	int ret;
	u8 pnum;
	const char *mtd_name = "spi-nand0";

	/* mtd device probe */
	if (mtd_probe_devices() < 0) {
		pr_err("mtd_probe_devices failed\n");
		fastboot_fail("no mtd devices", response);
		return -EINVAL;
	}

	*mtd = get_mtd_device_nm(mtd_name);
	if (IS_ERR_OR_NULL(*mtd)) {
		pr_err("MTD device %s not found, ret %ld\n",
				mtd_name, PTR_ERR(*mtd));
		fastboot_fail("no mtd devices", response);
		return -EINVAL;
	}
	put_mtd_device(*mtd);

	/* mtd part init and find */
	ret = mtdparts_init();
	if (ret) {
		pr_err("Cannot initialize MTD partitions\n");
		fastboot_fail("cannot init mtdparts", response);
<<<<<<< HEAD
		return -EINVAL;
=======
		return ret;
>>>>>>> b920437a (feat(release): LNX6.1.83_PL5.1_V1.0.7_20240617-1058)
	}

	ret = find_dev_and_part(partname, &dev, &pnum, part);
	if (ret) {
		pr_err("cannot find partition: '%s'\n", partname);
		fastboot_fail("cannot find partition", response);
<<<<<<< HEAD
		return -ENODEV;
=======
		return ret;
>>>>>>> b920437a (feat(release): LNX6.1.83_PL5.1_V1.0.7_20240617-1058)
	}

	if (dev->id->type != MTD_DEV_TYPE_SPINAND) {
		pr_err("partition '%s' is not stored on a NAND device",
		      partname);
		fastboot_fail("not a NAND device", response);
		return -EINVAL;
	}

	return 0;
}

static int _fb_spinand_erase_part(struct mtd_info *mtd, struct part_info *part)
{
	struct erase_info erase_op = {};
	u64 off, len;
	int ret;

	if (!mtd || !part)
		return -EINVAL;

	off = part->offset;
	len = part->size;

	if (!mtd_is_aligned_with_block_size(mtd, off)) {
		printf("%s(%d)Offset(0x%llx) not aligned with a block (0x%x)\n",
				__func__, __LINE__, off, mtd->erasesize);
		return -EINVAL;
	}

	if (!mtd_is_aligned_with_block_size(mtd, len)) {
		printf("%s(%d)Size(0x%llx) not a multiple of a block (0x%x)\n",
				__func__, __LINE__, len, mtd->erasesize);
		return -EINVAL;
	}

	erase_op.mtd = mtd;
	erase_op.addr = off;
	erase_op.len = len;
	erase_op.scrub = 0;

	while (erase_op.len) {
		ret = mtd_erase(mtd, &erase_op);

		/* Abort if its not a bad block error */
		if (ret != -EIO)
			break;

		printf("Skipping bad block at 0x%08llx\n",
				erase_op.fail_addr);

		/* Skip bad block and continue behind it */
		erase_op.len -= erase_op.fail_addr - erase_op.addr;
		erase_op.len -= mtd->erasesize;
		erase_op.addr = erase_op.fail_addr + mtd->erasesize;

		if (ret && ret != -EIO)
			return ret;
	}

	printf("........ erased 0x%llx bytes from '%s', offset '0x%llx'\n",
		part->size, part->name, part->offset);

	return 0;
}

static int _fb_spinand_erase_offset(struct mtd_info *mtd,
		u32 offset, size_t length)
{
	struct erase_info erase_op = {};
	u64 off, len;
	int ret;

	off = offset;
	len = length;

	if (!mtd_is_aligned_with_block_size(mtd, off)) {
		printf("%s(%d) Offset(0x%llx) not aligned with a block (0x%x)\n",
				__func__, __LINE__, off, mtd->erasesize);
		return -EINVAL;
	}

	if (!mtd_is_aligned_with_block_size(mtd, len)) {
		printf("%s(%d)Size(0x%llx) not a multiple of a block (0x%x)\n",
				__func__, __LINE__, len, mtd->erasesize);
		return -EINVAL;
	}

	erase_op.mtd = mtd;
	erase_op.addr = off;
	erase_op.len = len;
	erase_op.scrub = 0;

	while (erase_op.len) {
		ret = mtd_erase(mtd, &erase_op);

		/* Abort if its not a bad block error */
		if (ret != -EIO)
			break;

		printf("Skipping bad block at 0x%08llx\n",
				erase_op.fail_addr);

		/* Skip bad block and continue behind it */
		erase_op.len -= erase_op.fail_addr - erase_op.addr;
		erase_op.len -= mtd->erasesize;
		erase_op.addr = erase_op.fail_addr + mtd->erasesize;

		if (ret && ret != -EIO)
			return ret;
	}

	printf("........ erased 0x%llx bytes from '%s', offset '0x%llx'\n",
		len, mtd->name, off);

	return 0;
}

/* Logic taken from fs/ubifs/recovery.c:is_empty() */
static bool mtd_oob_write_is_empty(struct mtd_oob_ops *op)
{
	int i;

	for (i = 0; i < op->len; i++)
		if (op->datbuf[i] != 0xff)
			return false;

	for (i = 0; i < op->ooblen; i++)
		if (op->oobbuf[i] != 0xff)
			return false;

	return true;
}

static int mtd_special_write_oob(struct mtd_info *mtd, u64 off,
				 struct mtd_oob_ops *io_op,
				 bool write_empty_pages, bool woob)
{
	int ret = 0;

	/*
	 * By default, do not write an empty page.
	 * Skip it by simulating a successful write.
	 */
	if (!write_empty_pages && mtd_oob_write_is_empty(io_op)) {
		io_op->retlen = mtd->writesize;
		io_op->oobretlen = woob ? mtd->oobsize : 0;
	} else {
		ret = mtd_write_oob(mtd, off, io_op);
	}

	return ret;
}

static int _fb_spinand_write(struct mtd_info *mtd, struct part_info *part,
			  void *buffer, u32 offset,
			  size_t length, size_t *written)
{
	u64 remaining, off;
	int ret;

	pr_debug("%s, mtd(%p), part(%p), buffer(%p), offset(%u), length(%ld)\n",
			__func__, mtd, part, buffer, offset, length);
	if (!mtd_is_aligned_with_min_io_size(mtd, offset)) {
		printf("Offset(0x%x) not aligned with a page(0x%x)\n",
				offset, mtd->writesize);
		return -EINVAL;
	}

	if (!mtd_is_aligned_with_min_io_size(mtd, length)) {
		length = round_up(length, mtd->writesize);
		printf("Size not a page boundary (0x%x), rounding to 0x%lx\n",
				mtd->writesize, length);
	}

	bool has_pages = mtd->type == MTD_NANDFLASH ||
			mtd->type == MTD_MLCNANDFLASH;
	bool write_empty_pages = !has_pages;
	struct mtd_oob_ops io_op = {};

	/* Below is actually spinand write operation with mtd_write_oob */
	remaining = length;

	io_op.mode = MTD_OPS_AUTO_OOB;
	io_op.len = has_pages ? mtd->writesize : length;
	io_op.ooblen = 0;
	io_op.datbuf = buffer;
	io_op.oobbuf = NULL;

	/* Search for the first good block after the given offset */
	off = offset;
	while (mtd_block_isbad(mtd, off))
		off += mtd->erasesize;

	/* Loop over the pages to do the actual write */
	while (remaining) {
		/* Skip the block if it is bad */
		if (mtd_is_aligned_with_block_size(mtd, off) &&
				mtd_block_isbad(mtd, off)) {
			off += mtd->erasesize;
			continue;
		}

		ret = mtd_special_write_oob(mtd, off, &io_op,
				write_empty_pages, 0);
		if (ret) {
			printf("Failure while writing at offset 0x%llx, error(%d)\n",
					off, ret);
			return ret;
		}

		off += io_op.retlen;
		remaining -= io_op.retlen;
		io_op.datbuf += io_op.retlen;
		io_op.oobbuf += io_op.oobretlen;
	}

	/* written include the skip bad block */
	if (written)
		*written  = off - offset;

	return 0;
}

static lbaint_t fb_spinand_sparse_write(struct sparse_storage *info,
		lbaint_t blk, lbaint_t blkcnt, const void *buffer)
{
	struct fb_spinand_sparse *sparse = info->priv;
	lbaint_t actual_blkcnt;
	size_t written;
	int ret;

	ret = _fb_spinand_write(sparse->mtd, sparse->part, (void *)buffer,
			     blk * info->blksz,
			     blkcnt * info->blksz, &written);
	if (ret < 0) {
		printf("Failed to write sparse chunk\n");
		return ret;
	}

/* TODO - verify that the value "written" includes the "bad-blocks" ... */

	/*
	 * the return value must be 'blkcnt' ("good-blocks") plus the
	 * number of "bad-blocks" encountered within this space...
	 */
	actual_blkcnt = written / info->blksz;
	if (actual_blkcnt > blkcnt)	/* has bad-blocks */
		sparse_bad_blocks += (actual_blkcnt - blkcnt);
	return written / info->blksz;
}

static lbaint_t fb_spinand_sparse_reserve(struct sparse_storage *info,
		lbaint_t blk, lbaint_t blkcnt)
{
	int bad_blocks = sparse_bad_blocks;

/*
 * TODO - implement a function to determine the total number
 * of blocks which must be used in order to reserve the specified
 * number ("blkcnt") of "good-blocks", starting at "blk"...
 * ( possibly something like the "check_skip_len()" function )
 */

	/*
	 * the return value must be 'blkcnt' ("good-blocks") plus the
	 * number of "bad-blocks" encountered within this space...
	 */
	printf("%s reserve %d bad-blocks\n", __func__, bad_blocks);
	return blkcnt + bad_blocks;
}

/**
 * fastboot_spinand_get_part_info() - Lookup NAND partion by name
 *
 * @part_name: Named device to lookup
 * @part_info: Pointer to returned part_info pointer
 * @response: Pointer to fastboot response buffer
 */
int fastboot_spinand_get_part_info(const char *part_name,
		struct part_info **part_info, char *response)
{
	struct mtd_info *mtd = NULL;

	return fb_spinand_lookup(part_name, &mtd, part_info, response);
}

/**
 * fastboot_spinand_flash_write() - Write image to NAND for fastboot
 *
 * @cmd: Named device to write image to
 * @download_buffer: Pointer to image data
 * @download_bytes: Size of image data
 * @response: Pointer to fastboot response buffer
 */
void fastboot_spinand_flash_write(const char *cmd, void *download_buffer,
			       u32 download_bytes, char *response)
{
	struct part_info *part;
	static struct part_info *saved_part = NULL;	/* Only erase same part once */
	long start_addr = -1;
	static long saved_addr = -1;			/* Only erase same place once */
	struct mtd_info *mtd = NULL;
	char *s, *stringp;
	int ret = 0;

	ret = fb_spinand_lookup(cmd, &mtd, &part, response);
	if (ret) {
		if (!mtd)
			return;

		stringp = strdup(cmd);
		s = stringp;
		if ((strncmp(stringp, "addr:", 5) == 0) ||
				(strncmp(stringp, "addr_a:", 7) == 0) ||
				(strncmp(stringp, "addr_b:", 7) == 0))
			strsep(&s, ":");

		/* fallback on using the 'partition name' as a number */
		if (strict_strtoul(s, 16, (unsigned long *)&start_addr) < 0) {
			printf("cmd(%s) strict_strtoul failed\n", cmd);
			free(stringp);
			return;
		}
		free(stringp);

		printf("fastboot spinand flash start_addr: 0x%lx\n", start_addr);
	}

	if (start_addr == -1 && part) {
		ret = board_fastboot_write_partition_setup(part->name);
		if (ret)
			return;
	}

	/* nand flash need erase first, then write */
	if (start_addr == -1 && part) {
		/* same part only erase once  */
		if (saved_part != part) {
			printf("erase part (%s) from 0x%llx to 0x%llx\n",
					part->name, part->offset, part->size);
			ret = _fb_spinand_erase_part(mtd, part);
			saved_part = part;

			if (ret) {
				pr_err("erase spinand partition failed, ret(%d)\n", ret);
				fastboot_fail("erase spinand partition failed", response);
				return;
			}
		}
	} else if (saved_addr != start_addr) {
		/* same start_addr, only erase once */
		printf("erase from 0x%lx to %llx\n", start_addr,
				mtd->size - start_addr);
		ret = _fb_spinand_erase_offset(mtd, start_addr,
				mtd->size - start_addr);
		saved_addr = start_addr;

		if (ret) {
			pr_err("erase spinand partition failed, ret(%d)\n", ret);
			fastboot_fail("erase spinand partition failed", response);
			return;
		}
	}

	printf("begin to write data to spinand flash\n");
	if (is_sparse_image(download_buffer)) {
		struct fb_spinand_sparse sparse_priv;
		struct sparse_storage sparse;

		sparse_priv.mtd = mtd;

		if (start_addr == -1 && part) {
			sparse_priv.part = part;
			sparse.blksz = mtd->writesize;
			sparse.start = part->offset / sparse.blksz;
			sparse.size = part->size / sparse.blksz;
		} else {
			sparse_priv.part = NULL;
			sparse.blksz = mtd->writesize;
			sparse.start = start_addr;
			sparse.size  = mtd->size;
		}
		sparse.write = fb_spinand_sparse_write;
		sparse.reserve = fb_spinand_sparse_reserve;
		sparse.mssg = fastboot_fail;

		printf("Flashing sparse image at offset " LBAFU "\n",
		       sparse.start);

		sparse.priv = &sparse_priv;
		ret = write_sparse_image(&sparse, cmd, download_buffer,
					 response);
		if (!ret)
			fastboot_okay(NULL, response);
	} else {
		printf("Flashing raw image at offset 0x%llx\n",
		       (start_addr == -1 && part) ? part->offset : start_addr);

		if (start_addr == -1 && part) {
			ret = _fb_spinand_write(mtd, part, download_buffer, part->offset,
					     download_bytes, NULL);

			printf("........ wrote %u bytes to '%s'\n",
			       download_bytes, part->name);
		} else {
			ret = _fb_spinand_write(mtd, NULL, download_buffer, start_addr,
					     download_bytes, NULL);

			printf("........ wrote %u bytes to '0x%lx'\n",
			       download_bytes, start_addr);
		}
	}

	if (ret) {
		fastboot_fail("error writing the image", response);
		return;
	}

	fastboot_okay(NULL, response);
}

/**
 * fastboot_spinand_erase() - Erase NAND for fastboot
 *
 * @cmd: Named device to erase
 * @response: Pointer to fastboot response buffer
 */
void fastboot_spinand_erase(const char *cmd, char *response)
{
	struct part_info *part;
	struct mtd_info *mtd = NULL;
	int ret;

	ret = fb_spinand_lookup(cmd, &mtd, &part, response);
	if (ret)
		return;

	ret = board_fastboot_erase_partition_setup(part->name);
	if (ret)
		return;

	ret = _fb_spinand_erase_part(mtd, part);
	if (ret) {
		pr_err("failed erasing from device %s\n", mtd->name);
		fastboot_fail("failed erasing from device", response);
		return;
	}

	fastboot_okay(NULL, response);
}
