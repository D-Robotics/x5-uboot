// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 Horizon Robotics Co., Ltd
 */
#include <avb_verify.h>
#include <blk.h>
#include <fastboot.h>
#include <image.h>
#include <malloc.h>
#include <part.h>
#include <tee.h>
#include <tee/optee_ta_avb.h>
#include <mtd.h>
#include <linux/mtd/mtd.h>
#include <scsi.h>
#include <dm/uclass.h>
#include <dm/device-internal.h>
#include <cpu_func.h>
#include <nvme.h>

struct blk_avb_info {
	struct AvbOpsData ops_data;
	struct hb_avb_if_info *info;
	const char *if_typename;
};
struct blk_part {
	int dev_num;
	struct blk_desc *blk_desc;
	struct disk_partition info;
};

#define OPS_TO_INFO(ops)					\
	container_of(ops, struct blk_avb_info, ops_data.ops)

/**
 * ============================================================================
 * IO(mmc) auxiliary functions
 * ============================================================================
 */
static unsigned long blk_read_and_flush(struct blk_part *part,
					lbaint_t start,
					lbaint_t sectors,
					void *buffer)
{
	unsigned long blks;
	void *tmp_buf;
	size_t buf_size;
	bool unaligned = is_buf_unaligned(buffer);

	if (start < part->info.start) {
		printf("%s: partition start out of bounds\n", __func__);
		return 0;
	}
	if ((start + sectors) > (part->info.start + part->info.size)) {
		sectors = part->info.start + part->info.size - start;
		printf("%s: read sector aligned to partition bounds (%ld)\n",
		       __func__, sectors);
	}

	/*
	 * Reading fails on unaligned buffers, so we have to
	 * use aligned temporary buffer and then copy to destination
	 */

	if (unaligned) {
		printf("Handling unaligned read buffer..\n");
		tmp_buf = get_sector_buf();
		buf_size = get_sector_buf_size();
		if (sectors > buf_size / part->info.blksz)
			sectors = buf_size / part->info.blksz;
	} else {
		tmp_buf = buffer;
	}

	blks = blk_dread(part->blk_desc,
			 start, sectors, tmp_buf);
	/* flush cache after read */
	flush_cache((ulong)tmp_buf, sectors * part->info.blksz);

	if (unaligned)
		memcpy(buffer, tmp_buf, sectors * part->info.blksz);

	return blks;
}

static unsigned long blk_write(struct blk_part *part, lbaint_t start,
			       lbaint_t sectors, void *buffer)
{
	void *tmp_buf;
	size_t buf_size;
	bool unaligned = is_buf_unaligned(buffer);

	if (start < part->info.start) {
		printf("%s: partition start out of bounds\n", __func__);
		return 0;
	}
	if ((start + sectors) > (part->info.start + part->info.size)) {
		sectors = part->info.start + part->info.size - start;
		printf("%s: sector aligned to partition bounds (%ld)\n",
		       __func__, sectors);
	}
	if (unaligned) {
		tmp_buf = get_sector_buf();
		buf_size = get_sector_buf_size();
		printf("Handling unaligned wrire buffer..\n");
		if (sectors > buf_size / part->info.blksz)
			sectors = buf_size / part->info.blksz;

		memcpy(tmp_buf, buffer, sectors * part->info.blksz);
	} else {
		tmp_buf = buffer;
	}

	return blk_dwrite(part->blk_desc,
			  start, sectors, tmp_buf);
}

static struct blk_part *get_partition(AvbOps *ops, const char *partition)
{
	int ret;
	u8 dev_num;
	struct blk_part *part;
	struct blk_desc *desc;
	struct blk_avb_info *info = OPS_TO_INFO(ops);

	part = malloc(sizeof(struct blk_part));
	if (!part)
		return NULL;

	dev_num = get_boot_device(ops);

	desc = blk_get_dev(info->if_typename, dev_num);
	if (!desc) {
		printf("Can't find dev_num '%d' blk %p\n", dev_num, desc);
		goto err;
	}

	ret = part_get_info_by_name(desc, partition, &part->info);
	if (ret < 0) {
		printf("Can't find partition '%s'\n", partition);
		goto err;
	}

	part->dev_num = dev_num;
	part->blk_desc = desc;
	return part;
err:
	free(part);
	return NULL;
}

static AvbIOResult blk_byte_io(AvbOps *ops,
			       const char *partition,
			       s64 offset,
			       size_t num_bytes,
			       void *buffer,
			       size_t *out_num_read,
			       enum mmc_io_type io_type)
{
	ulong ret;
	struct blk_part *part;
	u64 start_offset, start_sector, sectors, residue, part_size;
	u8 *tmp_buf;
	size_t io_cnt = 0;

	if (!partition || !buffer || io_type > IO_WRITE)
		return AVB_IO_RESULT_ERROR_IO;

	part = get_partition(ops, partition);
	if (!part)
		return AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION;

	if (!part->info.blksz)
		return AVB_IO_RESULT_ERROR_IO;

	part_size = part->info.size * part->info.blksz;
	start_offset = offset < 0 ? part_size + offset : offset;
	if (start_offset + num_bytes > part_size)
		return AVB_IO_RESULT_ERROR_INVALID_VALUE_SIZE;

	while (num_bytes) {
		start_sector = start_offset / part->info.blksz;
		sectors = num_bytes / part->info.blksz;
		/* handle non block-aligned reads */
		if (start_offset % part->info.blksz ||
		    num_bytes < part->info.blksz) {
			tmp_buf = get_sector_buf();
			if (start_offset % part->info.blksz) {
				residue = part->info.blksz -
					(start_offset % part->info.blksz);
				if (residue > num_bytes)
					residue = num_bytes;
			} else {
				residue = num_bytes;
			}

			if (io_type == IO_READ) {
				ret = blk_read_and_flush(part,
							 part->info.start +
							 start_sector,
							 1, tmp_buf);

				if (ret != 1) {
					printf("%s: read error (%ld, %lld)\n",
					       __func__, ret, start_sector);
					return AVB_IO_RESULT_ERROR_IO;
				}
				/*
				 * if this is not aligned at sector start,
				 * we have to adjust the tmp buffer
				 */
				tmp_buf += (start_offset % part->info.blksz);
				memcpy(buffer, (void *)tmp_buf, residue);
			} else {
				ret = blk_read_and_flush(part,
							 part->info.start +
							 start_sector,
							 1, tmp_buf);

				if (ret != 1) {
					printf("%s: read error (%ld, %lld)\n",
					       __func__, ret, start_sector);
					return AVB_IO_RESULT_ERROR_IO;
				}
				memcpy((void *)tmp_buf +
					start_offset % part->info.blksz,
					buffer, residue);

				ret = blk_write(part, part->info.start +
						start_sector, 1, tmp_buf);
				if (ret != 1) {
					printf("%s: write error (%ld, %lld)\n",
					       __func__, ret, start_sector);
					return AVB_IO_RESULT_ERROR_IO;
				}
			}

			io_cnt += residue;
			buffer += residue;
			start_offset += residue;
			num_bytes -= residue;
			continue;
		}

		if (sectors) {
			if (io_type == IO_READ) {
				ret = blk_read_and_flush(part,
							 part->info.start +
							 start_sector,
							 sectors, buffer);
			} else {
				ret = blk_write(part,
						part->info.start +
						start_sector,
						sectors, buffer);
			}

			if (!ret || IS_ERR_VALUE(ret)) {
				printf("%s: sector read error\n", __func__);
				return AVB_IO_RESULT_ERROR_IO;
			}

			io_cnt += ret * part->info.blksz;
			buffer += ret * part->info.blksz;
			start_offset += ret * part->info.blksz;
			num_bytes -= ret * part->info.blksz;
		}
	}

	/* Set counter for read operation */
	if (io_type == IO_READ && out_num_read)
		*out_num_read = io_cnt;

	return AVB_IO_RESULT_OK;
}

/**
 * read_from_partition() - reads @num_bytes from  @offset from partition
 * identified by a string name
 *
 * @ops: contains AVB ops handlers
 * @partition_name: partition name, NUL-terminated UTF-8 string
 * @offset: offset from the beginning of partition
 * @num_bytes: amount of bytes to read
 * @buffer: destination buffer to store data
 * @out_num_read:
 *
 * @return:
 *      AVB_IO_RESULT_OK, if partition was found and read operation succeed
 *      AVB_IO_RESULT_ERROR_IO, if i/o error occurred from the underlying i/o
 *            subsystem
 *      AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION, if there is no partition with
 *      the given name
 */
static AvbIOResult read_from_partition(AvbOps *ops,
				       const char *partition_name,
				       s64 offset_from_partition,
				       size_t num_bytes,
				       void *buffer,
				       size_t *out_num_read)
{
	return blk_byte_io(ops, partition_name, offset_from_partition,
			   num_bytes, buffer, out_num_read, IO_READ);
}

/**
 * write_to_partition() - writes N bytes to a partition identified by a string
 * name
 *
 * @ops: AvbOps, contains AVB ops handlers
 * @partition_name: partition name
 * @offset_from_partition: offset from the beginning of partition
 * @num_bytes: amount of bytes to write
 * @buf: data to write
 * @out_num_read:
 *
 * @return:
 *      AVB_IO_RESULT_OK, if partition was found and read operation succeed
 *      AVB_IO_RESULT_ERROR_IO, if input/output error occurred
 *      AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION, if partition, specified in
 *            @partition_name was not found
 */
static AvbIOResult write_to_partition(AvbOps *ops,
				      const char *partition_name,
				      s64 offset_from_partition,
				      size_t num_bytes,
				      const void *buffer)
{
	return blk_byte_io(ops, partition_name, offset_from_partition,
			   num_bytes, (void *)buffer, NULL, IO_WRITE);
}

/**
 * get_unique_guid_for_partition() - gets the GUID for a partition identified
 * by a string name
 *
 * @ops: contains AVB ops handlers
 * @partition: partition name (NUL-terminated UTF-8 string)
 * @guid_buf: buf, used to copy in GUID string. Example of value:
 *      527c1c6d-6361-4593-8842-3c78fcd39219
 * @guid_buf_size: @guid_buf buffer size
 *
 * @return:
 *      AVB_IO_RESULT_OK, on success (GUID found)
 *      AVB_IO_RESULT_ERROR_IO, if incorrect buffer size (@guid_buf_size) was
 *             provided
 *      AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION, if partition was not found
 */
static AvbIOResult get_unique_guid_for_partition(AvbOps *ops,
						 const char *partition,
						 char *guid_buf,
						 size_t guid_buf_size)
{
	struct blk_part *part;
	size_t uuid_size;

	part = get_partition(ops, partition);
	if (!part)
		return AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION;

	uuid_size = sizeof(part->info.uuid);
	if (uuid_size > guid_buf_size)
		return AVB_IO_RESULT_ERROR_IO;

	memcpy(guid_buf, part->info.uuid, uuid_size);
	guid_buf[uuid_size - 1] = 0;

	return AVB_IO_RESULT_OK;
}

/**
 * get_size_of_partition() - gets the size of a partition identified
 * by a string name
 *
 * @ops: contains AVB ops handlers
 * @partition: partition name (NUL-terminated UTF-8 string)
 * @out_size_num_bytes: returns the value of a partition size
 *
 * @return:
 *      AVB_IO_RESULT_OK, on success (GUID found)
 *      AVB_IO_RESULT_ERROR_INSUFFICIENT_SPACE, out_size_num_bytes is NULL
 *      AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION, if partition was not found
 */
static AvbIOResult get_size_of_partition(AvbOps *ops,
					 const char *partition,
					 u64 *out_size_num_bytes)
{
	struct blk_part *part;

	if (!out_size_num_bytes)
		return AVB_IO_RESULT_ERROR_INSUFFICIENT_SPACE;

	part = get_partition(ops, partition);
	if (!part)
		return AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION;

	*out_size_num_bytes = part->info.blksz * part->info.size;

	return AVB_IO_RESULT_OK;
}

static inline struct AvbOpsData *set_ops(const char *if_typename, enum if_type type)
{
	struct blk_avb_info *info;

	info = avb_calloc(sizeof(*info));
	if (!info)
		return NULL;

	info->ops_data.if_type = type;
	info->if_typename = if_typename;
	printf("%s info->if_typename %s if_typename %s\n",__func__,info->if_typename, if_typename);
	info->ops_data.ops.read_from_partition = read_from_partition;
	info->ops_data.ops.write_to_partition = write_to_partition;
	info->ops_data.ops.get_unique_guid_for_partition =
		get_unique_guid_for_partition;
	info->ops_data.ops.get_size_of_partition = get_size_of_partition;

	return &info->ops_data;
}

#ifdef CONFIG_SCSI
static int ufs_avb_opsdata_alloc(struct AvbOpsData **ops,
				const char *if_typename, int boot_device)
{
	int ret;

	if (strcmp(if_typename, "scsi")) {
		return -EINVAL;
	}

	/* Ensure all devices (and their partitions) are probed */
	ret = scsi_scan(true);
	if (ret){
		printf("Failed to get ufs device\n");
		return -ENODEV;
	}

	*ops = set_ops("scsi", IF_TYPE_SCSI);

	return 0;
}

HB_AVB_IF_INFO(ufs, ufs_avb_opsdata_alloc);
#endif

#ifdef CONFIG_NVME
static int nvme_avb_opsdata_alloc(struct AvbOpsData **ops,
				const char *if_typename, int boot_device)
{
	int ret;

	if (strcmp(if_typename, "nvme")) {
		return -EINVAL;
	}

	/* Ensure all devices (and their partitions) are probed */
	ret = nvme_scan_namespace();
	if (ret){
		printf("Failed to get nvme device\n");
		return -ENODEV;
	}

	*ops = set_ops("nvme", IF_TYPE_NVME);

	return 0;
}

HB_AVB_IF_INFO(nvme, nvme_avb_opsdata_alloc);
#endif
