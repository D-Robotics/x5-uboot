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
#include <ubi_uboot.h>

struct mtd_avb_info {
	struct AvbOpsData ops_data;
	struct hb_avb_if_info *info;
	struct mtd_info *mtd;
};

enum flash_io_type {
	FLASH_IO_READ,
	FLASH_IO_WRITE
};

#define OPS_TO_INFO(ops)					\
	container_of(ops, struct mtd_avb_info, ops_data.ops)

/**
 * ============================================================================
 * AVB 2.0 operations
 * ============================================================================
 */

// static struct mtd_info *mtd_get_partition_level(
// 	struct mtd_info *mtd, const char *partition, int level)
// {
// 	struct mtd_info *part;
// 	struct mtd_info *out;

// 	list_for_each_entry(part, &mtd->partitions, node) {
// 		if (!strcmp(part->name, partition))
// 			return part;

// 		out = mtd_get_partition_level(part, partition, level + 1);
// 		if (out)
// 			return out;
// 	}

// 	return NULL;
// }

// static struct mtd_info *mtd_get_partition(struct mtd_info **mtd,
// 					const char *partition)
// {
// 	if (!mtd || !(*mtd)) {
// 		/* Ensure all devices (and their partitions) are probed */
// 		mtd_probe_devices();

// 		*mtd = get_mtd_device(NULL, 0);
// 		if (IS_ERR_OR_NULL(*mtd)) {
// 			*mtd = NULL;
// 			pr_err("Failed to get mtd device\n");
// 			return NULL;
// 		}
// 		put_mtd_device(*mtd);
// 	}

// 	return mtd_get_partition_level(*mtd, partition, 1);
// }

// static AvbIOResult mtd_byte_io(AvbOps *ops,
// 			const char *partition,
// 			s64 offset,
// 			size_t num_bytes,
// 			void *buffer,
// 			size_t *out_num_read,
// 			enum flash_io_type io_type)
// {
// 	struct mtd_avb_info *info = OPS_TO_INFO(ops);
// 	struct mtd_info *part = mtd_get_partition(&info->mtd, partition);
// 	u64 start_offset;
// 	int ret;

// 	if (!partition || !buffer || io_type > FLASH_IO_WRITE)
// 		return AVB_IO_RESULT_ERROR_IO;

// 	if (!part)
// 		return AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION;

// 	start_offset = offset < 0 ? part->size + offset : offset;
// 	if (io_type == FLASH_IO_READ) {
// 		ret = mtd_read(part, start_offset, num_bytes, out_num_read, buffer);
// 	} else {
// 		ret = mtd_write(part, start_offset, num_bytes, out_num_read, buffer);
// 	}

// 	if (ret)
// 		return AVB_IO_RESULT_ERROR_IO;

// 	return AVB_IO_RESULT_OK;
// }

static uint64_t ubi_read_and_flush(struct ubi_volume *part,
					lbaint_t start,
					lbaint_t len,
					void *buffer)
{
	lbaint_t total_len = len;
	uint64_t ret = 0;

	ret = ubi_volume_read(part->name, buffer, len);
	if (ret) {
		pr_err("UBI read %s failed!", part->name);
		return ret;
	}
	if ((start + len) > part->used_bytes) {
		pr_debug("Attempting to read beyond boundary, stop at boundary\n");
		len = part->used_bytes - start;
	}
	memmove(buffer, buffer + start, len);
	return total_len;
}

static uint64_t ubi_write(struct ubi_volume *part, lbaint_t start,
			       lbaint_t len, void *buffer)
{
	uint64_t ret = 0, vol_size = 0;
	void *tmp_buf;

	tmp_buf = (void *) malloc(part->used_bytes * sizeof(char));
	if (tmp_buf == NULL) {
		pr_err("avb nand malloc %llu Bytes failed!\n",
				part->used_bytes * sizeof(char));
		return -1;
	}
	ret = ubi_volume_read(part->name, tmp_buf, len);
	if (ret) {
		pr_err("UBI Volume %s access failed!\n", part->name);
		return -1;
	}
	vol_size = part->reserved_pebs * (part->ubi->leb_size - part->data_pad);
	if ((start + len) > vol_size) {
		pr_debug("Attempting to write outside of boundary, stops at boundary.\n");
		len = vol_size - start;
	}

	memcpy(tmp_buf + start, buffer, len);
	ret = ubi_volume_write(part->name, tmp_buf, len);

	return ret;
}

static struct ubi_volume *ubi_get_partition(const char *partition)
{
	char *p;
	char vol_name[64] = {0};

	if (!strstr(partition, "vbmeta")) {
		if (ubi_part((char *)partition, NULL))
			return NULL;
	}
	snprintf(vol_name, sizeof(vol_name), "%s", partition);
	if ((p = strstr(vol_name, "_a")) != NULL ||
		(p = strstr(vol_name, "_b")) != NULL) {
		*p = '\0';
	}

	return ubi_find_volume(vol_name);
}

static AvbIOResult ubi_byte_io(AvbOps *ops,
			       const char *partition,
			       s64 offset,
			       size_t num_bytes,
			       void *buffer,
			       size_t *out_num_read,
			       enum flash_io_type io_type)
{
	ulong ret;
	u64 start_offset;
	size_t io_cnt = 0;
	struct ubi_volume *part = NULL;

	if (!partition || !buffer || io_type > FLASH_IO_WRITE)
		return AVB_IO_RESULT_ERROR_IO;

	part = ubi_get_partition(partition);
	if (part == NULL)
		return AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION;

	start_offset = offset < 0 ? part->used_bytes + offset : offset;
	if (io_type == FLASH_IO_READ) {
		ret = ubi_read_and_flush(part,
						start_offset,
						num_bytes, buffer);
	} else {
		ret = ubi_write(part,
						start_offset,
						num_bytes,  buffer);
	}

	io_cnt += ret;

	/* Set counter for read operation */
	if (io_type == FLASH_IO_READ && out_num_read)
		*out_num_read = io_cnt;

	return AVB_IO_RESULT_OK;
}

// static AvbIOResult mtd_get_size_of_partition(AvbOps *ops, const char *partition,
//                                          u64 *out_size_num_bytes)
// {
// 	struct mtd_avb_info *info = OPS_TO_INFO(ops);
// 	struct mtd_info *part = mtd_get_partition(&info->mtd, partition);

// 	if (!out_size_num_bytes)
// 		return AVB_IO_RESULT_ERROR_INSUFFICIENT_SPACE;

// 	if (!part)
// 		return AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION;

// 	*out_size_num_bytes = part->size;

// 	return AVB_IO_RESULT_OK;
// }

static AvbIOResult ubi_get_size_of_partition(AvbOps *ops, const char *partition,
                                         u64 *out_size_num_bytes)
{
	struct ubi_volume *cur_part = ubi_get_partition(partition);

	if (!out_size_num_bytes)
		return AVB_IO_RESULT_ERROR_INSUFFICIENT_SPACE;

	if (!cur_part)
		return AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION;

	*out_size_num_bytes = cur_part->reserved_pebs
						* (cur_part->ubi->leb_size - cur_part->data_pad);

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
	char *boot_mode = env_get("boot_device");

	if(strncmp(boot_mode, "nand", strlen(boot_mode) + 1) == 0) {
		return ubi_byte_io(ops, partition_name, offset_from_partition,
				num_bytes, buffer, out_num_read, FLASH_IO_READ);
	} else if(strncmp(boot_mode, "nor", strlen(boot_mode) + 1) == 0) {
		return ubi_byte_io(ops, partition_name, offset_from_partition,
				num_bytes, buffer, out_num_read, FLASH_IO_READ);
		// return mtd_byte_io(ops, partition_name, offset_from_partition,
		// 		num_bytes, buffer, out_num_read, FLASH_IO_READ);
	} else {
		printf("boot device is invalid, set boot_mode to nand or nor");
		return AVB_IO_RESULT_ERROR_NO_SUCH_VALUE;
	}
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
	char *boot_mode = env_get("boot_device");

	if(strncmp(boot_mode, "nand", strlen(boot_mode) + 1) == 0) {
		return ubi_byte_io(ops, partition_name, offset_from_partition,
				num_bytes, (void *)buffer, NULL, FLASH_IO_WRITE);
	} else if(strncmp(boot_mode, "nor", strlen(boot_mode) + 1) == 0) {
		return ubi_byte_io(ops, partition_name, offset_from_partition,
				num_bytes, (void *)buffer, NULL, FLASH_IO_WRITE);
		// return mtd_byte_io(ops, partition_name, offset_from_partition,
		// 		num_bytes, (void *)buffer, NULL, FLASH_IO_WRITE);
	} else {
		printf("boot device is invalid, set boot_mode to nand or nor");
		return AVB_IO_RESULT_ERROR_NO_SUCH_VALUE;
	}
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
	static const char fake_guid[] = "00000000-0000-0000-0000-000000000000";
	if (guid_buf_size < sizeof(fake_guid)) {
		printf("guid_buf_size %ld < fack_guid size %ld\n", guid_buf_size, sizeof(fake_guid));
		return AVB_IO_RESULT_ERROR_OOM;
	}
	memcpy(guid_buf, fake_guid, sizeof(fake_guid));
	guid_buf[guid_buf_size - 1 ] = '\0';

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
static AvbIOResult get_size_of_partition(AvbOps *ops, const char *partition,
                                         u64 *out_size_num_bytes)
{
	char *boot_mode = env_get("boot_device");

	if(strncmp(boot_mode, "nand", strlen(boot_mode) + 1) == 0) {
		return ubi_get_size_of_partition(ops, partition, out_size_num_bytes);
	} else if(strncmp(boot_mode, "nor", strlen(boot_mode) + 1) == 0) {
		return ubi_get_size_of_partition(ops, partition, out_size_num_bytes);
		// return mtd_get_size_of_partition(ops, partition, out_size_num_bytes);
	} else {
		printf("boot device is invalid, set boot_mode to nand or nor");
		return AVB_IO_RESULT_ERROR_NO_SUCH_VALUE;
	}
}

/**
 * ============================================================================
 * AVB2.0 AvbOps alloc/initialisation/free
 * ============================================================================
 */
static int mtd_avb_opsdata_alloc(struct AvbOpsData **ops,
				const char *if_typename, int boot_device)
{
	struct mtd_info *mtd;
	struct mtd_avb_info *info;

	if (strcmp(if_typename, "mtd")) {
		return -EINVAL;
	}

	/* Ensure all devices (and their partitions) are probed */
	mtd_probe_devices();

	mtd = get_mtd_device(NULL, boot_device);
	if (!mtd) {
		printf("Failed to get mtd device\n");
		return -ENODEV;
	}
	put_mtd_device(mtd);

        info = avb_calloc(sizeof(*info));
	if (!info)
		return -ENOMEM;

	info->ops_data.if_type = HB_IF_TYPE_MTD;
	info->mtd = mtd;

	info->ops_data.ops.read_from_partition = read_from_partition;
	info->ops_data.ops.write_to_partition = write_to_partition;
	info->ops_data.ops.get_unique_guid_for_partition =
		get_unique_guid_for_partition;
	info->ops_data.ops.get_size_of_partition = get_size_of_partition;

	*ops =  &info->ops_data;

	return 0;
}

HB_AVB_IF_INFO(mtd, mtd_avb_opsdata_alloc);
