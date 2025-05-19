#ifndef _FB_STORAGE_H_
#define _FB_STORAGE_H_

#include "blk.h"
#include "fastboot.h"

struct fastboot_storage_ops {
	void (*flash_write)(const char *cmd, void *download_buffer,
						u32 download_bytes, char *response);

	int64_t (*flash_read)(struct fetch_info *info, void *upload_buffer,
						  u64 buffer_size, s64 offset, char *response);

	void (*erase)(const char *cmd, char *response);

	int (*get_part)(const char *part_name, size_t *size, char *response);

	int (*get_block_size)(const char *part_name, size_t *size, char *response);

	int (*get_part_type)(const char *part_name, char *response);

	void (*get_fetch_size)(const char *part_name, size_t offset,
		char *response);

	const char *type_name;  /* e.g. "mmc", "spinand" */
};

void fastboot_storage_init(void);

void fastboot_storage_register(fb_flash_type type,
							   const struct fastboot_storage_ops *ops);

const struct fastboot_storage_ops *get_current_storage_ops(void);

#endif  // _FB_STORAGE_H_
