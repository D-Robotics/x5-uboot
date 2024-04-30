/*
 * memdump.h --- Description
 *
 * Copyright (C) 2021, Schspa, all rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#ifndef MEMDUMP_H
#define MEMDUMP_H

#define ENABLE_USERDATA_MEMDUMP 1
#define	RAMDUMP_USERDATA_STORAGE "mmc"
#define	RAMDUMP_MMC_DEVICE_NUM	"0"
#define	RAMDUMP_USERDATA_PART_NAME "userdata"
/* The path relative to /userdata must end with '/' */
#define	RAMDUMP_USERDATA_PATH "/"

#define EXT4_MAX_FILE_LENGTH     SZ_2G
#define EXT4_TEST_FILE_LENGTH     SZ_1M
#define CONFIGURE_RESERVED_REGION   (0x100000)
#define ATF_RESERVED_REGION   (0x80000)
#define OPTEE_RESERVED_REGION  (0x08000000)
#define EL3_RESERVED_REGION	\
		(CONFIGURE_RESERVED_REGION + ATF_RESERVED_REGION + OPTEE_RESERVED_REGION)
#define	UNRESERVED_MEMORY_START	 (0xC0000000UL)
#define	UNRESERVED_MEMORY_END    (0xFFFFFFFFUL)
#define	TEMP_TRANSFER_MEMORY_SIZE \
		(UNRESERVED_MEMORY_END - UNRESERVED_MEMORY_START)
#define MMC_SDMA_HIGEST_ADDR     (0x1000000000UL)

int memdump_dump_blk_offset(char *intf, int dev, unsigned long dump_start);
int memdump_dump_blk(char *intf, int dev, char *partition);
int memdump_dump_ext4(char *intf, int dev, int part, char *directory);
int search_userdata_part(char *part_str, int str_len);
bool ramdump_path_exist(void);
void set_memdump_test_true(void);

#endif
