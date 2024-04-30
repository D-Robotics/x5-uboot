/*
 * memdump.c<common> --- Description
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

#include <command.h>
#include <part.h>
#include <config.h>
#include <command.h>
#include <image.h>
#include <linux/ctype.h>
#include <asm/byteorder.h>
#include <linux/stat.h>
#include <malloc.h>
#include <fs.h>
#include <mtd.h>
#include <linux/mtd/mtd.h>
#include <asm/sections.h>
#include <memdump.h>
#include <linux/arm-smccc.h>
#include <linux/sizes.h>
#include <part.h>

#ifndef MEMDUMP_PARTITION
#define MEMDUMP_PARTITION            "ramdump"
#endif

#ifndef MEMDUMP_BLOCK_SIZE
#define	MEMDUMP_BLOCK_SIZE           SZ_128M
#endif
#define MEMDUMP_BLOCK_TEST_SIZE      SZ_1M

#define HB_IPI_MAX_CORES 64
#define HB_IPI_MAX_REGS  128


static struct blk_desc *memdump_get_blk_desc(char *intf, int dev)
{
	struct blk_desc *blk_desc;
	enum if_type if_type = IF_TYPE_UNKNOWN;
	int i;

	for (i = 0; i < IF_TYPE_COUNT; i++) {
		const char *if_typename_str = blk_get_if_type_name(i);
		if (if_typename_str && !strcmp(intf, if_typename_str)) {
			if_type = i;
			break;
		}
	}

        if (if_type == IF_TYPE_UNKNOWN) {
		debug("Can't find block device interface %s\n", intf);

		return NULL;
        }

	blk_desc = blk_get_devnum_by_type(if_type, dev);
	if (!blk_desc) {
		debug("Can't find block device for %s:%d\n",
			intf, dev);

		return NULL;
	}

	return blk_desc;
}

int memdump_dump_blk_offset(char *intf, int dev, unsigned long dump_start)
{
	struct blk_desc *blk_desc;
	lbaint_t blkcnt, temp;
	lbaint_t offset;
	unsigned long count;
	unsigned long write_count;
	unsigned long dump_block_cnt;
	void *buffer;
	int i;

	blk_desc = memdump_get_blk_desc(intf, dev);
	if (!blk_desc)
		return -ENODEV;

	dump_block_cnt = MEMDUMP_BLOCK_SIZE >> blk_desc->log2blksz;

	offset = dump_start;

	for (i = 0; i < CONFIG_NR_DRAM_BANKS; ++i) {
		if (gd->bd->bi_dram[i].size == 0) {
			continue;
		}
		debug("DRAM bank= 0x%d\n", i);
		debug("-> start   = 0x%016llX\n", gd->bd->bi_dram[i].start);
		debug("-> size    = 0x%016llX\n", gd->bd->bi_dram[i].size);
		debug("-> offset  = 0x%016lX\n",
			((offset - dump_start) << blk_desc->log2blksz));

		buffer = (void *) gd->bd->bi_dram[i].start;
		blkcnt = gd->bd->bi_dram[i].size >> blk_desc->log2blksz;

		for (write_count = 0; write_count < blkcnt; write_count += dump_block_cnt) {
			temp = (blkcnt - write_count) >= dump_block_cnt ?
				dump_block_cnt : (blkcnt - write_count);
			printf("Dump %p -> offset: 0x%016lx, cnt: 0x%016lx\n",
				buffer, offset, temp);
			/*skip secure wolrd memory region*/
			if ((ulong)buffer < gd->bd->bi_dram[0].start + EL3_RESERVED_REGION) {
				buffer += temp << blk_desc->log2blksz;
				continue;
			}
			if (env_get_yesno("memdump_test") == 1)
				temp = MEMDUMP_BLOCK_TEST_SIZE >> blk_desc->log2blksz;
#ifdef CONFIG_MMC_SDHCI_SDMA
			if (buffer >= (void *)0x100000000UL) {
				memcpy((void *)gd->bd->bi_dram[0].start, buffer,
					temp << blk_desc->log2blksz);
				count = blk_dwrite(blk_desc, offset, temp, (void *)gd->bd->bi_dram[0].start);
			} else {
				count = blk_dwrite(blk_desc, offset, temp, buffer);
			}
#else
			count = blk_dwrite(blk_desc, offset, temp, buffer);
#endif
			if (count != temp) {
				debug("block write failed, cnt: 0x%016lx, ret: 0x%016lx\n",
					blkcnt, count);
				return -EIO;
			}
			if (env_get_yesno("memdump_test") == 1)
				return 0;
			offset += count;
			buffer += count << blk_desc->log2blksz;
		}
	}

	return 0;
}

int memdump_dump_blk(char *intf, int dev, char *partition)
{
	struct blk_desc *blk_desc;
	unsigned long dump_start;

	blk_desc = memdump_get_blk_desc(intf, dev);
	if (!blk_desc)
		return -ENODEV;

#if CONFIG_IS_ENABLED(PARTITIONS)
	{
		struct disk_partition info;
		int ret;

		ret = part_get_info_by_name(blk_desc, MEMDUMP_PARTITION, &info);
		if (ret < 0) {
			printf("Can't find partition '%s'\n", MEMDUMP_PARTITION);
			return -ENODEV;
		}

		dump_start = info.start;
	}
#else
	dump_start = simple_strtoul(partition, NULL, 16);
#endif

	return memdump_dump_blk_offset(intf, dev, dump_start);

	return 0;
}

#ifdef ENABLE_USERDATA_MEMDUMP
/*out:part_str,return string format for fs function usage*/
int search_userdata_part(char *part_str, int str_len)
{
	static int part_num = 0;
	int ret;
	struct disk_partition info;
	struct blk_desc *dev_desc;
	char *part;

	if (part_num != 0) {
		snprintf(part_str, str_len, "%s:%x", RAMDUMP_MMC_DEVICE_NUM, part_num);
		return 0;
	}

	ret = blk_get_device_by_str("mmc", RAMDUMP_MMC_DEVICE_NUM, &dev_desc);
	if (ret) {
		return 1;
	}

	part = env_get("ramdump_part_name");
	if (part == NULL)
		part = RAMDUMP_USERDATA_PART_NAME;

	for (part_num = 1; part_num < MAX_SEARCH_PARTITIONS; part_num++) {
		ret = part_get_info(dev_desc, part_num, &info);

		if (ret)
			continue;
		if (!strncmp((char *)info.name, part, strlen(part))) {
			snprintf(part_str, str_len, "%s:%x", RAMDUMP_MMC_DEVICE_NUM, part_num);
			return 0;
		}
	}
	return 1;
}

#if 0

static int memdump_dump_cpu_context(char *directory)
{
	ulong *cpu_context_buf;
	int cpu_context_buf_idx = 0;
	char filename[100] = {0};
	struct arm_smccc_res res;
	unsigned long sip_version;
	int cpu, reg, ret;
	loff_t len = 0;
	ulong time;

	cpu_context_buf = malloc(HB_IPI_MAX_CORES * HB_IPI_MAX_REGS *sizeof(cpu_context_buf[0]));
	if (!cpu_context_buf)
		return -ENOMEM;

	memset(cpu_context_buf, 0x0, HB_IPI_MAX_CORES * HB_IPI_MAX_REGS *sizeof(cpu_context_buf[0]));

	arm_smccc_smc(HB_SIP_IPI, HB_SIP_IPI_VERSION, 0x0, 0, 0, 0, 0, 0, &res);
	if (res.a0 != 0) {
		printf("Don't support cpu context dump, skip it %lu\n", res.a0);
		ret = -ENOSYS;
		goto free_buf;
	}
	sip_version = res.a1;

	for (cpu = 0; cpu < HB_IPI_MAX_CORES; cpu++) {
		for (reg = 0; reg < HB_IPI_MAX_REGS; reg++) {
			//arm_smccc_smc(HB_SIP_IPI, HB_SIP_IPI_GET_CORE_CONTEXT, cpu, reg, 0, 0, 0, 0, &res);
			if (res.a0 != 0) {
				break;
			}
			cpu_context_buf[cpu_context_buf_idx] = res.a2 & 0xffffffff;
			cpu_context_buf[cpu_context_buf_idx] <<= 32;
			cpu_context_buf[cpu_context_buf_idx] += res.a1 & 0xffffffff;
			cpu_context_buf_idx++;
		}
	}
	arm_smccc_smc(HB_SIP_IPI, HB_SIP_IPI_CLEAR_CORE_CONTEXT, sip_version, 0x0, 0, 0, 0, 0, &res);
	if (res.a0 != 0) {
		printf("Failed to clear core contexts withs tatus %lu\n", res.a0);
	}

	snprintf(filename, sizeof(filename), "%s/cpu-contexts.bin", directory);
	time = get_timer(0);
	ret = fs_write(filename, (ulong)cpu_context_buf, 0, cpu_context_buf_idx * sizeof(cpu_context_buf[0]), &len);
	time = get_timer(time);
	if (ret < 0) {
		printf("Fail to write cpu contexts to userdata:%s,ret=%d,len=%lld\n", filename, ret, len);
		ret = -EIO;
	} else {
		printf("cpu core context dumped to %s\n", filename);
		ret = 0;
	}

free_buf:
	free(cpu_context_buf);

	return ret;
}
#endif
int memdump_dump_ext4(char *intf, int dev, int part, char *directory)
{
	int ret, i, count;
	ulong time, length;
	char dev_part_str[10] = {0};
	loff_t len = 0, j;
	char filename[100] = {0};

	snprintf(dev_part_str, sizeof(dev_part_str), "%x:%x", dev, part);
	if (fs_set_blk_dev(intf, dev_part_str, FS_TYPE_EXT))
		return 1;
	//(void) memdump_dump_cpu_context(directory);

	for (i = 0; i < CONFIG_NR_DRAM_BANKS; ++i) {
		if (gd->bd->bi_dram[i].size == 0) {
			continue;
		}

		printf("DRAM bank= 0x%d\n", i);
		printf("-> start   = 0x%016llX\n", gd->bd->bi_dram[i].start);
		printf("-> size    = 0x%016llX\n", gd->bd->bi_dram[i].size);

		for (j = 0, count = 0; j < gd->bd->bi_dram[i].size; j += len) {
			char *buf = (char *)(gd->bd->bi_dram[i].start + j);

			snprintf(filename, sizeof(filename), "%sDDRCS%d-%d.bin", directory, i, count++);
			if ((ulong)buf >= MMC_SDMA_HIGEST_ADDR) {
				if ((gd->bd->bi_dram[i].size - j) > TEMP_TRANSFER_MEMORY_SIZE) {
					length = TEMP_TRANSFER_MEMORY_SIZE;
				} else {
					length = gd->bd->bi_dram[i].size - j;
				}
				memcpy((void *)UNRESERVED_MEMORY_START, buf, length);
				buf = (char *)UNRESERVED_MEMORY_START;
			} else {
				/*skip secure wolrd memory region*/
				if ((ulong)buf < gd->bd->bi_dram[0].start + EL3_RESERVED_REGION) {
					len = EL3_RESERVED_REGION;
					continue;
				}
				if ((gd->bd->bi_dram[i].size-j) > EXT4_MAX_FILE_LENGTH) {
					length = EXT4_MAX_FILE_LENGTH;
				} else {
					length = gd->bd->bi_dram[i].size - j;
				}
			}
			if (env_get_yesno("memdump_test") == 1) {
				snprintf(filename, sizeof(filename), "memdump_test.bin");
				length = EXT4_TEST_FILE_LENGTH;
			}
			printf("-> dumpfile = /%s/%s, from memory 0x%llx, length=%lu\n",
				RAMDUMP_USERDATA_PART_NAME, filename, gd->bd->bi_dram[i].start + j, length);
			if (fs_set_blk_dev(intf, dev_part_str, FS_TYPE_EXT))
				return 1;
			time = get_timer(0);
			ret = fs_write(filename, (ulong)buf, 0, length, &len);
			time = get_timer(time);
			if (ret < 0) {
				printf("Fail to write ramdump to /%s/%s,ret=%d,len=%lld\n",
					RAMDUMP_USERDATA_PART_NAME, filename, ret, len);
				return 1;
			}
			printf("%llu bytes written in %lu ms\n", len, time);
			if (env_get_yesno("memdump_test") == 1) {
				return 0;
			}
		}
	}

	return 0;
}
#endif
