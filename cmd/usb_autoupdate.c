#include <common.h>
#include <command.h>
#include <bootstage.h>
#include <usb.h>
#include <env.h>
#include <hb_info.h>
#include <fat.h>
#include <fs.h>
#include <string.h>
#include <asm/arch/hb_board.h>
#include <linux/mtd/mtd.h>
#include <mtd.h>
#include <linux/ctype.h>
#include <blk.h>
#include <part.h>
#include <linux/types.h>
#include <mmc.h>
#include <linux/delay.h>
#include <image-sparse.h>

#define BLK_SIZE			512
#define USB_UPDATE_LOAD_ADDR_DEFAULT	0x90000000
#define USB_UPDATE_ONESHOT_SIZE		(128 * 1024 * 1024)
#define WR_BLK_NUM			(USB_UPDATE_ONESHOT_SIZE / BLK_SIZE)
#define USB_UPDATE_FOLDER				"/x5_udr_1f4k7m/"
#define USB_UPDATE_SPARSE_SZ_ONESHOT	USB_UPDATE_ONESHOT_SIZE
#define USB_UPDATE_SKIP					1
#define USB_UPDATE_NO_IMG				2
#define USB_UPDATE_SPLIT_MAX_PARTS		32

#ifdef CONFIG_USB_STORAGE
static int usb_stor_curr_dev = -1; /* current device */
#endif

enum USB_UPDATE_TYPE_E{
	UPDATE_FULL_DISK,
	UPDATE_SINGLE_PART,
	UPDATE_AB_PART,
	UPDATE_BAK_PART,
};

#define A_PART_SUFFIX	"_a"
#define B_PART_SUFFIX	"_b"
#define BAK_PART_SUFFIX	"_bak1"

static int usb_update_mmc_raw_image(struct fs_dirent *dent, int flag);
static int usb_udpate_mmc_sparse_image(struct fs_dirent *dent, int flag);
static int usb_update_mtd_image(struct fs_dirent *dent, int flag);
static int _update_mmc_raw_part(char *part_name, char *image_name,
				loff_t image_size, lbaint_t flash_start_blk);
static ulong usb_update_get_load_addr(void);

typedef int (*USB_UPDATE_IMAGE_CALLBACK)(struct fs_dirent *dent, int flag);

typedef struct {
	char *image_name;
	int mmc_dev;
	USB_UPDATE_IMAGE_CALLBACK callback;
	int flag;
} USB_UPDATE_IMAGE_LIST_T;

static int usb_update_curr_mmc_dev;

static USB_UPDATE_IMAGE_LIST_T update_image_list[] = {
	{"emmc_disk.img",    0, usb_update_mmc_raw_image,    UPDATE_FULL_DISK},
	{"emmc_disk.simg",   0, usb_udpate_mmc_sparse_image, UPDATE_FULL_DISK},
	{"miniboot_all.img", 0, usb_update_mmc_raw_image,    UPDATE_FULL_DISK},
	{"sd_disk.img",      1, usb_update_mmc_raw_image,    UPDATE_FULL_DISK},
	{"nand_disk.img",   -1, usb_update_mtd_image,        UPDATE_FULL_DISK},
};

struct fb_mmc_sparse {
	struct blk_desc	*dev_desc;
};

#define FASTBOOT_MAX_BLK_WRITE 16384
#define FASTBOOT_MAX_BLK_READ 16384

static int usb_update_mtd_image(struct fs_dirent *dent, int flag)
{
	char *token = NULL;
	char buffer[128] = {0};
	char mtdparts[20];
	int32_t ret = 0;
	ulong load_addr = usb_update_get_load_addr();

	if(flag == UPDATE_SINGLE_PART){
		strcpy(mtdparts, dent->name);
		token = strtok(mtdparts, ".");
	} else if(flag == UPDATE_FULL_DISK){
		strcpy(mtdparts, "spi-nand0");
	}
	printf("load %s to mtdparts %s\n", dent->name, mtdparts);
	memset(buffer, 0, sizeof(buffer));
	snprintf(buffer, sizeof(buffer), "fatload usb 0 0x%lx %s%s", load_addr, USB_UPDATE_FOLDER, dent->name);
	printf("buffer:  %s\n", buffer);
	ret = run_command(buffer, 1);	if (ret) {
		printf("fatload %s from usb failed\n", dent->name);
		return -1;
	}
	memset(buffer, 0, sizeof(buffer));
	snprintf(buffer, sizeof(buffer), "mtd erase %s", mtdparts);
	printf("mtd erase %s parts\n", mtdparts);
	ret = run_command(buffer, 1);	if (ret) {
		printf("mtd erase %s error\n", mtdparts);
		return -1;
	}
	memset(buffer, 0, sizeof(buffer));
	snprintf(buffer, sizeof(buffer), "mtd write %s 0x%lx", mtdparts, load_addr);
	ret = run_command(buffer, 1);	if (ret) {
		printf("mtd write %s error\n", mtdparts);
		return -1;
	}
	printf("mtd write %s success\n", mtdparts);
	return 0;
}

static int usb_update_get_mmc_dev(void)
{
	return usb_update_curr_mmc_dev;
}

static ulong usb_update_get_load_addr(void)
{
	ulong addr;

	addr = env_get_ulong("loadaddr", 16, 0);
	if (addr)
		return addr;

	return USB_UPDATE_LOAD_ADDR_DEFAULT;
}

static int get_mmc_partition_info(const char *name, struct disk_partition *info, int mmc_dev)
{
	int ret;
	struct blk_desc *dev_desc = NULL;
	dev_desc = blk_get_dev("mmc", mmc_dev);
	if (!dev_desc) {
		printf("blk_get_dev: mmc-%d failed\n", mmc_dev);
		return -1;
	}

	ret = part_get_info_by_name(dev_desc, name, info);
	if (ret < 0) {
		printf("Can't find partition '%s'\n", name);
		return -ENODEV;
	}

	return 0;
}

static struct mmc *init_mmc_device(int dev, bool force_init,
				     enum bus_mode speed_mode)
{
	struct mmc *mmc;
	mmc = find_mmc_device(dev);
	if (!mmc) {
		printf("no mmc device at slot %x\n", dev);
		return NULL;
	}

	if (!mmc_getcd(mmc))
		force_init = true;

	if (force_init)
		mmc->has_init = 0;

	if (IS_ENABLED(CONFIG_MMC_SPEED_MODE_SET))
		mmc->user_speed_mode = speed_mode;

	if (mmc_init(mmc))
		return NULL;

#ifdef CONFIG_BLOCK_CACHE
	struct blk_desc *bd = mmc_get_blk_desc(mmc);
	blkcache_invalidate(bd->if_type, bd->devnum);
#endif

	return mmc;
}

static lbaint_t usb_mmc_blk_write(struct blk_desc *block_dev, lbaint_t start,
				 lbaint_t blkcnt, const void *buffer)
{
	lbaint_t blk = start;
	lbaint_t blks_written;
	lbaint_t cur_blkcnt;
	lbaint_t blks = 0;
	int32_t i;

	for (i = 0; i < blkcnt; i += FASTBOOT_MAX_BLK_WRITE) {
		cur_blkcnt = min((int)blkcnt - i, FASTBOOT_MAX_BLK_WRITE);
		if (buffer) {
			blks_written = blk_dwrite(block_dev, blk, cur_blkcnt,
						  buffer + (i * block_dev->blksz));
		} else {
			blks_written = blk_derase(block_dev, blk, cur_blkcnt);
		}
		blk += blks_written;
		blks += blks_written;
	}
	return blks;
}

static lbaint_t usb_mmc_sparse_write(struct sparse_storage *info,
		lbaint_t blk, lbaint_t blkcnt, const void *buffer)
{
	struct fb_mmc_sparse *sparse = info->priv;
	struct blk_desc *dev_desc = sparse->dev_desc;

	return usb_mmc_blk_write(dev_desc, blk, blkcnt, buffer);
}

static lbaint_t usb_mmc_sparse_reserve(struct sparse_storage *info,
		lbaint_t blk, lbaint_t blkcnt)
{
	return blkcnt;
}


static int usb_udpate_load_image(char *filepath, void *load_addr, uint64_t load_sz, uint64_t load_offset)
{
	int ret;
	char cmd[256] = {0};

	snprintf(cmd, sizeof(cmd), "fatload usb 0 0x%p %s %llx %llx", load_addr, filepath, load_sz, load_offset);
	// printf("Load:  %s\n", cmd);
	ret = run_command(cmd, 1);
	if (ret) {
		printf("fatload %s from usb failed\n", filepath);
		return -1;
	}
	return 0;
}

void print_sparse_header(sparse_header_t *sparse_header)
{
	printf("=== Sparse Image Header ===\n");
	printf("magic: 0x%x\n", sparse_header->magic);
	printf("major_version: 0x%x\n", sparse_header->major_version);
	printf("minor_version: 0x%x\n", sparse_header->minor_version);
	printf("file_hdr_sz: %d\n", sparse_header->file_hdr_sz);
	printf("chunk_hdr_sz: %d\n", sparse_header->chunk_hdr_sz);
	printf("blk_sz: %d\n", sparse_header->blk_sz);
	printf("total_blks: %d\n", sparse_header->total_blks);
	printf("total_chunks: %d\n", sparse_header->total_chunks);
}

void print_sparse_chunk_header(int i, chunk_header_t *chunk_header)
{
	printf("Chunk %3u: ", i);
	switch (chunk_header->chunk_type) {
        case CHUNK_TYPE_RAW:
            printf("CHUNK_TYPE_RAW");
            break;
        case CHUNK_TYPE_FILL:
            printf("CHUNK_TYPE_FILL");
            break;
        case CHUNK_TYPE_DONT_CARE:
            printf("CHUNK_TYPE_DONT_CARE");
            break;
        case CHUNK_TYPE_CRC32:
            printf("CHUNK_TYPE_CRC32");
            break;
        default:
            printf("Unknown (0x%04X)", chunk_header->chunk_type);
            break;
    }
	printf(", Blocks: %u", chunk_header->chunk_sz);
	printf(", Total Size: %u bytes ", chunk_header->total_sz);
	printf(" [%d]\n", chunk_header->chunk_sz * 4096);
}

int check_sparse_chunk_header(chunk_header_t *chunk_header)
{
	if (chunk_header->chunk_type != CHUNK_TYPE_RAW && \
		chunk_header->chunk_type != CHUNK_TYPE_FILL && \
		chunk_header->chunk_type != CHUNK_TYPE_DONT_CARE && \
		chunk_header->chunk_type != CHUNK_TYPE_CRC32) {
		return -1;
	}
	return 0;
}

static int usb_udpate_mmc_sparse_image(struct fs_dirent *dent, int flag)
{
	struct fb_mmc_sparse sparse_priv;
	struct sparse_storage sparse;
	sparse_header_t *sparse_header = NULL;
	chunk_header_t *chunk_header = NULL;
	void *chunk_data = NULL;
	int err = -1;

	void *buffer_addr = (void *)usb_update_get_load_addr();
	char image_full_path[64] = {0};

	// uint64_t raw_wr_sz = 0;
	uint64_t raw_total_size = 0;
	uint64_t raw_total_chunk = 0;
	uint64_t raw_offset = 0;
	int raw_blk_cnt = 0;

	int chunk_cnt = 0;
	int chunk_remain = 0;
	uint64_t sparse_offset = 0;
	uint64_t sparse_rd_sz = 0;
	uint64_t raw_flash_size = 0;
	uint64_t raw_flash_size_total = 0;

	struct blk_desc *dev_desc = NULL;
	int mmc_dev = usb_update_get_mmc_dev();

	dev_desc = blk_get_dev("mmc", mmc_dev);
	if (!dev_desc) {
		printf("blk_get_dev: mmc-%d failed\n", mmc_dev);
		return -1;
	}

	snprintf(image_full_path, sizeof(image_full_path), "%s/%s", USB_UPDATE_FOLDER, dent->name);
	printf("[USB update] Raw image path: %s\n", image_full_path);

	sparse_header = (sparse_header_t *)buffer_addr;
	if (0 != usb_udpate_load_image(image_full_path, sparse_header, sizeof(sparse_header_t), 0)) {
		printf("usb_udpate_load_image failed\n");
		return -1;
	}

	// debug
	print_sparse_header(sparse_header);

	chunk_header = buffer_addr + sparse_header->file_hdr_sz;
	chunk_data = chunk_header;
	sparse_offset += sparse_header->file_hdr_sz;
	raw_total_size = sparse_header->total_blks * sparse_header->blk_sz;
	raw_total_chunk = sparse_header->total_chunks;
	printf("=== Raw Image Info ===\n");
	printf("Raw total size: %lld\n", raw_total_size);
	printf("Raw total chunk: %lld\n", raw_total_chunk);

	chunk_remain = raw_total_chunk;
	while (chunk_remain > 0) {

		// Get Chunk Header
		if (0 != usb_udpate_load_image(image_full_path, chunk_header, sizeof(chunk_header_t), sparse_offset)) {
			printf("usb_udpate_load_image failed\n");
			return -1;
		}

		print_sparse_chunk_header(raw_total_chunk - chunk_remain, chunk_header);
		if (0 != check_sparse_chunk_header(chunk_header)) {
			printf("Chunk Header Error\n");
			return -1;
		}

		raw_flash_size = chunk_header->chunk_sz * sparse_header->blk_sz;
		raw_flash_size_total += raw_flash_size;
		sparse_rd_sz += chunk_header->total_sz;
		sparse_offset += chunk_header->total_sz;
		raw_blk_cnt += chunk_header->chunk_sz;
		chunk_remain--;
		chunk_cnt++;
		if (raw_flash_size_total < USB_UPDATE_SPARSE_SZ_ONESHOT) {
			if (chunk_remain > 0) {
				continue;
			}
		}

		// Get Total Chunk
		printf("[USB update] Loading [%s] size[0x%llx] offset[0x%llx]\n", image_full_path, sparse_rd_sz, sparse_offset - sparse_rd_sz);
		if (0 != usb_udpate_load_image(image_full_path, chunk_data, sparse_rd_sz, sparse_offset - sparse_rd_sz)) {
			printf("usb_udpate_load_image failed\n");
			return -1;
		}

		sparse_header->total_chunks = chunk_cnt;
		sparse_header->total_blks = raw_blk_cnt;

		sparse_priv.dev_desc = dev_desc;
		sparse.blksz = dev_desc->blksz;
		sparse.start = DIV_ROUND_UP_ULL(raw_offset, dev_desc->blksz);
		sparse.size  = DIV_ROUND_UP_ULL(raw_flash_size_total, dev_desc->blksz);
		sparse.write = usb_mmc_sparse_write;
		sparse.reserve = usb_mmc_sparse_reserve;
		sparse.mssg = NULL;

		printf("Flashing sparse image: offset[%ld] size[%ld]\n",
				sparse.start, sparse.size);

		sparse.priv = &sparse_priv;
		err = write_sparse_image(&sparse, "addr:0x0", buffer_addr,
						"USB-update");

		raw_offset += raw_flash_size_total;
		sparse_rd_sz = 0;
		raw_flash_size_total = 0;
		chunk_cnt = 0;
		raw_blk_cnt = 0;
	}

	return err;
}


static int get_prefix(const char *filename, char *prefix, int maxsize)
{

	const char *last_dot = strrchr(filename, '.');

	if (last_dot == NULL || last_dot == filename) {
		return -1;
	}

	size_t prefix_len = last_dot - filename;
	if (prefix_len >= maxsize) {
		return -1;
	}

	strncpy(prefix, filename, prefix_len);
	prefix[prefix_len] = '\0';

	return 0;
}

static int check_full_image(char *image_name)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(update_image_list); i++) {
		if (0 == strcmp(image_name, update_image_list[i].image_name)) {
			return i;
		}
	}
	return -1;
}

typedef struct {
	int part_num;
	char name[64];
	loff_t size;
} usb_split_part_t;

static int parse_split_disk_image(const char *name, char *base, int base_len,
				  int *part_num, int *mmc_dev)
{
	const char *p;
	char num_str[8];
	int i = 0;

	if (!strncmp(name, "emmc_disk.img.", 14)) {
		strncpy(base, "emmc_disk.img", base_len);
		*mmc_dev = 0;
		p = name + 14;
	} else if (!strncmp(name, "sd_disk.img.", 12)) {
		strncpy(base, "sd_disk.img", base_len);
		*mmc_dev = 1;
		p = name + 12;
	} else {
		return -1;
	}
	base[base_len - 1] = '\0';

	while (*p && i < (int)sizeof(num_str) - 1) {
		if (!isdigit(*p))
			return -1;
		num_str[i++] = *p++;
	}
	num_str[i] = '\0';
	if (!i || *p)
		return -1;

	*part_num = dectoul(num_str, NULL);

	return 0;
}

static int usb_update_sort_split_parts(usb_split_part_t *parts, int count)
{
	int i, j;

	for (i = 0; i < count - 1; i++) {
		for (j = i + 1; j < count; j++) {
			if (parts[j].part_num < parts[i].part_num) {
				usb_split_part_t tmp = parts[i];

				parts[i] = parts[j];
				parts[j] = tmp;
			}
		}
	}

	for (i = 0; i < count; i++) {
		if (parts[i].part_num != i) {
			printf("[USB update] split parts not continuous, missing part %02d\n",
			       i);
			return -1;
		}
	}

	return 0;
}

static int usb_update_fs_setup(void)
{
	if (fs_set_blk_dev("usb", "0", FS_TYPE_FAT)) {
		printf("[USB update] USB is not FAT32, skip update\n");
		return -1;
	}

	return 0;
}

static int usb_update_collect_split_parts(const char *dirname, const char *base,
					  usb_split_part_t *parts, int *count)
{
	struct fs_dir_stream *dirs;
	struct fs_dirent *dent;
	char part_base[32];
	int part_num, mmc_dev;
	int i, n = 0;

	if (usb_update_fs_setup())
		return -1;

	dirs = fs_opendir(dirname);
	if (!dirs)
		return -1;

	while ((dent = fs_readdir(dirs))) {
		if (parse_split_disk_image(dent->name, part_base,
					   sizeof(part_base), &part_num, &mmc_dev))
			continue;
		if (strcmp(part_base, base))
			continue;
		if (n >= USB_UPDATE_SPLIT_MAX_PARTS) {
			printf("[USB update] too many split parts for %s\n", base);
			fs_closedir(dirs);
			return -1;
		}

		parts[n].part_num = part_num;
		strncpy(parts[n].name, dent->name, sizeof(parts[n].name));
		parts[n].name[sizeof(parts[n].name) - 1] = '\0';
		parts[n].size = dent->size;
		n++;
	}
	fs_closedir(dirs);

	if (!n) {
		printf("[USB update] no split parts found for %s\n", base);
		return -1;
	}

	if (usb_update_sort_split_parts(parts, n))
		return -1;

	*count = n;
	printf("[USB update] found %d split parts for %s\n", n, base);
	for (i = 0; i < n; i++)
		printf("[USB update]   part%02d: %s size %lld\n",
		       parts[i].part_num, parts[i].name, parts[i].size);

	return 0;
}

static int usb_update_mmc_split_disk(const char *dirname, const char *base)
{
	usb_split_part_t parts[USB_UPDATE_SPLIT_MAX_PARTS];
	lbaint_t start_blk = 0;
	int count = 0;
	int i, ret;

	ret = usb_update_collect_split_parts(dirname, base, parts, &count);
	if (ret)
		return ret;

	for (i = 0; i < count; i++) {
		ret = _update_mmc_raw_part("addr:0x0", parts[i].name,
					   parts[i].size, start_blk);
		if (ret)
			return ret;
		start_blk += (parts[i].size + BLK_SIZE - 1) / BLK_SIZE;
	}

	printf("[USB update] split image [%s] success, %d parts\n", base, count);
	return 0;
}

static int check_part_image(char *image_name, int *part_type, int *mmc_dev)
{
	char partname[32] = {0};
	char a_partname[32] = {0};
	char b_partname[32] = {0};
	char bak_partname[32] = {0};
	struct disk_partition part_info = {0};
	const int dev = 0;

	if (0 != get_prefix(image_name, partname, 32)) {
		printf("[USB update] can not get prefix [%s]\n", image_name);
		return -1;
	}

	*mmc_dev = dev;

	if (0 == get_mmc_partition_info(partname, &part_info, dev)) {
		*part_type = UPDATE_SINGLE_PART;
		printf("[USB update] found part[%s] on mmc%d\n", partname, dev);

		snprintf(bak_partname, 32, "%s%s", partname, BAK_PART_SUFFIX);
		if (0 == get_mmc_partition_info(bak_partname, &part_info, dev)) {
			*part_type = UPDATE_BAK_PART;
			printf("[USB update] found BAK part[%s] on mmc%d\n", bak_partname, dev);
		}
		return 0;
	}

	snprintf(a_partname, 32, "%s%s", partname, A_PART_SUFFIX);
	if (0 != get_mmc_partition_info(a_partname, &part_info, dev))
		return -1;
	snprintf(b_partname, 32, "%s%s", partname, B_PART_SUFFIX);
	if (0 != get_mmc_partition_info(b_partname, &part_info, dev))
		return -1;
	*part_type = UPDATE_AB_PART;
	printf("[USB update] found AB part[%s] [%s] on mmc%d\n", a_partname, b_partname, dev);
	return 0;
}

static int _update_mmc_raw_part(char *part_name, char *image_name,
				loff_t image_size, lbaint_t flash_start_blk)
{
	int ret;
	struct disk_partition part_info = {0};

	loff_t remain_size = image_size;
	loff_t read_size = 0;
	loff_t oneshot_read_size = WR_BLK_NUM * BLK_SIZE;
	loff_t offset = 0;
	lbaint_t start_blk = flash_start_blk, write_blk = 0;

	char command[128] = {0};
	ulong load_addr = usb_update_get_load_addr();
	int mmc_dev = usb_update_get_mmc_dev();

	if(0 != strcmp(part_name, "addr:0x0")){
		ret = get_mmc_partition_info(part_name, &part_info, mmc_dev);
		if (ret != 0) {
			printf("[USB Update] get mmc [%s] partition info faild\n", part_name);
			return -1;
		}
		start_blk = part_info.start;
	}

	printf("\n\n[USB Update] Part[%s] Bgn: image_name[%s] image_size[%lld] start_blk[0x%lx]\n", part_name, image_name, image_size, start_blk);

	snprintf(command, sizeof(command), "mmc dev %d", mmc_dev);
	if (run_command(command, 1)) {
		printf("[USB Update] mmc dev %d failed\n", mmc_dev);
		return -1;
	}

	while (remain_size > 0) {
		read_size = (remain_size >= oneshot_read_size)? oneshot_read_size : remain_size;
		write_blk = (read_size + BLK_SIZE - 1) / BLK_SIZE;

		/* Read blocks from usb */
		memset(command, 0, sizeof(command));
		snprintf(command, sizeof(command), "fatload usb 0 0x%lx %s%s %llx %llx", load_addr, USB_UPDATE_FOLDER, image_name, read_size, offset);
		if (0 != run_command(command, 1)) {
			printf("[USB Update] fatload %s from usb error: read_size[%lld] offset[%lld]\n", image_name, read_size, offset);
			return -1;
		}
		/* Write blocks into emmc */
		memset(command, 0, sizeof(command));
		snprintf(command, sizeof(command), "mmc write 0x%lx 0x%lx 0x%lx", load_addr, start_blk, write_blk);
		if (0 != run_command(command, 1)) {
			printf("[USB Update] mmc write part[%s] error: start_blk[0x%lx] write_blk[%lu]\n", part_name, start_blk, write_blk);
			return -1;
		}
		printf("[USB Update] Part[%s]: start_blk[0x%lx] read_size[%lld]\n", part_name, start_blk, read_size);

		remain_size -= read_size;
		offset += read_size;
		start_blk += write_blk;
	}

	printf("[USB Update] Image[%s] to Part[%s] success\n\n\n", image_name, part_name);
	return 0;
}

static int usb_update_mmc_raw_image(struct fs_dirent *dent, int flag)
{
	int32_t ret = 0;
	char partname[32] = {0};
	char a_partname[32] = {0};
	char b_partname[32] = {0};
	char bak_partname[32] = {0};

	if (flag == UPDATE_FULL_DISK) {
		ret = _update_mmc_raw_part("addr:0x0", dent->name, dent->size, 0);
	}
	else if (flag == UPDATE_SINGLE_PART) {
		get_prefix(dent->name, partname, 32);
		ret = _update_mmc_raw_part(partname, dent->name, dent->size, 0);
	}
	else if (flag == UPDATE_AB_PART) {
		get_prefix(dent->name, partname, 32);
		snprintf(a_partname, 32, "%s%s", partname, A_PART_SUFFIX);
		snprintf(b_partname, 32, "%s%s", partname, B_PART_SUFFIX);
		if ((0 != _update_mmc_raw_part(a_partname, dent->name, dent->size, 0)) || \
			(0 != _update_mmc_raw_part(b_partname, dent->name, dent->size, 0))) {
			return -1;
		}
	}
	else if (flag == UPDATE_BAK_PART) {
		get_prefix(dent->name, partname, 32);
		snprintf(bak_partname, 32, "%s%s", partname, BAK_PART_SUFFIX);
		if ((0 != _update_mmc_raw_part(partname, dent->name, dent->size, 0)) || \
			(0 != _update_mmc_raw_part(bak_partname, dent->name, dent->size, 0))) {
			return -1;
		}
	}
	else {
		printf("[USB Update] Unknow part update flag[%d]\n", flag);
		return -1;
	}

	return ret;
}

static char *usb_update_get_type(int flag)
{
	switch (flag) {
	case UPDATE_FULL_DISK:
		return "Full Disk";
	case UPDATE_SINGLE_PART:
		return "Single";
	case UPDATE_AB_PART:
		return "Single";
	case UPDATE_BAK_PART:
		return "Bak";
	default:
		return "Unknow";
	}
}

static int usb_update_process(const char *dirname)
{
	struct fs_dir_stream *dirs = NULL;
	struct fs_dirent *dent = NULL;
	int32_t ret = 0;
	int updated = 0;
	struct mmc *mmc;
	int image_idx;
	int part_type = 0;
	int mmc_dev = 0;
	int split_part_num = 0;
	char split_base[32] = {0};

	if (usb_update_fs_setup())
		return USB_UPDATE_SKIP;

	dirs = fs_opendir(dirname);
	if (!dirs) {
		printf("[USB update] Open folder [%s] failed, skip update\n", dirname);
		return USB_UPDATE_SKIP;
	}

	/* USB update Start Process */
	printf("[USB update] Start...\n");
	while ((dent = fs_readdir(dirs))) {
		if (0 == strcmp(dent->name, ".") || 0 == strcmp(dent->name, "..")) {
			continue;
		}
		else if (!parse_split_disk_image(dent->name, split_base,
						 sizeof(split_base),
						 &split_part_num, &mmc_dev))
		{
			if (split_part_num != 0)
				continue;

			mmc = init_mmc_device(mmc_dev, false, MMC_MODES_END);
			if (!mmc) {
				printf("mmc%d init faild\n", mmc_dev);
				ret = -1;
				goto usb_update_process_out;
			}
			usb_update_curr_mmc_dev = mmc_dev;
			ret = usb_update_mmc_split_disk(dirname, split_base);
			if (0 != ret) {
				printf("USB update: [%s] split failed\n", split_base);
				goto usb_update_process_out;
			}
			updated = 1;
		}
		else if ((image_idx = check_full_image(dent->name)) != -1)		// Full Image Update
		{
			mmc_dev = update_image_list[image_idx].mmc_dev;
			if (mmc_dev >= 0) {
				mmc = init_mmc_device(mmc_dev, false, MMC_MODES_END);
				if (!mmc) {
					printf("mmc%d init faild\n", mmc_dev);
					ret = -1;
					goto usb_update_process_out;
				}
				usb_update_curr_mmc_dev = mmc_dev;
			}
			if (NULL != update_image_list[image_idx].callback) {
				ret = update_image_list[image_idx].callback(dent, update_image_list[image_idx].flag);
				if (0 != ret) {
					printf("USB update: [%s] failed\n", update_image_list[image_idx].image_name);
					goto usb_update_process_out;
				}
				updated = 1;
			}
		}
		else if(0 == check_part_image(dent->name, &part_type, &mmc_dev))	// Part Image Update
		{
			mmc = init_mmc_device(mmc_dev, false, MMC_MODES_END);
			if (!mmc) {
				printf("mmc%d init faild\n", mmc_dev);
				ret = -1;
				goto usb_update_process_out;
			}
			usb_update_curr_mmc_dev = mmc_dev;
			printf("[USB update] MMC update part image: %s [%s] mmc%d\n",
			       dent->name, usb_update_get_type(part_type), mmc_dev);
			if(0 != usb_update_mmc_raw_image(dent, part_type)){
				printf("[USB update] usb_update_mmc_raw_image [%s] faild\n", dent->name);
				ret = -1;
				goto usb_update_process_out;
			}
			updated = 1;
		}
	}

	if (!updated) {
		printf("[USB update] No valid image found, skip update\n");
		ret = USB_UPDATE_NO_IMG;
	}
usb_update_process_out:
	fs_closedir(dirs);
	printf("[USB update] Finish...\n");

	return ret;
}

static int usb_update_check_ready(void)
{
	struct fs_dir_stream *dirs;

	if (usb_update_fs_setup())
		return USB_UPDATE_SKIP;

	dirs = fs_opendir(USB_UPDATE_FOLDER);
	if (!dirs) {
		printf("[USB update] Folder [%s] not found, skip update\n",
		       USB_UPDATE_FOLDER);
		return USB_UPDATE_SKIP;
	}
	fs_closedir(dirs);

	return 0;
}

static int do_usb_update(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	int ret;
	char *reset_reason = "COLD_BOOT";
	extern char usb_started;

	if (0 != strcmp(env_get("reset_reason"), reset_reason))
		return 0;

	bootstage_mark_name(BOOTSTAGE_ID_USB_START, "usb_start");

	if (!usb_started) {
		if (usb_init() < 0)
			return -1;
	}

	usb_stor_curr_dev = usb_stor_scan(1);
	if (usb_stor_curr_dev < 0) {
		printf("[USB update] No USB storage, skip update\n");
		return 0;
	}

	ret = usb_update_check_ready();
	if (ret == USB_UPDATE_SKIP)
		return 0;

	ret = usb_update_process(USB_UPDATE_FOLDER);
	if (ret == USB_UPDATE_SKIP || ret == USB_UPDATE_NO_IMG)
		return 0;

	printf("USB Update: %s ..............\n\n\n", (ret == 0) ? "Success" : "Fail");
	udelay(300000);
	printf("Now Reset Uboot\n");
	if (0 != run_command("reset", 1)) {
		printf("reset faild\n");
		return -1;
	}

	return 0;
}

#ifdef CONFIG_USB_STORAGE
#ifdef CONFIG_USB_UPDATE
U_BOOT_CMD(usbupdate,	1,	1,	do_usb_update,
	"update by USB storage",
	"Simply plug in the USB flash drive and update part"
);
#endif
#endif
