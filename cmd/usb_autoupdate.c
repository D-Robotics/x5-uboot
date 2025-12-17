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

#define WR_BLK_NUM		0x10000
#define BLK_SIZE		512
#define USB_UPDATE_FOLDER				"/update_img/"
#define USB_UPDATE_SPARSE_SZ_ONESHOT	(128 * 1024 * 1024)

#ifdef CONFIG_USB_STORAGE
static int usb_stor_curr_dev = -1; /* current device */
#endif

enum USB_UPDATE_TYPE_E{
	NAND_DISK,
	NAND_SINGLE_PART,
	EMMC_DISK,
	EMMC_SINGLE_PART,
	EMMC_AB_PART,
};

#define A_PART_SUFFIX	"_a"
#define B_PART_SUFFIX	"_b"

static int usb_update_mmc_raw_image(struct fs_dirent *dent, int flag);
static int usb_udpate_mmc_sparse_image(struct fs_dirent *dent, int flag);
static int usb_update_mtd_image(struct fs_dirent *dent, int flag);

typedef int (*USB_UPDATE_IMAGE_CALLBACK)(struct fs_dirent *dent, int flag);

typedef struct {
	char *image_name;
	char *boot_device;
	USB_UPDATE_IMAGE_CALLBACK callback;
	int flag;
} USB_UPDATE_IMAGE_LIST_T;

static USB_UPDATE_IMAGE_LIST_T update_image_list[] = {
	{"emmc_disk.img",  "emmc", usb_update_mmc_raw_image,    EMMC_DISK},
	{"emmc_disk.simg", "emmc", usb_udpate_mmc_sparse_image, EMMC_DISK},
	{"nand_disk.img",  "nand", usb_update_mtd_image,        NAND_DISK},
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

	if(flag == NAND_SINGLE_PART){
		strcpy(mtdparts, dent->name);
		token = strtok(mtdparts, ".");
	} else if(flag == NAND_DISK){
		strcpy(mtdparts, "spi-nand0");
	}
	printf("load %s to mtdparts %s\n", dent->name, mtdparts);
	memset(buffer, 0, sizeof(buffer));
	snprintf(buffer, sizeof(buffer), "fatload usb 0 ${kernel_addr} %s%s", USB_UPDATE_FOLDER, dent->name);
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
	snprintf(buffer, sizeof(buffer), "mtd write %s ${kernel_addr}", mtdparts);
	ret = run_command(buffer, 1);	if (ret) {
		printf("mtd write %s error\n", mtdparts);
		return -1;
	}
	printf("mtd write %s success\n", mtdparts);
	return 0;
}

static int get_mmc_partition_info(const char *name, struct disk_partition *info)
{
	int ret;
	struct blk_desc *dev_desc = NULL;
	int mmc_dev = 0;
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

	void *buffer_addr = (void *)env_get_ulong("kernel_addr", 16, 0x90000000);;
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
	dev_desc = blk_get_dev("mmc", 0);
	if (!dev_desc) {
		printf("blk_get_dev: mmc-%d failed\n", 0);
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

static int check_part_image(char *image_name, int *isABpart)
{
	char partname[32] = {0};
	char a_partname[32] = {0};
	char b_partname[32] = {0};
	struct disk_partition part_info = {0};

	if (0 != get_prefix(image_name, partname, 32)) {
		printf("[USB update] can not get prefix [%s]\n", image_name);
		return -1;
	}

	if (0 != get_mmc_partition_info(partname, &part_info)) {
		snprintf(a_partname, 32, "%s%s", partname, A_PART_SUFFIX);
		if (0 != get_mmc_partition_info(a_partname, &part_info)) {
			return -1;
		}
		snprintf(b_partname, 32, "%s%s", partname, B_PART_SUFFIX);
		if (0 != get_mmc_partition_info(b_partname, &part_info)) {
			return -1;
		}
		*isABpart = 1;
		printf("[USB update] found AB part[%s] [%s]\n", a_partname, b_partname);
		return 0;
	}
	*isABpart = 0;
	return 0;
}

static int _update_mmc_raw_part(char *part_name, char *image_name, loff_t image_size)
{
	int ret;
	struct disk_partition part_info = {0};

	loff_t remain_size = image_size;
	loff_t read_size = 0;
	loff_t oneshot_read_size = WR_BLK_NUM * BLK_SIZE;
	loff_t offset = 0;
	loff_t start_blk = 0, write_blk = 0;

	char command[128] = {0};

	if(0 != strcmp(part_name, "addr:0x0")){
		ret = get_mmc_partition_info(part_name, &part_info);
		if (ret != 0) {
			printf("[USB Update] get mmc [%s] partition info faild\n", part_name);
			return -1;
		}
		start_blk = part_info.start;
	}

	printf("\n\n[USB Update] Part[%s] Bgn: image_name[%s] image_size[%lld] start_blk[0x%llx]\n", part_name, image_name, image_size, start_blk);

	while (remain_size > 0) {
		read_size = (remain_size >= oneshot_read_size)? oneshot_read_size : remain_size;
		write_blk = (read_size + BLK_SIZE - 1) / BLK_SIZE;

		/* Read blocks from usb */
		memset(command, 0, sizeof(command));
		snprintf(command, sizeof(command), "fatload usb 0 ${kernel_addr} %s%s %llx %llx", USB_UPDATE_FOLDER, image_name, read_size, offset);
		if (0 != run_command(command, 1)) {
			printf("[USB Update] fatload %s from usb error: read_size[%lld] offset[%lld]\n", image_name, read_size, offset);
			return -1;
		}
		/* Write blocks into emmc */
		memset(command, 0, sizeof(command));
		snprintf(command, sizeof(command), "mmc write ${kernel_addr} 0x%llx 0x%llx", start_blk, write_blk);
		if (0 != run_command(command, 1)) {
			printf("[USB Update] mmc write part[%s] error: start_blk[0x%llx] write_blk[%lld]\n", part_name, start_blk, write_blk);
			return -1;
		}
		printf("[USB Update] Part[%s]: start_blk[0x%llx] read_size[%lld]\n", part_name, start_blk, read_size);

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

	if (flag == EMMC_DISK) {
		ret = _update_mmc_raw_part("addr:0x0", dent->name, dent->size);
	}
	else if (flag == EMMC_SINGLE_PART) {
		get_prefix(dent->name, partname, 32);
		ret = _update_mmc_raw_part(partname, dent->name, dent->size);
	}
	else if (flag == EMMC_AB_PART) {
		get_prefix(dent->name, partname, 32);
		snprintf(a_partname, 32, "%s%s", partname, A_PART_SUFFIX);
		snprintf(b_partname, 32, "%s%s", partname, B_PART_SUFFIX);
		if ((0 != _update_mmc_raw_part(a_partname, dent->name, dent->size)) || \
			(0 != _update_mmc_raw_part(b_partname, dent->name, dent->size))) {
			return -1;
		}
	}
	else {
		printf("[USB Update] Unknow part update flag[%d]\n", flag);
		return -1;
	}

	return ret;
}

static int usb_update_process(const char *dirname)
{
	struct fs_dir_stream *dirs = NULL;
	struct fs_dirent *dent = NULL;
	int32_t ret = -1;
	struct mmc *mmc;
	int image_idx;
	int isABpart = 0;

	dirs = fs_opendir(dirname);
	if (!dirs){
		printf("Not update mode!\n");
		return -errno;
	}

	/* Init Flash */
	if(0 == strcmp(env_get("boot_device"), "emmc")){
		mmc = init_mmc_device(0, false, MMC_MODES_END);
		if (!mmc){
			printf("mmc init faild\n");
			return -1;
		}
	}

	/* USB update Start Process */
	printf("[USB update] Start...\n");
	while ((dent = fs_readdir(dirs))) {
		if (0 == strcmp(dent->name, ".") || 0 == strcmp(dent->name, "..")) {
			continue;
		}
		else if ((image_idx = check_full_image(dent->name)) != -1)		// Full Image Update
		{
			if (0 != strcmp(update_image_list[image_idx].boot_device, env_get("boot_device"))) {
				printf("[usb_udpate] boot_device [%s] error, should be [%s]", \
							env_get("boot_device"), update_image_list[image_idx].boot_device);
				continue;
			}
			if (NULL != update_image_list[image_idx].callback) {
				ret = update_image_list[image_idx].callback(dent, update_image_list[image_idx].flag);
				if (0 != ret) {
					printf("USB update: [%s] failed\n", update_image_list[image_idx].image_name);
				}
				goto usb_update_process_out;
			}
		}
		else if(0 == check_part_image(dent->name, &isABpart))			// Part Image Update
		{
			if(0 == strcmp(env_get("boot_device"), "nand")) {
				printf("[USB update] MTD update part image: %s\n", dent->name);
				if(0 != usb_update_mtd_image(dent, NAND_SINGLE_PART)){
					printf("[USB update] usb_update_mtd_image [%s] faild\n", dent->name);
					ret = -1;
					goto usb_update_process_out;
				}
			}else if(0 == strcmp(env_get("boot_device"), "emmc")) {
				printf("[USB update] EMMC update part image: %s [%s]\n", dent->name, (0 == isABpart)?"Single":"AB");
				if(0 != usb_update_mmc_raw_image(dent, (0 == isABpart)? EMMC_SINGLE_PART : EMMC_AB_PART)){
					printf("[USB update] usb_update_mmc_raw_image [%s] faild\n", dent->name);
					ret = -1;
					goto usb_update_process_out;
				}
			}
		}
	}
	ret = 0;
usb_update_process_out:
	fs_closedir(dirs);
	printf("[USB update] Finish...\n");

	return ret;
}

static int do_usb_update(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	int ret;
	char *reset_reason = "COLD_BOOT";

	if(0 == strcmp(env_get("reset_reason"), reset_reason)){
		bootstage_mark_name(BOOTSTAGE_ID_USB_START, "usb_start");

		if (usb_init() < 0)
			return -1;

		/* Driver model will probe the devices as they are found */
		/* try to recognize storage devices immediately */
		usb_stor_curr_dev = usb_stor_scan(1);
		if(usb_stor_curr_dev == 0){
			if(fs_set_blk_dev("usb", "0", FS_TYPE_FAT)){
				return -1;
			}

			ret = usb_update_process(USB_UPDATE_FOLDER);
			printf("USB Update: %s ..............\n\n\n", (ret==0)? "Success" : "Fail");
			udelay(300000);
			printf("Now Reset Uboot\n");
			/* Reset */
			if (0 != run_command("reset", 1)) {
				printf("reset faild\n");
				return -1;
			}
		}
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
