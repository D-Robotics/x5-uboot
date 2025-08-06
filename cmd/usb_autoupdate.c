#include <common.h>
#include <command.h>
#include <bootstage.h>
#include <usb.h>
#include <env.h>
#include <hb_info.h>
#include <fat.h>
#include <fs.h>
#include <string.h>
#include <fastboot.h>
#include <asm/arch/hb_board.h>
#include <linux/mtd/mtd.h>
#include <mtd.h>
#include <jffs2/load_kernel.h>
#include <linux/ctype.h>
#include <blk.h>
#include <part.h>
#include <fb_mmc.h>
#include <linux/types.h>
#include <mmc.h>

#define UPDATE_MODE 2
#define RECOVERY_MODE 0x010000
#define FASTBOOT_RESPONSE_LEN	(64 + 1)
#define BLKSIZE 0x10000

#ifdef CONFIG_USB_STORAGE
static int usb_stor_curr_dev = -1; /* current device */
#endif

enum {
	NAND_DISK,
	NAND_SINGLE_PART,
	EMMC_DISK,
	EMMC_SINGLE_PART,
};

int mtd_write_image(struct fs_dirent *dent, const char *dirname, int flag)
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
	snprintf(buffer, sizeof(buffer), "fatload usb 0 ${kernel_addr} %s%s", dirname, dent->name);
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

int mmc_write_image(struct fs_dirent *dent, const char *dirname, int flag)
{
	int32_t ret = 0;
	char *token = NULL;
	char mmcparts[20];
	char buffer[128] = {0};
	loff_t bytes = BLKSIZE * 512;
	loff_t pos = 0;
	loff_t load_size = dent->size;
	loff_t start_blk = 0;
	loff_t cnt = 0;

	struct disk_partition part_info = {0};

	if(flag == EMMC_SINGLE_PART){
		strcpy(mmcparts, dent->name);
		token = strtok(mmcparts, ".");
	} else if(flag == EMMC_DISK){
		strcpy(mmcparts, "addr:0x0");
		start_blk = 0;
	}
	printf("load %s to mmcparts %s\n", dent->name, mmcparts);
	memset(buffer, 0, sizeof(buffer));

	if(token != NULL){
		ret = get_mmc_partition_info(token, &part_info);
		if (ret != 0) {
			printf("get mmc [%s] partition info faild\n", token);
			return -1;
		}
		start_blk = part_info.start;
		printf("token %s, start_blk 0x%llx\n", token, start_blk);
	}
	cnt = (load_size + 511) / 512;

	while(load_size > bytes){
		snprintf(buffer, sizeof(buffer), "fatload usb 0 ${kernel_addr} %s%s %llx %llx", dirname, dent->name, bytes, pos);
		ret = run_command(buffer, 1);	if (ret) {
			printf("fatload %s from usb failed\n", dent->name);
			return -1;
		}
		memset(buffer, 0, sizeof(buffer));
		snprintf(buffer, sizeof(buffer), "mmc write ${kernel_addr} 0x%llx 0x%x", start_blk, BLKSIZE);
		ret = run_command(buffer, 1);	if (ret) {
			printf("fatload %s from usb failed\n", dent->name);
			return -1;
		}

		pos += bytes;
		load_size -= bytes;
		start_blk += BLKSIZE;
		cnt -= BLKSIZE;
		memset(buffer, 0, sizeof(buffer));
	}
	if(load_size > 0){
		printf("finally load %s to %s partition start, %lld, pos %lld, size %lld\n", dent->name, mmcparts, bytes, pos, load_size);
		snprintf(buffer, sizeof(buffer), "fatload usb 0 ${kernel_addr} %s%s %llx %llx", dirname, dent->name, load_size, pos);
		ret = run_command(buffer, 1);	if (ret) {
			printf("fatload %s from usb failed\n", dent->name);
			return -1;
		}

		memset(buffer, 0, sizeof(buffer));
		snprintf(buffer, sizeof(buffer), "mmc write ${kernel_addr} 0x%llx 0x%llx", start_blk, cnt);
		ret = run_command(buffer, 1);	if (ret) {
			printf("fatload %s from usb failed\n", dent->name);
			return -1;
		}
	}

	printf("load %s to %s partition success\n", dent->name, mmcparts);
	memset(buffer, 0, sizeof(buffer));
	return 0;
}

int fs_read_image(const char *dirname, struct fs_dir_stream *dirs)
{
	struct fs_dirent *dent;
	const char *nand_device = "nand";
	const char *emmc_device = "emmc";
	int32_t ret = -1;
	struct mmc *mmc;
	char buffer[128] = {0};

	dirs = fs_opendir(dirname);
	if (!dirs){
		printf("Not update mode!\n");
		return -errno;
	}

	if(0 == strcmp(env_get("boot_device"), emmc_device)){
		mmc = init_mmc_device(0, false, MMC_MODES_END);
		if (!mmc){
			printf("mmc init faild\n");
			return -1;
		}
	}
	printf("start update partition\n");
	while ((dent = fs_readdir(dirs))) {
		if(0 == strcmp(dent->name, "boot.img")
			|| 0 == strcmp(dent->name, "system.img")
			|| 0 == strcmp(dent->name, "hbre.img")
			|| 0 == strcmp(dent->name, "app.img")
			|| 0 == strcmp(dent->name, "userdata.img"))
			{
				if(0 == strcmp(env_get("boot_device"), nand_device)){
					printf("nand write %s\n", dent->name);
					ret = mtd_write_image(dent, dirname, NAND_SINGLE_PART);
					if(-1 == ret){
						printf("mtd write %s faild\n", dent->name);
						fs_closedir(dirs);
						return ret;
					}
				}else if(0 == strcmp(env_get("boot_device"), emmc_device)){
					printf("emmc load %s, size %lld\n", dent->name, dent->size);
					ret = mmc_write_image(dent, dirname, EMMC_SINGLE_PART);
					if(-1 == ret){
						printf("emmc write %s faild\n", dent->name);
						fs_closedir(dirs);
						return ret;
					}
				}
			}else if((0 == strcmp(dent->name, "nand_disk.img")) || (0 == strcmp(dent->name, "emmc_disk.img"))){
				if(0 == strcmp(env_get("boot_device"), nand_device)){
					ret = mtd_write_image(dent, dirname, NAND_DISK);
					if(-1 == ret){
						printf("mtd write %s faild\n", dent->name);
						fs_closedir(dirs);
						return ret;
					}
				}else if(0 == strcmp(env_get("boot_device"), emmc_device)){
					ret = mmc_write_image(dent, dirname, EMMC_DISK);
					if(-1 == ret){
						printf("emmc write %s faild\n", dent->name);
						fs_closedir(dirs);
						return ret;
					}
				}
				 goto finally;
			}
	}
finally:
	fs_closedir(dirs);
	fs_close();
	snprintf(buffer, sizeof(buffer), "reset");
	ret = run_command(buffer, 1);	if (ret) {
		printf("reset faild\n");
		return -1;
	}
	return 0;
}

static int do_usb_update(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	char *reset_reason = "COLD_BOOT";
	struct fs_dir_stream *dirs = NULL;

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
			fs_read_image("/update_img/", dirs);
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
