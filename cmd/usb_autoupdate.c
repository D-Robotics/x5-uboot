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

#define UPDATE_MODE 2
#define RECOVERY_MODE 0x010000
#define NAND_DISK 1
#define SINGLE_PARTITION 2

#ifdef CONFIG_USB_STORAGE
static int usb_stor_curr_dev = -1; /* current device */
#endif

int mtd_write_image(struct fs_dirent *dent, const char *dirname, int flag)
{
	char *token = NULL;
	char buffer[128] = {0};
	char mtdparts[20];
	int32_t ret = 0;

	if(flag == SINGLE_PARTITION){
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

int fs_read_image(const char *dirname, struct fs_dir_stream *dirs)
{
	struct fs_dirent *dent;
	int32_t ret = -1;

	dirs = fs_opendir(dirname);
	if (!dirs){
		printf("Not update mode!\n");
		return -errno;
	}

	printf("start update partition\n");
	while ((dent = fs_readdir(dirs))) {
		if(0 == strcmp(dent->name, "boot.img")
			|| 0 == strcmp(dent->name, "system.img")
			|| 0 == strcmp(dent->name, "hbre.img")
			|| 0 == strcmp(dent->name, "userdata.img")){
			ret = mtd_write_image(dent, dirname, SINGLE_PARTITION);
			if(-1 == ret){
				printf("mtd write %s faild\n", dent->name);
				fs_closedir(dirs);
				return ret;
			}
		}else if(0 == strcmp(dent->name, "nand_disk.img")){
			ret = mtd_write_image(dent, dirname, NAND_DISK);
			if(-1 == ret){
				printf("mtd write %s faild\n", dent->name);
				fs_closedir(dirs);
				return ret;
			}
			goto finally;
		}
	}
finally:
	fs_closedir(dirs);
	fs_close();
	return 0;
}

static int do_usb_update(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	char *reset_reason = "COLD_BOOT";
	char *boot_device = "nand";
	struct fs_dir_stream *dirs = NULL;

	if(0 == strcmp(env_get("reset_reason"), reset_reason) && 0 == strcmp(env_get("boot_device"), boot_device)){
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
