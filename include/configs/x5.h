/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright(C) 2024, D-Robotics Co., Ltd. All rights reserved
 */

#ifndef __X5_H__
#define __X5_H__

#include <asm/arch/hardware.h>

#define UART_CLK_FREQ    (202752000)
#define SD_EMMC_CLK_FREQ (200000000)
#define QSPI_CLK_FREQ    (200000000)
#define WDT_CLK_FREQ    (200000000)
#define ADC_CLK_FREQ    (32000000)

#define CONFIG_SYS_MONITOR_LEN (256 * 1024)

#define PHY_ANEG_TIMEOUT 8000

/*
 * We have two DDR memory range:
 * 0x0_88000000 - 0x0_78000000
 * 0x1_00000000 - 0x0_80000000
 */
#define X5_DDR_BASE       0x80000000
#define PHYS_SDRAM_1      0x84000000
#define PHYS_SDRAM_1_SIZE 0x7c000000
#define DDR_RESERVED_SIZE      (PHYS_SDRAM_1 - X5_DDR_BASE)

#define PHYS_SDRAM_2      0x100000000
#define PHYS_SDRAM_2_SIZE 0x80000000

/* Keep in sync with kernel/arch/arm64/boot/dts/hobot/x5-memory.dtsi
 * the lowest entry with "no-map" bindings.
 */
#define X5_RESERVED_ADDR_LOWEST   (0x9FE70000)

#define DROBOT_UBOOT_OUTPUT_LOG_ADDR 0x87FFC000
#define DROBOT_RAMDISK_ADDR          0x87D00000
#ifdef CONFIG_CONSOLE_RECORD_OUT_SIZE
#define DROBOT_UBOOT_LOG_SIZE CONFIG_CONSOLE_RECORD_OUT_SIZE
#else
#define DROBOT_UBOOT_LOG_SIZE (0)
#endif
#define DROBOT_UBOOT_INPUT_LOG_ADDR (DROBOT_UBOOT_OUTPUT_LOG_ADDR + DROBOT_UBOOT_LOG_SIZE)
#define X5_DTB_LMB_START   (DROBOT_UBOOT_OUTPUT_LOG_ADDR - X5_DTB_LMB_SIZE)
#define X5_DTB_LMB_SIZE    0x1000000
#define KERNEL_SPACE_SIZE     (64 * 1024 * 1024)
#define CONFIG_SYS_SDRAM_BASE (PHYS_SDRAM_1 + KERNEL_SPACE_SIZE)

#define X5_UBOOT_USE_RAM_SIZE   (128 * 1024 *1024)
#define X5_USABLE_RAM_TOP     (CONFIG_SYS_SDRAM_BASE + X5_UBOOT_USE_RAM_SIZE)
#define CONFIG_BOARD_EARLY_INIT_F

/* Start of ion related macro */
#define ION_CMA_NAME "ion_cma"
#define ION_RESERVED_NAME "ion_reserved"
#define ION_CARVEOUT_NAME "ion_carveout"

/* ION_RESERVED_OFFSET must be in sync with x5-memory.dtsi */
/* Macros for ion regions in DDR *L*ess than or *E*qual to 2G */
#define DEFAULT_KERNEL_MIN_HEAP (0xC800000u)
#define DEFAULT_ION_REGION_START (0xA4100000u - PHYS_SDRAM_1)

#define DEFAULT_ION_TOTAL_SIZE_LE_2G (0x4B800000u) /* 1208MiB */
#define DEFAULT_ION_RESERVED_SIZE_LE_2G (0x20000000u) /* 512MiB */
#define DEFAULT_ION_CARVEOUT_SIZE_LE_2G (0x20000000u) /* 512MiB */
#define DEFAULT_ION_CMA_SIZE_LE_2G \
	(DEFAULT_ION_TOTAL_SIZE_LE_2G - DEFAULT_ION_RESERVED_SIZE_LE_2G - \
	DEFAULT_ION_CARVEOUT_SIZE_LE_2G) /* 184M */

/* Macros for ion regions in DDR *G*reater *T*han 2G */

#define DEFAULT_ION_TOTAL_SIZE_GT_2G (0xA0000000u) /* 2560MiB */

#define DEFAULT_ION_RESERVED_SIZE_GT_2G (0x40000000u) /* 1GiB */
#define DEFAULT_ION_CARVEOUT_SIZE_GT_2G (0x40000000u) /* 1GiB */
#define DEFAULT_ION_CMA_SIZE_GT_2G \
	(DEFAULT_ION_TOTAL_SIZE_GT_2G - DEFAULT_ION_RESERVED_SIZE_GT_2G - \
	DEFAULT_ION_CARVEOUT_SIZE_GT_2G) /* 512MiB */
/* End of ion related macro */

#define CONFIG_SYS_MMC_MAX_BLK_COUNT 1024

#ifdef CONFIG_DISTRO_DEFAULTS
#define FDT_ADDR                0x84000000
#define BOOTSCR_ADDR			0x84100000
#define KERNEL_ADDR             0x85000000
//#define RAMDISK_ADDR			0x91000000

#define ENV_MEM_LAYOUT_SETTINGS \
	"kernel_addr_r="__stringify(KERNEL_ADDR)"\0" \
	"scriptaddr="__stringify(BOOTSCR_ADDR)"\0" \
	"fdt_addr_r="__stringify(FDT_ADDR)"\0" \
	//"ramdisk_addr_r="__stringify(RAMDISK_ADDR)"\0"

#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 0) \
	func(MMC, mmc, 1) \
	func(MMC, mmc, 2)

/*#define CONFIG_BOOTCOMMAND "if sd_detect; then run distro_bootcmd; " \
	"else echo SD card not detected; fi; " \
	"echo Boot from eMMC or SD Card failed"*/
#define CONFIG_BOOTCOMMAND "run distro_bootcmd; "
#else

#define KERNEL_ADDR     __stringify(0x90000000)
#define KERNEL_SIZE     __stringify(0x1000000)
#define FDT_ADDR        __stringify(0x88000000)
#define FDT_SIZE        __stringify(0x100000)
#define FDT_HIGH_ADDR   __stringify(DROBOT_UBOOT_OUTPUT_LOG_ADDR)
#define INITRD_HIGH_ADDR   __stringify(DROBOT_RAMDISK_ADDR)

#define DFU_MMC_SIZE    __stringify(0x400000) // 2G/512 blks, enlarge it if not enough

#define CONFIG_BOOTCOMMAND "run ab_select_cmd;" \
	"run avb_boot_test;"

#endif


#include <config_distro_bootcmd.h>

#if defined(CONFIG_CMD_AB_SELECT)
/**
 * When ab_select failed, it means all slot are invalid.
 * Reboot to recovery mode when this occured.
 */
#define AB_SELECT                               \
  "btype ab_select bootslot ||"         \
  "  run enter_safety_cmd;"                     \
  "  echo \"Using slot ${bootslot}\";"          \
  "  setenv slot_suffix _${bootslot};"
#else
#define AB_SELECT "setenv slot_suffix _a;"
#endif

#define ENV_AB_SELECT_CMD                         \
    "ab_select_cmd="                            \
    "btype ab_select bootslot ||"                 \
    "  run enter_safety_cmd;"                     \
    "  echo \"Using slot ${bootslot}\";"          \
    "  setenv slot_suffix _${bootslot};\0"
#define ENV_AVB_VERIFY_BOOT_CMD                   \
    "avb_boot="                                 \
        "hb_avb_helper $dev_name $dev_index ${slot_suffix};\0"

#define ENV_ENTER_SAFETY_CMD                       \
    "enter_safety_cmd="                           \
        "echo \"enter safety mode\"; btype reboot recovery;\0"

#define DEFAULT_BOOTARGS "earlycon console=ttyS0,115200 rootwait rw ignore_loglevel panic=0"

#define X5_DEFAULT_BOOTARGS " "

#define X5_PREPARE_BOOT_DEV                \
    "prepare_bootdev="                     \
    "if test ${boot_device} = emmc; then " \
    "echo boot from emmc; "                \
    "setenv dev_index 0; " /* emmc is 0 */ \
    "setenv dev_name mmc; "                \
    "mmc dev ${dev_index}; "               \
    "else "                                \
    "echo boot from sd; "                  \
    "setenv dev_index 1; " /* sd is 1 */   \
    "setenv dev_name mmc; "                \
    "mmc dev ${dev_index}; "               \
    "fi;\0"

#define X5_DFU_ALT_INFO_RAM                             \
    "dfu_alt_info_ram="                                 \
    "kernel ram" KERNEL_ADDR KERNEL_SIZE ";"            \
    "fdt ram" FDT_ADDR FDT_SIZE "\0"                    \

#define X5_DFU_RAM_ENV                                  \
    "dfu_ram_info="                                     \
    "setenv dfu_alt_info \"${dfu_alt_info_ram}\"\0"     \
    "dfu_ram=run dfu_ram_info && dfu 0 ram 0\0"

#define X5_DFU_ALT_INFO_SDCARD                          \
    "dfu_alt_info_sdcard="                              \
    "emmc0_disk.img raw 0x0 " DFU_MMC_SIZE "\0"

#define X5_DFU_ALT_INFO_EMMC                            \
    "dfu_alt_info_emmc="                                \
    "emmc1_disk.img raw 0x0 " DFU_MMC_SIZE "\0"

#define X5_DFU_EMMC_ENV                                 \
    "dfu_emmc_info="                                    \
    "setenv dfu_alt_info \"${dfu_alt_info_emmc}\"\0"    \
    "dfu_emmc=run dfu_emmc_info && dfu 0 mmc 1\0"

#define X5_DFU_SDCARD_ENV                               \
    "dfu_sdcard_info="                                  \
    "setenv dfu_alt_info \"${dfu_alt_info_sdcard}\"\0"  \
    "dfu_sdcard=run dfu_sdcard_info && dfu 0 mmc 0\0"

#define X5_USB_BOOT            \
    "usbboot="                 \
    "setenv boot_device usb; " \
    "run fitboot;\0"

#define X5_SD_BOOT            \
    "sdboot="                 \
    "setenv boot_device sd; " \
    "run fitboot;\0"

#define X5_EMMC_BOOT            \
    "emmcboot="                 \
    "setenv boot_device emmc; " \
    "run fitboot;\0"

#define X5_UPDATE_NANDFS						\
    "update_nandfs="                                                    \
    "echo loading ubifs...;"                                            \
    "setenv mtdids spi-nand0=nand0;"                                    \
    "setenv mtdparts "                                                  \
    "mtdparts=nand0:${nand_fs_size}@${nand_uboot_max_addr}(rootfs);"    \
    "mtd list;"                                                         \
    "mtdparts;"                                                         \
    "run netargs; "                                                  	\
    "if ${get_cmd} ${kernel_addr} ${serverip}:x5.ubifs; then "          \
    "setexpr blocksize ${filesize} + 0x1ffff; "                         \
    "setexpr blocksize ${blocksize} / 0x20000; "                        \
    "setexpr blocksize ${blocksize} * 0x20000; "                        \
    "echo blocksize: ${blocksize}; "                    	 	\
    "mtd erase spi-nand0 ${nand_uboot_max_addr} ${nand_fs_size}; "   	\
    "ubi part rootfs;"                                                  \
    "ubi create rootfs;"                                                \
    "ubi write ${kernel_addr} rootfs ${blocksize}; "                    \
    "echo update ubifs done; "                                          \
    "fi;\0"

#define X5_UPDATE_NORFS                                                                         \
    "update_norfs="                                                                             \
    "echo loading ubifs...;"                                            \
    "setenv mtdids nor0=nor0;"                                                                  \
    "setenv mtdparts "                                                                          \
    "mtdparts=nor0:${nor_fs_size}@${nor_uboot_max_addr}(rootfs);"                               \
    "mtd list;"                                                                                 \
    "mtdparts;"                                                                                 \
    "run netargs;"                                                                              \
    "if ${get_cmd} ${kernel_addr} ${serverip}:x5.ubifs; then "                                  \
    "setexpr sectorcnt ${filesize} + 0xfff; "                                                   \
    "setexpr sectorcnt ${sectorcnt} / 0x1000; "                                                 \
    "setexpr sectorcnt ${sectorcnt} * 0x1000; "                                                 \
    "echo sectorcnt: ${sectorcnt}; "                    	 	\
    "mtd erase nor0 ${nor_uboot_max_addr} ${nor_fs_size}; "                                          \
    "ubi part rootfs;"                                                                          \
    "ubi create rootfs;"                                                                        \
    "ubi write ${kernel_addr} rootfs ${sectorcnt};"                                             \
    "echo update ubifs done; "                                                                  \
    "fi;\0"

#define X5_MEM_BOOT                               \
    "memboot="                                    \
    "echo Boot from ${kernel_addr} ${fdt_addr}; " \
    "run updatebootargs; "                        \
    "if test ${boot_fit} = yes ; then "           \
    "bootm ${kernel_addr}; "                      \
    "else "                                       \
    "booti ${kernel_addr} - ${fdt_addr}; "        \
    "fi;\0"

#define X5_NET_ARGS                          \
    "netargs="                               \
    "if test ${ip_dyn} = yes; then "         \
    "echo getting ipaddr from dhcp server; " \
    "setenv get_cmd dhcp; "                  \
    "else "                                  \
    "echo using static ipaddr; "             \
    "setenv get_cmd tftp; "                  \
    "fi;\0"

#define X5_UPDATE_BOOTARGS                                \
    "updatebootargs="                                     \
    "if test ${dev_name} = mmc; then "                    \
    "btype bootargs "                                     \
    "else "                                               \
    "if test ${dev_name} = mtd; then "                    \
    "setenv bootargs ${x5_bootargs} maxcpus=${max_cpus} " \
    "rootfstype=ubifs ubi.mtd=0 root=ubi0:rootfs;"        \
    "fi;"                                                 \
    "fi;\0"

#define X5_FATFS_BOOT                                                          \
    "fatfsboot="                                                               \
    "echo Fat fs booting from ${boot_device}:${dev_index} ...; "               \
    "run prepare_bootdev; "                                                    \
    "if test ${boot_fit} = yes ; then "                                        \
    "echo load ${fit_name} ...; "                                              \
    "if fatload ${dev_name} ${dev_index} ${kernel_addr} ${fit_name}; then "    \
    "run memboot; "                                                            \
    "fi; "                                                                     \
    "else "                                                                    \
    "echo load ${kernel_name} ...; "                                           \
    "if fatload ${dev_name} ${dev_index} ${kernel_addr} ${kernel_name}; then " \
    "echo load dtb ...; "                                                      \
    "if fatload ${dev_name} ${dev_index} ${fdt_addr} ${fdt_name}; then "       \
    "run memboot; "                                                            \
    "fi; "                                                                     \
    "fi; "                                                                     \
    "fi;\0"

#define X5_FIT_BOOT \
    "fitboot=" \
        "echo Fit booting from ${boot_device}:${dev_index} ...; " \
        "run prepare_bootdev; " \
        "if test ${boot_fit} = yes ; then " \
            "echo load boot.img; " \
            "part size mmc 0 boot_a bootimagesize; " \
            "part start mmc 0 boot_a bootimageblk; " \
            "mmc read ${kernel_addr} ${bootimageblk} ${bootimagesize};" \
            "run memboot; " \
        "fi;\0"

#define X5_NOR_BOOT                                                            \
    "norboot="                                                                 \
    "setenv boot_device nor; "                                                 \
    "setenv mtdids nor0=nor0;"                                                 \
    "setenv mtdparts "                                                         \
    "mtdparts=nor0:${nor_fs_size}@${nor_uboot_max_addr}(rootfs);"              \
    "mtd list;"                                                                                    \
    "mtdparts;"                                                                                    \
    "ubi part rootfs;"                                                                             \
    "ubifsmount ubi0:rootfs;"                                                                                    \
    "if test ${boot_fit} = yes ; then "                                                         \
    "echo load ${fit_name} ...; "                                                               \
    "if ubifsload ${kernel_addr} /etc/${fit_name}; then " \
    "run memboot; "                                                                             \
    "fi; "                                                                                      \
    "fi;\0"

#define X5_NAND_BOOT                                                                            \
    "nandboot="                                                                                 \
    "setenv boot_device nand; "                                                                 \
    "setenv mtdids spi-nand0=nand0;"                                                               \
    "setenv mtdparts "                                                                             \
    "mtdparts=nand0:${nand_fs_size}@${nand_uboot_max_addr}(rootfs);"    				   \
    "mtd list;"                                                                                    \
    "mtdparts;"                                                                                    \
    "ubi part rootfs;"                                                                             \
    "ubifsmount ubi0:rootfs;"                                                                                    \
    "if test ${boot_fit} = yes ; then "                                                         \
    "echo load ${fit_name} ...; "                                                               \
    "if ubifsload ${kernel_addr} /etc/${fit_name}; then " \
    "run memboot; "                                                                             \
    "fi; "                                                                                      \
    "fi;\0"

#define X5_DHCPBOOT                                                  \
    "dhcpboot="                                                      \
    "run netargs; "                                                  \
    "run prepare_bootdev; "                                          \
    "if test ${boot_fit} = yes ; then "                              \
    "echo load ${fit_name} ...; "                                    \
    "if ${get_cmd} ${kernel_addr} ${serverip}:${fit_name}; then "    \
    "run memboot; "                                                  \
    "fi; "                                                           \
    "else "                                                          \
    "echo Loading ${kernel_name}...; "                               \
    "if ${get_cmd} ${kernel_addr} ${serverip}:${kernel_name}; then " \
    "echo Loading DTB...; "                                          \
    "${get_cmd} ${fdt_addr} ${serverip}:${fdt_name}; "               \
    "run memboot; "                                                  \
    "fi; "                                                           \
    "fi;\0"

#define X5_UPDATE_MMCDEV                                        \
    "update_mmcdev="                                            \
    "echo Loading ${p_n}.img...; "                              \
    "run netargs; "                                             \
    "if ${get_cmd} ${kernel_addr} ${serverip}:${p_n}.img; then " \
        "echo get offset and size...; " \
        "setexpr blkcnt ${filesize} / 0x200; " \
        "if test ${p_n} = bl2; then "                      \
            "part start mmc ${dev_index} ${p_n} p_s; " \
        "else "                                          \
            "part start mmc ${dev_index} ${p_n}_a p_s; " \
        "fi; "                                                     \
        "run prepare_bootdev; "                                     \
        "if test ${boot_device} = emmc; then "                      \
            "echo update main partition; "                              \
            "mmc dev ${dev_index}; "                                    \
            "mmc write ${kernel_addr} ${p_s} ${blkcnt}; "               \
            "echo update mmc ${dev_index} partition done; "         \
        "else "                                                     \
            "mmc write ${kernel_addr} ${p_s} ${blkcnt}; " \
            "echo update sd ${dev_index} done; "                        \
        "fi; "                                                      \
    "fi;\0"

#define X5_UPDATE_EMMCBOOT       \
    "update_emmcboot="           \
    "echo update emmc boot...; " \
    "setenv boot_device emmc; "  \
    "run update_mmcdev;\0"

#define X5_UPDATE_SDBOOT       \
    "update_sdboot="           \
    "echo update sd boot...; " \
    "setenv boot_device sd; "  \
    "run update_mmcdev;\0"

#define X5_UPDATE_NORBOOT                                       \
    "update_norboot="                                           \
    "echo Loading flash_nor.bin...; "                               \
    "run netargs; "                                             \
    "if ${get_cmd} ${kernel_addr} ${serverip}:flash_nor.bin; then " \
    "if test 0x${filesize} -gt ${nor_uboot_max_addr}; then "        \
    "echo Error: uboot size is larger than vloume; "            \
    "else "                                                     \
    "setexpr sectorcnt ${filesize} + 0xfff && setexpr sectorcnt ${sectorcnt} / 0x1000 && setexpr sectorcnt ${sectorcnt} * 0x1000; "                   \
    "sf probe; "                                                \
    "sf update ${kernel_addr} 0 ${sectorcnt}; "                 \
    "echo update spi nor boot done; "                           \
    "fi; "                                                      \
    "fi;\0"

#define X5_UPDATE_NANDBOOT                                           \
    "update_nandboot="                                               \
    "echo Loading ${p_n}.img...; "                               \
    "run netargs; "                                                  \
    "if ${get_cmd} ${kernel_addr} ${serverip}:${p_n}.img; then " \
    "setexpr blocksize ${filesize} + 0x1ffff; "                      \
    "setexpr blocksize ${blocksize} / 0x20000; "                     \
    "setexpr blocksize ${blocksize} * 0x20000; "                     \
    "mtd erase.dontskipbad ${p_n} 0 ${blocksize}; "                           \
    "mtd write ${p_n} ${kernel_addr} 0 ${blocksize}; "        \
    "echo update spi nand ${p_n}.img done; "                               \
    "fi;\0"

#define X5_TFTP_BJ2 \
	"tftp_bj2=" \
		"echo set tftp to bj2; " \
		"setenv ip_dyn no; " \
		"setenv serverip 192.168.1.123; " \
		"setenv ipaddr 192.168.1.100; \0"

#ifdef CONFIG_DISTRO_DEFAULTS
#define CONFIG_EXTRA_ENV_SETTINGS \
    ENV_MEM_LAYOUT_SETTINGS \
	BOOTENV
#else
#define CONFIG_EXTRA_ENV_SETTINGS                                                       \
    "bootdelay=3\0"                                                                     \
    "boot_fit=yes\0"                                                                    \
    "sdemmc_boot=yes\0"                                                                 \
    "fdt_addr=" FDT_ADDR "\0"                                                           \
    "fdt_size=" FDT_SIZE "\0"                                                           \
    "kernel_addr=" KERNEL_ADDR "\0"                                                     \
    "nor_uboot_max_addr=0x400000\0"                                                     \
    "nor_fs_size=0x1C00000\0"                                                           \
    "nand_uboot_max_addr=0x600000\0"                                                    \
    "nand_fs_size=0x6000000\0"                                                      \
    "kernel_size=" KERNEL_SIZE "\0"                                                     \
    "fdt_name=x5.dtb\0"                                                                 \
    "kernel_name=Image\0"                                                               \
    "fit_name=boot.img\0"                                                               \
    "boot_device=sd\0 "                                                                 \
    "dev_index=0\0 "                                                                    \
    "dev_name=mmc\0 "                                                                   \
    "max_cpus=1\0 "                                                                     \
    "kernel_comp_addr_r=0x91000000\0"                                                   \
    "kernel_comp_size=0x4000000\0"                                                      \
    "ip_dyn=yes\0 "                                                                     \
    "cdc_connect_timeout=360\0"                                                         \
    "serial#=sunrise5_0000\0"                                                           \
    "fdt_high=" FDT_HIGH_ADDR "\0"                                                      \
    "initrd_high=" INITRD_HIGH_ADDR "\0"                                                      \
	"enable_cpu_18g=yes\0 "                                                             \
    X5_PREPARE_BOOT_DEV  X5_NET_ARGS X5_DHCPBOOT X5_UPDATE_MMCDEV                       \
    X5_UPDATE_EMMCBOOT X5_UPDATE_SDBOOT X5_UPDATE_NORBOOT X5_UPDATE_NANDBOOT            \
    X5_DFU_ALT_INFO_RAM X5_DFU_ALT_INFO_EMMC X5_DFU_ALT_INFO_SDCARD                     \
    X5_DFU_RAM_ENV X5_DFU_EMMC_ENV X5_DFU_SDCARD_ENV                                    \
    X5_TFTP_BJ2 X5_FIT_BOOT                                                             \
    ENV_AB_SELECT_CMD                                                                   \
    ENV_AVB_VERIFY_BOOT_CMD                                                             \
    ENV_ENTER_SAFETY_CMD                                                                \
    X5_MEM_BOOT                                                                         \
    X5_UPDATE_BOOTARGS                                                                  \
    X5_NOR_BOOT  X5_UPDATE_NORFS                                    \
    X5_NAND_BOOT X5_UPDATE_NANDFS
#endif 
#endif /* __X5_H__ */
