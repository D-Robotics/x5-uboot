/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2023 VeriSilicon Holdings Co., Ltd.
 */

#ifndef __X5_H__
#define __X5_H__

#include <asm/arch/hardware.h>

#define UART_CLK_FREQ			(100000000)
#define SD_EMMC_CLK_FREQ		(200000000)
#define QSPI_CLK_FREQ			(200000000)

#define CONFIG_SYS_MONITOR_LEN		(256 * 1024)

/*
 * We have two DDR memory range:
 * 0x0_80000000 - 0x0_80000000
 * 0x0_80000000 - 0x2_80000000
 */
#define PHYS_SDRAM_1			0x80000000
#define PHYS_SDRAM_1_SIZE		0x80000000

//#define PHYS_SDRAM_2			0x100000000
//#define PHYS_SDRAM_2_SIZE		0x180000000

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1

#define CONFIG_BOARD_EARLY_INIT_F

#define CONFIG_SYS_MMC_MAX_BLK_COUNT	1024

#define SD_BOOT

#define DTB_LOAD_ADDR           "0x81200000"
#define IMAGE_LOAD_ADDR         "0x84280000"

#define FDTFILE			"x5.dtb"
#define DEFAULT_BOOTARGS	"earlycon console=ttyS1,115200 rootwait rw rootfstype=ext4 panic=0"

#define CONFIG_EXTRA_ENV_SETTINGS	\
	"load_addr="  IMAGE_LOAD_ADDR "\0" \
	"dtb_load_addr="  DTB_LOAD_ADDR "\0" \
	"fdtfile=" FDTFILE "\0" \
	"default_bootargs=" DEFAULT_BOOTARGS "\0" \
	"boot_device=sd\0 " \
	"dev_index=0\0 " \
	"dev_name=mmc\0 " \
	"rootfspath=/dev/mmcblk${dev_index}p2\0 " \
	"max_cpus=8\0 " \
	"ip_dyn=yes\0 " \
	"netargs=" \
		"if test ${ip_dyn} = yes; then " \
			"echo getting ipaddr from dhcp server; " \
			"setenv get_cmd dhcp; " \
		"else " \
			"echo using static ipaddr; " \
			"setenv get_cmd tftp; " \
		"fi;\0 " \
	"dhcpboot=" \
		"run netargs; " \
		"echo Loading Image...; " \
		"${get_cmd} ${load_addr} ${serverip}:Image; " \
		"echo Loading DTB...; " \
		"${get_cmd} ${dtb_load_addr} ${serverip}:${fdtfile}; " \
		"run lnxbootargs; " \
		"booti ${load_addr} - ${dtb_load_addr}\0 " \
	"prepare_bootdev=" \
		"if test ${boot_device} = emmc; then " \
			"echo boot from emmc; " \
			"setenv dev_index 1; " /* emmc is 1 */ \
			"setenv dev_name mmc; " \
			"mmc dev ${dev_index}; " \
		"else " \
			"echo boot from sd; " \
			"setenv dev_index 0; "  /* sd is 0 */ \
			"setenv dev_name mmc; " \
			"mmc dev ${dev_index}; " \
		"fi;fi;\0 " \
	"updatebootargs=" \
		"setenv bootargs ${default_bootargs} maxcpus=${max_cpus} root=/dev/mmcblk${mmcdev}p2; " \
	"fatfsboot=" \
		"echo Fat fs booting from ${boot_device}:${dev_index} ...; " \
		"run prepare_bootdev; " \
		"echo load Image ...; " \
		"if fatload ${dev_name} ${dev_index} ${load_addr} Image; then " \
		"run updatebootargs; " \
		"echo load dtb ...; " \
		"fatload ${dev_name} ${dev_index} ${dtb_load_addr} ${fdtfile}; " \
		"booti ${load_addr} - ${dtb_load_addr}; fi\0 " \
	"usbboot=" \
		"setenv boot_device usb; " \
		"run fatfsboot\0 " \
	"sdboot=" \
		"setenv boot_device sd; " \
		"run fatfsboot\0 " \
	"emmcboot=" \
		"setenv boot_device emmc; " \
		"run fatfsboot\0 " \
	"update_sdmmcboot=" \
		"echo Loading flash.bin...; " \
		"run netargs; " \
		"if ${get_cmd} ${load_addr} ${serverip}:flash.bin; then " \
		"setexpr blkcnt ${filesize} + 0x1ff " \
		"setexpr blkcnt ${blkcnt} / 0x200; " \
		"run prepare_bootdev; " \
		"mmc write ${load_addr} 0x28 ${blkcnt}; " \
		"echo update mmc ${dev_index} done; fi;\0 " \
	"update_spiboot=" \
		"echo Loading flash.bin...; " \
		"run netargs; " \
		"if ${get_cmd} ${load_addr} ${serverip}:flash.bin; then " \
		"setexpr sectorcnt ${filesize} + 0xfff " \
		"setexpr sectorcnt ${sectorcnt} / 0x1000 " \
		"setexpr sectorcnt ${sectorcnt} * 0x1000; " \
		"sf probe; " \
		"sf update ${load_addr} 0 ${sectorcnt}; " \
		"echo update spi boot done; fi;\0 " \

#endif /* __X5_H__ */
