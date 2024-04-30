/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2023 Horizon Robotics Co,. Ltd
 */
#ifndef __ASM_ARCH_HORIZON_AON_H__
#define __ASM_ARCH_HORIZON_AON_H__

#define AON_STATUS_REG_BASE         0x31021000
/*
* AON_STATUS_REG1
* bit[0:7] boot count
*/
#define AON_STATUS_REG0         (AON_STATUS_REG_BASE + 0x00)
#define AON_BOOTCOUNT_OFFSET  (0)
#define AON_BOOTCOUNT_MASK    (0xFFL)
/*
AON_STATUS_REG1
bit0~bit3 : ab slot
bit4~bit7: reset reason 0(cold boot) 1 (watchdog) 2(reboot command) 3(panic in kernel) 4(uboot reset)
bit8~bit11: 0(NORMAL) 1(usb3) 2(usb2) 3(uart) 4(RECOERY)
bit12~bit15: 0(NORMAL) 1(ubootonce) 2(udumpfastboot)
bit16~bit19: 0(dual pmic) 1(single pmic)
*/
#define AON_STATUS_REG1         (AON_STATUS_REG_BASE + 0x04)
#define AON_AB_SLOT_OFFSET  (0)
#define AON_AB_SLOT_MASK    (0x0FL)
#define AON_AB_SLOT_VALUE(x)    ((x >> AON_AB_SLOT_OFFSET) & AON_AB_SLOT_MASK)

#define AON_RESET_REASON_OFFSET  (4)
#define AON_RESET_REASON_MASK    (0x0FL)
#define AON_RESET_REASON_VALUE(x)    ((x >> AON_RESET_REASON_OFFSET) & AON_RESET_REASON_MASK)
#define AON_RESET_REASON_WATCHDOG   (1 << AON_RESET_REASON_OFFSET)
#define AON_RESET_REASON_UBOOT   (4 << AON_RESET_REASON_OFFSET)

#define AON_BOOT_ACTION_OFFSET  (8)
#define AON_BOOT_ACTION_MASK    (0x0FL)
#define AON_BOOT_ACTION_VALUE(x)    ((x >> AON_BOOT_ACTION_OFFSET) & AON_BOOT_ACTION_MASK)

#define AON_PANIC_ACTION_OFFSET  (12)
#define AON_PANIC_ACTION_MASK    (0x0FL)
#define AON_PANIC_ACTION_VALUE(x)    ((x >> AON_PANIC_ACTION_OFFSET) & AON_PANIC_ACTION_MASK)

#define AON_PMIC_TYPE_OFFSET  (16)
#define AON_PMIC_TYPE_MASK    (0x0FL)
#define AON_PMIC_TYPE_VALUE(x)    ((x >> AON_PANIC_ACTION_OFFSET) & AON_PANIC_ACTION_MASK)


/*
AON_STATUS_REG3 use for ddr informaion
bit0~bit3 : vendor
bit4~bit7: ddr size 0(1G) 1 (2G) 2(4G) 4(8G)
bit8~bit9: 0(lpddr4) 1(lpddr4x)
bit10: 0: disable inline ecc 1: enable inline ecc
*/
#define AON_STATUS_REG3   0x31021008
#define DDR_SIZE_BIT_OFFSET (4)
#define DDR_SIZE_BIT_MASK (0xf << DDR_SIZE_BIT_OFFSET)

#define DDR_ECC_BIT_OFFSET (10)
#define DDR_ECC_BIT_MASK (0x1 << DDR_ECC_BIT_OFFSET)
enum boot_action {
    BOOT_NORMAL = 0,
    BOOT_DEVICE_USB3,
    BOOT_DEVICE_USB2,
    BOOT_DEVICE_UART,
    BOOT_RECOVERY,
};

enum panic_action {
    PANIC_NORMAL = 0,
    PANIC_UBOOT_ONCE,
    PANIC_UDUMP_FASTBOOT,
};

#define PLAT_AON_CRM_REG_BASE                       0x31020000
#define PLAT_AON_CRM_REG_SIZE                       0x10000
#define PLAT_AON_CRM_SOFT_RST_REG                   (PLAT_AON_CRM_REG_BASE + 0x14)
#define PLAT_AON_CRM_SOFT_RST                       (1L << 0)

enum USB_UART_MODE {
    USB30_MODE = 1,
    USB20_MODE,
    UART_MODE
};
#endif
