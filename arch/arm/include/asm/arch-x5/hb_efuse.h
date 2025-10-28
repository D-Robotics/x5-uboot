/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2023
 * ming.yu@horizon.cc
 */
#ifndef __HORIZON_EFUSE_H__
#define __HORIZON_EFUSE_H__

enum {
    MBEDTLS_OTP_MODEL_ID_OFFSET            = 0x00U,
    MBEDTLS_OTP_DEVICE_ID_OFFSET           = 0x14U,
    MBEDTLS_OTP_LCS_OFFSET                 = 0x48U,
    MBEDTLS_OTP_USER_NON_SEC_REGION_OFFSET = 0x50U,
    MBEDTLS_OTP_USER_SOCID_HIGH_OFFSET     = 0x6CU,
    MBEDTLS_OTP_USER_SECUR_FLAG_OFFSET     = 0x78U,
    MBEDTLS_OTP_USER_SEC_REGION_OFFSET     = 0x88U,
    MBEDTLS_OTP_USER_SEC_BL2PUB_OFFSET     = 0xBCU,
};

enum {
    MBEDTLS_OTP_MODEL_ID_SIZE            = 0x04U,
    MBEDTLS_OTP_MODEL_KEY_SIZE           = 0x10U,
    MBEDTLS_OTP_DEVICE_ID_SIZE           = 0x04U,
    MBEDTLS_OTP_DEVICE_RK_SIZE           = 0x10U,
    MBEDTLS_OTP_SEC_BOOT_HASH_SIZE       = 0x20U,
    MBEDTLS_OTP_LCS_SIZE                 = 0x04U,
    MBEDTLS_OTP_LOCK_CTRL_SIZE           = 0x04U,
    MBEDTLS_OTP_USER_NON_SEC_REGION_SIZE = 0x38U,
    MBEDTLS_OTP_USER_SEC_REGION_SIZE     = 0x78U,
    MBEDTLS_OTP_USER_SOCID_HIGH_SIZE     = 0x08U,
    MBEDTLS_OTP_USER_SEC_BL2PUB_SIZE     = 0x20U,
};

enum {
    NOSEC_CHIP = 0,
    SEC_CHIP1_SEC_BOOT,
    SEC_CHIP1_NOSEC_BOOT,
    SEC_CHIP2,
};

enum {
    CHIP_X5_U = 0,
    CHIP_X5_H,
    CHIP_X5_M,
    CHIP_X5_B,
};

#define MAX_CPU	8
#define EFUSE_CPU_OPPTABLE_OFFSET	0x74
#define EFUSE_CPU_OPPTABLE_BIT		(7)
#define EFUSE_CPU_OPPTABLE_MASK		(0xF << EFUSE_CPU_OPPTABLE_BIT)

int hb_read_efuse(uint32_t offset, uint32_t size, char *output_buffer);
int hb_get_socuid(uint32_t *socuid);
int is_secure_boot(void);
int get_sec_info(uint32_t *sec_info);
int get_chip_type(uint32_t *chip_type);
int get_anti_ver_from_efuse(uint32_t *nosec_anti_ver);

enum efuse_type {
	EFUSE_SECURE = 0,
	EFUSE_NONSECURE
};

enum efuse_lock {
	EFUSE_UNLOCK = 0,
	EFUSE_LOCK
};

struct efuse_info {
    enum efuse_type type;
    uint32_t bank;
    uint32_t value;
    bool lock;
};

#define HB_EFUSE_PINMUX   0x31040008
#define HB_EFUSE_PINMUX_OFFSET 12
#define HB_EFUSE_PINMUX_MSAK    (0x3 << HB_EFUSE_PINMUX_OFFSET)
#define HB_EFUSE_FUNC_GPIO      (1 << HB_EFUSE_PINMUX_OFFSET)
#define HB_EFUSE_GPIO_DIR_REG  0x31000004
#define HB_EFUSE_GPIO_DIR_OFFSET    7
#define HB_EFUSE_GPIO_DIR_OUTPUT    (1 << HB_EFUSE_GPIO_DIR_OFFSET)
#define HB_EFUSE_GPIO_VALUE_REG  0x31000000
#define HB_EFUSE_GPIO_VALUE_OFFSET    7
#define HB_EFUSE_GPIO_HIGH    (1 << HB_EFUSE_GPIO_VALUE_OFFSET)

#endif /* __HOBOT_EFUSE_H__ */