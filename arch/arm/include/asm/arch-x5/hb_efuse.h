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

#define MAX_CPU	8
#define EFUSE_CPU_OPPTABLE_OFFSET	0x54
#define EFUSE_CPU_OPPTABLE_BIT		(28)
#define EFUSE_CPU_OPPTABLE_MASK		(0x3 << EFUSE_CPU_OPPTABLE_BIT)

int hb_read_efuse(uint32_t offset, uint32_t size, char *output_buffer);
int hb_get_socuid(uint32_t *socuid);
bool is_secure_boot(void);

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