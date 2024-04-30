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
#endif /* __HOBOT_EFUSE_H__ */