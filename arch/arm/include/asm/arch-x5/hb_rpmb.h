/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2023
 * ming.yu@horizon.cc
 */
#ifndef __HORIZON_RPMB_H__
#define __HORIZON_RPMB_H__

/**
 * @enum DataFlag
 * Define data use flag
 */
#define DATA_FLAG_ACCESS_READ         0x00000001u  /**< data read flag */
#define DATA_FLAG_ACCESS_WRITE        0x00000002u  /**< data write flag */
#define DATA_FLAG_ACCESS_WRITE_META   0x00000004u  /**< data write exclusive flag */
#define DATA_FLAG_SHARE_READ          0x00000010u  /**< data read share flag */
#define DATA_FLAG_SHARE_WRITE         0x00000020u  /**< data write share flag */
#define DATA_FLAG_OVERWRITE           0x00000400u  /**< data over write flag */
#define DATA_FLAG_CREATE              0x80000000u  /**< data create flag */


#define HBST_OK						0	/**< success return */

#define HBST_ERR_PARAM					-1	/**< error input parameter */
#define HBST_ERR_KEYNAME				-2	/**< illegal file path */
#define HBST_ERR_MALLOC					-3	/**< malloc fail */

#define HBST_ERR_OPEN					-4	/**< open file or object fail */
#define HBST_ERR_FILEOPEN				HBST_ERR_OPEN
#define HBST_ERR_DELETE					-5	/**< delete file or object fail */
#define HBST_ERR_WR					-6	/**< write file or object fail */
#define HBST_ERR_RD					-7	/**< read file or object fail */
#define HBST_ERR_FILERD					HBST_ERR_RD
#define HBST_ERR_NULLOBJ				-8	/**< null object */

#define TA_SECSTOR_UUID \
	{ 0x7e53c7a0, 0xd449, 0x423e, \
		{ 0x8d, 0xce, 0x7c, 0xb7, 0x94, 0x23, 0x6d, 0xdd } }

/**
 * @def TA_SECSTOR_CMD_CREATE
 * in	params[0].value a:storage_type, b:flag
 * in	params[1].memref filename
 * in	params[2].memref initial data
 * out	params[3] (value) a: object, b: unused
 */
#define TA_SECSTOR_CMD_CREATE		0x0

/**
 * @def TA_SECSTOR_CMD_OPEN
 * in	param[0].value a: storage_type, b: flag
 * in	param[1].memref filename
 * out	param[2].value a: object, b: unused
 */
#define TA_SECSTOR_CMD_OPEN		0x1

/**
 * @def TA_SECSTOR_CMD_CLOSE
 * in	param[0].value a: object, b: unused
 */
#define TA_SECSTOR_CMD_CLOSE		0x2

/**
 * @def TA_SECSTOR_CMD_READ
 * in	param[0].value a: object, b: read count
 * out	param[1].memref buffer
 */
#define TA_SECSTOR_CMD_READ		0x3

/**
 * @def TA_SECSTOR_CMD_WRITE
 * in	param[0].value a: object, b: unused
 * in	param[1].memref buffer
 */
#define TA_SECSTOR_CMD_WRITE		0x4

/**
 * @def TA_SECSTOR_CMD_SEEK
 * in	param[0].value a: object, b: offset
 * in	param[1].value a: whence, b: unused
 */
#define TA_SECSTOR_CMD_SEEK		0x5

/**
 * @def TA_SECSTOR_CMD_DELETE
 * param[0].value a: object, b: unused
 */
#define TA_SECSTOR_CMD_DELETE		0x6

/**
 * @def TA_SECSTOR_CMD_RENAME
 * param[0].value a: object, b: unused
 * param[1].memref new filename
 */
#define TA_SECSTOR_CMD_RENAME		0x7

/**
 * @def TA_SECSTOR_CMD_TRUNCATE
 * param[0].value a: object, b: len
 */
#define TA_SECSTOR_CMD_TRUNCATE		0x8

/* Storage is provided by the Rich Execution Environment (REE) */
#define SECURE_STORAGE_PRIVATE_REE		1u
/* Storage is the Replay Protected Memory Block partition of an eMMC device */
#define SECURE_STORAGE_PRIVATE_RPMB		0u

int64_t drobot_st_file_load(const char *file_name, char **output, uint32_t *len, uint32_t store_mode);
int64_t drobot_st_file_store(const char *file_name, char *input, uint32_t len, uint32_t store_mode);
#endif /* __HOBOT_RPMB_H__ */
