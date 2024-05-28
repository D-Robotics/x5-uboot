// SPDX-License-Identifier: GPL-2.0
/*
 * Author:
 *	Chuanhong Guo <gch981213 at gmail.com> - the main driver logic
 *	Martin Kurbanov <mmkurbanov at sberdevices.ru> - OOB layout
 */

#ifndef __UBOOT__
#include <linux/device.h>
#include <linux/kernel.h>
#endif
#include <linux/mtd/spinand.h>

/* ESMT uses GigaDevice 0xc8 JECDEC ID on some SPI NANDs */
#define SPINAND_MFR_ESMT_C8			0xc8
#define SPINAND_MFR_ESMT_2C			0x2c

static SPINAND_OP_VARIANTS(read_cache_variants,
			   SPINAND_PAGE_READ_FROM_CACHE_X4_OP(0, 1, NULL, 0),
			   SPINAND_PAGE_READ_FROM_CACHE_X2_OP(0, 1, NULL, 0),
			   SPINAND_PAGE_READ_FROM_CACHE_OP(true, 0, 1, NULL, 0),
			   SPINAND_PAGE_READ_FROM_CACHE_OP(false, 0, 1, NULL, 0));

static SPINAND_OP_VARIANTS(write_cache_variants,
			   SPINAND_PROG_LOAD_X4(true, 0, NULL, 0),
			   SPINAND_PROG_LOAD(true, 0, NULL, 0));

static SPINAND_OP_VARIANTS(update_cache_variants,
			   SPINAND_PROG_LOAD_X4(false, 0, NULL, 0),
			   SPINAND_PROG_LOAD(false, 0, NULL, 0));

/*
 * OOB spare area map (64 bytes)
 *
 * Bad Block Markers
 * filled by HW and kernel                 Reserved
 *   |                 +-----------------------+-----------------------+
 *   |                 |                       |                       |
 *   |                 |    OOB free data Area |non ECC protected      |
 *   |   +-------------|-----+-----------------|-----+-----------------|-----+
 *   |   |             |     |                 |     |                 |     |
 * +-|---|----------+--|-----|--------------+--|-----|--------------+--|-----|--------------+
 * | |   | section0 |  |     |    section1  |  |     |    section2  |  |     |    section3  |
 * +-v-+-v-+---+----+--v--+--v--+-----+-----+--v--+--v--+-----+-----+--v--+--v--+-----+-----+
 * |   |   |   |    |     |     |     |     |     |     |     |     |     |     |     |     |
 * |0:1|2:3|4:7|8:15|16:17|18:19|20:23|24:31|32:33|34:35|36:39|40:47|48:49|50:51|52:55|56:63|
 * |   |   |   |    |     |     |     |     |     |     |     |     |     |     |     |     |
 * +---+---+-^-+--^-+-----+-----+--^--+--^--+-----+-----+--^--+--^--+-----+-----+--^--+--^--+
 *           |    |                |     |                 |     |                 |     |
 *           |    +----------------|-----+-----------------|-----+-----------------|-----+
 *           |             ECC Area|(Main + Spare) - filled|by ESMT NAND HW        |
 *           |                     |                       |                       |
 *           +---------------------+-----------------------+-----------------------+
 *                         OOB ECC protected Area - not used due to
 *                         partial programming from some filesystems
 *                             (like JFFS2 with cleanmarkers)
 */

#define ESMT_OOB_SECTION_COUNT			4
#define ESMT_OOB_SECTION_SIZE(nand) \
	(nanddev_per_page_oobsize(nand) / ESMT_OOB_SECTION_COUNT)
#define ESMT_OOB_FREE_SIZE(nand) \
	(ESMT_OOB_SECTION_SIZE(nand) / 2)
#define ESMT_OOB_ECC_SIZE(nand) \
	(ESMT_OOB_SECTION_SIZE(nand) - ESMT_OOB_FREE_SIZE(nand))
#define ESMT_OOB_BBM_SIZE			2

static int f50l1g41lb_ooblayout_ecc(struct mtd_info *mtd, int section,
				    struct mtd_oob_region *region)
{
	struct nand_device *nand = mtd_to_nanddev(mtd);

	if (section >= ESMT_OOB_SECTION_COUNT)
		return -ERANGE;

	region->offset = section * ESMT_OOB_SECTION_SIZE(nand) +
			 ESMT_OOB_FREE_SIZE(nand);
	region->length = ESMT_OOB_ECC_SIZE(nand);

	return 0;
}

static int f50l1g41lb_ooblayout_free(struct mtd_info *mtd, int section,
				     struct mtd_oob_region *region)
{
	struct nand_device *nand = mtd_to_nanddev(mtd);

	if (section >= ESMT_OOB_SECTION_COUNT)
		return -ERANGE;

	/*
	 * Reserve space for bad blocks markers (section0) and
	 * reserved bytes (sections 1-3)
	 */
	region->offset = section * ESMT_OOB_SECTION_SIZE(nand) + 2;

	/* Use only 2 non-protected ECC bytes per each OOB section */
	region->length = 2;

	return 0;
}

static const struct mtd_ooblayout_ops f50l1g41lb_ooblayout = {
	.ecc = f50l1g41lb_ooblayout_ecc,
	.rfree = f50l1g41lb_ooblayout_free,
};

static int f50d4g41xb_ooblayout_ecc(struct mtd_info *mtd, int section,
				    struct mtd_oob_region *region)
{
	if (section >= 8)
		return -ERANGE;

	region->offset = 0x80 + section * 16;
	region->length = 16;

	return 0;
}

static int f50d4g41xb_ooblayout_free(struct mtd_info *mtd, int section,
				     struct mtd_oob_region *region)
{
	if (section >= 8)
		return -ERANGE;

	region->offset = section * 4;

	region->length = 4;

	return 0;
}

static const struct mtd_ooblayout_ops f50d4g41xb_ooblayout = {
	.ecc = f50d4g41xb_ooblayout_ecc,
	.rfree = f50d4g41xb_ooblayout_free,
};


static const struct spinand_info esmt_c8_spinand_table[] = {
	SPINAND_INFO("F50L1G41LB", 0x01,
		     NAND_MEMORG(1, 2048, 64, 64, 1024, 1, 1, 1),
		     NAND_ECCREQ(1, 512),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
					      &write_cache_variants,
					      &update_cache_variants),
		     0,
		     SPINAND_ECCINFO(&f50l1g41lb_ooblayout, NULL)),
	SPINAND_INFO("F50D1G41LB", 0x11,
		     NAND_MEMORG(1, 2048, 64, 64, 1024, 1, 1, 1),
		     NAND_ECCREQ(1, 512),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
					      &write_cache_variants,
					      &update_cache_variants),
		     0,
		     SPINAND_ECCINFO(&f50l1g41lb_ooblayout, NULL)),
	SPINAND_INFO("F50D4G41xB", 0x35,
		     NAND_MEMORG(1, 4096, 256, 64, 2048, 1, 1, 1),
		     NAND_ECCREQ(8, 512),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
					      &write_cache_variants,
					      &update_cache_variants),
		     0,
		     SPINAND_ECCINFO(&f50d4g41xb_ooblayout, NULL)),
};


static const struct spinand_info esmt_2c_spinand_table[] = {
	SPINAND_INFO("F50D4G41xB", 0x35,
		     NAND_MEMORG(1, 4096, 256, 64, 2048, 1, 1, 1),
		     NAND_ECCREQ(8, 512),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
					      &write_cache_variants,
					      &update_cache_variants),
		     0,
		     SPINAND_ECCINFO(&f50d4g41xb_ooblayout, NULL)),
};

static int esmt_spinand_detect(struct spinand_device *spinand)
{
	u8 *id = spinand->id.data;
	int ret;

	/*
	 * For GD NANDs, There is an address byte needed to shift in before IDs
	 * are read out, so the first byte in raw_id is dummy.
	 */
	if (id[1] == SPINAND_MFR_ESMT_C8) {
		ret = spinand_match_and_init(spinand, esmt_c8_spinand_table,
						ARRAY_SIZE(esmt_c8_spinand_table),
						id[2]);
	} else if (id[1] == SPINAND_MFR_ESMT_2C) {
		ret = spinand_match_and_init(spinand, esmt_2c_spinand_table,
						ARRAY_SIZE(esmt_2c_spinand_table),
						id[2]);
	} else {
		/* (id[1] != SPINAND_MFR_ESMT_C8 && id[1] != SPINAND_MFR_ESMT_2C) */
		return 0;
	}

	if (ret){
		/* 
		 * ESMT has same manufacturer ID with gigadevice and micron.
		 * So it can not return wrong device ID only by scanning ESMT.
		 */
		return 0;
	}

	return 1;
};

static const struct spinand_manufacturer_ops esmt_spinand_manuf_ops = {
	.detect = &esmt_spinand_detect,
};

const struct spinand_manufacturer esmt_c8_spinand_manufacturer = {
	.id = SPINAND_MFR_ESMT_C8,
	.name = "ESMT",
	.ops = &esmt_spinand_manuf_ops,
};

const struct spinand_manufacturer esmt_2c_spinand_manufacturer = {
	.id = SPINAND_MFR_ESMT_2C,
	.name = "ESMT",
	.ops = &esmt_spinand_manuf_ops,
};
