// SPDX-License-Identifier: GPL-2.0+
/*
 * Read MAC list from SPI NAND user OTP and set ethaddr / eth1addr / ...
 *
 * OTP blob layout (same as tools/nand_otp_mac):
 *   [0-1] version 01 01
 *   [2-3] payload length (LE): 1 + n*6
 *   [4]   MAC count n
 *   [5..] n MAC addresses, 6 bytes each
 *
 * Applied at board_late_init for the current boot only (no saveenv).
 */

#include <command.h>
#include <common.h>
#include <env.h>
#include <linux/err.h>
#include <linux/mtd/spinand.h>
#include <mtd.h>
#include <net.h>
#include <otp_mac.h>

#define OTP_MAC_HDR_LEN		5
#define OTP_MAC_LEN		6
#define OTP_MAC_VER0		0x01
#define OTP_MAC_VER1		0x01
#define OTP_MAC_MAX		32
#define OTP_MAC_READ_MAX	256

static struct spinand_device *mtd_to_spinand_dev(struct mtd_info *mtd)
{
	while (mtd_is_partition(mtd))
		mtd = mtd->parent;

	if (!mtd->dev || !mtd->dev->driver ||
	    strcmp(mtd->dev->driver->name, "spi_nand"))
		return NULL;

	return mtd_to_spinand(mtd);
}

static struct mtd_info *otp_mac_get_mtd(const char *name)
{
	struct mtd_info *mtd;

	mtd_probe_devices();
	mtd = get_mtd_device_nm(name);
	if (IS_ERR_OR_NULL(mtd))
		return NULL;

	return mtd;
}

static bool otp_mac_hdr_blank(const u8 *buf)
{
	int i;

	for (i = 0; i < OTP_MAC_HDR_LEN; i++) {
		if (buf[i] != 0xff)
			return false;
	}
	return true;
}

static int otp_mac_parse_blob(const u8 *buf, size_t len, unsigned int *nmac)
{
	unsigned int n, expect;
	u16 dlen;

	if (len < OTP_MAC_HDR_LEN)
		return -EINVAL;

	if (otp_mac_hdr_blank(buf))
		return -ENOENT;

	if (buf[0] != OTP_MAC_VER0 || buf[1] != OTP_MAC_VER1)
		return -EINVAL;

	n = buf[4];
	if (n < 1 || n > OTP_MAC_MAX)
		return -EINVAL;

	expect = 1 + n * OTP_MAC_LEN;
	dlen = buf[2] | (buf[3] << 8);
	if (dlen != expect)
		return -EINVAL;

	if (len < OTP_MAC_HDR_LEN + n * OTP_MAC_LEN)
		return -EINVAL;

	*nmac = n;
	return 0;
}

static int otp_mac_set_env(unsigned int index, const u8 *mac, bool verbose)
{
	char enetvar[16];
	char val[ARP_HLEN_ASCII + 1];

	if (index)
		sprintf(enetvar, "eth%daddr", index);
	else
		strcpy(enetvar, "ethaddr");

	if (!is_valid_ethaddr(mac)) {
		printf("OTP MAC%d invalid: %pM\n", index, mac);
		return -EINVAL;
	}

	/* Runtime env only; never saveenv — OTP is the source of truth. */
	snprintf(val, sizeof(val), "%pM", mac);
	env_set(enetvar, val);
	if (verbose)
		printf("  %s = %s\n", enetvar, val);
	return 0;
}

/**
 * otp_mac_apply() - Read OTP MAC blob and set eth*addr for this boot
 * @mtd_name: MTD device name, or NULL for "spi-nand0"
 * @verbose: print each address when true
 *
 * Return: 0 on success / blank OTP, negative errno on failure
 */
int otp_mac_apply(const char *mtd_name, bool verbose)
{
	const char *name = mtd_name ? mtd_name : "spi-nand0";
	struct mtd_info *mtd;
	struct spinand_device *spinand;
	u8 buf[OTP_MAC_READ_MAX];
	size_t retlen = 0;
	unsigned int i, nmac;
	int ret;

	mtd = otp_mac_get_mtd(name);
	if (!mtd) {
		if (verbose)
			printf("MTD device %s not found\n", name);
		return -ENODEV;
	}

	spinand = mtd_to_spinand_dev(mtd);
	if (!spinand) {
		if (verbose)
			printf("%s is not an SPI NAND device\n", mtd->name);
		put_mtd_device(mtd);
		return -ENODEV;
	}

	if (!spinand->otp.npages) {
		if (verbose)
			printf("SPI NAND %s has no user OTP\n", mtd->name);
		put_mtd_device(mtd);
		return -EOPNOTSUPP;
	}

	ret = spinand_otp_read(spinand, 0, OTP_MAC_READ_MAX, &retlen, buf);
	if (ret) {
		printf("OTP MAC read failed: %d\n", ret);
		put_mtd_device(mtd);
		return ret;
	}

	ret = otp_mac_parse_blob(buf, retlen, &nmac);
	if (ret == -ENOENT) {
		if (verbose)
			printf("OTP has no MAC data (blank)\n");
		put_mtd_device(mtd);
		return 0;
	}
	if (ret) {
		printf("OTP MAC blob invalid (ret %d)\n", ret);
		put_mtd_device(mtd);
		return ret;
	}

	printf("OTP MAC: %u address(es) from %s\n", nmac, mtd->name);
	for (i = 0; i < nmac; i++) {
		const u8 *mac = buf + OTP_MAC_HDR_LEN + i * OTP_MAC_LEN;

		/* Always show applied addresses during boot / command. */
		ret = otp_mac_set_env(i, mac, true);
		if (ret) {
			put_mtd_device(mtd);
			return ret;
		}
	}

	put_mtd_device(mtd);
	return 0;
}

static int do_otp_mac(struct cmd_tbl *cmdtp, int flag, int argc,
		      char *const argv[])
{
	const char *name = NULL;

	if (argc > 1)
		name = argv[1];

	if (otp_mac_apply(name, true))
		return CMD_RET_FAILURE;

	return CMD_RET_SUCCESS;
}

#ifdef CONFIG_SYS_LONGHELP
static char otp_mac_help_text[] =
	"[<mtd_name>]\n"
	"  - read MAC list from SPI NAND user OTP\n"
	"  - set ethaddr, eth1addr, ... for this boot (not saved)\n"
	"  - default mtd: spi-nand0";
#endif

U_BOOT_CMD(otp_mac, 2, 0, do_otp_mac,
	   "Read MAC from SPI NAND OTP and set ethaddr env vars",
	   otp_mac_help_text);
