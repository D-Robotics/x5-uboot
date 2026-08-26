/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef _OTP_MAC_H_
#define _OTP_MAC_H_

#include <linux/types.h>

/**
 * otp_mac_apply() - Apply SPI NAND OTP MAC list to eth*addr env
 * @mtd_name: MTD name or NULL for "spi-nand0"
 * @verbose: print each MAC when true
 *
 * Sets runtime env only; does not saveenv.
 *
 * Return: 0 on success or blank OTP, negative errno otherwise
 */
int otp_mac_apply(const char *mtd_name, bool verbose);

#endif /* _OTP_MAC_H_ */
