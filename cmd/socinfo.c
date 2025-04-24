/*
 *    COPYRIGHT NOTICE
 *   Copyright 2023 Horizon Robotics, Inc.
 *    All rights reserved.
*/
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/sizes.h>
#include <common.h>
#include <mapmem.h>
#include <asm/io.h>
#include <asm/global_data.h>
#include <fdt_support.h>
#include <env.h>
#include <dm/ofnode.h>
#include <regmap.h>
#include <asm/arch/hb_efuse.h>
#include <hb_info.h>

DECLARE_GLOBAL_DATA_PTR;

#define SCRATCHPAD	1024
static struct fdt_header *hb_dtb = NULL;


static int hb_dtb_property_config(int offset, char *prop, int value)
{
	int  len;		/* new length of the property */
	int  ret;		/* return value */
	static char node_data[SCRATCHPAD] __aligned(4);/* property storage */

	fdt_getprop(hb_dtb, offset, prop, &len);
	if (len > SCRATCHPAD) {
		printf("prop (%d) doesn't fit in scratchpad!\n",
				len);
		return 1;
	}

	snprintf(node_data, sizeof(node_data), "0x%04x", value);
	len = strlen(node_data) + 1;

	ret = fdt_setprop(hb_dtb, offset, prop, node_data, len);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	return 0;
}

static int hb_dtb_property_string_config(int offset, char *prop, const char *string)
{
        int  len;               /* new length of the property */
        int  ret;               /* return value */
        static char node_data[SCRATCHPAD] __aligned(4);/* property storage */

        fdt_getprop(hb_dtb, offset, prop, &len);
        if (len > SCRATCHPAD) {
                printf("prop (%d) doesn't fit in scratchpad!\n",
                                len);
                return 1;
        }

        snprintf(node_data, sizeof(node_data), "%s", string);
        len = strlen(node_data) + 1;

        ret = fdt_setprop(hb_dtb, offset, prop, node_data, len);
        if (ret < 0) {
                printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
                return 1;
        }

        return 0;
}

static int hb_set_board_id(int offset)
{
	int  ret;		/* return value */
	char *prop   = "board_id";
	uint32_t id = 0;

	/* set board id */
	ret = hb_board_id_get(&id);
	if (ret) {
		printf("get boardid failed\n");
		return ret;
	}

	ret = hb_dtb_property_config(offset, prop, id);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return ret;
	}

	return ret;
}

static int hb_set_bootdevice_name(int offset)
{
	int  ret;		/* return value */
	char *prop   = "bootdevice_name";
	const char *boot_mode = env_get("boot_device");

	ret = hb_dtb_property_string_config(offset, prop, boot_mode);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	return ret;
}

static int hb_set_socuid(int offset)
{
	int  len;		/* new length of the property */
	int  ret;		/* return value */
	const void *ptmp;
	char *prop = "soc_uid";
	static char node_data[SCRATCHPAD] __aligned(4);/* property storage */
	uint32_t hb_uid[4] = {0};

	ptmp = fdt_getprop(hb_dtb, offset, prop, &len);
	if (len > SCRATCHPAD) {
		printf("prop (%d) doesn't fit in scratchpad!\n",
				len);
		return 1;
	}

	if (ptmp != NULL)
		memcpy(node_data, ptmp, len);

	memset(node_data, 0, sizeof(node_data));
	ret = hb_get_socuid(hb_uid);
	if(ret < 0) {
		printf("get_socuid error\n");
		return 1;
	}
	snprintf(node_data, sizeof(node_data), "0x%.8x%.8x%.8x%.8x",
			hb_uid[3], hb_uid[2], hb_uid[1], hb_uid[0]);
	len = strlen(node_data) + 1;
	ret = fdt_setprop(hb_dtb, offset, prop, node_data, len);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	return 0;
}

static int hb_set_hw_name(int offset)
{
	int  ret;
	int  len = 0;
	char *prop = "hw_name";
	static char node_data[SCRATCHPAD] __aligned(4);
	const void *ptmp;
	char *data = NULL;

	memset(node_data, 0, sizeof(node_data));
	ptmp = fdt_getprop(hb_dtb, offset, prop, &len);
	if ((len > SCRATCHPAD) || ptmp == NULL) {
		printf("prop (%d) doesn't fit in scratchpad!\n", len);
		return 1;
	}

	memcpy(node_data, ptmp, len);

	/* set hw_name */
	len = strlen(node_data) + 1;
	data = hb_hardware_name_get();
	if (data == NULL) {
		strncpy(node_data, "unkown", strlen("unkown") + 1);
	} else {
		strncpy(node_data, data, strlen(data) + 1);
	}
	ret = fdt_setprop(hb_dtb, offset, prop, node_data, len);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}
	return ret;
}

static int hb_set_board_version(int offset)
{
	int  ret;
	int  len = 0;
	char *prop = "board_version";
	static char node_data[SCRATCHPAD] __aligned(4);
	const void *ptmp;
	char *data = NULL;

	memset(node_data, 0, sizeof(node_data));
	ptmp = fdt_getprop(hb_dtb, offset, prop, &len);
	if ((len > SCRATCHPAD) || ptmp == NULL) {
		printf("prop (%d) doesn't fit in scratchpad!\n", len);
		return 1;
	}

	memcpy(node_data, ptmp, len);

	/* set board_version */
	len = strlen(node_data) + 1;
	data = hb_board_version_get();
	if (data == NULL) {
		strncpy(node_data, "unkown", strlen("unkown") + 1);
	} else {
		strncpy(node_data, data, strlen(data) + 1);
	}
	ret = fdt_setprop(hb_dtb, offset, prop, node_data, len);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}
	return ret;
}

static int hb_set_hw_info(int offset)
{
	int  ret;
	int  len = 0;
	char *prop = "hw_info";
	static char node_data[SCRATCHPAD] __aligned(4);
	const void *ptmp;
	char *data = NULL;

	memset(node_data, 0, sizeof(node_data));
	ptmp = fdt_getprop(hb_dtb, offset, prop, &len);
	if ((len > SCRATCHPAD) || ptmp == NULL) {
		printf("prop (%d) doesn't fit in scratchpad!\n", len);
		return 1;
	}

	memcpy(node_data, ptmp, len);

	/* set hw_info */
	len = strlen(node_data) + 1;
	data = hb_hw_info_get();
	if (data == NULL) {
		strncpy(node_data, "unkown", strlen("unkown") + 1);
	} else {
		strncpy(node_data, data, strlen(data) + 1);
	}
	ret = fdt_setprop(hb_dtb, offset, prop, node_data, len);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}
	return ret;
}

static int hb_set_sec_chip(int offset, char *data)
{
	int  ret;
	int  len = 0;
	char *prop = "sec_chip";
	static char node_data[SCRATCHPAD] __aligned(4);
	const void *ptmp;

	memset(node_data, 0, sizeof(node_data));
	ptmp = fdt_getprop(hb_dtb, offset, prop, &len);
	if ((len > SCRATCHPAD) || ptmp == NULL) {
		printf("prop (%d) doesn't fit in scratchpad!\n", len);
		return 1;
	}

	memcpy(node_data, ptmp, len);

	/* set sec_chip */
	len = strlen(node_data) + 1;
	if (data == NULL) {
		strncpy(node_data, "unkown", strlen("unkown") + 1);
	} else {
		strncpy(node_data, data, strlen(data) + 1);
	}
	ret = fdt_setprop(hb_dtb, offset, prop, node_data, len);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}
	return ret;
}

static int hb_set_sec_boot(int offset, char *data)
{
	int  ret;
	int  len = 0;
	char *prop = "sec_boot";
	static char node_data[SCRATCHPAD] __aligned(4);
	const void *ptmp;

	memset(node_data, 0, sizeof(node_data));
	ptmp = fdt_getprop(hb_dtb, offset, prop, &len);
	if ((len > SCRATCHPAD) || ptmp == NULL) {
		printf("prop (%d) doesn't fit in scratchpad!\n", len);
		return 1;
	}

	memcpy(node_data, ptmp, len);

	/* set sec_boot */
	len = strlen(node_data) + 1;
	if (data == NULL) {
		strncpy(node_data, "unkown", strlen("unkown") + 1);
	} else {
		strncpy(node_data, data, strlen(data) + 1);
	}
	ret = fdt_setprop(hb_dtb, offset, prop, node_data, len);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}
	return ret;
}

static int hb_set_sec_info(int offset)
{
	int  ret;
	uint32_t sec_info = 0;

	ret = get_sec_info(&sec_info);
	if (ret) {
		goto exit;
	}

	if (sec_info == NOSEC_CHIP) {
		ret = hb_set_sec_chip(offset, "nosec_chip") || hb_set_sec_boot(offset, "disable");
	} else if (sec_info == SEC_CHIP1_SEC_BOOT) {
		ret = hb_set_sec_chip(offset, "sec_chip1") || hb_set_sec_boot(offset, "enable");
	} else if (sec_info == SEC_CHIP1_NOSEC_BOOT) {
		ret = hb_set_sec_chip(offset, "sec_chip1") || hb_set_sec_boot(offset, "disable");
	} else if (sec_info == SEC_CHIP2) {
		ret = hb_set_sec_chip(offset, "sec_chip2") || hb_set_sec_boot(offset, "enable");
	} else {
		ret = hb_set_sec_chip(offset, "no_support") || hb_set_sec_boot(offset, "no_support");
	}

exit:
	return ret;
}

static int hb_set_soc_name(int offset)
{
	int  ret;
	int  len = 0;
	char *prop = "soc_name";
	static char node_data[SCRATCHPAD] __aligned(4);
	const void *ptmp;
	uint32_t chip_type = 0;

	memset(node_data, 0, sizeof(node_data));
	ptmp = fdt_getprop(hb_dtb, offset, prop, &len);
	if ((len > SCRATCHPAD) || ptmp == NULL) {
		printf("prop (%d) doesn't fit in scratchpad!\n", len);
		return 1;
	}

	memcpy(node_data, ptmp, len);

	/* set hw_name */
	len = strlen(node_data) + 1;

	ret = get_chip_type(&chip_type);
	if (ret) {
		printf("read efuse chip type failed\n");
		return 1;
	}

	if (chip_type == CHIP_X5_H) {
		strncpy(node_data, "X5H", strlen("X5H") + 1);
	} else if (chip_type == CHIP_X5_M) {
		strncpy(node_data, "X5M", strlen("X5M") + 1);
	} else if (chip_type == CHIP_X5_B) {
		strncpy(node_data, "X5B", strlen("X5B") + 1);
	} else {
		strncpy(node_data, "UKNOWN", strlen("UKNOWN") + 1);
	}

	ret = fdt_setprop(hb_dtb, offset, prop, node_data, len);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}
	return ret;
}

int hb_fdt_set_board_info(void *fdt_blob)
{
	char *pathp  = "/soc/socinfo";
	int  nodeoffset;	/* node offset from libfdt */
	int  ret;		/* return value */

	hb_dtb = fdt_blob;
	nodeoffset = fdt_path_offset(fdt_blob, pathp);
	if (nodeoffset < 0) {
		/*
		 * Not found or something else bad happened.
		 */
		printf("%s: libfdt fdt_path_offset() returned %s\n",
			__func__, fdt_strerror(nodeoffset));
		return 1;
	}

	/* set bootmode */
	ret = hb_set_bootdevice_name(nodeoffset);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	/* set socuid */
	ret = hb_set_socuid(nodeoffset);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	/* set hw_name */
	ret = hb_set_hw_name(nodeoffset);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	/* set board_version */
	ret = hb_set_board_version(nodeoffset);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	/* set hw_info */
	ret = hb_set_hw_info(nodeoffset);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	/* set board id */
	ret = hb_set_board_id(nodeoffset);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	/* set sec_chip and sec_boot */
	ret = hb_set_sec_info(nodeoffset);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	/* set soc_name */
	ret = hb_set_soc_name(nodeoffset);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	return 0;
}
