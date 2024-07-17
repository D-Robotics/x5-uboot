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

#if 0
static uint32_t ddr_vender;
static uint32_t ddr_size;
static uint32_t ddr_type;
static uint32_t ddr_freq;
#endif

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

	snprintf(node_data, sizeof(node_data), "%x", value);
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
	char *prop_origin = "origin_board_id";
	uint32_t id = 0;

	/* set origin board id */
	ret = hb_board_id_get(&id);
	if (ret) {
		printf("get boardid failed\n");
		return ret;
	}
	ret = hb_dtb_property_config(offset, prop_origin, id);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return ret;
	}

	ret = hb_dtb_property_config(offset, prop, id);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return ret;
	}

	return ret;
}

static int hb_set_boot_mode(int offset)
{
	int  ret;		/* return value */
	char *prop   = "boot_mode";
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
	char *prop = "socuid";
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
    snprintf(node_data, sizeof(node_data), "0x%.8x%.8x%.8x%.8x\n",
            hb_uid[3], hb_uid[2], hb_uid[1], hb_uid[0]);
	len = strlen(node_data) + 1;
	ret = fdt_setprop(hb_dtb, offset, prop, node_data, len);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	return 0;
}

#if 0
static void hb_get_ddr_info(void)
{
    /* TODO */
    return;
}

static int hb_set_ddr_property(int offset)
{
	int  ret;
	char *prop_type = "ddr_type";
	char *prop_size = "ddr_size";
	char *prop_vender = "ddr_vender";
	char *prop_freq = "ddr_freq";

	hb_get_ddr_info();
	/* set ddr_vender */
	ret = hb_dtb_property_config(offset, prop_vender, ddr_vender);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	/* set ddr_size */
	ret = hb_dtb_property_config(offset, prop_size, ddr_size);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	/* set ddr_type */
	ret = hb_dtb_property_config(offset, prop_type, ddr_type);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	/* set ddr_freq */
	ret = hb_dtb_property_config(offset, prop_freq, ddr_freq);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	return ret;
}
#endif

static int hb_set_board_type(int offset)
{
	int  ret;
	int  len = 0;
	char *prop = "base_board_name";
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

	/* set base boardname */
	len = strlen(node_data) + 1;
	data = hb_board_name_get();
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
	ret = hb_set_boot_mode(nodeoffset);
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
#if 0
	/* set ddr size, ddr freq, ddr vender and ddr type */
	ret = hb_set_ddr_property(nodeoffset);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}
#endif

	/* set som type and base board type */
	ret = hb_set_board_type(nodeoffset);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}
	/* set board id and origin board id */
	ret = hb_set_board_id(nodeoffset);
	if (ret < 0) {
		printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
		return 1;
	}

	return 0;
}
