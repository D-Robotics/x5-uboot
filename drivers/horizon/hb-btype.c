/*
 * Copyright (C) 2024, all rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <common.h>
#include <asm/io.h>
#include <linux/printk.h>
#include <malloc.h>
#include <dm/device.h>
#include <dm/ofnode.h>
#include <asm-generic/gpio.h>
#include <hb_info.h>
#include <dt-bindings/board/hb_board_id.h>
#ifdef CONFIG_HOBOT_ADC_BTYPE
#include <adc.h>
#endif

typedef int board_type_fn(ofnode node, int32_t id);
typedef int (*board_setter)(const char* str);

static int do_hb_board_id(ofnode node, int32_t id)
{
	hb_board_id_set(id);
	return 0;
}

static int do_hb_board_type_common(ofnode node, int32_t id,  char *dts_name, board_setter setter)
{
	int32_t ret = 0;
	const char *str;

	debug("%s:%s:%d,dts name:%s, id:%d\n", __FILE__,__func__, __LINE__, dts_name, id);
	/* select board name from hardware_array via hdname_idx*/
	ret = ofnode_read_string_count(node, dts_name);
	if (ret > id) {
		ret = ofnode_read_string_index(node, dts_name, id, &str);
		if (!ret) {
			debug("parse %s success:%s\n", dts_name, str);
			setter(str);
		} else {
			pr_err("Failed to read hardware_array string at index %d, ret:%d\n",
				id, ret);
				return -1;
		}
	} else {
		pr_err("Failed to read hardware_array or num less than hdname_idx, ret:%d\n", ret);
		return -1;
	}
	return 0;
}
static int do_hb_hardware_array(ofnode node, int32_t id)
{
	return do_hb_board_type_common(node, id, "hardware_array", hb_hardware_name_set);
}

static int do_hb_eth_array(ofnode node, int32_t id)
{
	return do_hb_board_type_common(node, id, "ethact_array", hb_ethact_name_set);
}

static int do_hb_net0_ipaddr(ofnode node, int32_t id)
{
	return do_hb_board_type_common(node, id, "net_eth0_ipaddr", hb_eth0_ip_set);
}

static int do_hb_board_name(ofnode node, int32_t id)
{
	return do_hb_board_type_common(node, id, "board_name_array", hb_board_name_set);
}

static int do_hb_pmic_type(ofnode node, int32_t id)
{
	return do_hb_board_type_common(node, id, "pmic_type", hb_pmic_type_set);
}

enum array_num {
	HB_BOARD_ID = 4,
	HB_HARDWARE_ARRAY,
	HB_ETH_ARRAY,
	HB_NET0_IPADDR,
	HB_BOARD_NAME,
	HB_PMIC_TYPE,
	HB_ARRAY_NUM_MAX,
};

static board_type_fn *bard_type[] = {
	[HB_BOARD_ID] = do_hb_board_id,
	[HB_HARDWARE_ARRAY] = do_hb_hardware_array,
	[HB_ETH_ARRAY] = do_hb_eth_array,
	[HB_NET0_IPADDR] = do_hb_net0_ipaddr,
	[HB_BOARD_NAME] = do_hb_board_name,
	[HB_PMIC_TYPE] = do_hb_pmic_type,
};

__weak void btype_set(ulong board_type)
{
	printf("%s: Please implement this!!!\n", __func__);
}

#ifdef CONFIG_HOBOT_ADC_BTYPE
static int adc_btype_setup(ofnode node, unsigned int *values, unsigned int *channels)
{
	struct udevice *adc_dev;
	ofnode adc_node;
	int adc_dev_num;
	const char *name;
	int ret = 0;
	int i;
	int raw[8] = {0};

	adc_dev_num = ofnode_read_string_count(node, "adc_dev");
	if (adc_dev_num <= 0) {
		pr_err("Failed to get adc devices: %d!\n", adc_dev_num);
		return -EINVAL;
	}

	ret = ofnode_read_u32_array(node, "adc_channel", channels, adc_dev_num);
	if (ret < 0) {
		pr_err("Failed to get adc_channel with status %d\n", ret);
		return ret;
	}

	for (i = 0; i < adc_dev_num; i++) {
		ret = ofnode_read_string_index(node, "adc_dev", i, &name);
		if (ret < 0) {
			return ret;
		}
		debug("name:%s\n", name);
		adc_node = ofnode_path(name);
		debug("node:%p, offset:0x%lx\n", adc_node.np, adc_node.of_offset);

		ret = device_get_global_by_ofnode(adc_node, &adc_dev);
		if (ret) {
			pr_err("Failed to get adc devices!!!\n");
			return ret;
		}

		ret = adc_start_channel(adc_dev, channels[i]);
		if (ret) {
			pr_err("Failed to start adc channel with status %d\n",
				ret);
			return ret;
		}
		ret = adc_channel_data(adc_dev, channels[i], &raw[i]);
		ret |= adc_raw_to_uV(adc_dev, raw[i], &values[i]);
		if (ret) {
			pr_err("Failed to get adc channel data with status %d\n",
				ret);
			return ret;
		}
		values[i] = values[i] / 1000;
		(void) adc_stop(adc_dev);
		debug("adc: %s ch: %d values %d\n", name, channels[i], values[i]);
	}

	return ret;
}
#endif

static int btype_setup(ofnode node)
{
	u32 *btype_array;
	u32 btype_array_len;
	int ret = 0;
	int i;
#ifdef CONFIG_HOBOT_ADC_BTYPE
#define HB_BOARDID_ADC_NUM 2
	uint32_t values[HB_BOARDID_ADC_NUM] = {0};
	uint32_t channel[HB_BOARDID_ADC_NUM] = {0};
	int j;
#endif

	ret = ofnode_read_size(node, "board_type_array");
	if (ret < 0) {
		pr_err("Failed to get board_type_array!!!\n");
		return -EINVAL;
	}
	btype_array_len = ret / sizeof(u32);
	btype_array = malloc(ret);
	if (!btype_array) {
		pr_err("Failed to alloc memory for btype_array\n");
		ret = -ENOMEM;
		return ret;
	}

	ret = ofnode_read_u32_array(node, "board_type_array", btype_array, btype_array_len);
	if (ret < 0) {
		pr_err("Failed to read for btype_array with status %d\n",
			ret);

		goto free_btype_array;
	}

#ifdef CONFIG_HOBOT_ADC_BTYPE
	adc_btype_setup(node, values, channel);
	debug("value:%d, %d\n", values[0], values[1]);
	for (i = 0; i < btype_array_len; i += HB_ARRAY_NUM_MAX) {
		debug("bype: ");
		for (j = 0; j < HB_BOARDID_ADC_NUM; j++) {
			debug("[%u - %u] <%u> ", btype_array[i + (j * 2)],
				btype_array[i + (j * 2) + 1], values[j]);
		}
		debug("--> 0x%08x %d %d\n", btype_array[i + HB_BOARDID_ADC_NUM * 2],
				btype_array[i + HB_BOARDID_ADC_NUM * 2 + 1],
				btype_array[i + HB_BOARDID_ADC_NUM * 2 + 2]);

		for (j = 0; j < HB_BOARDID_ADC_NUM; j++) {
			if ((values[j] < btype_array[i + (j * 2)]) ||
				(values[j] > btype_array[i + (j * 2) + 1])) {
				break;
			}
		}

		if (j == HB_BOARDID_ADC_NUM) {
			board_type_fn *board_fn;
			int32_t m = HB_BOARD_ID;
			for (m = HB_BOARD_ID; m < HB_ARRAY_NUM_MAX; m++) {
				board_fn = bard_type[m];
				board_fn(node, btype_array[i + m]);
			}

			break;
		}
	}
#else
	for (i = 0; i < btype_array_len; i += HB_ARRAY_NUM_MAX) {
		if (HOBOT_X5_BOARD_ID == btype_array[i + HB_BOARD_ID]) {
			board_type_fn *board_fn;
			int32_t m = HB_BOARD_ID;
			for (m = HB_BOARD_ID; m < HB_ARRAY_NUM_MAX; m++) {
				board_fn = bard_type[m];
				board_fn(node, btype_array[i + m]);
			}
			break;
		}
	}
#endif
	if (i == btype_array_len) {
		pr_err("No matching btype found! Can NOT select proper dts!"\
				" Continue booting risks hw damage! Abort!"\
				" Please check board_type_array definition!\n");
		pr_err("ADC ch[0]:%d, ch[1]:%d\n",
				values[0], values[1]);
		ret = -EINVAL;
	} else {
		ret = 0;
	}
free_btype_array:
	free(btype_array);
	return ret;
}


/**
 * hobot_btype_probe() - Basic probe
 * @dev:	corresponding system controller interface device
 *
 * Return: 0 if all goes good, else appropriate error message.
 */
static int hobot_btype_probe(struct udevice *dev)
{
	debug("%s(dev=%p)\n", __func__, dev);
	int ret = 0;

	ret = btype_setup(dev->node_);
	if (ret < 0) {
		pr_err("btype setup failed\n");
		return ret;
	}
	return ret;
}

static const struct udevice_id hobot_btype_ids[] = {
	{
		.compatible = "hobot,btype",
	},
	{ /* Sentinel */ },
};

U_BOOT_DRIVER(hobot_adc_btype) = {
	.name = "hobot_board_btype",
	.id = UCLASS_FIRMWARE,
	.of_match = hobot_btype_ids,
	.probe = hobot_btype_probe,
};
