/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2024 D-Robotic AI Inc.
 */
#include <common.h>
#include <log.h>
#include <sound.h>
#include <audio_codec.h>

#include <dm.h>
#include <i2s.h>
#include <asm/io.h>
#include <linux/bitops.h>

#define CRM_GENERIC_BASE 	(0x32140000)
#define DSP_CLK_ENB		(0x10)
#define DSP_I2S_CLK_SOURCE_SEL	(0x1c)
#define I2S_CLOCK_BASE 		(0x32140800)
#define I2S0_SCLK 		(0xc0)
#define I2S0_SCLK_CHILD 	(0xd0)
#define I2S1_SCLK		(0x100)
#define I2S1_SCLK_CHILD		(0x110)

#define PLL			(1622016000)

#define I2S0_BASE (0x320b0000)
#define I2S1_BASE (0x320c0000)

#define IER 			(0x0)
#define ITER			(0x8)
#define CER			(0xc)
#define CCR			(0x10)
#define TXFFR			(0x18)
#define SR			(0x1c)
#define	LRBR_LTHR(x)		(0x40 * (x) + 0x020)
#define RRBR_RTHR(x)		(0x40 * (x) + 0x024)
#define TER(x)			(0x40 * (x) + 0x02c) //transmitter channel enable
#define TCR(x)			(0x40 * (x) + 0x034) //configuration transmitter(word len)
#define ISR(x)			(0x40 * (x) + 0x038) //interrupt status
#define IMR(x)			(0x40 * (x) + 0x03c) //interrupt mask/unmask
#define TOR(x)			(0x40 * (x) + 0x044) //check overrun transmitter
#define TFCR(x)			(0x40 * (x) + 0x04c) //transmitter fifo configuration(fifo level)
#define TFF(x)			(0x40 * (x) + 0x054) //flush transmitter fifo
#define COMP_PARAM_1		(0x1f4)
#define TSLOT(x)		(0x224 + 0X4 * (x))

#define TXFE			(0x1 << 4)
#define COMP1_FIFO_DEPTH_GLOBAL(r) (((r) & GENMASK(3, 2)) >> 2)

#define MAX_CHAN (2)
#define TDM_MODE (1)

static u32 fifo_th = 0;
static u32 transfer_mode = 0;

static inline void _writel(u32 value, u32 addr) {
	*(volatile u32 *)(uintptr_t)(addr) = (value);
}

static inline u32 _readl(const u32 addr) {
	return (*(volatile u32 *)(uintptr_t)(addr));
}

static inline void calculate_sclk_pre(uint32_t bitclk, int *pre, int *post, int *child) {
	int div_pre, div_post, div_child;
	uint32_t actual_bitclk;
	uint32_t max_offset = bitclk;
	uint32_t offset;

	for (div_pre = 1; div_pre <= 8; div_pre++) {
		for (div_post = 1; div_post <= 64; div_post++) {
			for (div_child = 1; div_child <= 32; div_child++) {
				actual_bitclk = PLL / (div_pre * div_post * div_child);
				offset = abs(bitclk - actual_bitclk);
				if (offset < max_offset) {
					*pre = div_pre - 1;
					*post = div_post - 1;
					*child = div_child - 1;
					max_offset = offset;
					if (offset == 0)
						break;
				}
			}
		}
	}
}

static inline void i2s_clock_set_divisor(int post, int pre, uint32_t offset) {
	uint32_t value;

	value = _readl(I2S_CLOCK_BASE + offset);
	value &= ~(0x3f | (0x07 << 16));
	value |= ((0x1 << 28) | post | (pre << 16));
	_writel(value, I2S_CLOCK_BASE + offset);
}

static inline void i2s_clock_set_child_divisor(int child, uint32_t offset) {
	_writel(child, I2S_CLOCK_BASE + offset);
}

static void i2s_set_clock(struct i2s_uc_priv *priv) {
	u32 value;
	int pre, post, child;
	u32 bitclk = priv->samplingrate * priv->bitspersample * priv->channels;

	if (transfer_mode == TDM_MODE)
		bitclk *= 2;

	//select clock source
	value = _readl(CRM_GENERIC_BASE + DSP_I2S_CLK_SOURCE_SEL);
	if (priv->id == 0)
		value |= (0x1 << 0);
	else
		value |= (0x1 << 2);
	_writel(value, CRM_GENERIC_BASE + DSP_I2S_CLK_SOURCE_SEL);

	//set clock rate
	calculate_sclk_pre(bitclk, &pre, &post, &child);
	if (priv->id == 0) {
		_writel(0x10010020, I2S_CLOCK_BASE + 0xe0);
		i2s_clock_set_divisor(post, pre, I2S0_SCLK);
		i2s_clock_set_child_divisor(child, I2S0_SCLK_CHILD);
	} else {
		_writel(0x10010020, I2S_CLOCK_BASE + 0x120);
		i2s_clock_set_divisor(post, pre, I2S1_SCLK);
                i2s_clock_set_child_divisor(child, I2S1_SCLK_CHILD);
	}
}

/*
 * Default I2S as Master
 */
static int hobot_i2s_init(struct i2s_uc_priv *priv) {
	unsigned int comp_param = 0;
	unsigned int xfer_resolution = 0, ccr = 0, val = 0;
	unsigned int ter = 0;

	switch (priv->bitspersample) {
	case 16:
		xfer_resolution = 2;
		ccr = 0;
		break;
	case 32:
		xfer_resolution = 5;
		ccr = 2;
		break;
	default:
		printf("Invalid bitspersample %d\n", priv->bitspersample);
		return -1;
	}

	comp_param = _readl(priv->base_address + COMP_PARAM_1);
	fifo_th = 1 << ( 1 + COMP1_FIFO_DEPTH_GLOBAL(comp_param));

	for (int i = 0; i < MAX_CHAN / 2; i++) {
		//disable channel
		_writel(0, priv->base_address + TER(i));

		//word len
		_writel(xfer_resolution, priv->base_address + TCR(i));

		//fifo level
		_writel(((fifo_th >> 1) - 1), priv->base_address + TFCR(i));
	}

	for (int i = 0; i < priv->channels; i++) {
		ter |= (0x1 | (0x1 << (i + 8)));
	}
	//enable transmit channel and enable slot
	_writel(ter, priv->base_address + TER(0));

	//bclk ratio
	_writel((ccr << 3), priv->base_address + CCR);

	//set clock
	i2s_set_clock(priv);

	//flush tx fifo
	_writel(1, priv->base_address + TXFFR);

       	if (transfer_mode == TDM_MODE) {
		_writel(((priv->channels - 1) << 8) | (1 << 1), priv->base_address + IER);
		val = _readl(priv->base_address + IER);
		val |= (1 << 5);
		_writel(0x8001 | val, priv->base_address + IER);
	} else {
		_writel(((priv->channels - 1) << 8), priv->base_address + IER);
		val = _readl(priv->base_address + IER);
		_writel(0x8001 | val, priv->base_address + IER);
	}

	//interrupt unmask
	for (int i = 0; i < MAX_CHAN / 2; i++) {
		val = _readl(priv->base_address + IMR(i));
		_writel(val & ~0x30, priv->base_address + IMR(i));

		//tranmist block enable
		_writel(1, priv->base_address + ITER);
	}

	_writel(1, priv->base_address + CER);

	return 0;
}

static int i2s_send_data(struct i2s_uc_priv *priv, void *data, int length) {
	unsigned int isr[4] = {0};
	u16 *ptr = data;

	for (int i = 0; i < fifo_th; i++) {
		if (transfer_mode == TDM_MODE) {
			for (int i = 0; i < priv->channels; i++) {
				_writel(*ptr++, priv->base_address + TSLOT(i));
				length--;
			}
		} else {
			_writel(*ptr++, priv->base_address + LRBR_LTHR(0));
			_writel(*ptr++, priv->base_address + RRBR_RTHR(0));
			length -= sizeof(*ptr);
		}
	}

	while (length > 0) {
		for (int i = 0; i < MAX_CHAN / 2; i++) {
			isr[i] = _readl(priv->base_address + ISR(i));
		}

		for (int i = 0; i < 4; i++) {
			_readl(priv->base_address + TOR(i));
		}

		if ((isr[0] & TXFE)) {
			if (transfer_mode ==  TDM_MODE) {
				for (int i = 0; i < priv->channels; i++) {
					_writel(*ptr++, priv->base_address + TSLOT(i));
					length--;
				}
			} else {
				_writel(*ptr++, priv->base_address + LRBR_LTHR(0));
				_writel(*ptr++, priv->base_address + RRBR_RTHR(0));
				length -= sizeof(*ptr);
			}
		}
	}

	return 0;
}

static int hobot_i2s_tx_data(struct udevice *dev, void *data, uint data_size) {
	struct i2s_uc_priv *priv = dev_get_uclass_priv(dev);

	return i2s_send_data(priv, data, (int)(data_size / (priv->bitspersample / 8)));
}

static int hobot_i2s_probe(struct udevice *dev) {
	struct i2s_uc_priv *priv = dev_get_uclass_priv(dev);
	u32 base;
	int ret;

	base = dev_read_addr(dev);
	if (base == FDT_ADDR_T_NONE) {
		log_debug("Missing i2s base\n");
		return -EINVAL;
	}

	priv->base_address = base;
	if (base == I2S0_BASE)
		priv->id = 0;
	else
		priv->id = 1;

	priv->samplingrate = 16000;
	priv->bitspersample = 16;
	priv->channels = 1;
	priv->rfs = 256;
	priv->bfs = 32;

	ret = hobot_i2s_init(priv);
	if (ret) {
		printf("%s failed\n", __func__);
		return ret;
	}

	printf("%s i2s%d success\n", __func__, priv->id);

	return ret;
}

static const struct i2s_ops hobot_i2s_ops = {
	.tx_data = hobot_i2s_tx_data,
};

static const struct udevice_id hobot_i2s_ids[] = {
	{ .compatible = "hobot, hobot-i2s" },
	{ }
};

U_BOOT_DRIVER(hobot_i2s) = {
	.name = "hobot_i2s",
	.id = UCLASS_I2S,
	.of_match = hobot_i2s_ids,
	.probe = hobot_i2s_probe,
	.ops = &hobot_i2s_ops,
};
