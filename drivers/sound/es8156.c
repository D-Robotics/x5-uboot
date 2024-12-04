/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2024 D-Robotic AI Inc.
 */
#include <common.h>
#include <log.h>
#include <dm/device.h>
#include <audio_codec.h>
#include <sound.h>
#include <linux/delay.h>
#include <i2c.h>

#define ES8156_RESET_REG00             0x00

#define ES8156_MAINCLOCK_CTL_REG01     0x01
#define ES8156_SCLK_MODE_REG02                   0x02
#define ES8156_LRCLK_DIV_H_REG03     0x03
#define ES8156_LRCLK_DIV_L_REG04    0x04
#define ES8156_SCLK_DIV_REG05    0x05
#define ES8156_NFS_CONFIG_REG06    0x06
#define ES8156_MISC_CONTROL1_REG07    0x07
#define ES8156_CLOCK_ON_OFF_REG08      0x08
#define ES8156_MISC_CONTROL2_REG09     0x09
#define ES8156_TIME_CONTROL1_REG0A        0x0a
#define ES8156_TIME_CONTROL2_REG0B        0x0b

#define ES8156_CHIP_STATUS_REG0C       0x0c
#define ES8156_P2S_CONTROL_REG0D           0x0d
#define ES8156_DAC_OSR_COUNTER_REG10       0x10

#define ES8156_DAC_SDP_REG11          0x11
#define ES8156_AUTOMUTE_SET_REG12           0x12
#define ES8156_DAC_MUTE_REG13         0x13
#define ES8156_VOLUME_CONTROL_REG14      0x14

/*
* ALC Control
*/
#define ES8156_ALC_CONFIG1_REG15         0x15
#define ES8156_ALC_CONFIG2_REG16         0x16
#define ES8156_ALC_CONFIG3_REG17        0x17
#define ES8156_MISC_CONTROL3_REG18     0x18
#define ES8156_EQ_CONTROL1_REG19         0x19
#define ES8156_EQ_CONTROL2_REG1A         0x1a
/*
* Analog System Control
*/
#define ES8156_ANALOG_SYS1_REG20        0x20
#define ES8156_ANALOG_SYS2_REG21        0x21
#define ES8156_ANALOG_SYS3_REG22   0x22
#define ES8156_ANALOG_SYS4_REG23      0x23
#define ES8156_ANALOG_LP_REG24      0x24
#define ES8156_ANALOG_SYS5_REG25         0x25
/*
* Chip Information
*/
#define ES8156_I2C_PAGESEL_REGFC         0xFC
#define ES8156_CHIPID1_REGFD       0xFD
#define ES8156_CHIPID0_REGFE        0xFE
#define ES8156_CHIP_VERSION_REGFF         0xFF

struct es8156_priv {
	struct udevice *dev;
};

static int es8156_init(struct es8156_priv *priv) {
	unsigned char reg_val;

	reg_val = 0x4;
	dm_i2c_write(priv->dev, ES8156_SCLK_MODE_REG02, &reg_val, 1);

	reg_val = 0x2a;
	dm_i2c_write(priv->dev, ES8156_ANALOG_SYS1_REG20, &reg_val, 1);

	reg_val = 0x3c;
	dm_i2c_write(priv->dev, ES8156_ANALOG_SYS2_REG21, &reg_val, 1);

	reg_val = 0x00;
        dm_i2c_write(priv->dev, ES8156_ANALOG_SYS3_REG22, &reg_val, 1);

	reg_val = 0x07;
        dm_i2c_write(priv->dev, ES8156_ANALOG_LP_REG24, &reg_val, 1);

	reg_val = 0x3a;
        dm_i2c_write(priv->dev, ES8156_ANALOG_SYS4_REG23, &reg_val, 1);

	reg_val = 0x01;
        dm_i2c_write(priv->dev, ES8156_TIME_CONTROL1_REG0A, &reg_val, 1);

	reg_val = 0x01;
        dm_i2c_write(priv->dev, ES8156_TIME_CONTROL2_REG0B, &reg_val, 1);

	reg_val = 0xbf;
        dm_i2c_write(priv->dev, ES8156_VOLUME_CONTROL_REG14, &reg_val, 1);

	reg_val = 0x21;
        dm_i2c_write(priv->dev, ES8156_MAINCLOCK_CTL_REG01, &reg_val, 1);

	reg_val = 0x14;
        dm_i2c_write(priv->dev, ES8156_P2S_CONTROL_REG0D, &reg_val, 1);

	reg_val = 0x00;
        dm_i2c_write(priv->dev, ES8156_MISC_CONTROL3_REG18, &reg_val, 1);

	reg_val = 0x3f;
        dm_i2c_write(priv->dev, ES8156_CLOCK_ON_OFF_REG08, &reg_val, 1);

	reg_val = 0x02;
        dm_i2c_write(priv->dev, ES8156_RESET_REG00, &reg_val, 1);

	reg_val = 0x03;
        dm_i2c_write(priv->dev, ES8156_RESET_REG00, &reg_val, 1);

	reg_val = 0x20;
        dm_i2c_write(priv->dev, ES8156_ANALOG_SYS5_REG25, &reg_val, 1);

	return 0;
}

static int es8156_set_fmt(struct es8156_priv *priv) {
	unsigned char reg_val;

	reg_val = 0x04;
	dm_i2c_write(priv->dev, ES8156_SCLK_MODE_REG02, &reg_val, 1);

	reg_val = 0x33;
        dm_i2c_write(priv->dev, ES8156_DAC_SDP_REG11, &reg_val, 1);

	reg_val = 0x0;
	dm_i2c_write(priv->dev, ES8156_MISC_CONTROL2_REG09, &reg_val, 1);

	reg_val = 0x11;
	dm_i2c_write(priv->dev, ES8156_DAC_MUTE_REG13, &reg_val, 1);
	return 0;
}

static int es8156_set_params(struct udevice *dev, int interface, int rate,
		int mclk_freq, int bits_per_sample, uint channels) {
	struct es8156_priv *priv = dev_get_priv(dev);

	es8156_init(priv);

	es8156_set_fmt(priv);

	printf("%s success\n", __func__);

	return 0;
}

static void es8156_device_init(struct es8156_priv *priv) {
	unsigned char reg_val1 = 0x1c;
	unsigned char reg_val2 = 0x3;

	//reset
	dm_i2c_write(priv->dev, ES8156_RESET_REG00, &reg_val1, 1);
	udelay(5000);
	dm_i2c_write(priv->dev, ES8156_RESET_REG00, &reg_val2, 1);
}

static int es8156_probe(struct udevice *dev) {
	struct es8156_priv *priv = dev_get_priv(dev);

	priv->dev = dev;
	es8156_device_init(priv);

	printf("%s success\n", __func__);

	return 0;
}

static const struct audio_codec_ops es8156_ops = {
	.set_params = es8156_set_params,
};

static const struct udevice_id es8156_ids[] = {
	{ .compatible = "hobot,es8156" },
	{ }
};

U_BOOT_DRIVER(es8156) = {
	.name = "es8156",
	.id = UCLASS_AUDIO_CODEC,
	.of_match = es8156_ids,
	.probe = es8156_probe,
	.ops = &es8156_ops,
	.priv_auto = sizeof(struct es8156_priv),
};
