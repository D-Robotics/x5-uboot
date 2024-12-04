/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2024 D-Robotic AI Inc.
 */
#include <common.h>
#include <log.h>
#include <dm/device.h>
#include <i2s.h>
#include <sound.h>
#include <audio_codec.h>

#include <dm.h>

static int hobot_sound_setup(struct udevice *dev) {
	struct sound_uc_priv *uc_priv = dev_get_uclass_priv(dev);
	struct i2s_uc_priv *i2c_priv = dev_get_uclass_priv(uc_priv->i2s);
	int ret;

	if (uc_priv->setup_done)
		return -EALREADY;
	ret = audio_codec_set_params(uc_priv->codec, i2c_priv->id,
			i2c_priv->samplingrate,
			i2c_priv->samplingrate * i2c_priv->rfs,
			i2c_priv->bitspersample,
			i2c_priv->channels);

	if (ret)
		return ret;

	uc_priv->setup_done = true;

	return 0;
}

static int hobot_sound_play(struct udevice *dev, void *data, uint data_size) {
	struct sound_uc_priv *uc_priv = dev_get_uclass_priv(dev);

	return i2s_tx_data(uc_priv->i2s, data, data_size);
}

static int hobot_sound_probe(struct udevice *dev) {
	struct sound_uc_priv *uc_priv = dev_get_uclass_priv(dev);
	struct ofnode_phandle_args args;
	ofnode node;
	int ret;

	node = ofnode_find_subnode(dev_ofnode(dev), "cpu");
	if (!ofnode_valid(node)) {
		log_debug("Failed to fond cpu subnode\n");
		return -EINVAL;
	}

	ret = ofnode_parse_phandle_with_args(node, "sound-dai",
			"#sound-dai-cells", 0, 0, &args);
	if (ret) {
		log_debug("Cannot find i2s phandle: %d\n", ret);
		return ret;
	}

	ret = uclass_get_device_by_ofnode(UCLASS_I2S, args.node, &uc_priv->i2s);
	if (ret) {
		log_debug("Cannot find i2s: %d\n", ret);
		return ret;
	}

	node = ofnode_find_subnode(dev_ofnode(dev), "codec");
	if (!ofnode_valid(node)) {
		log_debug("Failed to found codec subnode\n");
		return -EINVAL;
	}

	ret = ofnode_parse_phandle_with_args(node, "sound-dai",
			"#sound-dai-cells", 0, 0, &args);
	if (ret) {
		log_debug("Cannot find codec phandle: %d\n", ret);
		return ret;
	}

	ret = uclass_get_device_by_ofnode(UCLASS_AUDIO_CODEC, args.node,
			&uc_priv->codec);
	if (ret) {
		log_debug("Cannot find codec: %d\n", ret);
		return ret;
	}

	printf("Probed sound %s with codec %s and i2s %s\n", dev->name,
			uc_priv->codec->name, uc_priv->i2s->name);

	return 0;
}

static const struct sound_ops hobot_sound_ops = {
	.setup = hobot_sound_setup,
	.play = hobot_sound_play,
};

static const struct udevice_id hobot_sound_ids[] = {
	{ .compatible = "hobot,audio-codec" },
	{ }
};

U_BOOT_DRIVER(hobot_sound) = {
	.name = "hobot_sound",
	.id = UCLASS_SOUND,
	.of_match = hobot_sound_ids,
	.probe = hobot_sound_probe,
	.ops = &hobot_sound_ops,
};
