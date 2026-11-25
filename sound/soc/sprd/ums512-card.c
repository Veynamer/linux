// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Otto Pflüger
 */

#include <linux/module.h>
#include <sound/soc.h>

#include "sprd-card-common.h"

SND_SOC_DAILINK_DEFS(fe_fast_playback,
		     DAILINK_COMP_ARRAY(COMP_CPU("VBC_FE_FAST_PLAYBACK")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(fe_capture_dsp,
		     DAILINK_COMP_ARRAY(COMP_CPU("VBC_FE_CAPTURE_DSP")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(voice_hostless,
		     DAILINK_COMP_ARRAY(COMP_CPU("VBC_FE_VOICE")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(voice_capture,
		     DAILINK_COMP_ARRAY(COMP_CPU("VBC_FE_VOICE_CAPTURE")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(be_iis0,
		     DAILINK_COMP_ARRAY(COMP_CPU("VBC_IIS0")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(be_iis1,
		     DAILINK_COMP_ARRAY(COMP_CPU("VBC_IIS1")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(be_iis2,
		     DAILINK_COMP_ARRAY(COMP_CPU("VBC_IIS2")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(be_iis3,
		     DAILINK_COMP_ARRAY(COMP_CPU("VBC_IIS3")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

static const struct snd_soc_pcm_stream voice_params = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 8000,
	.rate_max = 8000,
	.channels_min = 1,
	.channels_max = 1,
};

static const struct snd_soc_dai_link ums512_card_dai_links[] = {
	/* DPCM frontend */
	{
		.name = "FE_FAST_PLAYBACK",
		.stream_name = "FE_FAST_PLAYBACK",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.playback_only = 1,
		SND_SOC_DAILINK_REG(fe_fast_playback),
	},
	{
		.name = "FE_CAPTURE_DSP",
		.stream_name = "FE_CAPTURE_DSP",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.capture_only = 1,
		SND_SOC_DAILINK_REG(fe_capture_dsp),
	},

	/* Modem voice channels */
	{
		.name = "VOICE",
		.stream_name = "VOICE",
		.c2c_params = &voice_params,
		.num_c2c_params = 1,
		SND_SOC_DAILINK_REG(voice_hostless),
	},
	{
		.name = "VOICE_CAPTURE",
		.stream_name = "VOICE_CAPTURE",
		.capture_only = 1,
		SND_SOC_DAILINK_REG(voice_capture),
	},

	/* DPCM backend */
	{
		.name = "IIS0",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBP_CFP,
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(be_iis0),
	},
	{
		.name = "IIS1",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBP_CFP,
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(be_iis1),
	},
	{
		.name = "IIS2",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBP_CFP,
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(be_iis2),
	},
	{
		.name = "IIS3",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBP_CFP,
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(be_iis3),
	},
};

static int ums512_card_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_dai_link *link;
	struct snd_soc_card *card;
	int i, ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	card->dev = dev;
	card->owner = THIS_MODULE;
	card->num_links = ARRAY_SIZE(ums512_card_dai_links);
	card->dai_link = devm_kmemdup(dev, ums512_card_dai_links,
				      sizeof(ums512_card_dai_links), GFP_KERNEL);
	if (!card->dai_link)
		return -ENOMEM;

	/* The DT parser updates platform components for each card instance. */
	for_each_card_prelinks(card, i, link) {
		if (!link->num_platforms)
			continue;
		link->platforms = devm_kmemdup(dev, link->platforms,
					       sizeof(*link->platforms) *
					       link->num_platforms, GFP_KERNEL);
		if (!link->platforms)
			return -ENOMEM;
	}

	ret = sprd_card_parse_of(card);
	if (ret)
		return ret;

	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id ums512_card_of_match[] = {
	{ .compatible = "sprd,ums512-audio-card" },
	{ }
};
MODULE_DEVICE_TABLE(of, ums512_card_of_match);

static struct platform_driver ums512_card_driver = {
	.driver = {
		.name = "snd-soc-ums512-card",
		.of_match_table = ums512_card_of_match,
	},
	.probe = ums512_card_probe,
};

module_platform_driver(ums512_card_driver);

MODULE_DESCRIPTION("Unisoc UMS512 ASoC machine driver");
MODULE_LICENSE("GPL");
