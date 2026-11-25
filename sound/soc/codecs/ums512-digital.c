// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * UMS512 digital audio interface, based on the UMS9230 driver.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mux/consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#define AUD_TOP_CTL		0x0000
#define AUD_AUD_CTL		0x0004
#define AUD_I2S_CTL		0x0008
#define AUD_DAC_CTL		0x000C
#define AUD_SDM_CTL0		0x0010
#define AUD_SDM_CTL1		0x0014
#define AUD_ADC_CTL		0x0018
#define AUD_LOOP_CTL		0x001C
#define AUD_AUD_STS0		0x0020
#define AUD_INT_CLR		0x0024
#define AUD_INT_EN		0x0028
#define AUDIF_FIFO_CTL		0x002C
#define AUD_DMIC_CTL		0x0030
#define AUD_ADC1_I2S_CTL	0x0034
#define AUD_DAC_SDM_L		0x0038
#define AUD_DAC_SDM_H		0x003C
#define AUD_DNS_AUTOGATE_EN	0x0084
#define AUD_DNS_SW		0x0088

struct ums512_aud_priv {
	struct regmap *regmap;
	struct clk *clk;
	struct mux_control *iis_mux[2];
};

static int ums512_aud_iis_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct ums512_aud_priv *priv = snd_soc_component_get_drvdata(component);
	struct mux_control *mux = priv->iis_mux[w->shift];

	if (SND_SOC_DAPM_EVENT_ON(event))
		return mux_control_select(mux, 1);

	return mux_control_deselect(mux);
}

static const struct snd_soc_dapm_widget ums512_aud_iis_widgets[] = {
	SND_SOC_DAPM_SUPPLY("Playback IIS Mux", SND_SOC_NOPM, 0, 0,
			    ums512_aud_iis_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("Capture IIS Mux", SND_SOC_NOPM, 1, 0,
			    ums512_aud_iis_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route ums512_aud_iis_routes[] = {
	{ "I2S RX", NULL, "Playback IIS Mux" },
	{ "I2S TX", NULL, "Capture IIS Mux" },
};

static int ums512_aud_component_probe(struct snd_soc_component *component)
{
	struct ums512_aud_priv *priv = snd_soc_component_get_drvdata(component);
	struct snd_soc_dapm_context *dapm = snd_soc_component_to_dapm(component);
	int ret;

	if (!priv->iis_mux[0])
		return 0;

	ret = snd_soc_dapm_new_controls(dapm, ums512_aud_iis_widgets,
					ARRAY_SIZE(ums512_aud_iis_widgets));
	if (ret)
		return ret;

	return snd_soc_dapm_add_routes(dapm, ums512_aud_iis_routes,
				       ARRAY_SIZE(ums512_aud_iis_routes));
}

static const struct snd_kcontrol_new ums512_aud_controls[] = {
	SOC_SINGLE("Swap ADC Channels", AUD_I2S_CTL, 2, 1, 0),
	SOC_SINGLE("Swap DAC Channels", AUD_I2S_CTL, 1, 1, 0),
};

static const struct snd_soc_dapm_widget ums512_aud_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("I2S RX", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("I2S TX", NULL, 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_OUTPUT("AUDIF_TX"),
	SND_SOC_DAPM_INPUT("AUDIF_RX"),

	SND_SOC_DAPM_SUPPLY("Digital DACL Switch", AUD_TOP_CTL, 0, 0,
			    NULL, 0),
	SND_SOC_DAPM_SUPPLY("Digital DACR Switch", AUD_TOP_CTL, 2, 0,
			    NULL, 0),
	SND_SOC_DAPM_SUPPLY("Digital ADCL Switch", AUD_TOP_CTL, 1, 0,
			    NULL, 0),
	SND_SOC_DAPM_SUPPLY("Digital ADCR Switch", AUD_TOP_CTL, 3, 0,
			    NULL, 0),
};

static const struct snd_soc_dapm_route ums512_aud_dapm_routes[] = {
	/* Playback */
	{"I2S RX", NULL, "UMS512 Playback"},
	{"AUDIF_TX", NULL, "I2S RX"},
	{"AUDIF_TX", NULL, "Digital DACL Switch"},
	{"AUDIF_TX", NULL, "Digital DACR Switch"},

	/* Capture */
	{"UMS512 Capture", NULL, "I2S TX"},
	{"I2S TX", NULL, "AUDIF_RX"},
	{"AUDIF_RX", NULL, "Digital ADCL Switch"},
	{"AUDIF_RX", NULL, "Digital ADCR Switch"},
};

static const struct snd_soc_component_driver ums512_codec = {
	.probe			= ums512_aud_component_probe,
	.controls		= ums512_aud_controls,
	.num_controls		= ARRAY_SIZE(ums512_aud_controls),
	.dapm_widgets		= ums512_aud_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(ums512_aud_dapm_widgets),
	.dapm_routes		= ums512_aud_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(ums512_aud_dapm_routes),
};

static int ums512_convert_playback_rate(int rate)
{
	switch (rate) {
	case 8000:
		return 10;
	case 9600:
		return 9;
	case 11025:
		return 8;
	case 12000:
		return 7;
	case 16000:
		return 6;
	case 22050:
		return 5;
	case 24000:
		return 4;
	case 32000:
		return 3;
	case 44100:
		return 2;
	case 48000:
		return 1;
	case 96000:
		return 0;
	default:
		return -EINVAL;
	}
}

static int ums512_convert_capture_rate(int rate)
{
	switch (rate) {
	case 8000:
		return 2;
	case 16000:
		return 4;
	case 32000:
		return 8;
	case 48000:
		return 12;
	default:
		return -EINVAL;
	}
}

static int ums512_digital_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct ums512_aud_priv *priv = snd_soc_component_get_drvdata(dai->component);
	int fs_mode, ret;

	switch (substream->stream) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		fs_mode = ums512_convert_playback_rate(params_rate(params));
		if (fs_mode < 0) {
			dev_err(dai->dev, "invalid playback rate %d\n",
				params_rate(params));
			return fs_mode;
		}

		ret = snd_soc_component_update_bits(dai->component, AUD_DAC_CTL,
						    0xf, fs_mode);
		if (ret < 0)
			return ret;

		/*
		 * The DNS block can override the SC2730 headphone gain and
		 * leave playback almost muted. Use the vendor bypass setup.
		 * Force the writes in case firmware has restored its defaults.
		 */
		ret = regmap_write_bits(priv->regmap, AUD_DNS_AUTOGATE_EN,
					GENMASK(15, 0), 0x3303);
		if (ret)
			return ret;
		ret = regmap_write_bits(priv->regmap, AUD_DNS_SW, BIT(0), 0);
		break;
	case SNDRV_PCM_STREAM_CAPTURE:
		fs_mode = ums512_convert_capture_rate(params_rate(params));
		if (fs_mode < 0) {
			dev_err(dai->dev, "invalid capture rate %d\n",
				params_rate(params));
			return fs_mode;
		}

		ret = snd_soc_component_update_bits(dai->component, AUD_ADC_CTL,
						    0xf, fs_mode);
		break;
	default:
		return -EINVAL;
	}

	return ret < 0 ? ret : 0;
}

static const struct snd_soc_dai_ops ums512_digital_dai_ops = {
	.hw_params = ums512_digital_hw_params,
};

#define SPRD_PCM_DA_RATES	(SNDRV_PCM_RATE_8000_48000 |	\
				 SNDRV_PCM_RATE_12000 |		\
				 SNDRV_PCM_RATE_24000 |		\
				 SNDRV_PCM_RATE_96000)

#define SPRD_PCM_AD_RATES	(SNDRV_PCM_RATE_8000 |		\
				 SNDRV_PCM_RATE_16000 |		\
				 SNDRV_PCM_RATE_32000 |		\
				 SNDRV_PCM_RATE_48000)

#define SPRD_PCM_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE |	\
				 SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_driver ums512_aud_dais[] = {
	{
		.name = "UMS512_AUDIF_OUT",
		.id = 0,
		.playback = {
			.stream_name = "UMS512 Playback",
			.rates = SPRD_PCM_DA_RATES,
			.formats = SPRD_PCM_FORMATS,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ums512_digital_dai_ops,
	},
	{
		.name = "UMS512_AUDIF_IN",
		.id = 1,
		.capture = {
			.stream_name = "UMS512 Capture",
			.rates = SPRD_PCM_AD_RATES,
			.formats = SPRD_PCM_FORMATS,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ums512_digital_dai_ops,
	},
};

static const struct regmap_range ums512_aud_wr_ranges[] = {
	regmap_reg_range(AUD_TOP_CTL, AUD_LOOP_CTL),
	regmap_reg_range(AUD_INT_CLR, AUD_DAC_SDM_H),
	regmap_reg_range(AUD_DNS_AUTOGATE_EN, AUD_DNS_SW),
};

static const struct regmap_access_table ums512_aud_wr_table = {
	.yes_ranges = ums512_aud_wr_ranges,
	.n_yes_ranges = ARRAY_SIZE(ums512_aud_wr_ranges),
};

static const struct regmap_range ums512_aud_rd_ranges[] = {
	regmap_reg_range(AUD_TOP_CTL, AUD_DAC_SDM_H),
	regmap_reg_range(AUD_DNS_AUTOGATE_EN, AUD_DNS_SW),
};

static const struct regmap_access_table ums512_aud_rd_table = {
	.yes_ranges = ums512_aud_rd_ranges,
	.n_yes_ranges = ARRAY_SIZE(ums512_aud_rd_ranges),
};

static const struct regmap_config ums512_aud_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = AUD_DNS_SW,
	.cache_type = REGCACHE_FLAT,
	.rd_table = &ums512_aud_rd_table,
	.wr_table = &ums512_aud_wr_table,
};

static void ums512_aud_disable(void *data)
{
	struct device *dev = data;
	struct ums512_aud_priv *priv = dev_get_drvdata(dev);

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		clk_disable_unprepare(priv->clk);
	pm_runtime_set_suspended(dev);
}

static int ums512_aud_dev_probe(struct platform_device *pdev)
{
	struct ums512_aud_priv *priv;
	struct device *dev = &pdev->dev;
	void __iomem *base;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	if (device_property_present(dev, "mux-controls")) {
		priv->iis_mux[0] = devm_mux_control_get(dev, "playback");
		if (IS_ERR(priv->iis_mux[0]))
			return dev_err_probe(dev, PTR_ERR(priv->iis_mux[0]),
					     "cannot get playback IIS mux\n");

		priv->iis_mux[1] = devm_mux_control_get(dev, "capture");
		if (IS_ERR(priv->iis_mux[1]))
			return dev_err_probe(dev, PTR_ERR(priv->iis_mux[1]),
					     "cannot get capture IIS mux\n");
	}

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk)) {
		return dev_err_probe(dev, PTR_ERR(priv->clk),
				     "cannot get clock");
	}

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->regmap = devm_regmap_init_mmio(dev, base,
					     &ums512_aud_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable clock\n");

	ret = pm_runtime_set_active(dev);
	if (ret) {
		clk_disable_unprepare(priv->clk);
		return ret;
	}
	pm_runtime_set_autosuspend_delay(dev, 3000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_enable(dev);
	ret = devm_add_action_or_reset(dev, ums512_aud_disable, dev);
	if (ret)
		return ret;

	return devm_snd_soc_register_component(dev, &ums512_codec,
					       ums512_aud_dais,
					       ARRAY_SIZE(ums512_aud_dais));
}

static int __maybe_unused ums512_aud_runtime_suspend(struct device *dev)
{
	struct ums512_aud_priv *priv = dev_get_drvdata(dev);

	regcache_cache_only(priv->regmap, true);
	regcache_mark_dirty(priv->regmap);

	clk_disable_unprepare(priv->clk);

	return 0;
}

static int __maybe_unused ums512_aud_runtime_resume(struct device *dev)
{
	struct ums512_aud_priv *priv = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(dev, "failed to enable clock\n");
		return ret;
	}

	regcache_cache_only(priv->regmap, false);
	ret = regcache_sync(priv->regmap);
	if (ret) {
		regcache_cache_only(priv->regmap, true);
		clk_disable_unprepare(priv->clk);
	}

	return ret;
}

static const struct dev_pm_ops ums512_aud_pm_ops = {
	SET_RUNTIME_PM_OPS(ums512_aud_runtime_suspend,
			   ums512_aud_runtime_resume, NULL)
};

static const struct of_device_id ums512_aud_match_table[] = {
	{ .compatible = "sprd,ums512-digital-codec" },
	{ }
};
MODULE_DEVICE_TABLE(of, ums512_aud_match_table);

static struct platform_driver ums512_aud_driver = {
	.probe = ums512_aud_dev_probe,
	.driver = {
		.name = "ums512-digital-codec",
		.of_match_table = ums512_aud_match_table,
		.pm = &ums512_aud_pm_ops,
	},
};

module_platform_driver(ums512_aud_driver);

MODULE_DESCRIPTION("Unisoc UMS512 digital audio codec driver");
MODULE_LICENSE("GPL");
