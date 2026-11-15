// SPDX-License-Identifier: GPL-2.0
//
// mt6785-mt6359.c  --  MT6785 (Redmi Note 8 Pro, begonia) + MT6359 machine driver
//
// Port of mt8192-mt6359-rt1015-rt5682 (Jiaxin Yu, MediaTek) to MT6785:
// a card with only the built-in MT6359 codec (earpiece / headphone / analog
// microphones). The external SmartPA (TFA9874 over I2S) - a separate step.

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../../codecs/mt6359.h"
#include "../../codecs/mt6359-accdet.h"
#include "../common/mtk-afe-platform-driver.h"
#include "../common/mtk-soc-card.h"
#include "../common/mtk-soundcard-driver.h"
#include "mt6785-afe-common.h"
#include "mt6785-afe-clk.h"
#include "mt6785-afe-gpio.h"

#define DRIVER_NAME "mt6785_mt6359"
#define MT6785_MT6359_CARD_NAME "mt6785_mt6359"
#define MT6785_MT6359_OF_NAME "mediatek,mt6785_mt6359"

static int mt6785_mt6359_mtkaif_calibration(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *cmpnt_afe =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct snd_soc_component *cmpnt_codec =
		snd_soc_rtd_to_codec(rtd, 0)->component;
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt_afe);
	struct mt6785_afe_private *afe_priv = afe->platform_priv;
	int phase;
	unsigned int monitor;
	int test_done_1, test_done_2, test_done_3;
	int cycle_1, cycle_2, cycle_3;
	int prev_cycle_1, prev_cycle_2, prev_cycle_3;
	int chosen_phase_1, chosen_phase_2, chosen_phase_3;
	int counter;
	int mtkaif_calib_ok;

	pm_runtime_get_sync(afe->dev);
	mt6785_afe_gpio_request(afe->dev, true, MT6785_DAI_ADDA, 1);
	mt6785_afe_gpio_request(afe->dev, true, MT6785_DAI_ADDA, 0);
	mt6785_afe_gpio_request(afe->dev, true, MT6785_DAI_ADDA_CH34, 1);
	mt6785_afe_gpio_request(afe->dev, true, MT6785_DAI_ADDA_CH34, 0);

	mt6359_mtkaif_calibration_enable(cmpnt_codec);

	/* set clock protocol 2 */
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x38);
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x39);

	/* set test type to synchronizer pulse */
	regmap_update_bits(afe_priv->topckgen,
			   CKSYS_AUD_TOP_CFG, 0xffff, 0x4);

	mtkaif_calib_ok = true;
	afe_priv->mtkaif_calibration_num_phase = 42;	/* mt6359: 0 ~ 42 */
	afe_priv->mtkaif_chosen_phase[0] = -1;
	afe_priv->mtkaif_chosen_phase[1] = -1;
	afe_priv->mtkaif_chosen_phase[2] = -1;

	for (phase = 0;
	     phase <= afe_priv->mtkaif_calibration_num_phase &&
	     mtkaif_calib_ok;
	     phase++) {
		mt6359_set_mtkaif_calibration_phase(cmpnt_codec,
						    phase, phase, phase);

		regmap_update_bits(afe_priv->topckgen,
				   CKSYS_AUD_TOP_CFG, 0x1, 0x1);

		test_done_1 = 0;
		test_done_2 = 0;
		test_done_3 = 0;
		cycle_1 = -1;
		cycle_2 = -1;
		cycle_3 = -1;
		counter = 0;
		while (test_done_1 == 0 ||
		       test_done_2 == 0 ||
		       test_done_3 == 0) {
			regmap_read(afe_priv->topckgen,
				    CKSYS_AUD_TOP_MON, &monitor);

			test_done_1 = (monitor >> 28) & 0x1;
			test_done_2 = (monitor >> 29) & 0x1;
			test_done_3 = (monitor >> 30) & 0x1;
			if (test_done_1 == 1)
				cycle_1 = monitor & 0xf;

			if (test_done_2 == 1)
				cycle_2 = (monitor >> 4) & 0xf;

			if (test_done_3 == 1)
				cycle_3 = (monitor >> 8) & 0xf;

			/* handle if never test done */
			if (++counter > 10000) {
				dev_err(afe->dev, "%s(), test fail, cycle_1 %d, cycle_2 %d, cycle_3 %d, monitor 0x%x\n",
					__func__,
					cycle_1, cycle_2, cycle_3, monitor);
				mtkaif_calib_ok = false;
				break;
			}
		}

		if (phase == 0) {
			prev_cycle_1 = cycle_1;
			prev_cycle_2 = cycle_2;
			prev_cycle_3 = cycle_3;
		}

		if (cycle_1 != prev_cycle_1 &&
		    afe_priv->mtkaif_chosen_phase[0] < 0) {
			afe_priv->mtkaif_chosen_phase[0] = phase - 1;
			afe_priv->mtkaif_phase_cycle[0] = prev_cycle_1;
		}

		if (cycle_2 != prev_cycle_2 &&
		    afe_priv->mtkaif_chosen_phase[1] < 0) {
			afe_priv->mtkaif_chosen_phase[1] = phase - 1;
			afe_priv->mtkaif_phase_cycle[1] = prev_cycle_2;
		}

		if (cycle_3 != prev_cycle_3 &&
		    afe_priv->mtkaif_chosen_phase[2] < 0) {
			afe_priv->mtkaif_chosen_phase[2] = phase - 1;
			afe_priv->mtkaif_phase_cycle[2] = prev_cycle_3;
		}

		regmap_update_bits(afe_priv->topckgen,
				   CKSYS_AUD_TOP_CFG, 0x1, 0x0);

		if (afe_priv->mtkaif_chosen_phase[0] >= 0 &&
		    afe_priv->mtkaif_chosen_phase[1] >= 0 &&
		    afe_priv->mtkaif_chosen_phase[2] >= 0)
			break;
	}

	if (afe_priv->mtkaif_chosen_phase[0] < 0)
		chosen_phase_1 = 0;
	else
		chosen_phase_1 = afe_priv->mtkaif_chosen_phase[0];

	if (afe_priv->mtkaif_chosen_phase[1] < 0)
		chosen_phase_2 = 0;
	else
		chosen_phase_2 = afe_priv->mtkaif_chosen_phase[1];

	if (afe_priv->mtkaif_chosen_phase[2] < 0)
		chosen_phase_3 = 0;
	else
		chosen_phase_3 = afe_priv->mtkaif_chosen_phase[2];

	mt6359_set_mtkaif_calibration_phase(cmpnt_codec,
					    chosen_phase_1,
					    chosen_phase_2,
					    chosen_phase_3);

	/* disable rx fifo */
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x38);

	mt6359_mtkaif_calibration_disable(cmpnt_codec);

	mt6785_afe_gpio_request(afe->dev, false, MT6785_DAI_ADDA, 1);
	mt6785_afe_gpio_request(afe->dev, false, MT6785_DAI_ADDA, 0);
	mt6785_afe_gpio_request(afe->dev, false, MT6785_DAI_ADDA_CH34, 1);
	mt6785_afe_gpio_request(afe->dev, false, MT6785_DAI_ADDA_CH34, 0);
	pm_runtime_put(afe->dev);

	dev_info(afe->dev, "MTKAIFCAL ok=%d chosen_phase[0/1/2]=%d/%d/%d phase_cycle[0/1/2]=%d/%d/%d\n",
		 mtkaif_calib_ok,
		 afe_priv->mtkaif_chosen_phase[0], afe_priv->mtkaif_chosen_phase[1],
		 afe_priv->mtkaif_chosen_phase[2],
		 afe_priv->mtkaif_phase_cycle[0], afe_priv->mtkaif_phase_cycle[1],
		 afe_priv->mtkaif_phase_cycle[2]);

	return 0;
}

/* 3.5 mm jack: headset/headphone detection (PMIC ACCDET) -> jack "Headset Jack" (+ an input device for PulseAudio) */
static struct snd_soc_jack_pin mt6785_mt6359_headset_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
};

static int mt6785_mt6359_accdet_init(struct snd_soc_pcm_runtime *rtd)
{
	struct mtk_soc_card_data *soc_card_data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_jack *jack = &soc_card_data->card_data->jacks[0];
	int ret;

	if (!soc_card_data->accdet)
		return 0;

	ret = snd_soc_card_jack_new_pins(rtd->card, "Headset Jack",
					 SND_JACK_HEADSET | SND_JACK_BTN_0 |
					 SND_JACK_BTN_1 | SND_JACK_BTN_2 |
					 SND_JACK_BTN_3,
					 jack, mt6785_mt6359_headset_jack_pins,
					 ARRAY_SIZE(mt6785_mt6359_headset_jack_pins));
	if (ret) {
		dev_err(rtd->dev, "Headset Jack create failed: %d\n", ret);
		return ret;
	}

	ret = mt6359_accdet_enable_jack_detect(soc_card_data->accdet, jack);
	if (ret)
		dev_err(rtd->dev, "Headset Jack enable failed: %d\n", ret);
	else
		dev_info(rtd->dev, "3.5 mm jack detection enabled (accdet)\n");
	return ret;
}

static int mt6785_mt6359_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *cmpnt_afe =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct snd_soc_component *cmpnt_codec =
		snd_soc_rtd_to_codec(rtd, 0)->component;
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt_afe);
	struct mt6785_afe_private *afe_priv = afe->platform_priv;

	/* set mtkaif protocol */
	mt6359_set_mtkaif_protocol(cmpnt_codec,
				   MT6359_MTKAIF_PROTOCOL_2_CLK_P2);
	afe_priv->mtkaif_protocol = MTKAIF_PROTOCOL_2_CLK_P2;

	/* mtkaif calibration */
	mt6785_mt6359_mtkaif_calibration(rtd);

	mt6785_mt6359_accdet_init(rtd);

	return 0;
}

/* FE */
SND_SOC_DAILINK_DEFS(playback1,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(playback12,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL12")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(playback2,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL2")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(playback3,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL3")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture1,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL1")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture2,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL2")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture3,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL3")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture_mono1,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL_MONO_1")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* BE */
SND_SOC_DAILINK_DEFS(primary_codec,
		     DAILINK_COMP_ARRAY(COMP_CPU("ADDA")),
		     DAILINK_COMP_ARRAY(COMP_CODEC("mt6359-sound",
						   "mt6359-snd-codec-aif1")),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(primary_codec_ch34,
		     DAILINK_COMP_ARRAY(COMP_CPU("ADDA_CH34")),
		     DAILINK_COMP_ARRAY(COMP_CODEC("mt6359-sound",
						   "mt6359-snd-codec-aif2")),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

/*
 * Speaker: SmartPA TFA9874 (i2c6@0x34) on I2S3. The link is always the last one in
 * the array; if the tfa9874 driver is not loaded/not in DT, soc_probe cuts it off
 * (num_links--) and uses the widgets/routes/controls sets without the speaker,
 * so that the card (headphones/microphone) comes up without the amplifier too.
 */
SND_SOC_DAILINK_DEFS(speaker,
		     DAILINK_COMP_ARRAY(COMP_CPU("I2S3")),
		     DAILINK_COMP_ARRAY(COMP_CODEC("6-0034", "tfa9874-aif")),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* According to the container the amplifier expects I2S with 2 slots of 32 bits, 48 kHz. */
static int mt6785_mt6359_i2s3_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					      struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
						      SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;
	snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
	snd_mask_set_format(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
			    SNDRV_PCM_FORMAT_S32_LE);
	return 0;
}

static struct snd_soc_dai_link mt6785_mt6359_dai_links[] = {
	{
		.name = "Playback_1",
		.stream_name = "Playback_1",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.playback_only = 1,
		SND_SOC_DAILINK_REG(playback1),
	},
	{
		.name = "Playback_12",
		.stream_name = "Playback_12",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.playback_only = 1,
		SND_SOC_DAILINK_REG(playback12),
	},
	{
		.name = "Playback_2",
		.stream_name = "Playback_2",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.playback_only = 1,
		SND_SOC_DAILINK_REG(playback2),
	},
	{
		.name = "Playback_3",
		.stream_name = "Playback_3",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.playback_only = 1,
		.ops = &mtk_soundcard_common_playback_ops,
		SND_SOC_DAILINK_REG(playback3),
	},
	{
		.name = "Capture_1",
		.stream_name = "Capture_1",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.capture_only = 1,
		.ops = &mtk_soundcard_common_capture_ops,
		SND_SOC_DAILINK_REG(capture1),
	},
	{
		.name = "Capture_2",
		.stream_name = "Capture_2",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.capture_only = 1,
		.ops = &mtk_soundcard_common_playback_ops,
		SND_SOC_DAILINK_REG(capture2),
	},
	{
		.name = "Capture_3",
		.stream_name = "Capture_3",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.capture_only = 1,
		SND_SOC_DAILINK_REG(capture3),
	},
	{
		.name = "Capture_Mono_1",
		.stream_name = "Capture_Mono_1",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.capture_only = 1,
		SND_SOC_DAILINK_REG(capture_mono1),
	},
	{
		.name = "Primary Codec",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.init = mt6785_mt6359_init,
		SND_SOC_DAILINK_REG(primary_codec),
	},
	{
		.name = "Primary Codec CH34",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(primary_codec_ch34),
	},
	/* MUST BE THE LAST ONE (see mt6785_mt6359_soc_card_probe) */
	{
		.name = "Speaker",
		.no_pcm = 1,
		.playback_only = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBC_CFC,
		.ignore_suspend = 1,
		.be_hw_params_fixup = mt6785_mt6359_i2s3_hw_params_fixup,
		SND_SOC_DAILINK_REG(speaker),
	},
};

static const struct snd_soc_dapm_widget mt6785_mt6359_widgets[] = {
	SND_SOC_DAPM_SPK("Earpiece", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Main Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
};

static const struct snd_soc_dapm_route mt6785_mt6359_routes[] = {
	{ "Earpiece", NULL, "Receiver" },
	{ "Headphone Jack", NULL, "Headphone L" },
	{ "Headphone Jack", NULL, "Headphone R" },
	/*
	 * TODO(hardware): which AINx is wired where on the begonia board - clarify
	 * with alsa-mixer (ADC_L_Mux/ADC_R_Mux) and by ear. For now: AIN0 - main
	 * microphone, AIN1 - headset.
	 */
	{ "AIN0", NULL, "Main Mic" },
	{ "AIN1", NULL, "Headset Mic" },
};

static const struct snd_kcontrol_new mt6785_mt6359_controls[] = {
	SOC_DAPM_PIN_SWITCH("Earpiece"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Main Mic"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
};

/* the same + speaker (used when tfa9874 is found and bound to its driver) */
static const struct snd_soc_dapm_widget mt6785_mt6359_widgets_spk[] = {
	SND_SOC_DAPM_SPK("Earpiece", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Main Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

static const struct snd_soc_dapm_route mt6785_mt6359_routes_spk[] = {
	{ "Earpiece", NULL, "Receiver" },
	{ "Headphone Jack", NULL, "Headphone L" },
	{ "Headphone Jack", NULL, "Headphone R" },
	{ "AIN0", NULL, "Main Mic" },
	{ "AIN1", NULL, "Headset Mic" },
	{ "Speaker", NULL, "Speaker Out" },
};

static const struct snd_kcontrol_new mt6785_mt6359_controls_spk[] = {
	SOC_DAPM_PIN_SWITCH("Earpiece"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Main Mic"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
};

static struct snd_soc_card mt6785_mt6359_card = {
	.name = MT6785_MT6359_CARD_NAME,
	.driver_name = DRIVER_NAME,
	.owner = THIS_MODULE,
	.dai_link = mt6785_mt6359_dai_links,
	.num_links = ARRAY_SIZE(mt6785_mt6359_dai_links),
	.controls = mt6785_mt6359_controls,
	.num_controls = ARRAY_SIZE(mt6785_mt6359_controls),
	.dapm_widgets = mt6785_mt6359_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt6785_mt6359_widgets),
	.dapm_routes = mt6785_mt6359_routes,
	.num_dapm_routes = ARRAY_SIZE(mt6785_mt6359_routes),
};

/*
 * The TINYCONN muxes of I2S1/I2S3 after reset = "DL1" (see mt6785-afe-pcm.c),
 * and reading their register at card registration bypasses the cache (-EBUSY), so
 * DAPM considers the DL1->I2S3 path connected and ANY playback (to the headphones)
 * would get attached to the speaker BE. Explicitly set NONE through a DAPM put: it updates both
 * the register and the path state. Enable the speaker deliberately:
 *   amixer -c 0 cset name='I2S3_TINYCONN_CH1_MUX' DL1_CH1   (etc.)
 */
static bool mt6785_mt6359_have_speaker;

static int mt6785_mt6359_late_probe(struct snd_soc_card *card)
{
	static const char * const names[] = {
		"I2S1_TINYCONN_CH1_MUX", "I2S1_TINYCONN_CH2_MUX",
		"I2S3_TINYCONN_CH1_MUX", "I2S3_TINYCONN_CH2_MUX",
	};
	struct snd_soc_component *afe_cmpnt = NULL;
	struct snd_soc_pcm_runtime *rtd;
	int i;

	for_each_card_rtds(card, rtd) {
		afe_cmpnt = snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
		if (afe_cmpnt)
			break;
	}
	if (afe_cmpnt)
		pm_runtime_get_sync(afe_cmpnt->dev);

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		struct snd_kcontrol *kctl = snd_soc_card_get_kcontrol(card, names[i]);
		struct snd_ctl_elem_value uc = {};

		if (!kctl)
			continue;
		uc.value.enumerated.item[0] = 0;	/* NONE */
		snd_soc_dapm_put_enum_double(kctl, &uc);
	}

	if (afe_cmpnt)
		pm_runtime_put(afe_cmpnt->dev);

	/*
	 * The "Speaker" pin is OFF by default. While it is connected, DPCM attaches the speaker
	 * BE to any DL1 stream (the DAPM mixer path state at card
	 * registration is stale: register reads bypass the cache), i.e. sound to the
	 * headphones would turn on the loud speaker. The speaker is turned on only by the
	 * PulseAudio path analog-output-speaker / spk-on (Speaker Switch + I2S3 route).
	 */
	if (mt6785_mt6359_have_speaker) {
		snd_soc_dapm_disable_pin(card->dapm, "Speaker");
		snd_soc_dapm_sync(card->dapm);
	}
	return 0;
}

static int mt6785_mt6359_soc_card_probe(struct mtk_soc_card_data *soc_card_data,
					bool legacy)
{
	struct snd_soc_card *card = soc_card_data->card_data->card;
	struct device_node *np;
	struct i2c_client *spk = NULL;
	int ret;

	card->late_probe = mt6785_mt6359_late_probe;

	ret = mt6785_afe_gpio_init(card->dev);
	if (ret)
		return dev_err_probe(card->dev, ret, "%s init gpio error\n",
				     __func__);

	/* there is a speaker only if tfa9874 is in DT and its driver is already bound */
	np = of_find_compatible_node(NULL, NULL, "nxp,tfa9874");
	if (np)
		spk = of_find_i2c_device_by_node(np);
	if (spk && spk->dev.driver) {
		/* the i2c bus number floats (no aliases) - bind by DT node */
		struct snd_soc_dai_link_component *c =
			mt6785_mt6359_dai_links[card->num_links - 1].codecs;

		c->name = NULL;
		c->of_node = np;	/* keep the reference for the whole lifetime of the card */
		np = NULL;
		card->dapm_widgets = mt6785_mt6359_widgets_spk;
		card->num_dapm_widgets = ARRAY_SIZE(mt6785_mt6359_widgets_spk);
		card->dapm_routes = mt6785_mt6359_routes_spk;
		card->num_dapm_routes = ARRAY_SIZE(mt6785_mt6359_routes_spk);
		card->controls = mt6785_mt6359_controls_spk;
		card->num_controls = ARRAY_SIZE(mt6785_mt6359_controls_spk);
		mt6785_mt6359_have_speaker = true;
		dev_info(card->dev, "speaker: tfa9874 found, Speaker link enabled\n");
	} else {
		card->num_links--;	/* cut off the Speaker link */
		dev_info(card->dev, "speaker: tfa9874 not found/not bound, no speaker\n");
	}
	if (spk)
		put_device(&spk->dev);
	of_node_put(np);	/* NULL-safe; non-NULL only if there is no speaker */

	return 0;
}

static const unsigned int mt6785_pcm_playback_channels[] = { 1, 2 };
static const unsigned int mt6785_pcm_playback_rates[] = { 48000 };

static const unsigned int mt6785_pcm_capture_channels[] = { 1, 2, 4 };
static const unsigned int mt6785_pcm_capture_rates[] = {
	8000, 16000, 32000, 48000, 96000, 192000
};

static const struct mtk_pcm_constraints_data mt6785_pcm_constraints[MTK_CONSTRAINT_CAPTURE + 1] = {
	[MTK_CONSTRAINT_PLAYBACK] = {
		.channels =  &(const struct snd_pcm_hw_constraint_list) {
			.list = mt6785_pcm_playback_channels,
			.count = ARRAY_SIZE(mt6785_pcm_playback_channels)
		},
		.rates =  &(const struct snd_pcm_hw_constraint_list) {
			.list = mt6785_pcm_playback_rates,
			.count = ARRAY_SIZE(mt6785_pcm_playback_rates)
		}
	},
	[MTK_CONSTRAINT_CAPTURE] = {
		.channels =  &(const struct snd_pcm_hw_constraint_list) {
			.list = mt6785_pcm_capture_channels,
			.count = ARRAY_SIZE(mt6785_pcm_capture_channels)
		},
		.rates =  &(const struct snd_pcm_hw_constraint_list) {
			.list = mt6785_pcm_capture_rates,
			.count = ARRAY_SIZE(mt6785_pcm_capture_rates)
		}
	}
};

static const struct mtk_soundcard_pdata mt6785_mt6359_pdata = {
	.card_name = MT6785_MT6359_CARD_NAME,
	.card_data = &(struct mtk_platform_card_data) {
		.card = &mt6785_mt6359_card,
		.num_jacks = 1,
		.pcm_constraints = mt6785_pcm_constraints,
		.num_pcm_constraints = ARRAY_SIZE(mt6785_pcm_constraints),
	},
	.soc_probe = mt6785_mt6359_soc_card_probe
};

static const struct of_device_id mt6785_mt6359_dt_match[] = {
	{ .compatible = MT6785_MT6359_OF_NAME, .data = &mt6785_mt6359_pdata },
	{}
};
MODULE_DEVICE_TABLE(of, mt6785_mt6359_dt_match);

static const struct dev_pm_ops mt6785_mt6359_pm_ops = {
	.poweroff = snd_soc_poweroff,
	.restore = snd_soc_resume,
};

static struct platform_driver mt6785_mt6359_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = mt6785_mt6359_dt_match,
		.pm = &mt6785_mt6359_pm_ops,
	},
	.probe = mtk_soundcard_common_probe,
};
module_platform_driver(mt6785_mt6359_driver);

MODULE_DESCRIPTION("MT6785-MT6359 ALSA SoC machine driver (begonia)");
MODULE_AUTHOR("Jiaxin Yu <jiaxin.yu@mediatek.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("mt6785_mt6359 soc card");
