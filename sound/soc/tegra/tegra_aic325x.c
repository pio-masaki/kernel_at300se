/*
 * tegra_aic325x.c - Tegra machine ASoC driver for boards using TI 3262 codec.
 *
 * Author: Vinod G. <vinodg@nvidia.com>
 * Copyright (C) 2011 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 *
 * (c) 2010, 2011 Nvidia Graphics Pvt. Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <asm/mach-types.h>

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif
#include <linux/input.h>
#include <linux/jiffies.h>

#include <mach/tegra_aic325x_pdata.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#ifdef CONFIG_SND_SOC_FM34
#include <sound/fm34.h>
#endif
#include "../codecs/tlv320aic325x.h"

#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#include "tegra20_das.h"
#else
#include "tegra30_ahub.h"
#include "tegra30_i2s.h"
#include "tegra30_dam.h"
#endif
#include "tegra_aic325x.h"


#define DRV_NAME "tegra-snd-aic325x"

#define GPIO_SPKR_EN    BIT(0)
#define GPIO_HP_MUTE    BIT(1)
#define GPIO_INT_MIC_EN BIT(2)
#define GPIO_EXT_MIC_EN BIT(3)
#define GPIO_HP_DET     BIT(4)
#define GPIO_EXT_MIC_DET     BIT(5)

#define DAI_LINK_HIFI		0
#define DAI_LINK_SPDIF		1
#define DAI_LINK_BTSCO		2
#define DAI_LINK_VOICE_CALL	3
#define DAI_LINK_BT_VOICE_CALL	4
#define NUM_DAI_LINKS	5

#define AEC_DISABLE 0
#define AEC_ENABLE 1

enum audio_source {
	AUDIO_SOURCE_DEFAULT = 0,
	AUDIO_SOURCE_MIC = 1,
	AUDIO_SOURCE_VOICE_UPLINK = 2,
	AUDIO_SOURCE_VOICE_DOWNLINK = 3,
	AUDIO_SOURCE_VOICE_CALL = 4,
	AUDIO_SOURCE_CAMCORDER = 5,
	AUDIO_SOURCE_VOICE_RECOGNITION = 6,
	AUDIO_SOURCE_VOICE_COMMUNICATION = 7,
	AUDIO_SOURCE_MAX = AUDIO_SOURCE_VOICE_COMMUNICATION,

	AUDIO_SOURCE_LIST_END  // must be last - used to validate audio source type
};

enum audio_devices {
	// output devices
	DEVICE_OUT_EARPIECE = 0x1,
	DEVICE_OUT_SPEAKER = 0x2,
	DEVICE_OUT_WIRED_HEADSET = 0x4,
	DEVICE_OUT_WIRED_HEADPHONE = 0x8,
	DEVICE_OUT_DEFAULT = 0x8000,
	// input devices
	DEVICE_IN_COMMUNICATION = 0x10000,
	DEVICE_IN_BUILTIN_MIC = 0x40000,
	DEVICE_IN_WIRED_HEADSET = 0x100000,
	DEVICE_IN_VOICE_CALL = 0x400000,
	DEVICE_IN_DEFAULT = 0x80000000,
};

enum audio_mode {
	MODE_INVALID = -2,
	MODE_CURRENT = -1,
	MODE_NORMAL = 0,
	MODE_RINGTONE,
	MODE_IN_CALL,
	MODE_IN_COMMUNICATION,
	NUM_MODES  // not a valid entry, denotes end-of-list
};

/* These values are copied from WiredAccessoryObserver */
enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};


#ifndef CONFIG_ARCH_TEGRA_2x_SOC
const char *tegra_aic325x_i2s_dai_name[TEGRA30_NR_I2S_IFC] = {
	"tegra30-i2s.0",
	"tegra30-i2s.1",
	"tegra30-i2s.2",
	"tegra30-i2s.3",
	"tegra30-i2s.4",
};
#endif

struct tegra_aic325x {
	struct tegra_asoc_utils_data util_data;
	struct tegra_aic325x_platform_data *pdata;
	struct regulator *audio_reg;
	struct regulator *hp_amp_reg;
	struct regulator *spk_reg;
	struct regulator *dmic_reg;
	struct regulator *vdd_5v0_sys;
	int gpio_requested;
	bool init_done;
	int is_call_mode;
	int is_device_bt;
	int input_source;
	int io_device;
	int audio_mode;
	bool is_recording;
	int headset_state;
	bool voice_call;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct codec_config codec_info[NUM_I2S_DEVICES];
#endif
#ifdef CONFIG_SND_SOC_FM34
	struct snd_pcm_substream *cap_substream;
	struct mutex fm34_lock;
#endif
	struct snd_soc_codec *codec;
	int route;
};

extern int fm34_enable_get(void);

int fm34_force_policy = 0;
module_param(fm34_force_policy, int, 0644);
MODULE_PARM_DESC(fm34_force_policy, "forced enable/disable fm34");


unsigned int voip_speaker_gain = 0x3;
module_param(voip_speaker_gain, int, 0644);
MODULE_PARM_DESC(voip_speaker_gain, "voip speaker gain");

int hp_auto_reset = 1;
module_param(hp_auto_reset, int, 0644);
MODULE_PARM_DESC(hp_auto_reset, "hp_auto_reset");

int startup_silence_ms = 18;
module_param(startup_silence_ms, int, 0644);
MODULE_PARM_DESC(startup_silence_ms, "startup_silence_ms in ms");

unsigned int fm34_profile_skype = 0;
module_param(fm34_profile_skype, int, 0644);
MODULE_PARM_DESC(fm34_profile_skype, "fm34_profile_skype");

unsigned int fm34_profile_filter = 0;
module_param(fm34_profile_filter, int, 0644);
MODULE_PARM_DESC(fm34_profile_filter, "fm34_profile_filter by input source");

int fm34_profile_mode = NOISE_SUPPRESSION;
module_param(fm34_profile_mode, int, 0644);
MODULE_PARM_DESC(fm34_profile_mode, "fm34_profile_mode");

int fm34_ext_mic_fltr = 0;
module_param(fm34_ext_mic_fltr, int, 0644);
MODULE_PARM_DESC(fm34_ext_mic_fltr, "fm34_ext_mic_fltr");

static unsigned int codec_shutdown = 1;
static unsigned long codec_startup_time = 99;

extern int in_rate;

static int tegra_aic325x_get_mclk(int srate)
{
	int mclk = 0;
	switch (srate) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		mclk = -EINVAL;
		break;
	}

	return mclk;
}

#ifdef CONFIG_SND_SOC_FM34
void tegra_aic325x_update_gain(void *vcodec, int profile_mode)
{
	struct snd_soc_codec *codec = (struct snd_soc_codec *)vcodec;
	struct snd_soc_card *card = codec->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	
	if ( profile_mode ) {
		if ( profile_mode == AEC_ENABLE+ECHO_CANCELLATION ) {
			if( machine->io_device == DEVICE_OUT_SPEAKER ) {
				/* increase +2db line out internal speaker when voip */
				snd_soc_update_bits(codec, LOL_GAIN, 0x3f, voip_speaker_gain);	/* 3dB */
				snd_soc_update_bits(codec, LOR_GAIN, 0x3f, voip_speaker_gain);
			}
		}
		/* decrease ADC due to fm34 will increase 18db + 6db */
		snd_soc_update_bits(codec, LADC_VOL, 0x7f, 0x00);	/* 0dB */
		snd_soc_update_bits(codec, RADC_VOL, 0x7f, 0x00);

	}
	else {
		snd_soc_update_bits(codec, LOL_GAIN, 0x3f, 0x01);	/* 1dB */
		snd_soc_update_bits(codec, LOR_GAIN, 0x3f, 0x01);

		snd_soc_update_bits(codec, LADC_VOL, 0x7f, 0x28);	/* 20dB */
		snd_soc_update_bits(codec, RADC_VOL, 0x7f, 0x28);
	}

}

void tegra_pcm_control(int int_mic, int enable, struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	int aec_st = AEC_DISABLE;

	mutex_lock(&machine->fm34_lock);
	if( int_mic == 0 ) {
		if ( enable ) {

			if(set_fM34_echo(fm34_profile_mode)) {
				aec_st = AEC_ENABLE+fm34_profile_mode;	
			}
		}
	}
	else if(enable && fm34_enable_get() ) {
		
		if ( fm34_force_policy == 1 ) {
			if(set_fM34_echo(fm34_profile_mode))
				aec_st = AEC_ENABLE+fm34_profile_mode;
		}
		else if ( fm34_force_policy == 2 ) {
			set_fM34_bypass(0);
		}
		else {

			printk(KERN_DEBUG "### input:%d mode:%d voice_call:%d\n",
					machine->input_source, machine->audio_mode, machine->voice_call);

			/* generic, skype voip */
			if ( (machine->input_source == AUDIO_SOURCE_VOICE_COMMUNICATION
						|| machine->audio_mode == MODE_IN_COMMUNICATION
						|| (fm34_profile_skype && machine->voice_call) ) ) {
					if(set_fM34_echo(ECHO_CANCELLATION)) {
						aec_st = AEC_ENABLE+ECHO_CANCELLATION;
					}
			}
			else if ( machine->input_source == AUDIO_SOURCE_CAMCORDER && 
				(fm34_profile_filter & (1<<machine->input_source))) {
				if(set_fM34_echo(CAMCORDING)) {
					aec_st = AEC_ENABLE+CAMCORDING;  
				}
			}
			else if ( machine->input_source > 0 && (fm34_profile_filter & (1<<machine->input_source))){

				if(set_fM34_echo(fm34_profile_mode)) {
					aec_st = AEC_ENABLE+fm34_profile_mode;  
				}
			}
		}		
	}

	if ( aec_st == AEC_DISABLE ) {
		set_fM34_bypass(0);
	}

	tegra_aic325x_update_gain((void*)codec, aec_st);

	mutex_unlock(&machine->fm34_lock);

}
#else
void tegra_pcm_control(int int_mic, int enable, struct snd_pcm_substream *substream) {}
#endif

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static int tegra_aic325x_set_dam_cif(int dam_ifc, int srate,
			int channels, int bit_size, int src_on, int src_srate,
			int src_channels, int src_bit_size)
{
	tegra30_dam_set_gain(dam_ifc, TEGRA30_DAM_CHIN1, 0x1000);
	tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHOUT,
				srate);
	tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHIN1,
				srate);
	tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHIN1,
		channels, bit_size, channels,
				bit_size);
	tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHOUT,
		channels, bit_size, channels,
				bit_size);

	if (src_on) {
		tegra30_dam_set_gain(dam_ifc, TEGRA30_DAM_CHIN0_SRC, 0x1000);
		tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHIN0_SRC,
			src_srate);
		tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHIN0_SRC,
			src_channels, src_bit_size, 1, 16);
	}

	return 0;
}
#endif

static int tegra_aic325x_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);
#endif
	int srate, mclk, sample_size, daifmt;
	int err;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sample_size = 16;
		break;
	default:
		return -EINVAL;
	}

	srate = params_rate(params);

	mclk = tegra_aic325x_get_mclk(srate);
	if (mclk < 0) {
		printk(" tegra_aic325x_get_mclk <0 \n");
		return mclk;
	}
	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	daifmt = SND_SOC_DAIFMT_I2S |	SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS;

	err = snd_soc_dai_set_fmt(codec_dai, daifmt);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, daifmt);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
					SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC1,
					TEGRA20_DAS_DAP_ID_1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dap-dac path\n");
		return err;
	}

	err = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_1,
					TEGRA20_DAS_DAP_SEL_DAC1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}
#else /*assumes tegra3*/
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK  && i2s->is_dam_used)
		tegra_aic325x_set_dam_cif(i2s->dam_ifc, srate,
			params_channels(params), sample_size, 0, 0, 0, 0);
#endif

#ifdef CONFIG_SND_SOC_FM34
	fm34_set_codec_link(codec, tegra_aic325x_update_gain );
	if(substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		machine->cap_substream = substream;
#endif

	return 0;
}

static int tegra_aic325x_spdif_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, min_mclk;
	int err;

	srate = params_rate(params);
	mclk = tegra_aic325x_get_mclk(srate);
	if (mclk < 0)
		return mclk;

	min_mclk = 128 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	return 0;
}

static int tegra_aic325x_bt_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
//	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
#endif
	struct snd_soc_card *card = rtd->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	int err, srate, mclk, min_mclk, sample_size;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sample_size = 16;
		break;
	default:
		return -EINVAL;
	}

	srate = params_rate(params);

	mclk = tegra_aic325x_get_mclk(srate);
	if (mclk < 0)
		return mclk;

	min_mclk = 64 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	err = snd_soc_dai_set_fmt(rtd->cpu_dai,
			SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		dev_err(rtd->codec->card->dev, "cpu_dai fmt not set\n");
		return err;
	}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	//if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	//	tegra_aic325x_set_dam_cif(i2s->dam_ifc, params_rate(params),
	//			params_channels(params), sample_size);
#endif

	return 0;
}

static int tegra_aic325x_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(rtd->card);

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 0);

	return 0;
}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static int tegra_aic325x_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);
	
	if( substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		codec_shutdown--;
		if( codec_shutdown == 0 )
			codec_startup_time = jiffies;
	}

	if ((substream->stream != SNDRV_PCM_STREAM_PLAYBACK) ||
		!(i2s->is_dam_used))
		return 0;

	/*dam configuration*/
	if (!i2s->dam_ch_refcount)
		i2s->dam_ifc = tegra30_dam_allocate_controller();

	tegra30_dam_allocate_channel(i2s->dam_ifc, TEGRA30_DAM_CHIN1);
	i2s->dam_ch_refcount++;
	tegra30_dam_enable_clock(i2s->dam_ifc);
	tegra30_dam_set_gain(i2s->dam_ifc, TEGRA30_DAM_CHIN1, 0x1000);

	tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
			(i2s->dam_ifc*2), i2s->txcif);

	/*
	*make the dam tx to i2s rx connection if this is the only client
	*using i2s for playback
	*/
	if (i2s->playback_ref_count == 1)
		tegra30_ahub_set_rx_cif_source(
			TEGRA30_AHUB_RXCIF_I2S0_RX0 + i2s->id,
			TEGRA30_AHUB_TXCIF_DAM0_TX0 + i2s->dam_ifc);

	/* enable the dam*/
	tegra30_dam_enable(i2s->dam_ifc, TEGRA30_DAM_ENABLE,
			TEGRA30_DAM_CHIN1);

	return 0;
}

static void tegra_aic325x_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);

	if( substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		codec_shutdown++;
	}
	if ((substream->stream != SNDRV_PCM_STREAM_PLAYBACK) ||
		!(i2s->is_dam_used))
		return;

	/* disable the dam*/
	tegra30_dam_enable(i2s->dam_ifc, TEGRA30_DAM_DISABLE,
			TEGRA30_DAM_CHIN1);

	/* disconnect the ahub connections*/
	tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
				(i2s->dam_ifc*2));

	/* disable the dam and free the controller */
	tegra30_dam_disable_clock(i2s->dam_ifc);
	tegra30_dam_free_channel(i2s->dam_ifc, TEGRA30_DAM_CHIN1);
	i2s->dam_ch_refcount--;
	if (!i2s->dam_ch_refcount)
		tegra30_dam_free_controller(i2s->dam_ifc);
}
#endif

static int tegra_aic325x_voice_call_hw_params(
			struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk;
	int err;

	srate = params_rate(params);
	mclk = tegra_aic325x_get_mclk(srate);
	if (mclk < 0)
		return mclk;


	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	err = snd_soc_dai_set_fmt(codec_dai,
					SND_SOC_DAIFMT_DSP_B |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
					SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* codec configuration */
	machine->codec_info[HIFI_CODEC].rate = params_rate(params);
	machine->codec_info[HIFI_CODEC].channels = params_channels(params);
	machine->codec_info[HIFI_CODEC].bitsize = 16;
	machine->codec_info[HIFI_CODEC].is_i2smaster = 1;
	machine->codec_info[HIFI_CODEC].is_format_dsp = 0;

	/* baseband configuration */
	machine->codec_info[BASEBAND].bitsize = 16;
	machine->codec_info[BASEBAND].is_i2smaster = 1;
	machine->codec_info[BASEBAND].is_format_dsp = 1;
#endif

	machine->is_device_bt = 0;

	return 0;
}

static void tegra_aic325x_voice_call_shutdown(
					struct snd_pcm_substream *substream)
{
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_aic325x *machine  =
			snd_soc_card_get_drvdata(rtd->codec->card);

	machine->codec_info[HIFI_CODEC].rate = 0;
	machine->codec_info[HIFI_CODEC].channels = 0;
#endif
}

static int tegra_aic325x_bt_voice_call_hw_params(
			struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	int err, srate, mclk, min_mclk;

	srate = params_rate(params);

	mclk = tegra_aic325x_get_mclk(srate);
	if (mclk < 0)
		return mclk;

	min_mclk = 64 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* codec configuration */
	machine->codec_info[BT_SCO].rate = params_rate(params);
	machine->codec_info[BT_SCO].channels = params_channels(params);
	machine->codec_info[BT_SCO].bitsize = 16;
	machine->codec_info[BT_SCO].is_i2smaster = 1;
	machine->codec_info[BT_SCO].is_format_dsp = 1;

	/* baseband configuration */
	machine->codec_info[BASEBAND].bitsize = 16;
	machine->codec_info[BASEBAND].is_i2smaster = 1;
	machine->codec_info[BASEBAND].is_format_dsp = 1;
#endif

	machine->is_device_bt = 1;

	return 0;
}

static void tegra_aic325x_bt_voice_call_shutdown(
				struct snd_pcm_substream *substream)
{
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_aic325x *machine  =
			snd_soc_card_get_drvdata(rtd->codec->card);

	machine->codec_info[BT_SCO].rate = 0;
	machine->codec_info[BT_SCO].channels = 0;
#endif
}

int pcm_startup_silence_update(void)
{
	if( startup_silence_ms && codec_shutdown==0 && codec_startup_time ) {
		codec_startup_time = 0;
		return startup_silence_ms;
	}
	return 0;
}

static struct snd_soc_ops tegra_aic325x_hifi_ops = {
	.hw_params = tegra_aic325x_hw_params,
	.hw_free = tegra_aic325x_hw_free,
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	.startup = tegra_aic325x_startup,
	.shutdown = tegra_aic325x_shutdown,
#endif
};

static struct snd_soc_ops tegra_aic325x_spdif_ops = {
	.hw_params = tegra_aic325x_spdif_hw_params,
	.hw_free = tegra_aic325x_hw_free,
};

#if 0
static struct snd_soc_ops tegra_aic325x_voice_call_ops = {
	.hw_params = tegra_aic325x_voice_call_hw_params,
	.shutdown = tegra_aic325x_voice_call_shutdown,
	.hw_free = tegra_aic325x_hw_free,
};

static struct snd_soc_ops tegra_aic325x_bt_voice_call_ops = {
	.hw_params = tegra_aic325x_bt_voice_call_hw_params,
	.shutdown = tegra_aic325x_bt_voice_call_shutdown,
	.hw_free = tegra_aic325x_hw_free,
};
#endif

static struct snd_soc_ops tegra_aic325x_bt_ops = {
	.hw_params = tegra_aic325x_bt_hw_params,
	.hw_free = tegra_aic325x_hw_free,
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	.startup = tegra_aic325x_startup,
	.shutdown = tegra_aic325x_shutdown,
#endif
};

static struct snd_soc_jack tegra_aic325x_hp_jack;

static struct snd_soc_jack_gpio tegra_aic325x_hp_jack_gpio = {
	.name = "headphone detect",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
	.invert = 1,
};

static struct snd_soc_jack tegra_aic325x_hook_key;

static struct snd_soc_jack_gpio tegra_aic325x_hook_key_gpio = {
	.name = "hook key detect",
	.report = SND_JACK_MECHANICAL,
	.debounce_time = 300,
	.invert = 1,
};

#ifdef CONFIG_SWITCH
static struct switch_dev aic325x_wired_switch_dev = {
	.name = "h2w",
};

static int aic325x_hook_key_notify(struct notifier_block *self,
	unsigned long action, void *dev)
{
	struct snd_soc_jack *jack = dev;
	struct input_dev *jack_input_dev = jack->jack->input_dev;

	printk(KERN_DEBUG "### %s, action:%lu\n", __func__, action);
	if(action == 0) {
		input_event(jack_input_dev, EV_KEY, KEY_MEDIA, 1);
		input_sync(jack_input_dev);
	} else if(action == 8) {
		input_event(jack_input_dev, EV_KEY, KEY_MEDIA, 0);
		input_sync(jack_input_dev);
	}

	return NOTIFY_OK;
}

static struct notifier_block aic325x_hook_key_nb = {
	.notifier_call = aic325x_hook_key_notify,
};

static int aic325x_headset_switch_notify(struct notifier_block *self,
	unsigned long action, void *dev)
{
	struct snd_soc_jack *jack = dev;
	struct snd_soc_codec *codec = jack->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	struct tegra_aic325x_platform_data *pdata = machine->pdata;

	if(jack == &tegra_aic325x_hp_jack) {
		if(action & SND_JACK_HEADPHONE) {

			msleep(350); //reduce the "bobo" noise during inserting

			if(!machine->is_recording) {
				/* enable gpio ext_mic_en for detecting iphone microphone */
				gpio_set_value(pdata->gpio_ext_mic_en, 0);
				/* enable internal micbias for detect external microphone */
				//snd_soc_update_bits(codec, MICBIAS_CTRL, 0xff, 0x78);
			}
			msleep(50);

			/* check ext_mic_det gpio if ther is microphone exist */
			if(!gpio_get_value(pdata->gpio_ext_mic_det)) {
				action |= SND_JACK_MICROPHONE;
				printk("### headset detected!!\n");
			}
			else {
				/* disable external micbias because there is no microphone inserted */
				gpio_set_value(pdata->gpio_ext_mic_en, 1);
				/* disable internal micbias because there is no microphone inserted */
				//snd_soc_update_bits(codec, MICBIAS_CTRL, 0x40, 0x00);
				printk("### headphone detected!!\n");
			}
		} 
		else{
			gpio_set_value(pdata->gpio_ext_mic_en, 1);
			//snd_soc_update_bits(codec, MICBIAS_CTRL, 0x40, 0x00);
		}

		gpio_set_value(pdata->gpio_ext_mic_en, 1); //reduce the "bobo" noise during removing
	}

	switch (action) {
	case SND_JACK_HEADPHONE:
		machine->headset_state |= BIT_HEADSET_NO_MIC;
		break;
	case SND_JACK_HEADSET:
		machine->headset_state |= BIT_HEADSET;
		//snd_soc_jack_notifier_register(&tegra_aic325x_hook_key,
		//							   &aic325x_hook_key_nb);
		break;
	default:
		machine->headset_state = BIT_NO_HEADSET;
		//snd_soc_jack_notifier_unregister(&tegra_aic325x_hook_key,
		//							   &aic325x_hook_key_nb);
	}

	printk("### headset_state=%d\n", machine->headset_state);
	switch_set_state(&aic325x_wired_switch_dev, machine->headset_state);

	return NOTIFY_OK;
}

int tegra_aic325x_get_headset_state(struct snd_soc_codec *codec)
{
	struct snd_soc_card *card = codec->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);

	return machine->headset_state;
}
EXPORT_SYMBOL(tegra_aic325x_get_headset_state);

static struct notifier_block aic325x_headset_switch_nb = {
	.notifier_call = aic325x_headset_switch_notify,
};
#else
static struct snd_soc_jack_pin tegra_aic325x_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static struct snd_soc_jack_pin tegra_aic325x_hook_key_pins[] = {
	{
		.pin = "Hook Key",
		.mask = SND_JACK_HEADSET,
	},
};
#endif

void aic325x_jack_resume(void)
{
	int val = 0;

	val = gpio_get_value(tegra_aic325x_hp_jack_gpio.gpio);
	val = tegra_aic325x_hp_jack_gpio.invert ? !val : val;
	val = val ? tegra_aic325x_hp_jack_gpio.report : 0;
	snd_soc_jack_report(&tegra_aic325x_hp_jack, val,
						tegra_aic325x_hp_jack_gpio.report);
}

static int tegra_aic325x_event_int_spk(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	struct tegra_aic325x_platform_data *pdata = machine->pdata;

	printk(KERN_DEBUG "### %s : %s\n",__func__,
		   SND_SOC_DAPM_EVENT_ON(event)?"ENABLE":"DISABLE");

	if (machine->spk_reg) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			regulator_enable(machine->spk_reg);
		else
			regulator_disable(machine->spk_reg);
	}

	if (!(machine->gpio_requested & GPIO_SPKR_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_spkr_en,
				SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_aic325x_event_hp(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	struct tegra_aic325x_platform_data *pdata = machine->pdata;
	unsigned int val;
	
	if ( event == SND_SOC_DAPM_POST_PMU) {

		printk(KERN_DEBUG "### %s : %s\n",__func__, "ENABLE");

		if( hp_auto_reset && (machine->headset_state) ) {
			val = snd_soc_read(machine->codec, PWR_CTRL_REG);
			if( ! (val & 0x04) ) {
				snd_soc_update_bits(machine->codec, OUT_PWR_CTRL, 0x30, 0x00);
				msleep(10);
				snd_soc_update_bits(machine->codec, OUT_PWR_CTRL, 0x30, 0x30);
				printk(KERN_INFO "%s: reset hp power\n", __func__ );
			}
		}

		if ( machine->hp_amp_reg ) {
			regulator_enable( machine->hp_amp_reg );
		}
	}
	else if ( event == SND_SOC_DAPM_POST_PMD) {
		printk(KERN_DEBUG "### %s : %s\n",__func__, "DISABLE");
	}
	else if ( event == SND_SOC_DAPM_PRE_PMD) {
		regulator_disable( machine->hp_amp_reg );
	}
	
	if (!(machine->gpio_requested & GPIO_HP_MUTE))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_hp_mute,
				!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_aic325x_event_int_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);

	printk(KERN_DEBUG "### %s : %s, is_recording:%d, headset_state:%d\n",__func__,
		   SND_SOC_DAPM_EVENT_ON(event)?"ENABLE":"DISABLE", machine->is_recording, machine->headset_state);


	machine->is_recording = SND_SOC_DAPM_EVENT_ON(event)?true:false;

#ifdef CONFIG_SND_SOC_FM34
	tegra_pcm_control(1, SND_SOC_DAPM_EVENT_ON(event), machine->cap_substream);
#endif

	return 0;
}

static int tegra_aic325x_event_ext_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	struct tegra_aic325x_platform_data *pdata = machine->pdata;

	printk(KERN_DEBUG "### %s : %s\n",__func__,
		   SND_SOC_DAPM_EVENT_ON(event)?"ENABLE":"DISABLE");

#ifdef CONFIG_SND_SOC_FM34
	if( fm34_ext_mic_fltr && in_rate && in_rate <= 16000 ) { /* apply if sample rate < 16K */
		tegra_pcm_control(0, SND_SOC_DAPM_EVENT_ON(event), machine->cap_substream);
	}
#endif

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		/* currently you are not recording, and just want to start recording and there is headset plugged, 
		    we should force enable external microphone power */
		if(!machine->is_recording && (machine->headset_state == BIT_HEADSET)) {
			//printk(KERN_DEBUG "### %s : ext_mic_en enable\n",__func__);
			gpio_set_value(pdata->gpio_ext_mic_en, 0);//enable external mic bias power
			//snd_soc_update_bits(codec, MICBIAS_CTRL, 0xff, 0x78);//enable internal mic bais power
		}
	} else {
		/* currently you are recording, and just want to stop recording and there is headset plugged,
		    we should forced disable external microphone power for saving battery */
		if(machine->headset_state == BIT_HEADSET) {
			//printk(KERN_DEBUG "### %s : ext_mic_en disable\n",__func__);
			gpio_set_value(pdata->gpio_ext_mic_en, 1);//disable external mic bias power
			//snd_soc_update_bits(codec, MICBIAS_CTRL, 0x40, 0x0);//enable internal mic bais power
		}
	}

	machine->is_recording = SND_SOC_DAPM_EVENT_ON(event)?true:false;
	return 0;
}

static const struct snd_soc_dapm_widget tegra_aic325x_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("SPK out", tegra_aic325x_event_int_spk),
	SND_SOC_DAPM_HP("Headphone jack", tegra_aic325x_event_hp),
	SND_SOC_DAPM_MIC("Int Mic", tegra_aic325x_event_int_mic),
	SND_SOC_DAPM_MIC("Ext Mic", tegra_aic325x_event_ext_mic),
};

static const struct snd_soc_dapm_route aic325x_audio_map[] = {

	/* Headphone connected to HPL, HPR */
	{"Headphone jack", NULL, "HPL"},
	{"Headphone jack", NULL, "HPR"},

	{"SPK out", NULL, "LOL"},
	{"SPK out", NULL, "LOR"},

	{"IN1_L", NULL, "Int Mic"},
	{"IN1_R", NULL, "Int Mic"},

	{"Left DMIC", NULL, "Int Mic"},
	{"Right DMIC", NULL, "Int Mic"},

	{"IN3_L", NULL, "Ext Mic"},
	{"IN3_R", NULL, "Ext Mic"},


};

static const struct snd_kcontrol_new tegra_aic325x_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Ext Mic"),
};

static int tegra_aic325x_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	struct tegra_aic325x_platform_data *pdata = machine->pdata;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
#endif
	int ret = -1;

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	if (machine->codec_info[BASEBAND].i2s_id != -1)
		i2s->is_dam_used = true;
#endif

	if (machine->init_done)
		return 0;

	machine->init_done = true;
	machine->codec = codec;
#if 0
	if (machine_is_whistler()) {
		machine->audio_reg = regulator_get(NULL, "avddio_audio");
		if (IS_ERR(machine->audio_reg)) {
			dev_err(card->dev, "cannot get avddio_audio reg\n");
			ret = PTR_ERR(machine->audio_reg);
			return ret;
		}

		ret = regulator_enable(machine->audio_reg);
		if (ret) {
			dev_err(card->dev, "cannot enable avddio_audio reg\n");
			regulator_put(machine->audio_reg);
			machine->audio_reg = NULL;
			return ret;
		}
	}else{
		dev_info(card->dev, "machine is no whistler\n");
	}
#endif

	//queue this 5v0 power block, that is tell 5v0 audio is using 5v0
	machine->vdd_5v0_sys = regulator_get(NULL, "vdd_5v0_sys");
	if (IS_ERR(machine->vdd_5v0_sys)) {
		dev_info(card->dev, "No vdd_5v0_sys regulator foun\n");
		machine->vdd_5v0_sys = NULL;
	}
	regulator_enable(machine->vdd_5v0_sys);

	/* AMP power switch */
	machine->spk_reg = regulator_get(card->dev, "vdd_spk_amp");
	if (IS_ERR(machine->spk_reg)) {
		dev_info(card->dev, "No speaker regulator found\n");
		machine->spk_reg = 0;
	}else{
		dev_info(card->dev, "speaker regulator found\n");
	}

	/* HP AMP power switch */
	machine->hp_amp_reg = regulator_get(card->dev, "vdd_hp_amp");
	if (IS_ERR(machine->hp_amp_reg)) {
		dev_info(card->dev, "No HP amp regulator found\n");
		machine->hp_amp_reg = 0;
	}else{
		dev_info(card->dev, "HP amp regulator found\n");
	}

	/* onboard mic power switch */
	machine->dmic_reg = regulator_get(card->dev, "vdd_dmic");
	if (IS_ERR(machine->dmic_reg)) {
		dev_info(card->dev, "No digital mic regulator found\n");
		machine->dmic_reg = 0;
	}else{
		dev_info(card->dev, "digital mic regulator found, enable dmic power\n");
		/* dmic power always on, it doesn't need to dynamic turn on/off(GPIO high/low) dmic power to save battery,
		   you just need to start/stop dmic clock */
		regulator_enable(machine->dmic_reg);
	}

	//currently is invalid, -1
	if (gpio_is_valid(pdata->gpio_spkr_en)) {
		ret = gpio_request(pdata->gpio_spkr_en, "spkr_en");
		if (ret) {
			dev_err(card->dev, "cannot get spkr_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_SPKR_EN;

		gpio_direction_output(pdata->gpio_spkr_en, 0);
		gpio_export(pdata->gpio_spkr_en, false);
	}else{
		dev_info(card->dev, "gpio_spkr_en is not valid\n");
	}

	//currently is invalid, -1
	if (gpio_is_valid(pdata->gpio_hp_mute)) {
		ret = gpio_request(pdata->gpio_hp_mute, "hp_mute");
		if (ret) {
			dev_err(card->dev, "cannot get hp_mute gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_HP_MUTE;

		gpio_direction_output(pdata->gpio_hp_mute, 0);
	}else{
		dev_info(card->dev, "gpio_hp_mute is not valid\n");
	}

	if (gpio_is_valid(pdata->gpio_int_mic_en)) {
		ret = gpio_request(pdata->gpio_int_mic_en, "int_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get int_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_INT_MIC_EN;

		/* Disable int mic; enable signal is active-high */
		gpio_direction_output(pdata->gpio_int_mic_en, 0);
	}else{
		dev_info(card->dev, "gpio_int_mic_en is not valid\n");
	}

	if (gpio_is_valid(pdata->gpio_ext_mic_en)) {
		ret = gpio_request(pdata->gpio_ext_mic_en, "ext_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get ext_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_EXT_MIC_EN;

		/* Enable ext mic; enable signal is active-low, this will let MICBIAS_EXT
		    output 3.3v for external microphone recording */
		gpio_direction_output(pdata->gpio_ext_mic_en, 1);
		gpio_export(pdata->gpio_ext_mic_en, false);
	}else{
		dev_info(card->dev, "gpio_ext_mic_en is not valid\n");
	}

	if (gpio_is_valid(pdata->gpio_ext_mic_det)) {
		tegra_aic325x_hook_key_gpio.gpio = pdata->gpio_ext_mic_det;
		ret = snd_soc_jack_new(codec, "Hook key", SND_JACK_HEADSET,
							   &tegra_aic325x_hook_key);
		if (ret < 0) {
			printk("**Err snd_soc_jack_new\n");
			return ret;
		}
#ifdef CONFIG_SWITCH
		//snd_soc_jack_notifier_register(&tegra_aic325x_hook_key,
		//							   &aic325x_hook_key_nb);
#else /*gpio based hook key detection*/
		snd_soc_jack_add_pins(&tegra_aic325x_hook_key,
							  ARRAY_SIZE(tegra_aic325x_hook_key_pins),
							  tegra_aic325x_hook_key_pins);
#endif
		ret = snd_soc_jack_add_gpios(&tegra_aic325x_hook_key, 1,
									 &tegra_aic325x_hook_key_gpio);
		if (ret < 0) {
			printk("**snd_soc_jack_add_gpios fail\n");
		}
		machine->gpio_requested |= GPIO_EXT_MIC_DET;
	}
	/* setup hook key input event */
	set_bit(EV_KEY, tegra_aic325x_hook_key.jack->input_dev->evbit);
	set_bit(KEY_MEDIA, tegra_aic325x_hook_key.jack->input_dev->keybit);



	//currently is valid, HP_DET_GPIO
	if (gpio_is_valid(pdata->gpio_hp_det)) {
		tegra_aic325x_hp_jack_gpio.gpio = pdata->gpio_hp_det;
		ret = snd_soc_jack_new(codec, "Headphone jack", SND_JACK_HEADPHONE,
							   &tegra_aic325x_hp_jack);
		if (ret < 0) {
			printk("**Err snd_soc_jack_new\n");
			return ret;
		}
#ifdef CONFIG_SWITCH
		snd_soc_jack_notifier_register(&tegra_aic325x_hp_jack,
									   &aic325x_headset_switch_nb);
#else /*gpio based headset detection*/
		snd_soc_jack_add_pins(&tegra_aic325x_hp_jack,
							  ARRAY_SIZE(tegra_aic325x_hp_jack_pins),
							  tegra_aic325x_hp_jack_pins);
#endif
		ret = snd_soc_jack_add_gpios(&tegra_aic325x_hp_jack, 1,
									 &tegra_aic325x_hp_jack_gpio);
		if (ret < 0) {
			printk("**snd_soc_jack_add_gpios fail\n");
		}
		machine->gpio_requested |= GPIO_HP_DET;
	}

	ret = snd_soc_add_controls(codec, tegra_aic325x_controls,
				   ARRAY_SIZE(tegra_aic325x_controls));
	if (ret < 0) {
		printk("**Err snd_soc_add_controls ret: %d\n", ret );
		return ret;
	}

	snd_soc_dapm_new_controls(dapm, tegra_aic325x_dapm_widgets,
					ARRAY_SIZE(tegra_aic325x_dapm_widgets));

	ret = snd_soc_dapm_add_routes(dapm, aic325x_audio_map,
					ARRAY_SIZE(aic325x_audio_map));
	if(ret)
	{
		printk(KERN_INFO "%s:failed adding routes\n",__func__);
	}


	snd_soc_dapm_sync(dapm);

	return 0;
}

int tegra_aic325x_suspend_pre(struct snd_soc_card *card)
{
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	regulator_disable(machine->vdd_5v0_sys);//for saving vdd_5v0 power consumption
	return 0;
}

int tegra_aic325x_resume_post(struct snd_soc_card *card)
{
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	regulator_enable(machine->vdd_5v0_sys);//to tell kernel audio is using vdd_5v0
	aic325x_jack_resume();
	return 0;
}

static struct snd_soc_dai_link tegra_aic325x_dai[] = {
	[DAI_LINK_HIFI] = {
		.name = "TLV320AIC325x",
		.stream_name = "TLV320AIC325x",
		.codec_name = "tlv320aic325x.0-0018",
		.platform_name = "tegra-pcm-audio",
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		.cpu_dai_name = "tegra20-i2s.0",
#else
		.cpu_dai_name = "tegra30-i2s.1",
#endif
		.codec_dai_name = "tlv320aic325x-MM_EXT",
		.init = tegra_aic325x_init,
		.ops = &tegra_aic325x_hifi_ops,
	},
	[DAI_LINK_SPDIF] = {
		.name = "SPDIF",
		.stream_name = "SPDIF PCM",
		.codec_name = "spdif-dit.0",
		.platform_name = "tegra-pcm-audio",
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		.cpu_dai_name = "tegra20-spdif",
#else
		.cpu_dai_name = "tegra30-spdif",
#endif
		.codec_dai_name = "dit-hifi",
		.ops = &tegra_aic325x_spdif_ops,
	},
	[DAI_LINK_BTSCO] = {
		.name = "BT-SCO",
		.stream_name = "BT SCO PCM",
		.codec_name = "spdif-dit.1",
		.platform_name = "tegra-pcm-audio",
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		.cpu_dai_name = "tegra20-i2s.1",
#else
		.cpu_dai_name = "tegra30-i2s.3",
#endif
		.codec_dai_name = "dit-hifi",
		.init = tegra_aic325x_init,
		.ops = &tegra_aic325x_bt_ops,
	},
};

static struct snd_soc_card snd_soc_tegra_aic325x = {
	.name = "tegra-aic325x",
	.dai_link = tegra_aic325x_dai,
	.num_links = ARRAY_SIZE(tegra_aic325x_dai),
	.suspend_pre = tegra_aic325x_suspend_pre,
	.resume_post = tegra_aic325x_resume_post,
};

ssize_t voice_call_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
       struct snd_soc_card *card = &snd_soc_tegra_aic325x;
       struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);

       if(*buf == '0')
               machine->voice_call = false;
       else if(*buf == '1')
               machine->voice_call = true;
       return len;
}

ssize_t voice_call_show(struct device *dev, struct device_attribute *attr, char *buf)
{
       struct snd_soc_card *card = &snd_soc_tegra_aic325x;
       struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);

       sprintf(buf, "%d\n", machine->voice_call);
       return strlen(buf);
}
DEVICE_ATTR(voice_call, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, voice_call_show, voice_call_store);

ssize_t input_source_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct snd_soc_card *card = &snd_soc_tegra_aic325x;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);

	if(len != 2){
		return -EINVAL;
	}

	machine->input_source = *buf-'0';
	return len;
}

ssize_t input_source_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct snd_soc_card *card = &snd_soc_tegra_aic325x;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);

	sprintf(buf, "%d\n", machine->input_source);
	return strlen(buf);
}
DEVICE_ATTR(input_source, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, input_source_show, input_source_store);

ssize_t io_device_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct snd_soc_card *card = &snd_soc_tegra_aic325x;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	int temp;

	temp = *buf-'0';
	if(machine->io_device != temp) {
		machine->io_device = temp;
	}
	return len;
}

ssize_t io_device_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct snd_soc_card *card = &snd_soc_tegra_aic325x;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);

	sprintf(buf, "%x\n", machine->io_device);
	return strlen(buf);
}
DEVICE_ATTR(io_device, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, io_device_show, io_device_store);

ssize_t audio_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct snd_soc_card *card = &snd_soc_tegra_aic325x;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);

	machine->audio_mode = *buf-'0';
	return len;
}

ssize_t audio_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct snd_soc_card *card = &snd_soc_tegra_aic325x;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);

	sprintf(buf, "%d\n", machine->audio_mode);
	return strlen(buf);
}
DEVICE_ATTR(audio_mode, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, audio_mode_show, audio_mode_store);

static long tegra_aic325x_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	struct snd_soc_card *card = &snd_soc_tegra_aic325x;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	int ret = 0;

	switch(cmd) {
	case TEGRA_AIC325X_SET_ENABLE_FM34:
		machine->voice_call = true;
		break;
	case TEGRA_AIC325X_SET_DISABLE_FM34:
		machine->voice_call = false;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
static const struct file_operations tegra_aic325x_fileops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = tegra_aic325x_ioctl,
};

static struct miscdevice tegra_aic325x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = TEGRA_AIC325X_NAME,
	.fops = &tegra_aic325x_fileops,
};

static __devinit int tegra_aic325x_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_aic325x;
	struct tegra_aic325x *machine;
	struct tegra_aic325x_platform_data *pdata;
	int ret, i;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	machine = kzalloc(sizeof(struct tegra_aic325x), GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_aic325x struct\n");
		printk("No MEMORY\n");
		return -ENOMEM;
	}

	machine->pdata = pdata;
#ifdef CONFIG_SND_SOC_FM34
	mutex_init(&machine->fm34_lock);
#endif

	ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev , card);
	if (ret) {
		printk(" tegra_asoc_utils_init failed\n");
		goto err_free_machine;
	}
	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	for (i = 0; i < NUM_I2S_DEVICES ; i++)
		machine->codec_info[i].i2s_id = pdata->audio_port_id[i];

	machine->codec_info[BASEBAND].rate = pdata->baseband_param.rate;
	machine->codec_info[BASEBAND].channels = pdata->baseband_param.channels;
	machine->codec_info[BASEBAND].i2s_id = -1;

#endif

#ifdef CONFIG_SWITCH
	/* Add h2w switch class support */
	ret = switch_dev_register(&aic325x_wired_switch_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "not able to register switch device %d\n",
			ret);
		goto err_unregister_card;
	}
#endif

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_fini_utils;
	}

	if (!card->instantiated) {
		dev_err(&pdev->dev, "No TI AIC3262 codec\n");
		goto err_unregister_card;
	}


#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	ret = tegra_asoc_utils_set_parent(&machine->util_data,
				pdata->i2s_param[HIFI_CODEC].is_i2s_master);
	if (ret) {
		dev_err(&pdev->dev, "tegra_asoc_utils_set_parent failed (%d)\n",
			ret);
		goto err_unregister_card;
	}
#endif


	ret = device_create_file(card->dev, &dev_attr_input_source);
	if (ret) {
		dev_err(card->dev, "%s: add_sysfs_entry input_source failed\n", __FUNCTION__);
	}

	ret = device_create_file(card->dev, &dev_attr_io_device);
	if (ret) {
		dev_err(card->dev, "%s: add_sysfs_entry output_source failed\n", __FUNCTION__);
	}

	ret = device_create_file(card->dev, &dev_attr_audio_mode);
	if (ret) {
		dev_err(card->dev, "%s: add_sysfs_entry audio_mode failed\n", __FUNCTION__);
	}

	ret = device_create_file(card->dev, &dev_attr_voice_call);
	if (ret) {
		dev_err(card->dev, "%s: add_sysfs_entry voice_call failed\n", __FUNCTION__);
	}

	ret = misc_register(&tegra_aic325x_device);
	if (ret) {
		pr_err("%s : Unable to register misc device!\n", __func__);
		return ret;
	}

	return 0;

err_unregister_card:
	snd_soc_unregister_card(card);
err_fini_utils:
	tegra_asoc_utils_fini(&machine->util_data);
err_free_machine:
	kfree(machine);
	return ret;
}

static int __devexit tegra_aic325x_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	struct tegra_aic325x_platform_data *pdata = machine->pdata;
	device_remove_file(card->dev, &dev_attr_input_source);
	device_remove_file(card->dev, &dev_attr_io_device);
	device_remove_file(card->dev, &dev_attr_audio_mode);
	device_remove_file(card->dev, &dev_attr_voice_call);

	snd_soc_unregister_card(card);
	misc_deregister(&tegra_aic325x_device);

#ifdef CONFIG_SWITCH
	switch_dev_unregister(&aic325x_wired_switch_dev);
#endif

	tegra_asoc_utils_fini(&machine->util_data);

	if (machine->gpio_requested & GPIO_EXT_MIC_EN)
		gpio_free(pdata->gpio_ext_mic_en);
	if (machine->gpio_requested & GPIO_INT_MIC_EN)
		gpio_free(pdata->gpio_int_mic_en);
	if (machine->gpio_requested & GPIO_HP_MUTE)
		gpio_free(pdata->gpio_hp_mute);
	if (machine->gpio_requested & GPIO_SPKR_EN)
		gpio_free(pdata->gpio_spkr_en);

	kfree(machine);

	return 0;
}

static void tegra_aic325x_power_off(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_codec *codec = machine->codec;
    struct aic325x_priv *aic325x = snd_soc_codec_get_drvdata(codec);

	/* software reset codec, page 0, reg1 = 0x01 */
	snd_soc_update_bits(codec, 0x1, 0xff, 0x01 );

	if ( regulator_is_enabled(aic325x->reg) )
		regulator_disable(aic325x->reg);

}

static struct platform_driver tegra_aic325x_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = tegra_aic325x_driver_probe,
	.remove = __devexit_p(tegra_aic325x_driver_remove),
	.shutdown = tegra_aic325x_power_off
};

static int __init tegra_aic325x_modinit(void)
{
	int ret;
	ret=platform_driver_register(&tegra_aic325x_driver);
	return ret;
}

module_init(tegra_aic325x_modinit);

static void __exit tegra_aic325x_modexit(void)
{
	platform_driver_unregister(&tegra_aic325x_driver);
}
module_exit(tegra_aic325x_modexit);

/* Module information */
MODULE_AUTHOR("Vinod G. <vinodg@nvidia.com>");
MODULE_DESCRIPTION("Tegra+AIC3262 machine ASoC driver");
MODULE_DESCRIPTION("Tegra ALSA SoC");
MODULE_LICENSE("GPL");

