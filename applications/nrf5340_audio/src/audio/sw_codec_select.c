/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "sw_codec_select.h"

#include <zephyr/kernel.h>
#include <errno.h>
#include <math.h>

#include "audio_sync_timer.h"
#include "board_version.h"
#include "led.h"
#include "dsp/transform_functions.h"
#include "dsp/basic_math_functions.h"
#include "channel_assignment.h"
#include "pcm_stream_channel_modifier.h"
#if (CONFIG_SW_CODEC_LC3)
#include "sw_codec_lc3.h"
#endif /* (CONFIG_SW_CODEC_LC3) */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sw_codec_select);

static struct sw_codec_config m_config;

#if (CONFIG_RGB_MUSIC_SYNC)
static enum led_color sw_codec_fft_analyse(q15_t *fft_data, uint32_t len)
{
	int32_t sub_bass_module, bass_module, low_mid_module, mid_module, high_mid_module,
		presence_module, brilliance_module;
	
	static int first = 0;
	if (first == 20) {
		printk("0: ");
		for (int i = 0; i < 256; i++){
			printk("%d ",fft_data[i]);
			if ((i%20) == 0 && i != 0){
				printk("\n%d:", i/20);
			}
			// LOG_INF("Data[%d]: %d", (960 + i), ctrl_blk.out.fifo[960 + i]);
		}
	}
	first ++;

	for (int i = 0; i < len; i++) {
		q15_t value = fft_data[i];
		if (i < SUB_BASS_RANGE) {
			sub_bass_module += value;
		} else if (i < BASS_RANGE) {
			bass_module += value;
		} else if (i < LOW_MID_RANGE) {
			low_mid_module += value;
		} else if (i < MID_RANGE) {
			mid_module += value;
		} else if (i < HIGH_MID_RANGE) {
			high_mid_module += value;
		} else if (i < PRESENCE_RANGE) {
			presence_module += value;
		} else {
			brilliance_module += value;
		}
	}

	LOG_INF("Sub_Bass: %d, Bass: %d, Low_mid: %d, Mid: %d, High_mid: %d, Presence: %d, Brilliance: %d",
		sub_bass_module, bass_module, low_mid_module, mid_module, high_mid_module,
		presence_module, brilliance_module);

	// if ((sub_bass_module > bass_module) && (sub_bass_module > low_mid_module) &&
	//     (sub_bass_module > mid_module) && (sub_bass_module > high_mid_module) &&
	//     (sub_bass_module > presence_module) && (sub_bass_module > brilliance_module)) {
	// 	return LED_COLOR_RED;
	// } else if ((bass_module > sub_bass_module) && (bass_module > low_mid_module) &&
	// 	   (bass_module > mid_module) && (bass_module > high_mid_module) &&
	// 	   (bass_module > presence_module) && (bass_module > brilliance_module)) {
	// 	return LED_COLOR_YELLOW;
	// } else if ((low_mid_module > bass_module) && (low_mid_module > sub_bass_module) &&
	// 	   (low_mid_module > mid_module) && (low_mid_module > high_mid_module) &&
	// 	   (low_mid_module > presence_module) && (low_mid_module > brilliance_module)) {
	// 	return LED_COLOR_GREEN;
	// } else if ((mid_module > bass_module) && (mid_module > low_mid_module) &&
	// 	   (mid_module > sub_bass_module) && (mid_module > high_mid_module) &&
	// 	   (mid_module > presence_module) && (mid_module > brilliance_module)) {
	// 	return LED_COLOR_BLUE;
	// } else if ((high_mid_module > bass_module) && (high_mid_module > low_mid_module) &&
	// 	   (high_mid_module > mid_module) && (high_mid_module > sub_bass_module) &&
	// 	   (high_mid_module > presence_module) && (high_mid_module > brilliance_module)) {
	// 	return LED_COLOR_CYAN;
	// } else if ((presence_module > bass_module) && (presence_module > low_mid_module) &&
	// 	   (presence_module > mid_module) && (presence_module > high_mid_module) &&
	// 	   (presence_module > sub_bass_module) && (presence_module > brilliance_module)) {
	// 	return LED_COLOR_MAGENTA;
	// } else {
	// 	return LED_COLOR_WHITE;
	// }
	return LED_COLOR_WHITE;
}

static int sw_codec_fft(char *pcm_data_mono)
{
	arm_rfft_instance_q15 fft_instance;
	arm_status status;
	q15_t output[CONFIG_FFT_SAMPLE_SIZE * 2];
	enum led_color fft_led_color;

	status = arm_rfft_init_q15(&fft_instance, CONFIG_FFT_SAMPLE_SIZE,
				   CONFIG_FFT_FLAG_INVERSE_TRANSFORM, CONFIG_FFT_FLAG_BIT_REVERSAL);
	if (status != ARM_MATH_SUCCESS) {
		LOG_ERR("Failed to init FFT instance - (err; %d)", status);
		return status;
	}

	arm_rfft_q15(&fft_instance, (q15_t *)pcm_data_mono, output);
	arm_abs_q15(output, output, CONFIG_FFT_SAMPLE_SIZE);
	fft_led_color = sw_codec_fft_analyse(output, CONFIG_FFT_SAMPLE_SIZE);
	led_on(LED_APP_RGB, fft_led_color);

	return 0;
}
#endif

int sw_codec_encode(void *pcm_data, size_t pcm_size, uint8_t **encoded_data, size_t *encoded_size)
{
	/* Temp storage for split stereo PCM signal */
	char pcm_data_mono[AUDIO_CH_NUM][PCM_NUM_BYTES_MONO] = { 0 };
	/* Make sure we have enough space for two frames (stereo) */
	static uint8_t m_encoded_data[ENC_MAX_FRAME_SIZE * AUDIO_CH_NUM];

	size_t pcm_block_size_mono;
	int ret;

	if (!m_config.encoder.enabled) {
		LOG_ERR("Encoder has not been initialized");
		return -ENXIO;
	}

	switch (m_config.sw_codec) {
	case SW_CODEC_LC3: {
#if (CONFIG_SW_CODEC_LC3)
		uint16_t encoded_bytes_written;

		/* Since LC3 is a single channel codec, we must split the
		 * stereo PCM stream
		 */
		ret = pscm_two_channel_split(pcm_data, pcm_size, CONFIG_AUDIO_BIT_DEPTH_BITS,
					     pcm_data_mono[AUDIO_CH_L], pcm_data_mono[AUDIO_CH_R],
					     &pcm_block_size_mono);
		if (ret) {
			return ret;
		}

		switch (m_config.encoder.channel_mode) {
		case SW_CODEC_MONO: {
			ret = sw_codec_lc3_enc_run(pcm_data_mono[m_config.encoder.audio_ch],
						   pcm_block_size_mono, LC3_USE_BITRATE_FROM_INIT,
						   0, sizeof(m_encoded_data), m_encoded_data,
						   &encoded_bytes_written);
			if (ret) {
				return ret;
			}
			break;
		}
		case SW_CODEC_STEREO: {
			ret = sw_codec_lc3_enc_run(pcm_data_mono[AUDIO_CH_L], pcm_block_size_mono,
						   LC3_USE_BITRATE_FROM_INIT, AUDIO_CH_L,
						   sizeof(m_encoded_data), m_encoded_data,
						   &encoded_bytes_written);
			if (ret) {
				return ret;
			}

			ret = sw_codec_lc3_enc_run(pcm_data_mono[AUDIO_CH_R], pcm_block_size_mono,
						   LC3_USE_BITRATE_FROM_INIT, AUDIO_CH_R,
						   sizeof(m_encoded_data) - encoded_bytes_written,
						   m_encoded_data + encoded_bytes_written,
						   &encoded_bytes_written);
			if (ret) {
				return ret;
			}
			encoded_bytes_written += encoded_bytes_written;
			break;
		}
		default:
			LOG_ERR("Unsupported channel mode: %d", m_config.encoder.channel_mode);
			return -ENODEV;
		}

		*encoded_data = m_encoded_data;
		*encoded_size = encoded_bytes_written;

#endif /* (CONFIG_SW_CODEC_LC3) */
		break;
	}
	default:
		LOG_ERR("Unsupported codec: %d", m_config.sw_codec);
		return -ENODEV;
	}

	return 0;
}

int sw_codec_decode(uint8_t const *const encoded_data, size_t encoded_size, bool bad_frame,
		    void **decoded_data, size_t *decoded_size)
{
	if (!m_config.decoder.enabled) {
		LOG_ERR("Decoder has not been initialized");
		return -ENXIO;
	}

	int ret;
	char pcm_data_mono[PCM_NUM_BYTES_MONO] = { 0 };
	static char pcm_data_stereo[PCM_NUM_BYTES_STEREO];

	size_t pcm_size_stereo = 0;
	size_t pcm_size_session = 0;

	switch (m_config.sw_codec) {
	case SW_CODEC_LC3: {
#if (CONFIG_SW_CODEC_LC3)
		/* Typically used for right channel if stereo signal */
		char pcm_data_mono_right[PCM_NUM_BYTES_MONO] = { 0 };

		switch (m_config.decoder.channel_mode) {
		case SW_CODEC_MONO: {
			if (bad_frame && IS_ENABLED(CONFIG_SW_CODEC_OVERRIDE_PLC)) {
				memset(pcm_data_mono, 0, PCM_NUM_BYTES_MONO);
				pcm_size_session = PCM_NUM_BYTES_MONO;
			} else {
				ret = sw_codec_lc3_dec_run(encoded_data, encoded_size,
							   LC3_PCM_NUM_BYTES_MONO, 0, pcm_data_mono,
							   (uint16_t *)&pcm_size_session,
							   bad_frame);
				if (ret) {
					return ret;
				}
			}

			/* For now, i2s is only stereo, so in order to send
			 * just one channel, we need to insert 0 for the
			 * other channel
			 */
			ret = pscm_zero_pad(pcm_data_mono, pcm_size_session,
					    m_config.decoder.audio_ch, CONFIG_AUDIO_BIT_DEPTH_BITS,
					    pcm_data_stereo, &pcm_size_stereo);
			if (ret) {
				return ret;
			}
			break;
		}
		case SW_CODEC_STEREO: {
			if (bad_frame && IS_ENABLED(CONFIG_SW_CODEC_OVERRIDE_PLC)) {
				memset(pcm_data_mono, 0, PCM_NUM_BYTES_MONO);
				memset(pcm_data_mono_right, 0, PCM_NUM_BYTES_MONO);
				pcm_size_session = PCM_NUM_BYTES_MONO;
			} else {
				/* Decode left channel */
				ret = sw_codec_lc3_dec_run(encoded_data, encoded_size / 2,
							   LC3_PCM_NUM_BYTES_MONO, AUDIO_CH_L,
							   pcm_data_mono,
							   (uint16_t *)&pcm_size_session,
							   bad_frame);
				if (ret) {
					return ret;
				}
				/* Decode right channel */
				ret = sw_codec_lc3_dec_run((encoded_data + (encoded_size / 2)),
							   encoded_size / 2, LC3_PCM_NUM_BYTES_MONO,
							   AUDIO_CH_R, pcm_data_mono_right,
							   (uint16_t *)&pcm_size_session,
							   bad_frame);
				if (ret) {
					return ret;
				}
			}
			ret = pscm_combine(pcm_data_mono, pcm_data_mono_right, pcm_size_session,
					   CONFIG_AUDIO_BIT_DEPTH_BITS, pcm_data_stereo,
					   &pcm_size_stereo);
			if (ret) {
				return ret;
			}
			break;
		}
		default:
			LOG_ERR("Unsupported channel mode: %d", m_config.encoder.channel_mode);
			return -ENODEV;
		}

		if (CONFIG_RGB_MUSIC_SYNC) {
			uint32_t delta_time = 100000;
			static uint32_t last_time_stamp;
			uint32_t time_now = audio_sync_timer_curr_time_get();
			if ((time_now - last_time_stamp) >= delta_time) {
				sw_codec_fft(pcm_data_mono);
				last_time_stamp = time_now;
			}
		}

		*decoded_size = pcm_size_stereo;
		*decoded_data = pcm_data_stereo;
#endif /* (CONFIG_SW_CODEC_LC3) */
		break;
	}
	default:
		LOG_ERR("Unsupported codec: %d", m_config.sw_codec);
		return -ENODEV;
	}
	return 0;
}

int sw_codec_uninit(struct sw_codec_config sw_codec_cfg)
{
	int ret;

	if (m_config.sw_codec != sw_codec_cfg.sw_codec) {
		LOG_ERR("Trying to uninit a codec that is not first initialized");
		return -ENODEV;
	}
	switch (m_config.sw_codec) {
	case SW_CODEC_LC3:
#if (CONFIG_SW_CODEC_LC3)
		if (sw_codec_cfg.encoder.enabled) {
			if (!m_config.encoder.enabled) {
				LOG_ERR("Trying to uninit encoder, it has not been initialized");
				return -EALREADY;
			}
			ret = sw_codec_lc3_enc_uninit_all();
			if (ret) {
				return ret;
			}
			m_config.encoder.enabled = false;
		}

		if (sw_codec_cfg.decoder.enabled) {
			if (!m_config.decoder.enabled) {
				LOG_WRN("Trying to uninit decoder, it has not been initialized");
				return -EALREADY;
			}
			ret = sw_codec_lc3_dec_uninit_all();
			if (ret) {
				return ret;
			}
			m_config.decoder.enabled = false;
		}
#endif /* (CONFIG_SW_CODEC_LC3) */
		break;
	default:
		LOG_ERR("Unsupported codec: %d", m_config.sw_codec);
		return false;
	}
	return 0;
}

int sw_codec_init(struct sw_codec_config sw_codec_cfg)
{
	switch (sw_codec_cfg.sw_codec) {
	case SW_CODEC_LC3: {
#if (CONFIG_SW_CODEC_LC3)
		int ret;

		if (m_config.sw_codec != SW_CODEC_LC3) {
			/* Check if LC3 is already initialized */
			ret = sw_codec_lc3_init(NULL, NULL, CONFIG_AUDIO_FRAME_DURATION_US);
			if (ret) {
				return ret;
			}
		}

		if (sw_codec_cfg.encoder.enabled) {
			if (m_config.encoder.enabled) {
				LOG_WRN("The LC3 encoder is already initialized");
				return -EALREADY;
			}
			uint16_t pcm_bytes_req_enc;

			ret = sw_codec_lc3_enc_init(
				CONFIG_AUDIO_SAMPLE_RATE_HZ, CONFIG_AUDIO_BIT_DEPTH_BITS,
				CONFIG_AUDIO_FRAME_DURATION_US, sw_codec_cfg.encoder.bitrate,
				sw_codec_cfg.encoder.channel_mode, &pcm_bytes_req_enc);
			if (ret) {
				return ret;
			}
		}

		if (sw_codec_cfg.decoder.enabled) {
			if (m_config.decoder.enabled) {
				LOG_WRN("The LC3 decoder is already initialized");
				return -EALREADY;
			}
			ret = sw_codec_lc3_dec_init(CONFIG_AUDIO_SAMPLE_RATE_HZ,
						    CONFIG_AUDIO_BIT_DEPTH_BITS,
						    CONFIG_AUDIO_FRAME_DURATION_US,
						    sw_codec_cfg.decoder.channel_mode);
			if (ret) {
				return ret;
			}
		}
		break;
#endif /* (CONFIG_SW_CODEC_LC3) */
		LOG_ERR("LC3 is not compiled in, please open menuconfig and select LC3");
		return -ENODEV;
	}
	default:
		LOG_ERR("Unsupported codec: %d", sw_codec_cfg.sw_codec);
		return false;
	}

	m_config = sw_codec_cfg;
	return 0;
}
