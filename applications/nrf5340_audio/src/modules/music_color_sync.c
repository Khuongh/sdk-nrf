/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "music_color_sync.h"

#include <zephyr/kernel.h>

#include "macros_common.h"
#include "board_version.h"
#include "sw_codec_select.h"
#include "dsp/basic_math_functions.h"
#include "dsp/transform_functions.h"
#include "dsp/complex_math_functions.h"
#include "led.h"
#include "nrfx_pwm.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(music_color_sync, CONFIG_LOG_MUSIC_COLOR_SYNC_LEVEL);

#define MUSIC_SYNC_RED_PIN DT_GPIO_PIN(DT_NODELABEL(rgb1_red), gpios)
#define MUSIC_SYNC_GREEN_PIN DT_GPIO_PIN(DT_NODELABEL(rgb1_green), gpios)
#define MUSIC_SYNC_BLUE_PIN DT_GPIO_PIN(DT_NODELABEL(rgb1_blue), gpios)

static struct k_thread music_color_sync_thread_data;
static k_tid_t music_color_sync_id;

K_THREAD_STACK_DEFINE(music_color_sync_thread_stack, CONFIG_MUSIC_COLOR_SYNC_STACK_SIZE);

K_SEM_DEFINE(music_color_sync_update_sem, 0, 1);
K_SEM_DEFINE(music_color_sync_value_sem, 0, 1);

char decoded_pcm_data_mono[PCM_NUM_BYTES_MONO];

static nrfx_pwm_t m_pwm = NRFX_PWM_INSTANCE(0);
static nrf_pwm_values_individual_t seq_values[] = { 0 };

nrf_pwm_sequence_t const seq = { .values.p_individual = seq_values,
				 .length = NRF_PWM_VALUES_LENGTH(seq_values),
				 .repeats = 0,
				 .end_delay = 0 };

static void music_color_sync_fft_analyse(q15_t *fft_data, uint16_t len)
{
	uint32_t frequency_modules[3] = { 0 };

	/* Assigning FFT bins into modules */
	for (int i = 0; i < len; i++) {
		if (fft_data[(i + 1) * 2] > 0) {
			if (i < CONFIG_RGB_RED_FREQ_BIN_RANGE) {
				frequency_modules[0] += fft_data[i];
			} else if (i < CONFIG_RGB_GREEN_FREQ_BIN_RANGE) {
				frequency_modules[1] += fft_data[i];
			} else {
				frequency_modules[2] += fft_data[i];
			}
		}
	}

	/*  Increasing intensity of more dominant frequencies and reducing the other */
	if ((frequency_modules[0] > frequency_modules[1]) &&
	    (frequency_modules[0] > frequency_modules[2])) {
		frequency_modules[0] *= 1.5;
		frequency_modules[1] *= 0.5;
		frequency_modules[2] *= 0.5;
	} else if ((frequency_modules[1] > frequency_modules[0]) &&
		   (frequency_modules[1] > frequency_modules[2])) {
		frequency_modules[1] *= 1.5;
		frequency_modules[0] *= 0.5;
		frequency_modules[2] *= 0.5;
	} else if ((frequency_modules[2] > frequency_modules[0]) &&
		   frequency_modules[2] > frequency_modules[1]) {
		frequency_modules[2] *= 1.5;
		frequency_modules[0] *= 0.5;
		frequency_modules[1] *= 0.5;
	}

	/* Calculating duty cycle values, +1 added to prevent division by 0 */
	uint32_t total_modules_value =
		frequency_modules[0] + frequency_modules[1] + frequency_modules[2] + 1;
	seq_values->channel_0 = 100 - (frequency_modules[0] * 100 / total_modules_value);
	seq_values->channel_1 = 100 - (frequency_modules[1] * 100 / total_modules_value);
	seq_values->channel_2 = 100 - (frequency_modules[2] * 100 / total_modules_value);
}

static int music_color_sync_thread(void *dummy1, void *dummy2, void *dummy3)
{
	arm_rfft_instance_q15 fft_instance;
	arm_status status;
	q15_t input[CONFIG_FFT_SAMPLE_SIZE];
	q15_t output[CONFIG_FFT_SAMPLE_SIZE * 2];

	status = arm_rfft_init_q15(&fft_instance, CONFIG_FFT_SAMPLE_SIZE,
				   CONFIG_FFT_FLAG_INVERSE_TRANSFORM, CONFIG_FFT_FLAG_BIT_REVERSAL);
	if (status != ARM_MATH_SUCCESS) {
		LOG_ERR("Failed to init FFT instance - (err: %d)", status);
		return status;
	}

	while (1) {
		k_sem_take(&music_color_sync_update_sem, K_FOREVER);
		k_sem_take(&music_color_sync_value_sem, K_FOREVER);
		memcpy(input, decoded_pcm_data_mono, CONFIG_FFT_SAMPLE_SIZE);
		k_sem_give(&music_color_sync_value_sem);

		arm_rfft_q15(&fft_instance, (q15_t *)decoded_pcm_data_mono, output);
		arm_abs_q15(output, output, CONFIG_FFT_SAMPLE_SIZE * 2);
		music_color_sync_fft_analyse(output, CONFIG_FFT_SAMPLE_SIZE);
		nrfx_pwm_simple_playback(&m_pwm, &seq, 1, NRFX_PWM_FLAG_LOOP);
		k_msleep(100);

		STACK_USAGE_PRINT("music_color_sync_thread", &music_color_sync_thread_data);
	}
}

int music_color_sync_data_update(char *pcm_data_mono, uint16_t len)
{
	if (k_sem_take(&music_color_sync_value_sem, K_NO_WAIT)) {
		memcpy(decoded_pcm_data_mono, pcm_data_mono, len);
		k_sem_give(&music_color_sync_value_sem);
		k_sem_give(&music_color_sync_update_sem);
		return 0;
	} else {
		return -EBUSY;
	}
}

int music_color_sync_init()
{
	int ret;
	static nrfx_pwm_config_t const pwm_cfg = 
    {
        .output_pins =
        {
            MUSIC_SYNC_RED_PIN,
            MUSIC_SYNC_GREEN_PIN,
            MUSIC_SYNC_BLUE_PIN,
            NRFX_PWM_PIN_NOT_USED,
        },
        .irq_priority = 6,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 100,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO,
        .skip_gpio_cfg = false,
        .skip_psel_cfg = false
    };

	ret = nrfx_pwm_init(&m_pwm, &pwm_cfg, NULL, NULL);

	music_color_sync_id =
		k_thread_create(&music_color_sync_thread_data, music_color_sync_thread_stack,
				CONFIG_MUSIC_COLOR_SYNC_STACK_SIZE,
				(k_thread_entry_t)music_color_sync_thread, NULL, NULL, NULL,
				K_PRIO_PREEMPT(CONFIG_MUSIC_COLOR_SYNC_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(music_color_sync_id, "MUSIC COLOR SYNC");
	if (ret) {
		LOG_ERR("Failed to name thread - (err: %d)", ret);
		return ret;
	}

	return 0;
}