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
#include "led.h"
#include "nrfx_pwm.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(music_color_sync, CONFIG_LOG_MUSIC_COLOR_SYNC_LEVEL);

/*
 *
 * Define global variables
 * - k_thread
 * - k_tid_t (id)
 * - thread_stack
 * - semaphore
 * - pcm signal
 */

static struct k_thread music_color_sync_thread_data;
static k_tid_t music_color_sync_id;
K_THREAD_STACK_DEFINE(music_color_sync_thread_stack, CONFIG_MUSIC_COLOR_SYNC_STACK_SIZE);

K_SEM_DEFINE(music_color_sync_update_sem, 0, 1);
K_SEM_DEFINE(music_color_sync_value_sem, 0, 1);

char decoded_pcm_data_mono[PCM_NUM_BYTES_MONO];

static nrfx_pwm_t m_pwm = NRFX_PWM_INSTANCE(0);

static nrf_pwm_values_individual_t seq_values[] = {0, 0, 0, 0};

nrf_pwm_sequence_t const seq = 
{
    .values.p_individual = seq_values,
    .length              = NRF_PWM_VALUES_LENGTH(seq_values),
    .repeats             = 0,
    .end_delay           = 0
};

static enum led_color sw_codec_fft_analyse(q15_t *fft_data, uint16_t len)
{
	uint16_t frequency_modules[7] = { 0 };
	uint16_t max_index;
	uint16_t max_value = 0;
	enum led_color frequency_color;
	
	/* Assigning FFT bins into modules */
	for (int i = 0; i < len; i++) {
		if(fft_data[i] >= 2){
			if (i < SUB_BASS_RANGE) {
				frequency_modules[0] += fft_data[i];
			} else if (i < BASS_RANGE) {
				frequency_modules[1] += fft_data[i];
			} else if (i < LOW_MID_RANGE) {
				frequency_modules[2] += fft_data[i];
			} else if (i < MID_RANGE) {
				frequency_modules[3] += fft_data[i];
			} else if (i < HIGH_MID_RANGE) {
				frequency_modules[4] += fft_data[i];
			} else if (i < PRESENCE_RANGE) {
				frequency_modules[5] += fft_data[i];
			} else {
				frequency_modules[6] += fft_data[i];
			}
		}
	}

	/* Finding max value from modules */
	for (int i = 0; i < sizeof(frequency_modules)/sizeof(frequency_modules[0]); i++) {
		if (frequency_modules[i] > max_value) {
			max_value = frequency_modules[i];
			/*  Enum for RGB LED index 0 is off */
			max_index = (i + 1);
		}
	}

	/* Returning enum index for led_color */
	return(max_index);
}

static void music_color_sync_thread(void *dummy1, void *dummy2, void *dummy3)
{
    /*
     *  init verdier
     * while(1)
     * take sem - value updated
     * take sem - wait forever
     *      memcopy values
     * give sem
     *      run fft of new data
     *      Assign result into 3 modules (R,G,B)
     *      update RGB led with new values
     */

    arm_rfft_instance_q15 fft_instance;
    arm_status status;
    q15_t input[CONFIG_FFT_SAMPLE_SIZE];
    q15_t output[CONFIG_FFT_SAMPLE_SIZE * 2];
    enum led_color fft_led_color;

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
	    arm_abs_q15(output, output, CONFIG_FFT_SAMPLE_SIZE);
	    fft_led_color = sw_codec_fft_analyse(output, CONFIG_FFT_SAMPLE_SIZE);
	    // led_on(LED_APP_RGB, fft_led_color);
        seq_values->channel_1 = 70;
        seq_values->channel_2 = 85;
        seq_values->channel_0 = 40;
        nrfx_pwm_simple_playback(&m_pwm, &seq, 1, NRFX_PWM_FLAG_LOOP);
        k_msleep(50);

	    STACK_USAGE_PRINT("music_color_sync_thread", &music_color_sync_thread_data);
    }
}   

int music_color_sync_data_update(char *pcm_data_mono, uint16_t len)
{
    /*
     * if() take sema, dont wait
     *      Update global value, using memcpy
     *      release k_sem
     *      release sem value updated
     *      return 0
     * else
     *      return something debug/wrn
     */

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

    /*
     * Initiate rgb led, such that it can be controlled with 3 diff, values
     * Remember to remove init from led.c
     * Create thread(trpd, stack, stacksize, entry?, NULL,NULL,NULL, prio, null no wait)
     * Name of thread
     * Check error (not ERR_CHK since its not a critical feature or?)
     * 
     */

    int ret;

    static nrfx_pwm_config_t const pwm_cfg = 
    {
        .output_pins =
        {
            7,
            25,
            26,
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