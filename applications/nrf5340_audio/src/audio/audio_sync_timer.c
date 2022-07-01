/*************************************************************************************************/
/*
 *  Copyright (c) 2021, PACKETCRAFT, INC.
 *  All rights reserved.
 */
/*************************************************************************************************/

/*
 * Redistribution and use of the Audio subsystem for nRF5340 Software, in binary
 * and source code forms, with or without modification, are permitted provided
 * that the following conditions are met:
 *
 * 1. Redistributions of source code form must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer.
 *
 * 2. Redistributions in binary code form, except as embedded into a Nordic
 *    Semiconductor ASA nRF53 chip or a software update for such product,
 *    must reproduce the above copyright notice, this list of conditions
 *    and the following disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. Neither the name of Packetcraft, Inc. nor Nordic Semiconductor ASA nor
 *    the names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA nRF53 chip.
 *
 * 5. Any software provided in binary or source code form under this license
 *    must not be reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY PACKETCRAFT, INC. AND NORDIC SEMICONDUCTOR ASA
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE HEREBY DISCLAIMED. IN NO EVENT SHALL PACKETCRAFT, INC.,
 * NORDIC SEMICONDUCTOR ASA, OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "audio_sync_timer.h"

#include <zephyr.h>
#include <zephyr/kernel.h>
#include <init.h>
#include <nrfx_timer.h>
#include <nrfx_dppi.h>
#include <nrfx_i2s.h>
#include <nrfx_ipc.h>
#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>



#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_sync_timer, CONFIG_LOG_AUDIO_SYNC_TIMER_LEVEL);

#define AUDIO_SYNC_TIMER_INSTANCE 1

#define AUDIO_SYNC_TIMER_I2S_FRAME_START_EVT_CAPTURE_CHANNEL 0
#define AUDIO_SYNC_TIMER_CURR_TIME_CAPTURE_CHANNEL 1

#define AUDIO_SYNC_TIMER_I2S_FRAME_START_EVT_CAPTURE NRF_TIMER_TASK_CAPTURE0

#define AUDIO_SYNC_TIMER_NET_APP_IPC_EVT NRF_IPC_EVENT_RECEIVE_4

static const nrfx_timer_t timer_instance = NRFX_TIMER_INSTANCE(AUDIO_SYNC_TIMER_INSTANCE);

#define BLINKLED DT_GPIO_PIN(DT_ALIAS(led6), gpios)

static uint8_t dppi_channel_timer_clear;
static uint8_t dppi_channel_i2s_frame_start;
static uint8_t dppi_channel_gpiote_clear;
static uint8_t dppi_channel_gpiote_set;

static nrfx_timer_config_t cfg = { .frequency = NRF_TIMER_FREQ_1MHz,
				   .mode = NRF_TIMER_MODE_TIMER,
				   .bit_width = NRF_TIMER_BIT_WIDTH_32,
				   .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
				   .p_context = NULL };

static void event_handler(nrf_timer_event_t event_type, void *ctx)
{	
}


void sync_led_1_compare_time_set_update(uint32_t toggle_time_us){
	nrfx_timer_compare(&timer_instance, NRF_TIMER_CC_CHANNEL3 , toggle_time_us, true);
}

void sync_led_1_compare_time_clear_update(uint32_t toggle_time_us){
	nrfx_timer_compare(&timer_instance, NRF_TIMER_CC_CHANNEL2 , toggle_time_us, true);
}

static void ppi_led_1_blink_init(void){

	//Connected the irq handler for timer 1
	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(timer1)),
	0,	nrfx_isr, nrfx_timer_1_irq_handler, 0);

	nrfx_err_t ret;
	uint32_t timer_set_event_addr;
	uint32_t gpiote_set_task_addr;
	uint32_t timer_clear_event_addr;
	uint32_t gpiote_clear_task_addr;

	//Check if GPIOTE is initiated, if not initiate GPIOTE
	if(nrfx_gpiote_is_init()){LOG_INF("GPIOTE is init");}
	else{nrfx_gpiote_init(0);}

	// //GPIOTE out configs for toggle led 1
	nrfx_gpiote_out_config_t const gpiote_cfg = NRFX_GPIOTE_CONFIG_OUT_TASK_LOW;
	nrfx_gpiote_out_init(BLINKLED, &gpiote_cfg);
	nrfx_gpiote_out_task_enable(BLINKLED);

	//Allocating DPPI channels
	ret = nrfx_dppi_channel_alloc(&dppi_channel_gpiote_set);
	if(ret - NRFX_ERROR_BASE_NUM){
		LOG_ERR("error_set_channel_alloc: %d", ret);
	}
	ret = nrfx_dppi_channel_alloc(&dppi_channel_gpiote_clear);
	if(ret - NRFX_ERROR_BASE_NUM){
		LOG_ERR("error_ppi_channel_alloc: %d", ret);
	}

	//Getting addresses for events and tasks
	timer_set_event_addr = nrfx_timer_compare_event_address_get(&timer_instance, NRF_TIMER_CC_CHANNEL2);
	gpiote_set_task_addr = nrfx_gpiote_set_task_addr_get(BLINKLED);
	timer_clear_event_addr = nrfx_timer_compare_event_address_get(&timer_instance, NRF_TIMER_CC_CHANNEL3);
	gpiote_clear_task_addr = nrfx_gpiote_clr_task_addr_get(BLINKLED);

	//Set up endpoints for DPPI channels
	nrfx_gppi_channel_endpoints_setup(dppi_channel_gpiote_set,
		timer_set_event_addr,
		gpiote_set_task_addr	
	);
	nrfx_gppi_channel_endpoints_setup(dppi_channel_gpiote_clear,
		timer_clear_event_addr,
		gpiote_clear_task_addr	
	);

	//Enable DPPI channels
	ret = nrfx_dppi_channel_enable(dppi_channel_gpiote_clear);
		if(ret - NRFX_ERROR_BASE_NUM){
			LOG_ERR("error_DPPI_channel");
		}
	ret = nrfx_dppi_channel_enable(dppi_channel_gpiote_set);
	if(ret - NRFX_ERROR_BASE_NUM){
		LOG_ERR("error_set_channel");
	}

	//Check if DPPI channel is enabled
	if (nrfx_gppi_channel_check(dppi_channel_gpiote_clear)){
	LOG_INF("DPPI channel for GPIOTE clear configured");
	}
	if (nrfx_gppi_channel_check(dppi_channel_gpiote_set)){
	LOG_INF("DPPI channel for GPIOTE set configured");
	}
}

/**
 * @brief Initialize audio sync timer
 *
 * @note Clearing of the nRF5340 APP core sync
 * timer is initialized here. The sync timers on
 * APP core and NET core are cleared at exactly
 * the same time using an IPC signal sent from
 * the NET core. This makes the two timers
 * synchronized.
 *
 * @param unused Unused
 *
 * @return 0 if successful, error otherwise
 */
static int audio_sync_timer_init(const struct device *unused)
{
	nrfx_err_t ret;

	ARG_UNUSED(unused);

	ret = nrfx_timer_init(&timer_instance, &cfg, event_handler);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx timer init error - Return value: %d", ret);
		return ret;
	}	

	ppi_led_1_blink_init();
	nrfx_timer_enable(&timer_instance);


	/* Initialize capturing of I2S frame start event timestamps */
	ret = nrfx_dppi_channel_alloc(&dppi_channel_i2s_frame_start);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel alloc error (I2S frame start) - Return value: %d", ret);
		return ret;
	}
	nrf_timer_subscribe_set(timer_instance.p_reg, AUDIO_SYNC_TIMER_I2S_FRAME_START_EVT_CAPTURE,
				dppi_channel_i2s_frame_start);
	nrf_i2s_publish_set(NRF_I2S0, NRF_I2S_EVENT_FRAMESTART, dppi_channel_i2s_frame_start);
	ret = nrfx_dppi_channel_enable(dppi_channel_i2s_frame_start);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel enable error (I2S frame start) - Return value: %d", ret);
		return ret;
	}

	/* Initialize functionality for synchronization between APP and NET core */
	ret = nrfx_dppi_channel_alloc(&dppi_channel_timer_clear);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel alloc error (timer clear) - Return value: %d", ret);
		return ret;
	}
	nrf_ipc_publish_set(NRF_IPC, AUDIO_SYNC_TIMER_NET_APP_IPC_EVT, dppi_channel_timer_clear);
	nrf_timer_subscribe_set(timer_instance.p_reg, NRF_TIMER_TASK_CLEAR,
				dppi_channel_timer_clear);
	ret = nrfx_dppi_channel_enable(dppi_channel_timer_clear);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel enable error (timer clear) - Return value: %d", ret);
		return ret;
	}

	LOG_INF("Audio sync timer initialized");

	return 0;
}

uint32_t audio_sync_timer_i2s_frame_start_ts_get(void)
{
	return nrfx_timer_capture_get(&timer_instance,
				      AUDIO_SYNC_TIMER_I2S_FRAME_START_EVT_CAPTURE_CHANNEL);
}

uint32_t audio_sync_timer_curr_time_get(void)
{
	return nrfx_timer_capture(&timer_instance, AUDIO_SYNC_TIMER_CURR_TIME_CAPTURE_CHANNEL);
}


SYS_INIT(audio_sync_timer_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
