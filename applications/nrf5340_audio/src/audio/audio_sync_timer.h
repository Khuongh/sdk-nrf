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

#ifndef _AUDIO_SYNC_TIMER_H_
#define _AUDIO_SYNC_TIMER_H_

#include <stdint.h>

/**
 * @brief Get the I2S TX timestamp
 *
 * @return Captured value
 */
uint32_t audio_sync_timer_i2s_frame_start_ts_get(void);

/**
 * @brief Get the currrent timer value
 *
 * @return Captured value
 */
uint32_t audio_sync_timer_curr_time_get(void);

/**
 * @brief Updates set compare time in us
 * 
 * @param toggle_time_us New set value
 */
void sync_led_1_compare_time_set_update(uint32_t toggle_time_us);

/**
 * @brief Updates clr compare time in us
 * 
 * @param toggle_time_us New clr value
 */
void sync_led_1_compare_time_clear_update(uint32_t toggle_time_us);

/**
 * @brief Turns LED_APP_1_BLUE on
 * 
 */
void sync_led_1_on();

#endif /* _AUDIO_SYNC_TIMER_H_ */
