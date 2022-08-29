
#include <zephyr/kernel.h>

#if (CONFIG_MUSIC_COLOR_SYNC)
#define SUB_BASS_RANGE 1
#define BASS_RANGE 3
#define LOW_MID_RANGE 6
#define MID_RANGE 22
#define HIGH_MID_RANGE 43
#define PRESENCE_RANGE 64
#define BRILLIANCE_RANGE 256
#else
#endif

#define NRFX_PWM_ENABLED 1
#define NRFX_PWM0_ENABLED 1 


/**
 * @brief Updates data
 * 
 * @param pcm_data_mono 
 * @param len 
 * @return int 
 */
int music_color_sync_data_update(char *pcm_data_mono, uint16_t len);

/**
 * @brief Initialize the music color synchronization
 * 
 * @return int 
 */
int music_color_sync_init();
