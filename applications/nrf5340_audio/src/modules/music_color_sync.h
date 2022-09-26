
#ifndef _MUSIC_COLOR_SYNC_H_
#define _MUSIC_COLOR_SYNC_H_

#include <zephyr/kernel.h>

/**@brief Updates decoded pcm data used for FFT 
 * 
 * @param[in] pcm_data_mono pointer to decoded pcm data (mono)
 * @param[in] len Size of decoded data
 * 
 * @return 0 on success, error otherwise
 */
int music_color_sync_data_update(char *pcm_data_mono, uint16_t len);

/**@brief Initialize the music color synchronization
 * 
 * @return 0 if success, error otherwise
 */
int music_color_sync_init();

#endif /* _MUSIC_COLOR_SYNC_H */