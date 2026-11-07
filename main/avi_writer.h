#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* Minimal AVI 1.0 writer: one MJPEG video stream ("00dc") and one 16-bit PCM
 * audio stream ("01wb"), with an idx1 index. Frame rate and lengths in the
 * headers are patched with the measured values when the file is closed, so a
 * file with dropped frames still plays in sync with its audio. Files must stay
 * below 4 GB (FAT32); the recorder starts a new file long before that. */

typedef struct avi_writer avi_writer_t;

typedef struct {
    uint16_t width, height;
    uint32_t audio_rate;
    uint16_t audio_channels;
    void *io_buf;           /* stdio buffer (NULL: libc default) */
    size_t io_buf_size;
} avi_config_t;

esp_err_t avi_open(const char *path, const avi_config_t *cfg, avi_writer_t **out);
esp_err_t avi_write_video(avi_writer_t *avi, const void *jpeg, size_t len);
esp_err_t avi_write_audio(avi_writer_t *avi, const void *pcm, size_t len);

/* Writes the index and patches the headers. duration_us is the time between
 * the first and the last video frame plus one frame period. The writer is
 * freed even if writing fails. */
esp_err_t avi_close(avi_writer_t *avi, int64_t duration_us);

uint32_t avi_video_frames(const avi_writer_t *avi);
uint64_t avi_bytes(const avi_writer_t *avi);
