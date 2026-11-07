#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Records the built-in camera (hardware JPEG, MJPEG) and the microphone
 * (16-bit PCM) into AVI files on the microSD card (APP_REC_* in app_config.h). */

typedef enum {
    RECORDER_IDLE,
    RECORDER_RECORDING,
    RECORDER_NO_CARD,       /* last start found no usable card */
    RECORDER_ERROR,         /* last recording stopped on a write error */
} recorder_state_t;

typedef struct {
    recorder_state_t state;
    uint32_t file_no;           /* current or last file, Rnnnn.AVI */
    uint32_t seconds;           /* since start of the current recording */
    uint32_t frames;            /* video frames written, whole recording */
    uint32_t frames_dropped;    /* no free slot or encode failure */
    uint32_t audio_chunks;
    uint32_t audio_dropped;
    uint64_t bytes;             /* size of the current or last file */
    uint32_t encode_us_max;
    uint32_t write_ms_max;      /* longest single chunk write */
} recorder_stats_t;

/* Mounts the card (never formats it), sets up the encoder, buffers and tasks.
 * Failing to mount is not an error: start retries it. */
esp_err_t recorder_init(void);

/* Called from the UI task only. */
esp_err_t recorder_start(void);
void recorder_stop(void);

void recorder_get_stats(recorder_stats_t *out);
