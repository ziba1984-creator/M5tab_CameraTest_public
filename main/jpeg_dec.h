#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* Hardware JPEG decoder wrapper (ESP32-P4): MJPEG frame -> RGB565. */

typedef struct {
    uint16_t w, h;      /* visible size */
    uint16_t stride;    /* buffer row length in pixels (width rounded up to 16) */
} jpeg_dec_result_t;

esp_err_t jpeg_dec_init(void);

/* Allocate an output buffer the decoder can write to (cache/DMA aligned PSRAM). */
void *jpeg_dec_alloc_output(size_t size, size_t *allocated);

/* Largest output (bytes) needed for a w x h picture. */
size_t jpeg_dec_output_size(uint16_t w, uint16_t h);

/* Decode one JPEG. `out` must come from jpeg_dec_alloc_output(). Not re-entrant. */
esp_err_t jpeg_dec_decode(const uint8_t *jpeg, size_t len, uint8_t *out, size_t out_cap,
                          jpeg_dec_result_t *result);
