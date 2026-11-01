#include <string.h>
#include "driver/jpeg_decode.h"
#include "esp_check.h"
#include "esp_log.h"
#include "app_config.h"
#include "jpeg_dec.h"

static const char *TAG = "jpeg_dec";

#define ALIGN16(x)  (((x) + 15) & ~15)

static jpeg_decoder_handle_t s_engine;
static uint8_t *s_in_buf;       /* the decoder needs its input in aligned, unencrypted PSRAM */
static size_t s_in_cap;

esp_err_t jpeg_dec_init(void)
{
    const jpeg_decode_engine_cfg_t cfg = {
        .intr_priority = 0,
        .timeout_ms = 100,
    };
    return jpeg_new_decoder_engine(&cfg, &s_engine);
}

void *jpeg_dec_alloc_output(size_t size, size_t *allocated)
{
    const jpeg_decode_memory_alloc_cfg_t cfg = { .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER };
    return jpeg_alloc_decoder_mem(size, &cfg, allocated);
}

size_t jpeg_dec_output_size(uint16_t w, uint16_t h)
{
    return (size_t)ALIGN16(w) * ALIGN16(h) * 2;
}

static esp_err_t ensure_input_capacity(size_t len)
{
    if (len <= s_in_cap) {
        return ESP_OK;
    }
    free(s_in_buf);
    s_in_buf = NULL;
    s_in_cap = 0;

    const jpeg_decode_memory_alloc_cfg_t cfg = { .buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER };
    size_t want = len + len / 4;    /* headroom so we do not realloc on every slightly bigger frame */
    size_t got = 0;
    s_in_buf = jpeg_alloc_decoder_mem(want, &cfg, &got);
    ESP_RETURN_ON_FALSE(s_in_buf, ESP_ERR_NO_MEM, TAG, "no memory for %u byte input buffer", (unsigned)want);
    s_in_cap = got;
    return ESP_OK;
}

esp_err_t jpeg_dec_decode(const uint8_t *jpeg, size_t len, uint8_t *out, size_t out_cap,
                          jpeg_dec_result_t *result)
{
    jpeg_decode_picture_info_t info;
    ESP_RETURN_ON_ERROR(jpeg_decoder_get_info(jpeg, len, &info), TAG, "bad JPEG header");

    size_t need = jpeg_dec_output_size(info.width, info.height);
    ESP_RETURN_ON_FALSE(need <= out_cap, ESP_ERR_NO_MEM, TAG, "%ux%u does not fit the output buffer",
                        (unsigned)info.width, (unsigned)info.height);
    ESP_RETURN_ON_ERROR(ensure_input_capacity(len), TAG, "input buffer");
    memcpy(s_in_buf, jpeg, len);

    const jpeg_decode_cfg_t cfg = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order = APP_UVC_RGB_ORDER,
        .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
    };
    uint32_t out_size = 0;
    ESP_RETURN_ON_ERROR(jpeg_decoder_process(s_engine, &cfg, s_in_buf, len, out, out_cap, &out_size),
                        TAG, "decode failed");

    result->w = info.width;
    result->h = info.height;
    result->stride = ALIGN16(info.width);
    return ESP_OK;
}
