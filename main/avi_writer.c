#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "avi_writer.h"

static const char *TAG = "avi";

/* Fixed header layout (offsets in the file):
 *   0 RIFF size 'AVI '
 *  12 LIST size 'hdrl'
 *  24   avih (56)
 *  88   LIST size 'strl'   video: strh (56) at 100, strf BITMAPINFOHEADER (40) at 164
 * 212   LIST size 'strl'   audio: strh (56) at 224, strf WAVEFORMATEX (18) at 288
 * 314 LIST size 'movi'
 * 326   chunks...
 * then idx1. Offsets in idx1 are relative to the 'movi' fourcc at 322. */
#define HDR_BYTES       326
#define MOVI_FOURCC_AT  322

#define AVIF_HASINDEX       0x10
#define AVIF_ISINTERLEAVED  0x100
#define AVIIF_KEYFRAME      0x10

typedef struct {
    uint32_t ckid, flags, offset, size;
} idx_entry_t;

struct avi_writer {
    FILE *f;
    avi_config_t cfg;
    uint64_t pos;               /* bytes written so far (the file position) */
    idx_entry_t *idx;
    size_t idx_n, idx_cap;
    uint32_t video_frames;
    uint64_t audio_bytes;
    uint32_t max_chunk;
    bool failed;
};

static void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static void put4cc(uint8_t *p, const char *s) { memcpy(p, s, 4); }
static uint32_t fourcc(const char *s) { return s[0] | s[1] << 8 | s[2] << 16 | (uint32_t)s[3] << 24; }

/* final: the movi list ends at pos and the idx1 follows it. The frame rate is
 * the measured one when duration_us is usable, else the nominal 30 fps. */
static void build_header(const avi_writer_t *avi, uint8_t *h, bool final, int64_t duration_us)
{
    const avi_config_t *c = &avi->cfg;
    uint32_t block = c->audio_channels * 2;
    uint32_t rate_num = 30, rate_den = 1, us_per_frame = 33333;
    if (duration_us >= 1000 && avi->video_frames > 0) {
        rate_num = avi->video_frames * 1000;            /* frames per (duration in ms) */
        rate_den = (uint32_t)(duration_us / 1000);
        us_per_frame = (uint32_t)(duration_us / avi->video_frames);
    }
    uint32_t movi_size = (uint32_t)(avi->pos - MOVI_FOURCC_AT);
    uint32_t riff_size = (uint32_t)(avi->pos + (final ? 8 + avi->idx_n * 16 : 0) - 8);

    memset(h, 0, HDR_BYTES);
    put4cc(h + 0, "RIFF"); put32(h + 4, riff_size); put4cc(h + 8, "AVI ");
    put4cc(h + 12, "LIST"); put32(h + 16, 294); put4cc(h + 20, "hdrl");

    uint8_t *a = h + 24;
    put4cc(a, "avih"); put32(a + 4, 56);
    put32(a + 8, us_per_frame);
    put32(a + 12, (uint32_t)(us_per_frame ? (uint64_t)avi->max_chunk * 1000000 / us_per_frame : 0));
    put32(a + 20, AVIF_HASINDEX | AVIF_ISINTERLEAVED);
    put32(a + 24, avi->video_frames);
    put32(a + 32, 2);                                   /* streams */
    put32(a + 36, avi->max_chunk);
    put32(a + 40, c->width);
    put32(a + 44, c->height);

    uint8_t *v = h + 88;
    put4cc(v, "LIST"); put32(v + 4, 116); put4cc(v + 8, "strl");
    put4cc(v + 12, "strh"); put32(v + 16, 56);
    put4cc(v + 20, "vids"); put4cc(v + 24, "MJPG");
    put32(v + 40, rate_den);                            /* dwScale */
    put32(v + 44, rate_num);                            /* dwRate */
    put32(v + 52, avi->video_frames);                   /* dwLength */
    put32(v + 56, avi->max_chunk);
    put32(v + 60, 0xffffffff);                          /* dwQuality: default */
    put16(v + 72, c->width); put16(v + 74, c->height);  /* rcFrame right, bottom */
    put4cc(v + 76, "strf"); put32(v + 80, 40);
    put32(v + 84, 40);
    put32(v + 88, c->width); put32(v + 92, c->height);
    put16(v + 96, 1); put16(v + 98, 24);
    put4cc(v + 100, "MJPG");
    put32(v + 104, (uint32_t)c->width * c->height * 3);

    uint8_t *s = h + 212;
    put4cc(s, "LIST"); put32(s + 4, 94); put4cc(s + 8, "strl");
    put4cc(s + 12, "strh"); put32(s + 16, 56);
    put4cc(s + 20, "auds");
    put32(s + 40, block);                               /* dwScale */
    put32(s + 44, c->audio_rate * block);               /* dwRate: bytes per second */
    put32(s + 52, (uint32_t)(avi->audio_bytes / block)); /* dwLength in blocks */
    put32(s + 56, c->audio_rate * block / 10);
    put32(s + 60, 0xffffffff);
    put32(s + 64, block);                               /* dwSampleSize */
    put4cc(s + 76, "strf"); put32(s + 80, 18);
    put16(s + 84, 1);                                   /* PCM */
    put16(s + 86, c->audio_channels);
    put32(s + 88, c->audio_rate);
    put32(s + 92, c->audio_rate * block);
    put16(s + 96, block);
    put16(s + 98, 16);

    put4cc(h + 314, "LIST"); put32(h + 318, movi_size); put4cc(h + 322, "movi");
}

static esp_err_t write_all(avi_writer_t *avi, const void *data, size_t len)
{
    if (len && fwrite(data, 1, len, avi->f) != len) {
        avi->failed = true;
        return ESP_FAIL;
    }
    avi->pos += len;
    return ESP_OK;
}

static esp_err_t write_chunk(avi_writer_t *avi, const char *id, const void *data, size_t len, uint32_t flags)
{
    ESP_RETURN_ON_FALSE(!avi->failed, ESP_FAIL, TAG, "file already failed");
    if (avi->idx_n == avi->idx_cap) {
        size_t cap = avi->idx_cap ? avi->idx_cap * 2 : 4096;
        idx_entry_t *grown = heap_caps_realloc(avi->idx, cap * sizeof(idx_entry_t), MALLOC_CAP_SPIRAM);
        ESP_RETURN_ON_FALSE(grown, ESP_ERR_NO_MEM, TAG, "no memory for the index");
        avi->idx = grown;
        avi->idx_cap = cap;
    }
    avi->idx[avi->idx_n++] = (idx_entry_t) {
        .ckid = fourcc(id), .flags = flags, .offset = (uint32_t)(avi->pos - MOVI_FOURCC_AT), .size = len,
    };
    uint8_t head[8];
    put4cc(head, id);
    put32(head + 4, len);
    static const uint8_t pad = 0;
    ESP_RETURN_ON_ERROR(write_all(avi, head, sizeof(head)), TAG, "write failed");
    ESP_RETURN_ON_ERROR(write_all(avi, data, len), TAG, "write failed");
    if (len & 1) {
        ESP_RETURN_ON_ERROR(write_all(avi, &pad, 1), TAG, "write failed");
    }
    if (len > avi->max_chunk) {
        avi->max_chunk = len;
    }
    return ESP_OK;
}

esp_err_t avi_open(const char *path, const avi_config_t *cfg, avi_writer_t **out)
{
    avi_writer_t *avi = calloc(1, sizeof(*avi));
    ESP_RETURN_ON_FALSE(avi, ESP_ERR_NO_MEM, TAG, "no memory");
    avi->cfg = *cfg;
    avi->f = fopen(path, "wb");
    if (!avi->f) {
        free(avi);
        ESP_LOGE(TAG, "cannot create %s", path);
        return ESP_FAIL;
    }
    if (cfg->io_buf) {
        setvbuf(avi->f, cfg->io_buf, _IOFBF, cfg->io_buf_size);
    }
    uint8_t h[HDR_BYTES];
    avi->pos = HDR_BYTES;   /* the provisional header describes an empty movi list */
    build_header(avi, h, false, 0);
    avi->pos = 0;
    if (write_all(avi, h, sizeof(h)) != ESP_OK) {
        fclose(avi->f);
        free(avi);
        return ESP_FAIL;
    }
    *out = avi;
    return ESP_OK;
}

esp_err_t avi_write_video(avi_writer_t *avi, const void *jpeg, size_t len)
{
    ESP_RETURN_ON_ERROR(write_chunk(avi, "00dc", jpeg, len, AVIIF_KEYFRAME), TAG, "video chunk");
    avi->video_frames++;
    return ESP_OK;
}

esp_err_t avi_write_audio(avi_writer_t *avi, const void *pcm, size_t len)
{
    ESP_RETURN_ON_ERROR(write_chunk(avi, "01wb", pcm, len, 0), TAG, "audio chunk");
    avi->audio_bytes += len;
    return ESP_OK;
}

esp_err_t avi_close(avi_writer_t *avi, int64_t duration_us)
{
    esp_err_t err = avi->failed ? ESP_FAIL : ESP_OK;
    if (err == ESP_OK) {
        uint8_t head[8];
        put4cc(head, "idx1");
        put32(head + 4, avi->idx_n * sizeof(idx_entry_t));
        err = write_all(avi, head, sizeof(head));
        uint64_t movi_end = avi->pos - sizeof(head);
        for (size_t i = 0; err == ESP_OK && i < avi->idx_n; i++) {
            uint8_t e[16];
            put32(e, avi->idx[i].ckid);
            put32(e + 4, avi->idx[i].flags);
            put32(e + 8, avi->idx[i].offset);
            put32(e + 12, avi->idx[i].size);
            err = write_all(avi, e, sizeof(e));
        }
        if (err == ESP_OK) {
            uint64_t end = avi->pos;
            avi->pos = movi_end;    /* build_header derives the movi size from pos */
            uint8_t h[HDR_BYTES];
            build_header(avi, h, true, duration_us);
            avi->pos = end;
            if (fseek(avi->f, 0, SEEK_SET) != 0 || fwrite(h, 1, sizeof(h), avi->f) != sizeof(h)) {
                err = ESP_FAIL;
            }
        }
    }
    if (fclose(avi->f) != 0) {
        err = ESP_FAIL;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "closing the file failed (card full or removed?)");
    }
    heap_caps_free(avi->idx);
    free(avi);
    return err;
}

uint32_t avi_video_frames(const avi_writer_t *avi)
{
    return avi->video_frames;
}

uint64_t avi_bytes(const avi_writer_t *avi)
{
    return avi->pos;
}
