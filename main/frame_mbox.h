#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/* One RGB565 frame. The memory is owned by the producer; the consumer must
 * hand it back with frame_mbox_release() when it is done with it. */
typedef struct {
    uint8_t *data;
    uint16_t w, h;      /* visible size in pixels */
    uint16_t stride;    /* row length of the buffer in pixels (>= w) */
    void *priv;         /* producer's private cookie (e.g. buffer index) */
} video_frame_t;

typedef void (*frame_release_fn)(const video_frame_t *frame, void *ctx);

typedef struct frame_mbox *frame_mbox_t;

typedef struct {
    uint32_t published;
    uint32_t dropped;   /* superseded before the consumer took them */
} frame_mbox_stats_t;

/* "Latest frame wins" mailbox between a producer task and the display task. */
frame_mbox_t frame_mbox_create(frame_release_fn release, void *release_ctx);

/* Consumer is woken through an event bit whenever a new frame is published. */
void frame_mbox_set_notify(frame_mbox_t mb, EventGroupHandle_t evt, EventBits_t bit);

/* Producer: hand over a frame. An older, untaken frame is released. */
void frame_mbox_publish(frame_mbox_t mb, const video_frame_t *frame);

/* Consumer: take the pending frame (non-blocking). Returns false if none. */
bool frame_mbox_take(frame_mbox_t mb, video_frame_t *out);

/* Consumer: give a taken frame back to its producer. */
void frame_mbox_release(frame_mbox_t mb, const video_frame_t *frame);

void frame_mbox_get_stats(frame_mbox_t mb, frame_mbox_stats_t *out);
