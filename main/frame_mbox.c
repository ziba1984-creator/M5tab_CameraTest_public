#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "frame_mbox.h"

struct frame_mbox {
    SemaphoreHandle_t lock;
    video_frame_t pending;
    bool has_pending;
    frame_release_fn release;
    void *release_ctx;
    EventGroupHandle_t evt;
    EventBits_t bit;
    frame_mbox_stats_t stats;
};

frame_mbox_t frame_mbox_create(frame_release_fn release, void *release_ctx)
{
    frame_mbox_t mb = calloc(1, sizeof(*mb));
    if (!mb) {
        return NULL;
    }
    mb->lock = xSemaphoreCreateMutex();
    if (!mb->lock) {
        free(mb);
        return NULL;
    }
    mb->release = release;
    mb->release_ctx = release_ctx;
    return mb;
}

void frame_mbox_set_notify(frame_mbox_t mb, EventGroupHandle_t evt, EventBits_t bit)
{
    xSemaphoreTake(mb->lock, portMAX_DELAY);
    mb->evt = evt;
    mb->bit = bit;
    xSemaphoreGive(mb->lock);
}

void frame_mbox_publish(frame_mbox_t mb, const video_frame_t *frame)
{
    video_frame_t old = {0};
    bool had_old;

    xSemaphoreTake(mb->lock, portMAX_DELAY);
    had_old = mb->has_pending;
    if (had_old) {
        old = mb->pending;
        mb->stats.dropped++;
    }
    mb->pending = *frame;
    mb->has_pending = true;
    mb->stats.published++;
    EventGroupHandle_t evt = mb->evt;
    EventBits_t bit = mb->bit;
    xSemaphoreGive(mb->lock);

    if (had_old) {
        mb->release(&old, mb->release_ctx);
    }
    if (evt) {
        xEventGroupSetBits(evt, bit);
    }
}

bool frame_mbox_take(frame_mbox_t mb, video_frame_t *out)
{
    bool ok;

    xSemaphoreTake(mb->lock, portMAX_DELAY);
    ok = mb->has_pending;
    if (ok) {
        *out = mb->pending;
        mb->has_pending = false;
    }
    xSemaphoreGive(mb->lock);
    return ok;
}

void frame_mbox_release(frame_mbox_t mb, const video_frame_t *frame)
{
    mb->release(frame, mb->release_ctx);
}

void frame_mbox_get_stats(frame_mbox_t mb, frame_mbox_stats_t *out)
{
    xSemaphoreTake(mb->lock, portMAX_DELAY);
    *out = mb->stats;
    xSemaphoreGive(mb->lock);
}
