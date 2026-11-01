#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "frame_mbox.h"

/* External UVC camera on the USB-A host port. MJPEG is decoded by the hardware
 * JPEG engine into RGB565 and published through a mailbox.
 *
 * The camera mode is (re)negotiated at start-up and every time a camera is
 * plugged in: the supported modes are read from the camera, the best match for
 * APP_UVC_W x APP_UVC_H @ APP_UVC_FPS is chosen and the stream is opened with it. */

typedef struct {
    uint32_t rx_frames;     /* MJPEG frames received from the camera */
    uint32_t rx_bytes;      /* total MJPEG bytes received */
    uint32_t decode_us;     /* accumulated JPEG decode time */
    uint32_t rx_dropped;    /* dropped because the decoder was behind */
    uint32_t decoded;
    uint32_t decode_errors;
    uint32_t no_buffer;     /* no free output buffer (display is behind) */
    uint32_t stall_restarts; /* stalls detected by the watchdog */
    uint32_t power_cycles;  /* USB-A power cycles (stall, wedged camera or requested) */
    bool streaming;
    uint8_t luma;           /* mean luma of the last decoded frame, sparse sample (APP_UVC_CTRL_TEST) */
    uint16_t w, h;          /* current mode (valid while streaming) */
} cam_uvc_stats_t;

/* Start the USB host + UVC driver and allocate buffers. */
esp_err_t cam_uvc_init(void);

frame_mbox_t cam_uvc_mbox(void);

/* Start the connection manager and decode tasks. */
esp_err_t cam_uvc_start(void);

void cam_uvc_get_stats(cam_uvc_stats_t *out);

/* Power-cycle the camera from the manager task, APP_UVC_APPLY_DELAY_MS after the
 * last request. It drops off the bus, re-enumerates and is probed and opened
 * again (about 2-3 s without video). */
void cam_uvc_request_power_cycle(void);
