#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "frame_mbox.h"

/* Compositor: shows two camera panes side by side (landscape 1280x720) on the
 * portrait DSI panel. Uses the PPA for scale+rotate and flips two frame buffers. */

typedef enum {
    DISP_PANE_LEFT = 0,
    DISP_PANE_RIGHT,
    DISP_PANE_COUNT,
} disp_pane_t;

typedef struct {
    uint32_t flips;
    /* accumulated timings (us) and call counts, for profiling */
    uint32_t srm_calls, srm_us;
    uint32_t fill_calls, fill_us;
    uint32_t flip_call_us;      /* esp_lcd_panel_draw_bitmap() */
    uint32_t flip_wait_us;      /* waiting for the old buffer to leave the scan-out */
} display_comp_stats_t;

/* Bring up panel, backlight, PPA and frame buffers. */
esp_err_t display_comp_init(void);

/* Bind a frame source to a pane. rot_deg: extra CCW rotation of the image
 * (0/90/180/270). no_signal_rgb565: colour shown while no frames arrive. */
esp_err_t display_comp_attach(disp_pane_t pane, frame_mbox_t mbox, int rot_deg, uint16_t no_signal_rgb565);

/* Start the compositor task. */
esp_err_t display_comp_start(void);

/* Redraw on the next opportunity even if no camera frame arrives (UI change). */
void display_comp_request_redraw(void);

void display_comp_get_stats(display_comp_stats_t *out);
