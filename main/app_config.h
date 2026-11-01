#pragma once

/* ---- Display ------------------------------------------------------------
 * The Tab5 panel is physically portrait (720x1280). We present a landscape
 * 1280x720 canvas split into two panes by rotating every image with the PPA.
 * Flip APP_DISPLAY_ROT_DEG between 90 and 270 if the picture is upside down. */
#define APP_LCD_PHYS_W          720
#define APP_LCD_PHYS_H          1280
#define APP_DISPLAY_ROT_DEG     90      /* 90 or 270 (CCW) */
#define APP_DISPLAY_BENCH       0       /* run the PPA benchmark once at start-up (profiling) */
#define APP_PANE_STALE_MS       1000    /* no new frame for this long -> "no signal" colour */

/* ---- Built-in camera (SC202CS, MIPI-CSI) -------------------------------- */
#define APP_CAM_BUILTIN_W       1280
#define APP_CAM_BUILTIN_H       720
#define APP_CAM_BUILTIN_BUFS    4
#define APP_CAM_BUILTIN_ROT_DEG 0       /* extra rotation of the image (0/90/180/270, CCW) */
#define APP_CAM_CONTROL_TEST    0       /* temporary live-control verification */
#define APP_CAM_CCM_TEST        0       /* temporary CCM persistence verification */

/* ---- Task settings (FreeRTOS) ------------------------------------------- */
#define APP_TASK_CAM_BUILTIN_PRIO   5
#define APP_TASK_CAM_BUILTIN_STACK  4096
#define APP_TASK_DISPLAY_PRIO       6
#define APP_TASK_DISPLAY_STACK      4096
#define APP_TASK_STATS_PERIOD_MS    5000

/* ---- Touch input --------------------------------------------------------- */
#define APP_TOUCH_POLL_MS           20
#define APP_TOUCH_LOG_PERIOD_MS     200
#define APP_TASK_TOUCH_PRIO         3
#define APP_TASK_TOUCH_STACK        3072

/* ---- Camera control bar ---------------------------------------------------- */
#define APP_UI_HIGHLIGHT_MS         150     /* tapped button stays highlighted this long */
#define APP_TASK_UI_PRIO            3
#define APP_TASK_UI_STACK           4096

/* ---- External UVC camera (USB-A host port) --------------------------------
 * The wanted mode is requested at start-up and on every connect; if the camera
 * does not offer it exactly, the closest MJPEG mode (<= MAX size) is used. */
#define APP_UVC_W               640
#define APP_UVC_H               480
#define APP_UVC_FPS             30
#define APP_UVC_MAX_W           1280    /* decode buffers are sized for this */
#define APP_UVC_MAX_H           720
#define APP_UVC_FRAME_BUFS      3       /* MJPEG frame buffers inside the UVC driver */
#define APP_UVC_OUT_BUFS        3       /* decoded RGB565 buffers */
#define APP_UVC_ROT_DEG         0       /* extra rotation of the image (0/90/180/270, CCW) */
#define APP_UVC_RGB_ORDER       JPEG_DEC_RGB_ELEMENT_ORDER_BGR  /* switch to ..._RGB if red/blue look swapped */
#define APP_UVC_RESCAN_MS       2000    /* while no camera is streaming, look for one this often */
#define APP_UVC_CTRL_PROBE      1       /* query Processing Unit controls (GET only) on connect */
#define APP_UVC_STALL_MS        3000    /* streaming but no frame this long -> power-cycle the camera */
#define APP_UVC_WATCHDOG_POLL_MS 500
#define APP_UVC_OPEN_RETRIES    3       /* failed opens of a present camera before a power cycle */
#define APP_UVC_POWER_OFF_MS    1000    /* USB-A 5 V off time of a power cycle */
#define APP_UVC_APPLY_DELAY_MS  1500    /* quiet time after the last tap before a settings restart */
#define APP_UVC_STALL_TEST      0       /* temporary: stop the stream once after 20 s to test recovery */
#define APP_UVC_CTRL_TEST       0       /* temporary: sweep UVC controls and measure the effect */

#define APP_TASK_UVC_DRIVER_PRIO    11  /* must be above the tasks using the driver */
#define APP_TASK_UVC_DECODE_PRIO    5
#define APP_TASK_UVC_DECODE_STACK   6144
#define APP_TASK_UVC_MGR_PRIO       4
#define APP_TASK_UVC_MGR_STACK      6144
