#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_video_device.h"
#include "esp_video_isp_ioctl.h"
#include "linux/videodev2.h"
#include "bsp/m5stack_tab5.h"
#include "app_config.h"
#include "cam_builtin.h"

static const char *TAG = "cam_builtin";

typedef struct {
    int fd;
    int isp_fd;
    uint8_t *buf[APP_CAM_BUILTIN_BUFS];
    uint32_t buf_len[APP_CAM_BUILTIN_BUFS];
    uint16_t w, h;
    frame_mbox_t mbox;
} cam_ctx_t;

static cam_ctx_t s_cam = { .fd = -1, .isp_fd = -1 };

esp_err_t cam_builtin_ccm_get(cam_builtin_ccm_t *out)
{
    ESP_RETURN_ON_FALSE(out, ESP_ERR_INVALID_ARG, TAG, "null CCM output");
    ESP_RETURN_ON_FALSE(s_cam.isp_fd >= 0, ESP_ERR_INVALID_STATE, TAG, "ISP unavailable");

    esp_video_isp_ccm_t value = {0};
    struct v4l2_ext_control control = {
        .id = V4L2_CID_USER_ESP_ISP_CCM,
        .p_u8 = (uint8_t *)&value,
        .size = sizeof(value),
    };
    struct v4l2_ext_controls controls = {
        .ctrl_class = V4L2_CID_USER_CLASS,
        .count = 1,
        .controls = &control,
    };
    ESP_RETURN_ON_FALSE(ioctl(s_cam.isp_fd, VIDIOC_G_EXT_CTRLS, &controls) == 0,
                        ESP_FAIL, TAG, "CCM get failed");
    out->enable = value.enable;
    memcpy(out->matrix, value.matrix, sizeof(out->matrix));
    return ESP_OK;
}

esp_err_t cam_builtin_ccm_set(const cam_builtin_ccm_t *ccm)
{
    ESP_RETURN_ON_FALSE(ccm, ESP_ERR_INVALID_ARG, TAG, "null CCM input");
    ESP_RETURN_ON_FALSE(s_cam.isp_fd >= 0, ESP_ERR_INVALID_STATE, TAG, "ISP unavailable");

    esp_video_isp_ccm_t value = { .enable = ccm->enable };
    memcpy(value.matrix, ccm->matrix, sizeof(value.matrix));
    struct v4l2_ext_control control = {
        .id = V4L2_CID_USER_ESP_ISP_CCM,
        .p_u8 = (uint8_t *)&value,
        .size = sizeof(value),
    };
    struct v4l2_ext_controls controls = {
        .ctrl_class = V4L2_CID_USER_CLASS,
        .count = 1,
        .controls = &control,
    };
    /* The esp_video driver does not copy matrix[] while disabling CCM. Prime
     * it once while enabled so a later re-enable cannot resurrect stale data. */
    if (!ccm->enable) {
        value.enable = true;
        ESP_RETURN_ON_FALSE(ioctl(s_cam.isp_fd, VIDIOC_S_EXT_CTRLS, &controls) == 0,
                            ESP_FAIL, TAG, "CCM restore failed");
        value.enable = false;
    }
    ESP_RETURN_ON_FALSE(ioctl(s_cam.isp_fd, VIDIOC_S_EXT_CTRLS, &controls) == 0,
                        ESP_FAIL, TAG, "CCM set failed");
    return ESP_OK;
}

typedef struct {
    int *fd;
    uint32_t id;
    const char *name;
} control_desc_t;

static const control_desc_t *control_desc(cam_builtin_ctrl_t control)
{
    static const control_desc_t table[CAM_BUILTIN_CTRL_COUNT] = {
        [CAM_BUILTIN_CTRL_BRIGHTNESS] = { &s_cam.isp_fd, V4L2_CID_BRIGHTNESS, "brightness" },
        [CAM_BUILTIN_CTRL_CONTRAST] = { &s_cam.isp_fd, V4L2_CID_CONTRAST, "contrast" },
        [CAM_BUILTIN_CTRL_SATURATION] = { &s_cam.isp_fd, V4L2_CID_SATURATION, "saturation" },
        [CAM_BUILTIN_CTRL_HUE] = { &s_cam.isp_fd, V4L2_CID_HUE, "hue" },
        [CAM_BUILTIN_CTRL_EXPOSURE] = { &s_cam.fd, V4L2_CID_EXPOSURE, "exposure" },
        [CAM_BUILTIN_CTRL_GAIN] = { &s_cam.fd, V4L2_CID_GAIN, "gain" },
    };
    return control >= 0 && control < CAM_BUILTIN_CTRL_COUNT ? &table[control] : NULL;
}

static esp_err_t control_get(const control_desc_t *desc, int32_t *value)
{
    if (!desc || *desc->fd < 0) {
        return ESP_ERR_INVALID_STATE;
    }
    struct v4l2_ext_control control = { .id = desc->id };
    struct v4l2_ext_controls controls = {
        .ctrl_class = V4L2_CTRL_ID2CLASS(desc->id),
        .count = 1,
        .controls = &control,
    };
    if (ioctl(*desc->fd, VIDIOC_G_EXT_CTRLS, &controls) != 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    *value = control.value;
    return ESP_OK;
}

esp_err_t cam_builtin_ctrl_query(cam_builtin_ctrl_t control, cam_builtin_ctrl_info_t *out)
{
    const control_desc_t *desc = control_desc(control);
    ESP_RETURN_ON_FALSE(desc && out, ESP_ERR_INVALID_ARG, TAG, "bad control");
    ESP_RETURN_ON_FALSE(*desc->fd >= 0, ESP_ERR_INVALID_STATE, TAG, "%s unavailable", desc->name);

    struct v4l2_query_ext_ctrl query = { .id = desc->id };
    ESP_RETURN_ON_FALSE(ioctl(*desc->fd, VIDIOC_QUERY_EXT_CTRL, &query) == 0,
                        ESP_ERR_NOT_SUPPORTED, TAG, "%s query failed", desc->name);
    int32_t current;
    ESP_RETURN_ON_ERROR(control_get(desc, &current), TAG, "%s get failed", desc->name);
    *out = (cam_builtin_ctrl_info_t) {
        .minimum = (int32_t)query.minimum,
        .maximum = (int32_t)query.maximum,
        .step = (int32_t)query.step,
        .default_value = (int32_t)query.default_value,
        .current = current,
    };
    return ESP_OK;
}

esp_err_t cam_builtin_ctrl_set(cam_builtin_ctrl_t control, int32_t value)
{
    const control_desc_t *desc = control_desc(control);
    cam_builtin_ctrl_info_t info;
    ESP_RETURN_ON_FALSE(desc, ESP_ERR_INVALID_ARG, TAG, "bad control");
    ESP_RETURN_ON_ERROR(cam_builtin_ctrl_query(control, &info), TAG, "%s unavailable", desc->name);
    ESP_RETURN_ON_FALSE(value >= info.minimum && value <= info.maximum,
                        ESP_ERR_INVALID_ARG, TAG, "%s value out of range", desc->name);

    struct v4l2_ext_control item = { .id = desc->id, .value = value };
    struct v4l2_ext_controls items = {
        .ctrl_class = V4L2_CTRL_ID2CLASS(desc->id),
        .count = 1,
        .controls = &item,
    };
    ESP_RETURN_ON_FALSE(ioctl(*desc->fd, VIDIOC_S_EXT_CTRLS, &items) == 0,
                        ESP_FAIL, TAG, "%s set failed", desc->name);
    return ESP_OK;
}

static void log_supported_controls(void)
{
    for (int i = 0; i < CAM_BUILTIN_CTRL_COUNT; i++) {
        cam_builtin_ctrl_info_t info;
        const control_desc_t *desc = control_desc(i);
        if (cam_builtin_ctrl_query(i, &info) == ESP_OK) {
            ESP_LOGI(TAG, "control %-10s min=%" PRId32 " max=%" PRId32 " step=%" PRId32
                          " default=%" PRId32 " current=%" PRId32,
                     desc->name, info.minimum, info.maximum, info.step,
                     info.default_value, info.current);
        } else {
            ESP_LOGW(TAG, "control %s unavailable", desc->name);
        }
    }
}

static esp_err_t queue_buffer(int index)
{
    struct v4l2_buffer buf = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
        .index = index,
    };
    return ioctl(s_cam.fd, VIDIOC_QBUF, &buf) == 0 ? ESP_OK : ESP_FAIL;
}

/* Called from normal task context by the consumer (or from publish). Re-queue
 * immediately: deferring this until the capture task runs again can deadlock,
 * because VIDIOC_DQBUF waits forever once every driver buffer is held. */
static void on_frame_release(const video_frame_t *frame, void *ctx)
{
    int index = (int)(intptr_t)frame->priv;
    if (queue_buffer(index) != ESP_OK) {
        ESP_LOGE(TAG, "QBUF %d failed", index);
    }
}

static void capture_task(void *arg)
{
    for (;;) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(s_cam.fd, VIDIOC_DQBUF, &buf) != 0) {
            ESP_LOGE(TAG, "DQBUF failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (!(buf.flags & V4L2_BUF_FLAG_DONE) || (buf.flags & V4L2_BUF_FLAG_ERROR)) {
            queue_buffer(buf.index);    /* bad frame: give it straight back */
            continue;
        }

        video_frame_t frame = {
            .data = s_cam.buf[buf.index],
            .w = s_cam.w,
            .h = s_cam.h,
            .stride = s_cam.w,
            .priv = (void *)(intptr_t)buf.index,
        };
        frame_mbox_publish(s_cam.mbox, &frame);
    }
}

esp_err_t cam_builtin_init(void)
{
    ESP_RETURN_ON_ERROR(bsp_camera_start(NULL), TAG, "bsp_camera_start failed");

    s_cam.fd = open(BSP_CAMERA_DEVICE, O_RDONLY);
    ESP_RETURN_ON_FALSE(s_cam.fd >= 0, ESP_FAIL, TAG, "open %s failed", BSP_CAMERA_DEVICE);

    s_cam.isp_fd = open(ESP_VIDEO_ISP1_DEVICE_NAME, O_RDONLY);
    if (s_cam.isp_fd < 0) {
        ESP_LOGW(TAG, "open %s failed; ISP controls disabled", ESP_VIDEO_ISP1_DEVICE_NAME);
    }

    struct v4l2_capability cap;
    ESP_RETURN_ON_FALSE(ioctl(s_cam.fd, VIDIOC_QUERYCAP, &cap) == 0, ESP_FAIL, TAG, "QUERYCAP failed");
    ESP_LOGI(TAG, "driver=%s card=%s", cap.driver, cap.card);

    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix.width = APP_CAM_BUILTIN_W,
        .fmt.pix.height = APP_CAM_BUILTIN_H,
        .fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565,
    };
    ESP_RETURN_ON_FALSE(ioctl(s_cam.fd, VIDIOC_S_FMT, &fmt) == 0, ESP_FAIL, TAG, "S_FMT failed");
    ESP_RETURN_ON_FALSE(ioctl(s_cam.fd, VIDIOC_G_FMT, &fmt) == 0, ESP_FAIL, TAG, "G_FMT failed");
    s_cam.w = fmt.fmt.pix.width;
    s_cam.h = fmt.fmt.pix.height;
    ESP_LOGI(TAG, "format %ux%u pixfmt=0x%08" PRIx32, s_cam.w, s_cam.h, (uint32_t)fmt.fmt.pix.pixelformat);
    ESP_RETURN_ON_FALSE(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB565, ESP_ERR_NOT_SUPPORTED, TAG,
                        "RGB565 not supported by the camera pipeline");

    struct v4l2_requestbuffers req = {
        .count = APP_CAM_BUILTIN_BUFS,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    ESP_RETURN_ON_FALSE(ioctl(s_cam.fd, VIDIOC_REQBUFS, &req) == 0, ESP_FAIL, TAG, "REQBUFS failed");
    ESP_RETURN_ON_FALSE(req.count == APP_CAM_BUILTIN_BUFS, ESP_FAIL, TAG, "got %u buffers", (unsigned)req.count);

    for (int i = 0; i < APP_CAM_BUILTIN_BUFS; i++) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        ESP_RETURN_ON_FALSE(ioctl(s_cam.fd, VIDIOC_QUERYBUF, &buf) == 0, ESP_FAIL, TAG, "QUERYBUF failed");
        s_cam.buf_len[i] = buf.length;
        s_cam.buf[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, s_cam.fd, buf.m.offset);
        ESP_RETURN_ON_FALSE(s_cam.buf[i], ESP_FAIL, TAG, "mmap failed");
        ESP_RETURN_ON_ERROR(queue_buffer(i), TAG, "QBUF failed");
    }

    s_cam.mbox = frame_mbox_create(on_frame_release, NULL);
    ESP_RETURN_ON_FALSE(s_cam.mbox, ESP_ERR_NO_MEM, TAG, "no memory");
    log_supported_controls();
    return ESP_OK;
}

frame_mbox_t cam_builtin_mbox(void)
{
    return s_cam.mbox;
}

esp_err_t cam_builtin_start(void)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_RETURN_ON_FALSE(ioctl(s_cam.fd, VIDIOC_STREAMON, &type) == 0, ESP_FAIL, TAG, "STREAMON failed");
    ESP_RETURN_ON_FALSE(xTaskCreate(capture_task, "cam_builtin", APP_TASK_CAM_BUILTIN_STACK, NULL,
                                    APP_TASK_CAM_BUILTIN_PRIO, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task create failed");
    ESP_LOGI(TAG, "streaming");
    return ESP_OK;
}
