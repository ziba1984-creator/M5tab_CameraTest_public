/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <string.h> // For memcpy

#include "esp_log.h"

#include "uvc_stream.h" // For uvc_host_stream_pause()
#include "uvc_types_priv.h"
#include "uvc_check_priv.h"
#include "uvc_frame_priv.h"
#include "uvc_critical_priv.h"

static const char *TAG = "uvc-isoc";

/* Local diagnostic (M5tab_try): what the isochronous packets actually contain.
 * Set UVC_ISOC_DIAG to 1 to print packet and frame statistics once a second. */
#define UVC_ISOC_DIAG 0
#define DIAG(stmt) do { if (UVC_ISOC_DIAG) { stmt; } } while (0)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static struct {
    uint32_t xfers, pkts, by_status[8], bytes, max_len, zero_len, bad_hdr, eof, fid_toggle, err_bit;
    uint32_t d_invalid, d_zero, d_zero_eof, d_sof, d_noframe, d_missed, d_eof, d_eof_skip, d_eof_noframe, d_deliver;
    uint8_t last_hdr[4];
    TickType_t next;
} s_diag;

static void diag_packet(const usb_isoc_packet_desc_t *d, const uint8_t *p)
{
    s_diag.pkts++;
    if ((unsigned)d->status < 8) {
        s_diag.by_status[d->status]++;
    }
    if (d->status != USB_TRANSFER_STATUS_COMPLETED) {
        return;
    }
    int n = d->actual_num_bytes;
    s_diag.bytes += n;
    if ((uint32_t)n > s_diag.max_len) {
        s_diag.max_len = n;
    }
    if (n == 0) {
        s_diag.zero_len++;
        return;
    }
    memcpy(s_diag.last_hdr, p, n < 4 ? n : 4);
    if (n < 2 || p[0] < 2 || p[0] > n) {
        s_diag.bad_hdr++;
        return;
    }
    if (p[1] & 0x02) s_diag.eof++;
    if (p[1] & 0x40) s_diag.err_bit++;
    static uint8_t last_fid = 0xff;
    if ((p[1] & 0x01) != last_fid) {
        s_diag.fid_toggle++;
        last_fid = p[1] & 0x01;
    }
}

static void diag_report(void)
{
    TickType_t now = xTaskGetTickCount();
    if ((int32_t)(now - s_diag.next) < 0) {
        return;
    }
    s_diag.next = now + pdMS_TO_TICKS(1000);
    ESP_LOGW(TAG, "diag: xfers %lu pkts %lu | st ok %lu err %lu tout %lu canc %lu stall %lu ovf %lu skip %lu nodev %lu "
             "| bytes %lu max %lu zero %lu badhdr %lu eof %lu fid %lu errbit %lu | hdr %02x %02x %02x %02x",
             s_diag.xfers, s_diag.pkts, s_diag.by_status[0], s_diag.by_status[1], s_diag.by_status[2],
             s_diag.by_status[3], s_diag.by_status[4], s_diag.by_status[5], s_diag.by_status[6], s_diag.by_status[7],
             s_diag.bytes, s_diag.max_len, s_diag.zero_len, s_diag.bad_hdr, s_diag.eof, s_diag.fid_toggle,
             s_diag.err_bit, s_diag.last_hdr[0], s_diag.last_hdr[1], s_diag.last_hdr[2], s_diag.last_hdr[3]);
    ESP_LOGW(TAG, "diag2: invalid %lu zero %lu zero+eof %lu | sof %lu nobuf %lu missedEoF %lu | eof %lu eof_skip %lu eof_noframe %lu delivered %lu",
             s_diag.d_invalid, s_diag.d_zero, s_diag.d_zero_eof, s_diag.d_sof, s_diag.d_noframe, s_diag.d_missed,
             s_diag.d_eof, s_diag.d_eof_skip, s_diag.d_eof_noframe, s_diag.d_deliver);
    uint8_t h[4];
    memcpy(h, s_diag.last_hdr, 4);
    TickType_t nx = s_diag.next;
    memset(&s_diag, 0, sizeof(s_diag));
    memcpy(s_diag.last_hdr, h, 4);
    s_diag.next = nx;
}

/**
 * @brief Callback function for handling Isochronous USB transfers from a UVC camera.
 *
 * This function processes isochronous transfer packets, which may contain video frame data. The following key points
 * are handled in the transfer:
 *
 * - **Start of Frame (SoF)**: Detected by a change in Frame ID, which toggles between 0 and 1.
 * - **End of Frame (EoF)**: Signaled in the packet header.
 * - **Transfer Characteristics**:
 *   - **No CRC**: Data corruption is possible.
 *   - **No ACK**: Packets can be missed.
 *   - **Packet Header**: Each packet includes a header used to detect errors, missed packets, and other issues.
 *
 * The callback performs the following tasks:
 * 1. Checks the status of each isochronous packet and handles various USB transfer statuses (e.g., completed,
 *    error, device disconnected).
 * 2. Parses packet headers to detect the start of new frames, handles errors, and manages frame buffers.
 * 3. Aggregates valid data into a frame buffer, ensuring no buffer overflow occurs.
 * 4. Signals the end of a frame and invokes user-defined callbacks if necessary.
 *
 * @param[in] transfer Pointer to the completed USB transfer structure.
 */
void isoc_transfer_callback(usb_transfer_t *transfer)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    uvc_stream_t *uvc_stream = (uvc_stream_t *)transfer->context;

    // USB_TRANSFER_STATUS_NO_DEVICE is set in transfer->status.
    // Other error codes are saved in status of each ISOC packet descriptor
    if (transfer->status == USB_TRANSFER_STATUS_NO_DEVICE) {
        ESP_ERROR_CHECK(uvc_host_stream_pause(uvc_stream)); // This should never fail
    }

    if (!UVC_ATOMIC_LOAD(uvc_stream->dynamic.streaming)) {
        return; // If the streaming was turned off, we don't have to do anything
    }

    const uint8_t *payload = transfer->data_buffer;
    if (UVC_ISOC_DIAG) {
        s_diag.xfers++;
        const uint8_t *p = transfer->data_buffer;
        for (int i = 0; i < transfer->num_isoc_packets; i++) {
            diag_packet(&transfer->isoc_packet_desc[i], p);
            p += transfer->isoc_packet_desc[i].num_bytes;
        }
        diag_report();
    }
    for (int i = 0; i < transfer->num_isoc_packets; i++) {
        usb_isoc_packet_desc_t *isoc_desc = &transfer->isoc_packet_desc[i];

        // Check USB status
        switch (isoc_desc->status) {
        case USB_TRANSFER_STATUS_COMPLETED:
            break;
        case USB_TRANSFER_STATUS_NO_DEVICE:
        case USB_TRANSFER_STATUS_CANCELED:
            ESP_ERROR_CHECK(uvc_host_stream_pause(uvc_stream)); // This should never fail
            return; // No need to process the rest
        case USB_TRANSFER_STATUS_ERROR:
        case USB_TRANSFER_STATUS_OVERFLOW:
        case USB_TRANSFER_STATUS_STALL:
            ESP_LOGW(TAG, "usb err %d", isoc_desc->status);
            uvc_stream->single_thread.skip_current_frame = true;
            goto next_isoc_packet; // Data corrupted

        case USB_TRANSFER_STATUS_TIMED_OUT:
        case USB_TRANSFER_STATUS_SKIPPED:
            goto next_isoc_packet; // Skipped and timed out ISOC transfers are not an issue
        default:
            assert(false);
        }

        // Local patch (M5tab_try): an empty isochronous packet only means the camera had
        // nothing to send in that (micro)frame. Without this, the header check below
        // rejects it and drops the whole frame being received (seen with VID 1908 /
        // PID 2311: ~70 % of packets are empty at 320x240, so no frame ever got through).
        if (isoc_desc->actual_num_bytes == 0) {
            goto next_isoc_packet;
        }

        // Check for start of new frame
        const uvc_payload_header_t *payload_header = (const uvc_payload_header_t *)payload;
        if (!uvc_frame_payload_header_validate(payload_header, isoc_desc->actual_num_bytes)) {
            DIAG(s_diag.d_invalid++;);
            ESP_LOGD(TAG, "invalid UVC payload header, %02x, %02x, len:%d", payload[0], payload[1], isoc_desc->actual_num_bytes);
            uvc_stream->single_thread.skip_current_frame = true;
            goto next_isoc_packet;
        }

        // Derive payload data pointer/length once and reuse below
        const uint8_t *payload_data = payload + payload_header->bHeaderLength;
        const size_t payload_data_len = isoc_desc->actual_num_bytes - payload_header->bHeaderLength;

        if (payload_data_len == 0) {
            // This is a zero-length packet, skip it
            DIAG(s_diag.d_zero++;);
            DIAG(if (payload_header->bmHeaderInfo.end_of_frame) s_diag.d_zero_eof++;);
            ESP_LOGD(TAG, "zero-length packet, skipping");
            goto next_isoc_packet;
        }

        // Check for error flag
        if (payload_header->bmHeaderInfo.error) {
            ESP_LOGW(TAG, "frame error");
            uvc_stream->single_thread.skip_current_frame = true;
        }

        const bool start_of_frame = (uvc_stream->single_thread.current_frame_id != payload_header->bmHeaderInfo.frame_id);
        if (start_of_frame) {
            // We detected start of new frame. Update Frame ID and start fetching this frame
            DIAG(s_diag.d_sof++;);
            uvc_stream->single_thread.current_frame_id   = payload_header->bmHeaderInfo.frame_id;
            uvc_stream->single_thread.skip_current_frame = payload_header->bmHeaderInfo.error;

            // Check mjpeg frame start
            if (uvc_stream->dynamic.vs_format.format == UVC_VS_FORMAT_MJPEG &&
                    (payload_data[0] != JPEG_MARKER || payload_data[1] != JPEG_SOI)) {
                // We received frame with invalid frame, skip this frame
                uvc_stream->single_thread.skip_current_frame = true;
                ESP_LOGW(TAG, "invalid MJPEG SOI");
            }

            // Get free frame buffer for this new frame
            UVC_ENTER_CRITICAL();
            const bool need_new_frame = (uvc_stream->dynamic.streaming && !uvc_stream->dynamic.current_frame);
            if (need_new_frame) {
                UVC_EXIT_CRITICAL();
                uvc_stream->dynamic.current_frame = uvc_frame_get_empty(uvc_stream);
                if (uvc_stream->dynamic.current_frame == NULL) {
                    // There is no free frame buffer now, skipping this frame
                    DIAG(s_diag.d_noframe++;);
                    uvc_stream->single_thread.skip_current_frame = true;

                    // Inform the user about the underflow
                    uvc_host_stream_callback_t stream_cb = uvc_stream->constant.stream_cb;
                    if (stream_cb) {
                        const uvc_host_stream_event_data_t event = {
                            .type = UVC_HOST_FRAME_BUFFER_UNDERFLOW,
                        };
                        stream_cb(&event, uvc_stream->constant.cb_arg);
                    }
                    goto next_isoc_packet;
                }
            } else {
                // We received SoF but current_frame is not NULL: We missed EoF - reset the frame buffer
                DIAG(s_diag.d_missed++;);
                ESP_EARLY_LOGW(TAG, "missed EoF");
                uvc_stream->single_thread.skip_current_frame = true;
                uvc_frame_reset(uvc_stream->dynamic.current_frame);
                UVC_EXIT_CRITICAL();
            }
        }

        // Add received data to frame buffer
        if (!uvc_stream->single_thread.skip_current_frame) {
            uvc_host_frame_t *current_frame = UVC_ATOMIC_LOAD(uvc_stream->dynamic.current_frame);

            esp_err_t ret = uvc_frame_add_data(current_frame, payload_data, payload_data_len);
            if (ret != ESP_OK) {
                // Frame buffer overflow, skip this frame
                uvc_stream->single_thread.skip_current_frame = true;

                // Inform the user about the overflow
                uvc_host_stream_callback_t stream_cb = uvc_stream->constant.stream_cb;
                if (stream_cb) {
                    const uvc_host_stream_event_data_t event = {
                        .type = UVC_HOST_FRAME_BUFFER_OVERFLOW,
                    };
                    stream_cb(&event, uvc_stream->constant.cb_arg);
                }
                goto next_isoc_packet;
            }
        }

        // End of Frame. Pass the frame to user
        if (payload_header->bmHeaderInfo.end_of_frame) {
            bool return_frame = true; // In case streaming is stopped ATM, we must return the frame

            // Check if the user did not stop the stream in the meantime
            UVC_ENTER_CRITICAL();
            uvc_host_frame_t *this_frame = uvc_stream->dynamic.current_frame;
            uvc_stream->dynamic.current_frame = NULL; // Stop writing more data to this frame

            // Determine if we should invoke the frame callback:
            // Only invoke the callback if streaming is active, a frame callback exists,
            // and we have a valid frame to pass to the user.
            const bool invoke_fb_callback = (uvc_stream->dynamic.streaming && uvc_stream->constant.frame_cb && this_frame && !uvc_stream->single_thread.skip_current_frame);
            UVC_EXIT_CRITICAL();

            DIAG(s_diag.d_eof++;);
            DIAG(if (uvc_stream->single_thread.skip_current_frame) s_diag.d_eof_skip++;);
            DIAG(if (!this_frame) s_diag.d_eof_noframe++;);
            if (invoke_fb_callback) {
                DIAG(s_diag.d_deliver++;);
                return_frame = uvc_stream->constant.frame_cb(this_frame, uvc_stream->constant.cb_arg);
            }
            if (return_frame) {
                // The user has processed the frame in his callback, return it back to empty queue
                uvc_host_frame_return(uvc_stream, this_frame);
            }
        }
next_isoc_packet:
        payload += isoc_desc->num_bytes;
        continue;
    }

    if (UVC_ATOMIC_LOAD(uvc_stream->dynamic.streaming)) {
        usb_host_transfer_submit(transfer); // Restart the transfer
    }
}
