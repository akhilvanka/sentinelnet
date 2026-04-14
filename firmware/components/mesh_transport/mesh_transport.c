#include "mesh_transport.h"
#include <string.h>
#include <stdio.h>

#ifdef ESP_PLATFORM
#include "esp_ieee802154.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "MESH";

/* Frame receive queue — ISR posts frames here, task drains them */
static QueueHandle_t s_rx_queue;

typedef struct {
    uint8_t  data[128];
    uint8_t  len;
    int8_t   rssi;
    uint8_t  src_addr;
} RxFrame;

/* IEEE 802.15.4 receive callback (runs in ISR context) */
void esp_ieee802154_receive_done(uint8_t *frame, esp_ieee802154_frame_info_t *frame_info) {
    RxFrame rxf;
    rxf.len  = frame[0]; /* first byte is length per 802.15.4 spec */
    rxf.rssi = frame_info->rssi;
    memcpy(rxf.data, frame + 1, rxf.len);
    /* Best-effort: drop if queue full (real-time) */
    BaseType_t woken;
    xQueueSendFromISR(s_rx_queue, &rxf, &woken);
    portYIELD_FROM_ISR(woken);
}

/* Transmit done (for stats) */
void esp_ieee802154_transmit_done(const uint8_t *frame,
                                  const esp_ieee802154_frame_info_t *frame_info,
                                  esp_ieee802154_status_t status) {
    (void)frame; (void)frame_info; (void)status;
}

#else
/* ---- Host simulation stubs (mesh_transport is replaced by sim/mesh_sim.c) ---- */
#include <stdio.h>
#endif

/* ---- Stats ---- */
static uint32_t s_tx_count, s_rx_count, s_drop_count;
static int8_t   s_last_rssi[256];
static uint8_t  s_self_id;
static MeshCallbacks s_cbs;

/* ---- Frame builder ---- */
#define HDR_SIZE 6  /* magic + type + pan_id(2) + src + dst */

static uint32_t crc32_frame(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int k = 0; k < 8; ++k)
            crc = (crc >> 1) ^ (crc & 1 ? 0xEDB88320u : 0);
    }
    return crc ^ 0xFFFFFFFF;
}

static size_t build_frame(uint8_t *out, MeshMsgType type,
                           const uint8_t *payload, size_t payload_len) {
    out[0] = MESH_MAGIC;
    out[1] = (uint8_t)type;
    out[2] = (MESH_PAN_ID >> 8) & 0xFF;
    out[3] =  MESH_PAN_ID       & 0xFF;
    out[4] = s_self_id;
    out[5] = MESH_BROADCAST;
    memcpy(out + HDR_SIZE, payload, payload_len);
    uint32_t crc = crc32_frame(out, HDR_SIZE + payload_len);
    memcpy(out + HDR_SIZE + payload_len, &crc, 4);