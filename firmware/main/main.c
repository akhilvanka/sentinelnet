
#include "bft_consensus.h"
#include "mesh_transport.h"
#include "sensor_hal.h"
#include "anomaly.h"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "esp_hmac.h"

static const char *TAG = "MAIN";
#define MEASURE_INTERVAL_S  10
#define ALERT_GPIO          GPIO_NUM_8

/* Hardware HMAC using ESP32-C6's HMAC peripheral (key never leaves chip) */
static void hw_hmac(const uint8_t *data, size_t len, uint8_t *out) {
    /* ESP32-C6 HMAC peripheral: HMAC key is burned into eFuse key block */
    esp_hmac_calculate(HMAC_KEY0, data, len, out);
}

static bool hw_verify_hmac(node_id_t node_id, const uint8_t *data, size_t len,
                            const uint8_t *expected) {
    /* In production: each node has a unique key in a shared key table
     * provisioned at manufacturing time. For dev: use a shared key. */
    uint8_t computed[BFT_HMAC_LEN];
    esp_hmac_calculate(HMAC_KEY0, data, len, computed);
    return memcmp(computed, expected, BFT_HMAC_LEN) == 0;
}

static uint32_t get_ts(void) { return (uint32_t)esp_timer_get_time(); }

#else  /* Host simulation — main is in sim/main.c */
void app_main(void) {}
#endif

#ifdef ESP_PLATFORM

static BFTContext  s_bft;
static CusumDetector s_cusum;

static void on_report(const SensorReport *r, int8_t rssi) {
    bft_on_report_received(&s_bft, r);
}
static void on_vote(const VoteMessage *v, int8_t rssi) {
    bft_on_vote_received(&s_bft, v);
}
static void bft_do_broadcast_report(BFTContext *ctx, const SensorReport *r) {
    mesh_broadcast_report(r);
}
static void bft_do_broadcast_vote(BFTContext *ctx, const VoteMessage *v) {
    /* votes are batched and sent after Phase 2 decides */
    (void)ctx; (void)v;
}

void app_main(void) {
    nvs_flash_init();

    /* Read node ID from NVS (provisioned at flash time) */
    nvs_handle_t nvs;
    nvs_open("sentinel", NVS_READONLY, &nvs);
    uint8_t node_id = 0, n_nodes = 5;
    nvs_get_u8(nvs, "node_id",  &node_id);
    nvs_get_u8(nvs, "n_nodes",  &n_nodes);
    nvs_close(nvs);
    ESP_LOGI(TAG, "Node %u of %u starting", node_id, n_nodes);

    /* GPIO for alert LED / buzzer */
    gpio_reset_pin(ALERT_GPIO);
    gpio_set_direction(ALERT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ALERT_GPIO, 0);

    /* Sensor */
    SensorConfig scfg = {
        .i2c_port    = 0,
        .sda_pin     = 6,
        .scl_pin     = 7,
        .freq_hz     = 400000,
        .temp_offset = 0.0f,
        .max_delta_c = 5.0f,
    };
    sensor_init(&scfg);
    if (!sensor_self_test()) {
        ESP_LOGE(TAG, "Sensor self-test FAILED");
        esp_restart();
    }

    /* 802.15.4 mesh */
    MeshCallbacks mcbs = { .on_report = on_report, .on_vote = on_vote };
    mesh_init(node_id, &mcbs);

    /* BFT consensus */
    BFTCallbacks bcbs = {
        .broadcast_report = bft_do_broadcast_report,
        .broadcast_vote   = bft_do_broadcast_vote,
        .get_timestamp_us = get_ts,
        .verify_hmac      = hw_verify_hmac,
        .compute_hmac     = hw_hmac,
    };
    bft_init(&s_bft, node_id, n_nodes, &bcbs);

    /* CUSUM: k=0.5°C, h=4°C */
    cusum_init(&s_cusum, 0.5, 4.0);

    /* Main consensus loop */
    for (;;) {
        SensorReading reading;
        if (!sensor_read(&reading)) {
            ESP_LOGW(TAG, "Sensor read failed");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ConsensusResult result;
        bool ok = bft_run_round(&s_bft, reading.temperature_c, &result);

        if (ok) {
            AnomalyFlags flags = cusum_update(&s_cusum, result.value);
            if (flags != ANOMALY_NONE) {
                ESP_LOGW(TAG, "ANOMALY detected: value=%.2f flags=%02X", result.value, flags);
                /* Pulse alert GPIO — LP core can latch this independently */
                gpio_set_level(ALERT_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(ALERT_GPIO, 0);
            } else {
                ESP_LOGI(TAG, "OK: consensus=%.2f°C (n=%u) cusum_h=%.2f",
                         result.value, result.committed, s_cusum.cusum_high);
            }
        }

        /* Sleep until next measurement window.
         * The LP core continues monitoring ALERT_GPIO during sleep. */
        esp_sleep_enable_timer_wakeup((uint64_t)MEASURE_INTERVAL_S * 1000000);
        esp_light_sleep_start();
    }
}
#endif
