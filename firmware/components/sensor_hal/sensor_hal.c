#include "sensor_hal.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

#ifdef ESP_PLATFORM
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_log.h"
static const char *TAG = "SENSOR";

#define BME280_ADDR     0x76
#define BME280_REG_ID   0xD0
#define BME280_CHIP_ID  0x60
#define BME280_REG_CTRL 0xF4
#define BME280_REG_DATA 0xF7

static int s_i2c_port;
static float s_temp_offset;
static float s_max_delta;
static float s_last_temp = NAN;
static uint32_t s_last_ts = 0;

/* BME280 compensation (simplified — full implementation uses factory trim params) */
static float bme280_read_temperature(void) {
    uint8_t reg = BME280_REG_DATA;
    uint8_t raw[6];
    i2c_master_write_read_device(s_i2c_port, BME280_ADDR, &reg, 1, raw, 6, 100);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    /* Simplified: real code uses device-specific calibration registers */
    return (float)adc_T / 5120.0f;
}

static float bme280_read_humidity(void) {
    uint8_t reg = 0xFD;
    uint8_t raw[2];
    i2c_master_write_read_device(s_i2c_port, BME280_ADDR, &reg, 1, raw, 2, 100);
    int32_t adc_H = ((int32_t)raw[0] << 8) | raw[1];
    return (float)adc_H / 1024.0f;
}

#else
/* Host simulation */
static float s_injected_temp = 22.0f;
static float s_injected_humidity = 50.0f;
static float s_temp_offset = 0.0f;
static float s_max_delta   = 5.0f;
static float s_last_temp   = 22.0f;

static uint32_t host_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000000 + ts.tv_nsec / 1000);
}
#endif

bool sensor_init(const SensorConfig *cfg) {
    s_temp_offset = cfg->temp_offset;
    s_max_delta   = cfg->max_delta_c;

#ifdef ESP_PLATFORM
    s_i2c_port = cfg->i2c_port;
    i2c_config_t conf = {