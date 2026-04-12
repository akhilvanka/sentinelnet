#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float temperature_c;
    float humidity_pct;
    float pressure_hpa;
    uint32_t timestamp_us;
    bool     valid;
} SensorReading;

typedef struct {
    int      i2c_port;     /* I2C_NUM_0 or I2C_NUM_1 */
    int      sda_pin;
    int      scl_pin;
    uint32_t freq_hz;      /* typically 400000 */
    float    temp_offset;  /* calibration offset in °C */
    float    max_delta_c;  /* max plausible temperature change per sample (°C/s) */
} SensorConfig;

bool sensor_init(const SensorConfig *cfg);
bool sensor_read(SensorReading *out);
void sensor_inject_test_value(float temp, float humidity); /* simulation/test only */
bool sensor_self_test(void);

#ifdef __cplusplus
}
#endif
