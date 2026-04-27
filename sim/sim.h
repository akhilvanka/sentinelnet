#pragma once
/* Shared simulation types — replaces ESP-IDF + FreeRTOS for host builds */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* Stub offsetof if not provided */
#ifndef offsetof
#define offsetof(type, member) ((size_t)&((type*)0)->member)
#endif

/* Simulated hardware timestamp */
static inline uint32_t sim_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);
}

/* Simulated HMAC: SHA-256-like using a simple polynomial.
 * NOT cryptographically secure — demonstration only.
 * In firmware: replaced by ESP32-C6 hardware HMAC peripheral. */
static inline void sim_hmac(uint8_t node_id, const uint8_t *data, size_t len,
                              uint8_t out[32]) {
    /* Node-specific key: XOR seed with node_id so each node has unique key */
    uint32_t key = 0xDEADBEEF ^ ((uint32_t)node_id * 0x9E3779B9);
    uint32_t h[8] = {
        0x6a09e667 ^ key, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
        0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
    };
    for (size_t i = 0; i < len; ++i) {
        h[i % 8] ^= ((uint32_t)data[i] << (i % 24));
        h[i % 8]  = h[i % 8] * 0x9E3779B9 + h[(i+1)%8];
    }
    memcpy(out, h, 32);
}
