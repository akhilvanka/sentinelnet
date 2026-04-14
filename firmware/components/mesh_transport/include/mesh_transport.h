#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "bft_consensus.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MESH_PAN_ID       0x1492   /* arbitrary PAN identifier for this deployment */
#define MESH_MAGIC        0x5E     /* 'SE' for SentinelNet */
#define MESH_BROADCAST    0xFF

typedef enum {
    MSG_SENSOR_REPORT = 0x01,
    MSG_VOTE          = 0x02,
    MSG_HEARTBEAT     = 0x03,
    MSG_ISOLATE_NOTIF = 0x04,
} MeshMsgType;

typedef struct {
    void (*on_report)(const SensorReport *r, int8_t rssi);
    void (*on_vote)  (const VoteMessage  *v, int8_t rssi);
} MeshCallbacks;

void mesh_init(uint8_t self_id, const MeshCallbacks *cbs);

void mesh_broadcast_report(const SensorReport *report);

void mesh_broadcast_votes(uint8_t self_id, round_t round,
                          const vote_t votes[BFT_MAX_NODES],
                          const uint8_t hmacs[BFT_MAX_NODES][BFT_HMAC_LEN],
                          uint8_t n_targets);

int8_t mesh_last_rssi(uint8_t node_id);

void mesh_get_stats(uint32_t *tx, uint32_t *rx, uint32_t *dropped);

#ifdef __cplusplus
}
#endif
