#include "bft_consensus.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifdef ESP_PLATFORM
#  include "freertos/FreeRTOS.h"
#  include "freertos/semphr.h"
#  include "esp_log.h"
#  define BFT_LOG(fmt, ...) ESP_LOGI("BFT", fmt, ##__VA_ARGS__)
#  define BFT_SLEEP_MS(ms)  vTaskDelay(pdMS_TO_TICKS(ms))
#else
/* Host simulation stubs */
#  include <stdio.h>
#  include <unistd.h>
#  define BFT_LOG(fmt, ...) printf("[BFT] " fmt "\n", ##__VA_ARGS__)
#  define BFT_SLEEP_MS(ms)  usleep((ms) * 1000)
#endif

/* (BFTContext definition lives in bft_consensus.h) */

/* ---- Helpers ---- */

/* Compare function for qsort */
static int float_cmp(const void *a, const void *b) {
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

float bft_median(float *values, uint8_t count) {
    if (count == 0) return 0.0f;
    float sorted[BFT_MAX_NODES];
    memcpy(sorted, values, count * sizeof(float));
    qsort(sorted, count, sizeof(float), float_cmp);
    if (count % 2 == 1)
        return sorted[count / 2];
    return (sorted[count / 2 - 1] + sorted[count / 2]) * 0.5f;
}

bool bft_quorum_check(const float *values, uint8_t count,
                      float candidate, float epsilon) {
    uint8_t agree = 0;
    for (uint8_t i = 0; i < count; ++i)
        if (fabsf(values[i] - candidate) <= epsilon) ++agree;
    /* Quorum: strictly more than 2/3 of all known nodes */
    return agree * 3 > count * 2;
}

static void clear_round_buffers(BFTContext *ctx) {
    memset(ctx->reports,         0, sizeof(ctx->reports));
    memset(ctx->report_received, 0, sizeof(ctx->report_received));
    memset(ctx->votes,           0, sizeof(ctx->votes));
    memset(ctx->vote_received,   0, sizeof(ctx->vote_received));
    ctx->report_count = 0;
    ctx->vote_count   = 0;
}

static bool wait_for_reports(BFTContext *ctx, uint32_t timeout_ms) {
    uint32_t elapsed = 0;
    uint8_t quorum_needed = (ctx->n_nodes * 2 + 2) / 3; /* ceil(2n/3) */
    while (elapsed < timeout_ms) {
        if (ctx->report_count >= quorum_needed) return true;
        BFT_SLEEP_MS(5);
        elapsed += 5;
    }
    return ctx->report_count >= quorum_needed;
}

static bool wait_for_votes(BFTContext *ctx, uint32_t timeout_ms) {
    uint32_t elapsed = 0;
    uint8_t quorum_needed = (ctx->n_nodes * 2 + 2) / 3;
    while (elapsed < timeout_ms) {
        if (ctx->vote_count >= quorum_needed) return true;
        BFT_SLEEP_MS(5);
        elapsed += 5;
    }
    return ctx->vote_count >= quorum_needed;
}

/* ---- Public API ---- */

void bft_init(BFTContext *ctx, node_id_t self_id, uint8_t n_nodes,
              const BFTCallbacks *cbs) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->self_id      = self_id;
    ctx->n_nodes      = n_nodes;
    ctx->cbs          = *cbs;
    ctx->current_round = 0;

    for (uint8_t i = 0; i < n_nodes; ++i) {
        ctx->node_states[i].id     = i;
        ctx->node_states[i].status = NODE_STATUS_OK;
    }
}

void bft_begin_round(BFTContext *ctx) {
    ctx->current_round++;
    clear_round_buffers(ctx);
}

bool bft_finalize_round(BFTContext *ctx, float local_value, ConsensusResult *result) {
    result->valid = false;
    result->round = ctx->current_round;

    /* ---- Phase 2: vote on each received report ---- */
    float valid_values[BFT_MAX_NODES];
    uint8_t valid_count = 0;

    for (uint8_t i = 0; i < ctx->n_nodes; ++i) {
        if (!ctx->report_received[i]) continue;
        if (ctx->node_states[i].status == NODE_STATUS_FAULTY) continue;

        const SensorReport *r = &ctx->reports[i];
        uint8_t sbuf[sizeof(node_id_t)+sizeof(round_t)+sizeof(float)+sizeof(uint32_t)];
        size_t soff = 0;
        memcpy(sbuf+soff, &r->node_id,      sizeof(node_id_t)); soff+=sizeof(node_id_t);
        memcpy(sbuf+soff, &r->round,        sizeof(round_t));   soff+=sizeof(round_t);
        memcpy(sbuf+soff, &r->value,        sizeof(float));      soff+=sizeof(float);
        memcpy(sbuf+soff, &r->timestamp_us, sizeof(uint32_t));
        if (!ctx->cbs.verify_hmac(i, sbuf, sizeof(sbuf), r->hmac)) {
            ctx->node_states[i].status = NODE_STATUS_FAULTY;
            continue;
        }
        valid_values[valid_count++] = r->value;
    }

    for (uint8_t i = 0; i < ctx->n_nodes; ++i) {
        if (!ctx->report_received[i]) continue;
        if (ctx->node_states[i].status == NODE_STATUS_FAULTY) continue;
        bool in_quorum = bft_quorum_check(valid_values, valid_count,
                                          ctx->reports[i].value, BFT_EPSILON);
        ctx->votes[ctx->self_id][i] = in_quorum ? VOTE_COMMIT : VOTE_REJECT;
    }
    ctx->vote_received[ctx->self_id] = true;
    ctx->vote_count++;

    /* ---- Phase 3: decide ---- */
    float committed_values[BFT_MAX_NODES];
    uint8_t committed_count = 0;

    for (uint8_t target = 0; target < ctx->n_nodes; ++target) {
        if (!ctx->report_received[target]) continue;
        if (ctx->node_states[target].status == NODE_STATUS_FAULTY) continue;

        uint8_t commit_votes = 0;
        for (uint8_t voter = 0; voter < ctx->n_nodes; ++voter) {
            if (!ctx->vote_received[voter]) continue;
            if (ctx->votes[voter][target] == VOTE_COMMIT) ++commit_votes;
        }
        bool committed = commit_votes * 3 > ctx->n_nodes * 2;
        if (committed) {
            committed_values[committed_count++] = ctx->reports[target].value;
            ctx->node_states[target].reject_streak  = 0;
            ctx->node_states[target].last_committed = ctx->reports[target].value;
            ctx->node_states[target].last_seen_round = ctx->current_round;
        } else {
            ctx->node_states[target].reject_streak++;
            if (ctx->node_states[target].reject_streak >= BFT_ISOLATION_ROUNDS) {
                ctx->node_states[target].status = NODE_STATUS_FAULTY;
                BFT_LOG("Round %u: node %u isolated after %d consecutive rejections",
                        ctx->current_round, target, BFT_ISOLATION_ROUNDS);
            }
        }
    }

    uint8_t quorum_needed = (ctx->n_nodes * 2 + 2) / 3;
    if (committed_count < quorum_needed) return false;

    result->value         = bft_median(committed_values, committed_count);
    result->committed     = committed_count;
    result->participating = ctx->report_count;
    result->valid         = true;
    return true;
}