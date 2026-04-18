#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Configuration ---- */

#define BFT_MAX_NODES        16
#define BFT_HMAC_LEN         32    /* SHA-256 */
#define BFT_EPSILON          2.0f  /* max tolerated sensor disagreement (°C, %, etc.) */
#define BFT_ISOLATION_ROUNDS 5     /* consecutive rejections before isolation */
#define BFT_ROUND_TIMEOUT_MS 500   /* max ms to wait for all reports per phase */

/* ---- Types ---- */

typedef uint8_t  node_id_t;
typedef uint32_t round_t;

typedef struct {
    node_id_t  node_id;
    round_t    round;
    float      value;          /* sensor reading */
    uint32_t   timestamp_us;   /* hardware timer value at sample time */
    uint8_t    hmac[BFT_HMAC_LEN]; /* HMAC-SHA256(node_key, node_id||round||value||ts) */
} __attribute__((packed)) SensorReport;

typedef enum {
    VOTE_COMMIT = 1,
    VOTE_REJECT = 2,
} vote_t;

typedef struct {
    node_id_t  voter;
    node_id_t  target;         /* node_id of report being voted on */
    round_t    round;
    vote_t     decision;
    uint8_t    hmac[BFT_HMAC_LEN]; /* voter signs their vote */
} __attribute__((packed)) VoteMessage;

typedef enum {
    NODE_STATUS_OK       = 0,
    NODE_STATUS_SLOW     = 1,  /* not responding in time */
    NODE_STATUS_FAULTY   = 2,  /* repeatedly rejected — isolated */
} NodeStatus;

typedef struct {
    node_id_t   id;
    NodeStatus  status;
    int         reject_streak;   /* consecutive rounds rejected */
    float       last_committed;  /* last committed value from this node */
    uint32_t    last_seen_round;
} NodeState;

typedef struct {
    float     value;           /* consensus-decided value */
    round_t   round;
    uint8_t   participating;   /* number of nodes that voted */
    uint8_t   committed;       /* number of committed reports in median */
    bool      valid;           /* false if quorum not reached */
} ConsensusResult;

/* Forward declaration so BFTCallbacks can reference BFTContext */
typedef struct BFTContext BFTContext;

typedef struct {
    void (*broadcast_report)(BFTContext *ctx, const SensorReport *report);
    void (*broadcast_vote)  (BFTContext *ctx, const VoteMessage  *vote);
    uint32_t (*get_timestamp_us)(void);
    bool (*verify_hmac)(node_id_t node_id, const uint8_t *data, size_t len,
                        const uint8_t *expected_hmac);
    void (*compute_hmac)(const uint8_t *data, size_t len, uint8_t *out_hmac);
} BFTCallbacks;

/* ---- Context (full definition for embedded-by-value use) ---- */

struct BFTContext {
    node_id_t    self_id;
    uint8_t      n_nodes;
    BFTCallbacks cbs;
    round_t      current_round;

    SensorReport reports[BFT_MAX_NODES];
    bool         report_received[BFT_MAX_NODES];
    uint8_t      report_count;

    vote_t       votes[BFT_MAX_NODES][BFT_MAX_NODES];
    bool         vote_received[BFT_MAX_NODES];
    uint8_t      vote_count;

    NodeState    node_states[BFT_MAX_NODES];
};

/* ---- API ---- */

void bft_init(BFTContext *ctx, node_id_t self_id, uint8_t n_nodes,
              const BFTCallbacks *cbs);

bool bft_run_round(BFTContext *ctx, float local_value, ConsensusResult *result);

void bft_begin_round(BFTContext *ctx);
bool bft_finalize_round(BFTContext *ctx, float local_value, ConsensusResult *result);

void bft_on_report_received(BFTContext *ctx, const SensorReport *report);

void bft_on_vote_received(BFTContext *ctx, const VoteMessage *vote);

void bft_get_node_states(const BFTContext *ctx,
                         NodeState out_states[BFT_MAX_NODES], uint8_t *out_count);

void bft_rehabilitate_node(BFTContext *ctx, node_id_t id);

/* ---- Internal (exposed for unit testing) ---- */
float bft_median(float *values, uint8_t count);
bool  bft_quorum_check(const float *values, uint8_t count,
                       float candidate, float epsilon);

#ifdef __cplusplus
}
#endif
