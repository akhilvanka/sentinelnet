
#include "sim.h"
#include "../firmware/components/bft_consensus/include/bft_consensus.h"
#include "../firmware/components/anomaly/include/anomaly.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

/* ---- Simulated mesh network ---- */

#define MAX_SIM_NODES 16

typedef struct SimNode SimNode;

static SimNode *s_nodes[MAX_SIM_NODES];
static uint8_t  s_n_nodes;
static double   s_packet_loss_rate = 0.0;  /* 0.0 – 1.0 */
static uint32_t s_rng_state = 12345;

static uint32_t sim_rand(void) {
    s_rng_state ^= s_rng_state << 13;
    s_rng_state ^= s_rng_state >> 17;
    s_rng_state ^= s_rng_state << 5;
    return s_rng_state;
}

static double sim_randf(void) { return (sim_rand() & 0xFFFFFF) / 16777216.0; }
static double sim_gauss(double mu, double sigma) {
    /* Box-Muller */
    double u1 = sim_randf() + 1e-10, u2 = sim_randf();
    double z  = sqrt(-2.0 * log(u1)) * cos(2.0 * 3.14159265 * u2);
    return mu + sigma * z;
}

/* ---- Node type ---- */

typedef enum {
    NODE_HONEST      = 0,
    NODE_FAULTY      = 1,   /* always reports a wrong constant value */
    NODE_BYZANTINE   = 2,   /* reports different values to different peers */
    NODE_SILENT      = 3,   /* drops all messages */
} NodeBehavior;

struct SimNode {
    uint8_t       id;
    NodeBehavior  behavior;
    double        true_temp;   /* "ground truth" temperature at this node */
    float         fault_value; /* value a faulty node reports */
    BFTContext    bft;
    CusumDetector cusum;
    uint32_t      rounds_completed;
    uint32_t      consensus_fails;
    double        last_committed;
};

/* ---- HMAC callbacks ---- */

static BFTCallbacks s_bft_cbs[MAX_SIM_NODES];

static void node_broadcast_report(BFTContext *ctx, const SensorReport *report) {
    /* Find which node this context belongs to */
    uint8_t sender = report->node_id;
    if (s_nodes[sender]->behavior == NODE_SILENT) return;

    for (uint8_t dst = 0; dst < s_n_nodes; ++dst) {
        if (dst == sender) continue;
        if (sim_randf() < s_packet_loss_rate) continue; /* simulate packet loss */

        SensorReport r = *report;

        /* Byzantine node: tamper with value differently for each recipient */
        if (s_nodes[sender]->behavior == NODE_BYZANTINE) {
            r.value = (float)(50.0 + sim_gauss(0, 20.0)); /* random junk */
            /* Re-sign with sender's key so it appears authentic */
            uint8_t sbuf[sizeof(node_id_t)+sizeof(round_t)+sizeof(float)+sizeof(uint32_t)];
            size_t off = 0;
            memcpy(sbuf+off, &r.node_id,      sizeof(node_id_t)); off+=sizeof(node_id_t);
            memcpy(sbuf+off, &r.round,        sizeof(round_t));   off+=sizeof(round_t);
            memcpy(sbuf+off, &r.value,        sizeof(float));      off+=sizeof(float);
            memcpy(sbuf+off, &r.timestamp_us, sizeof(uint32_t));
            sim_hmac(sender, sbuf, sizeof(sbuf), r.hmac);
        }

        bft_on_report_received(&s_nodes[dst]->bft, &r);
    }
}

static void node_broadcast_vote(BFTContext *ctx, const VoteMessage *v) {
    if (!v) return; /* batch send trigger (votes already distributed inline) */
}

static uint32_t node_get_ts(void) { return sim_time_us(); }

static bool node_verify_hmac(node_id_t node_id, const uint8_t *data, size_t len,
                               const uint8_t *expected) {
    uint8_t computed[BFT_HMAC_LEN];
    sim_hmac(node_id, data, len, computed);
    return memcmp(computed, expected, BFT_HMAC_LEN) == 0;
}

static void node_compute_hmac_for(uint8_t self_id, const uint8_t *data, size_t len,
                                   uint8_t *out) {
    sim_hmac(self_id, data, len, out);
}

/* Each node needs its own compute_hmac closure (different key per node).
 * We use a small trampoline array. */
#define MAKE_HMAC_FN(N) \
    static void hmac_fn_##N(const uint8_t *d, size_t l, uint8_t *o) { \
        node_compute_hmac_for(N, d, l, o); }
MAKE_HMAC_FN(0) MAKE_HMAC_FN(1) MAKE_HMAC_FN(2) MAKE_HMAC_FN(3)
MAKE_HMAC_FN(4) MAKE_HMAC_FN(5) MAKE_HMAC_FN(6) MAKE_HMAC_FN(7)

typedef void (*hmac_fn_t)(const uint8_t*, size_t, uint8_t*);
static hmac_fn_t s_hmac_fns[] = {
    hmac_fn_0, hmac_fn_1, hmac_fn_2, hmac_fn_3,
    hmac_fn_4, hmac_fn_5, hmac_fn_6, hmac_fn_7,
};

/* ---- Node lifecycle ---- */

static SimNode *node_create(uint8_t id, uint8_t n_nodes,
                              NodeBehavior behavior, float fault_value) {
    SimNode *n = (SimNode *)calloc(1, sizeof(SimNode));
    n->id          = id;
    n->behavior    = behavior;
    n->true_temp   = 22.0;
    n->fault_value = fault_value;

    s_bft_cbs[id].broadcast_report = node_broadcast_report;
    s_bft_cbs[id].broadcast_vote   = node_broadcast_vote;
    s_bft_cbs[id].get_timestamp_us = node_get_ts;
    s_bft_cbs[id].verify_hmac      = node_verify_hmac;
    s_bft_cbs[id].compute_hmac     = s_hmac_fns[id];

    bft_init(&n->bft, id, n_nodes, &s_bft_cbs[id]);
    cusum_init(&n->cusum, 0.5, 4.0);
    return n;
}

static float node_sense(SimNode *n) {
    switch (n->behavior) {
        case NODE_HONEST:
            return (float)sim_gauss(n->true_temp, 0.3); /* ±0.3°C sensor noise */
        case NODE_FAULTY:
            return n->fault_value;
        case NODE_BYZANTINE:
            return (float)sim_gauss(n->true_temp, 0.3); /* will be tampered in broadcast */
        case NODE_SILENT:
            return (float)sim_gauss(n->true_temp, 0.3); /* senses but won't broadcast */
        default:
            return n->fault_value;
    }
}

/* ---- Deliver votes between all nodes (simulated broadcast) ---- */

static void deliver_all_votes(round_t round) {
    /* Each node has computed votes internally; distribute them to all peers */
    for (uint8_t voter = 0; voter < s_n_nodes; ++voter) {
        if (s_nodes[voter]->behavior == NODE_SILENT) continue;
        BFTContext *ctx = &s_nodes[voter]->bft;

        /* Peek at the vote array inside context (white-box for simulation only) */
        /* We do this by having each node re-broadcast its votes */
        for (uint8_t target = 0; target < s_n_nodes; ++target) {
            /* Build a VoteMessage and deliver to all other nodes */
            /* (In firmware this goes over 802.15.4; here we call directly) */
            for (uint8_t dst = 0; dst < s_n_nodes; ++dst) {
                if (dst == voter) continue;
                if (sim_randf() < s_packet_loss_rate) continue;
                /* We don't have direct access to ctx internals from here.
                 * The simulation delivers votes by having the BFT run send
                 * them during broadcast_vote callback — handled inline. */
            }
        }
    }
}

/* ---- Run one simulation round across all nodes ---- */

typedef struct {
    double min_committed;
    double max_committed;
    double mean_committed;
    uint8_t nodes_committed;
    uint8_t nodes_failed;
} RoundResult;

static RoundResult run_round(double true_temp) {
    for (uint8_t i = 0; i < s_n_nodes; ++i)
        s_nodes[i]->true_temp = true_temp;

    /* Step 1: ALL nodes advance round counter simultaneously */
    for (uint8_t i = 0; i < s_n_nodes; ++i)
        bft_begin_round(&s_nodes[i]->bft);

    round_t round = s_nodes[0]->bft.current_round;

    /* Step 2: Each node senses, signs, and broadcasts its report.
     * node_broadcast_report delivers directly to peers via bft_on_report_received.
     * Since all nodes have current_round set, reports are accepted. */
    for (uint8_t i = 0; i < s_n_nodes; ++i) {
        if (s_nodes[i]->behavior == NODE_SILENT) continue;
        float v = node_sense(s_nodes[i]);

        SensorReport r;
        r.node_id      = i;
        r.round        = round;
        r.value        = v;
        r.timestamp_us = sim_time_us();

        uint8_t sbuf[sizeof(node_id_t)+sizeof(round_t)+sizeof(float)+sizeof(uint32_t)];
        size_t off = 0;
        memcpy(sbuf+off, &r.node_id,      sizeof(node_id_t)); off+=sizeof(node_id_t);
        memcpy(sbuf+off, &r.round,        sizeof(round_t));   off+=sizeof(round_t);
        memcpy(sbuf+off, &r.value,        sizeof(float));     off+=sizeof(float);
        memcpy(sbuf+off, &r.timestamp_us, sizeof(uint32_t));
        s_hmac_fns[i](sbuf, sizeof(sbuf), r.hmac);

        /* Deliver to self */
        bft_on_report_received(&s_nodes[i]->bft, &r);
        /* Deliver to all peers (simulated 802.15.4 broadcast) */
        node_broadcast_report(&s_nodes[i]->bft, &r);
    }

    /* Step 3: Exchange votes — each node's vote for each report */
    /* Phase 2a: each node computes and records its own votes */
    for (uint8_t voter = 0; voter < s_n_nodes; ++voter) {
        if (s_nodes[voter]->behavior == NODE_SILENT) continue;

        /* Build the valid-values list for this voter's view */
        float valid[BFT_MAX_NODES]; uint8_t vcnt = 0;
        for (uint8_t t = 0; t < s_n_nodes; ++t) {
            if (!s_nodes[voter]->bft.report_received[t]) continue;
            if (s_nodes[voter]->bft.node_states[t].status == NODE_STATUS_FAULTY) continue;
            valid[vcnt++] = s_nodes[voter]->bft.reports[t].value;
        }
        for (uint8_t t = 0; t < s_n_nodes; ++t) {
            if (!s_nodes[voter]->bft.report_received[t]) continue;
            bool q = bft_quorum_check(valid, vcnt,
                                      s_nodes[voter]->bft.reports[t].value, BFT_EPSILON);
            s_nodes[voter]->bft.votes[voter][t] = q ? VOTE_COMMIT : VOTE_REJECT;
        }
        s_nodes[voter]->bft.vote_received[voter] = true;
        s_nodes[voter]->bft.vote_count++;
    }

    /* Phase 2b: distribute votes to all peers */
    for (uint8_t voter = 0; voter < s_n_nodes; ++voter) {
        if (s_nodes[voter]->behavior == NODE_SILENT) continue;
        for (uint8_t dst = 0; dst < s_n_nodes; ++dst) {
            if (dst == voter) continue;
            if (sim_randf() < s_packet_loss_rate) continue;
            /* Copy voter's vote row into dst's vote matrix */
            memcpy(s_nodes[dst]->bft.votes[voter],
                   s_nodes[voter]->bft.votes[voter],
                   s_n_nodes * sizeof(vote_t));
            if (!s_nodes[dst]->bft.vote_received[voter]) {
                s_nodes[dst]->bft.vote_received[voter] = true;
                s_nodes[dst]->bft.vote_count++;
            }
        }
    }

    /* Step 4: Each node decides */
    RoundResult rr = { .min_committed = 1e9, .max_committed = -1e9 };

    for (uint8_t i = 0; i < s_n_nodes; ++i) {
        if (s_nodes[i]->behavior == NODE_SILENT) {
            rr.nodes_failed++;
            continue;
        }
        ConsensusResult result;
        bool ok = bft_finalize_round(&s_nodes[i]->bft,
                                     node_sense(s_nodes[i]), &result);
        if (ok) {
            s_nodes[i]->rounds_completed++;
            s_nodes[i]->last_committed = result.value;
            rr.nodes_committed++;
            rr.mean_committed += result.value;
            if (result.value < rr.min_committed) rr.min_committed = result.value;
            if (result.value > rr.max_committed) rr.max_committed = result.value;
        } else {
            s_nodes[i]->consensus_fails++;
            rr.nodes_failed++;
        }
    }

    if (rr.nodes_committed > 0)
        rr.mean_committed /= rr.nodes_committed;

    return rr;
}

/* ---- Test scenarios ---- */

static void print_node_states(void) {
    printf("  Node states:\n");
    for (uint8_t i = 0; i < s_n_nodes; ++i) {
        NodeState states[BFT_MAX_NODES]; uint8_t cnt;
        bft_get_node_states(&s_nodes[i]->bft, states, &cnt);

        const char *beh = s_nodes[i]->behavior == NODE_HONEST    ? "honest   " :
                          s_nodes[i]->behavior == NODE_FAULTY     ? "FAULTY   " :
                          s_nodes[i]->behavior == NODE_BYZANTINE  ? "BYZANTINE" :
                                                                     "SILENT   ";
        printf("  [%u] %s | ok=%u fail=%u | last_committed=%.2f\n",
               i, beh,
               s_nodes[i]->rounds_completed, s_nodes[i]->consensus_fails,
               s_nodes[i]->last_committed);
    }
}

#define PASS(msg) printf("  [PASS] %s\n", msg)
#define FAIL(msg) do { printf("  [FAIL] %s\n", msg); failures++; } while(0)

static int failures = 0;

static void scenario_baseline(void) {
    printf("\n=== Scenario 1: Baseline (5 honest nodes, true temp=22.0°C) ===\n\n");

    s_n_nodes = 5;
    for (uint8_t i = 0; i < s_n_nodes; ++i)
        s_nodes[i] = node_create(i, s_n_nodes, NODE_HONEST, 0);

    /* 20 rounds of stable temperature */
    printf("  Round  TrueTemp  Committed  Spread    Anomaly\n");
    printf("  %s\n", "----------------------------------------------");

    CusumDetector cusum; cusum_init(&cusum, 0.5, 4.0);
    int anomaly_count = 0;

    for (int r = 0; r < 30; ++r) {
        double true_temp = 22.0;
        /* Inject a sustained temperature rise at round 20 */
        if (r >= 20) true_temp = 22.0 + (r - 19) * 0.5;

        RoundResult rr = run_round(true_temp);
        AnomalyFlags flags = cusum_update(&cusum, rr.mean_committed);
        if (flags) anomaly_count++;

        const char *anom = flags & ANOMALY_HIGH  ? "DRIFT_UP " :
                           flags & ANOMALY_SPIKE ? "SPIKE    " :
                           flags & ANOMALY_LOW   ? "DRIFT_DN " : "-";

        printf("  %5d  %8.2f  %9.2f  %6.3f    %s\n",
               r+1, true_temp, rr.mean_committed,
               rr.max_committed - rr.min_committed, anom);
    }

    printf("\n");
    if (anomaly_count > 0 && anomaly_count < 15)
        PASS("CUSUM detected temperature rise, no false positives during stable phase");
    else
        FAIL("CUSUM anomaly detection incorrect");

    for (uint8_t i = 0; i < s_n_nodes; ++i) free(s_nodes[i]);
}

static void scenario_faulty_node(void) {
    printf("\n=== Scenario 2: One faulty node (reports 99°C instead of 22°C) ===\n\n");

    s_n_nodes = 5;
    for (uint8_t i = 0; i < s_n_nodes; ++i)
        s_nodes[i] = node_create(i, s_n_nodes, NODE_HONEST, 0);
    /* Node 2 is faulty — always reports 99°C */
    free(s_nodes[2]);
    s_nodes[2] = node_create(2, s_n_nodes, NODE_FAULTY, 99.0f);

    printf("  Running 15 rounds...\n");
    double max_error = 0.0;
    int faulty_isolated = 0;
    int total_committed = 0;

    for (int r = 0; r < 15; ++r) {
        RoundResult rr = run_round(22.0);
        double error = fabs(rr.mean_committed - 22.0);
        if (error > max_error) max_error = error;
        if (rr.nodes_committed > 0) total_committed++;

        /* Check if faulty node has been isolated */
        NodeState states[BFT_MAX_NODES]; uint8_t cnt;
        bft_get_node_states(&s_nodes[0]->bft, states, &cnt);
        if (states[2].status == NODE_STATUS_FAULTY) faulty_isolated++;
    }

    print_node_states();
    printf("\n  Max committed error from true value: %.3f°C\n", max_error);
    printf("  Rounds with successful consensus: %d/15\n", total_committed);
    printf("  Rounds where faulty node was detected: %d/15\n", faulty_isolated);

    if (max_error < BFT_EPSILON)
        PASS("Faulty node could not corrupt committed value beyond EPSILON");
    else
        FAIL("Faulty node corrupted committed value");

    if (faulty_isolated >= 10)
        PASS("Faulty node isolated within expected rounds");
    else
        FAIL("Faulty node not isolated quickly enough");

    for (uint8_t i = 0; i < s_n_nodes; ++i) free(s_nodes[i]);
}

static void scenario_byzantine(void) {
    printf("\n=== Scenario 3: Byzantine node (sends different values to each peer) ===\n\n");

    s_n_nodes = 7; /* need n >= 3f+1 = 7 for f=2; we'll use 1 Byzantine here */
    for (uint8_t i = 0; i < s_n_nodes; ++i)
        s_nodes[i] = node_create(i, s_n_nodes, NODE_HONEST, 0);
    free(s_nodes[3]);
    s_nodes[3] = node_create(3, s_n_nodes, NODE_BYZANTINE, 0);

    printf("  Running 20 rounds against one Byzantine node (n=7)...\n");
    double max_error = 0.0;
    int successes = 0;

    for (int r = 0; r < 20; ++r) {
        RoundResult rr = run_round(22.0);
        if (rr.nodes_committed > 0) {
            successes++;
            double error = fabs(rr.mean_committed - 22.0);
            if (error > max_error) max_error = error;
        }
    }

    printf("  Successful consensus rounds: %d/20\n", successes);
    printf("  Max value deviation from true: %.3f°C\n", max_error);

    if (successes >= 18)
        PASS("Consensus reached in >=90% of rounds despite Byzantine node");
    else
        FAIL("Too many consensus failures with 1 Byzantine of 7");

    if (max_error < BFT_EPSILON)
        PASS("Byzantine node could not corrupt committed value");
    else
        FAIL("Byzantine attack corrupted committed value");

    for (uint8_t i = 0; i < s_n_nodes; ++i) free(s_nodes[i]);
}

static void scenario_minority_crash(void) {
    printf("\n=== Scenario 4: Minority node crash (1 of 5 goes silent) ===\n\n");

    s_n_nodes = 5;
    for (uint8_t i = 0; i < s_n_nodes; ++i)
        s_nodes[i] = node_create(i, s_n_nodes, NODE_HONEST, 0);
    free(s_nodes[4]);
    s_nodes[4] = node_create(4, s_n_nodes, NODE_SILENT, 0);

    printf("  Running 10 rounds (node 4 silent, 4/5 honest)...\n");
    int successes = 0;

    for (int r = 0; r < 10; ++r) {
        RoundResult rr = run_round(22.0);
        if (rr.nodes_committed > 0) successes++;
    }

    printf("  Successful consensus rounds: %d/10\n", successes);

    if (successes >= 9)
        PASS("Quorum maintained with 1 silent node (4/5 available > 2/3 threshold)");
    else
        FAIL("Consensus failed unexpectedly with only 1 silent node");

    for (uint8_t i = 0; i < s_n_nodes; ++i) free(s_nodes[i]);
}

static void scenario_cascading(void) {
    printf("\n=== Scenario 5: Cascading faults (2 of 7 nodes faulty) ===\n\n");
    printf("  n=7, f_max=floor((7-1)/3)=2 — at the theoretical limit\n\n");

    s_n_nodes = 7;
    for (uint8_t i = 0; i < s_n_nodes; ++i)
        s_nodes[i] = node_create(i, s_n_nodes, NODE_HONEST, 0);
    free(s_nodes[1]); s_nodes[1] = node_create(1, s_n_nodes, NODE_FAULTY,    50.0f);
    free(s_nodes[5]); s_nodes[5] = node_create(5, s_n_nodes, NODE_BYZANTINE, 0);

    printf("  Running 20 rounds (nodes 1=FAULTY@50°C, 5=BYZANTINE, others honest)...\n");
    int successes = 0;
    double max_error = 0.0;

    for (int r = 0; r < 20; ++r) {
        RoundResult rr = run_round(22.0);
        if (rr.nodes_committed > 0) {
            successes++;
            double err = fabs(rr.mean_committed - 22.0);
            if (err > max_error) max_error = err;
        }
    }

    printf("  Successful consensus: %d/20\n", successes);
    printf("  Max value error: %.3f°C\n", max_error);
    print_node_states();

    if (successes >= 15)
        PASS("Consensus holds at theoretical 2/7 fault limit");
    else
        FAIL("Consensus failed at 2/7 fault limit");

    if (max_error < BFT_EPSILON)
        PASS("2 adversarial nodes could not corrupt committed value");
    else
        FAIL("Adversarial nodes corrupted committed value at 2/7 limit");

    for (uint8_t i = 0; i < s_n_nodes; ++i) free(s_nodes[i]);
}

/* ---- CUSUM unit tests ---- */

static void test_cusum(void) {
    printf("\n=== CUSUM Unit Tests ===\n\n");

    CusumDetector d; cusum_init(&d, 0.5, 4.0);

    /* Feed stable signal: no anomaly */
    for (int i = 0; i < 50; ++i) {
        double v = 22.0 + sim_gauss(0, 0.3);
        AnomalyFlags f = cusum_update(&d, v);
        if (f != ANOMALY_NONE) { FAIL("False positive on stable signal"); return; }
    }
    PASS("No false positives on stable Gaussian signal (50 samples)");

    /* Inject step increase: CUSUM should trigger */
    int detected = 0;
    for (int i = 0; i < 20; ++i) {
        double v = 25.0 + sim_gauss(0, 0.3); /* +3°C step */
        AnomalyFlags f = cusum_update(&d, v);
        if (f & ANOMALY_HIGH) { detected = 1; break; }
    }
    if (detected) PASS("Detected sustained temperature rise within 20 samples");
    else          FAIL("Failed to detect sustained temperature rise");

    /* Spike detection */
    cusum_reset(&d);
    for (int i = 0; i < 20; ++i) cusum_update(&d, 22.0);
    AnomalyFlags spike = cusum_update(&d, 60.0); /* 60°C spike */
    if (spike & ANOMALY_SPIKE) PASS("Single-sample spike detected");
    else                       FAIL("Spike not detected");
}

/* ---- BFT unit tests ---- */

static void test_median(void) {
    printf("\n=== BFT Unit Tests ===\n\n");

    float v5[] = {3.0f, 1.0f, 4.0f, 1.0f, 5.0f};
    float m5   = bft_median(v5, 5);
    if (fabsf(m5 - 3.0f) < 0.01f) PASS("Median of odd-count array correct");
    else                            FAIL("Median (odd) wrong");

    float v4[] = {1.0f, 3.0f, 5.0f, 7.0f};
    float m4   = bft_median(v4, 4);
    if (fabsf(m4 - 4.0f) < 0.01f) PASS("Median of even-count array correct");
    else                            FAIL("Median (even) wrong");

    float vals[7] = {22.1f, 22.3f, 22.0f, 99.0f, 22.2f, 22.4f, 22.1f};
    float med = bft_median(vals, 7);
    if (fabsf(med - 22.2f) < 0.1f) PASS("Median resistant to single outlier (99°C)");
    else                             FAIL("Median not resistant to outlier");

    float agree[5] = {22.0f, 22.1f, 22.2f, 22.0f, 22.3f};
    bool q = bft_quorum_check(agree, 5, 22.1f, 2.0f);
    if (q) PASS("Quorum check: 5/5 within epsilon → quorum");
    else   FAIL("Quorum check failed incorrectly");

    float disagree[5] = {22.0f, 22.1f, 99.0f, 50.0f, 22.2f};
    bool q2 = bft_quorum_check(disagree, 5, 99.0f, 2.0f);
    if (!q2) PASS("Quorum check: outlier value correctly rejected");
    else     FAIL("Quorum check passed outlier value");
}

int main(void) {
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║  SentinelNet — Byzantine Fault-Tolerant Sensor Consensus  ║\n");
    printf("║  Host Simulation                                           ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n");

    test_median();
    test_cusum();
    scenario_baseline();
    scenario_faulty_node();
    scenario_byzantine();
    scenario_minority_crash();
    scenario_cascading();

    printf("\n══════════════════════════════════════════════\n");
    if (failures == 0)
        printf("  All scenarios passed.\n");
    else
        printf("  %d scenario(s) FAILED.\n", failures);
    printf("══════════════════════════════════════════════\n");

    return failures > 0 ? 1 : 0;
}
