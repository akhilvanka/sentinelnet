#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ANOMALY_NONE  = 0,
    ANOMALY_HIGH  = 1,   /* sustained upward drift detected */
    ANOMALY_LOW   = 2,   /* sustained downward drift detected */
    ANOMALY_SPIKE = 4,   /* single-sample outlier (|x - mu| > 3*sigma) */
} AnomalyFlags;

typedef struct {
    double  mean;           /* running mean (Welford) */
    double  m2;             /* running sum of squared deviations */
    double  cusum_high;     /* upper CUSUM statistic */
    double  cusum_low;      /* lower CUSUM statistic */
    uint32_t count;
    double  k;              /* slack parameter */
    double  h;              /* threshold */
} CusumDetector;

void   cusum_init(CusumDetector *d, double k, double h);
AnomalyFlags cusum_update(CusumDetector *d, double value);
void   cusum_reset(CusumDetector *d);

double cusum_mean(const CusumDetector *d);
double cusum_stddev(const CusumDetector *d);

#ifdef __cplusplus
}
#endif
