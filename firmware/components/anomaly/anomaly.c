#include "anomaly.h"
#include <math.h>
#include <string.h>

void cusum_init(CusumDetector *d, double k, double h) {
    memset(d, 0, sizeof(*d));
    d->k = k;
    d->h = h;
}

AnomalyFlags cusum_update(CusumDetector *d, double value) {
    /* Welford online mean/variance */
    d->count++;
    double delta  = value - d->mean;
    d->mean      += delta / d->count;
    double delta2 = value - d->mean;
    d->m2        += delta * delta2;

    double sigma = (d->count > 1) ? sqrt(d->m2 / (d->count - 1)) : 1.0;
    double z     = (d->count > 5 && sigma > 1e-9) ? (value - d->mean) / sigma : 0.0;

    /* CUSUM update */
    d->cusum_high = fmax(0.0, d->cusum_high + (value - d->mean) - d->k);
    d->cusum_low  = fmax(0.0, d->cusum_low  + (d->mean - value) - d->k);

    AnomalyFlags flags = ANOMALY_NONE;

    if (d->cusum_high > d->h) {
        flags |= ANOMALY_HIGH;
        d->cusum_high = 0.0; /* reset after alarm */
    }
    if (d->cusum_low > d->h) {
        flags |= ANOMALY_LOW;
        d->cusum_low = 0.0;
    }
    if (d->count > 10 && fabs(z) > 3.0) {
        flags |= ANOMALY_SPIKE;
    }

    return flags;
}

void cusum_reset(CusumDetector *d) {
    double k = d->k, h = d->h;
    memset(d, 0, sizeof(*d));
    d->k = k; d->h = h;
}

double cusum_mean(const CusumDetector *d)   { return d->mean; }
double cusum_stddev(const CusumDetector *d) {
    if (d->count < 2) return 0.0;
    return sqrt(d->m2 / (d->count - 1));
}
