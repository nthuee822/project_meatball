#include "kalman.h"

void kalman_predict(KalmanStruct *kf) {
    // x = F * x
    // theta = theta + omega * dt
    kf->x[0] = kf->x[0] + kf->x[1] * kf->dt;

    // P = F*P*F' + Q
    float p00 = kf->P[0][0];
    float p01 = kf->P[0][1];
    float p10 = kf->P[1][0];
    float p11 = kf->P[1][1];
    float dt = kf->dt;

    kf->P[0][0] = p00 + (p10 + p01) * dt + p11 * dt * dt + kf->Q[0][0];
    kf->P[0][1] = p01 + p11 * dt + kf->Q[0][1];
    kf->P[1][0] = p10 + p11 * dt + kf->Q[1][0];
    kf->P[1][1] = p11 + kf->Q[1][1];

    return;
}

void kalman_update(KalmanStruct *kf, float measurement) {
    // y = z - H*x
    float y = measurement - kf->x[0];

    // S = H*P*H' + R
    float S = kf->P[0][0] + kf->R;
    
    // K = P*H' / S
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    // x = x + K*y
    kf->x[0] += K0 * y;
    kf->x[1] += K1 * y;

    // P = (I - KH) * P
    kf->P[0][0] -= K0 * kf->P[0][0];
    kf->P[0][1] -= K0 * kf->P[0][1];
    kf->P[1][0] -= K1 * kf->P[0][0];
    kf->P[1][1] -= K1 * kf->P[0][1];

    return;
}