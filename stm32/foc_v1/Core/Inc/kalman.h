// Protection macro for kalman.h
#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float dt;       // 50us
    float x[2];     // 0: angle (0-4095), 1: angular velocity
    float P[2][2];  // Error covariance matrix
    float Q[2][2];  // Process noise
    float R;        // Measurement noise
} KalmanStruct;

// Kalman filter with constant angular velocity
void kalman_predict(KalmanStruct *kf);
void kalman_update(KalmanStruct *kf, float measurement);

#endif // KALMAN_H