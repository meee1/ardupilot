/*
Extended Kalman Filter class by Sam Tabor, 2013.
* http://diydrones.com/forum/topics/autonomous-soaring
* Set up for identifying thermals of Gaussian form, but could be adapted to other
* purposes by adapting the equations for the jacobians.
*/

#pragma once

#include <AP_Math/matrixN.h>

#define NUM_EKF_STATES 4

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter(void) {}
    
    VectorN<float,NUM_EKF_STATES> X;
    MatrixN<float,NUM_EKF_STATES> P;
    MatrixN<float,NUM_EKF_STATES> Q;
    float R;
    void reset(const VectorN<float,NUM_EKF_STATES> &x, const MatrixN<float,NUM_EKF_STATES> &p, const MatrixN<float,NUM_EKF_STATES> q, float r);
    void update(float z, float Vx, float Vy);

private:
    float measurementpredandjacobian(VectorN<float,NUM_EKF_STATES> &A);
};
