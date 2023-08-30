
#include "utilities/mahony_filter.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>

const double MahonyFilter::kGravity_ = 9.81;

MahonyFilter::MahonyFilter() : kp_( 0.1 ), ki_( 0.0 ), do_free_acc_pred_( true ), integralFBx_( 0.0 ), integralFBy_( 0.0 ), integralFBz_( 0.0 ), q0_( 1 ), q1_( 0 ), q2_( 0 ), q3_( 0 ) {}

MahonyFilter::~MahonyFilter() {}

float MahonyFilter::GetKp() const {
    return kp_;
}

void MahonyFilter::SetKp( float kp ) {
    kp_ = kp;
}

float MahonyFilter::GetKi() const {
    return ki_;
}

void MahonyFilter::SetKi( float ki ) {
    ki_ = ki;
}

void MahonyFilter::GetOrientation( double& q0, double& q1, double& q2, double& q3 ) const {
    q0 = q0_;
    q1 = q1_;
    q2 = q2_;
    q3 = q3_;
}

void MahonyFilter::SetDoFreeAccPred( bool do_free_acc_pred ) {
    do_free_acc_pred_ = do_free_acc_pred;
}

float MahonyFilter::InvSqrt( float x ) {
    float halfx = 0.5f * x;
    float y     = x;
    long  i     = *( long* )&y;
    i           = 0x5f3759df - ( i >> 1 );
    y           = *( float* )&i;
    y           = y * ( 1.5f - ( halfx * y * y ) );
    return y;
}

void MahonyFilter::Update( double ax, double ay, double az, double wx, double wy, double wz, double dt ) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    double ax_orig        = ax;
    double ay_orig        = ay;
    double az_orig        = az;
    int    free_acc_cur   = iter_ % free_acc_pred_size_;
    int    free_acc_pre   = ( iter_ - 1 ) % free_acc_pred_size_;
    int    free_acc_ppre  = ( iter_ - 2 ) % free_acc_pred_size_;
    int    free_acc_pppre = ( iter_ - 3 ) % free_acc_pred_size_;

    if ( do_free_acc_pred_ ) {
        ax = ax - 0.3 * ( 0.6 * free_acc_[ free_acc_pre ][ 0 ] + 0.3 * free_acc_[ free_acc_ppre ][ 0 ] + 0.1 * free_acc_[ free_acc_pppre ][ 0 ] );
        ay = ay - 0.3 * ( 0.6 * free_acc_[ free_acc_pre ][ 1 ] + 0.3 * free_acc_[ free_acc_ppre ][ 1 ] + 0.1 * free_acc_[ free_acc_pppre ][ 1 ] );
        az = az - 0.3 * ( 0.6 * free_acc_[ free_acc_pre ][ 2 ] + 0.3 * free_acc_[ free_acc_ppre ][ 2 ] + 0.1 * free_acc_[ free_acc_pppre ][ 2 ] );
    }

    if ( !( ( ax == 0.0f ) && ( ay == 0.0f ) && ( az == 0.0f ) ) ) {

        // Normalise accelerometer measurement
        recipNorm = InvSqrt( ax * ax + ay * ay + az * az );
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1_ * q3_ - q0_ * q2_;
        halfvy = q0_ * q1_ + q2_ * q3_;
        halfvz = q0_ * q0_ - 0.5f + q3_ * q3_;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = ( ay * halfvz - az * halfvy );
        halfey = ( az * halfvx - ax * halfvz );
        halfez = ( ax * halfvy - ay * halfvx );

        // Compute and apply integral feedback if enabled
        if ( ki_ > 0.0f ) {
            integralFBx_ += ki_ * halfex * dt;  // integral error scaled by Ki
            integralFBy_ += ki_ * halfey * dt;
            integralFBz_ += ki_ * halfez * dt;
            wx += integralFBx_;  // apply integral feedback
            wy += integralFBy_;
            wz += integralFBz_;
        }
        else {
            integralFBx_ = 0.0f;  // prevent integral windup
            integralFBy_ = 0.0f;
            integralFBz_ = 0.0f;
        }

        // Apply proportional feedback
        wx += kp_ * halfex;
        wy += kp_ * halfey;
        wz += kp_ * halfez;
    }

    // Integrate rate of change of quaternion
    wx *= ( 0.5f * dt );  // pre-multiply common factors
    wy *= ( 0.5f * dt );
    wz *= ( 0.5f * dt );
    qa = q0_;
    qb = q1_;
    qc = q2_;
    q0_ += ( -qb * wx - qc * wy - q3_ * wz );
    q1_ += ( qa * wx + qc * wz - q3_ * wy );
    q2_ += ( qa * wy - qb * wz + q3_ * wx );
    q3_ += ( qa * wz + qb * wy - qc * wx );

    // Normalise quaternion
    recipNorm = InvSqrt( q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_ );
    q0_ *= recipNorm;
    q1_ *= recipNorm;
    q2_ *= recipNorm;
    q3_ *= recipNorm;

    double free_ax, free_ay, free_az;
    GetFreeAcc( ax_orig, ay_orig, az_orig, free_ax, free_ay, free_az );

    free_acc_[ free_acc_cur ][ 0 ] = free_ax;
    free_acc_[ free_acc_cur ][ 1 ] = free_ay;
    free_acc_[ free_acc_cur ][ 2 ] = free_az;

    iter_++;
}

void MahonyFilter::GetFreeAcc( double ax, double ay, double az, double& free_ax, double& free_ay, double& free_az ) {
    double ag_x, ag_y, ag_z;
    RotateVectorByQuaternion( 0, 0, kGravity_, q0_, q1_, q2_, q3_, ag_x, ag_y, ag_z );
    free_ax = ax - ag_x;
    free_ay = ay - ag_y;
    free_az = az - ag_z;
}

void MahonyFilter::RotateVectorByQuaternion( double x, double y, double z, double q0, double q1, double q2, double q3, double& vx, double& vy, double& vz ) {
    vx = ( q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3 ) * x + 2 * ( q1 * q2 + q0 * q3 ) * y + 2 * ( q1 * q3 - q0 * q2 ) * z;
    vy = 2 * ( q1 * q2 - q0 * q3 ) * x + ( q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3 ) * y + 2 * ( q2 * q3 + q0 * q1 ) * z;
    vz = 2 * ( q1 * q3 + q0 * q2 ) * x + 2 * ( q2 * q3 - q0 * q1 ) * y + ( q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3 ) * z;
}