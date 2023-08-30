/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

    @section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. Neither the name of the City College of New York nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL the CCNY ROBOTICS LAB BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "utilities/complementary_filter.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>

// NOTE: this gravity is differ from one we use in HardwareBridge, because
// we have not rescale the accelerometer
const double ComplementaryFilter::kGravity_ = 9.81;
const double ComplementaryFilter::kGamma_   = 0.01;
// Bias estimation steady state thresholds
const double ComplementaryFilter::kAngularVelocityThreshold_       = 0.2;
const double ComplementaryFilter::kAccelerationThreshold_           = 0.1;
const double ComplementaryFilter::kDeltaAngularVelocityThreshold_ = 0.01;

ComplementaryFilter::ComplementaryFilter()
    : gain_acc_( 0.01 ), bias_alpha_( 0.01 ), do_bias_estimation_( true ), do_adaptive_gain_( false ), do_free_acc_pred_( false ), initialized_( false ), steady_state_( false ), q0_( 1 ), q1_( 0 ),
      q2_( 0 ), q3_( 0 ), wx_prev_( 0 ), wy_prev_( 0 ), wz_prev_( 0 ), wx_bias_( 0 ), wy_bias_( 0 ), wz_bias_( 0 ) {}

ComplementaryFilter::~ComplementaryFilter() {}

void ComplementaryFilter::SetDoBiasEstimation( bool do_bias_estimation ) {
    do_bias_estimation_ = do_bias_estimation;
}

bool ComplementaryFilter::GetDoBiasEstimation() const {
    return do_bias_estimation_;
}

void ComplementaryFilter::SetDoAdaptiveGain( bool do_adaptive_gain ) {
    do_adaptive_gain_ = do_adaptive_gain;
}

bool ComplementaryFilter::GetDoAdaptiveGain() const {
    return do_adaptive_gain_;
}

void ComplementaryFilter::SetDoFreeAccPred( bool do_free_acc_pred ) {
    do_free_acc_pred_ = do_free_acc_pred;
}

bool ComplementaryFilter::GetDoFreeAccPred() const {
    return do_free_acc_pred_;
}

bool ComplementaryFilter::SetGainAcc( double gain ) {
    if ( gain >= 0 && gain <= 1.0 ) {
        gain_acc_ = gain;
        return true;
    }
    else
        return false;
}

double ComplementaryFilter::GetGainAcc() const {
    return gain_acc_;
}

bool ComplementaryFilter::GetSteadyState() const {
    return steady_state_;
}

bool ComplementaryFilter::SetBiasAlpha( double bias_alpha ) {
    if ( bias_alpha >= 0 && bias_alpha <= 1.0 ) {
        bias_alpha_ = bias_alpha;
        return true;
    }
    else
        return false;
}

double ComplementaryFilter::GetBiasAlpha() const {
    return bias_alpha_;
}

void ComplementaryFilter::SetOrientation( double q0, double q1, double q2, double q3 ) {
    // Set the state to inverse (state is fixed wrt body).
    InvertQuaternion( q0, q1, q2, q3, q0_, q1_, q2_, q3_ );
}

double ComplementaryFilter::GetAngularVelocityBiasX() const {
    return wx_bias_;
}

double ComplementaryFilter::GetAngularVelocityBiasY() const {
    return wy_bias_;
}

double ComplementaryFilter::GetAngularVelocityBiasZ() const {
    return wz_bias_;
}

void ComplementaryFilter::Update( double ax, double ay, double az, double wx, double wy, double wz, double dt ) {

    if ( !initialized_ ) {
        // First time - ignore prediction:
        GetMeasurement( ax, ay, az, q0_, q1_, q2_, q3_ );
        initialized_ = true;
        return;
    }

    int free_acc_cur   = iter_ % free_acc_pred_size_;
    int free_acc_pre   = ( iter_ - 1 ) % free_acc_pred_size_;
    int free_acc_ppre  = ( iter_ - 2 ) % free_acc_pred_size_;
    int free_acc_pppre = ( iter_ - 3 ) % free_acc_pred_size_;

    if ( do_free_acc_pred_ ) {
        ax = ax - 0.6 * free_acc_[ free_acc_pre ][ 0 ] - 0.3 * free_acc_[ free_acc_ppre ][ 0 ] - 0.1 * free_acc_[ free_acc_pppre ][ 0 ];
        ay = ay - 0.6 * free_acc_[ free_acc_pre ][ 1 ] - 0.3 * free_acc_[ free_acc_ppre ][ 1 ] - 0.1 * free_acc_[ free_acc_pppre ][ 1 ];
        az = az - 0.6 * free_acc_[ free_acc_pre ][ 2 ] - 0.3 * free_acc_[ free_acc_ppre ][ 2 ] - 0.1 * free_acc_[ free_acc_pppre ][ 2 ];
    }
    // Bias estimation.
    if ( do_bias_estimation_ )
        UpdateBiases( ax, ay, az, wx, wy, wz );

    // Prediction.
    double q0_pred, q1_pred, q2_pred, q3_pred;
    GetPrediction( wx, wy, wz, dt, q0_pred, q1_pred, q2_pred, q3_pred );

    // Correction (from acc):
    // q_ = q_pred * [(1-gain) * qI + gain * dq_acc]
    // where qI = identity quaternion
    double dq0_acc, dq1_acc, dq2_acc, dq3_acc;
    GetAccCorrection( ax, ay, az, q0_pred, q1_pred, q2_pred, q3_pred, dq0_acc, dq1_acc, dq2_acc, dq3_acc );

    double gain;
    if ( do_adaptive_gain_ ) {
        gain = GetAdaptiveGain( gain_acc_, ax, ay, az );
        // std::cout << "\r" << "adaptive gain: " << gain;
        // printf("\radaptive gain: %lf", gain);
    }
    else {
        gain = gain_acc_;
    }

    ScaleQuaternion( gain, dq0_acc, dq1_acc, dq2_acc, dq3_acc );

    QuaternionMultiplication( q0_pred, q1_pred, q2_pred, q3_pred, dq0_acc, dq1_acc, dq2_acc, dq3_acc, q0_, q1_, q2_, q3_ );

    NormalizeQuaternion( q0_, q1_, q2_, q3_ );

    double free_ax, free_ay, free_az;
    GetFreeAcc( ax, ay, az, free_ax, free_ay, free_az );

    free_acc_[ free_acc_cur ][ 0 ] = free_ax;
    free_acc_[ free_acc_cur ][ 1 ] = free_ay;
    free_acc_[ free_acc_cur ][ 2 ] = free_az;

    iter_++;
}

void ComplementaryFilter::GetFreeAcc( double ax, double ay, double az, double& free_ax, double& free_ay, double& free_az ) {
    double ag_x, ag_y, ag_z;
    RotateVectorByQuaternion( 0, 0, kGravity_, q0_, q1_, q2_, q3_, ag_x, ag_y, ag_z );
    free_ax = ax - ag_x;
    free_ay = ay - ag_y;
    free_az = az - ag_z;
}

bool ComplementaryFilter::CheckState( double ax, double ay, double az, double wx, double wy, double wz ) const {
    double acc_magnitude = sqrt( ax * ax + ay * ay + az * az );
    if ( fabs( acc_magnitude - kGravity_ ) > kAccelerationThreshold_ ) {
        return false;
    }

    if ( fabs( wx - wx_prev_ ) > kDeltaAngularVelocityThreshold_ || fabs( wy - wy_prev_ ) > kDeltaAngularVelocityThreshold_ || fabs( wz - wz_prev_ ) > kDeltaAngularVelocityThreshold_ ) {
        return false;
    }

    if ( fabs( wx - wx_bias_ ) > kAngularVelocityThreshold_ || fabs( wy - wy_bias_ ) > kAngularVelocityThreshold_ || fabs( wz - wz_bias_ ) > kAngularVelocityThreshold_ ) {
        return false;
    }

    return true;
}

void ComplementaryFilter::UpdateBiases( double ax, double ay, double az, double wx, double wy, double wz ) {
    steady_state_ = CheckState( ax, ay, az, wx, wy, wz );

    if ( steady_state_ ) {
        wx_bias_ += bias_alpha_ * ( wx - wx_bias_ );
        wy_bias_ += bias_alpha_ * ( wy - wy_bias_ );
        wz_bias_ += bias_alpha_ * ( wz - wz_bias_ );
    }

    wx_prev_ = wx;
    wy_prev_ = wy;
    wz_prev_ = wz;
}

void ComplementaryFilter::GetPrediction( double wx, double wy, double wz, double dt, double& q0_pred, double& q1_pred, double& q2_pred, double& q3_pred ) {
    double wx_unb = wx - wx_bias_;
    double wy_unb = wy - wy_bias_;
    double wz_unb = wz - wz_bias_;

    q0_pred = q0_ + 0.5 * dt * ( wx_unb * q1_ + wy_unb * q2_ + wz_unb * q3_ );
    q1_pred = q1_ + 0.5 * dt * ( -wx_unb * q0_ - wy_unb * q3_ + wz_unb * q2_ );
    q2_pred = q2_ + 0.5 * dt * ( wx_unb * q3_ - wy_unb * q0_ - wz_unb * q1_ );
    q3_pred = q3_ + 0.5 * dt * ( -wx_unb * q2_ + wy_unb * q1_ - wz_unb * q0_ );

    NormalizeQuaternion( q0_pred, q1_pred, q2_pred, q3_pred );
}

void ComplementaryFilter::GetMeasurement( double ax, double ay, double az, double& q0_meas, double& q1_meas, double& q2_meas, double& q3_meas ) {
    // q_acc is the quaternion obtained from the acceleration vector representing
    // the orientation of the Global frame wrt the Local frame with arbitrary yaw
    // (intermediary frame). q3_acc is defined as 0.

    // Normalize acceleration vector.
    NormalizeVector( ax, ay, az );

    if ( az >= 0 ) {
        q0_meas = sqrt( ( az + 1 ) * 0.5 );
        q1_meas = -ay / ( 2.0 * q0_meas );
        q2_meas = ax / ( 2.0 * q0_meas );
        q3_meas = 0;
    }
    else {
        double X = sqrt( ( 1 - az ) * 0.5 );
        q0_meas  = -ay / ( 2.0 * X );
        q1_meas  = X;
        q2_meas  = 0;
        q3_meas  = ax / ( 2.0 * X );
    }
}

void ComplementaryFilter::GetAccCorrection( double ax, double ay, double az, double p0, double p1, double p2, double p3, double& dq0, double& dq1, double& dq2, double& dq3 ) {
    // Normalize acceleration vector.
    NormalizeVector( ax, ay, az );

    // Acceleration reading rotated into the world frame by the inverse predicted
    // quaternion (predicted gravity):
    double gx, gy, gz;
    RotateVectorByQuaternion( ax, ay, az, p0, -p1, -p2, -p3, gx, gy, gz );

    // Delta quaternion that rotates the predicted gravity into the real gravity:
    dq0 = sqrt( ( gz + 1 ) * 0.5 );
    dq1 = -gy / ( 2.0 * dq0 );
    dq2 = gx / ( 2.0 * dq0 );
    dq3 = 0.0;
}

void ComplementaryFilter::GetOrientation( double& q0, double& q1, double& q2, double& q3 ) const {
    // Return the inverse of the state (state is fixed wrt body).
    InvertQuaternion( q0_, q1_, q2_, q3_, q0, q1, q2, q3 );
}

double ComplementaryFilter::GetAdaptiveGain( double alpha, double ax, double ay, double az ) {
    double a_mag = sqrt( ax * ax + ay * ay + az * az );
    double error = fabs( a_mag - kGravity_ ) / kGravity_;
    double factor;
    double error1 = 0.1;
    double error2 = 0.2;
    // double m = 1.0/(error1 - error2);
    // double b = 1.0 - m*error1;
    if ( error < error1 )
        factor = 1.0;
    else if ( error < error2 )
        factor = -10 * error + 2;
    else
        factor = 0.0;
    // printf("FACTOR: %f \n", factor);
    return factor * alpha;
    ////    double a_mag = sqrt(ax*ax + ay*ay + az*az);
    //    double error = fabs(ax)/kGravity_ + fabs(ay)/kGravity_ + fabs(az - kGravity_)/kGravity_;
    ////    double factor;
    //    double b11=alpha;
    //    b11 +=1;
    //    double error1 = 0.06;
    //    double error2 = 0.2;
    //    double error3 = 0.5;
    //    double gain1 = 0.01;
    //    double gain2 = 0.0001;
    //    double m = 0.0;
    //    double b = 0.0;
    //    if (error < error1)
    //    {
    //        m = 0.0;
    //        b = gain1;
    //    }
    //    else if (error < error2)
    //    {
    //        m = (gain1 - gain2)/(error1 - error2);
    //        b = gain1 - m*error1;
    //    }
    //    else if (error < error3)
    //    {
    //        m = (gain2 - 0.0)/(error2 - error3);
    //        b = gain2 - m*error2;
    //    }
    //    else
    //    {
    //        m = 0.0;
    //        b = 0.0;
    //    }
    //
    //    printf("FACTOR: %f %f %f %f\n", m, b, error, (m * error + b));
    //    return m * error + b;

    // double a_mag = sqrt(ax*ax + ay*ay + az*az);
    // double error = fabs(a_mag - kGravity_)/kGravity_;
    // double factor;
    // double error1 = 0.1;
    // double error2 = 0.2;
    // // double m = 1.0/(error1 - error2);
    // // double b = 1.0 - m*error1;
    // if (error < error1)
    //   factor = 1.0;
    // else if (error < error2)
    //   factor = -10*error + 2;
    // else
    //   factor = 0.0;
    // //printf("FACTOR: %f \n", factor);
    // return fabs(qw_dot0)*10;
}

void ComplementaryFilter::NormalizeVector( double& x, double& y, double& z ) {
    double norm = sqrt( x * x + y * y + z * z );

    x /= norm;
    y /= norm;
    z /= norm;
}

void ComplementaryFilter::NormalizeQuaternion( double& q0, double& q1, double& q2, double& q3 ) {
    double norm = sqrt( q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3 );
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

void ComplementaryFilter::InvertQuaternion( double q0, double q1, double q2, double q3, double& q0_inv, double& q1_inv, double& q2_inv, double& q3_inv ) {
    // Assumes quaternion is normalized.
    q0_inv = q0;
    q1_inv = -q1;
    q2_inv = -q2;
    q3_inv = -q3;
}

void ComplementaryFilter::ScaleQuaternion( double gain, double& dq0, double& dq1, double& dq2, double& dq3 ) {
    // std::cout << "\r" << "dq0: " << dq0;
    if ( dq0 < 0.0 )  // 0.9
    {
        // Slerp (Spherical linear interpolation):
        double angle = acos( dq0 );
        double A     = sin( angle * ( 1.0 - gain ) ) / sin( angle );
        double B     = sin( angle * gain ) / sin( angle );
        dq0          = A + B * dq0;
        dq1          = B * dq1;
        dq2          = B * dq2;
        dq3          = B * dq3;
    }
    else {
        // Lerp (Linear interpolation):
        dq0 = ( 1.0 - gain ) + gain * dq0;
        dq1 = gain * dq1;
        dq2 = gain * dq2;
        dq3 = gain * dq3;
    }

    NormalizeQuaternion( dq0, dq1, dq2, dq3 );
}

void ComplementaryFilter::QuaternionMultiplication( double p0, double p1, double p2, double p3, double q0, double q1, double q2, double q3, double& r0, double& r1, double& r2, double& r3 ) {
    // r = p q
    r0 = p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3;
    r1 = p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2;
    r2 = p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1;
    r3 = p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0;
}

void ComplementaryFilter::RotateVectorByQuaternion( double x, double y, double z, double q0, double q1, double q2, double q3, double& vx, double& vy, double& vz ) {
    vx = ( q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3 ) * x + 2 * ( q1 * q2 - q0 * q3 ) * y + 2 * ( q1 * q3 + q0 * q2 ) * z;
    vy = 2 * ( q1 * q2 + q0 * q3 ) * x + ( q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3 ) * y + 2 * ( q2 * q3 - q0 * q1 ) * z;
    vz = 2 * ( q1 * q3 - q0 * q2 ) * x + 2 * ( q2 * q3 + q0 * q1 ) * y + ( q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3 ) * z;
}
