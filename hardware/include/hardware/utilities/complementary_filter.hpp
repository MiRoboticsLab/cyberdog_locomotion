#ifndef COMPLEMENTARY_FILTER_HPP_
#define COMPLEMENTARY_FILTER_HPP_

class ComplementaryFilter {
public:
    ComplementaryFilter();
    virtual ~ComplementaryFilter();

    bool   SetGainAcc( double gain );
    double GetGainAcc() const;

    bool   SetBiasAlpha( double bias_alpha );
    double GetBiasAlpha() const;

    // When the filter is in the steady state, bias estimation will occur (if the
    // parameter is enabled).
    bool GetSteadyState() const;

    void SetDoBiasEstimation( bool do_bias_estimation );
    bool GetDoBiasEstimation() const;

    void SetDoAdaptiveGain( bool do_adaptive_gain );
    bool GetDoAdaptiveGain() const;

    void SetDoFreeAccPred( bool do_free_acc_pred );
    bool GetDoFreeAccPred() const;

    double GetAngularVelocityBiasX() const;
    double GetAngularVelocityBiasY() const;
    double GetAngularVelocityBiasZ() const;

    // Set the orientation, as a Hamilton Quaternion, of the body frame wrt the
    // fixed frame.
    void SetOrientation( double q0, double q1, double q2, double q3 );

    // Get the orientation, as a Hamilton Quaternion, of the body frame wrt the
    // fixed frame.
    void GetOrientation( double& q0, double& q1, double& q2, double& q3 ) const;

    // Update from accelerometer and gyroscope data.
    // [ax, ay, az]: Normalized gravity vector.
    // [wx, wy, wz]: Angular veloctiy, in rad / s.
    // dt: time delta, in seconds.
    void Update( double ax, double ay, double az, double wx, double wy, double wz, double dt );

private:
    static const double kGravity_;
    static const double kGamma_;
    // Bias estimation steady state thresholds
    static const double kAngularVelocityThreshold_;
    static const double kAccelerationThreshold_;
    static const double kDeltaAngularVelocityThreshold_;

    // Gain parameter for the complementary filter, belongs in [0, 1].
    double gain_acc_;

    // Bias estimation gain parameter, belongs in [0, 1].
    double bias_alpha_;

    // Parameter whether to do bias estimation or not.
    bool do_bias_estimation_;

    // Parameter whether to do adaptive gain or not.
    bool do_adaptive_gain_;

    bool do_free_acc_pred_;

    bool initialized_;
    bool steady_state_;

    int   free_acc_pred_size_ = 3;
    int   iter_               = 3;
    float free_acc_[ 3 ][ 3 ] = { { 0. } };
    // The orientation as a Hamilton quaternion (q0 is the scalar). Represents
    // the orientation of the fixed frame wrt the body frame.
    double q0_, q1_, q2_, q3_;

    // Bias in angular velocities;
    double wx_prev_, wy_prev_, wz_prev_;

    // Bias in angular velocities;
    double wx_bias_, wy_bias_, wz_bias_;

    void UpdateBiases( double ax, double ay, double az, double wx, double wy, double wz );

    bool CheckState( double ax, double ay, double az, double wx, double wy, double wz ) const;

    void GetPrediction( double wx, double wy, double wz, double dt, double& q0_pred, double& q1_pred, double& q2_pred, double& q3_pred );

    void GetMeasurement( double ax, double ay, double az, double& q0_meas, double& q1_meas, double& q2_meas, double& q3_meas );

    void GetAccCorrection( double ax, double ay, double az, double p0, double p1, double p2, double p3, double& dq0, double& dq1, double& dq2, double& dq3 );

    double GetAdaptiveGain( double alpha, double ax, double ay, double az );

    void GetFreeAcc( double ax, double ay, double az, double& free_ax, double& free_ay, double& free_az );
    // Utility math functions:

    static void NormalizeVector( double& x, double& y, double& z );

    static void NormalizeQuaternion( double& q0, double& q1, double& q2, double& q3 );

    static void ScaleQuaternion( double gain, double& dq0, double& dq1, double& dq2, double& dq3 );

    static void InvertQuaternion( double q0, double q1, double q2, double q3, double& q0_inv, double& q1_inv, double& q2_inv, double& q3_inv );

    static void QuaternionMultiplication( double p0, double p1, double p2, double p3, double q0, double q1, double q2, double q3, double& r0, double& r1, double& r2, double& r3 );

    static void RotateVectorByQuaternion( double x, double y, double z, double q0, double q1, double q2, double q3, double& vx, double& vy, double& vz );
};

#endif  // COMPLEMENTARY_FILTER_HPP_
