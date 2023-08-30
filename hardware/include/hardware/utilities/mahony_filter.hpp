#ifndef MAHONY_FILTER_HPP_
#define MAHONY_FILTER_HPP_

class MahonyFilter {
public:
    MahonyFilter();
    virtual ~MahonyFilter();

    float GetKp() const;
    void  SetKp( float kp );

    float GetKi() const;
    void  SetKi( float ki );

    void SetDoFreeAccPred( bool do_free_acc_pred );

    void GetOrientation( double& q0, double& q1, double& q2, double& q3 ) const;

    void Update( double ax, double ay, double az, double wx, double wy, double wz, double dt );

private:
    static const double kGravity_;

    void GetFreeAcc( double ax, double ay, double az, double& free_ax, double& free_ay, double& free_az );

    float InvSqrt( float x );

    static void RotateVectorByQuaternion( double x, double y, double z, double q0, double q1, double q2, double q3, double& vx, double& vy, double& vz );

    float kp_, ki_;

    bool do_free_acc_pred_;

    float integralFBx_ = 0.0f, integralFBy_ = 0.0f, integralFBz_ = 0.0f;

    int   free_acc_pred_size_ = 3;
    int   iter_               = 3;
    float free_acc_[ 3 ][ 3 ] = { { 0. } };

    double q0_, q1_, q2_, q3_;
};
#endif  // MAHONY_FILTER_HPP_