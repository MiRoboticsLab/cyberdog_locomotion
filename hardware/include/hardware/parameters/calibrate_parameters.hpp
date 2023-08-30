#ifndef CALIBRATE_PARAMETERS_HPP_
#define CALIBRATE_PARAMETERS_HPP_

#include "control_parameters/control_parameters.hpp"

/**
 * @brief This class containsall all IMU calibration parameters.
 *
 */
class IMUCalibrateParameters : public ControlParameters {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct a new IMUCalibrateParameters object
     *
     */
    IMUCalibrateParameters()
        : ControlParameters( "imu-parameters" ), INIT_PARAMETER( id_high ), INIT_PARAMETER( id_low ), INIT_PARAMETER( Ta ), INIT_PARAMETER( a_bias ), INIT_PARAMETER( Tg ), INIT_PARAMETER( w_bias ),
          INIT_PARAMETER( w0_bias ), INIT_PARAMETER( inrun_w_bias ) {}

    DECLARE_PARAMETER( s64, id_high );
    DECLARE_PARAMETER( s64, id_low );

    DECLARE_PARAMETER( Mat3< double >, Ta );
    DECLARE_PARAMETER( Vec3< double >, a_bias );
    DECLARE_PARAMETER( Mat3< double >, Tg );
    DECLARE_PARAMETER( Vec3< double >, w_bias );

    // w0_bias got by rotor, but not very reliable
    DECLARE_PARAMETER( Vec3< double >, w0_bias );

    // Online calibration w bias
    DECLARE_PARAMETER( Vec3< double >, inrun_w_bias );
};

/**
 * @brief This class contains position calibration parameters.
 *
 */
class PosCalibrateParameters : public ControlParameters {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct a new Pos Calibrate Parameters object
     *
     */
    PosCalibrateParameters() : ControlParameters( "pose-parameters" ), INIT_PARAMETER( init_acc ) {}

    DECLARE_PARAMETER( Vec3< double >, init_acc );
};

/**
 * @brief This class contains speed compensation parameters.
 *
 */
class SpeedCalibrateParameters : public ControlParameters {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct a new Speed Calibrate Parameters object
     *
     */
    SpeedCalibrateParameters()
        : ControlParameters( "speed-parameters" ), INIT_PARAMETER( speed_offset_trot_24_16 ), INIT_PARAMETER( speed_offset_trot_10_5 ), INIT_PARAMETER( speed_offset_trot_8_3 ),

          INIT_PARAMETER( speed_offset_trot_10_4 ), INIT_PARAMETER( speed_offset_trot_follow ), INIT_PARAMETER( speed_offset_trot_medium ), INIT_PARAMETER( speed_offset_trot_slow ),
          INIT_PARAMETER( speed_offset_trot_fast ), INIT_PARAMETER( speed_offset_ballet ), INIT_PARAMETER( speed_offset_pronk ), INIT_PARAMETER( speed_offset_bound ),
          INIT_PARAMETER( speed_offset_walk ), INIT_PARAMETER( se_ori_cali_offset ), INIT_PARAMETER( se_ori_cali_gain ), INIT_PARAMETER( speed_offset_gait1 ), INIT_PARAMETER( speed_offset_gait2 ),
          INIT_PARAMETER( speed_offset_gait3 ), INIT_PARAMETER( speed_offset_gait4 ) {}

    DECLARE_PARAMETER( Vec3< double >, speed_offset_trot_24_16 );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_trot_10_5 );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_trot_8_3 );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_trot_10_4 );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_trot_follow );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_trot_medium );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_trot_slow );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_trot_fast );

    DECLARE_PARAMETER( Vec3< double >, speed_offset_ballet );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_pronk );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_bound );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_walk );

    DECLARE_PARAMETER( Vec3< double >, se_ori_cali_offset );
    DECLARE_PARAMETER( Vec3< double >, se_ori_cali_gain );

    DECLARE_PARAMETER( Vec3< double >, speed_offset_gait1 );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_gait2 );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_gait3 );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_gait4 );
};

/**
 * @brief This class contains all the joints angle calibration parameters.
 *
 */
class JointsCalibrateParameters : public ControlParameters {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct a new joints Parameters object
     *
     */
    JointsCalibrateParameters() : ControlParameters( "joints-calibrate-parameters" ), INIT_PARAMETER( abad_calibrate ), INIT_PARAMETER( hip_calibrate ), INIT_PARAMETER( knee_calibrate ) {}

    DECLARE_PARAMETER( Vec4< double >, abad_calibrate )
    DECLARE_PARAMETER( Vec4< double >, hip_calibrate )
    DECLARE_PARAMETER( Vec4< double >, knee_calibrate )
};

#endif  // CALIBRATE_PARAMETERS_HPP_
