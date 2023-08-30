#ifndef ROBOT_PARAMETERS_HPP_
#define ROBOT_PARAMETERS_HPP_

#include "control_parameters/control_parameters.hpp"

/**
 * @brief ControlParameters shared among all robot controllers
 *
 */
class RobotControlParameters : public ControlParameters {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct RobotControlParameters
     */
    RobotControlParameters()
        : ControlParameters( "robot-parameters" ), INIT_PARAMETER( control_mode ), INIT_PARAMETER( gait_id ), INIT_PARAMETER( controller_dt ), INIT_PARAMETER( cheater_mode ),
          INIT_PARAMETER( imu_process_noise_position ), INIT_PARAMETER( imu_process_noise_velocity ), INIT_PARAMETER( foot_process_noise_position ), INIT_PARAMETER( foot_sensor_noise_position ),
          INIT_PARAMETER( foot_sensor_noise_velocity ), INIT_PARAMETER( foot_height_sensor_noise ), INIT_PARAMETER( use_rc ), INIT_PARAMETER( imu_bias_estimation ),
          INIT_PARAMETER( imu_adaptive_gain ), INIT_PARAMETER( gain_acc ), INIT_PARAMETER( speed_offset_trot_10_4 ), INIT_PARAMETER( speed_offset_trot_follow ),
          INIT_PARAMETER( speed_offset_trot_medium ), INIT_PARAMETER( speed_offset_trot_slow ), INIT_PARAMETER( speed_offset_trot_fast ), INIT_PARAMETER( speed_offset_trot_24_16 ),
          INIT_PARAMETER( speed_offset_trot_8_3 ), INIT_PARAMETER( speed_offset_ballet ), INIT_PARAMETER( speed_offset_bound ), INIT_PARAMETER( speed_offset_pronk ),
          INIT_PARAMETER( se_ori_cali_gain ), INIT_PARAMETER( se_ori_cali_offset ), INIT_PARAMETER( filter_type ), INIT_PARAMETER( delt_roll ), INIT_PARAMETER( delt_pitch ),
          INIT_PARAMETER( complementaryfilter_source ), INIT_PARAMETER( lcm_debug_switch ) {}

    DECLARE_PARAMETER( s64, control_mode )
    DECLARE_PARAMETER( s64, gait_id );
    DECLARE_PARAMETER( double, controller_dt )

    // state estimator
    DECLARE_PARAMETER( s64, cheater_mode )
    DECLARE_PARAMETER( double, imu_process_noise_position )
    DECLARE_PARAMETER( double, imu_process_noise_velocity )
    DECLARE_PARAMETER( double, foot_process_noise_position )
    DECLARE_PARAMETER( double, foot_sensor_noise_position )
    DECLARE_PARAMETER( double, foot_sensor_noise_velocity )
    DECLARE_PARAMETER( double, foot_height_sensor_noise )

    DECLARE_PARAMETER( s64, use_rc )

    DECLARE_PARAMETER( s64, imu_bias_estimation )
    DECLARE_PARAMETER( s64, imu_adaptive_gain )
    DECLARE_PARAMETER( double, gain_acc )

    DECLARE_PARAMETER( Vec3< double >, speed_offset_trot_10_4 );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_trot_follow );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_trot_medium );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_trot_slow );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_trot_fast );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_trot_24_16 );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_trot_8_3 );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_ballet );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_bound );
    DECLARE_PARAMETER( Vec3< double >, speed_offset_pronk );
    DECLARE_PARAMETER( Vec3< double >, se_ori_cali_gain );
    DECLARE_PARAMETER( Vec3< double >, se_ori_cali_offset );

    DECLARE_PARAMETER( s64, filter_type )

    DECLARE_PARAMETER( double, delt_roll )
    DECLARE_PARAMETER( double, delt_pitch )
    DECLARE_PARAMETER( double, complementaryfilter_source )
    DECLARE_PARAMETER( s64, lcm_debug_switch )
};

#endif  // ROBOT_PARAMETERS_HPP_
