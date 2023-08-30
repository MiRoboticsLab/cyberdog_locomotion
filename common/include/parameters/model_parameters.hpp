#ifndef MODEL_PARAMETERS_HPP_
#define MODEL_PARAMETERS_HPP_

#include "control_parameters/control_parameters.hpp"

/**
 * @brief Dynamic model parameters of robot
 * 
 */
class ModelParameters : public ControlParameters {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ModelParameters()
        : ControlParameters( "model-parameters" ),

          INIT_PARAMETER( abad_link_length ), INIT_PARAMETER( hip_link_length ), INIT_PARAMETER( knee_link_length ),

          INIT_PARAMETER( body_length ), INIT_PARAMETER( body_width ),

          INIT_PARAMETER( body_mass ), INIT_PARAMETER( body_com ), INIT_PARAMETER( body_inertia ),

          INIT_PARAMETER( abad_mass ), INIT_PARAMETER( abad_com ), INIT_PARAMETER( abad_inertia ),

          INIT_PARAMETER( hip_mass ), INIT_PARAMETER( hip_com ), INIT_PARAMETER( hip_inertia ),

          INIT_PARAMETER( knee_mass ), INIT_PARAMETER( knee_com ), INIT_PARAMETER( knee_inertia ),

          INIT_PARAMETER( abad_torque_max ), INIT_PARAMETER( hip_torque_max ), INIT_PARAMETER( knee_torque_max ),

          INIT_PARAMETER( rotor_mass ), INIT_PARAMETER( rotor_inertia_z ),

          INIT_PARAMETER( foot_radius ),

          INIT_PARAMETER( abad_gear_ratio ), INIT_PARAMETER( hip_gear_ratio ), INIT_PARAMETER( knee_gear_ratio ),

          INIT_PARAMETER( abad_lower_bound ), INIT_PARAMETER( abad_upper_bound ),

          INIT_PARAMETER( front_hip_lower_bound ), INIT_PARAMETER( front_hip_upper_bound ),

          INIT_PARAMETER( rear_hip_lower_bound ), INIT_PARAMETER( rear_hip_upper_bound ),

          INIT_PARAMETER( knee_lower_bound ), INIT_PARAMETER( knee_upper_bound ),

          INIT_PARAMETER( abad_rotor_location ), INIT_PARAMETER( hip_rotor_location ), INIT_PARAMETER( knee_rotor_location ),
          
          INIT_PARAMETER( knee_rubber ),  

          INIT_PARAMETER( nose_location ), INIT_PARAMETER( ear_location ),
          
          INIT_PARAMETER( hip_cover_location ) {}

    DECLARE_PARAMETER( double, abad_link_length );
    DECLARE_PARAMETER( double, hip_link_length );
    DECLARE_PARAMETER( double, knee_link_length );

    DECLARE_PARAMETER( double, body_length );
    DECLARE_PARAMETER( double, body_width );

    DECLARE_PARAMETER( double, body_mass );
    DECLARE_PARAMETER( Vec3< double >, body_com );
    DECLARE_PARAMETER( Vec9< double >, body_inertia );
    DECLARE_PARAMETER( double, abad_mass );
    DECLARE_PARAMETER( Vec3< double >, abad_com );
    DECLARE_PARAMETER( Vec9< double >, abad_inertia );
    DECLARE_PARAMETER( double, hip_mass );
    DECLARE_PARAMETER( Vec3< double >, hip_com );
    DECLARE_PARAMETER( Vec9< double >, hip_inertia );
    DECLARE_PARAMETER( double, knee_mass );
    DECLARE_PARAMETER( Vec3< double >, knee_com );
    DECLARE_PARAMETER( Vec9< double >, knee_inertia );
    DECLARE_PARAMETER( double, abad_torque_max );
    DECLARE_PARAMETER( double, hip_torque_max );
    DECLARE_PARAMETER( double, knee_torque_max );
    DECLARE_PARAMETER( double, rotor_mass );
    DECLARE_PARAMETER( Vec9< double >, rotor_inertia_z );
    DECLARE_PARAMETER( double, foot_radius );
    DECLARE_PARAMETER( double, abad_gear_ratio );
    DECLARE_PARAMETER( double, hip_gear_ratio );
    DECLARE_PARAMETER( double, knee_gear_ratio );
    DECLARE_PARAMETER( double, abad_lower_bound );
    DECLARE_PARAMETER( double, abad_upper_bound );
    DECLARE_PARAMETER( double, front_hip_lower_bound );
    DECLARE_PARAMETER( double, front_hip_upper_bound );
    DECLARE_PARAMETER( double, rear_hip_lower_bound );
    DECLARE_PARAMETER( double, rear_hip_upper_bound );
    DECLARE_PARAMETER( double, knee_lower_bound );
    DECLARE_PARAMETER( double, knee_upper_bound );
    DECLARE_PARAMETER( double, abad_rotor_location );
    DECLARE_PARAMETER( double, hip_rotor_location );
    DECLARE_PARAMETER( double, knee_rotor_location );

    DECLARE_PARAMETER( double, knee_rubber );
    DECLARE_PARAMETER( Vec3< double >, nose_location );
    DECLARE_PARAMETER( Vec3< double >, ear_location );
    DECLARE_PARAMETER( double, hip_cover_location );
};

#endif  // MODEL_PARAMETERS_HPP_