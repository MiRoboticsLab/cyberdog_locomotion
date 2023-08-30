#ifndef CYBERDOG_HPP_
#define CYBERDOG_HPP_

#include "Configuration.h"
#include "dynamics/floating_base_model.hpp"
#include "dynamics/quadruped.hpp"
#include "parameters/model_parameters.hpp"

/**
 * @brief Generate a quadruped model of Cyberdog
 *
 * @param robot_type Robot type
 * @return A quadruped model of Cyberdog
 */
template < typename T > Quadruped< T > BuildCyberdog( const RobotType& robot_type, const RobotAppearanceType& appearance_type = RobotAppearanceType::CURVED ) {
    Quadruped< T >  cyberdog;
    ModelParameters model_params;
    if ( robot_type == RobotType::CYBERDOG2 ) {
        if ( appearance_type == RobotAppearanceType::CURVED )
            model_params.InitializeFromYamlFile( THIS_COM "common/config/cyberdog2-description.yaml" );
        else
            model_params.InitializeFromYamlFile( THIS_COM "common/config/cyberdog2-angular-description.yaml" );
    }
    else
        model_params.InitializeFromYamlFile( THIS_COM "common/config/cyberdog-description.yaml" );
    if ( !model_params.IsFullyInitialized() ) {
        printf( "Failed to initialize all model parameters\n" );
        exit( 1 );
    }
    cyberdog.robot_type_ = robot_type;

    cyberdog.robot_appearance_type_ = appearance_type;

    if ( robot_type == RobotType::CYBERDOG2 ) {
        cyberdog.abad_n1_ = 11.5192, cyberdog.abad_n2_ = 30.9970, cyberdog.abad_n3_ = 60.0;
        cyberdog.hip_n1_ = 11.5192, cyberdog.hip_n2_ = 30.9970, cyberdog.hip_n3_ = 60.0;
        cyberdog.knee_n1_ = 11.5192, cyberdog.knee_n2_ = 30.9970, cyberdog.knee_n3_ = 60.0;
    }
    else {
        cyberdog.abad_n1_ = 11.4145, cyberdog.abad_n2_ = 20.9136, cyberdog.abad_n3_ = 60.0;
        cyberdog.hip_n1_ = 11.4145, cyberdog.hip_n2_ = 20.9136, cyberdog.hip_n3_ = 60.0;
        cyberdog.knee_n1_ = 11.4145, cyberdog.knee_n2_ = 20.9136, cyberdog.knee_n3_ = 60.0;
    }

    cyberdog.body_mass_   = model_params.body_mass;        // 3.3
    cyberdog.body_length_ = model_params.body_length * 2;  // 0.23536*2;//0.19 * 2;
    cyberdog.body_width_  = model_params.body_width * 2;   // 0.05*2;// 0.049 * 2;//

    cyberdog.body_height_    = 0.05 * 2;
    cyberdog.abad_gear_ratio_ = model_params.abad_gear_ratio;
    cyberdog.hip_gear_ratio_  = model_params.hip_gear_ratio;
    cyberdog.knee_gear_ratio_    = model_params.knee_gear_ratio;
    cyberdog.abad_link_length_   = model_params.abad_link_length;                            // 0.10715;//0.062;//
    cyberdog.hip_link_length_    = model_params.hip_link_length;                             // 0.200;//0.211;
    cyberdog.knee_link_y_offset_ = 0;                                                      // 0.004;
    cyberdog.knee_link_length_   = model_params.knee_link_length + model_params.foot_radius;  // 0.217;//0.20;
    cyberdog.max_leg_length_     = ( model_params.hip_link_length + model_params.knee_link_length );

    cyberdog.abad_motor_tau_max_  = model_params.abad_torque_max / cyberdog.abad_gear_ratio_;
    cyberdog.hip_motor_tau_max_   = model_params.hip_torque_max / cyberdog.hip_gear_ratio_;
    cyberdog.knee_motor_tau_max_  = model_params.knee_torque_max / cyberdog.knee_gear_ratio_;
    cyberdog.battery_V_         = 24;
    cyberdog.motor_KT_          = .05;  // this is flux linkage * pole pairs
    cyberdog.motor_R_           = 0.173;
    cyberdog.joint_damping_     = .01;
    cyberdog.joint_dry_friction_ = .246;

    cyberdog.abad_lower_bound_     = model_params.abad_lower_bound;
    cyberdog.abad_upper_bound_     = model_params.abad_upper_bound;
    cyberdog.front_hip_lower_bound_ = model_params.front_hip_lower_bound;
    cyberdog.front_hip_upper_bound_ = model_params.front_hip_upper_bound;
    cyberdog.rear_hip_lower_bound_  = model_params.rear_hip_lower_bound;
    cyberdog.rear_hip_upper_bound_  = model_params.rear_hip_upper_bound;
    cyberdog.knee_lower_bound_     = model_params.knee_lower_bound;
    cyberdog.knee_upper_bound_     = model_params.knee_upper_bound;

    // rotor inertia if the rotor is oriented so it spins around the z-axis
    Mat3< T > rotor_inertia_z;
    rotor_inertia_z << model_params.rotor_inertia_z[ 0 ], model_params.rotor_inertia_z[ 1 ], model_params.rotor_inertia_z[ 2 ],
        model_params.rotor_inertia_z[ 3 ], model_params.rotor_inertia_z[ 4 ], model_params.rotor_inertia_z[ 5 ], model_params.rotor_inertia_z[ 6 ],
        model_params.rotor_inertia_z[ 7 ], model_params.rotor_inertia_z[ 8 ];
    Mat3< T > RY                      = CoordinateRotation< T >( CoordinateAxis::Y, M_PI / 2 );
    Mat3< T > RX                      = CoordinateRotation< T >( CoordinateAxis::X, M_PI / 2 );
    Mat3< T > rotorRotationalInertiaX = RY * rotor_inertia_z * RY.transpose();
    Mat3< T > rotorRotationalInertiaY = RX * rotor_inertia_z * RX.transpose();

    // spatial inertias
    Mat3< T > abad_inertia;
    abad_inertia << model_params.abad_inertia[ 0 ], model_params.abad_inertia[ 1 ], model_params.abad_inertia[ 2 ], model_params.abad_inertia[ 3 ],
        model_params.abad_inertia[ 4 ], model_params.abad_inertia[ 5 ], model_params.abad_inertia[ 6 ], model_params.abad_inertia[ 7 ],
        model_params.abad_inertia[ 8 ];
    Vec3< T > abad_com( model_params.abad_com[ 0 ], model_params.abad_com[ 1 ], model_params.abad_com[ 2 ] );
    SpatialInertia< T > abadInertia( model_params.abad_mass, abad_com, abad_inertia );

    Mat3< T > hip_inertia;
    hip_inertia << model_params.hip_inertia[ 0 ], model_params.hip_inertia[ 1 ], model_params.hip_inertia[ 2 ], model_params.hip_inertia[ 3 ],
        model_params.hip_inertia[ 4 ], model_params.hip_inertia[ 5 ], model_params.hip_inertia[ 6 ], model_params.hip_inertia[ 7 ],
        model_params.hip_inertia[ 8 ];
    Vec3< T >           hip_com( model_params.hip_com[ 0 ], model_params.hip_com[ 1 ], model_params.hip_com[ 2 ] );
    SpatialInertia< T > hipInertia( model_params.hip_mass, hip_com, hip_inertia );

    Mat3< T > knee_inertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << model_params.knee_inertia[ 0 ], model_params.knee_inertia[ 1 ], model_params.knee_inertia[ 2 ], model_params.knee_inertia[ 3 ],
        model_params.knee_inertia[ 4 ], model_params.knee_inertia[ 5 ], model_params.knee_inertia[ 6 ], model_params.knee_inertia[ 7 ],
        model_params.knee_inertia[ 8 ];
    knee_inertia = RY * kneeRotationalInertiaRotated * RY.transpose();
    Vec3< T >           knee_com( model_params.knee_com[ 0 ], model_params.knee_com[ 1 ], model_params.knee_com[ 2 ] );
    SpatialInertia< T > kneeInertia( model_params.knee_mass, knee_com, knee_inertia );

    Vec3< T >           rotorCOM( 0, 0, 0 );
    SpatialInertia< T > rotorInertiaX( model_params.rotor_mass, rotorCOM, rotorRotationalInertiaX );
    SpatialInertia< T > rotorInertiaY( model_params.rotor_mass, rotorCOM, rotorRotationalInertiaY );

    Mat3< T > body_inertia;
    body_inertia << model_params.body_inertia[ 0 ], model_params.body_inertia[ 1 ], model_params.body_inertia[ 2 ], model_params.body_inertia[ 3 ],
        model_params.body_inertia[ 4 ], model_params.body_inertia[ 5 ], model_params.body_inertia[ 6 ], model_params.body_inertia[ 7 ],
        model_params.body_inertia[ 8 ];
    Vec3< T >           body_com( model_params.body_com[ 0 ], model_params.body_com[ 1 ], model_params.body_com[ 2 ] );
    SpatialInertia< T > bodyInertia( cyberdog.body_mass_, body_com, body_inertia );

    cyberdog.abad_inertia_      = abadInertia;
    cyberdog.hip_inertia_       = hipInertia;
    cyberdog.knee_inertia_      = kneeInertia;
    cyberdog.abad_rotor_inertia_ = rotorInertiaX;
    cyberdog.hip_rotor_inertia_  = rotorInertiaY;
    cyberdog.knee_rotor_inertia_ = rotorInertiaY;
    cyberdog.body_inertia_      = bodyInertia;

    cyberdog.abad_rotor_location_ = Vec3< T >( cyberdog.body_length_, cyberdog.body_width_, 0 ) * 0.5 + Vec3< T >( -model_params.abad_rotor_location, 0, 0 );
    ;
    cyberdog.abad_location_      = Vec3< T >( cyberdog.body_length_, cyberdog.body_width_, 0 ) * 0.5;
    cyberdog.hip_location_       = Vec3< T >( 0, cyberdog.abad_link_length_, 0 );
    cyberdog.hip_rotor_location_  = Vec3< T >( model_params.abad_rotor_location, cyberdog.abad_link_length_ - model_params.hip_rotor_location, 0 );
    cyberdog.knee_location_      = Vec3< T >( 0, 0, -cyberdog.hip_link_length_ );
    cyberdog.knee_rotor_location_ = Vec3< T >( 0, model_params.hip_rotor_location - model_params.knee_rotor_location, 0 );

    // contact points only for c++ code
    cyberdog.knee_rubber_       = model_params.knee_rubber;
    cyberdog.head_nose_location_ = Vec3< T >( model_params.nose_location[ 0 ], model_params.nose_location[ 1 ], model_params.nose_location[ 2 ] );
    cyberdog.head_ear_location_  = Vec3< T >( model_params.ear_location[ 0 ], model_params.ear_location[ 1 ], model_params.ear_location[ 2 ] );
    cyberdog.hip_cover_location_ = model_params.hip_cover_location;

    return cyberdog;
}

#endif  // CYBERDOG_HPP_
