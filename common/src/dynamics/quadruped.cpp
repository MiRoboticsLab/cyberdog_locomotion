#include "dynamics/quadruped.hpp"
#include "dynamics/spatial.hpp"
#include "math/orientation_tools.hpp"

using namespace ori;
using namespace spatial;

/**
 * @brief Build a FloatingBaseModel of the quadruped.
 *
 * @param model FloatingBaseModel of the quadruped
 * @return true if success
 * @return false if fail
 */
template < typename T > bool Quadruped< T >::BuildModel( FloatingBaseModel< T >& model ) {
    // we assume the cyberdog's body (not including rotors) can be modeled as a
    // uniformly distributed box.
    Vec3< T > body_dims( body_length_, body_width_, body_height_ );

    model.AddBase( body_inertia_ );
    // add contact for the cyberdog's body
    model.AddGroundContactBoxPoints( 5, body_dims );

    const int base_id   = 5;
    int       body_id   = base_id;
    T         side_sign = -1;

    Mat3< T > I3 = Mat3< T >::Identity();

    // loop over 4 legs
    for ( int leg_id = 0; leg_id < 4; leg_id++ ) {
        body_id++;
        Mat6< T > xtree_abad       = CreateSXform( I3, WithLegSigns< T >( abad_location_, leg_id ) );
        Mat6< T > xtree_abad_rotor = CreateSXform( I3, WithLegSigns< T >( abad_rotor_location_, leg_id ) );

        if ( side_sign < 0 ) {
            model.AddBody( abad_inertia_.FlipAlongAxis( CoordinateAxis::Y ), abad_rotor_inertia_.FlipAlongAxis( CoordinateAxis::Y ), abad_gear_ratio_, base_id, JointType::kRevolute, CoordinateAxis::X,
                           xtree_abad, xtree_abad_rotor );
        }
        else {
            model.AddBody( abad_inertia_, abad_rotor_inertia_, abad_gear_ratio_, base_id, JointType::kRevolute, CoordinateAxis::X, xtree_abad, xtree_abad_rotor );
        }

        // Hip Joint
        body_id++;
        Mat6< T > xtree_hip       = CreateSXform( CoordinateRotation< T >( CoordinateAxis::Z, T( M_PI ) ), WithLegSigns< T >( hip_location_, leg_id ) );
        Mat6< T > xtree_hip_rotor = CreateSXform( CoordinateRotation< T >( CoordinateAxis::Z, T( M_PI ) ), WithLegSigns< T >( hip_rotor_location_, leg_id ) );
        if ( side_sign < 0 ) {
            model.AddBody( hip_inertia_.FlipAlongAxis( CoordinateAxis::Y ), hip_rotor_inertia_.FlipAlongAxis( CoordinateAxis::Y ), hip_gear_ratio_, body_id - 1, JointType::kRevolute,
                           CoordinateAxis::Y, xtree_hip, xtree_hip_rotor );
        }
        else {
            model.AddBody( hip_inertia_, hip_rotor_inertia_, hip_gear_ratio_, body_id - 1, JointType::kRevolute, CoordinateAxis::Y, xtree_hip, xtree_hip_rotor );
        }

        // add knee ground contact point
        // model.AddGroundContactPoint( body_id, Vec3< T >( 0, 0, -hip_link_length_ ) );
        model.AddGroundContactPoint( body_id, Vec3< T >( 0, 0, -hip_link_length_ ) );

        // Knee Joint
        body_id++;
        Mat6< T > xtree_knee       = CreateSXform( I3, knee_location_ );
        Mat6< T > xtree_knee_rotor = CreateSXform( I3, knee_rotor_location_ );
        if ( side_sign < 0 ) {
            model.AddBody( knee_inertia_.FlipAlongAxis( CoordinateAxis::Y ), knee_rotor_inertia_.FlipAlongAxis( CoordinateAxis::Y ), knee_gear_ratio_, body_id - 1, JointType::kRevolute,
                           CoordinateAxis::Y, xtree_knee, xtree_knee_rotor );

            model.AddGroundContactPoint( body_id, Vec3< T >( 0, knee_link_y_offset_, -knee_link_length_ ), true );
        }
        else {
            model.AddBody( knee_inertia_, knee_rotor_inertia_, knee_gear_ratio_, body_id - 1, JointType::kRevolute, CoordinateAxis::Y, xtree_knee, xtree_knee_rotor );

            model.AddGroundContactPoint( body_id, Vec3< T >( 0, -knee_link_y_offset_, -knee_link_length_ ), true );
        }

        // add foot
        // model.AddGroundContactPoint(body_id, Vec3<T>(0, 0, -knee_link_length_), true);

        side_sign *= -1;
    }

    // Making simulation more real, add 1 contact point (knee-protection rubber) on each leg
    for ( int leg_id = 0; leg_id < 4; leg_id++ ) {
        if ( side_sign < 0 ) {
            model.AddGroundContactPoint( 8 + 3 * leg_id, Vec3< T >( knee_rubber_, 0, 0 ) );
        }
        else {
            model.AddGroundContactPoint( 8 + 3 * leg_id, Vec3< T >( knee_rubber_, 0, 0 ) );
        }
        side_sign *= -1;
    }

    // Making simulation more real, add 2 contact points (hip cover) on each leg
    for ( int leg_id = 0; leg_id < 4; leg_id++ ) {
        if ( side_sign < 0 ) {
            model.AddGroundContactPoint( 7 + 3 * leg_id, Vec3< T >( 0, hip_cover_location_, 0 ) );
            model.AddGroundContactPoint( 7 + 3 * leg_id, Vec3< T >( 0, hip_cover_location_, -hip_link_length_ ) );
        }
        else {
            model.AddGroundContactPoint( 7 + 3 * leg_id, Vec3< T >( 0, -hip_cover_location_, 0 ) );
            model.AddGroundContactPoint( 7 + 3 * leg_id, Vec3< T >( 0, -hip_cover_location_, -hip_link_length_ ) );
        }
        side_sign *= -1;
    }

    // Making simulation more real, add 4 contact points ( 2 for nose, 2 for ear )
    model.AddGroundContactPoint( 5, Vec3< T >( head_nose_location_[ 0 ], head_nose_location_[ 1 ], head_nose_location_[ 2 ] ) );
    model.AddGroundContactPoint( 5, Vec3< T >( head_nose_location_[ 0 ], -head_nose_location_[ 1 ], head_nose_location_[ 2 ] ) );
    model.AddGroundContactPoint( 5, Vec3< T >( head_ear_location_[ 0 ], head_ear_location_[ 1 ], head_ear_location_[ 2 ] ) );
    model.AddGroundContactPoint( 5, Vec3< T >( head_ear_location_[ 0 ], -head_ear_location_[ 1 ], head_ear_location_[ 2 ] ) );

    Vec3< T > g( 0, 0, -9.81 );
    model.SetGravity( g );

    model.SetAbadTorqueLimit( abad_motor_tau_max_ * abad_gear_ratio_ );
    model.SetHipTorqueLimit( hip_motor_tau_max_ * hip_gear_ratio_ );
    model.SetKneeTorqueLimit( knee_motor_tau_max_ * knee_gear_ratio_ );

    return true;
}

/**
 * @brief Build a FloatingBaseModel of the quadruped.
 *
 * @return FloatingBaseModel of the quadruped
 */
template < typename T > FloatingBaseModel< T > Quadruped< T >::BuildModel() {
    FloatingBaseModel< T > model;
    BuildModel( model );
    return model;
}

/**
 * @brief Flip signs of elements of a vector V depending on which leg it belongs to.
 *
 * @param v A 3x1 Eigen matrix
 * @param leg_id ID of the leg to specify the sign for
 * @return A 3D vector with specified leg signs
 */
template < typename T, typename T2 > Vec3< T > WithLegSigns( const Eigen::MatrixBase< T2 >& v, int leg_id ) {
    static_assert( T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 matrix" );
    switch ( leg_id ) {
    case 0:
        return Vec3< T >( v[ 0 ], -v[ 1 ], v[ 2 ] );
    case 1:
        return Vec3< T >( v[ 0 ], v[ 1 ], v[ 2 ] );
    case 2:
        return Vec3< T >( -v[ 0 ], -v[ 1 ], v[ 2 ] );
    case 3:
        return Vec3< T >( -v[ 0 ], v[ 1 ], v[ 2 ] );
    default:
        throw std::runtime_error( "Invalid leg id!" );
    }
}

/**
 * @brief Build actuator models for a leg.
 *
 * @return Actuator models of the leg
 */
template < typename T > std::vector< ActuatorModel< T > > Quadruped< T >::BuildActuatorModels() {
    std::vector< ActuatorModel< T > > models;
    models.emplace_back( abad_gear_ratio_, motor_KT_, motor_R_, battery_V_, joint_damping_, joint_dry_friction_, abad_motor_tau_max_, abad_n1_, abad_n2_, abad_n3_ );
    models.emplace_back( hip_gear_ratio_, motor_KT_, motor_R_, battery_V_, joint_damping_, joint_dry_friction_, hip_motor_tau_max_, hip_n1_, hip_n2_, hip_n3_ );
    models.emplace_back( knee_gear_ratio_, motor_KT_, motor_R_, battery_V_, joint_damping_, joint_dry_friction_, knee_motor_tau_max_, knee_n1_, knee_n2_, knee_n3_ );
    return models;
}

template class Quadruped< double >;
template class Quadruped< float >;
