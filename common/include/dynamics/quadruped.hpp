#ifndef QUADRUPED_HPP_
#define QUADRUPED_HPP_

#include <vector>

#include <Eigen/StdVector>

#include "dynamics/actuator_model.hpp"
#include "dynamics/floating_base_model.hpp"
#include "dynamics/spatial_inertia.hpp"

using std::vector;

/**
 * @brief Basic parameters for a cyberdog2-shaped robot.
 *
 */
namespace cyberdog2 {
constexpr size_t kNumActJoint = 12;
constexpr size_t kNumQ        = 19;
constexpr size_t kDimConfig   = 18;
constexpr size_t kNumLeg      = 4;
constexpr size_t kNumLegJoint = 3;
}  // namespace cyberdog2

/**
 * @brief Link indices for cyberdog2-shaped robots.
 *
 */
namespace linkID {
constexpr size_t kFr = 9;   // Front Right Foot
constexpr size_t kFl = 11;  // Front Left Foot
constexpr size_t kHr = 13;  // Hind Right Foot
constexpr size_t kHl = 15;  // Hind Left Foot

constexpr size_t kFrAbd = 2;  // Front Right Abduction
constexpr size_t kFlAbd = 0;  // Front Left Abduction
constexpr size_t kHrAbd = 3;  // Hind Right Abduction
constexpr size_t kHlAbd = 1;  // Hind Left Abduction
}  // namespace linkID

/**
 * @brief Representation of a quadruped robot's physical properties.
 *        When viewed from the top, the quadruped's legs are:
 *        FRONT
 *        2 1   RIGHT
 *        4 3
 *        BACK
 *
 */
template < typename T > class Quadruped {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RobotType           robot_type_;
    RobotAppearanceType robot_appearance_type_;
    T                   body_length_, body_width_, body_height_, body_mass_;
    T                   abad_gear_ratio_, hip_gear_ratio_, knee_gear_ratio_;
    T                   abad_n1_, abad_n2_, abad_n3_, hip_n1_, hip_n2_, hip_n3_, knee_n1_, knee_n2_, knee_n3_;
    T                   abad_link_length_, hip_link_length_, knee_link_length_, knee_link_y_offset_, max_leg_length_;
    T                   motor_KT_, motor_R_, battery_V_;
    T                   abad_motor_tau_max_, hip_motor_tau_max_, knee_motor_tau_max_;
    T                   joint_damping_, joint_dry_friction_;
    SpatialInertia< T > abad_inertia_, hip_inertia_, knee_inertia_, abad_rotor_inertia_, hip_rotor_inertia_, knee_rotor_inertia_, body_inertia_;
    Vec3< T >           abad_location_, abad_rotor_location_, hip_location_, hip_rotor_location_, knee_location_, knee_rotor_location_;
    T                   abad_lower_bound_, abad_upper_bound_, front_hip_lower_bound_, front_hip_upper_bound_, rear_hip_lower_bound_, rear_hip_upper_bound_, knee_lower_bound_, knee_upper_bound_;
    T                   knee_rubber_, hip_cover_location_;
    Vec3< T >           head_nose_location_, head_ear_location_;

    FloatingBaseModel< T >            BuildModel();
    bool                              BuildModel( FloatingBaseModel< T >& model );
    std::vector< ActuatorModel< T > > BuildActuatorModels();

    /**
     * @brief Get if the i-th leg is on the left (+) or right (-) of the robot.
     *
     * @param leg The leg index
     * @return The side sign (-1 for right legs, +1 for left legs)
     */
    static T GetSideSign( int leg ) {
        const T side_signs[ 4 ] = { -1, 1, -1, 1 };
        assert( leg >= 0 && leg < 4 );
        return side_signs[ leg ];
    }

    /**
     * @brief Get location of the hip for the given leg in robot frame.
     *
     * @param leg The leg index
     * @return The location of the hip
     */
    Vec3< T > GetHipLocation( int leg ) {
        assert( leg >= 0 && leg < 4 );
        Vec3< T > p_hip( ( leg == 0 || leg == 1 ) ? abad_location_( 0 ) : -abad_location_( 0 ), ( leg == 1 || leg == 3 ) ? abad_location_( 1 ) : -abad_location_( 1 ), abad_location_( 2 ) );
        return p_hip;
    }
};

template < typename T, typename T2 > Vec3< T > WithLegSigns( const Eigen::MatrixBase< T2 >& v, int leg_id );

#endif  // QUADRUPED_HPP_
