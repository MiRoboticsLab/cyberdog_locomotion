#ifndef IMU_TYPES_HPP_
#define IMU_TYPES_HPP_

#include "cpp_types.hpp"

/**
 * @brief Cyberdog2's IMU
 *
 */
struct VectorNavData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VectorNavData() {
        accelerometer = Vec3< float >( 0, 0, 9.81 );
        gyro.setZero();
        quat = Quat< float >( 0, 0, 0, 1 );
    }

    Vec3< float > accelerometer;
    Vec3< float > gyro;
    Quat< float > quat;
    // todo is there status for the vectornav?
};

/**
 * @brief "Cheater" state sent to the robot from simulator
 *
 */
template < typename T > struct CheaterState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Quat< T > orientation;
    Vec3< T > position;
    Vec3< T > angular_velocity_in_body_frame;
    Vec3< T > velocity_in_body_frame;
    Vec3< T > acceleration;
};

#endif  // IMU_TYPES_HPP_
