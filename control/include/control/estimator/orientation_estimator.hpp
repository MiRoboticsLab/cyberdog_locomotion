#ifndef ORIENTATION_ESTIMATOR_HPP_
#define ORIENTATION_ESTIMATOR_HPP_

#include <lcm/lcm-cpp.hpp>
#include <thread>

#include "controllers/state_estimator_container.hpp"

/**
 * @brief All Orientation Estimation Algorithms
 *
 * This file will contain all orientation algorithms.
 * Orientation estimators should compute:
 * - orientation: a quaternion representing orientation
 * - world2body_rotation_matrix: coordinate transformation matrix (satisfies velocity_in_body_frame = Rbody * velocity_in_world_frame)
 * - angular_velocity_in_body_frame: angular velocity in body frame
 * - angular_velocity_in_world_frame: angular velocity in world frame
 * - rpy: roll pitch yaw
 *
 * "Cheater" estimator for orientation which always returns the correct value in simulation
 *
 */
template < typename T > class CheaterOrientationEstimator : public GenericEstimator< T > {
public:
    virtual void Run();
    virtual void Setup() {}
};

/**
 * @brief Estimator for the VectorNav IMU.
 *
 * The VectorNav provides an orientation already and
 * we just return that.
 *
 */
template < typename T > class VectorNavOrientationEstimator : public GenericEstimator< T > {
public:
    virtual void Run();
    virtual void Setup() {}
    VectorNavOrientationEstimator();

protected:
    bool      first_visit_ = true;
    Quat< T > ori_ini_inv_;
    Quat< T > ori_cali_;
    void      CaliOriFromVel();
};

#endif  // ORIENTATION_ESTIMATOR_HPP_
