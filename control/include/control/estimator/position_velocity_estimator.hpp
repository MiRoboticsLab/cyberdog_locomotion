#ifndef POSITION_VELOCITY_ESTIMATOR_HPP_
#define POSITION_VELOCITY_ESTIMATOR_HPP_

#include <lcm/lcm-cpp.hpp>
#include <thread>

#include "controllers/state_estimator_container.hpp"

/**
 * @brief Position and velocity estimator based on a Kalman Filter.
 * This is the algorithm used in Cyberdog2.
 * PositionVelocityEstimators should compute:
 * - body position/velocity in world/body frames
 * - foot positions/velocities in body/world frame
 *
 */
template < typename T > class LinearKFPositionVelocityEstimator : public GenericEstimator< T > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinearKFPositionVelocityEstimator();
    virtual void Run();
    virtual void Setup();

private:
    void GetCurPosByLeg();

    Eigen::Matrix< T, 18, 1 >  xhat_;
    Eigen::Matrix< T, 12, 1 >  ps_;
    Eigen::Matrix< T, 12, 1 >  vs_;
    Eigen::Matrix< T, 18, 18 > A_;
    Eigen::Matrix< T, 18, 18 > Q0_;
    Eigen::Matrix< T, 18, 18 > P_;
    Eigen::Matrix< T, 28, 28 > R0_;
    Eigen::Matrix< T, 18, 3 >  B_;
    Eigen::Matrix< T, 28, 18 > C_;

    bool is_four_leg_stand_ = true;
    int  full_contact_iter_ = 0;

    Vec3< T > cur_pos_;
    Vec3< T > ini_pos_;

    Vec3< T > dp_relative_previous_[ 4 ];
};

/**
 * @brief "Cheater" position and velocity estimator which will return the correct position and
 * velocity when running in simulation.
 * 
 */
template < typename T > class CheaterPositionVelocityEstimator : public GenericEstimator< T > {
public:
    virtual void Run();
    virtual void Setup() {}
};

#endif  // POSITION_VELOCITY_ESTIMATOR_HPP_
