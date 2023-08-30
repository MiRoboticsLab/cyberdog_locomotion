#ifndef BODY_SWING_TRAJECTORY_HPP_
#define BODY_SWING_TRAJECTORY_HPP_

#include <iostream>

#include "cpp_types.hpp"

/**
 * @brief Body swing trajectory class
 *
 */
template < typename T > class BodySwingTrajectory {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BodySwingTrajectory() {
        initial_position_.setZero();
        final_position_.setZero();
        mid_position1_.setZero();
        mid_position2_.setZero();
        position_.setZero();
        velocity_.setZero();
        acceleration_.setZero();
        stability_ = 0;
    }

    /**
     * @brief Set the starting location of the body
     *
     * @param initial_position the initial body position
     */
    void SetInitialPosition( Vec6< T > initial_position ) {
        initial_position_ = initial_position;
    }

    /**
     * @brief Set the desired final position of the body
     *
     * @param final_position the final body posiiton
     */
    void SetFinalPosition( Vec6< T > final_position ) {
        final_position_ = final_position;
    }

    /**
     * @brief Set the Stability object
     *
     * @param stability
     */
    void SetStability( T stability ) {
        stability_ = stability;
    }

    void ComputeSwingTrajectoryBezier( T phase, T swingTime );
    void ComputeSwingTrajectoryBezierWalking( T phase, T swingTime );

    /**
     * @brief Get the body position at the current point along the swing
     *
     * @return the body position
     */
    Vec6< T > GetPosition() {
        return position_;
    }

    /**
     * @brief Get the body velocity at the current point along the swing
     *
     * @return the body velocity
     */
    Vec6< T > GetVelocity() {
        return velocity_;
    }

    /**
     * @brief Get the body acceleration at the current point along the swin
     *
     * @return the body acceleration
     */
    Vec6< T > GetAcceleration() {
        return acceleration_;
    }

private:
    // order is  rpy -> xyz
    Vec6< T > initial_position_, final_position_, mid_position1_, mid_position2_, position_, velocity_, acceleration_;
    T         stability_;
};

#endif  // BODY_SWING_TRAJECTORY_HPP_
