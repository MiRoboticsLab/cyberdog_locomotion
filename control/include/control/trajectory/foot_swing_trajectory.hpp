#ifndef FOOT_SWING_TRAJECTORY_HPP_
#define FOOT_SWING_TRAJECTORY_HPP_

#include "cpp_types.hpp"

/**
 * @brief Foot swing trajectory for a single foot
 *
 */
template < typename T > class FootSwingTrajectory {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Construct a new foot swing trajectory with everything set to zero
    FootSwingTrajectory() {
        initial_position_.setZero();
        final_position_.setZero();
        position_.setZero();
        velocity_.setZero();
        acceleration_.setZero();
        height_     = 0;
        depth_      = 0;
        use_bezier_ = 0;
    }

    /**
     * @brief Set the starting location of the foot
     *
     * @param initial_position : the initial foot position
     */
    void SetInitialPosition( Vec3< T > initial_position ) {
        initial_position_ = initial_position;
    }

    void SetUseBezier( T use_bezier ) {
        use_bezier_ = use_bezier;
    }

    /**
     * @brief Set the desired final position of the foot
     *
     * @param final_position : the final foot posiiton
     */
    void SetFinalPosition( Vec3< T > final_position ) {
        final_position_ = final_position;
    }

    /**
     * @brief Set the maximum height of the swing
     *
     * @param height : the maximum height of the swing, achieved halfway through the swing
     */
    void SetHeight( T height ) {
        height_ = height;
    }

    /**
     * @brief Set the middle position of the swing
     *
     * @param mid_position: the middle position of the swing, achieved halfway through the swing
     */
    void SetMidPosition( Vec3< T > mid_position ) {
        mid_position_ = mid_position;
    }

    /**
     * @brief Set the maximum depth of the swing
     *
     * @param depth : the maximum depth of the swing
     */
    void SetDepth( T depth ) {
        depth_ = depth;
    }

    void ComputeSwingTrajectoryBezier( T phase, T swing_time );
    void ComputeSwingTrajectoryPolynomial( T phase, T swing_time );
    void ComputeSwingTrajectoryMidPosBezier( T phase, T swing_time );
    void ComputeSwingTrajectoryBezier_downstairs( T phase, T swing_time );
    void ComputeSwingTrajectoryTstair( T phase, T swing_time );
    void ComputeSwingTrajectoryTsimilar( T phase, T swing_time );

    /**
     * @brief Get the foot position at the current point along the swing
     *
     * @return : the foot position
     */
    Vec3< T > GetPosition() {
        return position_;
    }

    /**
     * @brief Get the foot position at the current point along the swing
     *
     * @return : the foot position
     */
    Vec3< T > GetInitialPosition() {
        return initial_position_;
    }

    /**
     * @brief Get the foot velocity at the current point along the swing
     *
     * @return : the foot velocity
     */
    Vec3< T > GetVelocity() {
        return velocity_;
    }

    /**
     * @brief Get the foot acceleration at the current point along the swing
     *
     * @return : the foot acceleration
     */
    Vec3< T > GetAcceleration() {
        return acceleration_;
    }

private:
    Vec3< T > initial_position_, final_position_, position_, velocity_, acceleration_, mid_position_;
    T         height_;
    T         depth_;
    T         use_bezier_;
};

#endif  // FOOT_SWING_TRAJECTORY_HPP_
