#ifndef CONTROL_UTILITIES_HPP_
#define CONTROL_UTILITIES_HPP_

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <unordered_map>
#include <vector>

#include "cpp_types.hpp"
#include "utilities/utilities.hpp"

/**
 * @brief This file contains utility functions used on motion controller,
 * for example, velocity and accelation limitation and so on.
 *
 */

/**
 * @brief Apply the pose meets position velocity and accelatin limits.
 *
 * @param current_pose current pose, (normally, translation or rotation)
 * @param current_velocity current velocity, (normally, angular velocity or linear velocity)
 * @param target_pose target pose, the same dimension with 'current_pose'
 * @param pose_minium minimum pose limit, this can be a minus value
 * @param pose_maxium maximum pose limit
 * @param velocity_minium minimum velocity limit, this can be a minus value
 * @param velocity_maxium maximum velocity limit
 * @param accelation_minium minimum acceleration limit, this can be a minus value
 * @param accelation_maxium maximum acceleration limit
 * @param dt controller dt, unit is second
 * @return target pose meets all limits
 */
template < typename T >
T ApplyPoseMeetVelocityAccelationLimits( const T& current_pose, const T& current_velocity, const T& target_pose, const T& pose_minium, const T& pose_maxium, const T& velocity_minium,
                                         const T& velocity_maxium, const T& accelation_minium, const T& accelation_maxium, double dt ) {
    assert( fabs( dt ) > 1e-6 );  // dt cannot be zero
    assert( pose_minium <= pose_maxium && velocity_minium <= velocity_maxium && accelation_minium <= accelation_maxium );

    T result = target_pose;

    T valid_target_val = WrapRange( target_pose, pose_minium, pose_maxium );
    // apply deadband
    if ( std::fabs( current_pose - valid_target_val ) < 0.02 ) {
        return valid_target_val;
    }

    // Firstly, check acc limit, target_pose should in the range:
    // [x0 + v0*dt + 0.5 * acc_min * dt^2, x0 + v0*dt + 0.5 * acc_max * dt^2]
    double dt_2            = dt * dt;
    T      acc_limit_root  = current_pose + current_velocity * dt;
    T      val_acc_lim_min = acc_limit_root + 0.5 * accelation_minium * dt_2;
    T      val_acc_lim_max = acc_limit_root + 0.5 * accelation_maxium * dt_2;
    result                 = WrapRange( result, val_acc_lim_min, val_acc_lim_max );

    // Secondly, check velocity limit, based on the vel_acc_lim_apply
    T val_vel_lim_min = current_pose + velocity_minium * dt;
    T val_vel_lim_max = current_pose + velocity_maxium * dt;
    result            = WrapRange( result, val_vel_lim_min, val_vel_lim_max );

    // Thirdly, check the value limit
    result = WrapRange( result, pose_minium, pose_maxium );

    return result;
}

/**
 * @brief Apply velocity meets velocity and accelatin limits.
 *
 * @param current_velocity current velocity, (normally, angular velocity or linear velocity)
 * @param target_velocity target velocity, the same dimension with 'current_velocity'
 * @param velocity_minium minimumvelocity limit, this can be a minus value
 * @param velocity_maxium maximum velocity limit
 * @param accelation_minium minimum acceleration limit, this can be a minus value
 * @param accelation_maxium maximum acceleration limit
 * @param dt controller dt, unit is second
 * @return target velocity meets all limits
 */
template < typename T >
T ApplyVelocityMeetAccelationLimit( const T& current_velocity, const T& target_velocity, const T& velocity_minium, const T& velocity_maxium, const T& accelation_minium, const T& accelation_maxium,
                                    double dt ) {
    // apply deadband
    T valid_target_vel = WrapRange( target_velocity, velocity_minium, velocity_maxium );
    if ( std::fabs( current_velocity - valid_target_vel ) < 0.02 ) {
        return valid_target_vel;
    }

    T min_vel_acc_limited = current_velocity + accelation_minium * dt;
    T max_vel_acc_limited = current_velocity + accelation_maxium * dt;

    T result_vel = target_velocity;
    // apply min limit
    if ( min_vel_acc_limited <= max_vel_acc_limited )
        result_vel = WrapRange( result_vel, min_vel_acc_limited, max_vel_acc_limited );
    else
        result_vel = WrapRange( result_vel, max_vel_acc_limited, min_vel_acc_limited );
    result_vel = WrapRange( result_vel, velocity_minium, velocity_maxium );

    return result_vel;
}

/**
 * @brief Compliant moving with external force, used to passive trot.
 *
 * @param current_velocity current velocity, (normally, angular velocity or linear velocity)
 * @param target_velocity target velocity, the same dimension with 'current_velocity'
 * @param velocity_minium minimumvelocity limit, this can be a minus value
 * @param velocity_maxium maximum velocity limit
 * @param deadzone_threshold deadzone of error between current_velocity and target_velocity
 * @param accelation_threshold maximum change velocity of every time step
 * @param velocity_filter filter params
 * @param velocity_trigger a threshold that triggers velocity_hold
 * @param velocity_hold after triggers velocity_hold, target velocity hold velocity_hold
 * @return T target velocity meets all limits
 */
template < typename T >
T FollowMoving( const T& current_velocity, const T& target_velocity, const T& velocity_minium, const T& velocity_maxium, const T& deadzone_threshold, const T& accelation_threshold,
                const T& velocity_filter, const T& velocity_trigger, const T& velocity_hold ) {
    // apply deadband
    T valid_target_vel = WrapRange( target_velocity, velocity_minium, velocity_maxium );
    if ( std::fabs( current_velocity - valid_target_vel ) < deadzone_threshold ) {
        return valid_target_vel;
    }

    if ( velocity_trigger > 0.01 ) {
        if ( current_velocity > velocity_trigger )
            valid_target_vel = velocity_hold;
        else if ( current_velocity > 0.5 * velocity_trigger )
            valid_target_vel = 0.5 * velocity_hold;
    }

    T delta_velocity = accelation_threshold;
    T result_vel     = velocity_filter * current_velocity + ( 1.0 - velocity_filter ) * valid_target_vel;

    if ( result_vel - current_velocity > delta_velocity ) {
        result_vel = current_velocity + delta_velocity;
    }
    if ( result_vel - current_velocity < -1 * delta_velocity ) {
        result_vel = current_velocity - delta_velocity;
    }

    result_vel = WrapRange( result_vel, velocity_minium, velocity_maxium );

    return result_vel;
}

/**
 * @brief Compliant moving of yaw-direction with external force, used to passive trot.
 *
 * @param target_velocity_yaw target velocity of yaw direction
 * @param current_velocity current velocity of yaw direction
 * @param error_vx error between current and target velocity of x-direction
 * @param error_vy error between current and target velocity of y-direction
 * @param velocity_minium minimumvelocity limit, this can be a minus value
 * @param velocity_maxium maximum velocity limit
 * @param deadzone_threshold deadzone of error between current_velocity and target_velocity
 * @param accelation_threshold maximum change velocity of every time step
 * @param yaw_velocity_compensation propotion of yaw velocity
 * @return T target yaw velocity meets all limits
 */
template < typename T >
T FollowMovingYaw( const T& target_velocity_yaw, const T& current_velocity, const T& error_vx, const T& error_vy, const T& velocity_minium, const T& velocity_maxium, const T& deadzone_threshold,
                   const T& accelation_threshold, const T& yaw_velocity_compensation ) {
    // apply deadband
    if ( std::fabs( error_vx ) < deadzone_threshold || std::fabs( error_vy ) < deadzone_threshold ) {
        return target_velocity_yaw;
    }
    T vel_err_angle        = atan2( -1.0 * error_vy, -1.0 * error_vx );
    T vel_err_angle_relate = vel_err_angle;

    T angle_bound = 15 * M_PI / 16;

    if ( vel_err_angle >= angle_bound && vel_err_angle <= M_PI ) {  // 15/16~16/16
        vel_err_angle_relate = vel_err_angle - M_PI;
    }
    else if ( vel_err_angle <= -1 * angle_bound && vel_err_angle >= -M_PI ) {  // -16/16~-15/16
        vel_err_angle_relate = vel_err_angle + M_PI;
    }

    T result_vel                 = target_velocity_yaw;
    T vel_err_angle_relate_tresh = 0.05;
    T yaw_comp_prop_new          = yaw_velocity_compensation * ( 1 - 0.6 * std::fabs( vel_err_angle_relate ) / angle_bound );
    T err_vxy_square             = error_vx * error_vx + error_vy * error_vy;

    if ( std::fabs( vel_err_angle_relate ) > vel_err_angle_relate_tresh ) {
        result_vel = target_velocity_yaw + yaw_comp_prop_new * vel_err_angle_relate * err_vxy_square;
    }
    T delta_velocity = accelation_threshold;
    if ( result_vel - current_velocity > delta_velocity ) {
        result_vel = current_velocity + delta_velocity;
    }
    if ( result_vel - current_velocity < -1 * delta_velocity ) {
        result_vel = current_velocity - delta_velocity;
    }

    result_vel = WrapRange( result_vel, velocity_minium, velocity_maxium );

    return result_vel;
}

/**
 * @brief Lowpass filter of Butterworth filter.
 *
 * @param update_value update current value
 * @param filter_value_in input value of filter
 * @param filter_value_out output value of filter
 * @param filter_params_in filter parameters
 * @param filter_params_out filter parameters
 * @return T filtered value
 */
template < typename T > T LowpassFilterButterworth( T update_value, T* filter_value_in, T* filter_value_out, T* filter_params_in, T* filter_params_out ) {

    filter_value_in[ 2 ] = filter_value_in[ 1 ];
    filter_value_in[ 1 ] = filter_value_in[ 0 ];
    filter_value_in[ 0 ] = update_value;

    T filtered_result = filter_params_out[ 0 ] * filter_value_in[ 0 ] + filter_params_out[ 1 ] * filter_value_in[ 1 ] + filter_params_out[ 2 ] * filter_value_in[ 2 ]
                        - filter_params_in[ 1 ] * filter_value_out[ 0 ] - filter_params_in[ 2 ] * filter_value_out[ 1 ];

    filter_value_out[ 2 ] = filter_value_out[ 1 ];
    filter_value_out[ 1 ] = filter_value_out[ 0 ];
    filter_value_out[ 0 ] = filtered_result;

    return filtered_result;
}

#endif  // CONTROL_UTILITIES_HPP_
