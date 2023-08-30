#ifndef SAFETY_CHECKER_HPP_
#define SAFETY_CHECKER_HPP_

#include <iostream>

#include "control_flags.hpp"
#include "control_fsm_data.hpp"

/**
 * @brief The SafetyChecker handles the checks requested by state machine
 *
 *
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 */
template < typename T > class SafetyChecker {
public:
    SafetyChecker( ControlFsmData< T >* dataIn ) : data_( dataIn ){};

    // Pre checks to make sure controls are safe to run
    bool CheckSafeOrientation();  // robot's orientation is safe to control
    bool CheckRobotLifted( Vec3< T > max_foot_pos_for_lift, T max_leg_length_for_lift, Vec3< T > min_joint_torque_for_lift, Vec3< T > max_omega_for_lift,
                           Vec3< T > max_rpy_for_lift );  // legs are in workspace

    // Post checks to make sure controls can be sent to robot
    bool CheckLegWorkspace( T max_roll_angle, T max_pitch_angle, T max_leg_percent );  // desired foot position is not out of workspace
    bool CheckForceFeedForward();                              // desired feedforward forces are not too large

    // Stores the data from the ControlFsm
    ControlFsmData< T >* data_;

    int32_t foot_pos_error_flag_ = 0;  // record the error of foot pos
private:
    Vec3< T > foot_pos_feedback_[ 4 ], joint_torque_feedback_[ 4 ], foot_pos_feedback_leg_[ 4 ];
    Vec3< T > foot_pos_feedback_sum_, foot_pos_feedback_average_, foot_pos_feedback_front_average_, foot_pos_feedback_rear_average_;
    Vec3< T > foot_pos_feedback_front_min_ = Vec3< T >::Zero();
    Vec3< T > foot_pos_feedback_rear_min_  = Vec3< T >::Zero();
    Vec3< T > joint_torque_feedback_sum_, joint_torque_feedback_average_, joint_torque_feedback_front_average_, joint_torque_feedback_rear_average_;
    Vec3< T > joint_torque_feedback_average_over_time_       = Vec3< T >::Zero();
    Vec3< T > joint_torque_feedback_front_average_over_time_ = Vec3< T >::Zero();
    Vec3< T > joint_torque_feedback_rear_average_over_time_  = Vec3< T >::Zero();
    Vec3< T > rpy_thresh_                                    = Vec3< T >::Zero();
    int       time_joint_torque_average_                     = 35;
    int       gait_id_last_                                  = GaitId::kTrot10v5;  // last gait_cmd
    int       gait_trans_iter_                               = 0;
    int       gait_trans_thresh_                             = 750;
    bool      gait_trans_done_                               = true;  // gait transition is done
    bool      rpy_cmd_good_                                  = true;  // pitch are not too large
    T         rpy_cmd_thresh_                                = 0.1;   // threshold for rpy_cmd_good_
    bool      all_leg_needed_                                = true;  // only front or rear legs can trigger lift
    T         vel_thresh_                                    = -0.3;  // threshold for backward locomotion that triggers all_leg_needed_
    bool      omega_cmd_good_                                = true;  // large omega command disable lift
    T         front_delta_                                   = 0.0;   // when only front/rear is considered, conditions changes a little bit
    T         rear_delta_                                    = 0.0;
    T         torque_ratio_                                  = 1.0;
    T         max_slope_                                     = 0.436;
    T         foot_pos_x_thresh_                             = -0.14;
    T         foot_pos_y_thresh_                             = 0.071;
};

#endif  // SAFETY_CHECKER_HPP_
