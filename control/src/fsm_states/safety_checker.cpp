#include "fsm_states/safety_checker.hpp"

/**
 * @brief Check robot posture
 *
 * @return true if robot posture is in good condition
 * @return false if robot falls
 */
template < typename T > bool SafetyChecker< T >::CheckSafeOrientation() {
    rpy_thresh_ << 1.3, 1.3, 6.28;
    if ( abs( data_->state_estimator->GetResult().rpy( 0 ) ) >= rpy_thresh_( 0 ) || abs( data_->state_estimator->GetResult().rpy( 1 ) ) >= rpy_thresh_( 1 ) ) {
        printf( "[Safety Checker] Orientation safety check failed!\n" );
        return false;
    }
    else {
        return true;
    }
}

/**
 * @return CheckRobotLifted true if robot is not lifted
 */

/**
 * @brief Check whether robot is lifted
 *
 * @param max_foot_pos_for_lift maximum foot position that triggers lifted protection
 * @param max_leg_length_for_lift maximum leg length that triggers lifted protection
 * @param min_joint_torque_for_lift minium joint torque that triggers lifted protection
 * @param max_omega_for_lift maximum omega that triggers lifted protection
 * @param max_rpy_for_lift maximum robot posutre that triggers lifted protection
 * @return true if robot is in good condition
 * @return false if robot is lifted
 */
template < typename T >
bool SafetyChecker< T >::CheckRobotLifted( Vec3< T > max_foot_pos_for_lift, T max_leg_length_for_lift, Vec3< T > min_joint_torque_for_lift, Vec3< T > max_omega_for_lift, Vec3< T > max_rpy_for_lift ) {

    bool  robot_not_lifted = true;
    auto& seResult         = data_->state_estimator->GetResult();

    foot_pos_feedback_sum_.setZero();
    foot_pos_feedback_average_.setZero();
    foot_pos_feedback_front_average_.setZero();
    foot_pos_feedback_rear_average_.setZero();
    foot_pos_feedback_front_min_.setZero();
    foot_pos_feedback_rear_min_.setZero();
    joint_torque_feedback_sum_.setZero();
    joint_torque_feedback_average_.setZero();
    joint_torque_feedback_front_average_.setZero();
    joint_torque_feedback_rear_average_.setZero();
    rpy_cmd_thresh_    = 0.05;
    vel_thresh_        = -0.3;
    front_delta_       = 0.02;
    rear_delta_        = 0.02;
    torque_ratio_      = 1.3;
    max_slope_         = 0.436;
    foot_pos_x_thresh_ = -0.14;
    foot_pos_y_thresh_ = 0.071;

    for ( int i = 0; i < 4; i++ ) {
        foot_pos_feedback_[ i ]     = seResult.world2body_rotation_matrix.transpose() * data_->leg_controller->datas_[ i ].p;
        foot_pos_feedback_sum_      = foot_pos_feedback_sum_ + foot_pos_feedback_[ i ];
        joint_torque_feedback_[ i ] = data_->leg_controller->datas_[ i ].tau_actual;
        joint_torque_feedback_sum_  = joint_torque_feedback_sum_ + joint_torque_feedback_[ i ];
    }
    foot_pos_feedback_average_           = foot_pos_feedback_sum_ * 0.25;
    foot_pos_feedback_front_average_     = seResult.world2body_rotation_matrix.transpose() * ( data_->leg_controller->datas_[ 0 ].p + data_->leg_controller->datas_[ 1 ].p ) * 0.5;
    foot_pos_feedback_rear_average_      = seResult.world2body_rotation_matrix.transpose() * ( data_->leg_controller->datas_[ 2 ].p + data_->leg_controller->datas_[ 3 ].p ) * 0.5;
    joint_torque_feedback_average_       = joint_torque_feedback_sum_ * 0.25;
    joint_torque_feedback_front_average_ = ( data_->leg_controller->datas_[ 0 ].tau_actual + data_->leg_controller->datas_[ 1 ].tau_actual ) * 0.5;
    joint_torque_feedback_rear_average_  = ( data_->leg_controller->datas_[ 2 ].tau_actual + data_->leg_controller->datas_[ 3 ].tau_actual ) * 0.5;

    for ( int i = 0; i < 3; i++ ) {
        foot_pos_feedback_front_min_( i ) = fmin( data_->leg_controller->datas_[ 0 ].p( i ), data_->leg_controller->datas_[ 1 ].p( i ) );
        foot_pos_feedback_rear_min_( i )  = fmin( data_->leg_controller->datas_[ 2 ].p( i ), data_->leg_controller->datas_[ 3 ].p( i ) );
    }
    // average in a period
    if ( data_->command->gait_id == GaitId::kPronk ) {
        time_joint_torque_average_ = 75;
    }
    else if ( data_->command->gait_id == GaitId::kBound ) {
        time_joint_torque_average_ = 50;
    }
    else if ( data_->command->gait_id == GaitId::kTrotSlow ) {
        time_joint_torque_average_ = 50;
    }
    else if ( data_->command->gait_id == GaitId::kTrotMedium ) {
        time_joint_torque_average_ = 50;
    }
    else if ( data_->command->gait_id == GaitId::kStand || data_->command->gait_id == GaitId::kStandNoPr ) {
        time_joint_torque_average_ = 30;
    }
    else if ( data_->command->gait_id == GaitId::kTrot10v4 || data_->command->gait_id == GaitId::kTrotFast || data_->command->gait_id == GaitId::kTrot8v3
              || ( ( data_->command->gait_id == GaitId::kTrot24v16 || data_->command->gait_id == GaitId::kTrotAuto || data_->command->gait_id == GaitId::kTrot20v12Follow )
                   && data_->robot_current_state->gait_id == GaitId::kTrot8v3 ) ) {
        time_joint_torque_average_ = 45.0 + 120.0 * ( seResult.velocity_in_body_frame.norm() >= 0.2 ? seResult.velocity_in_body_frame.norm() : 0.0 );
    }
    else if ( data_->command->gait_id == GaitId::kPassiveTrot ) {
        if ( data_->robot_current_state->gait_id == GaitId::kPassiveTrot ) {
            time_joint_torque_average_ = 200; //200
        }
        else if ( data_->robot_current_state->gait_id == GaitId::kStandPassive ) {
            time_joint_torque_average_ = 200;
        }
        else {
            time_joint_torque_average_ = 200;
        }
    }
    else if ( data_->command->gait_id <= GaitId::kStandNoPr ) {  // locomotion gaits
        time_joint_torque_average_ = 50;
    }
    else if ( data_->command->gait_id >= GaitId::kSpecialPronk && data_->command->gait_id <= GaitId::kUserGait ) {  // motion gaits
        time_joint_torque_average_ = 50;
    }
    else {  // stair or slope gaits
        time_joint_torque_average_ = 100;
    }

    // adapt for slope
    if ( data_->robot_current_state->step_on_slope && data_->command->mode == kLocomotion ) {
        time_joint_torque_average_ = 100;
        max_foot_pos_for_lift( 2 ) /= cos( max_slope_ );
    }

    joint_torque_feedback_average_over_time_ = ( joint_torque_feedback_average_over_time_ * ( time_joint_torque_average_ - 1 ) + joint_torque_feedback_average_ ) / time_joint_torque_average_;
    joint_torque_feedback_front_average_over_time_ =
        ( joint_torque_feedback_front_average_over_time_ * ( time_joint_torque_average_ - 1 ) + joint_torque_feedback_front_average_ ) / time_joint_torque_average_;
    joint_torque_feedback_rear_average_over_time_ =
        ( joint_torque_feedback_rear_average_over_time_ * ( time_joint_torque_average_ - 1 ) + joint_torque_feedback_rear_average_ ) / time_joint_torque_average_;

    joint_torque_feedback_average_       = joint_torque_feedback_average_over_time_;
    joint_torque_feedback_front_average_ = joint_torque_feedback_front_average_over_time_;
    joint_torque_feedback_rear_average_  = joint_torque_feedback_rear_average_over_time_;

    // gait changing triggers a timer that disables lift error
    // auto switch gaits gait_cmd != current_gait
    if ( ( gait_id_last_ != data_->command->gait_id && data_->command->gait_id != GaitId::kTrot24v16 && data_->command->gait_id != GaitId::kTrotAuto
           && data_->command->gait_id != GaitId::kTrot20v12Follow ) ) {
        gait_trans_iter_ = 0;
        gait_trans_done_ = false;
        // transition between locomotion needs more time
        if ( data_->command->gait_id == GaitId::kStand || data_->command->gait_id == GaitId::kStandNoPr || data_->command->gait_id == GaitId::kStandPassive ) {
            gait_trans_thresh_ = 150;
        }
        else {
            gait_trans_thresh_ = 750;
        }
    }
    if ( gait_trans_done_ == false ) {
        if ( data_->command->gait_id == data_->robot_current_state->gait_id  // iter after trans done
             || data_->command->mode == kQpStand ) {                         // kQpStand cmd gait_id is zero
            gait_trans_iter_++;
        }
    }
    if ( gait_trans_iter_ >= gait_trans_thresh_ ) {
        gait_trans_done_ = true;
        gait_trans_iter_ = 0;
    }

    // large pitch cmd during locomotion disables lift error
    if ( data_->command->mode == kLocomotion && fabs( data_->command->rpy_des[ 1 ] ) > rpy_cmd_thresh_ ) {
        rpy_cmd_good_ = false;
    }
    else {
        rpy_cmd_good_ = true;
    }

    // large rpy with large omege in kQpStand disables lift error
    if ( data_->command->mode == kQpStand && fabs( seResult.rpy[ 1 ] > max_rpy_for_lift( 1 ) ) && seResult.angular_velocity_in_body_frame.norm() > max_omega_for_lift( 1 ) ) {
        omega_cmd_good_ = false;
    }
    else {
        omega_cmd_good_ = true;
    }

    // bound and pronk need all legs
    // backward fast moving need all legs
    if ( ( data_->command->mode == kLocomotion && ( data_->command->gait_id == GaitId::kBound || data_->command->gait_id == GaitId::kPronk || data_->command->gait_id == GaitId::kTrot8v3 ) )
         || seResult.velocity_in_body_frame( 0 ) <= vel_thresh_ ) {
        all_leg_needed_ = true;
    }
    else {
        all_leg_needed_ = false;
    }

    // STAND doesn't consider max_omega_for_lift
    if ( data_->robot_current_state->gait_id == GaitId::kStand ) {
        max_omega_for_lift << 0.0, 0.08, 0.0;
        // in case of last gait is PRONK or BOUND, torso desired height is lower than other case
        max_foot_pos_for_lift( 2 )     = max_foot_pos_for_lift( 2 ) - 0.01;
        min_joint_torque_for_lift( 2 ) = min_joint_torque_for_lift( 2 ) - 0.1;
    }
    else if ( data_->robot_current_state->gait_id == GaitId::kStandNoPr ) {
        // relax conditions for StandNoPr
        max_omega_for_lift << 0.0, 0.1, 0.0;
        max_foot_pos_for_lift( 2 )     = max_foot_pos_for_lift( 2 ) - 0.0;
        min_joint_torque_for_lift( 2 ) = min_joint_torque_for_lift( 2 ) - 0.0;
    }

    // determine whether robot is lifted
    for ( int i = 0; i < 4; i++ ) {
        // support knee torque is negative, lift knee is positive
        // front or rear leg must work with corresponding omega
        if ( foot_pos_feedback_[ i ].norm() > max_leg_length_for_lift && gait_trans_done_ && rpy_cmd_good_ && omega_cmd_good_
             && ( ( fabs( foot_pos_feedback_front_average_( 2 ) ) > max_foot_pos_for_lift( 2 ) && joint_torque_feedback_front_average_( 2 ) > ( min_joint_torque_for_lift( 2 ) * torque_ratio_ )
                    && seResult.angular_velocity_in_body_frame( 1 ) < -max_omega_for_lift( 1 ) && !all_leg_needed_ )
                  || ( fabs( foot_pos_feedback_front_min_( 2 ) ) > max_foot_pos_for_lift( 2 ) + front_delta_ && seResult.angular_velocity_in_body_frame( 1 ) > max_omega_for_lift( 1 )
                       && seResult.rpy( 1 ) > max_rpy_for_lift( 1 ) )
                  || ( fabs( foot_pos_feedback_rear_average_( 2 ) ) > max_foot_pos_for_lift( 2 ) && joint_torque_feedback_rear_average_( 2 ) > ( min_joint_torque_for_lift( 2 ) * torque_ratio_ )
                       && seResult.angular_velocity_in_body_frame( 1 ) > max_omega_for_lift( 1 ) && !all_leg_needed_ )
                  || ( fabs( foot_pos_feedback_rear_min_( 2 ) ) > max_foot_pos_for_lift( 2 ) + rear_delta_ && seResult.angular_velocity_in_body_frame( 1 ) < -max_omega_for_lift( 1 )
                       && seResult.rpy( 1 ) < -max_rpy_for_lift( 1 ) )
                  || ( fabs( foot_pos_feedback_average_( 2 ) ) > max_foot_pos_for_lift( 2 ) && joint_torque_feedback_average_( 2 ) > ( min_joint_torque_for_lift( 2 ) * torque_ratio_ ) ) ) ) {
            robot_not_lifted = false;
            break;
        }
        else {
            robot_not_lifted = true;
        }
    }

    // another case
    // feet close to each other leads to falling down in kQpStand, apply lift to reset feet position
    for ( int i = 0; i < 4; i++ ) {
        foot_pos_feedback_leg_[ i ] = data_->leg_controller->datas_[ i ].p;
    }
    foot_pos_x_thresh_ = -0.14;
    foot_pos_y_thresh_ = 0.071;
    if ( data_->command->mode == kQpStand ) {
        if ( ( foot_pos_feedback_leg_[ 0 ]( 0 ) - foot_pos_feedback_leg_[ 2 ]( 0 ) < foot_pos_x_thresh_ && foot_pos_feedback_leg_[ 1 ]( 0 ) - foot_pos_feedback_leg_[ 3 ]( 0 ) < foot_pos_x_thresh_ )
             || ( foot_pos_feedback_leg_[ 1 ]( 1 ) - foot_pos_feedback_leg_[ 0 ]( 1 ) < foot_pos_y_thresh_
                  && foot_pos_feedback_leg_[ 3 ]( 1 ) - foot_pos_feedback_leg_[ 2 ]( 1 ) < foot_pos_y_thresh_ ) ) {
            robot_not_lifted = false;
        }
        else {
            robot_not_lifted = true;
        }
    }

    // backup last gait id
    if ( data_->command->mode == kLocomotion ) {
        gait_id_last_ = data_->command->gait_id;
    }
    else {  // in case of PRONK->kQpStand->PRONK, gait_trans_done_ is skipped
        gait_id_last_ = GaitId::kTrot10v5;
    }

    return robot_not_lifted;

    // // DEBUG
    // std::cout << "max_foot_pos_for_lift: " << max_foot_pos_for_lift.transpose() << "   max_leg_length_for_lift: " << max_leg_length_for_lift << std::endl;
    // for ( int i = 0; i < 4; i++ ) {
    //     std::cout << "foot_pos_feedback_[" << i << "]: " << foot_pos_feedback_[ i ].transpose() << "   norm: " << foot_pos_feedback_[ i ].norm() << std::endl;
    //     std::cout << "foot_pos_feedback_leg_[" << i << "]: " << foot_pos_feedback_leg_[ i ].transpose() << std::endl;
    // }
    // std::cout << "foot_pos_feedback_average_: " << foot_pos_feedback_average_.transpose() << std::endl;
    // std::cout << "foot_pos_feedback_front_average_: " << foot_pos_feedback_front_average_.transpose() << std::endl;
    // std::cout << "foot_pos_feedback_rear_average_: " << foot_pos_feedback_rear_average_.transpose() << std::endl;
    // std::cout << "joint_torque_feedback_average_: " << joint_torque_feedback_average_.transpose() << std::endl;
    // std::cout << "joint_torque_feedback_front_average_: " << joint_torque_feedback_front_average_.transpose() << std::endl;
    // std::cout << "joint_torque_feedback_rear_average_: " << joint_torque_feedback_rear_average_.transpose() << std::endl;
    // std::cout << "gait_trans_thresh_: " << gait_trans_thresh_ << std::endl;
    // std::cout << "gait_trans_done_: " << gait_trans_done_ << std::endl;
    // std::cout << "rpy_cmd_good_: " << rpy_cmd_good_ << std::endl;
    // std::cout << "omega_cmd_good_: " << omega_cmd_good_ << std::endl;
    // std::cout << "all_leg_needed_: " << all_leg_needed_ << std::endl;
    // std::cout << "foot_pos_feedback_front_min_: " << foot_pos_feedback_front_min_( 2 ) << "  max_foot_pos_for_lift: " << max_foot_pos_for_lift( 2 ) << std::endl;
    // std::cout << "foot_pos_feedback_rear_min_: " << foot_pos_feedback_rear_min_( 2 ) << "  max_foot_pos_for_lift: " << max_foot_pos_for_lift( 2 ) << std::endl;
    // std::cout << "seResult.angular_velocity_in_body_frame: " << seResult.angular_velocity_in_body_frame( 1 ) << "  max_omega_for_lift: " << max_omega_for_lift( 1 ) << std::endl;
    // std::cout << "seResult.rpy: " << seResult.rpy( 1 ) << "  max_rpy_for_lift: " << max_rpy_for_lift( 1 ) << std::endl;
    // std::cout << std::endl;
}

/**
 * @brief Check robot legs are in workspace
 *
 * @param max_angle maximum leg length multipies sin( max_angle ) is the maximum for x and y direction
 * @param max_leg_percent maximum percentage of maximun leg length is used as minium position in z direction
 * @return true if legs are in good conditions
 * @return false if legs are out of workspace
 */
template < typename T > bool SafetyChecker< T >::CheckLegWorkspace( T max_roll_angle, T max_pitch_angle, T max_leg_percent ) {
    // Assumed safe to start
    bool safePDesFoot = true;

    // max leg pos/length
    static T max_leg_pos_x_y = 0.0;

    if ( data_->command->motion_id == MotionId::kMotionPushUp ) {
        return true;
    }

    // Safety parameters
    foot_pos_error_flag_ = 0;
    // Check all of the legs
    for ( int leg = 0; leg < 4; leg++ ) {
        max_leg_pos_x_y = data_->quadruped->max_leg_length_ * sin( max_pitch_angle );
        // Keep the foot from going too far from the body in +x
        if ( data_->leg_controller->datas_[ leg ].p( 0 ) > max_leg_pos_x_y ) {
            std::cout << "[SafetyChecker] Out of leg workspace: | leg " << leg << " | coordinate: "
                      << "x"
                      << "\n";
            std::cout << "   position: " << data_->leg_controller->datas_[ leg ].p( 0 ) << " | max accept: " << max_leg_pos_x_y << std::endl;
            data_->leg_controller->datas_[ leg ].p( 0 ) = max_leg_pos_x_y;
            safePDesFoot                                = false;
            foot_pos_error_flag_ |= 1 << leg * 8;
        }

        // Keep the foot from going too far from the body in -x
        if ( data_->leg_controller->datas_[ leg ].p( 0 ) < -max_leg_pos_x_y ) {
            std::cout << "[SafetyChecker] Out of leg workspace: | leg " << leg << " | coordinate: "
                      << "x"
                      << "\n";
            std::cout << "   position: " << data_->leg_controller->datas_[ leg ].p( 0 ) << " | max accept: " << -max_leg_pos_x_y << std::endl;
            data_->leg_controller->datas_[ leg ].p( 0 ) = -max_leg_pos_x_y;
            safePDesFoot                                = false;
            foot_pos_error_flag_ |= 1 << leg * 8;
        }

        max_leg_pos_x_y = data_->quadruped->max_leg_length_ * sin( max_roll_angle ) + data_->quadruped->GetSideSign( leg ) * data_->quadruped->abad_link_length_ * cos( max_roll_angle );
        // Keep the foot from going too far from the body in +y
        if ( data_->leg_controller->datas_[ leg ].p( 1 ) > max_leg_pos_x_y ) {
            std::cout << "[SafetyChecker] Out of leg workspace: | leg " << leg << " | coordinate: "
                      << "y"
                      << "\n";
            std::cout << "   position: " << data_->leg_controller->datas_[ leg ].p( 1 ) << " | max accept: " << max_leg_pos_x_y << std::endl;
            data_->leg_controller->datas_[ leg ].p( 1 ) = max_leg_pos_x_y;
            safePDesFoot                                = false;
            foot_pos_error_flag_ |= 1 << ( leg * 8 + 1 );
        }

        max_leg_pos_x_y = data_->quadruped->max_leg_length_ * sin( max_roll_angle ) - data_->quadruped->GetSideSign( leg ) * data_->quadruped->abad_link_length_ * cos( max_roll_angle );
        // Keep the foot from going too far from the body in -y
        if ( data_->leg_controller->datas_[ leg ].p( 1 ) < -max_leg_pos_x_y ) {
            std::cout << "[SafetyChecker] Out of leg workspace: | leg " << leg << " | coordinate: "
                      << "y"
                      << "\n";
            std::cout << "   position: " << data_->leg_controller->datas_[ leg ].p( 1 ) << " | max accept: " << -max_leg_pos_x_y << std::endl;
            data_->leg_controller->datas_[ leg ].p( 1 ) = -max_leg_pos_x_y;
            safePDesFoot                                = false;
            foot_pos_error_flag_ |= 1 << ( leg * 8 + 1 );
        }

        // Keep the leg under the motor module (don't raise above body or crash into
        // module)
        if ( data_->leg_controller->datas_[ leg ].p( 2 ) > -data_->quadruped->max_leg_length_ / max_leg_percent ) {
            std::cout << "[SafetyChecker] Out of leg workspace: | leg " << leg << " | coordinate: "
                      << "z"
                      << "\n";
            std::cout << "   position: " << data_->leg_controller->datas_[ leg ].p( 2 ) << " | max accept: " << -data_->quadruped->max_leg_length_ / max_leg_percent << std::endl;
            data_->leg_controller->datas_[ leg ].p( 2 ) = -data_->quadruped->max_leg_length_ / max_leg_percent;
            safePDesFoot                                = false;
            foot_pos_error_flag_ |= 1 << ( leg * 8 + 2 );
        }

        // Keep the foot within the kinematic limits
        T maxLegLength = sqrt( pow( data_->quadruped->max_leg_length_, 2 ) + pow( data_->quadruped->abad_link_length_, 2 ) );
        if ( data_->leg_controller->datas_[ leg ].p( 2 ) < -maxLegLength ) {
            std::cout << "[SafetyChecker] Out of leg workspace: | leg " << leg << " | coordinate: "
                      << "z"
                      << "\n";
            std::cout << "   position: " << data_->leg_controller->datas_[ leg ].p( 2 ) << " | max accept: " << -data_->quadruped->max_leg_length_ << std::endl;
            data_->leg_controller->datas_[ leg ].p( 2 ) = -maxLegLength;
            safePDesFoot                                = false;
            foot_pos_error_flag_ |= 1 << ( leg * 8 + 2 );
        }
    }

    return safePDesFoot;
}

/**
 * @brief Checke feedforward contact forces are in limitation
 *
 * @return true if feedforward contact forces are OK
 * @return false if feedforward contact forces exceed limitation
 */
template < typename T > bool SafetyChecker< T >::CheckForceFeedForward() {
    // Assumed safe to start
    bool safeForceFeedForward = true;

    // Initialize maximum vertical and lateral forces
    static T maxLateralForce  = 0;
    static T maxVerticalForce = 0;

    // Maximum force limits for each robot
    if ( data_->quadruped->robot_type_ == RobotType::CYBERDOG ) {
        maxLateralForce  = 350;
        maxVerticalForce = 350;
    }
    else if ( data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        maxLateralForce  = 250;
        maxVerticalForce = 250;
    }

    // Check all of the legs
    for ( int leg = 0; leg < 4; leg++ ) {
        // Limit the lateral forces in +x body frame
        if ( data_->leg_controller->commands_[ leg ].force_feed_forward( 0 ) > maxLateralForce ) {
            std::cout << "[SafetyChecker] Feedforward force:  " << leg << " | coordinate: "
                      << "x"
                      << "\n";
            std::cout << "   commanded: " << data_->leg_controller->commands_[ leg ].force_feed_forward( 0 ) << " | modified: " << maxLateralForce << std::endl;
            data_->leg_controller->commands_[ leg ].force_feed_forward( 0 ) = maxLateralForce;
            safeForceFeedForward                                            = false;
        }

        // Limit the lateral forces in -x body frame
        if ( data_->leg_controller->commands_[ leg ].force_feed_forward( 0 ) < -maxLateralForce ) {
            std::cout << "[SafetyChecker] Feedforward force:  " << leg << " | coordinate: "
                      << "x"
                      << "\n";
            std::cout << "   commanded: " << data_->leg_controller->commands_[ leg ].force_feed_forward( 0 ) << " | modified: " << -maxLateralForce << std::endl;
            data_->leg_controller->commands_[ leg ].force_feed_forward( 0 ) = -maxLateralForce;
            safeForceFeedForward                                            = false;
        }

        // Limit the lateral forces in +y body frame
        if ( data_->leg_controller->commands_[ leg ].force_feed_forward( 1 ) > maxLateralForce ) {
            std::cout << "[SafetyChecker] Feedforward force:  " << leg << " | coordinate: "
                      << "y"
                      << "\n";
            std::cout << "   commanded: " << data_->leg_controller->commands_[ leg ].force_feed_forward( 1 ) << " | modified: " << maxLateralForce << std::endl;
            data_->leg_controller->commands_[ leg ].force_feed_forward( 1 ) = maxLateralForce;
            safeForceFeedForward                                            = false;
        }

        // Limit the lateral forces in -y body frame
        if ( data_->leg_controller->commands_[ leg ].force_feed_forward( 1 ) < -maxLateralForce ) {
            std::cout << "[SafetyChecker] Feedforward force:  " << leg << " | coordinate: "
                      << "y"
                      << "\n";
            std::cout << "   commanded: " << data_->leg_controller->commands_[ leg ].force_feed_forward( 1 ) << " | modified: " << -maxLateralForce << std::endl;
            data_->leg_controller->commands_[ leg ].force_feed_forward( 1 ) = -maxLateralForce;
            safeForceFeedForward                                            = false;
        }

        // Limit the vertical forces in +z body frame
        if ( data_->leg_controller->commands_[ leg ].force_feed_forward( 2 ) > maxVerticalForce ) {
            std::cout << "[SafetyChecker] Feedforward force:  " << leg << " | coordinate: "
                      << "z"
                      << "\n";
            std::cout << "   commanded: " << data_->leg_controller->commands_[ leg ].force_feed_forward( 2 ) << " | modified: " << -maxVerticalForce << std::endl;
            data_->leg_controller->commands_[ leg ].force_feed_forward( 2 ) = maxVerticalForce;
            safeForceFeedForward                                            = false;
        }

        // Limit the vertical forces in -z body frame
        if ( data_->leg_controller->commands_[ leg ].force_feed_forward( 2 ) < -maxVerticalForce ) {
            std::cout << "[SafetyChecker] Feedforward force:  " << leg << " | coordinate: "
                      << "z"
                      << "\n";
            std::cout << "   commanded: " << data_->leg_controller->commands_[ leg ].force_feed_forward( 2 ) << " | modified: " << maxVerticalForce << std::endl;
            data_->leg_controller->commands_[ leg ].force_feed_forward( 2 ) = -maxVerticalForce;
            safeForceFeedForward                                            = false;
        }
    }

    return safeForceFeedForward;
}

template class SafetyChecker< float >;
