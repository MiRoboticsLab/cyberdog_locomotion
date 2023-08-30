#include "fsm_states/fsm_state_two_leg_stand.hpp"
#include <wbc_ctrl/locomotion_ctrl/locomotion_ctrl.hpp>

/**
 * @brief Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param control_fsm_data holds all of the relevant control data
 */
template < typename T > FsmStateTwoLegStand< T >::FsmStateTwoLegStand( ControlFsmData< T >* control_fsm_data ) : FsmState< T >( control_fsm_data, FsmStateName::kTwoLegStand, "two_leg_stand" ) {
    // Set the pre controls safety checks
    this->TurnOnAllSafetyChecks();
    // Turn off Foot pos command since it is set in WBC as operational task
    this->check_desired_foot_position_ = false;
    // Turn off  oritation check
    this->check_safe_orientation_ = false;
    this->check_robot_lifted_     = false;
}

/**
 * @brief Behavior to be carried out when entering a state
 *
 */
template < typename T > void FsmStateTwoLegStand< T >::OnEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    // Reset the transition data
    this->transition_data_.ResetTransitionDone();

    this->motion_progress_bar_ = 0;

    this->data_->robot_current_state->gait_id = 0;

    if ( this->data_->command->gait_id >= 1 && this->data_->command->gait_id < 15 )
        this->data_->robot_current_state->gait_id = this->data_->command->gait_id;
    else
        this->data_->robot_current_state->gait_id = 0;

    // get  initial  body pos and  ori
    ini_body_pos_     = ( this->data_->state_estimator->GetResult() ).position;
    ini_body_ori_rpy_ = ( this->data_->state_estimator->GetResult() ).rpy;

    jump_iter_  = 0;
    jump_pitch_ = this->data_->state_estimator->GetResult().rpy[ 1 ];
    // 0 ~ -90 ~ 0 -> 0 ~ -90 ~ -180
    if ( this->data_->state_estimator->GetResult().world2body_rotation_matrix( 2, 2 ) < 0 )
        jump_pitch_ = -M_PI - jump_pitch_;
    if ( jump_pitch_ < Deg2Rad( -50. ) )
        jump_flag_ = 2;
    else
        jump_flag_ = 0;

    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        is_cyberdog2_   = 1;
        safe_angle_low_ = Deg2Rad( -35. );
        safe_angle_upp_ = Deg2Rad( -100. );
    }  // RobotType::CYBERDOG2
    else {
        is_cyberdog2_   = 0;
        safe_angle_low_ = Deg2Rad( -60. );
        safe_angle_upp_ = Deg2Rad( -120. );
    }  // RobotType::CYBERDOG

    // calculate joint position of stand pose
    T L1           = this->data_->quadruped->hip_link_length_;
    T L2           = this->data_->quadruped->knee_link_length_;
    T com_offset_x = 0.0;
    T H            = L1 * cos( -0.69 ) + L2 * cos( 0.7 );
    T D            = std::sqrt( H * H + com_offset_x * com_offset_x );
    T alpha2       = 3.141592653589793 - std::acos( ( L1 * L1 + L2 * L2 - D * D ) / ( 2.0 * L1 * L2 ) );
    T alpha1       = std::atan( com_offset_x / H ) - std::acos( ( L1 * L1 + D * D - L2 * L2 ) / ( 2.0 * L1 * D ) );
    // Stand pos
    for ( size_t i( 0 ); i < 4; ++i ) {
        stand_j_pos_[ i ] << 0.f, alpha1, alpha2;
    }
    stop_swing_[ 0 ] = false;
    stop_swing_[ 1 ] = false;

#if ( ONBOARD_BUILD != 1 )
    this->data_->user_parameters->knee_angle_adj = 10;
    this->data_->user_parameters->jump_force     = 25;
#else
    if ( is_cyberdog2_ )
        this->data_->user_parameters->jump_force = 30;
    else
        this->data_->user_parameters->jump_force = 70;
#endif
}

/**
 * @brief Run the normal behavior for the state.Calls the functions
 * to be executed on each control loop iteration.
 *
 */
template < typename T > void FsmStateTwoLegStand< T >::Run() {
    contact_states_ << 0.5, 0.5, 0.5, 0.5;
    for ( int i = 0; i < 4; i++ )
        leg_cmd_old_[ i ] = this->data_->leg_controller->commands_[ i ];
    this->data_->leg_controller->ZeroCommand();
    this->data_->leg_controller->legs_enabled_ = true;

    TwoLegStandStep();

    // useless
    this->data_->state_estimator->SetContactPhase( contact_states_ );
}

/**
 * @brief Checks for any transition triggers.
 *
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template < typename T > FsmStateName FsmStateTwoLegStand< T >::CheckTransition() {
    // Get the next state
    iter_++;
    auto& cmd = this->data_->command;
    // Switch FSM control mode
    switch ( cmd->mode ) {
    case MotionMode::kTwoLegStand:
        break;
    case MotionMode::kQpStand:
    case MotionMode::kRecoveryStand:
    case MotionMode::kRlReset:
    case MotionMode::kOff:
    case MotionMode::kPureDamper:
    case MotionMode::kJump3d:
    case MotionMode::kPoseCtrl:
        this->next_state_name_ = ( FsmStateName )cmd->mode;
        break;
    default:
        if ( this->iter_printf_ >= this->iter_printf_reset_ ) {
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << ( int )MotionMode::kTwoLegStand << " to " << ( int )this->data_->command->mode << std::endl;
            this->iter_printf_ = 0;
        }
        this->iter_printf_++;
    }
    // Return the next state name to the FSM
    return this->next_state_name_;
}

/**
 * @brief Manages state specific transitions.
 *
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template < typename T > TransitionData< T > FsmStateTwoLegStand< T >::Transition() {
    // Switch FSM control mode
    switch ( this->next_state_name_ ) {
    case FsmStateName::kQpStand:
    case FsmStateName::kJump3d:
    case FsmStateName::kPoseCtrl:
    case FsmStateName::kLocomotion:
        Run();
        if ( this->motion_progress_bar_ >= 100 )  // After done TwoLegStand
            this->transition_data_.done = true;
        else
            this->transition_data_.done = false;
        break;
    case FsmStateName::kOff:
        this->TurnOffAllSafetyChecks();
        this->transition_data_.done = true;
        break;
    case FsmStateName::kRecoveryStand:
    case FsmStateName::kRlReset:
    case FsmStateName::kPureDamper:
        this->transition_data_.done = true;
        break;
    default:
        std::cout << "[CONTROL FSM] Something went wrong in transition" << std::endl;
    }
    // Return the transition data to the FSM
    return this->transition_data_;
}

/**
 * @brief Behavior to be carried out when exiting a state.
 *
 * Cleans up the state information on exiting the state.
 */
template < typename T > void FsmStateTwoLegStand< T >::OnExit() {
    iter_ = 0;
}

/**
 * @brief Calculate the commands for support legs(WBC) and swing legs(Joint PD).
 */
template < typename T > void FsmStateTwoLegStand< T >::TwoLegStandStep() {
    if ( !stop_swing_[ 0 ] && !stop_swing_[ 1 ] )
        jump_iter_++;
    double    current_time = jump_iter_ * this->data_->control_parameters->controller_dt;
    Vec3< T > leg_angle_des;
    leg_angle_des.setZero();
    Vec3< T > foot_pos_des;
    foot_pos_des.setZero();
    Vec3< T > foot_pos;
    foot_pos.setZero();
    Vec3< T > f_force;
    f_force.setZero();
    Vec3< T > t_force;
    t_force.setZero();
    Mat3< T > f_kp;
    f_kp.setZero();
    Mat3< T > f_kd;
    f_kd.setZero();

    Mat3< T > kdJoint_init;
    kdJoint_init.setZero();
    Mat3< T > kdJoint_goal;
    kdJoint_goal.setZero();
    Mat3< T > kdJoint_des;
    kdJoint_des.setZero();
    Vec3< T > tau_init;
    tau_init.setZero();
    Vec3< T > tau_goal;
    tau_goal.setZero();
    Vec3< T > tau_des;
    tau_des.setZero();

    double jump_force_init{ 0.0 };
    double jump_force_goal{ 0.0 };
    double jump_force_des{ 0.0 };
    double jump_force_ratio{ 2.0 };

    jump_pitch_     = this->data_->state_estimator->GetResult().rpy[ 1 ];
    jump_pitch_vel_ = this->data_->state_estimator->GetResult().angular_velocity_in_body_frame[ 1 ];
    // 0 ~ -90 ~ 0 -> 0 ~ -90 ~ -180
    if ( this->data_->state_estimator->GetResult().world2body_rotation_matrix( 2, 2 ) < 0 ) {
        jump_pitch_ = -M_PI - jump_pitch_;
    }
    if ( this->data_->robot_current_state->gait_id > 10 )
        jump_force_ratio *= 1.25;

    if ( jump_flag_ == 0 ) {   /// kneel down
        double time_unit = 1;  // 1
        for ( int i = 0; i < 4; ++i ) {
            if ( jump_iter_ == 1 ) {
                for ( int j = 0; j < 3; j++ ) {
                    if ( fabs( stand_j_pos_[ i ]( j ) - this->data_->leg_controller->datas_[ i ].q( j ) ) > 0.05 )
                        leg_angle_init_( i * 3 + j ) = this->data_->leg_controller->datas_[ i ].q( j );
                    else
                        leg_angle_init_( i * 3 + j ) = stand_j_pos_[ i ]( j );
                }
                if ( i == 0 || i == 1 )
                    leg_angle_goal_.segment( i * 3, 3 ) << 0, Deg2Rad( -79. ), Deg2Rad( 138. );
                if ( i == 2 || i == 3 ) {
                    if ( is_cyberdog2_ == 1 )
                        leg_angle_goal_.segment( i * 3, 3 ) << 0, Deg2Rad( 22. ) - Deg2Rad( 30. ), Deg2Rad( 78. ) + Deg2Rad( 20. );
                    else
                        leg_angle_goal_.segment( i * 3, 3 ) << 0, Deg2Rad( 22. ), Deg2Rad( 78. );
                }
            }
            leg_angle_des_.segment( i * 3, 3 ) =
                leg_angle_init_.segment( i * 3, 3 ) + ( leg_angle_goal_.segment( i * 3, 3 ) - leg_angle_init_.segment( i * 3, 3 ) ) * ( current_time > time_unit ? 1 : current_time / time_unit );

            this->data_->leg_controller->commands_[ i ].q_des << leg_angle_des_.segment( i * 3, 3 );
            this->data_->leg_controller->commands_[ i ].qd_des << 0, 0, 0;
            this->data_->leg_controller->commands_[ i ].kp_joint << 80, 0, 0, 0, 80, 0, 0, 0, 100;
            this->data_->leg_controller->commands_[ i ].kd_joint << 1, 0, 0, 0, 1, 0, 0, 0, 2;
            this->data_->leg_controller->commands_[ i ].tau_feed_forward << 0, 0, 0;
            this->data_->leg_controller->commands_[ i ].force_feed_forward << 0, 0, 0;
        }
        rear_leg_angle_1_ << leg_angle_des_.segment( 6, 3 );
        // set contact states by scheduler
        contact_states_ << 0.5, 0.5, 0.5, 0.5;

        if ( current_time > time_unit ) {
            jump_iter_                   = 0;
            jump_flag_                   = 1;
            front_leg_jump_ok_[ 0 ]      = 0;
            front_leg_jump_ok_[ 1 ]      = 0;
            this->motion_progress_bar_   = 25;
            front_leg_force_to_pos_flag_ = false;
            jump_pitch_ang_tmp_          = jump_pitch_;
            leg_angle_log_ << this->data_->leg_controller->commands_[ 0 ].q_des, this->data_->leg_controller->commands_[ 1 ].q_des, this->data_->leg_controller->commands_[ 2 ].q_des,
                this->data_->leg_controller->commands_[ 3 ].q_des;
            std::cout << "[FSM TWOLEGSTAND] jump_flag_ = " << jump_flag_ << std::endl;
            std::cout << "[FSM TWOLEGSTAND] current time = " << current_time << std::endl;
        }
    }
    else if ( jump_flag_ == 1 ) {  // push up
        double time_unit = 0.6;
        for ( int i = 0; i < 4; ++i ) {
            if ( i == 0 || i == 1 ) {
                // calculate the footPos of front leg in the body framework
                foot_pos = ForwardKinematic( *this->data_->quadruped, i, this->data_->leg_controller->datas_[ i ].q );

                if ( foot_pos[ 2 ] >= -0.35 * ( ( this->data_->quadruped->hip_link_length_ + this->data_->quadruped->knee_link_length_ ) / 0.4 ) && !front_leg_jump_ok_[ 0 ]
                     && !front_leg_jump_ok_[ 1 ] ) {  // push with force first
                    float force_lasting_time = 0.2;
                    jump_force_init          = this->data_->user_parameters->jump_force;
                    jump_force_goal          = jump_force_ratio * this->data_->user_parameters->jump_force;
                    jump_force_des           = jump_force_init + ( jump_force_goal - jump_force_init ) * ( current_time > force_lasting_time ? 1 : current_time / force_lasting_time );
                    f_kp << 30, 0, 0, 0, 0, 0, 0, 0, 0;
                    f_kd << 0, 0, 0, 0, 0, 0, 0, 0, 0;
                    f_force << 0, 0, -jump_force_des;
                    t_force << 0, 0, 0;

                    this->data_->leg_controller->commands_[ i ].q_des << 0, 0, 0;
                    this->data_->leg_controller->commands_[ i ].qd_des << 0, 0, 0;
                    this->data_->leg_controller->commands_[ i ].kp_joint           = f_kp;
                    this->data_->leg_controller->commands_[ i ].kd_joint           = f_kd;
                    this->data_->leg_controller->commands_[ i ].tau_feed_forward   = t_force;
                    this->data_->leg_controller->commands_[ i ].force_feed_forward = f_force;
                    // set contact states by scheduler
                    contact_states_[ 0 ] = 0.5;
                    contact_states_[ 1 ] = 0.5;
                }
                else {  // front legs leave the ground
                    if ( !front_leg_force_to_pos_flag_ ) {
                        leg_angle_temp_.segment( 0, 3 ) = this->data_->leg_controller->datas_[ 0 ].q;
                        leg_angle_temp_.segment( 3, 3 ) = this->data_->leg_controller->datas_[ 1 ].q;
                        jump_pitch_vel_tmp_             = this->data_->state_estimator->GetResult().angular_velocity_in_body_frame[ 1 ];
                        rear_leg_angle_2_               = this->data_->leg_controller->datas_[ 2 ].q;
                        t_1_                            = current_time;
                        front_leg_force_to_pos_flag_    = true;
                    }
                    if ( current_time < time_unit ) {  // if reach early, keep position first
                        leg_angle_des = leg_angle_temp_.segment( i * 3, 3 );
                    }
                    else {  // go to target position
                        foot_pos_goal_.segment( 0, 6 ) << 0, -0.1, -0.25, 0, 0.1, -0.25;
                        if ( is_cyberdog2_ == 1 ) {
                            foot_pos_goal_.segment( 0, 6 ) << 0, -0.09, -0.24, 0, 0.09, -0.24;
                        }
                        leg_angle_init_.segment( 0, 3 ) = leg_angle_temp_.segment( 0, 3 );
                        leg_angle_init_.segment( 3, 3 ) = leg_angle_temp_.segment( 3, 3 );
                        leg_angle_goal_.segment( 0, 3 ) = InverseKinematic< T >( *this->data_->quadruped, 0, foot_pos_goal_.segment( 0, 3 ) );
                        leg_angle_goal_.segment( 3, 3 ) = InverseKinematic< T >( *this->data_->quadruped, 1, foot_pos_goal_.segment( 3, 3 ) );
                        leg_angle_des                   = leg_angle_init_.segment( i * 3, 3 )
                                        + ( leg_angle_goal_.segment( i * 3, 3 ) - leg_angle_init_.segment( i * 3, 3 ) )
                                              * ( ( current_time - time_unit ) > 2 * time_unit ? 1 : ( current_time - time_unit ) / time_unit / 2.0 );
                    }
                    f_force << 0, 0, 0;
                    t_force << 0, 0, 0;
                    f_kp << 50, 0, 0, 0, 50, 0, 0, 0, 60;
                    f_kd << 1, 0, 0, 0, 1, 0, 0, 0, 2;
                    front_leg_jump_ok_[ i ] = 1;

                    this->data_->leg_controller->commands_[ i ].q_des = leg_angle_des;
                    this->data_->leg_controller->commands_[ i ].qd_des << 0, 0, 0;
                    this->data_->leg_controller->commands_[ i ].kp_joint           = f_kp;
                    this->data_->leg_controller->commands_[ i ].kd_joint           = f_kd;
                    this->data_->leg_controller->commands_[ i ].tau_feed_forward   = t_force;
                    this->data_->leg_controller->commands_[ i ].force_feed_forward = f_force;
                    // set contact states by scheduler
                    contact_states_[ 0 ] = 0.0;
                    contact_states_[ 1 ] = 0.0;
                }
            }
            if ( i == 2 || i == 3 ) {
                if ( front_leg_jump_ok_[ 0 ] == 0 || front_leg_jump_ok_[ 1 ] == 0 ) {
                    double kneeLink_pitchAng = Deg2Rad( 7.5 );
                    double hip_pitchAng      = this->data_->leg_controller->datas_[ 2 ].q[ 1 ];
                    double knee_pitchAng     = M_PI / 2.0 + jump_pitch_ - hip_pitchAng - kneeLink_pitchAng;
                    double leave_ground_time = t_1_;

                    leg_angle_des      = rear_leg_angle_1_;
                    leg_angle_des[ 1 ] = leg_angle_des[ 1 ] + jump_pitch_ - jump_pitch_ang_tmp_;
                    leg_angle_des[ 2 ] = knee_pitchAng;

                    this->data_->leg_controller->commands_[ i ].kp_joint << 80, 0, 0, 0, 100, 0, 0, 0, 120;
                    this->data_->leg_controller->commands_[ i ].kd_joint << 1, 0, 0, 0, 1, 0, 0, 0, 2;
                    if ( is_cyberdog2_ == 1 )
                        this->data_->leg_controller->commands_[ i ].kp_joint *= 0.5;
                    tau_init << 0, -10, -10;
                    tau_goal << 0, -10, -10;
                    tau_des = tau_init + ( tau_goal - tau_init ) * ( current_time > leave_ground_time ? 1 : current_time / leave_ground_time );
                    if ( is_cyberdog2_ == 1 )
                        tau_des *= 0.2;
                    this->data_->leg_controller->commands_[ i ].tau_feed_forward = tau_des;
                }
                else if ( ( current_time >= t_1_ ) && ( current_time <= t_1_ + 1.0 ) ) {
                    double rear_hipAng_goal  = Deg2Rad( -117. );
                    double rear_kneeAng_goal = Deg2Rad( 113. );
                    if ( is_cyberdog2_ == 1 ) {
                        if ( this->data_->robot_current_state->gait_id < 10 ) {
                            rear_hipAng_goal  = Deg2Rad( -52.7 );
                            rear_kneeAng_goal = Deg2Rad( 82.5 );
                        }
                        else {
                            rear_hipAng_goal  = Deg2Rad( -82.7 );
                            rear_kneeAng_goal = Deg2Rad( 92.5 );
                        }
                    }
                    double jump_pitch_duration = ( rear_hipAng_goal - rear_leg_angle_2_[ 1 ] ) / jump_pitch_vel_tmp_;

                    leg_angle_des = rear_leg_angle_2_;
                    leg_angle_des[ 1 ] =
                        rear_leg_angle_2_[ 1 ] + ( rear_hipAng_goal - rear_leg_angle_2_[ 1 ] ) * ( ( current_time - t_1_ ) > jump_pitch_duration ? 1 : ( current_time - t_1_ ) / jump_pitch_duration );
                    leg_angle_des[ 2 ] =
                        rear_leg_angle_2_[ 2 ] + ( rear_kneeAng_goal - rear_leg_angle_2_[ 2 ] ) * ( ( current_time - t_1_ ) > jump_pitch_duration ? 1 : ( current_time - t_1_ ) / jump_pitch_duration );

                    this->data_->leg_controller->commands_[ i ].kp_joint << 80, 0, 0, 0, 100, 0, 0, 0, 150;
                    if ( is_cyberdog2_ == 1 )
                        this->data_->leg_controller->commands_[ i ].kp_joint *= 0.5;
                    kdJoint_init << 1, 0, 0, 0, 1, 0, 0, 0, 2;
                    kdJoint_goal << 1, 0, 0, 0, 5, 0, 0, 0, 10;
                    kdJoint_des = kdJoint_init + ( kdJoint_goal - kdJoint_init ) * ( ( current_time - t_1_ ) > jump_pitch_duration ? 1 : ( current_time - t_1_ ) / jump_pitch_duration );
                    this->data_->leg_controller->commands_[ i ].kd_joint = kdJoint_des;
                    tau_init << 0, -10, -10;
                    tau_goal << 0, -5, 0;
                    tau_des = tau_init + ( tau_goal - tau_init ) * ( ( current_time - t_1_ ) > jump_pitch_duration ? 1 : ( current_time - t_1_ ) / jump_pitch_duration );
                    if ( is_cyberdog2_ == 1 )
                        tau_des *= 0.2;
                    this->data_->leg_controller->commands_[ i ].tau_feed_forward = tau_des;
                    rear_leg_angle_3_                                            = leg_angle_des;
                    t_2_                                                         = current_time;
                }
                else {
                    double keep_still_duration = 3 * time_unit - t_1_ - 1.0;
                    leg_angle_des              = rear_leg_angle_3_;

                    this->data_->leg_controller->commands_[ i ].kp_joint << 80, 0, 0, 0, 100, 0, 0, 0, 150;
                    if ( is_cyberdog2_ == 1 )
                        this->data_->leg_controller->commands_[ i ].kp_joint *= 0.5;

                    kdJoint_init << 1, 0, 0, 0, 5, 0, 0, 0, 10;
                    kdJoint_goal << 1, 0, 0, 0, 5, 0, 0, 0, 10;
                    kdJoint_des = kdJoint_init + ( kdJoint_goal - kdJoint_init ) * ( ( current_time - t_2_ ) > keep_still_duration ? 1 : ( current_time - t_2_ ) / keep_still_duration );
                    this->data_->leg_controller->commands_[ i ].kd_joint = kdJoint_des;
                    tau_init << 0, -5, 0;
                    tau_goal << 0, -5, 0;
                    tau_des = tau_init + ( tau_goal - tau_init ) * ( ( current_time - t_2_ ) > keep_still_duration ? 1 : ( current_time - t_2_ ) / keep_still_duration );
                    if ( is_cyberdog2_ == 1 )
                        tau_des *= 0.2;
                    this->data_->leg_controller->commands_[ i ].tau_feed_forward = tau_des;
                    rear_leg_angle_4_                                            = leg_angle_des;
                }
                this->data_->leg_controller->commands_[ i ].q_des = leg_angle_des;
                this->data_->leg_controller->commands_[ i ].qd_des << 0, 0.0, 0;
                this->data_->leg_controller->commands_[ i ].force_feed_forward << 0, 0, 0;
                // set contact states by scheduler
                contact_states_[ 2 ] = 0.5;
                contact_states_[ 3 ] = 0.5;
            }
            if ( ( jump_pitch_ > -3 * M_PI / 4 ) && current_time > 3 * time_unit ) {  // ready to check
                jump_flag_                 = 2;                                       // done
                jump_iter_                 = 0;
                this->motion_progress_bar_ = 50;
                if ( jump_pitch_ > safe_angle_low_ ) {  // too low, fail
                    jump_flag_                 = 3;
                    jump_iter_                 = 0;
                    this->motion_progress_bar_ = 0;
                }
                leg_angle_log_ << this->data_->leg_controller->commands_[ 0 ].q_des, this->data_->leg_controller->commands_[ 1 ].q_des, this->data_->leg_controller->commands_[ 2 ].q_des,
                    this->data_->leg_controller->commands_[ 3 ].q_des;
                std::cout << "[FSM TWOLEGSTAND] jump_flag_ = " << jump_flag_ << std::endl;
                std::cout << "[FSM TWOLEGSTAND] current time = " << current_time << std::endl;
            }
            if ( jump_pitch_ < safe_angle_upp_ ) {  // overturn, fail
                jump_flag_                 = 4;
                jump_iter_                 = 0;
                this->motion_progress_bar_ = 0;
                leg_angle_log_ << this->data_->leg_controller->commands_[ 0 ].q_des, this->data_->leg_controller->commands_[ 1 ].q_des, this->data_->leg_controller->commands_[ 2 ].q_des,
                    this->data_->leg_controller->commands_[ 3 ].q_des;
                std::cout << "[FSM TWOLEGSTAND] jump_flag_ = " << jump_flag_ << std::endl;
                std::cout << "[FSM TWOLEGSTAND] current time = " << current_time << std::endl;
            }
        }
    }
    else if ( jump_flag_ == 2 ) {  // forearm motion
        double time_unit = 0.5;
        int    n         = 0;
        float  m         = 0.5;
        float  t_wel     = current_time;
        float  t_xin     = current_time;
        float  t_thk     = current_time;
        float  a_2 = 0, pos_x = 0, pos_y = 0, pos_z = 0;
        float  duration = 7.0;

        for ( int i( 0 ); i < 4; ++i ) {
            if ( i == 0 || i == 1 ) {
                if ( is_cyberdog2_ == 1 ) {
                    if ( current_time < 0.005 )
                        leg_angle_cmd_init_.segment( i * 3, 3 ) = ForwardKinematic< T >( *this->data_->quadruped, i, leg_cmd_old_[ i ].q_des );
                    foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) = leg_angle_cmd_init_.segment( ( i / 2 ) * 6, 6 );
                    switch ( this->data_->robot_current_state->gait_id ) {
                    case 0:  // Thanks
                    case 10:
                        if ( t_thk < time_unit ) {
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << -0.06, -0.03, -0.22, -0.06, 0.03, -0.22;
                        }
                        else if ( t_thk < 2 * time_unit ) {
                            n = 1;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << -0.06, -0.03, -0.22, -0.06, 0.03, -0.22;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0.15, -0.03, -0.14, 0.15, 0.03, -0.14;
                        }
                        else if ( t_thk < 3 * time_unit ) {
                            n = 2;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0.15, -0.03, -0.14, 0.15, 0.03, -0.14;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << -0.06, -0.03, -0.22, -0.06, 0.03, -0.22;
                        }
                        else if ( t_thk < 4 * time_unit ) {
                            n = 3;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << -0.06, -0.03, -0.22, -0.06, 0.03, -0.22;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0.15, -0.03, -0.14, 0.15, 0.03, -0.14;
                        }
                        else if ( t_thk < 5 * time_unit ) {
                            n = 4;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0.15, -0.03, -0.14, 0.15, 0.03, -0.14;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << -0.06, -0.03, -0.22, -0.06, 0.03, -0.22;
                        }
                        else if ( t_thk < 6 * time_unit ) {
                            n = 5;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << -0.06, -0.03, -0.22, -0.06, 0.03, -0.22;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.09, -0.24, 0, 0.09, -0.24;
                        }
                        foot_pos_des_.segment( i * 3, 3 ) =
                            foot_pos_init_.segment( i * 3, 3 )
                            + ( foot_pos_goal_.segment( i * 3, 3 ) - foot_pos_init_.segment( i * 3, 3 ) ) * ( ( t_thk - n * time_unit ) > time_unit ? 1 : ( t_thk - n * time_unit ) / time_unit );
                        break;
                    case 1:  // Heart
                    case 11:
                        // 0.08, 0.03,-0.2, 0.08, -0.03, -0.2
                        if ( t_xin < time_unit + 0.01 ) {
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.09, -0.24, 0, 0.09, -0.24;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0.15, 0.03, -0.20, 0.15, -0.03, -0.20;
                            foot_pos_des_.segment( i * 3, 3 ) = foot_pos_init_.segment( i * 3, 3 )
                                                                + ( foot_pos_goal_.segment( i * 3, 3 ) - foot_pos_init_.segment( i * 3, 3 ) ) * ( ( t_xin ) > time_unit ? 1 : ( t_xin ) / time_unit );
                        }
                        else if ( t_xin < 5 * time_unit ) {
                            a_2 = 0, pos_x = 0, pos_y = 0;
                            if ( t_xin < 4 * time_unit ) {
                                a_2   = ( 1 - 2 * ( t_xin - time_unit ) / ( 3 * time_unit ) ) * ( M_PI / 2.0 );
                                pos_y = 0.18 * cos( a_2 );
                                pos_x = 0.1 * ( sin( a_2 ) + pow( cos( a_2 ), 2 / 3.0 ) + 0.2 );
                                if ( t_xin < 2 * time_unit )
                                    pos_z = -0.20 + 0.08 * ( t_xin - time_unit ) / time_unit;
                                else
                                    pos_z = -0.12 - 0.15 * ( t_xin - 2 * time_unit ) / ( 2 * time_unit );
                            }
                            else {
                                pos_x = -0.07;
                                pos_y = 0;
                                pos_z = -0.27;
                            }
                            foot_pos_des_.segment( i * 3, 3 ) << pos_x, ( -0.03 + pos_y ) * ( ( i % 2 ) * 2 - 1 ), pos_z;
                        }
                        else if ( t_xin < 7 * time_unit ) {
                            n = 5;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << -0.07, 0.03, -0.27, -0.07, -0.03, -0.27;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.09, -0.24, 0, 0.09, -0.24;
                            foot_pos_des_.segment( i * 3, 3 ) = foot_pos_init_.segment( i * 3, 3 )
                                                                + ( foot_pos_goal_.segment( i * 3, 3 ) - foot_pos_init_.segment( i * 3, 3 ) )
                                                                      * ( ( t_xin - n * time_unit ) > 2 * time_unit ? 1 : ( t_xin - n * time_unit ) / ( 2 * time_unit ) );
                        }
                        break;

                    case 2:  // Welcome
                    case 12:
                        if ( t_wel < time_unit ) {
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.08, -0.2, 0, 0.08, -0.2;
                        }
                        else if ( t_wel < 2 * time_unit ) {
                            n = 1;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.08, -0.2, 0, 0.08, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, 0.04, -0.2, 0, -0.04, -0.2;
                        }
                        else if ( t_wel < 3 * time_unit ) {
                            n = 2;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0, 0.04, -0.2, 0, -0.04, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.08, -0.2, 0, 0.08, -0.2;
                        }
                        else if ( t_wel < 4 * time_unit ) {
                            n = 3;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.08, -0.2, 0, 0.08, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, 0.04, -0.2, 0, -0.04, -0.2;
                        }
                        else if ( t_wel < 5 * time_unit ) {
                            n = 4;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0, 0.04, -0.2, 0, -0.04, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.08, -0.2, 0, 0.08, -0.2;
                        }
                        else if ( t_wel < 6 * time_unit ) {
                            n = 5;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.08, -0.2, 0, 0.08, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.09, -0.25, 0, 0.09, -0.25;
                        }
                        foot_pos_des_.segment( i * 3, 3 ) =
                            foot_pos_init_.segment( i * 3, 3 )
                            + ( foot_pos_goal_.segment( i * 3, 3 ) - foot_pos_init_.segment( i * 3, 3 ) ) * ( ( t_wel - n * time_unit ) > time_unit ? 1 : ( t_wel - n * time_unit ) / time_unit );
                        break;
                    case 3:  // Hand up and Down
                    case 13:
                        time_unit = 0.4;
                        if ( t_thk < m * time_unit ) {
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << -0.15, -0.03, -0.22, 0.15, 0.03, -0.14;
                            foot_pos_des_.segment( i * 3, 3 ) = foot_pos_init_.segment( i * 3, 3 )
                                                                + ( foot_pos_goal_.segment( i * 3, 3 ) - foot_pos_init_.segment( i * 3, 3 ) )
                                                                      * ( ( t_thk - 0.0 * time_unit ) > ( m * time_unit ) ? 1 : ( t_thk - 0.0 * time_unit ) / ( m * time_unit ) );
                        }
                        else if ( t_thk < 1 * time_unit ) {
                            foot_pos_des_.segment( ( i / 2 ) * 6, 6 ) << -0.15, -0.03, -0.22, 0.15, 0.03, -0.14;
                        }
                        else if ( t_thk < ( 1 + m ) * time_unit ) {
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << -0.15, -0.03, -0.22, 0.15, 0.03, -0.14;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0.15, -0.03, -0.14, -0.15, 0.03, -0.22;
                            foot_pos_des_.segment( i * 3, 3 ) = foot_pos_init_.segment( i * 3, 3 )
                                                                + ( foot_pos_goal_.segment( i * 3, 3 ) - foot_pos_init_.segment( i * 3, 3 ) )
                                                                      * ( ( t_thk - 1.0 * time_unit ) > ( m * time_unit ) ? 1 : ( t_thk - 1.0 * time_unit ) / ( m * time_unit ) );
                        }
                        else if ( t_thk < 2 * time_unit ) {
                            foot_pos_des_.segment( ( i / 2 ) * 6, 6 ) << 0.15, -0.03, -0.14, -0.15, 0.03, -0.22;
                        }
                        else if ( t_thk < ( 2.0 + m ) * time_unit ) {
                            n = 2;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0.15, -0.03, -0.14, -0.15, 0.03, -0.22;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << -0.15, -0.03, -0.22, 0.15, 0.03, -0.14;
                            foot_pos_des_.segment( i * 3, 3 ) = foot_pos_init_.segment( i * 3, 3 )
                                                                + ( foot_pos_goal_.segment( i * 3, 3 ) - foot_pos_init_.segment( i * 3, 3 ) )
                                                                      * ( ( t_thk - 2.0 * time_unit ) > ( m * time_unit ) ? 1 : ( t_thk - 2.0 * time_unit ) / ( m * time_unit ) );
                        }
                        else if ( t_thk < 3 * time_unit ) {
                            foot_pos_des_.segment( ( i / 2 ) * 6, 6 ) << -0.15, -0.03, -0.22, 0.15, 0.03, -0.14;
                        }
                        else {
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << -0.15, -0.03, -0.22, 0.15, 0.03, -0.14;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.09, -0.24, 0, 0.09, -0.24;
                            foot_pos_des_.segment( i * 3, 3 ) = foot_pos_init_.segment( i * 3, 3 )
                                                                + ( foot_pos_goal_.segment( i * 3, 3 ) - foot_pos_init_.segment( i * 3, 3 ) )
                                                                      * ( ( t_thk - 3.0 * time_unit ) > ( 1 * time_unit ) ? 1 : ( t_thk - 3.0 * time_unit ) / ( 0.5 * time_unit ) );
                            if ( t_thk > 3.5 * time_unit - 0.004 && i == 1 )
                                current_time = 7 * time_unit;
                        }
                        break;
                    case 6:  // Stop when contact to obstacle
                        duration  = 10;
                        time_unit = 1.1;
                        CheckContactObstacle( ( int8_t )i, current_time, time_unit );
                        break;
                    case 7:  // Rebound when contact to obstacle
                        duration  = 10;
                        time_unit = 1.1;
                        CheckContactObstacle( ( int8_t )i, current_time, time_unit, true );
                        break;

                    default:
                        foot_pos_des_.segment( i * 3, 3 ) << 0, 0.1 * ( ( i % 2 ) * 2 - 1 ), -0.3;
                        break;
                    }
                }
                else {
                    switch ( this->data_->robot_current_state->gait_id ) {
                    case 0:  // Thanks
                        if ( t_thk < time_unit ) {
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.1, -0.25, 0, 0.1, -0.25;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << -0.1, -0.01, -0.2, -0.1, 0.01, -0.2;
                        }
                        else if ( t_thk < 2 * time_unit ) {
                            n = 1;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << -0.1, -0.01, -0.2, -0.1, 0.01, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0.1, -0.01, -0.2, 0.1, 0.01, -0.2;
                        }
                        else if ( t_thk < 3 * time_unit ) {
                            n = 2;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0.1, -0.01, -0.2, 0.1, 0.01, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << -0.1, -0.01, -0.2, -0.1, 0.01, -0.2;
                        }
                        else if ( t_thk < 4 * time_unit ) {
                            n = 3;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << -0.1, -0.01, -0.2, -0.1, 0.01, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0.1, -0.01, -0.2, 0.1, 0.01, -0.2;
                        }
                        else if ( t_thk < 5 * time_unit ) {
                            n = 4;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0.1, -0.01, -0.2, 0.1, 0.01, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << -0.1, -0.01, -0.2, -0.1, 0.01, -0.2;
                        }
                        else if ( t_thk < 6 * time_unit ) {
                            n = 5;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << -0.1, -0.01, -0.2, -0.1, 0.01, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.1, -0.25, 0, 0.1, -0.25;
                        }
                        foot_pos_des_.segment( i * 3, 3 ) =
                            foot_pos_init_.segment( i * 3, 3 )
                            + ( foot_pos_goal_.segment( i * 3, 3 ) - foot_pos_init_.segment( i * 3, 3 ) ) * ( ( t_thk - n * time_unit ) > time_unit ? 1 : ( t_thk - n * time_unit ) / time_unit );
                        break;
                    case 1:  // Heart
                        // 0.08, 0.03,-0.2, 0.08, -0.03, -0.2
                        if ( t_xin < time_unit + 0.03 ) {
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.1, -0.25, 0, 0.1, -0.25;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0.08, -0.03, -0.2, 0.08, 0.03, -0.2;
                            foot_pos_des_.segment( i * 3, 3 ) = foot_pos_init_.segment( i * 3, 3 )
                                                                + ( foot_pos_goal_.segment( i * 3, 3 ) - foot_pos_init_.segment( i * 3, 3 ) ) * ( ( t_xin ) > time_unit ? 1 : ( t_xin ) / time_unit );
                        }
                        else if ( t_xin < 5 * time_unit ) {
                            a_2 = 0, pos_x = 0, pos_y = 0;
                            if ( t_xin < 4 * time_unit ) {
                                a_2   = ( 1 - 2 * ( t_xin - time_unit ) / ( 3 * time_unit ) ) * ( M_PI / 2.0 );
                                pos_y = 0.15 * cos( a_2 );
                                pos_x = 0.10 * ( sin( a_2 ) + pow( cos( a_2 ), 2 / 3.0 ) - 0.25 );
                            }
                            else {
                                pos_x = -0.125;
                                pos_y = 0;
                            }
                            foot_pos_des_.segment( i * 3, 3 ) << pos_x, ( -0.03 + pos_y ) * ( ( i % 2 ) * 2 - 1 ), -0.2;
                            // std::cout<<"i=" <<i<<"leg_angle_des=" <<leg_angle_des <<std::endl;
                        }
                        else if ( t_xin < 7 * time_unit ) {
                            n = 5;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << -0.125, 0.03, -0.2, -0.125, -0.03, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.1, -0.25, 0, 0.1, -0.25;
                            foot_pos_des_.segment( i * 3, 3 ) = foot_pos_init_.segment( i * 3, 3 )
                                                                + ( foot_pos_goal_.segment( i * 3, 3 ) - foot_pos_init_.segment( i * 3, 3 ) )
                                                                      * ( ( t_xin - n * time_unit ) > 2 * time_unit ? 1 : ( t_xin - n * time_unit ) / ( 2 * time_unit ) );
                        }
                        break;
                    case 2:  // Welcome
                        if ( t_wel < time_unit ) {
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.1, -0.25, 0, 0.1, -0.25;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.08, -0.2, 0, 0.08, -0.2;
                        }
                        else if ( t_wel < 2 * time_unit ) {
                            n = 1;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.08, -0.2, 0, 0.08, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, 0.04, -0.2, 0, -0.04, -0.2;
                        }
                        else if ( t_wel < 3 * time_unit ) {
                            n = 2;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0, 0.04, -0.2, 0, -0.04, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.08, -0.2, 0, 0.08, -0.2;
                        }
                        else if ( t_wel < 4 * time_unit ) {
                            n = 3;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.08, -0.2, 0, 0.08, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, 0.04, -0.2, 0, -0.04, -0.2;
                        }
                        else if ( t_wel < 5 * time_unit ) {
                            n = 4;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0, 0.04, -0.2, 0, -0.04, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.08, -0.2, 0, 0.08, -0.2;
                        }
                        else if ( t_wel < 6 * time_unit ) {
                            n = 5;
                            foot_pos_init_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.08, -0.2, 0, 0.08, -0.2;
                            foot_pos_goal_.segment( ( i / 2 ) * 6, 6 ) << 0, -0.1, -0.25, 0, 0.1, -0.25;
                        }
                        foot_pos_des_.segment( i * 3, 3 ) =
                            foot_pos_init_.segment( i * 3, 3 )
                            + ( foot_pos_goal_.segment( i * 3, 3 ) - foot_pos_init_.segment( i * 3, 3 ) ) * ( ( t_wel - n * time_unit ) > time_unit ? 1 : ( t_wel - n * time_unit ) / time_unit );
                        break;
                    default:
                        foot_pos_des_.segment( i * 3, 3 ) << 0, 0.1 * ( ( i % 2 ) * 2 - 1 ), -0.3;
                        break;
                    }
                }
                leg_angle_des = InverseKinematic< T >( *this->data_->quadruped, i, foot_pos_des_.segment( i * 3, 3 ) );

                this->data_->leg_controller->commands_[ i ].q_des = leg_angle_des;
                this->data_->leg_controller->commands_[ i ].qd_des << 0, 0, 0;
                this->data_->leg_controller->commands_[ i ].kp_joint << 50, 0, 0, 0, 50, 0, 0, 0, 80;
                this->data_->leg_controller->commands_[ i ].kd_joint << 1, 0, 0, 0, 1, 0, 0, 0, 2;
                this->data_->leg_controller->commands_[ i ].tau_feed_forward << 0, 0, 0;
                this->data_->leg_controller->commands_[ i ].force_feed_forward << 0, 0, 0;
            }
            else {
                leg_angle_des                                     = rear_leg_angle_4_;
                this->data_->leg_controller->commands_[ i ].q_des = leg_angle_des;
                this->data_->leg_controller->commands_[ i ].qd_des << 0, 0, 0;
                this->data_->leg_controller->commands_[ i ].kp_joint << 80, 0, 0, 0, 100, 0, 0, 0, 150;
                this->data_->leg_controller->commands_[ i ].kd_joint << 1, 0, 0, 0, 5, 0, 0, 0, 10;
                this->data_->leg_controller->commands_[ i ].tau_feed_forward << 0, -5, 0;
                this->data_->leg_controller->commands_[ i ].force_feed_forward << 0, 0, 0;
                if ( is_cyberdog2_ == 1 ) {
                    this->data_->leg_controller->commands_[ i ].kp_joint *= 0.5;
                    this->data_->leg_controller->commands_[ i ].tau_feed_forward *= 0.2;
                }
                this->data_->leg_controller->commands_[ i ] = leg_cmd_old_[ i ];
            }
            // set contact states by scheduler
            contact_states_ << 0.0, 0.0, 0.5, 0.5;

            if ( current_time >= duration * time_unit ) {
                if ( this->data_->robot_current_state->gait_id )
                    jump_flag_ = 7;
                else
                    jump_flag_ = 3;
                jump_iter_                 = 0;
                this->motion_progress_bar_ = 75;
                leg_angle_log_ << this->data_->leg_controller->commands_[ 0 ].q_des, this->data_->leg_controller->commands_[ 1 ].q_des, this->data_->leg_controller->commands_[ 2 ].q_des,
                    this->data_->leg_controller->commands_[ 3 ].q_des;
                std::cout << "[FSM TWOLEGSTAND] current time = " << current_time << std::endl;
                std::cout << "[FSM TWOLEGSTAND] jump_flag_ = " << jump_flag_ << std::endl;
            }
            if ( jump_pitch_ < safe_angle_upp_ ) {  // overturn, fail
                jump_flag_                 = 4;
                jump_iter_                 = 0;
                this->motion_progress_bar_ = 0;
                leg_angle_log_ << this->data_->leg_controller->commands_[ 0 ].q_des, this->data_->leg_controller->commands_[ 1 ].q_des, this->data_->leg_controller->commands_[ 2 ].q_des,
                    this->data_->leg_controller->commands_[ 3 ].q_des;
                std::cout << "[FSM TWOLEGSTAND] current time = " << current_time << std::endl;
                std::cout << "[FSM TWOLEGSTAND] jump_flag_ = " << jump_flag_ << std::endl;
            }
            if ( jump_pitch_ > safe_angle_low_ ) {  // too low, fail
                jump_flag_                 = 5;
                jump_iter_                 = 0;
                this->motion_progress_bar_ = 0;
                leg_angle_log_ << this->data_->leg_controller->commands_[ 0 ].q_des, this->data_->leg_controller->commands_[ 1 ].q_des, this->data_->leg_controller->commands_[ 2 ].q_des,
                    this->data_->leg_controller->commands_[ 3 ].q_des;
                std::cout << "[FSM TWOLEGSTAND] current time = " << current_time << std::endl;
                std::cout << "[FSM TWOLEGSTAND] jump_flag_ = " << jump_flag_ << std::endl;
            }
        }
    }
    else if ( jump_flag_ == 3 ) {  // recover
        double time_unit = 0.6;    // 1
        for ( int i = 0; i < 4; ++i ) {
            if ( current_time < 2 * time_unit ) {
                leg_angle_init_.segment( i * 3, 3 ) = leg_angle_log_.segment( i * 3, 3 );
                leg_angle_goal_.segment( i * 3, 3 ) = stand_j_pos_[ i ];
                if ( is_cyberdog2_ == 1 ) {
                    if ( i == 2 || i == 3 )
                        leg_angle_goal_.segment( i * 3, 3 ) << 0, Deg2Rad( -60. ), Deg2Rad( 105. );
                    else
                        leg_angle_goal_.segment( i * 3, 3 ) << 0, Deg2Rad( -60. ), Deg2Rad( 120. );
                }
                leg_angle_des_.segment( i * 3, 3 ) =
                    leg_angle_init_.segment( i * 3, 3 )
                    + ( leg_angle_goal_.segment( i * 3, 3 ) - leg_angle_init_.segment( i * 3, 3 ) ) * ( current_time > 2 * time_unit ? 1 : current_time / ( time_unit * 2 ) );
                leg_angle_temp_.segment( i * 3, 3 ) = leg_angle_des_.segment( i * 3, 3 );
                this->data_->leg_controller->commands_[ i ].q_des << leg_angle_des_.segment( i * 3, 3 );
                this->data_->leg_controller->commands_[ i ].qd_des << 0, 0, 0;
                this->data_->leg_controller->commands_[ i ].kp_joint << 100, 0, 0, 0, 60, 0, 0, 0, 30;
                this->data_->leg_controller->commands_[ i ].kd_joint << 2, 0, 0, 0, 4, 0, 0, 0, 6;
                this->data_->leg_controller->commands_[ i ].tau_feed_forward << 0, 0, 0;
                this->data_->leg_controller->commands_[ i ].force_feed_forward << 0, 0, 0;
                // set contact states by scheduler, determine whether contact ground by knee joint's torque
                if ( current_time < 0.96 )
                    contact_states_ << 0.0, 0.0, 0.5, 0.5;
                else
                    contact_states_ << 0.5, 0.5, 0.5, 0.5;
            }
            else {
                float t_tmp                         = ( current_time - 2 * time_unit > time_unit ? 1 : ( current_time - 2 * time_unit ) / time_unit );
                leg_angle_init_.segment( i * 3, 3 ) = leg_angle_temp_.segment( i * 3, 3 );
                leg_angle_goal_.segment( i * 3, 3 ) = stand_j_pos_[ i ];
                leg_angle_des_.segment( i * 3, 3 )  = leg_angle_init_.segment( i * 3, 3 ) + ( leg_angle_goal_.segment( i * 3, 3 ) - leg_angle_init_.segment( i * 3, 3 ) ) * t_tmp;
                this->data_->leg_controller->commands_[ i ].q_des << leg_angle_des_.segment( i * 3, 3 );
                this->data_->leg_controller->commands_[ i ].qd_des << 0, 0, 0;
                this->data_->leg_controller->commands_[ i ].kp_joint << 100, 0, 0, 0, 60 + 60 * t_tmp, 0, 0, 0, 30 + 90 * t_tmp;
                this->data_->leg_controller->commands_[ i ].kd_joint << 2, 0, 0, 0, 2, 0, 0, 0, 6 - 4 * t_tmp;
                this->data_->leg_controller->commands_[ i ].tau_feed_forward << 0, 0, 0;
                this->data_->leg_controller->commands_[ i ].force_feed_forward << 0, 0, 0;
                // set contact states by scheduler
                contact_states_ << 0.5, 0.5, 0.5, 0.5;
            }
        }
        if ( current_time >= 3 * time_unit )
            this->motion_progress_bar_ = 100;
    }
    else if ( jump_flag_ == 4 ) {  // handle of overturn failure
        double time_unit = 0.15;
        for ( int i( 0 ); i < 4; ++i ) {
            if ( i == 0 || i == 1 ) {
                if ( jump_iter_ == 0 || jump_iter_ == 1 ) {
                    leg_angle_init_.segment( i * 3, 3 ) = leg_angle_log_.segment( i * 3, 3 );
                    if ( is_cyberdog2_ == 1 )
                        leg_angle_goal_.segment( i * 3, 3 ) << 0, this->data_->leg_controller->q_fronthip_lowerbound_ + 0.1, this->data_->leg_controller->q_knee_lowerbound_ + 0.1;
                    else
                        leg_angle_goal_.segment( i * 3, 3 ) << 0, Deg2Rad( -220. ), Deg2Rad( 90. );
                }
                leg_angle_des_.segment( i * 3, 3 ) =
                    leg_angle_init_.segment( i * 3, 3 ) + ( leg_angle_goal_.segment( i * 3, 3 ) - leg_angle_init_.segment( i * 3, 3 ) ) * ( current_time > time_unit ? 1 : current_time / time_unit );
                this->data_->leg_controller->commands_[ i ].q_des = leg_angle_des_.segment( i * 3, 3 );
                this->data_->leg_controller->commands_[ i ].qd_des << 0, 0, 0;
                this->data_->leg_controller->commands_[ i ].kp_joint << 50, 0, 0, 0, 50, 0, 0, 0, 50;
                this->data_->leg_controller->commands_[ i ].kd_joint << 1, 0, 0, 0, 2, 0, 0, 0, 2;
            }
            else {
                if ( is_cyberdog2_ == 1 )
                    this->data_->leg_controller->commands_[ i ].q_des = rear_leg_angle_4_;
                else
                    this->data_->leg_controller->commands_[ i ].q_des << 0, Deg2Rad( -120.0 ), M_PI / 2 + Deg2Rad( 20.0 + this->data_->user_parameters->knee_angle_adj );
                this->data_->leg_controller->commands_[ i ].qd_des << 0, 0, 0;
                this->data_->leg_controller->commands_[ i ].kp_joint << 80, 0, 0, 0, 80, 0, 0, 0, 80;
                this->data_->leg_controller->commands_[ i ].kd_joint << 1, 0, 0, 0, 2, 0, 0, 0, 2;
            }
            this->data_->leg_controller->commands_[ i ].tau_feed_forward << 0, 0, 0;
            this->data_->leg_controller->commands_[ i ].force_feed_forward << 0, 0, 0;
        }
        if ( current_time >= 2 )
            this->motion_progress_bar_ = 100;
    }
    else if ( jump_flag_ == 5 ) {  // handle of low tilting
        double time_unit = 0.15;
        for ( int i( 0 ); i < 4; ++i ) {
            if ( jump_iter_ == 1 ) {
                leg_angle_init_.segment( i * 3, 3 ) = leg_angle_log_.segment( i * 3, 3 );
                leg_angle_goal_.segment( i * 3, 3 ) << 0, Deg2Rad( -45. ), Deg2Rad( 90. );
            }
            leg_angle_des_.segment( i * 3, 3 ) =
                leg_angle_init_.segment( i * 3, 3 ) + ( leg_angle_goal_.segment( i * 3, 3 ) - leg_angle_init_.segment( i * 3, 3 ) ) * ( current_time > time_unit ? 1 : current_time / time_unit );
            this->data_->leg_controller->commands_[ i ].q_des << leg_angle_des_.segment( i * 3, 3 );
            this->data_->leg_controller->commands_[ i ].qd_des << 0, 0, 0;
            this->data_->leg_controller->commands_[ i ].kp_joint << 30, 0, 0, 0, 30, 0, 0, 0, 30;
            this->data_->leg_controller->commands_[ i ].kd_joint << 5, 0, 0, 0, 5, 0, 0, 0, 6;
            this->data_->leg_controller->commands_[ i ].tau_feed_forward << 0, 0, 0;
            this->data_->leg_controller->commands_[ i ].force_feed_forward << 0, 0, 0;
        }
        if ( current_time >= 2 )
            this->motion_progress_bar_ = 100;
    }
    else if ( jump_flag_ == 7 ) {  // faster recover
        double time_unit = 0.2;
        for ( int i = 0; i < 4; ++i ) {
            if ( current_time < 2 * time_unit ) {
                leg_angle_init_.segment( i * 3, 3 ) = leg_angle_log_.segment( i * 3, 3 );
                if ( i == 0 || i == 1 )
                    leg_angle_goal_.segment( i * 3, 3 ) = leg_angle_init_.segment( i * 3, 3 ) + Vec3< T >( 0, Deg2Rad( 10. ), 0 );
                else
                    leg_angle_goal_.segment( i * 3, 3 ) << 0, Deg2Rad( -70. ), Deg2Rad( 120. );  //-1.84203  -82.7927   92.6037

                leg_angle_des_.segment( i * 3, 3 ) =
                    leg_angle_init_.segment( i * 3, 3 )
                    + ( leg_angle_goal_.segment( i * 3, 3 ) - leg_angle_init_.segment( i * 3, 3 ) ) * ( current_time > 2 * time_unit ? 1 : current_time / ( time_unit * 2 ) );
                leg_angle_temp_.segment( i * 3, 3 ) = leg_angle_des_.segment( i * 3, 3 );

                this->data_->leg_controller->commands_[ i ].q_des << leg_angle_des_.segment( i * 3, 3 );
                this->data_->leg_controller->commands_[ i ].qd_des << 0, 0, 0;
                this->data_->leg_controller->commands_[ i ].kp_joint << 50, 0, 0, 0, 50, 0, 0, 0, 30;
                this->data_->leg_controller->commands_[ i ].kd_joint << 2, 0, 0, 0, 4, 0, 0, 0, 4;
                this->data_->leg_controller->commands_[ i ].tau_feed_forward << 0, 0, 0;
                this->data_->leg_controller->commands_[ i ].force_feed_forward << 0, 0, 0;
                // set contact states by scheduler, determine whether contact ground by knee joint's torque
                contact_states_ << 0.0, 0.0, 0.5, 0.5;
            }
            else {
                float _t                            = ( current_time - 2 * time_unit > time_unit ? 1 : ( current_time - 2 * time_unit ) / time_unit );
                float _t2                           = ( current_time - 3 * time_unit > time_unit ? 1 : ( current_time - 3 * time_unit ) / time_unit );
                leg_angle_init_.segment( i * 3, 3 ) = leg_angle_temp_.segment( i * 3, 3 );
                leg_angle_goal_.segment( i * 3, 3 ) = stand_j_pos_[ i ];
                leg_angle_des_.segment( i * 3, 3 )  = leg_angle_init_.segment( i * 3, 3 ) + ( leg_angle_goal_.segment( i * 3, 3 ) - leg_angle_init_.segment( i * 3, 3 ) ) * _t;
                this->data_->leg_controller->commands_[ i ].q_des << leg_angle_des_.segment( i * 3, 3 );
                this->data_->leg_controller->commands_[ i ].qd_des << 0, 0, 0;
                this->data_->leg_controller->commands_[ i ].kp_joint << 40, 0, 0, 0, 30, 0, 0, 0, 30;
                this->data_->leg_controller->commands_[ i ].kd_joint << 2, 0, 0, 0, 2, 0, 0, 0, 4 - 2 * ( _t2 > 0 ? _t2 : 0 );
                this->data_->leg_controller->commands_[ i ].tau_feed_forward << 0, 0, 0;
                this->data_->leg_controller->commands_[ i ].force_feed_forward << 0, 0, 0;
                // set contact states by scheduler
            }
        }
        if ( current_time >= 4 * time_unit ) {
            this->motion_progress_bar_ = 100;
        }
    }
}

template < typename T > void FsmStateTwoLegStand< T >::CheckContactObstacle( int8_t leg_id, float current_time, float unit_period, bool rebound_after_contact ) {
    const int    swing_leg_id_one  = 0;
    const int    swing_leg_id_two  = 0;
    float        move_dist         = 0.3;
    float        base_pitch        = 0.79;
    static int   contact_iter      = 0;
    static int   free_iter         = 0;
    static int   protect_iter      = 0;
    static bool  protect_flag[ 2 ] = { false };
    static float phase_offset      = M_PI / 6.0;

    float torque_threshold_contact_down = -0.25;
    float torque_threshold_contact_up   = 0.5;
    float torque_threshold_free         = 0.2;

    Vec3< float > move_pos;
    move_pos << move_dist * sin( base_pitch ), 0.0, move_dist * cos( base_pitch );

    float filter                           = 0.05;
    joint_torque_filter_[ 2 * leg_id ]     = ( 1 - filter ) * joint_torque_filter_[ 2 * leg_id ] + filter * this->data_->leg_controller->datas_[ leg_id ].tau_actual( 1 );
    joint_torque_filter_[ 2 * leg_id + 1 ] = ( 1 - filter ) * joint_torque_filter_[ 2 * leg_id + 1 ] + filter * this->data_->leg_controller->datas_[ leg_id ].tau_actual( 2 );

    if ( leg_id == swing_leg_id_one || leg_id == swing_leg_id_two ) {
        // check joint torque if contact obstacle
        if ( !protect_flag[ leg_id ] && !stop_swing_[ leg_id ]
             && ( ( joint_torque_filter_[ 2 * leg_id ] < torque_threshold_contact_down || joint_torque_filter_[ 2 * leg_id + 1 ] < torque_threshold_contact_down )
                  || ( joint_torque_filter_[ 2 * leg_id ] > torque_threshold_contact_up || joint_torque_filter_[ 2 * leg_id + 1 ] > torque_threshold_contact_up ) ) ) {
            contact_iter++;
            if ( contact_iter >= 5 ) {
                stop_swing_[ leg_id ] = true;
                free_iter             = 0;
            }
            if ( contact_iter % 5 == 0 )
                std::cout << "[FSM TWOLEGSTAND] Passive Swing, contact: " << contact_iter << "  " << stop_swing_[ leg_id ] << std::endl;
        }
        if ( !rebound_after_contact ) {  // stop after contacting obstacle
            // check the leg is free
            if ( stop_swing_[ leg_id ] && ( abs( joint_torque_filter_[ 2 * leg_id ] ) < torque_threshold_free && abs( joint_torque_filter_[ 2 * leg_id + 1 ] ) < torque_threshold_free ) ) {
                free_iter++;
                if ( free_iter >= 500 ) {
                    stop_swing_[ leg_id ] = false;
                    contact_iter          = 0;
                }
                if ( free_iter % 50 == 0 )
                    std::cout << "[FSM TWOLEGSTAND] Passive Swing, free: " << free_iter << "  " << stop_swing_[ leg_id ] << std::endl;
            }
        }
        else {  // rebound after contacting obstacle
            if ( stop_swing_[ leg_id ] ) {
                free_iter++;
                if ( free_iter >= 100 ) {
                    stop_swing_[ leg_id ]  = false;
                    protect_flag[ leg_id ] = true;
                    protect_iter           = 0;
                    contact_iter           = 0;
                    float current_phase    = current_time / unit_period - floor( current_time / unit_period );
                    // TODO: make sure phase_up <= 0.5, will run okay
                    float phase_up   = ( M_PI_2 + phase_offset ) / ( 2 * M_PI );
                    float phase_down = ( 3 * M_PI_2 + phase_offset ) / ( 2 * M_PI );

                    if ( current_phase < phase_up ) {
                        jump_iter_ += round( ( phase_up - current_phase ) * 2 * unit_period / this->data_->control_parameters->controller_dt );
                    }
                    else if ( current_phase < phase_down ) {
                        jump_iter_ += round( ( phase_up - current_phase ) * 2 * unit_period / this->data_->control_parameters->controller_dt );
                    }
                    else {
                        jump_iter_ += round( ( phase_up + 1 - current_phase ) * 2 * unit_period / this->data_->control_parameters->controller_dt );
                    }
                }
                if ( free_iter % 50 == 0 )
                    std::cout << "[FSM TWOLEGSTAND] Passive Swing, free: " << free_iter << "  " << stop_swing_[ leg_id ] << std::endl;
            }
            // make the detecter not effort for a moment
            if ( protect_flag[ leg_id ] ) {
                protect_iter++;
                if ( protect_iter > 150 )
                    protect_flag[ leg_id ] = false;
            }
        }

        foot_pos_des_.segment( 3 * leg_id, 3 ) << foot_pos_init_.segment( 3 * leg_id, 3 ) + ( sin( phase_offset ) + sin( current_time / unit_period * 2 * M_PI - phase_offset ) ) * 0.5 * move_pos;
    }
    else {
        foot_pos_des_.segment( 3 * leg_id, 3 ) << foot_pos_init_.segment( 3 * leg_id, 3 );
    }
}

template class FsmStateTwoLegStand< float >;
