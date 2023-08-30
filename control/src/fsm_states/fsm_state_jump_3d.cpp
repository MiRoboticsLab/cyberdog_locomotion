#include "fsm_states/fsm_state_jump_3d.hpp"
#include <fstream>

/**
 * @brief Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param control_fsm_data holds all of the relevant control data
 */
template < typename T > FsmStateJump3d< T >::FsmStateJump3d( ControlFsmData< T >* control_fsm_data ) : FsmState< T >( control_fsm_data, FsmStateName::kJump3d, "jump_3d" ) {
    // Do nothing
    // Set the pre controls safety checks
    this->check_safe_orientation_ = false;
    this->check_robot_lifted_     = false;

    // Post control safety checks
    this->check_desired_foot_position_ = false;
    this->check_feed_forward_force_    = false;

    data_reader_yaw_p90_    = new DataReader( this->data_->quadruped->robot_type_, FsmStateName::kJump3d, JumpId::kJumpPosYaw90 );
    data_reader_x_p60_      = new DataReader( this->data_->quadruped->robot_type_, FsmStateName::kJump3d, JumpId::kJumpPosX60 );
    data_reader_yaw_n90_    = new DataReader( this->data_->quadruped->robot_type_, FsmStateName::kJump3d, JumpId::kJumpNegYaw90 );
    data_reader_z_p30_      = new DataReader( this->data_->quadruped->robot_type_, FsmStateName::kJump3d, JumpId::kJumpPosZ30 );
    data_reader_down_stair_ = new DataReader( this->data_->quadruped->robot_type_, FsmStateName::kJump3d, JumpId::kJumpDownStair );
    data_reader_y_p20_      = new DataReader( this->data_->quadruped->robot_type_, FsmStateName::kJump3d, JumpId::kJumpPosY20 );
    data_reader_x_p30_      = new DataReader( this->data_->quadruped->robot_type_, FsmStateName::kJump3d, JumpId::kJumpPosX30 );
    data_reader_y_n20_      = new DataReader( this->data_->quadruped->robot_type_, FsmStateName::kJump3d, JumpId::kJumpNegY20 );

    jump_ctrl_yaw_p90_    = new OfflineDataCtrl< T >( data_reader_yaw_p90_, this->data_->control_parameters->controller_dt );
    jump_ctrl_x_p60_      = new OfflineDataCtrl< T >( data_reader_x_p60_, this->data_->control_parameters->controller_dt );
    jump_ctrl_yaw_n90_    = new OfflineDataCtrl< T >( data_reader_yaw_n90_, this->data_->control_parameters->controller_dt );
    jump_ctrl_z_p30_      = new OfflineDataCtrl< T >( data_reader_z_p30_, this->data_->control_parameters->controller_dt );
    jump_ctrl_down_stair_ = new OfflineDataCtrl< T >( data_reader_down_stair_, this->data_->control_parameters->controller_dt );
    jump_ctrl_y_p20_      = new OfflineDataCtrl< T >( data_reader_y_p20_, this->data_->control_parameters->controller_dt );
    jump_ctrl_x_p30_      = new OfflineDataCtrl< T >( data_reader_x_p30_, this->data_->control_parameters->controller_dt );
    jump_ctrl_y_n20_      = new OfflineDataCtrl< T >( data_reader_y_n20_, this->data_->control_parameters->controller_dt );
    wbc_ctrl_         = new LocomotionCtrl< T >( control_fsm_data->quadruped->BuildModel() );
    wbc_data_         = new LocomotionCtrlData< T >();
    offline_opt_data_ = new OfflineOptCtrlData< T >();
}

/**
 * @brief Behavior to be carried out when entering a state
 *
 */
template < typename T > void FsmStateJump3d< T >::OnEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    // Reset the transition data
    this->transition_data_.ResetTransitionDone();

    // Reset private variables
    body_rot_matrix_yaw_init_ = RotMat< T >::Zero();
    use_wbc_                  = true;
    first_visit_              = true;
    curr_time_                = 0;
    pre_mode_duration_        = 100;
    landing_delay_            = 100;
    landing_count_            = 0;
    landing_filter_           = 0.995;
    touch_down_delay_         = 0;
    data_end_                 = false;
    height_good_for_trans_    = false;
    use_height_good_          = true;
    flying_phase_             = false;
    touch_down_               = false;
    touch_down_count_         = 0;
    touch_down_threh_         = this->data_->user_parameters->mpc_body_mass * 9.81 * 0.125;
    for ( int i = 0; i < 4; i++ ) {
        joint_pos_end_pos_[ i ] << 0.0, -0.9506, 1.5120;
    }
    weight_vec_.setZero( 6, 1 );
    for ( size_t i( 0 ); i < 4; i++ ) {
        initial_jpos_[ i ] = Vec3< T >::Zero();
    }

    // initial configuration, position
    for ( size_t i( 0 ); i < 4; ++i ) {
        initial_jpos_[ i ] = this->data_->leg_controller->datas_[ i ].q;
    }
    SetJumpId();
    touch_down_count_ = 0;
    touch_down_threh_ = this->data_->user_parameters->mpc_body_mass * 9.81 * 0.125;
    ComputeCommand();
    curr_time_ += this->data_->control_parameters->controller_dt;
}

/**
 * @brief Calls the functions to be executed on each control loop iteration
 *
 */
template < typename T > void FsmStateJump3d< T >::Run() {

    ComputeCommand();
    this->data_->robot_current_state->gait_id = this->data_->command->gait_id;
    curr_time_ += this->data_->control_parameters->controller_dt;
}

/**
 * @brief Set Jump ID based on command
 *
 */
template < typename T > void FsmStateJump3d< T >::SetJumpId() {
    auto cmd           = this->data_->command;
    pre_mode_duration_ = 100;
    use_wbc_           = true;
    kp_swing_          = Vec3< T >::Constant( 40.0 );
    kd_swing_          = Vec3< T >::Constant( 2.5 );
    kp_support_        = Vec3< T >::Constant( 30.0 );
    kd_support_        = Vec3< T >::Constant( 1.0 );
    kp_land_           = Vec3< T >::Constant( 40.0 );
    kd_land_           = Vec3< T >::Constant( 2.5 );
    kp_end_            = Vec3< T >::Constant( 60.0 );
    kd_end_            = Vec3< T >::Constant( 2.5 );
    kp_trans_          = Vec3< T >::Constant( 60.0 );
    kd_trans_          = Vec3< T >::Constant( 2.5 );
    tau_front_ratio_   = Vec3< T >::Constant( 1.0 );
    tau_rear_ratio_    = Vec3< T >::Constant( 1.0 );
    landing_delay_     = 100;
    landing_filter_    = 0.95;

    for ( int i = 0; i < 4; i++ ) {
        joint_pos_end_pos_[ i ] << 0.0, -0.9506, 1.5120;
    }
    this->data_->robot_current_state->gait_id = cmd->gait_id;
    use_height_good_                          = true;

    switch ( cmd->gait_id ) {
    case JumpId::kJumpPosYaw90:
        jump_ctrl_ = jump_ctrl_yaw_p90_;
        kp_swing_ << 40.0, 40.0, 40.0;
        kd_swing_ << 2.5, 2.5, 2.5;
        kp_land_ << 40.0, 40.0, 40.0;
        kd_land_ << 2.5, 2.5, 2.5;
        break;
    case JumpId::kJumpPosX60:
        jump_ctrl_ = jump_ctrl_x_p60_;
        use_wbc_   = false;
        kp_swing_ << 18.0, 18.0, 18.0;
        kd_swing_ << 1.2, 1.2, 1.2;
        kp_land_ << 18.0, 24.0, 24.0;
        kd_land_ << 1.2, 2.5, 2.5;
        break;
    case JumpId::kJumpNegYaw90:
        jump_ctrl_ = jump_ctrl_yaw_n90_;
        kp_swing_ << 40.0, 40.0, 40.0;
        kd_swing_ << 2.5, 2.5, 2.5;
        kp_land_ << 40.0, 40.0, 40.0;
        kd_land_ << 2.5, 2.5, 2.5;
        break;
    case JumpId::kJumpPosZ30:
        jump_ctrl_ = jump_ctrl_z_p30_;
        kp_swing_ << 40.0, 40.0, 40.0;
        kd_swing_ << 2.5, 2.5, 2.5;
        kp_land_ << 40.0, 40.0, 40.0;
        kd_land_ << 2.5, 2.5, 2.5;
        break;
    case JumpId::kJumpDownStair:
        jump_ctrl_ = jump_ctrl_down_stair_;
        kp_swing_ << 15.0, 15.0, 15.0;
        kd_swing_ << 1.0, 1.0, 1.0;
        kp_land_ << 15.0, 15.0, 15.0;
        kd_land_ << 1.0, 1.0, 1.0;
        use_height_good_ = false;
        break;
    case JumpId::kJumpPosX30:
        jump_ctrl_ = jump_ctrl_x_p30_;
        use_wbc_   = false;
        kp_swing_ << 30.0, 30.0, 30.0;
        kd_swing_ << 2.0, 2.0, 2.0;
        kp_land_ << 30.0, 30.0, 30.0;
        kd_land_ << 2.0, 2.0, 2.0;
        break;
    case JumpId::kJumpNegY20:
        jump_ctrl_ = jump_ctrl_y_n20_;
        use_wbc_   = false;
        kp_swing_ << 40.0, 40.0, 40.0;
        kd_swing_ << 2.5, 2.5, 2.5;
        kp_land_ << 40.0, 40.0, 40.0;
        kd_land_ << 2.5, 2.5, 2.5;
        break;
    case JumpId::kJumpPosY20:
        jump_ctrl_ = jump_ctrl_y_p20_;
        use_wbc_   = false;
        kp_swing_ << 40.0, 40.0, 40.0;
        kd_swing_ << 2.5, 2.5, 2.5;
        kp_land_ << 40.0, 40.0, 40.0;
        kd_land_ << 2.5, 2.5, 2.5;
        break;
    default:
        jump_ctrl_                                = jump_ctrl_yaw_p90_;
        this->data_->robot_current_state->gait_id = 0;
        break;
    }
}

/**
 * @brief Compute joint command based
 *
 */
template < typename T > void FsmStateJump3d< T >::ComputeCommand() {
    static Vec4< T > contact_state_est = Vec4< T >::Constant( 0.5 );
    if ( first_visit_ ) {
        jump_ctrl_->FirstVisit( curr_time_ );
        first_visit_ = false;

        jump_ctrl_->SetBodyInit( this->data_->state_estimator->GetResult().position, this->data_->state_estimator->GetResult().rpy );
    }

    weight_vec_.setZero( 6, 1 );
    for ( int i = 0; i < 6; i++ )
        weight_vec_[ i ] = this->data_->user_parameters->wbc_weight_jump[ i ];

    jump_ctrl_->OfflineDataCtrlRun( curr_time_, false, offline_opt_data_, pre_mode_duration_ );
    data_end_ = jump_ctrl_->GetDataEndFlag();
    TounchDownDetection();

    wbc_ctrl_->SetFloatingBaseWeight( weight_vec_ );

    if ( !data_end_ ) {
        contact_state_est = 0.5 * offline_opt_data_->contact_state;
        this->data_->state_estimator->SetContactPhase( contact_state_est );
        if ( use_wbc_ ) {
            if ( offline_opt_data_->contact_state[ 0 ] < 1e-2 && offline_opt_data_->contact_state[ 1 ] < 1e-2 && offline_opt_data_->contact_state[ 2 ] < 1e-2
                 && offline_opt_data_->contact_state[ 3 ] < 1e-2 ) {
                // All PD
                for ( int i = 0; i < 4; i++ ) {
                    this->data_->leg_controller->commands_[ i ].q_des    = jump_ctrl_->GetJointPosDesire( i );
                    this->data_->leg_controller->commands_[ i ].qd_des   = jump_ctrl_->GetJointVelDesire( i );
                    this->data_->leg_controller->commands_[ i ].kp_joint = kp_swing_.asDiagonal();
                    this->data_->leg_controller->commands_[ i ].kd_joint = kd_swing_.asDiagonal();
                }
            }
            else {
                ConvertData();
                wbc_ctrl_->Run( wbc_data_, *this->data_ );
            }
        }
        else {
            // Front legs
            if ( offline_opt_data_->contact_state[ 0 ] < 1e-2 && offline_opt_data_->contact_state[ 1 ] < 1e-2 ) {
                for ( int i = 0; i < 2; i++ ) {
                    this->data_->leg_controller->commands_[ i ].q_des    = jump_ctrl_->GetJointPosDesire( i );
                    this->data_->leg_controller->commands_[ i ].qd_des   = jump_ctrl_->GetJointVelDesire( i );
                    this->data_->leg_controller->commands_[ i ].kp_joint = kp_swing_.asDiagonal();
                    this->data_->leg_controller->commands_[ i ].kd_joint = kd_swing_.asDiagonal();
                    this->data_->leg_controller->commands_[ i ].tau_feed_forward << 0.0, 0.0, 0.0;
                    // this->data_->leg_controller->commands_[ i ].force_feed_forward << 0.0, 0.0, 0.0;
                }
            }
            else if ( offline_opt_data_->contact_state[ 0 ] > 1e-2 && offline_opt_data_->contact_state[ 1 ] > 1e-2 ) {
                for ( int i = 0; i < 2; i++ ) {
                    this->data_->leg_controller->commands_[ i ].q_des            = jump_ctrl_->GetJointPosDesire( i );
                    this->data_->leg_controller->commands_[ i ].qd_des           = jump_ctrl_->GetJointVelDesire( i );
                    this->data_->leg_controller->commands_[ i ].kp_joint         = kp_support_.asDiagonal();
                    this->data_->leg_controller->commands_[ i ].kd_joint         = kd_support_.asDiagonal();
                    this->data_->leg_controller->commands_[ i ].tau_feed_forward = tau_front_ratio_.asDiagonal() * jump_ctrl_->GetJointTorqueFeedforward( i );
                    // this->data_->leg_controller->commands_[ i ].force_feed_forward = -tau_front_ratio_.asDiagonal() * this->data_->state_estimator->GetResult().world2body_rotation_matrix *
                    // jump_ctrl_->GetReactionForceDesire( i );
                }
            }
            // Rear legs
            if ( offline_opt_data_->contact_state[ 2 ] < 1e-2 && offline_opt_data_->contact_state[ 3 ] < 1e-2 ) {
                for ( int i = 2; i < 4; i++ ) {
                    this->data_->leg_controller->commands_[ i ].q_des    = jump_ctrl_->GetJointPosDesire( i );
                    this->data_->leg_controller->commands_[ i ].qd_des   = jump_ctrl_->GetJointVelDesire( i );
                    this->data_->leg_controller->commands_[ i ].kp_joint = kp_swing_.asDiagonal();
                    this->data_->leg_controller->commands_[ i ].kd_joint = kd_swing_.asDiagonal();
                    this->data_->leg_controller->commands_[ i ].tau_feed_forward << 0.0, 0.0, 0.0;
                    // this->data_->leg_controller->commands_[ i ].force_feed_forward << 0.0, 0.0, 0.0;
                }
            }
            else if ( offline_opt_data_->contact_state[ 2 ] > 1e-2 && offline_opt_data_->contact_state[ 3 ] > 1e-2 ) {
                for ( int i = 2; i < 4; i++ ) {
                    this->data_->leg_controller->commands_[ i ].q_des            = jump_ctrl_->GetJointPosDesire( i );
                    this->data_->leg_controller->commands_[ i ].qd_des           = jump_ctrl_->GetJointVelDesire( i );
                    this->data_->leg_controller->commands_[ i ].kp_joint         = kp_support_.asDiagonal();
                    this->data_->leg_controller->commands_[ i ].kd_joint         = kd_support_.asDiagonal();
                    this->data_->leg_controller->commands_[ i ].tau_feed_forward = tau_rear_ratio_.asDiagonal() * jump_ctrl_->GetJointTorqueFeedforward( i );
                    // this->data_->leg_controller->commands_[ i ].force_feed_forward = -tau_rear_ratio_.asDiagonal() * this->data_->state_estimator->GetResult().world2body_rotation_matrix *
                    // jump_ctrl_->GetReactionForceDesire( i );
                }
            }
        }
    }
    else {
        contact_state_est << 0.5, 0.5, 0.5, 0.5;
        this->data_->state_estimator->SetContactPhase( contact_state_est );
        if ( !touch_down_ ) {
            // contact_state_est = 0.5 * offline_opt_data_->contact_state; // hold data last contact_state
            // this->data_->state_estimator->SetContactPhase( contact_state_est );
            for ( int i = 0; i < 4; i++ ) {
                this->data_->leg_controller->commands_[ i ].q_des = jump_ctrl_->GetJointPosDesire( i );
                this->data_->leg_controller->commands_[ i ].qd_des.setZero();
                this->data_->leg_controller->commands_[ i ].kp_joint = kp_land_.asDiagonal();
                this->data_->leg_controller->commands_[ i ].kd_joint = kd_land_.asDiagonal();
            }
        }
        else {
            // contact_state_est.setConstant( 0.5 ); // touch down
            // this->data_->state_estimator->SetContactPhase( contact_state_est );
            landing_count_++;
            for ( int i = 0; i < 4; i++ ) {
                if ( landing_count_ >= landing_delay_ ) {
                    this->data_->leg_controller->commands_[ i ].q_des = this->data_->leg_controller->commands_[ i ].q_des * landing_filter_ + joint_pos_end_pos_[ i ] * ( 1.0 - landing_filter_ );
                }
                else {
                    this->data_->leg_controller->commands_[ i ].q_des = jump_ctrl_->GetJointPosDesire( i );
                }
                this->data_->leg_controller->commands_[ i ].qd_des.setZero();
                kp_trans_                                            = landing_filter_ * kp_land_ + ( 1.0 - landing_filter_ ) * kp_end_;
                kd_trans_                                            = landing_filter_ * kd_land_ + ( 1.0 - landing_filter_ ) * kd_end_;
                this->data_->leg_controller->commands_[ i ].kp_joint = kp_trans_.asDiagonal();
                this->data_->leg_controller->commands_[ i ].kd_joint = kd_trans_.asDiagonal();
            }
        }
    }

    if ( jump_ctrl_->EndOfPhase( this->data_->leg_controller->datas_ ) ) {
        jump_ctrl_->LastVisit();
    }

    // Set motion_progress_bar_ for lcm
    if ( this->data_->state_estimator->GetResult().position( 2 ) > height_min_thresh || !use_height_good_ ) {
        height_good_for_trans_ = true;
    }
    else {
        height_good_for_trans_ = false;
    }
    if ( data_end_ && touch_down_ && height_good_for_trans_ ) {
        touch_down_delay_++;
        if ( touch_down_delay_ > touhc_down_delay_thresh_ ) {
            this->motion_progress_bar_ = 100;
        }
    }
    else {
        this->motion_progress_bar_ = 0;
        touch_down_delay_          = 0;
    }

    // // DEBUG
    // std::cout << "contact_state_est: " << contact_state_est.transpose() << std::endl;
}

/**
 * @brief Reset joint commnand in case of error
 *
 */
template < typename T > void FsmStateJump3d< T >::SafeCommand() {
    for ( int leg = 0; leg < 4; ++leg ) {
        for ( int jidx = 0; jidx < 3; ++jidx ) {
            this->data_->leg_controller->commands_[ leg ].tau_feed_forward[ jidx ] = 0.;
            this->data_->leg_controller->commands_[ leg ].q_des[ jidx ]            = this->data_->leg_controller->datas_[ leg ].q[ jidx ];
            this->data_->leg_controller->commands_[ leg ].qd_des[ jidx ]           = 0.;
        }
    }
}

/**
 * @brief Detect touch down
 *
 * @return true if robot touches down
 * @return false if robot doesn't touch down
 */
template < typename T > void FsmStateJump3d< T >::TounchDownDetection() {
    static int leg_count;
    leg_count = 0;

    // Trigger flying and touch down
    if ( offline_opt_data_->contact_state[ 0 ] < 1e-2 && offline_opt_data_->contact_state[ 1 ] < 1e-2 && offline_opt_data_->contact_state[ 2 ] < 1e-2
         && offline_opt_data_->contact_state[ 3 ] < 1e-2 ) {
        flying_phase_ = true;
    }
    // touch_down_ only triggers in fly phase
    if ( flying_phase_ ) {
        for ( int i = 0; i < 4; i++ ) {
            if ( this->data_->leg_controller->datas_[ i ].foot_force_actual( 2 ) < -touch_down_threh_ ) {
                leg_count++;
            }
        }
        // At least 3 feet satisfy torque/force condition
        if ( leg_count >= 2 && !touch_down_ ) {
            touch_down_count_++;
        }
        else {
            touch_down_count_ = 0;
        }

        if ( touch_down_count_ >= 100 ) {
            if ( !touch_down_ ) {
                std::cout << "[FsmStateJump3d] Touch down at " << jump_ctrl_->GetCurrentIteration() << std::endl;
            }
            touch_down_ = true;
        }
    }

    if ( touch_down_ ) {
        flying_phase_ = false;
    }
}

/**
 * @brief Convert offline data to wbc data
 *
 */
template < typename T > void FsmStateJump3d< T >::ConvertData() {
    wbc_data_->body_pos_des = offline_opt_data_->body_pos_des;
    wbc_data_->body_vel_des = offline_opt_data_->body_vel_des;
    wbc_data_->body_acc_des = offline_opt_data_->body_acc_des;
    wbc_data_->body_rpy_des = offline_opt_data_->body_rpy_des;
    wbc_data_->body_omg_des = offline_opt_data_->body_omg_des;

    for ( int i = 0; i < 4; i++ ) {
        wbc_data_->foot_pos_des[ i ]       = offline_opt_data_->foot_pos_des[ i ];
        wbc_data_->foot_vel_des[ i ]       = offline_opt_data_->foot_vel_des[ i ];
        wbc_data_->foot_acc_des[ i ]       = offline_opt_data_->foot_acc_des[ i ];
        wbc_data_->reaction_force_des[ i ] = offline_opt_data_->reaction_force_des[ i ];
    }

    wbc_data_->contact_state = offline_opt_data_->contact_state;
}

/**
 * @brief Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template < typename T > FsmStateName FsmStateJump3d< T >::CheckTransition() {
    this->next_state_name_ = this->state_name_;
    auto cmd               = this->data_->command;

    // Switch FSM control mode
    switch ( cmd->mode ) {
    case MotionMode::kJump3d:
        break;
    // case MotionMode::kQpStand:
    case MotionMode::kRecoveryStand:
    case MotionMode::kRlReset:
    case MotionMode::kPoseCtrl:
    case MotionMode::kPureDamper:
    // case MotionMode::kLocomotion:
    case MotionMode::kOff:  // normal c
        this->next_state_name_ = ( FsmStateName )cmd->mode;
        break;

    default:
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << ( int )MotionMode::kJump3d << " to " << ( int )this->data_->command->mode << std::endl;
    }

    return this->next_state_name_;
}

/**
 * @brief Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template < typename T > TransitionData< T > FsmStateJump3d< T >::Transition() {
    // Finish Transition
    switch ( this->next_state_name_ ) {
    case FsmStateName::kOff:  // normal
        this->transition_data_.done = true;
        break;

    // case FsmStateName::kQpStand:
    case FsmStateName::kRecoveryStand:
    case FsmStateName::kRlReset:
    case FsmStateName::kPoseCtrl:
    case FsmStateName::kPureDamper:
        // case FsmStateName::kLocomotion:
        if ( data_end_ && touch_down_ && height_good_for_trans_ && touch_down_delay_ >= touhc_down_delay_thresh_ + 10 ) {
            this->transition_data_.done = true;
        }
        ComputeCommand();
        break;
    default:
        std::cout << "[CONTROL FSM] Something went wrong in transition" << std::endl;
    }

    // Return the transition data to the FSM
    return this->transition_data_;
}

/**
 * @brief Cleans up the state information on exiting the state.
 *
 */
template < typename T > void FsmStateJump3d< T >::OnExit() {
    // nothing to clean up
}

template class FsmStateJump3d< float >;
