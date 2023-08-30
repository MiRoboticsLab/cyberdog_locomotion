/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "fsm_states/fsm_state_posectrl.hpp"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param control_fsm_data holds all of the relevant control data
 */
template < typename T > FsmStatePoseCtrl< T >::FsmStatePoseCtrl( ControlFsmData< T >* control_fsm_data ) : FsmState< T >( control_fsm_data, FsmStateName::kPoseCtrl, "pose_ctrl" ) {

    p_ctrl_ = new PositionController;

    // Set the pre controls safety checks
    this->check_safe_orientation_ = false;
    this->check_robot_lifted_     = false;

    // Post control safety checks
    this->check_desired_foot_position_ = false;
    this->check_feed_forward_force_    = false;
}

template < typename T > void FsmStatePoseCtrl< T >::OnEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    // Reset the transition data
    this->transition_data_.ResetTransitionDone();

    this->ready_for_switch_ = false;

    this->motion_progress_bar_ = 0;

    for ( int leg( 0 ); leg < 4; leg++ ) {
        Vec3< T > data_q     = this->data_->leg_controller->datas_[ leg ].q;
        Vec3< T > cmd_q      = this->data_->leg_controller->commands_[ leg ].q_des;
        public_q_cmd_[ leg ] = data_q;
        for ( int j( 0 ); j < 3; ++j ) {
            if ( fabs( data_q[ j ] - cmd_q[ j ] ) > 8 / 57.3 )
                public_q_cmd_[ leg ][ j ] = data_q[ j ];
            else
                public_q_cmd_[ leg ][ j ] = cmd_q[ j ];
        }
    }
    poseFirstRun_  = true;
    transFirshRun_ = true;
    trans_iter_    = 0;
    dampSwingLeg_  = false;

    this->data_->robot_current_state->gait_id = this->data_->command->gait_id;
}

template < typename T > void FsmStatePoseCtrl< T >::PoseStep( const bool SwinglegFlag ) {
    static float delta = 0.06;
    for ( int leg( 0 ); leg < 4; leg++ ) {
        if ( p_ctrl_->joint_p_cmd_[ leg ]( 0 ) < ( this->data_->leg_controller->q_abad_lowerbound_ + delta ) )
            p_ctrl_->joint_p_cmd_[ leg ]( 0 ) = this->data_->leg_controller->q_abad_lowerbound_ + delta;
        if ( p_ctrl_->joint_p_cmd_[ leg ]( 0 ) > ( this->data_->leg_controller->q_abad_upperbound_ - delta ) )
            p_ctrl_->joint_p_cmd_[ leg ]( 0 ) = this->data_->leg_controller->q_abad_upperbound_ - delta;

        if ( p_ctrl_->joint_p_cmd_[ leg ]( 2 ) < ( this->data_->leg_controller->q_knee_lowerbound_ + delta ) )
            p_ctrl_->joint_p_cmd_[ leg ]( 2 ) = this->data_->leg_controller->q_knee_lowerbound_ + delta;
        if ( p_ctrl_->joint_p_cmd_[ leg ]( 2 ) > ( this->data_->leg_controller->q_knee_upperbound_ - delta ) )
            p_ctrl_->joint_p_cmd_[ leg ]( 2 ) = this->data_->leg_controller->q_knee_upperbound_ - delta;

        if ( leg == 0 || leg == 1 ) {
            if ( p_ctrl_->joint_p_cmd_[ leg ]( 1 ) < ( this->data_->leg_controller->q_fronthip_lowerbound_ + delta ) )
                p_ctrl_->joint_p_cmd_[ leg ]( 1 ) = this->data_->leg_controller->q_fronthip_lowerbound_ + delta;
            if ( p_ctrl_->joint_p_cmd_[ leg ]( 1 ) > ( this->data_->leg_controller->q_fronthip_upperbound_ - delta ) )
                p_ctrl_->joint_p_cmd_[ leg ]( 1 ) = this->data_->leg_controller->q_fronthip_upperbound_ - delta;
        }
        else {
            if ( p_ctrl_->joint_p_cmd_[ leg ]( 1 ) < ( this->data_->leg_controller->q_rearhip_lowerbound_ + delta ) )
                p_ctrl_->joint_p_cmd_[ leg ]( 1 ) = this->data_->leg_controller->q_rearhip_lowerbound_ + delta;
            if ( p_ctrl_->joint_p_cmd_[ leg ]( 1 ) > ( this->data_->leg_controller->q_rearhip_upperbound_ - delta ) )
                p_ctrl_->joint_p_cmd_[ leg ]( 1 ) = this->data_->leg_controller->q_rearhip_upperbound_ - delta;
        }
        for ( int i = 0; i < 3; i++ )
            if ( std::isnan( p_ctrl_->joint_p_cmd_[ leg ]( i ) ) )
                p_ctrl_->joint_p_cmd_[ leg ]( i ) = this->data_->leg_controller->datas_[ leg ].q( i );

        this->data_->leg_controller->commands_[ leg ].q_des = p_ctrl_->joint_p_cmd_[ leg ];
        public_q_cmd_[ leg ]                                 = p_ctrl_->joint_p_cmd_[ leg ];
        this->data_->leg_controller->commands_[ leg ].qd_des.setZero();
        if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG ) {
            this->data_->leg_controller->commands_[ leg ].kp_joint << 100, 0, 0, 0, 100, 0, 0, 0, 120;
            this->data_->leg_controller->commands_[ leg ].kd_joint << 3, 0, 0, 0, 3, 0, 0, 0, 3;
        }
        else {
            this->data_->leg_controller->commands_[ leg ].kp_joint << 40, 0, 0, 0, 30, 0, 0, 0, 30;
            this->data_->leg_controller->commands_[ leg ].kd_joint << 2.0, 0, 0, 0, 1.5, 0, 0, 0, 1.5;
        }
    }
    if ( SwinglegFlag ) {
        // constant :swing leg 0
        int swingleg = 0;
        for ( int i( 0 ); i < 4; i++ ) {
            if ( pose_foot_support_( i ) < 0.5 )
                swingleg = i;
        }
        this->data_->leg_controller->commands_[ swingleg ].kp_joint << 40, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5;
        this->data_->leg_controller->commands_[ swingleg ].kd_joint << 2.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05;
    }
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template < typename T > void FsmStatePoseCtrl< T >::Run() {
    int   k;
    auto& cmd      = this->data_->command;
    int   contact  = cmd->contact;
    float duration = ( float )( cmd->duration );
    this->data_->leg_controller->ZeroCommand();
    this->data_->leg_controller->legs_enabled_ = true;
    this->data_->robot_current_state->gait_id     = cmd->gait_id;

    if ( cmd->contact == 0 )
        contact = 0x0F;
    if ( cmd->duration == 0 )
        duration = 1800000;
    this->motion_progress_bar_ = trans_iter_ < duration ? ( int32_t )( trans_iter_ / duration * 100 ) : 100;

    if ( cmd->gait_id != 5 ) {  // pose increase
        if ( poseFirstRun_ ) {
            p_ctrl_->InitState( this->data_, public_q_cmd_ );
            poseFirstRun_ = false;
        }
        if ( trans_iter_ > duration )
            trans_iter_ = 0;
        for ( k = 0; k < 3; k++ ) {
            pose_body_cmd_( k )     = cmd->rpy_des[ k ] / duration;
            pose_body_cmd_( k + 3 ) = cmd->pos_des[ k ] / duration;
            pose_foot_cmd_( k )     = cmd->foot_pose[ k ] / duration;
            pose_ctrl_point_( k )   = cmd->ctrl_point[ k ];
        }
        for ( k = 0; k < 4; k++ ) {
            pose_foot_support_( k ) = ( contact >> k ) & 0x01;
        }
        if ( contact != 0 )
            p_ctrl_->TransBody( pose_body_cmd_, pose_foot_cmd_, pose_ctrl_point_, pose_foot_support_ );
        transFirshRun_ = true;

        dampSwingLeg_ = false;
        if ( cmd->gait_id == 2 )
            dampSwingLeg_ = true;
    }
    else if ( cmd->gait_id == 5 ) {  // transform
        if ( height_ != cmd->pos_des[ 2 ] || ( trans_iter_ - duration > 50 ) ) {
            transFirshRun_ = true;
            trans_iter_    = 0;
        }
        if ( transFirshRun_ ) {
            height_ = cmd->pos_des[ 2 ];
            if ( height_ < 0.10 )
                height_ = 0.10;
            else if ( height_ > 0.30 )
                height_ = 0.30;
            T com_x  = 0.0;
            T L1     = this->data_->quadruped->hip_link_length_;
            T L2     = this->data_->quadruped->knee_link_length_;
            T D      = std::sqrt( height_ * height_ + com_x * com_x );
            T alpha2 = M_PI - std::acos( ( L1 * L1 + L2 * L2 - D * D ) / ( 2.0 * L1 * L2 ) );
            T alpha1 = std::atan( com_x / height_ ) - std::acos( ( L1 * L1 + D * D - L2 * L2 ) / ( 2.0 * L1 * D ) );
            for ( int leg( 0 ); leg < 4; leg++ ) {
                trans_end_q_[ leg ] << 0, alpha1, alpha2;
                trans_start_q_[ leg ] = public_q_cmd_[ leg ];
            }
            transFirshRun_ = false;
        }
        for ( int leg( 0 ); leg < 4; leg++ )
            p_ctrl_->joint_p_cmd_[ leg ] = trans_start_q_[ leg ] + ( trans_end_q_[ leg ] - trans_start_q_[ leg ] ) * ( trans_iter_ > duration ? 1 : ( trans_iter_ / duration ) );
        poseFirstRun_ = true;

        dampSwingLeg_ = false;
    }
    else {
        transFirshRun_ = true;
        poseFirstRun_  = true;
        trans_iter_    = 0;
    }
    trans_iter_++;
    PoseStep( dampSwingLeg_ );
}
template < typename T > int FsmStatePoseCtrl< T >::SaftyCheck() {
    return 1;
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template < typename T > FsmStateName FsmStatePoseCtrl< T >::CheckTransition() {
    this->next_state_name_ = this->state_name_;
    auto& cmd             = this->data_->command;

    // Switch FSM control mode
    switch ( cmd->mode ) {
    case MotionMode::kPoseCtrl:
        break;

    case MotionMode::kPureDamper:
    case MotionMode::kQpStand:
    case MotionMode::kRecoveryStand:
    case MotionMode::kOff:  // normal c
    case MotionMode::kLocomotion:
    case MotionMode::kTwoLegStand:
    case MotionMode::kJump3d:
    case MotionMode::kForceJump:
    case MotionMode::kRlReset:
        this->next_state_name_ = ( FsmStateName )cmd->mode;
        break;

    default:
        if ( this->iter_printf_ >= this->iter_printf_reset_ ) {
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << ( int )MotionMode::kPoseCtrl << " to " << ( int )this->data_->command->mode << std::endl;
            this->iter_printf_ = 0;
        }
        this->iter_printf_++;
    }

    // Get the next state
    return this->next_state_name_;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template < typename T > TransitionData< T > FsmStatePoseCtrl< T >::Transition() {
    // Finish Transition
    switch ( this->next_state_name_ ) {
    case FsmStateName::kOff:  // normal
    case FsmStateName::kQpStand:
    case FsmStateName::kLocomotion:
    case FsmStateName::kPureDamper:
    case FsmStateName::kRecoveryStand:
    case FsmStateName::kTwoLegStand:
    case FsmStateName::kJump3d:
    case FsmStateName::kForceJump:
    case FsmStateName::kRlReset:
        this->transition_data_.done = true;
        break;
    default:
        std::cout << "[CONTROL FSM] Something went wrong in transition" << std::endl;
    }

    // Return the transition data to the FSM
    return this->transition_data_;
}

/**
 * Cleans up the state information on exiting the state.
 */
template < typename T > void FsmStatePoseCtrl< T >::OnExit() {
    // Nothing to clean up when exiting
}

// template class FsmStatePoseCtrl<double>;
template class FsmStatePoseCtrl< float >;
