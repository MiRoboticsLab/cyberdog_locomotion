/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "fsm_states/fsm_state_forcejump.hpp"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param control_fsm_data holds all of the relevant control data
 */
template < typename T > FsmStateForceJump< T >::FsmStateForceJump( ControlFsmData< T >* control_fsm_data ) : FsmState< T >( control_fsm_data, FsmStateName::kForceJump, "force_jump" ) {

    lsm_ = new BalanceControllerLSM( control_fsm_data );

    // Set the pre controls safety checks
    this->check_safe_orientation_ = false;
    this->check_robot_lifted_     = false;

    // Post control safety checks
    this->check_desired_foot_position_         = false;
    this->check_feed_forward_force_ = false;
}

template < typename T > void FsmStateForceJump< T >::OnEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    // Reset the transition data
    this->transition_data_.ResetTransitionDone();

    this->ready_for_switch_ = false;

    this->motion_progress_bar_ = 0;

    jump_first_run_ = true;
    public_iter_  = 0;

    if( this->data_->command->gait_id == 1 || this->data_->command->gait_id == 4 )
        this->data_->robot_current_state->gait_id = this->data_->command->gait_id;
    else
        this->data_->robot_current_state->gait_id = 0;

}

template < typename T > void FsmStateForceJump< T >::JumpStep() {

    float t = 0;
    if ( jump_first_run_ ) {
        for ( int leg( 0 ); leg < 4; leg++ )
            public_q_cmd_[ leg ] = this->data_->leg_controller->datas_[ leg ].q;
        jump_contact_ = jump_foot_support_;
        jump_isjump_  = true;
        jump_first_run_ = false;
        public_iter_  = 0;
    }

    for ( int leg( 0 ); leg < 4; leg++ ) {
        //有待增加蹬空判断/打滑判断/触底判断
        if ( this->data_->leg_controller->datas_[ leg ].q[ 2 ] <= 35 / 57.3 )
            jump_contact_ << -1.f, -1.f, -1.f, -1.f;
    }

    if ( jump_x_acc_cmd_[ 2 ] >= 15 )
        jump_time_ = 200;
    else
        jump_time_ = 60;

    if ( public_iter_ > jump_time_ || public_iter_ > 200 )
        jump_contact_ << -1.f, -1.f, -1.f, -1.f;

    if ( ( jump_contact_( 0 ) < 0 ) && ( jump_contact_( 1 ) < 0 ) && ( jump_contact_( 2 ) < 0 ) && ( jump_contact_( 3 ) < 0 ) ) {
        jump_isjump_ = false;
    }

    if ( jump_isjump_ ) {
        lsm_->SolverLSM( this->data_, jump_x_acc_cmd_, jump_w_acc_cmd_, jump_contact_ );
        public_iter_jump_ = public_iter_;
    }

    if ( ( jump_x_acc_cmd_[ 0 ] >= 8 && jump_x_acc_cmd_[ 2 ] >= 15 ) ) {
        Vec12< T > leg_point_adjust;
        leg_point_adjust << -10 / 57.3, 20 / 57.3, -10 / 57.3, 10 / 57.3, 20 / 57.3, -10 / 57.3, -10 / 57.3, 20 / 57.3, -10 / 57.3, 10 / 57.3, 20 / 57.3, -10 / 57.3;
        if ( public_iter_ > public_iter_jump_ && public_iter_ < public_iter_jump_ + 50 ) {
            if ( public_iter_ == public_iter_jump_ + 1 ) {
                for ( int leg( 0 ); leg < 4; leg++ ) {
                    leg_angle_goal12_.segment( leg * 3, 3 ) << 0 / 57.3, -150 / 57.3, 140 / 57.3;  // 0/57.3,=public_q_cmd_[leg];
                    leg_angle_init12_.segment( leg * 3, 3 ) = this->data_->leg_controller->datas_[ leg ].q;
                }
            }
            t               = ( public_iter_ - public_iter_jump_ ) * 0.002 + 0.03;
            leg_angle_des12_ = leg_angle_init12_ + ( leg_angle_goal12_ - leg_angle_init12_ ) * ( t > 0.15 ? 1 : t / 0.15 );
        }
        else if ( public_iter_ >= public_iter_jump_ + 50 ) {
            if ( public_iter_ == public_iter_jump_ + 50 ) {
                for ( int leg( 0 ); leg < 4; leg++ ) {
                    public_q_cmd_[ leg ] += leg_point_adjust.segment( leg * 3, 3 );
                    leg_angle_goal12_.segment( leg * 3, 3 ) = public_q_cmd_[ leg ];
                    leg_angle_init12_.segment( leg * 3, 3 ) = this->data_->leg_controller->datas_[ leg ].q;
                }
            }
            t               = ( public_iter_ - public_iter_jump_ - 50 ) * 0.002 + 0.04;
            leg_angle_des12_ = leg_angle_init12_ + ( leg_angle_goal12_ - leg_angle_init12_ ) * ( t > 0.1 ? 1 : t / 0.1 );
        }
    }
    else {
        if ( public_iter_ > public_iter_jump_ ) {
            if ( public_iter_ == public_iter_jump_ + 1 ) {
                for ( int leg( 0 ); leg < 4; leg++ ) {
                    leg_angle_goal12_.segment( leg * 3, 3 ) = public_q_cmd_[ leg ];
                    leg_angle_init12_.segment( leg * 3, 3 ) = this->data_->leg_controller->datas_[ leg ].q;
                }
            }
            t               = ( public_iter_ - public_iter_jump_ ) * 0.002 + 0.04;
            leg_angle_des12_ = leg_angle_init12_ + ( leg_angle_goal12_ - leg_angle_init12_ ) * ( t > 0.2 ? 1 : t / 0.2 );
        }
    }
    // std::cout<<"public_iter=" <<public_iter_ << " "<<jump_contact_[0]<< " "<<jump_contact_[1]<< " "<<jump_contact_[2]<< " "<<jump_contact_[3]<<std::endl;

    for ( int leg( 0 ); leg < 4; leg++ ) {
        if ( jump_contact_( leg ) < 0 ) {
            this->data_->leg_controller->commands_[ leg ].q_des = leg_angle_des12_.segment( leg * 3, 3 );
            this->data_->leg_controller->commands_[ leg ].force_feed_forward.setZero();
            this->data_->leg_controller->commands_[ leg ].qd_des.setZero();
            this->data_->leg_controller->commands_[ leg ].kp_joint << 80, 0, 0, 0, 30, 0, 0, 0, 40;
            this->data_->leg_controller->commands_[ leg ].kd_joint << 5, 0, 0, 0, 5, 0, 0, 0, 5;
        }
    }
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template < typename T > void FsmStateForceJump< T >::Run() {
    int   k;
    auto& cmd = this->data_->command;
    public_iter_++;

    this->data_->leg_controller->ZeroCommand();
    this->data_->leg_controller->legs_enabled_ = true;

    if ( cmd->gait_id == 4 || 1 ) {
        this->data_->robot_current_state->gait_id = this->data_->command->gait_id;
        for ( k = 0; k < 3; k++ ) {
            jump_w_acc_cmd_( k ) = cmd->acc_des[ k ];
            jump_x_acc_cmd_( k ) = cmd->acc_des[ k + 3 ];
        }
        for ( k = 0; k < 4; k++ ) {
            jump_foot_support_( k ) = ( cmd->contact >> k ) & 0x01;
        }
    }
    else
        this->data_->robot_current_state->gait_id = 0;
    JumpStep();
}
template < typename T > int FsmStateForceJump< T >::SaftyCheck() {
    return 1;
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template < typename T > FsmStateName FsmStateForceJump< T >::CheckTransition() {
    this->next_state_name_ = this->state_name_;
    auto& cmd           = this->data_->command;

    // Switch FSM control mode
    switch ( cmd->mode ) {
    case MotionMode::kForceJump:
        break;

    case MotionMode::kPoseCtrl:
    case MotionMode::kPureDamper:
    case MotionMode::kQpStand:
    case MotionMode::kRecoveryStand:
    case MotionMode::kRlReset:
    case MotionMode::kOff:  // normal c
    case MotionMode::kLocomotion:
    case MotionMode::kTwoLegStand:
        this->next_state_name_ = ( FsmStateName )cmd->mode;
        break;

    default:
        if ( this->iter_printf_ >= this->iter_printf_reset_ ) {
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << ( int )MotionMode::kForceJump << " to " << ( int )this->data_->command->mode << std::endl;
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
template < typename T > TransitionData< T > FsmStateForceJump< T >::Transition() {
    // Finish Transition
    switch ( this->next_state_name_ ) {
    case FsmStateName::kOff:  // normal
    case FsmStateName::kQpStand:
    case FsmStateName::kLocomotion:
    case FsmStateName::kPureDamper:
    case FsmStateName::kRecoveryStand:
    case FsmStateName::kRlReset:
    case FsmStateName::kTwoLegStand:
    case FsmStateName::kJump3d:
    case FsmStateName::kPoseCtrl:
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
template < typename T > void FsmStateForceJump< T >::OnExit() {
    // Nothing to clean up when exiting
}

// template class FsmStateForceJump<double>;
template class FsmStateForceJump< float >;
