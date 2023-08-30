#include "fsm_states/fsm_state_lifted.hpp"

/**
 * @brief Construct a new Fsm State Lifted<  T >:: Fsm State Lifted object
 *
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 *
 * @param control_fsm_data holds all of the relevant control data
 */
template < typename T > FsmStateLifted< T >::FsmStateLifted( ControlFsmData< T >* control_fsm_data ) : FsmState< T >( control_fsm_data, FsmStateName::kLifted, "lifted" ) {
    // Do nothing
    // Set the pre controls safety checks
    this->check_safe_orientation_ = false;
    this->check_robot_lifted_     = false;

    // Post control safety checks
    this->check_desired_foot_position_ = false;
    this->check_feed_forward_force_    = false;

    iter_         = 0;
    landing_flag_ = false;
    joint_pos_lifted_ << 0.0, -0.9046f, 1.5097f;
    tau_trigger_landing_ << 0.15f, 0.35f, 0.75f;
    for ( size_t leg( 0 ); leg < 4; ++leg ) {
        tau_lift_avg_[ leg ].setZero();
    }
}

/**
 * @brief Behavior to be carried out when entering a state.
 *
 */
template < typename T > void FsmStateLifted< T >::OnEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    // Reset the transition data
    this->transition_data_.ResetTransitionDone();

    this->ready_for_switch_ = false;

    iter_         = 0;
    iter_falling_ = 0;
    iter_landing_ = 0;

    // this->data_->robot_current_state->gait_id = 0;
}

/**
 * @brief Calls the functions to be executed on each control loop iteration.
 *
 */
template < typename T > void FsmStateLifted< T >::Run() {

    this->data_->leg_controller->ZeroCommand();
    this->data_->leg_controller->legs_enabled_ = true;

    for ( size_t leg( 0 ); leg < 4; ++leg ) {
        this->data_->leg_controller->commands_[ leg ].tau_feed_forward.setZero();
        this->data_->leg_controller->commands_[ leg ].q_des = joint_pos_lifted_;
        this->data_->leg_controller->commands_[ leg ].kp_joint << 35.0, 0, 0, 0, 35.0, 0, 0, 0, 35.0;
        this->data_->leg_controller->commands_[ leg ].qd_des.setZero();
        this->data_->leg_controller->commands_[ leg ].kd_joint << 2.5, 0, 0, 0, 2.5, 0, 0, 0, 2.5;
    }
    if ( iter_ < duration_for_tau_avg_ ) {
        iter_++;
        for ( size_t leg( 0 ); leg < 4; ++leg ) {
            tau_lift_avg_[ leg ] = tau_lift_avg_[ leg ] * ( iter_ >= 1 ? iter_ - 1 : 0 ) + this->data_->leg_controller->datas_[ leg ].tau_actual;
            tau_lift_avg_[ leg ] = tau_lift_avg_[ leg ] / ( float )( iter_ >= 1 ? iter_ : 1 );
        }
    }
    else {  // waiting for triggering landing event
        if ( fabs( ( this->data_->state_estimator->GetResult() ).rpy( 0 ) ) > falling_threshold_ || fabs( ( this->data_->state_estimator->GetResult() ).rpy( 1 ) ) > falling_threshold_ ) {
            // falling down
            iter_landing_                             = 0;
            landing_flag_                             = false;
            this->data_->robot_current_state->gait_id = 1;

            iter_falling_++;
            if ( iter_falling_ >= duration_for_falling_ ) {
                falling_flag_           = true;
                this->ready_for_switch_ = true;
            }
            else {
                falling_flag_           = false;
                this->ready_for_switch_ = false;
            }
        }
        else {
            // landing
            iter_falling_                             = 0;
            falling_flag_                             = false;
            this->data_->robot_current_state->gait_id = 0;

            for ( size_t leg( 0 ); leg < 4; ++leg ) {
                if ( fabs( this->data_->leg_controller->datas_[ leg ].tau_actual[ 2 ] - tau_lift_avg_[ leg ][ 2 ] ) >= tau_trigger_landing_[ 2 ]
                     || fabs( this->data_->leg_controller->datas_[ leg ].tau_actual[ 2 ] ) >= tau_trigger_landing_[ 2 ] )  // only knee is used
                {
                    landing_flag_ = true;
                }
                else {
                    landing_flag_ = false;
                }
            }
            if ( landing_flag_ ) {
                iter_landing_++;
            }
            else {
                iter_landing_ = 0;
            }

            if ( iter_landing_ >= duration_for_landing_ ) {
                this->ready_for_switch_ = true;
            }
            else {
                this->ready_for_switch_ = false;
            }
        }
    }

    // // DEBUG
    // for ( int i = 0; i < 4; i++ ) {
    //     std::cout << "taul_lift_avg[" << i << "]: " << tau_lift_avg_[i].transpose() << std::endl;
    //     std::cout << "fabs[" << i << "]:" << fabs( this->data_->leg_controller->datas_[ i ].tau_actual[ 2 ] - tau_lift_avg_[ i ][ 2 ] ) << std::endl;
    // }
}

/**
 * @brief  Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template < typename T > FsmStateName FsmStateLifted< T >::CheckTransition() {
    this->next_state_name_ = this->state_name_;
    auto cmd               = this->data_->command;

    // Switch FSM control mode
    switch ( cmd->mode ) {
    case MotionMode::kLifted:
        break;
    case MotionMode::kPureDamper:
    case MotionMode::kRecoveryStand:
    case MotionMode::kOff:
        this->next_state_name_ = ( FsmStateName )cmd->mode;
        break;

    case MotionMode::kLocomotion:
    case MotionMode::kQpStand:
        if ( this->ready_for_switch_ && landing_flag_ ) {
            this->next_state_name_ = ( FsmStateName )cmd->mode;
        }
        break;

    case MotionMode::kRlReset:
        if ( this->ready_for_switch_ && falling_flag_ ) {
            this->next_state_name_ = ( FsmStateName )cmd->mode;
        }
        break;

    default:
        if ( this->iter_printf_ >= this->iter_printf_reset_ ) {
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << ( int )MotionMode::kLifted << " to " << ( int )this->data_->command->mode << std::endl;
            this->iter_printf_ = 0;
        }
        this->iter_printf_++;
    }

    return this->next_state_name_;
}

/**
 * @brief Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template < typename T > TransitionData< T > FsmStateLifted< T >::Transition() {

    this->transition_data_.done = false;
    // Finish Transition
    switch ( this->next_state_name_ ) {

    case FsmStateName::kPureDamper:
    case FsmStateName::kOff:  // normal
    case FsmStateName::kRecoveryStand:
        this->transition_data_.done = true;
        break;

    case FsmStateName::kLocomotion:
    case FsmStateName::kQpStand:
        if ( this->ready_for_switch_ && landing_flag_ ) {
            this->transition_data_.done = true;
        }
        break;

    case FsmStateName::kRlReset:
        if ( this->ready_for_switch_ && falling_flag_ ) {
            this->transition_data_.done = true;
        }
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
template < typename T > void FsmStateLifted< T >::OnExit() {
    // Nothing to clean up when exiting
}

// template class FSMStateRecoveryStand<double>;
template class FsmStateLifted< float >;
