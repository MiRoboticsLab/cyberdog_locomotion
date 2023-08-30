#include "fsm_states/fsm_state_passive.hpp"

/**
 * @brief Constructor for the FSM State that passes in state specific info to
 *        the generic FSM State constructor.
 *
 * @param control_fsm_data holds all of the relevant control data
 */
template < typename T > FsmStatePassive< T >::FsmStatePassive( ControlFsmData< T >* control_fsm_data ) : FsmState< T >( control_fsm_data, FsmStateName::kOff, "passive" ) {
    // Set the pre controls safety checks
    this->check_safe_orientation_ = false;
    this->check_robot_lifted_     = false;

    // Post control safety checks
    this->check_desired_foot_position_ = false;
    this->check_feed_forward_force_    = false;
}

/**
 * @brief Behavior to be carried out when entering a state.
 *
 */
template < typename T > void FsmStatePassive< T >::OnEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    // Reset the transition data
    this->transition_data_.ResetTransitionDone();

    this->ready_for_switch_    = true;
    this->motion_progress_bar_ = 100;
    this->data_->leg_controller->ZeroCommand();
    this->data_->leg_controller->legs_enabled_ = true;
    this->data_->robot_current_state->gait_id   = 0;
}

/**
 * @brief Calls the functions to be executed on each control loop iteration.
 *
 */
template < typename T > void FsmStatePassive< T >::Run() {
    // Do nothing, all commands should begin as zeros
    this->data_->leg_controller->ZeroCommand();
    this->data_->leg_controller->legs_enabled_ = true;
    TestTransition();
}

/**
 * @brief Handles the actual transition for the robot between states.
 *        Returns true when the transition is completed.
 * 
 * @return true if transition is complete
 */
template < typename T > TransitionData< T > FsmStatePassive< T >::TestTransition() {
    this->transition_data_.done = true;
    return this->transition_data_;
}

/**
 * @brief Manages which states can be transitioned into either by the user
 *        commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template < typename T > FsmStateName FsmStatePassive< T >::CheckTransition() {
    this->next_state_name_ = this->state_name_;
    iter_++;
    auto cmd = this->data_->command;

    // Switch FSM control mode
    switch ( cmd->mode ) {
    case MotionMode::kOff:  // normal c (0)
        // Normal operation for state based transitions
        break;

    case MotionMode::kPureDamper:
    case MotionMode::kRecoveryStand:
    case MotionMode::kMotorCtrl:
    case MotionMode::kRlReset:
        this->next_state_name_ = ( FsmStateName )( cmd->mode );
        break;

    default:
        if ( this->iter_printf_ >= this->iter_printf_reset_ ) {
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << ( int )MotionMode::kOff << " to " << ( int )this->data_->command->mode << std::endl;
            this->iter_printf_ = 0;
        }
        this->iter_printf_++;
    }

    // Get the next state
    return this->next_state_name_;
}

/**
 * @brief Handles the actual transition for the robot between states.
 *        Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template < typename T > TransitionData< T > FsmStatePassive< T >::Transition() {
    switch ( this->next_state_name_ ) {
    case FsmStateName::kOff:
    case FsmStateName::kRecoveryStand:
    case FsmStateName::kPureDamper:
    case FsmStateName::kMotorCtrl:
    case FsmStateName::kRlReset:
        // Finish Transition
        this->transition_data_.done = true;
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
template < typename T > void FsmStatePassive< T >::OnExit() {
    // Nothing to clean up when exiting
}

template class FsmStatePassive< float >;
