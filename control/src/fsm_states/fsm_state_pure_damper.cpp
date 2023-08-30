#include "fsm_states/fsm_state_pure_damper.hpp"

/**
 * @brief Constructor for the FSM State that passes in state specific info to
 *        the generic FSM State constructor.
 *
 * @param control_fsm_data holds all of the relevant control data
 */
template < typename T > FsmStatePureDamper< T >::FsmStatePureDamper( ControlFsmData< T >* control_fsm_data ) : FsmState< T >( control_fsm_data, FsmStateName::kPureDamper, "pure_damper" ) {
    // Set the pre controls safety checks
    this->check_safe_orientation_ = false;
    this->check_robot_lifted_     = false;

    // Post control safety checks
    this->check_desired_foot_position_ = false;
    this->check_feed_forward_force_    = false;

    zero_vec3_.setZero();
}

/**
 * @brief Behavior to be carried out when entering a state.
 *
 */
template < typename T > void FsmStatePureDamper< T >::OnEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    // Reset the transition data
    this->transition_data_.ResetTransitionDone();

    this->ready_for_switch_    = false;
    this->motion_progress_bar_ = 0;

    iter_ = 0;

    // Initial configuration, position
    for ( int i( 0 ); i < 4; ++i ) {
        Vec3< T > data_q = this->data_->leg_controller->datas_[ i ].q;
        Vec3< T > cmd_q  = this->data_->leg_controller->commands_[ i ].q_des;
        for ( int j( 0 ); j < 3; ++j ) {
            if ( fabs( data_q( j ) - cmd_q( j ) ) > 4 / 57.3 ) {
                initial_jpos_[ i ]( j ) = data_q( j );
            }
            else {
                initial_jpos_[ i ]( j ) = cmd_q( j );
            }
        }
    }
    target_jpos_[ 0 ] << -0.08, -1.11, 2.57;
    target_jpos_[ 1 ] << 0.08, -1.11, 2.57;
    target_jpos_[ 2 ] << -0.20, -1.11, 2.57;
    target_jpos_[ 3 ] << 0.20, -1.11, 2.57;

    if ( this->data_->command->gait_id == 1 ) {
        this->data_->robot_current_state->gait_id = 1;
    }
    else {
        this->data_->robot_current_state->gait_id = 0;
    }
}

/**
 * @brief Calls the functions to be executed on each control loop iteration.
 *
 */
template < typename T > void FsmStatePureDamper< T >::Run() {
    this->data_->leg_controller->ZeroCommand();

    if ( this->data_->robot_current_state->gait_id == 1 && this->data_->command->mode == MotionMode::kPureDamper ) {
        float time1 = 600.0;  // 600 / 500 = 1.2 s
        float time2 = 400.0;  // 400 / 500 = 0.8 s

        float ratio = ( iter_ / time1 ) > 1 ? 1 : ( iter_ / time1 );  // ratio: (0, 1)

        for ( int leg( 0 ); leg < 4; ++leg ) {
            // 0 ~ 1.2 s
            if ( iter_ < time1 ) {
                this->data_->leg_controller->legs_enabled_         = true;
                this->data_->leg_controller->commands_[ leg ].q_des = initial_jpos_[ leg ] + ( target_jpos_[ leg ] - initial_jpos_[ leg ] ) * ratio;
                this->data_->leg_controller->commands_[ leg ].kp_joint << 60, 0, 0, 0, 80, 0, 0, 0, 60;
                this->data_->leg_controller->commands_[ leg ].kd_joint << 2.0, 0, 0, 0, 2, 0, 0, 0, 1.5;
            }
            // 1.2 ~ 2.0 s
            else if ( iter_ < time1 + time2 ) {
                this->data_->leg_controller->legs_enabled_ = true;
                this->data_->leg_controller->commands_[ leg ].kd_joint << 2.0, 0, 0, 0, 2, 0, 0, 0, 1.5;
            } else {
                this->ready_for_switch_    = true;
            }

            // motion_progress_bar_: (0, 100)
            this->motion_progress_bar_ = ( int )( iter_ > time1 ? 100 : 100 * iter_ / time1 );
        }
    }
    else {
        if ( iter_ < pd_iter_ ) {
            this->data_->leg_controller->legs_enabled_ = true;

            for ( int leg( 0 ); leg < 4; ++leg ) {
                this->data_->leg_controller->commands_[ leg ].qd_des = zero_vec3_;
                this->data_->leg_controller->commands_[ leg ].kd_joint << 1.75 * ( iter_ < pd_iter_ * 0.8 ), 0, 0, 0, 3.5, 0, 0, 0, 3.5;
            }

            this->motion_progress_bar_ = ( int )( iter_ / ( float )pd_iter_ * 100 );

            if ( iter_ > 2000 ) {
                this->motion_progress_bar_ = 100;
            }
        }
        else {
            this->ready_for_switch_    = true;
            this->motion_progress_bar_ = 100;
        }
    }

    iter_++;
}

/**
 * @brief Manages which states can be transitioned into either by the user
 *        commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template < typename T > FsmStateName FsmStatePureDamper< T >::CheckTransition() {
    this->next_state_name_ = this->state_name_;
    auto cmd              = this->data_->command;

    // Switch FSM control mode
    switch ( cmd->mode ) {
    case MotionMode::kPureDamper:
        break;
    case MotionMode::kMotorCtrl:
    case MotionMode::kRecoveryStand:
    case MotionMode::kOff:  // normal c
    case MotionMode::kRlReset:
        this->next_state_name_ = ( FsmStateName )cmd->mode;
        break;

    default:
        if ( this->iter_printf_ >= this->iter_printf_reset_ ) {
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << ( int )MotionMode::kPureDamper << " to " << ( int )this->data_->command->mode << std::endl;
            this->iter_printf_ = 0;
        }
        this->iter_printf_++;
    }

    return this->next_state_name_;
}

/**
 * @brief Handles the actual transition for the robot between states.
 *        Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template < typename T > TransitionData< T > FsmStatePureDamper< T >::Transition() {
    // Finish Transition
    switch ( this->next_state_name_ ) {

    case FsmStateName::kOff:  // normal
    case FsmStateName::kRecoveryStand:
    case FsmStateName::kRlReset:
    case FsmStateName::kMotorCtrl:
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
template < typename T > void FsmStatePureDamper< T >::OnExit() {
    // Nothing to clean up when exiting
}

template class FsmStatePureDamper< float >;
