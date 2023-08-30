#include "fsm_states/fsm_state_motor_ctrl.hpp"

#define Ctrl_Over_Time_War 10
#define Ctrl_Over_Time_Err 500

template < typename T >
FsmStateMotorCtrl< T >::FsmStateMotorCtrl( ControlFsmData< T >* control_fsm_data )
    : FsmState< T >( control_fsm_data, FsmStateName::kMotorCtrl, "motor_ctrl" ), motor_ctrl_state_lcm_( GetLcmUrlWithPort( 7667, 255 ) ) {
    this->TurnOnAllSafetyChecks();
    this->check_desired_foot_position_ = false;
    this->check_safe_orientation_      = false;
    this->check_robot_lifted_          = false;
    this->motion_progress_bar_         = 100;
    err_flag_                          = 0;
    for ( int j = 0; j < 12; j++ ) {
        ctrl_zero_.kp_des[ j ]  = 0;
        ctrl_zero_.kd_des[ j ]  = 0;
        ctrl_zero_.tau_des[ j ] = 0;
    }
}
template < typename T > void FsmStateMotorCtrl< T >::OnEnter() {
    this->next_state_name_ = this->state_name_;
    firstRun_              = true;
    err_flag_              = err_flag_ | 0x02; // Need reset error first for safty
    wait_reconnect_        = 0;
    std::cout << "[FSM MotorCtrl] On Enter" << std::endl;
    this->data_->robot_current_state->gait_id         = 0;
    this->data_->control_parameters->lcm_debug_switch = 1;
}

template < typename T > void FsmStateMotorCtrl< T >::Run() {
    float diff[ 3 ];
    bool  clear_flag     = false;
    int   software_limit = 1;
    float time_diff      = this->data_->command->cmd_time_delay;
    this->data_->leg_controller->ZeroCommand();
    this->data_->leg_controller->legs_enabled_ = true;

    ctrl_      = this->data_->command->motor_ctrl;
    clear_flag = CheckZero();
    for ( int i( 0 ); i < 4; i++ ) {
        if ( time_diff > Ctrl_Over_Time_War ) {
            float divi = time_diff / Ctrl_Over_Time_War;
            for ( int j = 0; j < 3; j++ )
                ctrl_.qd_des[ i * 3 + j ] /= divi;
            for ( int j = 0; j < 3; j++ )
                ctrl_.tau_des[ i * 3 + j ] /= divi;
            if ( !( err_flag_ & 0x1 ) )
                printf( "[FSM MotorCtrl] Warn:lcm control message lost over 10ms!!!\n" );
            err_flag_ |= 0x1;
            if ( time_diff > Ctrl_Over_Time_Err ) {
                if ( !( err_flag_ & 0x2 ) ) {
                    printf( "[FSM MotorCtrl] ERROR:lcm control message lost over 500ms!!! Enter high damping mode\n" );
                    err_flag_ |= 0x2;
                }
            }
        }
        else {
            if ( err_flag_ & 0x1 )
                printf( "[FSM MotorCtrl] Warn:LCM recive new motor ctrl_ request\n" );
            err_flag_ &= 0xFFFFFFFE;
        }
        if ( err_flag_ & 0x2 ) {  // Over 500ms
            wait_reconnect_++;
            ctrl_ = ctrl_zero_;
            if ( wait_reconnect_ < 5000 ) {
                // high damping mode
                for ( int j = 0; j < 3; j++ )
                    ctrl_.kd_des[ i * 3 + j ] = 4;
            }
            else if ( clear_flag && ( time_diff < Ctrl_Over_Time_Err ) ) {
                err_flag_ &= ( 0xFFFFFFFF - 0x02 );
                printf( "[FSM MotorCtrl] Reset kMotorCtrl communication timeout! \n" );
            }
        }
        else
            wait_reconnect_ = 0;

        if ( software_limit ) {
            float delta = 0.04;
            if ( ctrl_.q_des[ i * 3 + 0 ] < ( this->data_->leg_controller->q_abad_lowerbound_ + delta ) )
                ctrl_.q_des[ i * 3 + 0 ] = this->data_->leg_controller->q_abad_lowerbound_ + delta;  // abad
            if ( ctrl_.q_des[ i * 3 + 0 ] > ( this->data_->leg_controller->q_abad_upperbound_ - delta ) )
                ctrl_.q_des[ i * 3 + 0 ] = this->data_->leg_controller->q_abad_upperbound_ - delta;
            if ( ctrl_.q_des[ i * 3 + 2 ] < ( this->data_->leg_controller->q_knee_lowerbound_ + delta ) )
                ctrl_.q_des[ i * 3 + 2 ] = this->data_->leg_controller->q_knee_lowerbound_ + delta;  // knee
            if ( ctrl_.q_des[ i * 3 + 2 ] > ( this->data_->leg_controller->q_knee_upperbound_ - delta ) )
                ctrl_.q_des[ i * 3 + 2 ] = this->data_->leg_controller->q_knee_upperbound_ - delta;
            if ( i == 0 || i == 1 ) {
                if ( ctrl_.q_des[ i * 3 + 1 ] < ( this->data_->leg_controller->q_fronthip_lowerbound_ + delta ) )
                    ctrl_.q_des[ i * 3 + 1 ] = this->data_->leg_controller->q_fronthip_lowerbound_ + delta;  // hip
                if ( ctrl_.q_des[ i * 3 + 1 ] > ( this->data_->leg_controller->q_fronthip_upperbound_ - delta ) )
                    ctrl_.q_des[ i * 3 + 1 ] = this->data_->leg_controller->q_fronthip_upperbound_ - delta;
            }
            else {
                if ( ctrl_.q_des[ i * 3 + 1 ] < ( this->data_->leg_controller->q_rearhip_lowerbound_ + delta ) )
                    ctrl_.q_des[ i * 3 + 1 ] = this->data_->leg_controller->q_rearhip_lowerbound_ + delta;
                if ( ctrl_.q_des[ i * 3 + 1 ] > ( this->data_->leg_controller->q_rearhip_upperbound_ - delta ) )
                    ctrl_.q_des[ i * 3 + 1 ] = this->data_->leg_controller->q_rearhip_upperbound_ - delta;
            }
        }
        if ( this->data_->user_parameters->motor_sdk_position_mutation_limit > 0.5 ) {

            diff[ 0 ] = ctrl_.q_des[ i * 3 + 0 ] - this->data_->leg_controller->datas_[ i ].q( 0 );
            diff[ 1 ] = ctrl_.q_des[ i * 3 + 1 ] - this->data_->leg_controller->datas_[ i ].q( 1 );
            diff[ 2 ] = ctrl_.q_des[ i * 3 + 2 ] - this->data_->leg_controller->datas_[ i ].q( 2 );

            if ( diff[ 0 ] > Deg2Rad( 8.0 ) || diff[ 0 ] < Deg2Rad( -8.0 ) ) {
                err_flag_ |= 0x4;
                ctrl_.q_des[ i * 3 + 0 ] -= ( diff[ 0 ] + Deg2Rad( 8.0 ) * ( diff[ 0 ] > 0 ? -1 : 1 ) );
            }
            else
                err_flag_ &= ( 0xffffffff - 0x4 );
            if ( diff[ 1 ] > Deg2Rad( 10.0 ) || diff[ 1 ] < Deg2Rad( -10.0 ) ) {
                err_flag_ |= 0x8;
                ctrl_.q_des[ i * 3 + 1 ] -= ( diff[ 1 ] + Deg2Rad( 10.0 ) * ( diff[ 1 ] > 0 ? -1 : 1 ) );
            }
            else
                err_flag_ &= ( 0xffffffff - 0x8 );
            if ( diff[ 2 ] > Deg2Rad( 12.0 ) || diff[ 2 ] < Deg2Rad( -12.0 ) ) {
                err_flag_ |= 0x10;
                ctrl_.q_des[ i * 3 + 2 ] -= ( diff[ 2 ] + Deg2Rad( 12.0 ) * ( diff[ 2 ] > 0 ? -1 : 1 ) );
            }
            else
                err_flag_ &= ( 0xffffffff - 0x10 );
        }
        this->data_->leg_controller->commands_[ i ].q_des << ctrl_.q_des[ i * 3 + 0 ], ctrl_.q_des[ i * 3 + 1 ], ctrl_.q_des[ i * 3 + 2 ];
        this->data_->leg_controller->commands_[ i ].qd_des << ctrl_.qd_des[ i * 3 + 0 ], ctrl_.qd_des[ i * 3 + 1 ], ctrl_.qd_des[ i * 3 + 2 ];
        this->data_->leg_controller->commands_[ i ].kp_joint << ctrl_.kp_des[ i * 3 + 0 ], 0, 0, 0, ctrl_.kp_des[ i * 3 + 1 ], 0, 0, 0, ctrl_.kp_des[ i * 3 + 2 ];
        this->data_->leg_controller->commands_[ i ].kd_joint << ctrl_.kd_des[ i * 3 + 0 ], 0, 0, 0, ctrl_.kd_des[ i * 3 + 1 ], 0, 0, 0, ctrl_.kd_des[ i * 3 + 2 ];
        this->data_->leg_controller->commands_[ i ].tau_feed_forward << ctrl_.tau_des[ i * 3 + 0 ], ctrl_.tau_des[ i * 3 + 1 ], ctrl_.tau_des[ i * 3 + 2 ];
    }
    ctrl_state_.err_flag            = err_flag_;
    ctrl_state_.ctrl_topic_interval = time_diff;
    motor_ctrl_state_lcm_.publish( "motor_ctrl_state", &ctrl_state_ );
    // err_flag_:
    // bit0:lost control over 10ms warn, divide the tau and qd_des by (over_time/10.0)
    // bit1:lost control over 500ms err, enter high damping mode, kd=10
    // bit2:abad motor diff angle over 5 degrees
    // bit3:hip motor diff angle over 8 degrees
    // bit4:knee motor diff angle over 10 degrees
}

template < typename T > bool FsmStateMotorCtrl< T >::CheckZero() {
    for ( int i = 0; i < 4; i++ )
        for ( int j = 0; j < 3; j++ ) {
            if ( fabs( ctrl_.kp_des[ i * 3 + j ] ) > 0.001 || fabs( ctrl_.kd_des[ i * 3 + j ] ) > 0.001 || fabs( ctrl_.tau_des[ i * 3 + j ] ) > 0.001 )
                return false;
        }
    return true;
}

template < typename T > FsmStateName FsmStateMotorCtrl< T >::CheckTransition() {
    auto& cmd = this->data_->command;
    switch ( cmd->mode ) {
    case MotionMode::kMotorCtrl:
        break;
    case MotionMode::kOff:
    case MotionMode::kPureDamper:
    case MotionMode::kRecoveryStand:
    case MotionMode::kRlReset:
        this->next_state_name_     = ( FsmStateName )cmd->mode;
        this->transition_duration_ = 0.;
        break;
    default:
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << ( int )MotionMode::kMotion << " to " << cmd->mode << std::endl;
    }
    // Return the next state name to the FSM
    return this->next_state_name_;
}

template < typename T > TransitionData< T > FsmStateMotorCtrl< T >::Transition() {
    // Switch FSM control mode
    switch ( this->next_state_name_ ) {
    case FsmStateName::kOff:
        this->TurnOffAllSafetyChecks();
        this->transition_data_.done = true;
        break;
    case FsmStateName::kPureDamper:
    case FsmStateName::kRecoveryStand:
    case FsmStateName::kRlReset:
        this->transition_data_.done = true;
        break;
    default:
        std::cout << "[CONTROL FSM] Something went wrong in transition" << std::endl;
    }
    // Return the transition data to the FSM
    return this->transition_data_;
}

template < typename T > void FsmStateMotorCtrl< T >::OnExit() {
    err_flag_ = 0;
}

// template class FsmStateMotorCtrl<double>;
template class FsmStateMotorCtrl< float >;
