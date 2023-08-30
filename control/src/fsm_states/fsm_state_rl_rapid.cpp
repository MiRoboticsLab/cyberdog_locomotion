#include <unistd.h>

#include "fsm_states/fsm_state_rl_rapid.hpp"
#include <Configuration.h>

/**
 * @brief Constructor for the FSM State that passes in state specific info to
 *        the generic FSM State constructor.
 *
 * @param control_fsm_data holds all of the relevant control data
 */
template < typename T >
FsmStateRlRapid< T >::FsmStateRlRapid( ControlFsmData< T >* control_fsm_data )
    : FsmState< T >( control_fsm_data, FsmStateName::kRlRapid, "rl_rapid" ), policy_( { 512, 256, 128 } ), adaptation_( { 256, 32 } ) {
    // Set the pre controls safety checks
    this->check_safe_orientation_ = true;
    this->check_robot_lifted_     = false;

    // Post control safety checks
    this->check_desired_foot_position_ = true;
    this->check_feed_forward_force_    = false;

    // Load rl model parameters
    policy_path_     = THIS_COM "control/rl_models/cyberdog2/rapid/model_base.txt";
    adaptation_path_ = THIS_COM "control/rl_models/cyberdog2/rapid/model_adaptation.txt";
    policy_.UpdateParamsFromTxt( policy_path_ );
    adaptation_.UpdateParamsFromTxt( adaptation_path_ );

    // Init observation
    projected_gravity_.setZero();
    command_.setZero();
    joint_pos_.setZero();
    joint_vel_.setZero();
    prev_action_.setZero();

    obs_.setZero();
    policy_obs_.setZero();
    obs_history_.setZero();
    adaptation_out_.setZero();
    action_.setZero();

    default_joint_pos_ << 0., -0.75, 1.4, 0., -0.75, 1.4, 0., -0.75, 1.4, 0., -0.75, 1.4;
    motor_remap_ << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
    gravity_vec_ << 0., 0., -1.;

    kp_mat_ = Vec3< float >( 15., 15., 15. ).asDiagonal();
    kd_mat_ = Vec3< float >( 0.5, 0.5, 0.5 ).asDiagonal();

    // Init scale
    command_scale_ << 2.0, 2.0, 0.25;
    joint_pos_scale_      = 1.0;
    joint_vel_scale_      = 0.05;
    action_scale_         = 0.25;
    abad_scale_reduction_ = 0.5;

    disable_lin_vel_ = true;

    control_dt_   = 0.02;  // 50Hz
    filter_ratio_ = 1.0;

    rl_model_inference_running_ = false;
    CreateRlModelInferenceThread();
}

/**
 * @brief Behavior to be carried out when entering a state.
 *
 */
template < typename T > void FsmStateRlRapid< T >::OnEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    // Reset the transition data
    this->transition_data_.ResetTransitionDone();

    // Reset iteration counter
    iter_ = 0;

    this->motion_progress_bar_ = 100;

    this->data_->robot_current_state->gait_id = 0;

    obs_.setZero();
    policy_obs_.setZero();
    obs_history_.setZero();
    action_.setZero();
    prev_action_.setZero();

    des_joint_pos_               = default_joint_pos_;
    des_joint_pos_filtered_      = des_joint_pos_;
    last_des_joint_pos_filtered_ = des_joint_pos_filtered_;

    cur_lin_vel_x_   = 0.0;
    cur_lin_vel_y_   = 0.0;
    cur_ang_vel_yaw_ = 0.0;

    stop_update_cmd_ = false;

    SetUserParameters();

    rl_model_inference_running_ = true;
}

/**
 * @brief Calls the functions to be executed on each control loop iteration.
 *
 */
template < typename T > void FsmStateRlRapid< T >::Run() {
    this->data_->leg_controller->ZeroCommand();
    this->data_->leg_controller->legs_enabled_ = true;

    SetUserParameters();

    for ( int leg( 0 ); leg < 4; ++leg ) {
        for ( int jidx( 0 ); jidx < 3; ++jidx ) {
            this->data_->leg_controller->commands_[ leg ].q_des[ jidx ]  = des_joint_pos_filtered_( leg * 3 + jidx );
            this->data_->leg_controller->commands_[ leg ].qd_des[ jidx ] = 0.;
        }
        this->data_->leg_controller->commands_[ leg ].kp_joint << kp_mat_;
        this->data_->leg_controller->commands_[ leg ].kd_joint << kd_mat_;
    }
}

/**
 * @brief Manages which states can be transitioned into either by the user
 *        commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template < typename T > FsmStateName FsmStateRlRapid< T >::CheckTransition() {
    this->next_state_name_ = this->state_name_;
    iter_++;
    auto& cmd = this->data_->command;

    // Switch FSM control mode
    switch ( cmd->mode ) {
    case MotionMode::kRlRapid:
        break;

    case MotionMode::kOff:  // normal c
    case MotionMode::kPureDamper:
    case MotionMode::kRecoveryStand:
    case MotionMode::kLocomotion:
    case MotionMode::kRlReset:
        this->next_state_name_ = ( FsmStateName )cmd->mode;
        stop_update_cmd_       = true;
        break;

    default:
        if ( this->iter_printf_ >= this->iter_printf_reset_ ) {
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << ( int )MotionMode::kRlRapid << " to " << ( int )this->data_->command->mode << std::endl;
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
template < typename T > TransitionData< T > FsmStateRlRapid< T >::Transition() {
    // Finish Transition
    switch ( this->next_state_name_ ) {
    case FsmStateName::kOff:  // normal
    case FsmStateName::kPureDamper:
    case FsmStateName::kRecoveryStand:
    case FsmStateName::kLocomotion:
    case FsmStateName::kRlReset:
        Run();
        static float threshold = 0.03;
        if ( std::abs( cur_lin_vel_x_ - lin_vel_x_offset_ ) < threshold && std::abs( cur_lin_vel_y_ - lin_vel_y_offset_ ) < threshold
             && std::abs( cur_ang_vel_yaw_ - ang_vel_yaw_offset_ ) < threshold ) {
            this->transition_data_.done = true;
        }
        else {
            this->transition_data_.done = false;
        }
        break;
    case FsmStateName::kRlRapid:
        stop_update_cmd_ = false;
        break;

    default:
        std::cout << "[CONTROL FSM RL Controller] Something went wrong in transition" << std::endl;
    }

    // Return the transition data to the FSM
    return this->transition_data_;
}

/**
 * @brief Cleans up the state information on exiting the state.
 *
 */
template < typename T > void FsmStateRlRapid< T >::OnExit() {
    // Nothing to clean up when exiting
    StopRlModelInference();
}

/**
 * @brief Set user parameters from yaml config.
 *
 */
template < typename T > void FsmStateRlRapid< T >::SetUserParameters() {
    // Command range
    min_lin_vel_x_   = -4.0;
    max_lin_vel_x_   = 4.0;
    min_lin_vel_y_   = -2.0;
    max_lin_vel_y_   = 2.0;
    min_ang_vel_yaw_ = this->data_->user_parameters->vel_xy_yaw_min_rapid_rl[ 2 ];
    max_ang_vel_yaw_ = this->data_->user_parameters->vel_xy_yaw_max_rapid_rl[ 2 ];

    // Velocity offset
#if ( ONBOARD_BUILD == 1 )
    lin_vel_x_offset_   = this->data_->user_parameters->x_offset_rapid_rl;
    lin_vel_y_offset_   = this->data_->user_parameters->y_offset_rapid_rl;
    ang_vel_yaw_offset_ = this->data_->user_parameters->yaw_offset_rapid_rl;
#else
    lin_vel_x_offset_   = -0.1;
    lin_vel_y_offset_   = 0.0;
    ang_vel_yaw_offset_ = 0.15;
#endif

    // Acceleration limit
    min_acc_lin_vel_x_   = -2.0;
    max_acc_lin_vel_x_   = 2.0;
    min_acc_lin_vel_y_   = -2.0;
    max_acc_lin_vel_y_   = 2.0;
    min_acc_ang_vel_yaw_ = this->data_->user_parameters->acc_xy_yaw_min_rapid_rl[ 2 ];
    max_acc_ang_vel_yaw_ = this->data_->user_parameters->acc_xy_yaw_max_rapid_rl[ 2 ];
}

/**
 * @brief Remap joint data for real robot (different in isaac gym).
 *
 * @param joint_data joint position or joint velocity
 */
template < typename T > void FsmStateRlRapid< T >::Remap( Eigen::Matrix< float, 12, 1 >& joint_data ) {
    Eigen::Matrix< float, 12, 1 > new_joint_data;
    for ( int i( 0 ); i < 12; i++ ) {
        new_joint_data[ i ] = joint_data[ motor_remap_[ i ] ];
    }
    joint_data = new_joint_data;
}

/**
 * @brief Run model inference at specified frequency.
 *
 */
template < typename T > void FsmStateRlRapid< T >::RlModelInferenceThread() {
    auto timer_fd = timerfd_create( CLOCK_MONOTONIC, 0 );

    int seconds     = ( int )control_dt_;
    int nanoseconds = ( int )( 1e9 * std::fmod( control_dt_, 1.f ) );

    itimerspec timer_spec;
    timer_spec.it_interval.tv_sec  = seconds;
    timer_spec.it_value.tv_sec     = seconds;
    timer_spec.it_value.tv_nsec    = nanoseconds;
    timer_spec.it_interval.tv_nsec = nanoseconds;

    timerfd_settime( timer_fd, 0, &timer_spec, nullptr );
    unsigned long long missed = 0;

    while ( true ) {
        if ( rl_model_inference_running_ ) {
            UpdateObservation();
            UpdateHistory();
            UpdateAction();
        }

        int m = read( timer_fd, &missed, sizeof( missed ) );
        ( void )m;
    }
}

/**
 * @brief Start the model inference thread.
 *
 */
template < typename T > void FsmStateRlRapid< T >::CreateRlModelInferenceThread() {
    rl_model_inference_thread_ = new std::thread( &FsmStateRlRapid::RlModelInferenceThread, this );

    int         policy = SCHED_RR;
    sched_param sch_params;
    sch_params.sched_priority = sched_get_priority_max( policy );
    pthread_setschedparam( rl_model_inference_thread_->native_handle(), policy, &sch_params );
    int rc = pthread_getschedparam( rl_model_inference_thread_->native_handle(), &policy, &sch_params );
    ( void )rc;
    std::cout << "[check pthread] [RLRapid] rl_model_inference  " << policy << "   " << sch_params.sched_priority << std::endl;
}

/**
 * @brief Stop the model inference thread.
 *
 */
template < typename T > void FsmStateRlRapid< T >::StopRlModelInference() {
    rl_model_inference_running_ = false;
}

/**
 * @brief Use the trained policy to update the robot actions.
 *
 */
template < typename T > void FsmStateRlRapid< T >::UpdateAction() {
    adaptation_out_ = adaptation_.Forward( obs_history_ );
    policy_obs_ << obs_, adaptation_out_;
    action_      = policy_.Forward( policy_obs_ );
    prev_action_ = action_;
    action_ *= action_scale_;
    action_[ 0 ] *= abad_scale_reduction_;
    action_[ 3 ] *= abad_scale_reduction_;
    action_[ 6 ] *= abad_scale_reduction_;
    action_[ 9 ] *= abad_scale_reduction_;

    Remap( action_ );

    des_joint_pos_               = action_ + default_joint_pos_;
    last_des_joint_pos_filtered_ = des_joint_pos_filtered_;
    des_joint_pos_filtered_      = filter_ratio_ * des_joint_pos_ + ( 1 - filter_ratio_ ) * last_des_joint_pos_filtered_;
}

/**
 * @brief Update speed command.
 *
 */
template < typename T > void FsmStateRlRapid< T >::UpdateCommand() {
    auto& cmd = this->data_->command;

    float cmd_des_lin_vel_x   = cmd->vel_des[ 0 ];
    float cmd_des_lin_vel_y   = cmd->vel_des[ 1 ];
    float cmd_des_ang_vel_yaw = cmd->vel_des[ 2 ];

    if ( stop_update_cmd_ ) {
        cmd_des_lin_vel_x   = 0;
        cmd_des_lin_vel_y   = 0;
        cmd_des_ang_vel_yaw = 0;
    }

    float lin_vel_x_coff   = ( cmd_des_lin_vel_x > 0 ) ? max_lin_vel_x_ : -min_lin_vel_x_;
    float lin_vel_y_coff   = ( cmd_des_lin_vel_y > 0 ) ? max_lin_vel_y_ : -min_lin_vel_y_;
    float ang_vel_yaw_coff = ( cmd_des_ang_vel_yaw > 0 ) ? max_ang_vel_yaw_ : -min_ang_vel_yaw_;

    float des_lin_vel_x   = cmd_des_lin_vel_x * lin_vel_x_coff + lin_vel_x_offset_;
    float des_lin_vel_y   = cmd_des_lin_vel_y * lin_vel_y_coff + lin_vel_y_offset_;
    float des_ang_vel_yaw = cmd_des_ang_vel_yaw * ang_vel_yaw_coff + ang_vel_yaw_offset_;

    des_lin_vel_x   = ApplyVelocityMeetAccelationLimit( cur_lin_vel_x_, des_lin_vel_x, min_lin_vel_x_, max_lin_vel_x_, min_acc_lin_vel_x_, max_acc_lin_vel_x_, control_dt_ );
    des_lin_vel_y   = ApplyVelocityMeetAccelationLimit( cur_lin_vel_y_, des_lin_vel_y, min_lin_vel_y_, max_lin_vel_y_, min_acc_lin_vel_y_, max_acc_lin_vel_y_, control_dt_ );
    des_ang_vel_yaw = ApplyVelocityMeetAccelationLimit( cur_ang_vel_yaw_, des_ang_vel_yaw, min_ang_vel_yaw_, max_ang_vel_yaw_, min_acc_ang_vel_yaw_, max_acc_ang_vel_yaw_, control_dt_ );

    if ( disable_lin_vel_ ) {
        command_ << lin_vel_x_offset_, lin_vel_y_offset_, des_ang_vel_yaw;
        cur_lin_vel_x_   = lin_vel_x_offset_;
        cur_lin_vel_y_   = lin_vel_y_offset_;
        cur_ang_vel_yaw_ = des_ang_vel_yaw;
    }
    else {
        command_ << des_lin_vel_x, des_lin_vel_y, des_ang_vel_yaw;
        cur_lin_vel_x_   = des_lin_vel_x;
        cur_lin_vel_y_   = des_lin_vel_y;
        cur_ang_vel_yaw_ = des_ang_vel_yaw;
    }
}

/**
 * @brief Update historical observation data.
 *
 */
template < typename T > void FsmStateRlRapid< T >::UpdateHistory() {
    obs_history_ << obs_history_.segment( kNumObsRapid, kNumObsRapid * ( kNumObsHistoryRapid - 1 ) ), obs_;
}

/**
 * @brief Update current observation data.
 *
 */
template < typename T > void FsmStateRlRapid< T >::UpdateObservation() {
    robot_quat_ << this->data_->state_estimator->GetResult().orientation;
    UpdateProjectedGravity( robot_quat_, gravity_vec_ );

    UpdateCommand();

    joint_pos_ << this->data_->leg_controller->datas_[ 0 ].q, this->data_->leg_controller->datas_[ 1 ].q, this->data_->leg_controller->datas_[ 2 ].q, this->data_->leg_controller->datas_[ 3 ].q;
    joint_vel_ << this->data_->leg_controller->datas_[ 0 ].qd, this->data_->leg_controller->datas_[ 1 ].qd, this->data_->leg_controller->datas_[ 2 ].qd, this->data_->leg_controller->datas_[ 3 ].qd;

    Remap( joint_pos_ );
    Remap( joint_vel_ );

    obs_ << projected_gravity_, command_.cwiseProduct( command_scale_ ), ( joint_pos_ - default_joint_pos_ ) * joint_pos_scale_, joint_vel_ * joint_vel_scale_, prev_action_;
}

/**
 * @brief The projected gravity is a unit vector that points to the direction of the gravity
 *        in the robot base frame, capturing the robot orientation w.r.t. gravity.
 *
 * @param robot_quat body orientation quaternion: (w, x, y, z)
 * @param gravity_vec unit gravity vector: {0, 0, -1}
 */
template < typename T > void FsmStateRlRapid< T >::UpdateProjectedGravity( Eigen::Vector4f& robot_quat, Eigen::Vector3f& gravity_vec ) {
    float           s   = robot_quat[ 0 ];
    Eigen::Vector3f v   = robot_quat.segment( 1, 3 );
    Eigen::Vector3f vxp = v.cross( gravity_vec );
    float           vdp = v.dot( gravity_vec );
    projected_gravity_  = 2 * vdp * v + ( 2 * s * s - 1 ) * gravity_vec - 2 * s * vxp;
}

template class FsmStateRlRapid< float >;
