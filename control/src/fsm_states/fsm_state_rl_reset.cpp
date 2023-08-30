#include <unistd.h>

#include "fsm_states/fsm_state_rl_reset.hpp"
#include <Configuration.h>

/**
 * @brief Constructor for the FSM State that passes in state specific info to
 *        the generic FSM State constructor.
 *
 * @param control_fsm_data holds all of the relevant control data
 */
template < typename T >
FsmStateRlReset< T >::FsmStateRlReset( ControlFsmData< T >* control_fsm_data )
    : FsmState< T >( control_fsm_data, FsmStateName::kRlReset, "rl_reset" ), policy_( { 256, 128, 64 } ) {
    // Set the pre controls safety checks
    this->check_safe_orientation_ = false;
    this->check_robot_lifted_     = false;

    // Post control safety checks
    this->check_desired_foot_position_ = false;
    this->check_feed_forward_force_    = false;

    // Load rl model parameters
    load_path_   = THIS_COM "control/rl_models/cyberdog2/reset/model.txt";
    policy_.UpdateParamsFromTxt( load_path_ );

    // Init observation
    projected_gravity_.setZero();
    joint_pos_.setZero();
    joint_vel_.setZero();
    prev_action_.setZero();

    obs_.setZero();
    obs_history_.setZero();
    action_.setZero();

    default_joint_pos_recovery_ << 0., -0.88, 1.44, 0., -0.88, 1.44, 0., -0.88, 1.44, 0., -0.88, 1.44;

    default_joint_pos_mid_ << 0., -42., 87.5, 0., -42., 87.5, 0., -62., 87.5, 0., -62., 87.5;
    default_joint_pos_mid_ *= M_PI / 180;

    default_joint_pos_range_ << 39., 118., 57.5, 39., 118., 57.5, 39., 118., 57.5, 39., 118., 57.5;
    default_joint_pos_range_ *= M_PI / 180;

    motor_remap_ << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
    gravity_vec_ << 0., 0., -1.;

    // Init scale
    joint_pos_scale_ = 1.0;
    joint_vel_scale_ = 0.05;
    action_scale_    = 1.0;

    action_clip_ = 1.0;
    control_dt_  = 0.03;  // 33Hz

    rl_model_inference_running_ = false;
    CreateRlModelInferenceThread();

    // For final posture correction
    correct_posture_flag_          = true;
    slower_reset_correct_progress_ = 70;
    slower_reset_correct_kp_       = 50.;
}

/**
 * @brief Behavior to be carried out when entering a state.
 *
 */
template < typename T > void FsmStateRlReset< T >::OnEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    // Reset the transition data
    this->transition_data_.ResetTransitionDone();

    // Reset iteration counter
    iter_ = 0;

    this->motion_progress_bar_ = 0;

    obs_.setZero();
    obs_history_.setZero();
    action_.setZero();
    prev_action_.setZero();

    des_joint_pos_ = default_joint_pos_mid_;

    SetPolicyParams();

    rl_model_inference_running_ = true;
}

/**
 * @brief Calls the functions to be executed on each control loop iteration.
 *
 */
template < typename T > void FsmStateRlReset< T >::Run() {
    this->data_->leg_controller->ZeroCommand();
    this->data_->leg_controller->legs_enabled_ = true;

    SetPolicyParams();

    for ( int leg( 0 ); leg < 4; ++leg ) {
        for ( int jidx( 0 ); jidx < 3; ++jidx ) {
            this->data_->leg_controller->commands_[ leg ].q_des[ jidx ]  = des_joint_pos_( leg * 3 + jidx );
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
template < typename T > FsmStateName FsmStateRlReset< T >::CheckTransition() {
    this->next_state_name_ = this->state_name_;
    iter_++;
    auto& cmd = this->data_->command;

    // Switch FSM control mode
    switch ( cmd->mode ) {
    case MotionMode::kRlReset:
        break;

    case MotionMode::kPoseCtrl:
    case MotionMode::kPureDamper:
    case MotionMode::kQpStand:
    case MotionMode::kOff:  // normal c
    case MotionMode::kLocomotion:
    case MotionMode::kTwoLegStand:
    case MotionMode::kJump3d:
    case MotionMode::kForceJump:
    case MotionMode::kLifted:
    case MotionMode::kRecoveryStand:
    case MotionMode::kRlRapid:
        this->next_state_name_ = ( FsmStateName )cmd->mode;
        break;

    default:
        if ( this->iter_printf_ >= this->iter_printf_reset_ ) {
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << ( int )MotionMode::kRlReset << " to " << ( int )this->data_->command->mode << std::endl;
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
template < typename T > TransitionData< T > FsmStateRlReset< T >::Transition() {
    // Finish Transition
    switch ( this->next_state_name_ ) {
    case FsmStateName::kOff:  // normal
    case FsmStateName::kPureDamper:
    case FsmStateName::kRecoveryStand:
    case FsmStateName::kRlRapid:
    case FsmStateName::kTwoLegStand:
    case FsmStateName::kJump3d:
    case FsmStateName::kForceJump:
    case FsmStateName::kLifted:
    case FsmStateName::kPoseCtrl:
    case FsmStateName::kQpStand:
    case FsmStateName::kLocomotion:
        this->transition_data_.done = true;
        // if ( this->motion_progress_bar_ >= 100 )
        //     this->transition_data_.done = true;
        // else
        //     this->transition_data_.done = false;
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
template < typename T > void FsmStateRlReset< T >::OnExit() {
    // Nothing to clean up when exiting
    StopRlModelInference();
}

/**
 * @brief Remap joint data for real robot (different in isaac gym).
 *
 * @param joint_data joint position or joint velocity
 */
template < typename T > void FsmStateRlReset< T >::Remap( Eigen::Matrix< float, 12, 1 >& joint_data ) {
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
template < typename T > void FsmStateRlReset< T >::RlModelInferenceThread() {
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
template < typename T > void FsmStateRlReset< T >::CreateRlModelInferenceThread() {
    rl_model_inference_thread_ = new std::thread( &FsmStateRlReset::RlModelInferenceThread, this );

    int         policy = SCHED_RR;
    sched_param sch_params;
    sch_params.sched_priority = sched_get_priority_max( policy );
    pthread_setschedparam( rl_model_inference_thread_->native_handle(), policy, &sch_params );
    int rc = pthread_getschedparam( rl_model_inference_thread_->native_handle(), &policy, &sch_params );
    ( void )rc;
    std::cout << "[check pthread] [rl_reset] rl_model_inference  " << policy << "   " << sch_params.sched_priority << std::endl;
}

/**
 * @brief Stop the model inference thread.
 *
 */
template < typename T > void FsmStateRlReset< T >::StopRlModelInference() {
    rl_model_inference_running_ = false;
}

/**
 * @brief Set different rl_reset policy params.
 *
 */
template < typename T > void FsmStateRlReset< T >::SetPolicyParams() {
    auto& cmd = this->data_->command;
    this->data_->robot_current_state->gait_id = cmd->gait_id;

    kp_mat_ = Vec3< float >( 15., 15., 15. ).asDiagonal();
    kd_mat_ = Vec3< float >( 1.1, 1.1, 1.1 ).asDiagonal();

    if ( correct_posture_flag_ && this->motion_progress_bar_ > slower_reset_correct_progress_ ) {
        float kp = 15 + ( slower_reset_correct_kp_ - 15 ) * ( this->motion_progress_bar_ - slower_reset_correct_progress_ ) / ( 100 - slower_reset_correct_progress_ );
        kp_mat_  = Vec3< float >( kp, kp, kp ).asDiagonal();
    }

    action_delta_clip_     = 0.1;
    use_action_delta_clip_ = true;    
}

/**
 * @brief Use the trained policy to update the robot actions.
 *
 */
template < typename T > void FsmStateRlReset< T >::UpdateAction() {
    action_ = policy_.Forward( obs_history_ );
    action_ = action_.cwiseMin( action_clip_ ).cwiseMax( -action_clip_ );
    if ( use_action_delta_clip_ && !prev_action_.isZero() ) {
        auto upper_bound = prev_action_ + Eigen::MatrixXf::Ones( 12, 1 ) * action_delta_clip_;
        auto lower_bound = prev_action_ - Eigen::MatrixXf::Ones( 12, 1 ) * action_delta_clip_;
        action_          = action_.cwiseMin( upper_bound ).cwiseMax( lower_bound );
        action_          = action_.cwiseMin( action_clip_ ).cwiseMax( -action_clip_ );
    }
    prev_action_ = action_;
    action_ *= action_scale_;
    Remap( action_ );
    des_joint_pos_ = action_.cwiseProduct( default_joint_pos_range_ ) + default_joint_pos_mid_;
    // std::cout << "[RL reset] des_joint_pos_: " << des_joint_pos_.transpose() << std::endl;
}

/**
 * @brief Update historical observation data.
 *
 */
template < typename T > void FsmStateRlReset< T >::UpdateHistory() {
    obs_history_ << obs_history_.segment( kNumObsReset, kNumObsReset * ( kNumObsHistoryReset - 1 ) ), obs_;
}

/**
 * @brief Update current observation data.
 *
 */
template < typename T > void FsmStateRlReset< T >::UpdateObservation() {
    robot_quat_ << this->data_->state_estimator->GetResult().orientation;
    UpdateProjectedGravity( robot_quat_, gravity_vec_ );

    joint_pos_ << this->data_->leg_controller->datas_[ 0 ].q, this->data_->leg_controller->datas_[ 1 ].q, this->data_->leg_controller->datas_[ 2 ].q, this->data_->leg_controller->datas_[ 3 ].q;
    joint_vel_ << this->data_->leg_controller->datas_[ 0 ].qd, this->data_->leg_controller->datas_[ 1 ].qd, this->data_->leg_controller->datas_[ 2 ].qd, this->data_->leg_controller->datas_[ 3 ].qd;

    joint_pos_normal_ = ( joint_pos_ - default_joint_pos_mid_ ).cwiseQuotient( default_joint_pos_range_ );

    Remap( joint_pos_normal_ );
    Remap( joint_vel_ );

    obs_ << projected_gravity_, joint_pos_normal_ * joint_pos_scale_, joint_vel_ * joint_vel_scale_, prev_action_;

    CalculateMotionProgress();
}

/**
 * @brief The projected gravity is a unit vector that points to the direction of the gravity
 *        in the robot base frame, capturing the robot orientation w.r.t. gravity.
 *
 * @param robot_quat body orientation quaternion: (w, x, y, z)
 * @param gravity_vec unit gravity vector: {0, 0, -1}
 */
template < typename T > void FsmStateRlReset< T >::UpdateProjectedGravity( Eigen::Vector4f& robot_quat, Eigen::Vector3f& gravity_vec ) {
    float           s   = robot_quat[ 0 ];
    Eigen::Vector3f v   = robot_quat.segment( 1, 3 );
    Eigen::Vector3f vxp = v.cross( gravity_vec );
    float           vdp = v.dot( gravity_vec );
    projected_gravity_  = 2 * vdp * v + ( 2 * s * s - 1 ) * gravity_vec - 2 * s * vxp;
}

/**
 * @brief Calculate motion progress based on rewards.
 *
 */
template < typename T > void FsmStateRlReset< T >::CalculateMotionProgress() {
    cos_dist_ = gravity_vec_.dot( projected_gravity_ );
    r_reset_  = 0.5 * ( 0.5 + 0.5 * cos_dist_ ) * ( 0.5 + 0.5 * cos_dist_ );
    if ( cos_dist_ > cos( M_PI / 6 ) ) {
        r_dofpos_ = exp( -1.0 * ( joint_pos_ - default_joint_pos_recovery_ ).array().pow( 2 ).sum() );
        r_dofvel_ = exp( -0.02 * joint_vel_.array().pow( 2 ).sum() );
        r_stand_  = 0.2 + 0.6 * r_dofpos_ + 0.2 * r_dofvel_;
    }
    else {
        r_dofpos_ = 0;
        r_dofvel_ = 0;
        r_stand_  = 0;
    }
    r_reset_ += 0.5 * r_stand_;
    // std::cout << "r_reset:\t" << r_reset_ << std::endl;

    this->motion_progress_bar_ = std::min( int( r_reset_ * 100 + 5 ), 100 );  // reward > 0.95
    // this->motionProgressBar = std::min( int( r_reset_ * 100 + 10 ), 100 );  // reward > 0.90
    // std::cout << "motionProgressBar:\t" << this->motion_progress_bar_ << std::endl;
}

template class FsmStateRlReset< float >;
