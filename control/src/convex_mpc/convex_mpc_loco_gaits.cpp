#include <iostream>
#include <parameters.hpp>
#include <unistd.h>

#include "utilities/control_utilities.hpp"
#include "utilities/pseudoInverse.hpp"
#include "utilities/timer.hpp"

#include "convex_mpc/convex_mpc_loco_gaits.hpp"

// #define DRAW_DEBUG_SWINGS
// #define DRAW_DEBUG_PATH
// #define USE_TERRAIN_DETECTER

// #define DEV_MODE
// #define WALK_GAIT_LOW_FREQ

static const int gait_period_base = 10;

/**
 * @brief Construct a new Convex Mpc Loco Gaits:: Convex Mpc Loco Gaits object
 *
 * @param dt control time step
 * @param iterations_between_mpc iteration between mpc updates
 */
ConvexMpcLocoGaits::ConvexMpcLocoGaits( float dt, int iterations_between_mpc )
    : iter_between_mpc_( iterations_between_mpc ), horizon_length_( 10 ), dt_( dt ), trot_medium_( int( gait_period_base * 1.2 ), Vec4< int >( 0, 6, 6, 0 ), Vec4< int >( 6, 6, 6, 6 ), "TrotMedium" ),
      trot_fast_( int( gait_period_base * 1.0 ), Vec4< int >( 0, 5, 5, 0 ), Vec4< int >( 4, 4, 4, 4 ), "TrotFast" ),
      trot_slow_( int( gait_period_base * 2.0 ), Vec4< int >( 0, 10, 10, 0 ), Vec4< int >( 12, 12, 12, 12 ), "TrotSlow" ),
      bound_( gait_period_base * 1.2, Vec4< int >( 6, 6, 0, 0 ), Vec4< int >( 8, 8, 8, 8 ), "Bound" ), pronk_( gait_period_base, Vec4< int >( 0, 0, 0, 0 ), Vec4< int >( 4, 4, 4, 4 ), "Pronk" ),
#ifdef WALK_GAIT_LOW_FREQ
      // stable for hip 0.05
      walk_forward_( int( gait_period_base * 4.4 ), Vec4< int >( 22, 0, 11, 33 ), Vec4< int >( 34, 34, 34, 34 ), "Walk" ),
      walk_backward_( int( gait_period_base * 4.4 ), Vec4< int >( 0, 22, 11, 33 ), Vec4< int >( 34, 34, 34, 34 ), "Walk" ),
#else
      // stable for hip 0.08
      walk_forward_( int( gait_period_base * 2.4 ), Vec4< int >( 12, 0, 6, 18 ), Vec4< int >( 18, 18, 18, 18 ), "Walk" ),
      walk_backward_( int( gait_period_base * 2.4 ), Vec4< int >( 0, 12, 6, 18 ), Vec4< int >( 18, 18, 18, 18 ), "Walk" ),
#endif
      pace_( gait_period_base, Vec4< int >( 5, 0, 5, 0 ), Vec4< int >( 5, 5, 5, 5 ), "Pacing" ),
      passive_trot_( gait_period_base * 1.2, Vec4< int >( 0, 6, 6, 0 ), Vec4< int >( 6, 6, 6, 6 ), "PassiveTrot" ),
      trot_24_16_( int( gait_period_base * 2.4 ), Vec4< int >( 0, 12, 12, 0 ), Vec4< int >( 16, 16, 16, 16 ), "Trot24_16" ),
      trot_22_14_( int( gait_period_base * 2.2 ), Vec4< int >( 0, 11, 11, 0 ), Vec4< int >( 14, 14, 14, 14 ), "Trot22_14" ),
      trot_20_12_( int( gait_period_base * 2.0 ), Vec4< int >( 0, 10, 10, 0 ), Vec4< int >( 12, 12, 12, 12 ), "Trot20_10" ),
      trot_18_11_( int( gait_period_base * 1.8 ), Vec4< int >( 0, 9, 9, 0 ), Vec4< int >( 11, 11, 11, 11 ), "Trot18_9" ),
      trot_16_10_( int( gait_period_base * 1.6 ), Vec4< int >( 0, 8, 8, 0 ), Vec4< int >( 10, 10, 10, 10 ), "Trot16_10" ),
      trot_14_8_( int( gait_period_base * 1.4 ), Vec4< int >( 0, 7, 7, 0 ), Vec4< int >( 8, 8, 8, 8 ), "Trot14_8" ),
      trot_12_6_( int( gait_period_base * 1.2 ), Vec4< int >( 0, 6, 6, 0 ), Vec4< int >( 6, 6, 6, 6 ), "Trot12_6" ),
      trot_10_5_( gait_period_base, Vec4< int >( 0, 5, 5, 0 ), Vec4< int >( 5, 5, 5, 5 ), "Trot10_5" ),
      trot_10_4_( gait_period_base, Vec4< int >( 0, 5, 5, 0 ), Vec4< int >( 4, 4, 4, 4 ), "Trot10_4" ),
      trot_8_3_( int( gait_period_base * 0.8 ), Vec4< int >( 0, 4, 4, 0 ), Vec4< int >( 3, 3, 3, 3 ), "Trot8_3" ),
      stand_( int( gait_period_base * 1.0 ), Vec4< int >( 0, 0, 0, 0 ), Vec4< int >( 10, 10, 10, 10 ), "Stand" ),
      stand_no_pr_( int( gait_period_base * 1.0 ), Vec4< int >( 0, 0, 0, 0 ), Vec4< int >( 10, 10, 10, 10 ), "StandNoPr" ),
      mini_pronk_( gait_period_base, Vec4< int >( 0, 0, 0, 0 ), Vec4< int >( 6, 6, 6, 6 ), "MiniPronk" ) {
    dt_mpc_ = dt_ * iter_between_mpc_;
    printf( "[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt_, iter_between_mpc_, dt_mpc_ );

    vel_cmd_.setZero();
#ifdef USE_ABSOLUTE_ODOM_FOR_ALL
    pos_cmd_rel_ << 0.0, 0.0, 0.3;
#else
    pos_cmd_ << 0.0, 0.0, 0.3;
#endif
    rpy_cmd_.setZero();
    step_height_cmd_.setConstant( 0.05 );
    step_height_ratio_ = 1.0;
    gait_cmd_          = GaitId::kTrot10v5;
    omni_cmd_          = false;
    duration_mode_     = false;
#ifdef USE_ABSOLUTE_ODOM_FOR_ALL
    body_pos_des_ = pos_cmd_rel_;
#else
    body_pos_des_ = pos_cmd_;
#endif
    body_vel_des_.setZero();
    body_acc_des_.setZero();
    body_rpy_des_.setZero();
    body_omg_des_.setZero();

    for ( int i = 0; i < 4; i++ ) {
        foot_pos_des_[ i ].setZero();
        foot_vel_des_[ i ].setZero();
        foot_acc_des_[ i ].setZero();
        foot_force_des_[ i ] << 0.0, 0.0, 70.0;
    }

    current_gait_          = gait_cmd_;
    gait_check_transition_ = false;
    gait_allow_transition_ = true;
    ban_exit_loco_         = false;
    vel_transition_        = 0.2;

    contact_states_.setOnes();
    swing_states_.setZero();

    floating_base_weight_wbc_.setZero();

    push_recovery_flag_ = false;
    mu_for_wbc_         = 0.4;
    mu_for_mpc_         = mu_for_wbc_;

    kp_body_for_wbc_.setConstant( 60.0 );
    kd_body_for_wbc_.setConstant( 6.0 );
    kp_ori_for_wbc_.setConstant( 60.0 );
    kd_ori_for_wbc_.setConstant( 6.0 );
    kp_foot_for_wbc_.setConstant( 300.0 );
    kd_foot_for_wbc_.setConstant( 36.0 );
    kp_joint_for_wbc_.setConstant( 3.0 );
    kd_joint_for_wbc_.setConstant( 1.0 );

    gait_period_ = 10;

    transform_last_steps_ = false;
    transform_first_run_  = true;

    solver_mpc_interface_ = new SolveMpcInterface();

    solve_mpc_by_single_thread_ = false;
    start_solve_mpc_            = false;
}

/**
 * @brief Destroy the Convex Mpc Loco Gaits:: Convex Mpc Loco Gaits object
 *
 */
ConvexMpcLocoGaits::~ConvexMpcLocoGaits() {
    delete solver_mpc_interface_;
}

/**
 * @brief Initialize all variables
 *
 * @param data robot data
 */
void ConvexMpcLocoGaits::Initialize( ControlFsmData< float >& data ) {
    user_params_     = data.user_parameters;
    robot_params_    = data.control_parameters;
    state_estimator_ = data.state_estimator;
    ctrl_cmd_        = data.command;
    state_est_       = data.state_estimator->GetResult();
    robot_model_     = data.quadruped;
    gait_type_       = &trot_10_5_;
    leg_ctrl_        = data.leg_controller;

    se_ori_cali_offset_ = Vec3< float >::Zero();
    se_ori_cali_gain_   = Vec3< float >::Zero();

    dev_mode_scale_ = 1.0;

    gait_cmd_is_passive_trot_   = false;
    trot_gait_                  = false;
    auto_switch_gait_           = false;
    auto_switch_enabled_        = ( user_params_->auto_switch_enabled > 0.1 ) ? true : false;
    push_recovery_enabled_      = false;
    push_recovery_flag_stand_   = false;
    push_recovery_flag_trot_    = false;
    push_recovery_vel_update_   = true;
    push_recovery_trigger_iter_ = 0;

    vel_max_ratio_( 0 ) = user_params_->vel_max_ratio[ 0 ];
    vel_max_ratio_( 1 ) = user_params_->vel_max_ratio[ 1 ];
    vel_max_ratio_( 2 ) = user_params_->vel_max_ratio[ 2 ];

    for ( int i = 0; i < TROT_AUTO_SWITCH_NUM; i++ ) {
        vel_max_for_switch_[ i ] = Vec3< double >::Zero();
        vel_min_for_switch_[ i ] = Vec3< double >::Zero();
    }
    vel_max_for_switch_cmd_.setConstant( 1.0 );
    vel_min_for_switch_cmd_.setConstant( -1.0 );
    gait_id_all_ = gait_id_all_auto_switch_;

    gait_step_iter_ = 0;
    loco_iter_      = 0;
    gait_iter_      = 0;

    gait_first_step_ = true;
    gait_first_run_  = true;

    step_height_scale_ = 1.0;
    step_height_des_   = step_height_cmd_;
    step_height_max_   = 0.08;
    step_height_min_   = 0.00;

    vel_des_.setZero();
    vel_des_robot_ = vel_des_;
    vel_des_last_  = vel_des_;
    vel_cmd_scale_.setOnes();
    vel_cmd_max_ << 2.0, 0.5, 0.5;
    vel_cmd_min_ = -1.0f * vel_cmd_max_;
    acc_cmd_scale_.setOnes();
    acc_cmd_max_ << 1.0, 1.0, 2.0;
    acc_cmd_min_      = -1.0f * acc_cmd_max_;
    vel_cmd_max_last_ = vel_cmd_max_;
    vel_cmd_min_last_ = vel_cmd_min_;
    vel_cmd_zero_offset_.setZero();
#ifdef USE_ABSOLUTE_ODOM_FOR_ALL
    pos_cmd_rel_ << 0.0, 0.0, user_params_->des_roll_pitch_height[ 2 ];
    pos_des_rel_ = pos_cmd_rel_;
    pos_des_rel_last_ << state_est_.position( 0 ), state_est_.position( 1 ), state_est_.height;
    pos_cmd_rel_scale_.setOnes();
    pos_cmd_rel_max_ << 0.0, 0.0, user_params_->des_roll_pitch_height[ 2 ] + 0.07;  // Only z is enabled
    pos_cmd_rel_min_ << 0.0, 0.0, 0.23;
    pos_des_ << 0.0, 0.0, user_params_->des_roll_pitch_height[ 2 ] + state_est_.position( 2 ) - state_est_.height;
#else
    pos_cmd_ << 0.0, 0.0, user_params_->des_roll_pitch_height[ 2 ];
    pos_des_      = pos_cmd_;
    pos_des_last_ = state_est_.position;
    pos_cmd_max_ << 0.0, 0.0, user_params_->des_roll_pitch_height[ 2 ] + 0.07;  // Only z is enabled
    pos_cmd_min_ << 0.0, 0.0, 0.23;
#endif
    init_pos_des_ << 0.0, 0.0, 0.25;
    init_rpy_des_.setZero();
    rpy_des_ = rpy_cmd_;
    rpy_cmd_scale_.setOnes();  // Only pitch is enabled
    rpy_cmd_max_ << 0.0, 0.2, 0.0;
    rpy_cmd_min_ = -1.0f * rpy_cmd_max_;
    vel_est_avg_.setZero();
    vel_est_sum_.setZero();

    yaw_absolute_ = false;

    for ( int i = 0; i < 3; i++ ) {
        vel_buffer_in_x_[ i ]    = 0.0;
        vel_buffer_in_y_[ i ]    = 0.0;
        vel_buffer_in_yaw_[ i ]  = 0.0;
        vel_buffer_out_x_[ i ]   = 0.0;
        vel_buffer_out_y_[ i ]   = 0.0;
        vel_buffer_out_yaw_[ i ] = 0.0;
    }

    yaw_integral_              = 0.0;
    yaw_des_last_              = 0.0;
    body_height_delta_max_     = 0.03;
    body_height_delta_vel_max_ = 1.6;
    body_height_filter_        = 0.01;

    rpy_integral_.setZero();
    rpy_comp_.setZero();

    for ( int i = 0; i < 4; i++ ) {
        foot_pos_feedback_[ i ] = state_est_.position + state_est_.world2body_rotation_matrix.transpose() * ( robot_model_->GetHipLocation( i ) + leg_ctrl_->datas_[ i ].p );
        foot_first_swing_[ i ]  = true;
    }

    swing_time_ << 0.0, 0.0, 0.0, 0.0;
    stance_time_ << 1.0, 1.0, 1.0, 1.0;
    swing_time_remain_ << 0.0, 0.0, 0.0, 0.0;
    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ].setZero();
        landing_pos_[ i ].setZero();
    }
    landing_pos_ratio_.setOnes();
    landing_pos_ratio_last_ = landing_pos_ratio_;
    landing_gain_params_.setZero();
    landing_pos_offset_scale_ = 1.0;

    se_contact_states_.setZero();
    se_contact_states_stand_.setConstant( 0.5 );
    trans_state_ = 0.01;

    kd_joint_for_mpc_.setZero();
    kp_leg_swing_for_mpc_.setZero();
    kd_leg_swing_for_mpc_.setZero();
    kp_leg_stance_for_mpc_.setZero();
    kd_leg_stance_for_mpc_.setZero();
    kp_leg_stance_for_mpc_wbc_.setZero();
    kd_leg_stance_for_mpc_wbc_.setZero();

    for ( int i = 0; i < 4; i++ ) {
        foot_force_leg_result_[ i ] << 0.0, 0.0, data.user_parameters->mpc_body_mass * 9.81 / 4;
        foot_force_result_[ i ] << 0.0, 0.0, data.user_parameters->mpc_body_mass * 9.81 / 4;
        foot_force_des_[ i ] << 0.0, 0.0, data.user_parameters->mpc_body_mass * 9.81 / 4;
    }

    max_pos_err_mpc_.setConstant( 0.1 );

    comp_integral_   = 0;
    x_comp_integral_ = 0;
    y_comp_integral_ = 0;

    // traj_all_;
    opt_trajectory_pre_.setZero();
    opt_trajectory_pre_( 3 ) = state_est_.position( 0 );
    opt_trajectory_pre_( 4 ) = state_est_.position( 1 );
    opt_trajectory_pre_( 5 ) = state_est_.position( 2 );
    opt_trajectory_next_.setZero();
    opt_trajectory_next_( 3 ) = state_est_.position( 0 );
    opt_trajectory_next_( 4 ) = state_est_.position( 1 );
    opt_trajectory_next_( 5 ) = state_est_.position( 2 );
    interpolate_count_        = 1;
    interpolate_max_num_      = 14;  // 9;

    rpy_des_with_comp_.setZero();
    est_terrain_coef_.setConstant( 1.0 );
    est_terrain_rot_matrix_.setZero();
    enable_terrain_comp_        = false;
    slope_up_threshold_         = 0.1;
    slope_down_threshold_       = 0.1;
    slope_threshold_num_        = 0;
    heading_direction_on_slope_ = 1;
    slope_                      = 0.0;

    walk_direction_ = 0;

    robot_type_ = data.quadruped->robot_type_;

    use_quick_break_mode_ = true;

    // // DEBUG
    // std::cout << "ConvexMpcLocoGaits initialization finished." << std::endl;
}

template <> void ConvexMpcLocoGaits::RunMpc( ControlFsmData< float >& data ) {
    SetCurrentGait();

    SetGaitParams( current_gait_ );
    gait_type_->SetIterations( iter_between_mpc_, gait_iter_ );
    AccumulateIteration();

    SetCmd( data );
    CheckGaitFirstStep();

    SetBodyWorldDesire();
    GetFootPosFeedback();
    if ( gait_first_run_ ) {
        SetGaitFirstRunVars();
    }
#ifdef USE_TERRAIN_DETECTER
    if ( ( current_gait_ >= GaitId::kTrotMedium && current_gait_ <= GaitId::kTrot10v4 ) || ( current_gait_ >= GaitId::kTrot10v5 && current_gait_ <= GaitId::kTrot20v12Follow ) )
        SetTerrainComp();
    else
        enable_terrain_comp_ = false;
#endif
    SetRpyComp();
    SetLandingPos();
    CheckGaitTransition();
    ComputeVelAverage();
    ComputeMpc();

    SetLegKpKd();
    SetFootDesire();
    state_estimator_->SetContactPhase( se_contact_states_ );

    SetBodyDesireTrajectory();

#if ( defined DRAW_DEBUG_SWINGS ) || ( defined DRAW_DEBUG_PATH )
    DrawResult( data );
#endif
}

void ConvexMpcLocoGaits::RunStand() {
    SetCurrentGait();

    SetGaitParams( current_gait_ );
    AccumulateIteration();

    CheckGaitFirstStep();

    GetFootPosFeedback();
    if ( gait_first_run_ ) {
        SetGaitFirstRunVars();
    }
    CheckGaitTransition();
    ComputeVelAverage();

    SetLegKpKd();
    state_estimator_->SetContactPhase( se_contact_states_stand_ );

    SetBodyDesireTrajectory();
}

void ConvexMpcLocoGaits::SetCurrentGait() {
    state_est_ = state_estimator_->GetResult();
    if ( !transform_last_steps_ ) {
        SetPushRecoveryFlag();
        gait_cmd_is_passive_trot_ = false;
        if ( ctrl_cmd_->gait_id == GaitId::kPassiveTrot )
            PassiveTrotAutoSwitch();
        else if ( ctrl_cmd_->gait_id == GaitId::kWalk )
            WalkAutoSwitch();
        else
            TrotAutoSwitch();
    }
    else
        ExitLocomotionAutoSwitch();

    current_gait_ = gait_cmd_;
}

void ConvexMpcLocoGaits::AccumulateIteration() {
    loco_iter_++;
    gait_iter_++;
}

void ConvexMpcLocoGaits::SetPushRecoveryFlag() {
    static bool          vel_norm_large            = true;  // when vel norm is too large, push recovery disables
    static Vec3< float > body_vel_average          = Vec3< float >::Zero();
    static int           time_for_average          = 10;
    static float         enlarge_push_recovery_vel = 1.5;  // enlarge push_recovery_vel_trot without delay

    // Compute vel average for push recovery
    body_vel_average = ( body_vel_average * ( time_for_average - 1 ) + state_est_.velocity_in_body_frame ) / time_for_average;

    // Large vel norm disables push recovery
    // vel_cmd is not limited by min and max, larger than trot_12_6 limitatioin disables push recovery
    if ( fabs( vel_cmd_( 0 ) ) >= user_params_->vel_xy_yaw_max_trot_12_6[ 0 ] || fabs( vel_cmd_( 1 ) ) >= user_params_->vel_xy_yaw_max_trot_12_6[ 1 ]
         || fabs( vel_cmd_( 2 ) ) >= user_params_->vel_xy_yaw_max_trot_12_6[ 2 ] ) {
        vel_norm_large = true;
    }
    else {
        vel_norm_large = false;
    }

    if ( ctrl_cmd_->gait_id != GaitId::kStand && ctrl_cmd_->gait_id != GaitId::kStandNoPr && ctrl_cmd_->gait_id != GaitId::kStandPassive ) {
        push_recovery_flag_stand_ = false;
        // Set push_recovery_flag_trot_
        // Trigger when x y vel have large error
        // Average velocity condition eliminates the steady error of velocity
        // Average velocity condition must be compatible with auto_switch
        // Delay a few seconds after entering a new gait and enlarge threshold without delvy
        if ( ( ( ( ( fabs( vel_des_robot_( 0 ) - body_vel_average[ 0 ] ) > user_params_->enable_push_recovery_vel_trot[ 0 ]
                     && fabs( vel_est_avg_( 0 ) - body_vel_average[ 0 ] ) > user_params_->enable_push_recovery_vel_trot[ 0 ] )
                   || ( fabs( vel_des_robot_( 1 ) - body_vel_average[ 1 ] ) > user_params_->enable_push_recovery_vel_trot[ 1 ]
                        && fabs( vel_est_avg_( 1 ) - body_vel_average[ 1 ] ) > user_params_->enable_push_recovery_vel_trot[ 1 ] ) )
                 && gait_iter_ > user_params_->enable_push_recovery_delay )
               || ( ( fabs( vel_des_robot_( 0 ) - body_vel_average[ 0 ] ) > enlarge_push_recovery_vel * user_params_->enable_push_recovery_vel_trot[ 0 ]
                      && fabs( vel_est_avg_( 0 ) - body_vel_average[ 0 ] ) > enlarge_push_recovery_vel * user_params_->enable_push_recovery_vel_trot[ 0 ] )
                    || ( fabs( vel_des_robot_( 1 ) - body_vel_average[ 1 ] ) > enlarge_push_recovery_vel * user_params_->enable_push_recovery_vel_trot[ 1 ]
                         && fabs( vel_est_avg_( 1 ) - body_vel_average[ 1 ] ) > enlarge_push_recovery_vel * user_params_->enable_push_recovery_vel_trot[ 1 ] ) ) )
             && !vel_norm_large && push_recovery_enabled_ ) {
            push_recovery_flag_trot_ = true;
        }
        else if ( ( fabs( vel_des_robot_( 0 ) - body_vel_average[ 0 ] ) < user_params_->disable_push_recovery_vel_trot[ 0 ]
                    && fabs( vel_des_robot_( 0 ) - vel_est_avg_( 0 ) ) < user_params_->disable_push_recovery_vel_trot[ 0 ]
                    && fabs( vel_des_robot_( 1 ) - body_vel_average[ 1 ] ) < user_params_->disable_push_recovery_vel_trot[ 1 ]
                    && fabs( vel_des_robot_( 1 ) - vel_est_avg_( 1 ) ) < user_params_->disable_push_recovery_vel_trot[ 1 ] && fabs( vel_est_avg_( 0 ) ) < vel_max_ratio_( 0 ) * vel_cmd_max_last_( 0 )
                    && fabs( vel_est_avg_( 1 ) ) < vel_max_ratio_( 1 ) * vel_cmd_max_last_( 1 ) && fabs( vel_est_avg_( 2 ) ) < vel_max_ratio_( 2 ) * vel_cmd_max_last_( 2 )
                    && gait_iter_ > push_recovery_trigger_iter_ + user_params_->disable_push_recovery_delay )
                  || vel_norm_large || !push_recovery_enabled_ ) {
            push_recovery_flag_trot_ = false;
        }
    }
    else {
        push_recovery_flag_trot_ = false;
        // Set push_recovery_flag_stand_
        // Trigger when x y vel have large error or x y pos have large error and current_gait_ is stand
        // pos_des_ is init_pos_des_ when current_gait_ is STAND
        if ( ( ( fabs( vel_des_robot_( 0 ) - body_vel_average[ 0 ] ) > user_params_->enable_push_recovery_vel_stand[ 0 ]
                 && fabs( vel_est_avg_( 0 ) - body_vel_average[ 0 ] ) > user_params_->enable_push_recovery_vel_stand[ 0 ] )
               || ( fabs( vel_des_robot_( 1 ) - body_vel_average[ 1 ] ) > user_params_->enable_push_recovery_vel_stand[ 1 ]
                    && fabs( vel_est_avg_( 1 ) - body_vel_average[ 1 ] ) > user_params_->enable_push_recovery_vel_stand[ 1 ] )
               || ( ( fabs( pos_des_( 0 ) - state_est_.position( 0 ) ) > user_params_->enable_push_recovery_pos_stand[ 0 ]
                      || fabs( pos_des_( 1 ) - state_est_.position( 1 ) ) > user_params_->enable_push_recovery_pos_stand[ 1 ] ) ) )
             && current_gait_ == GaitId::kStand && gait_iter_ > user_params_->enable_push_recovery_delay && push_recovery_enabled_ ) {
            push_recovery_flag_stand_ = true;
        }
        else if ( ( fabs( vel_des_robot_( 0 ) - body_vel_average[ 0 ] ) < user_params_->disable_push_recovery_vel_stand[ 0 ]
                    && fabs( vel_des_robot_( 0 ) - vel_est_avg_( 0 ) ) < user_params_->disable_push_recovery_vel_stand[ 0 ]
                    && fabs( vel_des_robot_( 1 ) - body_vel_average[ 1 ] ) < user_params_->disable_push_recovery_vel_stand[ 1 ]
                    && fabs( vel_des_robot_( 1 ) - vel_est_avg_( 1 ) ) < user_params_->disable_push_recovery_vel_stand[ 1 ]
                    && gait_iter_ > push_recovery_trigger_iter_ + user_params_->disable_push_recovery_delay )
                  || !push_recovery_enabled_ ) {
            push_recovery_flag_stand_ = false;
        }
    }

    // Set push_recovery_flag_
    if ( push_recovery_flag_trot_ || push_recovery_flag_stand_ ) {
        push_recovery_flag_ = true;
    }
    else if ( !push_recovery_flag_trot_ && !push_recovery_flag_stand_ ) {
        push_recovery_flag_ = false;
    }

    // // DEBUG
    // std::cout << "push_recovery_flag_trot_:  " << push_recovery_flag_trot_ << std::endl;
    // std::cout << "push_recovery_flag_stand_: " << push_recovery_flag_stand_ << std::endl;
    // std::cout << "push_recovery_flag_:       " << push_recovery_flag_ << std::endl;
}

void ConvexMpcLocoGaits::TrotAutoSwitch() {
    static int   push_recovery_iter      = 0;
    static float vel_enter_stand_thresh  = 0.15;  // a treshold for entering STAND
    static bool  decelerate_beofre_stand = false;
    static float local_foot_pos_x_sum    = 0.0;  // guarantee foot land at regular pos before STAND
    static float local_foot_pos_y_sum    = 0.0;
    static int   loco_iter_first_trans   = 0;  // guarantee SwitchByAbsolute() only execute onece every gait period

    // set auto_switch_gait_
    if ( ctrl_cmd_->gait_id == GaitId::kTrot24v16 || ctrl_cmd_->gait_id == GaitId::kTrot20v12Follow || ctrl_cmd_->gait_id == GaitId::kTrotAuto ) {
        auto_switch_gait_ = true;
    }
    else {
        auto_switch_gait_ = false;
    }

    // set trot/stand auto switch
    if ( push_recovery_flag_ ) {
        gait_cmd_ = GaitId::kTrot10v5;

        pos_des_( 0 ) = state_est_.position( 0 );  // no pos err during push recovery
        pos_des_( 1 ) = state_est_.position( 1 );
        rpy_des_( 2 ) = state_est_.rpy( 2 );

        push_recovery_vel_update_ = false;

        if ( gait_cmd_ != current_gait_ ) {
            // make swing phase continuous when switch gait
            // "10" is gait_period_ of GaitId::kTrot10v5
            gait_iter_ = ( int )( ( ( float )( gait_iter_ % ( iter_between_mpc_ * gait_period_ ) ) / ( float )( iter_between_mpc_ * gait_period_ ) ) * ( iter_between_mpc_ * 10 ) ) + 1;
            push_recovery_trigger_iter_ = gait_iter_;
            std::cout << "[ConvexMpcLocoGaits] Push Recovery is enabled !!!" << std::endl;
        }
        push_recovery_iter++;
    }
    else {
        if ( push_recovery_iter >= 1 ) {
            std::cout << "[ConvexMpcLocoGaits] Push Recovery lasts " << 2.0 * push_recovery_iter << " ms" << std::endl;
            push_recovery_iter = 0;
        }

        if ( gait_check_transition_ ) {
            if ( !( auto_switch_gait_ && auto_switch_enabled_ ) && ctrl_cmd_->gait_id != GaitId::kStand ) {
                gait_cmd_ = ctrl_cmd_->gait_id;
            }
            // STAND must enter with zero velocity command, STANDNOPR doesn't consider that
            else if ( ctrl_cmd_->gait_id == GaitId::kStand ) {
                // to judge whether the pos of foot is too far from zero pos in local frame
                local_foot_pos_x_sum = 0.0;
                local_foot_pos_y_sum = 0.0;
                for ( size_t i = 0; i < 4; i++ ) {
                    local_foot_pos_x_sum += fabs( leg_ctrl_->datas_[ i ].p[ 0 ] - landing_pos_offset_[ i ]( 0 ) );
                    local_foot_pos_y_sum += fabs( leg_ctrl_->datas_[ i ].p[ 1 ] - landing_pos_offset_[ i ]( 1 ) );
                }
                if ( vel_est_avg_.norm() > vel_enter_stand_thresh || local_foot_pos_x_sum > 0.1 ) {
                    gait_cmd_ = current_gait_;
                    if ( !decelerate_beofre_stand ) {
                        std::cout << "[ConvexMpcLocoGaits] Decelerating before entering STAND " << std::endl;
                    }
                    decelerate_beofre_stand = true;
                }
                else {
                    gait_cmd_ = ctrl_cmd_->gait_id;
                    if ( decelerate_beofre_stand ) {
                        std::cout << "[ConvexMpcLocoGaits] Finished deceleration, entering STAND" << std::endl;
                    }
                    decelerate_beofre_stand = false;
                }
            }
            // Auto switch
            else {
                // compute when current_gait_ is not auto switch gait
                if ( loco_iter_ == loco_iter_first_trans + 1 || SearchGaitIdAutoSwitch( current_gait_, gait_id_all_ ) == TROT_AUTO_SWITCH_NUM - 1 ) {
                    // SwitchByPercentage();
                    // SwitchByAbsolute();
                    // SwitchByAbsoluteAndGaitCmd();
                    if ( ctrl_cmd_->gait_id == GaitId::kTrotAuto ) {
                        SwitchByAbsoluteAndGaitCmd();
                    }
                    else {
                        SwitchByAbsolute();
                    }
                }
            }

            // Reset iter when gait changes
            if ( gait_cmd_ != current_gait_ ) {
                ResetIter();

                if ( auto_switch_gait_ ) {
                    std::cout << "[ConvexMpcLocoGaits] Auto switch to GaitId: " << gait_cmd_ << " in trot mode" << std::endl;
                }
            }
            // Record last vel_cmd_max_/min_ to make vel_cmd continuous
            vel_cmd_max_last_         = vel_cmd_max_;
            vel_cmd_min_last_         = vel_cmd_min_;
            push_recovery_vel_update_ = true;
        }
        else {
            loco_iter_first_trans = loco_iter_;
        }
    }

    // // DEBUG
    // std::cout << "gait_cmd_: " << gait_cmd_ << std::endl;
    // std::cout << "ctrl_cmd_->gait_id: " << (int)ctrl_cmd_->gait_id << std::endl;
    // std::cout << "gait_iter_: " << gait_iter_ << std::endl;
    // std::cout << "vel_cmd_max_last_: " << vel_cmd_max_last_.transpose() << std::endl;
}

void ConvexMpcLocoGaits::PassiveTrotAutoSwitch() {
    static Vec3< double > vel_max_thresh_up_, vel_max_thresh_down_;
    static Vec3< float >  pose_threshold( 0.015, 0.018, 0.15 );  // x y yaw
    vel_max_thresh_up_ << 0.08, 0.08, 0.3;
    vel_max_thresh_down_ << 0.0, 0.0, 0.0;
    static int64_t passive_trot_hold_iter = 0;

    if ( gait_check_transition_ ) {
        if ( ctrl_cmd_->gait_id == GaitId::kPassiveTrot ) {
            float yaw_abs_error = fabs( rpy_des_( 2 ) - state_est_.rpy( 2 ) );
            yaw_abs_error       = yaw_abs_error > M_PI ? ( 2 * M_PI - yaw_abs_error ) : yaw_abs_error;
            if ( ( current_gait_ == GaitId::kStandPassive && abs( vel_est_avg_( 0 ) ) < 0.5 * ( vel_max_thresh_up_[ 0 ] + vel_max_thresh_down_[ 0 ] )
                   && abs( vel_est_avg_( 1 ) ) < 0.5 * ( vel_max_thresh_up_[ 1 ] + vel_max_thresh_down_[ 1 ] )
                   && abs( vel_est_avg_( 2 ) ) < 0.5 * ( vel_max_thresh_up_[ 2 ] + vel_max_thresh_down_[ 2 ] )
                   && ( pow( ( pos_des_( 0 ) - state_est_.position( 0 ) ) / pose_threshold( 0 ), 2 ) + pow( ( pos_des_( 1 ) - state_est_.position( 1 ) ) / pose_threshold( 1 ), 2 ) <= 1.0 )
                   && ( yaw_abs_error < pose_threshold( 2 ) ) )
                 || ( abs( vel_est_avg_( 0 ) ) < vel_max_thresh_up_[ 0 ] && abs( vel_est_avg_( 1 ) ) < vel_max_thresh_up_[ 1 ] && abs( vel_est_avg_( 2 ) ) < vel_max_thresh_up_[ 2 ]
                      && passive_trot_hold_iter > 360 ) ) {
                gait_cmd_ = GaitId::kStandPassive;
            }
            else {
                gait_cmd_ = ctrl_cmd_->gait_id;
            }
            // std::cout << "[check position error] " << pos_des_( 0 ) - state_est_.position( 0 ) << "  " << pos_des_( 1 ) - state_est_.position( 1 ) << std::endl;
            // std::cout << "[check yaw error] " << rpy_des_( 2 ) << "  " << state_est_.rpy( 2 ) << "  " << yaw_abs_error << std::endl;
        }
    }
    if ( gait_cmd_ == GaitId::kPassiveTrot ) {
        passive_trot_hold_iter++;
        gait_cmd_is_passive_trot_ = true;
    }
    else
        passive_trot_hold_iter = 0;
}

void ConvexMpcLocoGaits::SwitchByPercentage() {
    static Vec3< float > vel_cmd_max_auto_switch = Vec3< float >::Ones();
    static float         vel_cmd_weight_norm     = 0.0;
    static Vec3< float > vel_cmd_weight          = Vec3< float >::Ones();

    SetVelMaxMinForSwitch();

    vel_cmd_weight << 1.0, 1.0, 2.0;
    vel_cmd_weight_norm = 0.0;

    // set vel_cmd_max_, in case vel_max/min are not symmetric
    for ( int i = 0; i < 3; i++ ) {
        if ( vel_cmd_( i ) >= 0.0 ) {
            vel_cmd_max_auto_switch( i ) = user_params_->vel_xy_yaw_max_trot_8_3[ i ];
        }
        else {
            vel_cmd_max_auto_switch( i ) = user_params_->vel_xy_yaw_min_trot_8_3[ i ];
        }
    }
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_weight_norm += vel_cmd_weight( i ) * pow( vel_cmd_( i ) / ( fabs( vel_cmd_max_auto_switch( i ) ) >= 1e-3 ? vel_cmd_max_auto_switch( i ) : 1.0 ), 2 );
    }
    vel_cmd_weight_norm = sqrt( vel_cmd_weight_norm );
    // Deadzone for switching
    // vel_robot_des is continuous, vel_cmd_ is discrete
    // vel_cmd_ / vel_cmd_max_ is used since different vel_cmd_max_ of each gait
    // Cyberdog kTrot10v4 is better but Cyberdog2 is reversed
    if ( vel_cmd_weight_norm >= user_params_->vel_switch( 7 ) || ( vel_cmd_weight_norm >= user_params_->vel_switch( 7 ) - user_params_->vel_switch( 11 ) && gait_cmd_ == GaitId::kTrot8v3 ) ) {
        gait_cmd_ = gait_id_all_[ 0 ];
    }
    else if ( vel_cmd_weight_norm >= user_params_->vel_switch( 6 ) || ( vel_cmd_weight_norm >= user_params_->vel_switch( 6 ) - user_params_->vel_switch( 11 ) && gait_cmd_ == GaitId::kTrot10v5 ) ) {
        gait_cmd_ = GaitCmdSelect( gait_id_all_[ 1 ], current_gait_, vel_max_for_switch_[ 1 ], vel_min_for_switch_[ 1 ] );
    }
    else if ( vel_cmd_weight_norm >= user_params_->vel_switch( 5 ) || ( vel_cmd_weight_norm >= user_params_->vel_switch( 5 ) - user_params_->vel_switch( 11 ) && gait_cmd_ == GaitId::kTrot12v6 ) ) {
        gait_cmd_ = GaitCmdSelect( gait_id_all_[ 2 ], current_gait_, vel_max_for_switch_[ 2 ], vel_min_for_switch_[ 2 ] );
    }
    else if ( vel_cmd_weight_norm >= user_params_->vel_switch( 4 ) || ( vel_cmd_weight_norm >= user_params_->vel_switch( 4 ) - user_params_->vel_switch( 11 ) && gait_cmd_ == GaitId::kTrot14v8 ) ) {
        gait_cmd_ = GaitCmdSelect( gait_id_all_[ 3 ], current_gait_, vel_max_for_switch_[ 3 ], vel_min_for_switch_[ 3 ] );
    }
    else if ( vel_cmd_weight_norm >= user_params_->vel_switch( 3 ) || ( vel_cmd_weight_norm >= user_params_->vel_switch( 3 ) - user_params_->vel_switch( 11 ) && gait_cmd_ == GaitId::kTrot16v10 ) ) {
        gait_cmd_ = GaitCmdSelect( gait_id_all_[ 4 ], current_gait_, vel_max_for_switch_[ 4 ], vel_min_for_switch_[ 4 ] );
    }
    else if ( vel_cmd_weight_norm >= user_params_->vel_switch( 2 ) || ( vel_cmd_weight_norm >= user_params_->vel_switch( 2 ) - user_params_->vel_switch( 11 ) && gait_cmd_ == GaitId::kTrot18v11 ) ) {
        gait_cmd_ = GaitCmdSelect( gait_id_all_[ 5 ], current_gait_, vel_max_for_switch_[ 5 ], vel_min_for_switch_[ 5 ] );
    }
    else if ( vel_cmd_weight_norm >= user_params_->vel_switch( 1 ) || ( vel_cmd_weight_norm >= user_params_->vel_switch( 1 ) - user_params_->vel_switch( 11 ) && gait_cmd_ == GaitId::kTrot20v12 ) ) {
        gait_cmd_ = GaitCmdSelect( gait_id_all_[ 6 ], current_gait_, vel_max_for_switch_[ 6 ], vel_min_for_switch_[ 6 ] );
    }
    else if ( vel_cmd_weight_norm >= user_params_->vel_switch( 0 ) || ( vel_cmd_weight_norm >= user_params_->vel_switch( 0 ) - user_params_->vel_switch( 11 ) && gait_cmd_ == GaitId::kTrot22v14 ) ) {
        gait_cmd_ = GaitCmdSelect( gait_id_all_[ 7 ], current_gait_, vel_max_for_switch_[ 7 ], vel_min_for_switch_[ 7 ] );
    }
    else {
        gait_cmd_ = GaitCmdSelect( gait_id_all_[ 8 ], current_gait_, vel_max_for_switch_[ 8 ], vel_min_for_switch_[ 8 ] );
    }
}

void ConvexMpcLocoGaits::SwitchByAbsolute() {
    SetVelMaxMinForSwitch();

    if ( VelVectorCompare( gait_id_all_[ 0 ], vel_cmd_, vel_max_for_switch_[ 1 ], vel_max_for_switch_[ 2 ], vel_min_for_switch_[ 1 ], vel_min_for_switch_[ 2 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 0 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else if ( VelVectorCompare( gait_id_all_[ 1 ], vel_cmd_, vel_max_for_switch_[ 2 ], vel_max_for_switch_[ 3 ], vel_min_for_switch_[ 2 ], vel_min_for_switch_[ 3 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 1 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else if ( VelVectorCompare( gait_id_all_[ 2 ], vel_cmd_, vel_max_for_switch_[ 3 ], vel_max_for_switch_[ 4 ], vel_min_for_switch_[ 3 ], vel_min_for_switch_[ 4 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 2 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else if ( VelVectorCompare( gait_id_all_[ 3 ], vel_cmd_, vel_max_for_switch_[ 4 ], vel_max_for_switch_[ 5 ], vel_min_for_switch_[ 4 ], vel_min_for_switch_[ 5 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 3 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else if ( VelVectorCompare( gait_id_all_[ 4 ], vel_cmd_, vel_max_for_switch_[ 5 ], vel_max_for_switch_[ 6 ], vel_min_for_switch_[ 5 ], vel_min_for_switch_[ 6 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 4 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else if ( VelVectorCompare( gait_id_all_[ 5 ], vel_cmd_, vel_max_for_switch_[ 6 ], vel_max_for_switch_[ 7 ], vel_min_for_switch_[ 6 ], vel_min_for_switch_[ 7 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 5 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else if ( VelVectorCompare( gait_id_all_[ 6 ], vel_cmd_, vel_max_for_switch_[ 7 ], vel_max_for_switch_[ 8 ], vel_min_for_switch_[ 7 ], vel_min_for_switch_[ 8 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 6 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else if ( VelVectorCompare( gait_id_all_[ 7 ], vel_cmd_, vel_max_for_switch_[ 8 ], vel_max_for_switch_[ 9 ], vel_min_for_switch_[ 8 ], vel_min_for_switch_[ 9 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 7 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 8 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
}

void ConvexMpcLocoGaits::SetVelMaxMinForSwitch() {
    static Vec3< double > vel_max_zero = Vec3< double >::Zero();
    static Vec3< double > vel_min_zero = Vec3< double >::Zero();
    static Vec3< double > vel_max_inf  = Vec3< double >::Constant( 100.0 );
    static Vec3< double > vel_min_inf  = Vec3< double >::Constant( -100.0 );
    static bool           consider_yaw = true;
    static int            yaw_max_id   = GaitId::kTrot12v6;
    static int            gait_id_num  = 0;

    //  Follow gait starts from kTrot20v12Follow
    if ( ctrl_cmd_->gait_id == GaitId::kTrot20v12Follow ) {
        vel_max_for_switch_[ 0 ] = vel_max_inf;
        vel_max_for_switch_[ 1 ] = vel_max_inf;
        vel_max_for_switch_[ 2 ] = user_params_->vel_xy_yaw_max_trot_12_6;
        vel_max_for_switch_[ 3 ] = user_params_->vel_xy_yaw_max_trot_14_8;
        vel_max_for_switch_[ 4 ] = user_params_->vel_xy_yaw_max_trot_16_10;
        vel_max_for_switch_[ 5 ] = user_params_->vel_xy_yaw_max_trot_18_11;
        vel_max_for_switch_[ 6 ] = user_params_->vel_xy_yaw_max_trot_20_12;
        vel_max_for_switch_[ 7 ] = vel_max_zero;
        vel_max_for_switch_[ 8 ] = vel_max_zero;
        vel_max_for_switch_[ 9 ] = vel_max_zero;
        vel_min_for_switch_[ 0 ] = vel_min_inf;
        vel_min_for_switch_[ 1 ] = vel_min_inf;
        vel_min_for_switch_[ 2 ] = user_params_->vel_xy_yaw_min_trot_12_6;
        vel_min_for_switch_[ 3 ] = user_params_->vel_xy_yaw_min_trot_14_8;
        vel_min_for_switch_[ 4 ] = user_params_->vel_xy_yaw_min_trot_16_10;
        vel_min_for_switch_[ 5 ] = user_params_->vel_xy_yaw_min_trot_18_11;
        vel_min_for_switch_[ 6 ] = user_params_->vel_xy_yaw_min_trot_20_12;
        vel_min_for_switch_[ 7 ] = vel_min_zero;
        vel_min_for_switch_[ 8 ] = vel_min_zero;
        vel_min_for_switch_[ 9 ] = vel_min_zero;

        vel_max_for_switch_cmd_ = user_params_->vel_xy_yaw_max_trot_10_4;  // user_params_->vel_xy_yaw_max_trot_12_6;
        vel_min_for_switch_cmd_ = user_params_->vel_xy_yaw_min_trot_10_4;  // user_params_->vel_xy_yaw_min_trot_12_6;
        gait_id_all_            = gait_id_all_follow_;
    }
    else {
        vel_max_for_switch_[ 0 ] = vel_max_inf;  // user_params_->vel_xy_yaw_max_trot_8_3;
        vel_max_for_switch_[ 1 ] = vel_max_inf;  // user_params_->vel_xy_yaw_max_trot_10_4; // user_params_->vel_xy_yaw_max_trot_10_5;
        vel_max_for_switch_[ 2 ] = user_params_->vel_xy_yaw_max_trot_12_6;
        vel_max_for_switch_[ 3 ] = user_params_->vel_xy_yaw_max_trot_14_8;
        vel_max_for_switch_[ 4 ] = user_params_->vel_xy_yaw_max_trot_16_10;
        vel_max_for_switch_[ 5 ] = user_params_->vel_xy_yaw_max_trot_18_11;
        vel_max_for_switch_[ 6 ] = user_params_->vel_xy_yaw_max_trot_20_12;
        vel_max_for_switch_[ 7 ] = user_params_->vel_xy_yaw_max_trot_22_14;
        vel_max_for_switch_[ 8 ] = user_params_->vel_xy_yaw_max_trot_24_16;
        vel_max_for_switch_[ 9 ] = vel_max_zero;
        vel_min_for_switch_[ 0 ] = vel_min_inf;  // user_params_->vel_xy_yaw_min_trot_8_3;
        vel_min_for_switch_[ 1 ] = vel_min_inf;  // user_params_->vel_xy_yaw_min_trot_10_4; // user_params_->vel_xy_yaw_min_trot_10_5;
        vel_min_for_switch_[ 2 ] = user_params_->vel_xy_yaw_min_trot_12_6;
        vel_min_for_switch_[ 3 ] = user_params_->vel_xy_yaw_min_trot_14_8;
        vel_min_for_switch_[ 4 ] = user_params_->vel_xy_yaw_min_trot_16_10;
        vel_min_for_switch_[ 5 ] = user_params_->vel_xy_yaw_min_trot_18_11;
        vel_min_for_switch_[ 6 ] = user_params_->vel_xy_yaw_min_trot_20_12;
        vel_min_for_switch_[ 7 ] = user_params_->vel_xy_yaw_min_trot_22_14;
        vel_min_for_switch_[ 8 ] = user_params_->vel_xy_yaw_min_trot_24_16;
        vel_min_for_switch_[ 9 ] = vel_min_zero;

        vel_max_for_switch_cmd_ = user_params_->vel_xy_yaw_max_trot_10_4;  // user_params_->vel_xy_yaw_max_trot_8_3; // user_params_->vel_xy_yaw_max_trot_10_5;
        vel_min_for_switch_cmd_ = user_params_->vel_xy_yaw_min_trot_10_4;  // user_params_->vel_xy_yaw_min_trot_8_3; // user_params_->vel_xy_yaw_min_trot_10_5;
        gait_id_all_            = gait_id_all_auto_switch_;
    }

    // consider yaw or not
    if ( consider_yaw ) {
        gait_id_num = SearchGaitIdAutoSwitch( yaw_max_id, gait_id_all_ );
        for ( int i = 0; i < gait_id_num + 1; i++ ) {
            vel_max_for_switch_[ i ]( 2 ) = vel_max_inf( 2 );
            vel_min_for_switch_[ i ]( 2 ) = vel_min_inf( 2 );
        }
    }
    else {
        for ( int i = 0; i < TROT_AUTO_SWITCH_NUM; i++ ) {
            vel_max_for_switch_[ i ]( 2 ) = vel_max_inf( 2 );
            vel_min_for_switch_[ i ]( 2 ) = vel_min_inf( 2 );
        }
    }
}

bool ConvexMpcLocoGaits::VelVectorCompare( const int gait_id, Vec3< float > vel_cmd, Vec3< double > vel_max1, Vec3< double > vel_max2, Vec3< double > vel_min1, Vec3< double > vel_min2 ) {
    static bool result[ 3 ] = { true };

    // x y yaw
    for ( int i = 0; i < 3; i++ ) {
        if ( ( vel_cmd( i ) >= vel_max1( i ) || ( vel_cmd( i ) >= 0.5 * ( vel_max1( i ) + vel_max2( i ) ) && gait_cmd_ == gait_id ) )
             || ( vel_cmd( i ) <= vel_min1( i ) || ( vel_cmd( i ) <= 0.5 * ( vel_min1( i ) + vel_min2( i ) ) && gait_cmd_ == gait_id ) ) ) {
            result[ i ] = true;
        }
        else {
            result[ i ] = false;
        }
    }

    return ( result[ 0 ] || result[ 1 ] || result[ 2 ] );

    // DEBUG
    // std::cout << "result: " << result[ 0 ] << " " << result[ 1 ] << " " << result[ 2 ] << " " << std::endl;
}

int ConvexMpcLocoGaits::SearchGaitIdAutoSwitch( const int gait_id, const int gait_id_all[ TROT_AUTO_SWITCH_NUM ] ) {
    static int gait_id_num = 0;

    // search gait_id_num
    gait_id_num = 0;
    while ( gait_id_num <= TROT_AUTO_SWITCH_NUM - 1 ) {
        if ( gait_id_all[ gait_id_num ] == gait_id ) {
            break;
        }
        else {
            gait_id_num++;
        }

        // erro input
        if ( gait_id_num == TROT_AUTO_SWITCH_NUM ) {
            gait_id_num = TROT_AUTO_SWITCH_NUM - 1;
            // std::cout << "[ConvexMpcLocoGaits] Error in searching gait id for auto switch, " << gait_id << " is not in the list" << std::endl;
            break;
        }
    }

    return gait_id_num;
}

int ConvexMpcLocoGaits::GaitCmdSelect( const int gait_id, const int current_gait, Vec3< double > vel_max, Vec3< double > vel_min ) {
    static int           gait_cmd_result = 0;
    static Vec3< float > vel_ratio       = vel_max_ratio_;  // enlarge the vel_max/min for auto switch

    vel_ratio = vel_max_ratio_;
    // In case of sudden deceleration, transition delays a few steps
    if ( ( vel_est_avg_( 0 ) >= vel_ratio( 0 ) * vel_max( 0 ) || vel_est_avg_( 0 ) <= vel_ratio( 0 ) * vel_min( 0 ) )
         || ( vel_est_avg_( 1 ) >= vel_ratio( 1 ) * vel_max( 1 ) || vel_est_avg_( 1 ) <= vel_ratio( 1 ) * vel_min( 1 ) )
         || ( vel_est_avg_( 2 ) >= vel_ratio( 2 ) * vel_max( 2 ) || vel_est_avg_( 2 ) <= vel_ratio( 2 ) * vel_min( 2 ) ) ) {
        gait_cmd_result = current_gait;  // Keeps current gait if averge velocity is too high
    }
    else {
        gait_cmd_result = gait_id;
    }

    return gait_cmd_result;
}

int ConvexMpcLocoGaits::GaitCmdSelectFromAll( const int gait_id, const int gait_id_all[ TROT_AUTO_SWITCH_NUM ], Vec3< double > vel_max[ TROT_AUTO_SWITCH_NUM ],
                                              Vec3< double > vel_min[ TROT_AUTO_SWITCH_NUM ] ) {
    static Vec3< int >   gait_num_vector = Vec3< int >::Constant( TROT_AUTO_SWITCH_NUM - 2 );  // the lowest freq gait
    static int           gait_num_result = 0;
    static int           gait_id_select  = 0;
    static int           gait_id_num     = 0;
    static Vec3< float > vel_ratio       = vel_max_ratio_;  // enlarge the vel_max/min for auto switch
    static float         vel_hold        = 0.95;            // for hold current gait
    static int           freq_down_max   = 3;               // max gait differential with respect to last gait during deceleration

    gait_id_num = SearchGaitIdAutoSwitch( gait_id, gait_id_all );
    // vel_ratio = vel_max_ratio_;
    vel_ratio << 1.0, 1.0, 1.0;
    // search gait with lowest frequency
    for ( int i = 0; i < 3; i++ ) {
        gait_id_select = gait_id_num;
        while ( gait_id_select >= 0 ) {
            // in case of vel_est_avg_ fluctuates near vel_max[gait_id_select]
            // high frequency gaits don't decrease if vel_est_avg_ is larger than vel_hold * vel_max[gait_id_select]
            if ( ( vel_est_avg_( i ) <= vel_ratio( i ) * vel_max[ gait_id_select ]( i ) && vel_est_avg_( i ) >= vel_ratio( i ) * vel_min[ gait_id_select ]( i ) )
                 && !( gait_num_vector( i ) == gait_id_select - 1
                       && ( vel_est_avg_( i ) >= vel_hold * vel_ratio( i ) * vel_max[ gait_id_select ]( i ) || vel_est_avg_( i ) <= vel_hold * vel_ratio( i ) * vel_min[ gait_id_select ]( i ) ) ) ) {
                // in case of frequency changes too much between two gait steps
                if ( gait_id_select - gait_num_vector( i ) >= freq_down_max ) {
                    gait_num_vector( i ) = gait_num_vector( i ) + freq_down_max;
                }
                else {
                    gait_num_vector( i ) = gait_id_select;
                }
                break;
            }
            else {
                if ( gait_id_select == 0 ) {
                    gait_num_vector( i ) = 0;
                    break;
                }
                else {
                    gait_id_select--;
                }
            }
        }
    }
    // choose the highest frequency gait among x y yaw
    gait_num_result = std::min( gait_num_vector( 0 ), gait_num_vector( 1 ) );
    gait_num_result = std::min( gait_num_vector( 2 ), gait_num_result );

    return gait_id_all[ gait_num_result ];

    // // DEBUG
    // std::cout << "gait_id_num: " << gait_id_num << std::endl;
    // std::cout << "gait_id_select: " << gait_id_select << std::endl;
    // std::cout << "gait_num_vector: " << gait_num_vector.transpose() << std::endl;
}

void ConvexMpcLocoGaits::SwitchByAbsoluteAndGaitCmd() {
    SetVelMaxMinForSwitch();

    if ( VelVectorCompare( gait_id_all_[ 0 ], gait_cmd_, vel_cmd_, vel_max_for_switch_[ 1 ], vel_max_for_switch_[ 2 ], vel_min_for_switch_[ 1 ], vel_min_for_switch_[ 2 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 0 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else if ( VelVectorCompare( gait_id_all_[ 1 ], gait_cmd_, vel_cmd_, vel_max_for_switch_[ 2 ], vel_max_for_switch_[ 3 ], vel_min_for_switch_[ 2 ], vel_min_for_switch_[ 3 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 1 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else if ( VelVectorCompare( gait_id_all_[ 2 ], gait_cmd_, vel_cmd_, vel_max_for_switch_[ 3 ], vel_max_for_switch_[ 4 ], vel_min_for_switch_[ 3 ], vel_min_for_switch_[ 4 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 2 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else if ( VelVectorCompare( gait_id_all_[ 3 ], gait_cmd_, vel_cmd_, vel_max_for_switch_[ 4 ], vel_max_for_switch_[ 5 ], vel_min_for_switch_[ 4 ], vel_min_for_switch_[ 5 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 3 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else if ( VelVectorCompare( gait_id_all_[ 4 ], gait_cmd_, vel_cmd_, vel_max_for_switch_[ 5 ], vel_max_for_switch_[ 6 ], vel_min_for_switch_[ 5 ], vel_min_for_switch_[ 6 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 4 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else if ( VelVectorCompare( gait_id_all_[ 5 ], gait_cmd_, vel_cmd_, vel_max_for_switch_[ 6 ], vel_max_for_switch_[ 7 ], vel_min_for_switch_[ 6 ], vel_min_for_switch_[ 7 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 5 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else if ( VelVectorCompare( gait_id_all_[ 6 ], gait_cmd_, vel_cmd_, vel_max_for_switch_[ 7 ], vel_max_for_switch_[ 8 ], vel_min_for_switch_[ 7 ], vel_min_for_switch_[ 8 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 6 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else if ( VelVectorCompare( gait_id_all_[ 7 ], gait_cmd_, vel_cmd_, vel_max_for_switch_[ 8 ], vel_max_for_switch_[ 9 ], vel_min_for_switch_[ 8 ], vel_min_for_switch_[ 9 ] ) ) {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 7 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
    else {
        gait_cmd_ = GaitCmdSelectFromAll( gait_id_all_[ 8 ], gait_id_all_, vel_max_for_switch_, vel_min_for_switch_ );
    }
}

bool ConvexMpcLocoGaits::VelVectorCompare( const int gait_id, const int gait_cmd, Vec3< float > vel_cmd, Vec3< double > vel_max1, Vec3< double > vel_max2, Vec3< double > vel_min1,
                                           Vec3< double > vel_min2 ) {
    static bool result[ 3 ]       = { true };
    static int  gait_id_num       = 0;
    static int  gait_cmd_num      = 0;
    static int  freq_increase_max = 3;  // max gait differential with respect to last gait during acceleration

    gait_id_num  = SearchGaitIdAutoSwitch( gait_id, gait_id_all_ );
    gait_cmd_num = SearchGaitIdAutoSwitch( gait_cmd, gait_id_all_ );

    // x y yaw
    for ( int i = 0; i < 3; i++ ) {
        if ( ( gait_cmd_num - gait_id_num <= freq_increase_max )
             && ( ( vel_cmd( i ) >= vel_max1( i ) || ( vel_cmd( i ) >= 0.5 * ( vel_max1( i ) + vel_max2( i ) ) && gait_cmd_ == gait_id ) )
                  || ( vel_cmd( i ) <= vel_min1( i ) || ( vel_cmd( i ) <= 0.5 * ( vel_min1( i ) + vel_min2( i ) ) && gait_cmd_ == gait_id ) ) ) ) {
            result[ i ] = true;
        }
        else {
            result[ i ] = false;
        }
    }

    return ( result[ 0 ] || result[ 1 ] || result[ 2 ] );

    // DEBUG
    // std::cout << "gait_id_num: " << gait_id_num << std::endl;
    // std::cout << "gait_cmd_num: " << gait_cmd_num << std::endl;
    // std::cout << "result: " << result[ 0 ] << " " << result[ 1 ] << " " << result[ 2 ] << " " << std::endl;
}

void ConvexMpcLocoGaits::WalkAutoSwitch() {
    if ( gait_check_transition_ ) {
        gait_cmd_ = ctrl_cmd_->gait_id;
        // for walk autoswitch
        if ( walk_direction_ == 0 ) {
            if ( vel_cmd_[ 0 ] < -0.0 && vel_est_avg_( 0 ) < 0.005 )
                walk_direction_ = 1;
        }
        else if ( walk_direction_ == 1 ) {
            if ( vel_cmd_[ 0 ] > 0.0 && vel_est_avg_( 0 ) > -0.005 )
                walk_direction_ = 0;
        }
    }
}

void ConvexMpcLocoGaits::ExitLocomotionAutoSwitch() {
    gait_cmd_                 = GaitId::kTrot10v5;  // leave locoGait by trot_10_5
    landing_pos_offset_scale_ = 0.0;                // delete hip landing offset
    if ( transform_first_run_ ) {
        ResetIter();
        transform_first_run_ = false;
        std::cout << "[ConvexMpcLocoGaits] Auto switch to GaitId: " << gait_cmd_ << " before exiting Locomotion" << std::endl;
    }
}
void ConvexMpcLocoGaits::ResetIter() {
    // loco_iter_      = 0;
    gait_step_iter_ = 0;
    gait_iter_      = 0;

    // // DEBUG
    // std::cout << "Reset gait iteration." << std::endl;
}

void ConvexMpcLocoGaits::SetGaitParams( const int gait_id ) {
    SetDefaultParams();
    switch ( gait_id ) {
    case GaitId::kTrotMedium:
        SetTrotMediumParams();
        break;
    case GaitId::kTrotFast:
        SetTrotFastParams();
        break;
    case GaitId::kTrotSlow:
        SetTrotSlowParams();
        break;
    case GaitId::kBound:
        SetBoundParams();
        break;
    case GaitId::kPronk:
        SetPronkParams();
        break;
    case GaitId::kWalk:
        SetWalkParams();
        break;
    case GaitId::kPace:
        SetPaceParams();
        break;
    case GaitId::kPassiveTrot:
        SetPassiveTrotParams();
        break;
    case GaitId::kTrot24v16:
        SetTrot24v16Params();
        break;
    case GaitId::kTrot22v14:
        SetTrot22v14Params();
        break;
    case GaitId::kTrot20v12:
        SetTrot20v12Params();
        break;
    case GaitId::kTrot18v11:
        SetTrot18v11Params();
        break;
    case GaitId::kTrot16v10:
        SetTrot16v10Params();
        break;
    case GaitId::kTrot14v8:
        SetTrot14v8Params();
        break;
    case GaitId::kTrot12v6:
        SetTrot12v6Params();
        break;
    case GaitId::kTrot10v5:
        SetTrot10v5Params();
        break;
    case GaitId::kTrot10v4:
        SetTrot10v4Params();
        break;
    case GaitId::kTrot8v3:
        SetTrot8v3Params();
        break;
    case GaitId::kTrot24v16Follow:
        SetTrot24v16FollowParams();
        break;
    case GaitId::kTrot22v14Follow:
        SetTrot22v14FollowParams();
        break;
    case GaitId::kTrot20v12Follow:
        SetTrot20v12FollowParams();
        break;
    case GaitId::kTrot18v11Follow:
        SetTrot18v11FollowParams();
        break;
    case GaitId::kTrot16v10Follow:
        SetTrot16v10FollowParams();
        break;
    case GaitId::kTrot14v8Follow:
        SetTrot14v8FollowParams();
        break;
    case GaitId::kTrot12v6Follow:
        SetTrot12v6FollowParams();
        break;
    case GaitId::kTrot10v4Follow:
        SetTrot10v4FollowParams();
        break;
    case GaitId::kTrot8v3Follow:
        SetTrot8v3FollowParams();
        break;
    case GaitId::kStand:
        SetStandParams();
        break;
    case GaitId::kStandNoPr:
    case GaitId::kStandPassive:
        SetStandNoPushRecoveryParams();
        break;
    default:
        SetDefaultParams();
        break;
    }
    gait_period_ = gait_type_->GetSegmentNumber();
}

void ConvexMpcLocoGaits::SetDefaultParams() {
    gait_type_ = &trot_10_5_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    rpy_cmd_scale_ << 0.0, 1.0, 0.0;  // only pitch is enabled

    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_default[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_default[ i ];
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }
    vel_cmd_zero_offset_.setZero();
    rpy_cmd_min_ << 0.0, -0.1, 0.0;  // only pitch is enabled
    rpy_cmd_max_ << 0.0, 0.1, 0.0;

    yaw_absolute_ = false;

    step_height_scale_ = 1.0;
    step_height_max_   = step_height_scale_ * ( float )user_params_->step_height_max;
    step_height_min_   = 0.0;
    step_height_ratio_ = 1.0;

    body_height_delta_max_     = 0.0;
    body_height_delta_vel_max_ = 1.6;
    body_height_filter_        = 5.0e-4;

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ].setZero();
    }
    landing_pos_ratio_last_ = landing_pos_ratio_;
    landing_pos_ratio_.setOnes();

    landing_gain_params_( 0 ) = user_params_->cmpc_bonus_swing;  // x kp and kd
    landing_gain_params_( 1 ) = user_params_->swing_p_gain( 0 );
    landing_gain_params_( 2 ) = user_params_->cmpc_bonus_swing;  // y kp and kd
    landing_gain_params_( 3 ) = user_params_->swing_p_gain( 1 );

    trans_state_ = 0.01;

    max_pos_err_mpc_ << 0.1, 0.1, 0.0;

    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_default( i );
    }

    trot_gait_             = false;
    push_recovery_enabled_ = false;

    vel_transition_ = 0.2;
    mu_for_wbc_     = user_params_->wbc_friction_default;
    mu_for_mpc_     = user_params_->mpc_friction_coef;

    for ( int i = 0; i < 3; i++ ) {
        kp_body_for_wbc_( i )  = user_params_->wbc_body_kp( i );
        kd_body_for_wbc_( i )  = user_params_->wbc_body_kd( i );
        kp_ori_for_wbc_( i )   = user_params_->wbc_orient_kp( i );
        kd_ori_for_wbc_( i )   = user_params_->wbc_orient_kd( i );
        kp_foot_for_wbc_( i )  = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i )  = user_params_->wbc_foot_kd( i );
        kp_joint_for_wbc_( i ) = user_params_->wbc_joint_kp( i );
        kd_joint_for_wbc_( i ) = user_params_->wbc_joint_kd( i );
    }

    for ( int i = 0; i < 3; i++ ) {
        se_ori_cali_offset_( i ) = robot_params_->se_ori_cali_offset( i );
        se_ori_cali_gain_( i )   = 0.0;
        // se_ori_cali_gain_( i )   = robot_params_->se_ori_cali_gain( i );
    }
    state_estimator_->SetOriCaliOffset( se_ori_cali_offset_ );
    state_estimator_->SetOriCaliGain( se_ori_cali_gain_ );
}

void ConvexMpcLocoGaits::SetTrotMediumParams() {
    gait_type_ = &trot_medium_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_medium[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_medium[ i ];
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }
    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }
    trot_gait_             = true;
    push_recovery_enabled_ = false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }
}

void ConvexMpcLocoGaits::SetTrotFastParams() {
    gait_type_ = &trot_10_4_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_fast[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_fast[ i ];
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }

    body_height_delta_max_     = 0.02;
    body_height_delta_vel_max_ = 1.5;
    body_height_filter_        = 5.0e-4;

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_trot_10_4[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot_10_4[ i ];
        }
    }
    landing_gain_params_( 0 ) = user_params_->cmpc_bonus_swing_large;  // x kp and kd
    landing_gain_params_( 1 ) = user_params_->swing_p_gain_large( 0 );
    landing_gain_params_( 2 ) = user_params_->cmpc_bonus_swing_large;  // y kp and kd
    landing_gain_params_( 3 ) = user_params_->swing_p_gain_large( 1 );

    step_height_ratio_ = 1.2;
    step_height_max_   = step_height_ratio_ * step_height_scale_ * ( float )user_params_->step_height_max;
    step_height_min_   = 0.03;

    for ( int i = 0; i < 3; i++ ) {
        se_ori_cali_gain_( i ) = robot_params_->se_ori_cali_gain( i );
    }
    state_estimator_->SetOriCaliGain( se_ori_cali_gain_ );

    trans_state_ = 0.1;

    trot_gait_             = true;
    push_recovery_enabled_ = false;
}

void ConvexMpcLocoGaits::SetTrotSlowParams() {
    gait_type_ = &trot_slow_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_slow[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_slow[ i ];
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }
}

void ConvexMpcLocoGaits::SetBoundParams() {
    gait_type_ = &bound_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 0.4, 0.4, 0.4;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i )         = dev_mode_scale_ * user_params_->vel_xy_yaw_min_bound[ i ];
        vel_cmd_max_( i )         = dev_mode_scale_ * user_params_->vel_xy_yaw_max_bound[ i ];
        acc_cmd_min_( i )         = dev_mode_scale_ * user_params_->acc_xy_yaw_min_bound[ i ];
        acc_cmd_max_( i )         = dev_mode_scale_ * user_params_->acc_xy_yaw_max_bound[ i ];
        vel_cmd_zero_offset_( i ) = user_params_->vel_xy_yaw_zero_bound[ i ];
    }
    rpy_cmd_min_ << 0.0, -0.1, 0.0;  // only pitch is enabled
    rpy_cmd_max_ << 0.0, 0.1, 0.0;

    step_height_scale_ = 0.8;
    if ( gait_first_step_ ) {
        step_height_max_ = 0.005;
        step_height_min_ = 0.0;
    }
    else {
        step_height_max_ = step_height_scale_ * ( float )user_params_->step_height_max;
        step_height_min_ = 0.005;
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_bound[ i ];
    }

    // landing_pos_ratio_last_ = landing_pos_ratio_;
    if ( vel_des_robot_( 0 ) > 0.2 ) {
        landing_pos_ratio_( 0 ) = 1.0 - ( fabs( vel_des_robot_( 0 ) ) - 0.2 ) * 0.3;
    }
    landing_pos_ratio_[ 0 ] = WrapRange( landing_pos_ratio_[ 0 ], 0.85f, 1.0f );
    landing_pos_ratio_      = 0.005 * landing_pos_ratio_ + ( 1.0 - 0.005 ) * landing_pos_ratio_last_;

    trans_state_ = 0.1;
    mu_for_wbc_  = user_params_->wbc_friction_bound;
    mu_for_mpc_  = user_params_->mpc_friction_coef_bound;
}

void ConvexMpcLocoGaits::SetPronkParams() {
    if ( robot_type_ == RobotType::CYBERDOG2 )
        gait_type_ = &mini_pronk_;
    else
        gait_type_ = &pronk_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 0.4, 0.4, 0.4;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i )         = dev_mode_scale_ * user_params_->vel_xy_yaw_min_pronk[ i ];
        vel_cmd_max_( i )         = dev_mode_scale_ * user_params_->vel_xy_yaw_max_pronk[ i ];
        acc_cmd_min_( i )         = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i )         = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
        vel_cmd_zero_offset_( i ) = user_params_->vel_xy_yaw_zero_pronk[ i ];
    }
    rpy_cmd_min_ << 0.0, -0.1, 0.0;  // only pitch is enabled
    rpy_cmd_max_ << 0.0, 0.1, 0.0;

    step_height_scale_ = 1.2;
    if ( gait_first_step_ ) {
        step_height_max_ = 0.005;
        step_height_min_ = 0.0;
    }
    else {
        step_height_max_ = step_height_scale_ * ( float )user_params_->step_height_max;
        step_height_min_ = 0.015;
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_pronk;
    }

    trans_state_ = 0.55;
}

void ConvexMpcLocoGaits::SetTrot10v4Params() {
    gait_type_ = &trot_10_4_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.1, 1.1, 1.1;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_10_4[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_10_4[ i ];
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }

    body_height_delta_max_     = 0.02;
    body_height_delta_vel_max_ = 1.5;
    body_height_filter_        = 5.0e-4;

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_trot_10_4[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot_10_4[ i ];
        }
    }
    landing_gain_params_( 0 ) = user_params_->cmpc_bonus_swing_large;  // x kp and kd
    landing_gain_params_( 1 ) = user_params_->swing_p_gain_large( 0 );
    landing_gain_params_( 2 ) = user_params_->cmpc_bonus_swing_large;  // y kp and kd
    landing_gain_params_( 3 ) = user_params_->swing_p_gain_large( 1 );

    step_height_ratio_ = 1.2;
    step_height_max_   = step_height_ratio_ * step_height_scale_ * ( float )user_params_->step_height_max;
    step_height_min_   = 0.03;

    for ( int i = 0; i < 3; i++ ) {
        se_ori_cali_gain_( i ) = robot_params_->se_ori_cali_gain( i );
    }
    state_estimator_->SetOriCaliGain( se_ori_cali_gain_ );

    trans_state_ = 0.1;

    trot_gait_             = true;
    push_recovery_enabled_ = false;
}

void ConvexMpcLocoGaits::SetWalkParams() {
    // set different gait of walk
    if ( walk_direction_ == 0 )
        gait_type_ = &walk_forward_;
    else
        gait_type_ = &walk_backward_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 0.1, 0.25, 0.25;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_walk[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_walk[ i ];
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->walk_offset_x( i );
        landing_pos_offset_[ i ]( 1 ) = user_params_->walk_offset_y( i );
    }

    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_walk( i );
    }
}

void ConvexMpcLocoGaits::SetPaceParams() {
    gait_type_ = &pace_;
}

void ConvexMpcLocoGaits::SetPassiveTrotParams() {
    gait_type_ = &passive_trot_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_passive_trot[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_passive_trot[ i ];
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }
    rpy_cmd_min_ << 0.0, 0.0, 0.0;  // rpy_cmd_[2] is zero
    rpy_cmd_max_ << 0.0, 0.0, 0.0;

    yaw_absolute_ = true;

    // max_pos_err_mpc_ << 0.05, 0.05, 0.0;

    // for ( int i = 0; i < 6; i++ ) {
    //     floating_base_weight_wbc_( i ) = user_params_->wbc_weight_passive_trot( i );
    // }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }
    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }

    push_recovery_enabled_ = false;
}

void ConvexMpcLocoGaits::SetTrot8v3Params() {
    gait_type_ = &trot_8_3_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_8_3[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_8_3[ i ];
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }

    body_height_delta_max_     = 0.01;
    body_height_delta_vel_max_ = 1.5;

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }
    landing_gain_params_( 0 ) = user_params_->cmpc_bonus_swing_large;  // x kp and kd
    landing_gain_params_( 1 ) = user_params_->swing_p_gain_large( 0 );
    landing_gain_params_( 2 ) = user_params_->cmpc_bonus_swing_large;  // y kp and kd
    landing_gain_params_( 3 ) = user_params_->swing_p_gain_large( 1 );

    step_height_ratio_ = 1.2;
    step_height_max_   = step_height_ratio_ * step_height_scale_ * ( float )user_params_->step_height_max;
    step_height_min_   = 0.03;

    trans_state_ = 0.125;

    trot_gait_             = true;
    push_recovery_enabled_ = false;

    for ( int i = 0; i < 3; i++ ) {
        se_ori_cali_gain_( i ) = robot_params_->se_ori_cali_gain( i );
    }
    state_estimator_->SetOriCaliGain( se_ori_cali_gain_ );
}

void ConvexMpcLocoGaits::SetTrot10v5Params() {
    gait_type_ = &trot_10_5_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        if ( push_recovery_vel_update_ ) {
            vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_10_5[ i ];
            vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_10_5[ i ];
        }
        else {
            vel_cmd_min_( i ) = dev_mode_scale_ * vel_cmd_min_last_( i );
            vel_cmd_max_( i ) = dev_mode_scale_ * vel_cmd_max_last_( i );
        }
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }

    body_height_delta_max_     = 0.0;
    body_height_delta_vel_max_ = 1.5;

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = ( user_params_->push_recovery_enabled > 0.1 ) ? true : false;

    for ( int i = 0; i < 3; i++ ) {
        se_ori_cali_gain_( i ) = robot_params_->se_ori_cali_gain( i );
    }
    state_estimator_->SetOriCaliGain( se_ori_cali_gain_ );
}

void ConvexMpcLocoGaits::SetTrot12v6Params() {
    gait_type_ = &trot_12_6_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_12_6[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_12_6[ i ];
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = ( user_params_->push_recovery_enabled > 0.1 ) ? true : false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }
}

void ConvexMpcLocoGaits::SetTrot14v8Params() {
    gait_type_ = &trot_14_8_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_14_8[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_14_8[ i ];
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = ( user_params_->push_recovery_enabled > 0.1 ) ? true : false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }
}

void ConvexMpcLocoGaits::SetTrot16v10Params() {
    gait_type_ = &trot_16_10_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_16_10[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_16_10[ i ];
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = ( user_params_->push_recovery_enabled > 0.1 ) ? true : false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }
}

void ConvexMpcLocoGaits::SetTrot18v11Params() {
    gait_type_ = &trot_18_11_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_18_11[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_18_11[ i ];
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = ( user_params_->push_recovery_enabled > 0.1 ) ? true : false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }
}

void ConvexMpcLocoGaits::SetTrot20v12Params() {
    gait_type_ = &trot_20_12_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_20_12[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_20_12[ i ];
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = ( user_params_->push_recovery_enabled > 0.1 ) ? true : false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }
}

void ConvexMpcLocoGaits::SetTrot22v14Params() {
    gait_type_ = &trot_22_14_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_22_14[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_22_14[ i ];
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = ( user_params_->push_recovery_enabled > 0.1 ) ? true : false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }
}

void ConvexMpcLocoGaits::SetTrot24v16Params() {
    gait_type_ = &trot_24_16_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_24_16[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_24_16[ i ];
        acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = ( user_params_->push_recovery_enabled > 0.1 ) ? true : false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }
}

void ConvexMpcLocoGaits::SetTrot8v3FollowParams() {
    gait_type_ = &trot_8_3_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_8_3[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_8_3[ i ];
        if ( use_quick_break_mode_ ) {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min_follow[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max_follow[ i ];
        }
        else {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
        }
    }

    body_height_delta_max_     = 0.01;
    body_height_delta_vel_max_ = 1.5;

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    if ( use_quick_break_mode_ ) {
        landing_gain_params_( 0 ) = user_params_->cmpc_bonus_swing_follow;  // x kp and kd
        landing_gain_params_( 1 ) = user_params_->swing_p_gain_follow( 0 );
        landing_gain_params_( 2 ) = user_params_->cmpc_bonus_swing_follow;  // y kp and kd
        landing_gain_params_( 3 ) = user_params_->swing_p_gain_follow( 1 );
    }
    else {
        landing_gain_params_( 0 ) = user_params_->cmpc_bonus_swing_large;  // x kp and kd
        landing_gain_params_( 1 ) = user_params_->swing_p_gain_large( 0 );
        landing_gain_params_( 2 ) = user_params_->cmpc_bonus_swing_large;  // y kp and kd
        landing_gain_params_( 3 ) = user_params_->swing_p_gain_large( 1 );
    }

    step_height_ratio_ = 1.2;
    step_height_max_   = step_height_ratio_ * step_height_scale_ * ( float )user_params_->step_height_max;
    step_height_min_   = 0.03;

    trans_state_ = 0.125;

    trot_gait_             = true;
    push_recovery_enabled_ = false;

    for ( int i = 0; i < 3; i++ ) {
        se_ori_cali_gain_( i ) = robot_params_->se_ori_cali_gain( i );
    }
    state_estimator_->SetOriCaliGain( se_ori_cali_gain_ );

    if ( use_quick_break_mode_ ) {
        mu_for_wbc_ = user_params_->wbc_friction_follow;
        mu_for_mpc_ = user_params_->mpc_friction_coef_follow;
    }
}

void ConvexMpcLocoGaits::SetTrot10v4FollowParams() {
    gait_type_ = &trot_10_4_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.1, 1.1, 1.1;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_10_4[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_10_4[ i ];
        if ( use_quick_break_mode_ ) {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min_follow[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max_follow[ i ];
        }
        else {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
        }
    }

    body_height_delta_max_     = 0.02;
    body_height_delta_vel_max_ = 1.5;
    body_height_filter_        = 5.0e-4;

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_trot_10_4[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot_10_4[ i ];
        }
    }
    if ( use_quick_break_mode_ ) {
        landing_gain_params_( 0 ) = user_params_->cmpc_bonus_swing_follow;  // x kp and kd
        landing_gain_params_( 1 ) = user_params_->swing_p_gain_follow( 0 );
        landing_gain_params_( 2 ) = user_params_->cmpc_bonus_swing_follow;  // y kp and kd
        landing_gain_params_( 3 ) = user_params_->swing_p_gain_follow( 1 );
    }
    else {
        landing_gain_params_( 0 ) = user_params_->cmpc_bonus_swing_large;  // x kp and kd
        landing_gain_params_( 1 ) = user_params_->swing_p_gain_large( 0 );
        landing_gain_params_( 2 ) = user_params_->cmpc_bonus_swing_large;  // y kp and kd
        landing_gain_params_( 3 ) = user_params_->swing_p_gain_large( 1 );
    }

    step_height_ratio_ = 1.2;
    step_height_max_   = step_height_ratio_ * step_height_scale_ * ( float )user_params_->step_height_max;
    step_height_min_   = 0.03;

    for ( int i = 0; i < 3; i++ ) {
        se_ori_cali_gain_( i ) = robot_params_->se_ori_cali_gain( i );
    }
    state_estimator_->SetOriCaliGain( se_ori_cali_gain_ );

    trans_state_ = 0.1;

    trot_gait_             = true;
    push_recovery_enabled_ = false;

    if ( use_quick_break_mode_ ) {
        mu_for_wbc_ = user_params_->wbc_friction_follow;
        mu_for_mpc_ = user_params_->mpc_friction_coef_follow;
    }
}

void ConvexMpcLocoGaits::SetTrot12v6FollowParams() {
    gait_type_ = &trot_12_6_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_12_6[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_12_6[ i ];
        if ( use_quick_break_mode_ ) {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min_follow[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max_follow[ i ];
        }
        else {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
        }
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }

    if ( use_quick_break_mode_ ) {
        mu_for_wbc_ = user_params_->wbc_friction_follow;
        mu_for_mpc_ = user_params_->mpc_friction_coef_follow;
    }
}

void ConvexMpcLocoGaits::SetTrot14v8FollowParams() {
    gait_type_ = &trot_14_8_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_14_8[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_14_8[ i ];
        if ( use_quick_break_mode_ ) {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min_follow[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max_follow[ i ];
        }
        else {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
        }
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }

    if ( use_quick_break_mode_ ) {
        mu_for_wbc_ = user_params_->wbc_friction_follow;
        mu_for_mpc_ = user_params_->mpc_friction_coef_follow;
    }
}

void ConvexMpcLocoGaits::SetTrot16v10FollowParams() {
    gait_type_ = &trot_16_10_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_16_10[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_16_10[ i ];
        if ( use_quick_break_mode_ ) {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min_follow[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max_follow[ i ];
        }
        else {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
        }
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }

    if ( use_quick_break_mode_ ) {
        mu_for_wbc_ = user_params_->wbc_friction_follow;
        mu_for_mpc_ = user_params_->mpc_friction_coef_follow;
    }
}

void ConvexMpcLocoGaits::SetTrot18v11FollowParams() {
    gait_type_ = &trot_18_11_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_18_11[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_18_11[ i ];
        if ( use_quick_break_mode_ ) {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min_follow[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max_follow[ i ];
        }
        else {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
        }
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }

    if ( use_quick_break_mode_ ) {
        mu_for_wbc_ = user_params_->wbc_friction_follow;
        mu_for_mpc_ = user_params_->mpc_friction_coef_follow;
    }
}

void ConvexMpcLocoGaits::SetTrot20v12FollowParams() {
    gait_type_ = &trot_20_12_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_20_12[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_20_12[ i ];
        if ( use_quick_break_mode_ ) {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min_follow[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max_follow[ i ];
        }
        else {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
        }
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }

    if ( use_quick_break_mode_ ) {
        mu_for_wbc_ = user_params_->wbc_friction_follow;
        mu_for_mpc_ = user_params_->mpc_friction_coef_follow;
    }
}

void ConvexMpcLocoGaits::SetTrot22v14FollowParams() {
    gait_type_ = &trot_22_14_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_22_14[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_22_14[ i ];
        if ( use_quick_break_mode_ ) {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min_follow[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max_follow[ i ];
        }
        else {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
        }
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }

    if ( use_quick_break_mode_ ) {
        mu_for_wbc_ = user_params_->wbc_friction_follow;
        mu_for_mpc_ = user_params_->mpc_friction_coef_follow;
    }
}

void ConvexMpcLocoGaits::SetTrot24v16FollowParams() {
    gait_type_ = &trot_24_16_;

    vel_cmd_scale_ << 1.0, 1.0, 1.0;
    acc_cmd_scale_ << 1.0, 1.0, 1.0;
    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_min_trot_24_16[ i ];
        vel_cmd_max_( i ) = dev_mode_scale_ * user_params_->vel_xy_yaw_max_trot_24_16[ i ];
        if ( use_quick_break_mode_ ) {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min_follow[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max_follow[ i ];
        }
        else {
            acc_cmd_min_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_min[ i ];
            acc_cmd_max_( i ) = dev_mode_scale_ * user_params_->acc_xy_yaw_max[ i ];
        }
    }

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ]( 0 ) = user_params_->x_offset_default[ i ];
        if ( user_params_->use_energy_saving_mode < 0.9 || enable_terrain_comp_ ) {
            landing_pos_offset_[ i ]( 1 ) = 0;
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = user_params_->y_offset_trot[ i ];
        }
    }

    trot_gait_             = true;
    push_recovery_enabled_ = false;

    for ( int i = 0; i < 3; i++ ) {
        kp_foot_for_wbc_( i ) = user_params_->wbc_foot_kp( i );
        kd_foot_for_wbc_( i ) = user_params_->wbc_foot_kd( i );
    }

    if ( use_quick_break_mode_ ) {
        mu_for_wbc_ = user_params_->wbc_friction_follow;
        mu_for_mpc_ = user_params_->mpc_friction_coef_follow;
    }
}

void ConvexMpcLocoGaits::SetStandParams() {
    gait_type_ = &stand_;

    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = 0.0;
        vel_cmd_max_( i ) = 0.0;
        acc_cmd_min_( i ) = 0.0;
        acc_cmd_max_( i ) = 0.0;
    }
    rpy_cmd_min_ << 0.0, 0.0, 0.0;
    rpy_cmd_max_ << 0.0, 0.0, 0.0;

    push_recovery_enabled_ = true;
}

void ConvexMpcLocoGaits::SetStandNoPushRecoveryParams() {
    gait_type_ = &stand_no_pr_;

    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = 0.0;
        vel_cmd_max_( i ) = 0.0;
        acc_cmd_min_( i ) = 0.0;
        acc_cmd_max_( i ) = 0.0;
    }
    rpy_cmd_min_ << 0.0, 0.0, 0.0;
    rpy_cmd_max_ << 0.0, 0.0, 0.0;

    push_recovery_enabled_ = false;
}

void ConvexMpcLocoGaits::SetCmd( ControlFsmData< float >& data ) {
    static Vec3< float > vel_cmd_ratio_positive = Vec3< float >::Ones();
    static Vec3< float > vel_cmd_ratio_negative = Vec3< float >::Ones();

#ifdef DEV_MODE
    dev_mode_scale_ = 2.0;
#else
    dev_mode_scale_ = 1.0;
#endif

    vel_des_last_ = vel_des_robot_;

#ifdef USE_ABSOLUTE_ODOM_FOR_ALL
    pos_des_rel_last_ = pos_des_rel_;
#else
    pos_des_last_   = pos_des_;
#endif
    // set vel_des_
    if ( ctrl_cmd_->gait_id == GaitId::kStand || ctrl_cmd_->gait_id == GaitId::kStandNoPr || ctrl_cmd_->gait_id == GaitId::kStandPassive || ctrl_cmd_->mode != MotionMode::kLocomotion ) {
        vel_cmd_.setZero();
    }
    else if ( duration_mode_ || ctrl_cmd_->cmd_source > kCyberdogLcmCmd ) {
        for ( int i = 0; i < 3; i++ ) {
            vel_cmd_( i ) = ctrl_cmd_->vel_des[ i ] + vel_cmd_zero_offset_( i );
        }
    }
    else {
        for ( int i = 0; i < 3; i++ ) {
            // auto switch use TROT_8_3/12_7 vel_cmd_max_
            if ( auto_switch_gait_ ) {
                vel_cmd_ratio_negative( i ) = dev_mode_scale_ * vel_min_for_switch_cmd_( i );
                vel_cmd_ratio_positive( i ) = dev_mode_scale_ * vel_max_for_switch_cmd_( i );
            }
            else {
                vel_cmd_ratio_negative( i ) = vel_cmd_min_( i );
                vel_cmd_ratio_positive( i ) = vel_cmd_max_( i );
            }
            // In case max and min are not symmetric
            if ( ctrl_cmd_->vel_des[ i ] >= 0.0 ) {
                vel_cmd_( i ) = ctrl_cmd_->vel_des[ i ] * ( vel_cmd_ratio_positive( i ) - vel_cmd_zero_offset_( i ) ) + vel_cmd_zero_offset_( i );
            }
            else {
                vel_cmd_( i ) = ctrl_cmd_->vel_des[ i ] * ( -vel_cmd_ratio_negative( i ) + vel_cmd_zero_offset_( i ) ) + vel_cmd_zero_offset_( i );
            }
        }
    }

    SetCmdOffset( data );
#ifdef USE_TERRAIN_DETECTER
    SetVelCompOnSlope();
#endif
    SetCmdScale();
    SetAccCmdScale();
    SetVelDesireRobot();

    // set step_height_des_
    if ( ctrl_cmd_->mode != MotionMode::kLocomotion ) {  // exit to other modes
        if ( robot_type_ == RobotType::CYBERDOG2 ) {
            step_height_cmd_.setConstant( 0.035 );
        }
        else {
            step_height_cmd_.setConstant( 0.05 );
        }
    }
    else {
        step_height_cmd_.setConstant( step_height_ratio_ * ctrl_cmd_->step_height[ 0 ] );
    }

    for ( int i = 0; i < 4; i++ ) {
        step_height_des_( i ) = WrapRange( step_height_cmd_[ i ], step_height_min_, step_height_max_ );
    }

    // set rpy_des_
    SetYawDesire();
    rpy_des_( 0 ) = user_params_->des_roll_pitch_height[ 0 ];
    rpy_cmd_[ 1 ] = ctrl_cmd_->rpy_des[ 1 ];
    rpy_des_( 1 ) = WrapRange( rpy_cmd_[ 1 ], rpy_cmd_scale_( 1 ) * rpy_cmd_min_( 1 ), rpy_cmd_scale_( 1 ) * rpy_cmd_max_( 1 ) );

    if ( ctrl_cmd_->mode != MotionMode::kLocomotion )  // Hold pitch angle when exit locomotion
        rpy_des_( 1 ) = state_estimator_->GetResult().rpy( 1 );

#ifdef USE_ABSOLUTE_ODOM_FOR_ALL
    // set pos_des_
    pos_cmd_rel_[ 2 ] = user_params_->des_roll_pitch_height[ 2 ] +  // user params and delta based on x vel
                        WrapRange( fabs( vel_des_robot_( 0 ) * body_height_delta_max_ ) / body_height_delta_vel_max_, 0.0f, body_height_delta_max_ );
    pos_des_rel_[ 2 ] = pos_des_rel_last_[ 2 ] * ( 1.0 - body_height_filter_ ) + pos_cmd_rel_[ 2 ] * body_height_filter_;
    pos_des_rel_[ 2 ] = WrapRange( pos_des_rel_[ 2 ], pos_cmd_rel_min_[ 2 ], pos_cmd_rel_max_[ 2 ] );
    if ( user_params_->use_energy_saving_mode < 0.9 ) {
        pos_des_rel_[ 2 ] = pos_cmd_rel_min_[ 2 ];
    }
    pos_des_( 2 ) = pos_des_rel_[ 2 ] + state_est_.position( 2 ) - state_est_.height;
#else
    // set pos_des_
    if ( !enable_terrain_comp_ )
        pos_cmd_[ 2 ] = user_params_->des_roll_pitch_height[ 2 ] +  // user params and delta based on x vel
                        WrapRange( fabs( vel_des_robot_( 0 ) * body_height_delta_max_ ) / body_height_delta_vel_max_, 0.0f, body_height_delta_max_ );
    else
        pos_cmd_[ 2 ] = user_params_->des_roll_pitch_height[ 2 ] * cos( slope_ ) +  // user params and delta based on x vel
                        WrapRange( fabs( vel_des_robot_( 0 ) * body_height_delta_max_ ) / body_height_delta_vel_max_, 0.0f, body_height_delta_max_ );
    pos_des_( 2 ) = pos_des_last_( 2 ) * ( 1.0 - body_height_filter_ ) + pos_cmd_[ 2 ] * body_height_filter_;
    pos_des_( 2 ) = WrapRange( pos_des_( 2 ), pos_cmd_min_( 2 ), pos_cmd_max_( 2 ) );
    if ( user_params_->use_energy_saving_mode < 0.9 ) {
        pos_des_( 2 ) = pos_cmd_min_( 2 );
    }
#endif
    state_estimator_->SetRemoterVelocityResult( vel_des_robot_ );  // vel_des_robot_ is used in lcm

    // select if using quick break mode
    if ( ctrl_cmd_->cmd_source == kCyberdog2LcmCmd || ctrl_cmd_->cmd_source == kCyberdogLcmCmd ) {
        if ( ctrl_cmd_->value & 0x08 )
            use_quick_break_mode_ = true;
        else
            use_quick_break_mode_ = false;
    }

    // // DEBUG
    // std::cout << "vel_cmd_: " << vel_cmd_.transpose() << std::endl;
    // std::cout << "vel_des_robot_: " << vel_des_robot_.transpose() << std::endl;
    // std::cout << "pos_des_:"        << pos_des_.transpose() << std::endl;
    // std::cout << "step_height_des_: " << step_height_des_.transpose() << std::endl;
}

void ConvexMpcLocoGaits::SetVelCompOnSlope() {
    static float slope            = 0.0;
    static float lateral_divid    = 0.0;
    static float lateral_comp_max = 0.0;
    if ( enable_terrain_comp_ ) {
        // move direction of robots on slope
        slope            = sqrt( rpy_des_with_comp_[ 0 ] * rpy_des_with_comp_[ 0 ] + rpy_des_with_comp_[ 1 ] * rpy_des_with_comp_[ 1 ] );
        lateral_divid    = rpy_des_with_comp_[ 0 ] / slope;
        lateral_comp_max = lateral_divid > 0 ? ( 0.573 * slope - 0.15 ) : ( 0.382 * slope - 0.1 );

        vel_cmd_( 1 ) += lateral_divid * lateral_comp_max;
    }
}

void ConvexMpcLocoGaits::SetCmdOffset( ControlFsmData< float >& data ) {
    switch ( gait_cmd_ ) {
    case GaitId::kTrot24v16:
        vel_cmd_( 0 ) += data.control_parameters->speed_offset_trot_24_16[ 0 ];
        vel_cmd_( 1 ) += data.control_parameters->speed_offset_trot_24_16[ 1 ];
        vel_cmd_( 2 ) += data.control_parameters->speed_offset_trot_24_16[ 2 ];
        break;
    case GaitId::kTrot20v12Follow:
    case GaitId::kTrot20v12:
        vel_cmd_( 0 ) += data.control_parameters->speed_offset_trot_follow[ 0 ];
        vel_cmd_( 1 ) += data.control_parameters->speed_offset_trot_follow[ 1 ];
        vel_cmd_( 2 ) += data.control_parameters->speed_offset_trot_follow[ 2 ];
        break;
    case GaitId::kTrotSlow:
        vel_cmd_( 0 ) += data.control_parameters->speed_offset_trot_slow[ 0 ];
        vel_cmd_( 1 ) += data.control_parameters->speed_offset_trot_slow[ 1 ];
        vel_cmd_( 2 ) += data.control_parameters->speed_offset_trot_slow[ 2 ];
        break;
    case GaitId::kTrotMedium:
        vel_cmd_( 0 ) += data.control_parameters->speed_offset_trot_medium[ 0 ];
        vel_cmd_( 1 ) += data.control_parameters->speed_offset_trot_medium[ 1 ];
        vel_cmd_( 2 ) += data.control_parameters->speed_offset_trot_medium[ 2 ];
        break;
    case GaitId::kTrotFast:
        vel_cmd_( 0 ) += data.control_parameters->speed_offset_trot_fast[ 0 ];
        vel_cmd_( 1 ) += data.control_parameters->speed_offset_trot_fast[ 1 ];
        vel_cmd_( 2 ) += data.control_parameters->speed_offset_trot_fast[ 2 ];
        break;
    case GaitId::kBound:
        vel_cmd_( 0 ) += data.control_parameters->speed_offset_bound[ 0 ];
        vel_cmd_( 1 ) += data.control_parameters->speed_offset_bound[ 1 ];
        vel_cmd_( 2 ) += data.control_parameters->speed_offset_bound[ 2 ];
        break;
    case GaitId::kPronk:
        vel_cmd_( 0 ) += data.control_parameters->speed_offset_pronk[ 0 ];
        vel_cmd_( 1 ) += data.control_parameters->speed_offset_pronk[ 1 ];
        vel_cmd_( 2 ) += data.control_parameters->speed_offset_pronk[ 2 ];
        break;
    default:
        break;
    }
}

void ConvexMpcLocoGaits::SetCmdScale() {
    static float         vel_cmd_weight_norm = 0.0;
    static float         vel_cmd_norm_thresh = 1.0;
    static Vec3< float > vel_des_last_max    = Vec3< float >::Ones();

    if ( user_params_->vel_scale_limit_type < 0.9 ) {
        // x_dot has effect on y_dot, yaw_dot and pitch
        // backward has strict limit of yaw_dot
        if ( vel_des_last_( 0 ) >= 0.0 ) {
            vel_cmd_scale_( 1 ) *= fabs( vel_des_last_( 0 ) ) * user_params_->x_effect_scale_pos + 1.0;  // y_dot
            vel_cmd_scale_( 2 ) *= fabs( vel_des_last_( 0 ) ) * user_params_->x_effect_scale_pos + 1.0;  // yaw_dot
            rpy_cmd_scale_( 1 ) *= fabs( vel_des_last_( 0 ) ) * user_params_->x_effect_scale_pos + 1.0;  // pitch
        }
        else {
            vel_cmd_scale_( 1 ) *= fabs( vel_des_last_( 0 ) ) * user_params_->x_effect_scale_neg + 1.0;  // y_dot
            vel_cmd_scale_( 2 ) *= fabs( vel_des_last_( 0 ) ) * user_params_->x_effect_scale_neg + 1.0;  // yaw_dot
            rpy_cmd_scale_( 1 ) *= fabs( vel_des_last_( 0 ) ) * user_params_->x_effect_scale_neg + 1.0;  // pitch
        }

        // yaw_dot has effect on y_dot and pitch
        // backward run is bad, the velocity has to be limited as yaw_dot increases
        if ( vel_des_last_( 0 ) >= 0.0 ) {
            vel_cmd_scale_( 0 ) *= fabs( vel_des_last_( 2 ) ) * user_params_->yaw_effect_scale[ 0 ] + 1.0;  // x_dot
        }
        else {
            vel_cmd_scale_( 0 ) *= fabs( vel_des_last_( 2 ) ) * user_params_->yaw_effect_scale[ 1 ] + 1.0;  // x_dot
        }
        vel_cmd_scale_( 1 ) *= fabs( vel_des_last_( 2 ) ) * user_params_->yaw_effect_scale[ 2 ] + 1.0;  // y_dot
        rpy_cmd_scale_( 1 ) *= fabs( vel_des_last_( 2 ) ) * user_params_->yaw_effect_scale[ 2 ] + 1.0;  // pitch

        // y_dot has effect on yaw_dot and pitch
        vel_cmd_scale_( 2 ) *= fabs( vel_des_last_( 1 ) ) * user_params_->y_effect_scale + 1.0;  // yaw_dot
        rpy_cmd_scale_( 1 ) *= fabs( vel_des_last_( 1 ) ) * user_params_->y_effect_scale + 1.0;  // pitch

        for ( int i = 0; i < 3; i++ ) {
            if ( vel_cmd_scale_( i ) < 0.0 ) {
                vel_cmd_scale_( i ) = 0.0;
            }
            if ( rpy_cmd_scale_( i ) < 0.0 ) {
                rpy_cmd_scale_( i ) = 0.0;
            }
        }
    }
    else {
        // x y
        vel_cmd_weight_norm = 0.0;
        for ( int i = 0; i < 2; i++ ) {
            if ( vel_des_last_( i ) >= 0.0 ) {
                vel_des_last_max( i ) = vel_cmd_max_( i );
            }
            else {
                vel_des_last_max( i ) = vel_cmd_min_( i );
            }
            vel_cmd_weight_norm += pow( vel_des_last_( i ) / vel_des_last_max( i ), 2 );
        }
        vel_cmd_weight_norm = sqrt( vel_cmd_weight_norm );
        if ( vel_cmd_weight_norm >= vel_cmd_norm_thresh ) {
            vel_cmd_scale_ << vel_cmd_norm_thresh / vel_cmd_weight_norm, vel_cmd_norm_thresh / vel_cmd_weight_norm, 1.0;
        }
        else {
            vel_cmd_scale_ << 1.0, 1.0, 1.0;
        }
        vel_cmd_weight_norm = vel_cmd_scale_( 0 ) * vel_cmd_weight_norm;
        // yaw v * omega
        if ( fabs( vel_cmd_weight_norm * vel_des_last_( 2 ) ) > user_params_->centrifugal_thresh ) {
            for ( int i = 0; i < 3; i++ ) {
                vel_cmd_scale_( i ) *= user_params_->centrifugal_thresh / fabs( vel_cmd_weight_norm * vel_des_last_( 2 ) );
            }
        }
        vel_cmd_weight_norm = vel_cmd_scale_( 0 ) * vel_cmd_weight_norm;
        // pitch
        rpy_cmd_scale_( 1 ) = 1.0 - fmin( 1.0, vel_cmd_weight_norm );
    }

    // // DEBUG
    // std::cout << "vel_cmd_scale_: " << vel_cmd_scale_.transpose() << std::endl;
    // std::cout << "rpy_cmd_scale_: " << rpy_cmd_scale_.transpose() << std::endl;
    // std::cout << "vel_cmd_weight_norm: " << vel_cmd_weight_norm << std::endl;
}

/**
 * @brief Set absolute acceleration that not affected by motion direction
 *
 */
void ConvexMpcLocoGaits::SetAccCmdScale() {
    for ( int i = 0; i < 3; i++ ) {
        if ( vel_des_last_[ i ] < 0.0 )
            acc_cmd_scale_[ i ] *= -1;
    }
    // printf log
    // std::cout << "[vel cmd] " << vel_cmd( 0 ) << " [vel cmd last] " << vel_des_last_[ 0 ] << " [acc_cmd_scale] " << acc_cmd_scale_[ 0 ] << std::endl;
}

void ConvexMpcLocoGaits::SetVelDesireRobot() {
    // temp vars for passive_trot
    static Vec3< float > vel_current( state_est_.velocity_in_body_frame( 0 ), state_est_.velocity_in_body_frame( 1 ), state_est_.angular_velocity_in_body_frame[ 2 ] );  // x y yaw
    static Vec3< float > vel_cmd_passive( 0.0, 0.0, 0.0 );
    static Vec3< float > zero_tresh( 0.12, 0.2, 0.45 );            // deadzone of error between current_vel and target_vel
    static Vec3< float > acc_tresh( 0.1, 0.2, 0.5 );               // maximum change velocity of every time step
    static Vec3< float > vel_filter( 0.95, 0.8, 0.95 );            // filter params
    static Vec3< float > vel_trigger( 0.8, 0.0, 0.0 );             // a threshold that triggers vel_hold
    static Vec3< float > vel_hold( vel_cmd_max_( 0 ), 0.0, 0.0 );  // after triggers vel_hold, target velocity is vel_hold
    static Vec3< float > vel_current_filter( 0.0, 0.0, 0.0 );
    static float         err_vx                           = 0.0;
    static float         err_vy                           = 0.0;
    static float         vel_tresh                        = 0.08;                         // deadzone of yaw velocity changing
    static float         yaw_comp_prop                    = 40.0;                         // yaw vel ratio based on vel_err norm
    static float         low_pass_butterworth_coeff1[ 3 ] = { 1.0000, -1.8227, 0.8372 };  // 10Hz
    static float         low_pass_butterworth_coeff2[ 3 ] = { 0.0036, 0.0072, 0.0036 };

    vel_current << state_est_.velocity_in_body_frame( 0 ), state_est_.velocity_in_body_frame( 1 ), state_est_.angular_velocity_in_body_frame[ 2 ];
    for ( int i = 0; i < 3; i++ ) {
        if ( current_gait_ == GaitId::kPassiveTrot && gait_cmd_is_passive_trot_ ) {  // PASSIVE_TROT doesn't follow RC or GAMEPAD
            vel_current_filter( 0 ) = LowpassFilterButterworth( vel_current[ 0 ], vel_buffer_in_x_, vel_buffer_out_x_, low_pass_butterworth_coeff1, low_pass_butterworth_coeff2 );
            vel_current_filter( 1 ) = LowpassFilterButterworth( vel_current[ 1 ], vel_buffer_in_y_, vel_buffer_out_y_, low_pass_butterworth_coeff1, low_pass_butterworth_coeff2 );
            vel_current_filter( 2 ) = LowpassFilterButterworth( vel_current[ 2 ], vel_buffer_in_yaw_, vel_buffer_out_yaw_, low_pass_butterworth_coeff1, low_pass_butterworth_coeff2 );
            if ( gait_iter_ > 1000 ) {
                vel_des_robot_( i ) = FollowMoving( vel_current_filter( i ), vel_cmd_passive( i ), vel_cmd_min_( i ), vel_cmd_max_( i ), zero_tresh( i ), acc_tresh( i ), vel_filter( i ),
                                                    vel_trigger( i ), vel_hold( i ) );
                if ( i == 2 ) {
                    err_vx = vel_cmd_passive( 0 ) - vel_current_filter( 0 );
                    err_vy = vel_cmd_passive( 1 ) - vel_current_filter( 1 );
                    vel_des_robot_( 2 ) =
                        FollowMovingYaw( vel_des_robot_( 2 ), vel_current_filter[ 2 ], err_vx, err_vy, vel_cmd_min_( 2 ), vel_cmd_max_( 2 ), vel_tresh, acc_tresh( 2 ), yaw_comp_prop );
                }
            }
            else {
                vel_des_robot_.setZero();
            }
        }
        else {
            vel_des_robot_( i ) = ApplyVelocityMeetAccelationLimit( vel_des_last_( i ), vel_cmd_( i ), vel_cmd_scale_( i ) * vel_cmd_min_( i ), vel_cmd_scale_( i ) * vel_cmd_max_( i ),
                                                                    acc_cmd_scale_( i ) * acc_cmd_min_( i ), acc_cmd_scale_( i ) * acc_cmd_max_( i ), dt_ );
        }
    }

    // // DEBUG
    // std::cout << "vel_des_robot_: " << vel_des_robot_.transpose() << std::endl;
    // std::cout << "vel_cmd_max_: " << vel_cmd_max_.transpose() << std::endl;
}

void ConvexMpcLocoGaits::SetYawDesire() {
    static float yaw_dot_deadzone       = 1e-4;
    static float k_integral             = 1e-3;
    static float yaw_err_max            = 0.12;
    static float yaw_err_integral_reset = 0.01;

    if ( yaw_absolute_ ) {  // use absolute yaw
        rpy_des_( 2 ) = rpy_des_( 2 ) + vel_des_robot_( 2 ) * dt_;
        rpy_des_( 2 ) = WrapRange( rpy_des_( 2 ), ( float )( state_est_.rpy( 2 ) - yaw_err_max ), ( float )( state_est_.rpy( 2 ) + yaw_err_max ) );
        yaw_des_last_ = rpy_des_( 2 );
        yaw_integral_ = 0.0;
    }
    else {                                                                                                                    // use relative yaw
        if ( fabs( vel_des_robot_( 2 ) ) > yaw_dot_deadzone || fabs( rpy_des_( 2 ) - state_est_.rpy( 2 ) ) > yaw_err_max ) {  // set yaw based on yaw dot
            rpy_des_( 2 ) = state_est_.rpy( 2 ) + vel_des_robot_( 2 ) * dt_;
            yaw_des_last_ = rpy_des_( 2 );
            yaw_integral_ = 0.0;
        }
        else {  // integral yaw error
            yaw_integral_ += yaw_des_last_ - state_est_.rpy( 2 );
            if ( fabs( yaw_des_last_ - state_est_.rpy( 2 ) ) < yaw_err_integral_reset ) {
                yaw_integral_ = 0.0;
            }
            rpy_des_( 2 ) = state_est_.rpy( 2 ) + k_integral * yaw_integral_;
        }
    }
}

void ConvexMpcLocoGaits::CheckGaitFirstStep() {
    if ( gait_iter_ < horizon_length_ * iter_between_mpc_ ) {
        gait_first_step_ = true;
    }
    else {
        gait_first_step_ = false;
    }
}

void ConvexMpcLocoGaits::SetBodyWorldDesire() {
    static Vec3< float > v_robot_xyz( vel_des_robot_( 0 ), vel_des_robot_( 1 ), 0.0 );

    v_robot_xyz << vel_des_robot_( 0 ), vel_des_robot_( 1 ), 0.0;
    vel_des_ = omni_cmd_ ? v_robot_xyz : state_est_.world2body_rotation_matrix.transpose() * v_robot_xyz;
    pos_des_( 0 ) += dt_ * vel_des_[ 0 ];
    pos_des_( 1 ) += dt_ * vel_des_[ 1 ];
}

void ConvexMpcLocoGaits::GetFootPosFeedback() {
    for ( int i = 0; i < 4; i++ ) {
        foot_pos_feedback_[ i ] = state_est_.position + state_est_.world2body_rotation_matrix.transpose() * ( robot_model_->GetHipLocation( i ) + leg_ctrl_->datas_[ i ].p );
    }
}

void ConvexMpcLocoGaits::SetGaitFirstRunVars() {
    pos_des_( 0 ) = state_est_.position( 0 );
    pos_des_( 1 ) = state_est_.position( 1 );
    rpy_des_( 2 ) = state_est_.rpy( 2 );

    init_pos_des_ = state_est_.position;
    init_rpy_des_ = state_est_.rpy;

    for ( int i = 0; i < 4; i++ ) {
        foot_swing_trajectory_[ i ].SetHeight( step_height_des_( i ) );
        foot_swing_trajectory_[ i ].SetInitialPosition( foot_pos_feedback_[ i ] );
        foot_swing_trajectory_[ i ].SetFinalPosition( foot_pos_feedback_[ i ] );
    }
    gait_first_run_ = false;

    // // DEBUG
    // std::cout << "pos_des_: " << pos_des_.transpose() << std::endl;
    // std::cout << "init_pos_des_: " << init_pos_des_.transpose() << std::endl;
}

void ConvexMpcLocoGaits::SetRpyComp() {
    if ( fabs( state_est_.velocity_in_body_frame( 0 ) ) > 0.02 ) {  // x vel affects pitch
        rpy_integral_( 1 ) += 5.0 * dt_ * ( rpy_des_( 1 ) - state_est_.rpy( 1 ) ) / state_est_.velocity_in_body_frame( 0 );
    }
    if ( fabs( state_est_.velocity_in_body_frame( 1 ) ) > 0.01 ) {  // y vel affects roll
        rpy_integral_( 0 ) += 1.0 * dt_ * ( rpy_des_( 0 ) - state_est_.rpy( 0 ) ) / state_est_.velocity_in_body_frame( 1 );
    }

    rpy_integral_( 0 ) = WrapRange( rpy_integral_( 0 ), -0.25f, 0.25f );
    rpy_integral_( 1 ) = WrapRange( rpy_integral_( 1 ), -0.25f, 0.25f );
    rpy_comp_( 0 )     = state_est_.velocity_in_body_frame( 1 ) * rpy_integral_( 0 ) * ( current_gait_ != GaitId::kPronk );  // turnoff for pronking
    rpy_comp_( 1 )     = state_est_.velocity_in_body_frame( 0 ) * rpy_integral_( 1 );
#ifdef USE_TERRAIN_DETECTER
    // add terrain detecter
    if ( enable_terrain_comp_ ) {
        Vec3< float >                orienvecdes( rpy_des_( 0 ) + rpy_comp_( 0 ), rpy_des_( 1 ) + rpy_comp_( 1 ), rpy_des_( 2 ) );
        Eigen::Matrix< float, 4, 1 > quatdes;
        est_terrain_rot_matrix_ = state_est_.terrain_rotation_matrix;
        quatdes                 = RotationMatrixToQuaternion( RpyToRotMat( orienvecdes ) * est_terrain_rot_matrix_.transpose() );
        Eigen::Matrix< float, 4, 1 > quatprev( state_est_.orientation[ 0 ], state_est_.orientation[ 1 ], state_est_.orientation[ 2 ], state_est_.orientation[ 3 ] );
        // The step below could avoid sign error
        if ( ( quatprev - quatdes ).norm() > ( quatprev + quatdes ).norm() ) {
            quatdes = -quatdes;
        }
        rpy_des_with_comp_ = QuatToRPY( quatdes );
        if ( fabs( rpy_des_with_comp_[ 2 ] - state_est_.rpy( 2 ) ) > 3.14 ) {
            rpy_des_with_comp_[ 2 ] = -rpy_des_with_comp_[ 2 ];
        }
    }
    else {
        rpy_des_with_comp_ = rpy_des_ + rpy_comp_;
    }
#else
    rpy_des_with_comp_ = rpy_des_ + rpy_comp_;
#endif
    // // DEBUG
    // std::cout << "rpy_comp_: " << rpy_comp_.transpose() << std::endl;
}

void ConvexMpcLocoGaits::SetLandingPos() {
    static float         side_sign[ 4 ]                = { -1.0f, 1.0f, -1.0f, 1.0f };  // dirction of abad_link_length_
    static float         avoid_collision_y_offset[ 4 ] = { 0.0, 0.0, 0.0, -0.0 };       //{0.0, 0.0, 0.02, -0.02};
    static float         landing_pos_ctrl_max          = 0.35f;
    static Vec3< float > hip_pos_robot_frame( 0.0, 0.0, 0.0 );
    static Vec3< float > hip_foot_offset( 0.0, 0.0, 0.0 );
    static Vec3< float > pos_yaw_corrected( 0.0, 0.0, 0.0 );
    static Vec3< float > vel_des_pf( 0.0, 0.0, 0.0 );
    static Vec3< float > landing_pos_ctrl( 0.0, 0.0, 0.0 );
    static Vec3< float > landing_pos_ctrl_world( 0.0, 0.0, 0.0 );

    SetLandingOffsetByVel();
    for ( int i = 0; i < 4; i++ ) {
        swing_time_( i )  = gait_type_->GetCurrentSwingTime( dt_mpc_, i );
        stance_time_( i ) = gait_type_->GetCurrentStanceTime( dt_mpc_, i );

        if ( foot_first_swing_[ i ] ) {
            swing_time_remain_( i ) = swing_time_( i );
        }
        else {
            swing_time_remain_( i ) -= dt_;
        }
        hip_foot_offset << 0.0, side_sign[ i ] * robot_model_->abad_link_length_, 0.0;
        hip_pos_robot_frame =
            landing_pos_ratio_.cwiseProduct( robot_model_->GetHipLocation( i ) + hip_foot_offset + landing_pos_offset_scale_ * landing_pos_offset_[ i ] );  // feet landing pos withdraw or extend
        hip_pos_robot_frame[ 1 ] += avoid_collision_y_offset[ i ] * fabs( vel_des_robot_( 0 ) );                                                            // rear feet land outside to avoid collision

        pos_yaw_corrected = CoordinateRotation( CoordinateAxis::Z, -vel_des_robot_( 2 ) * stance_time_( i ) / 2 ) * hip_pos_robot_frame;

        vel_des_pf[ 0 ] = vel_des_robot_( 0 );
        vel_des_pf[ 1 ] = vel_des_robot_( 1 );
        vel_des_pf[ 2 ] = 0.0;

        landing_pos_[ i ] = state_est_.position
                            + state_est_.world2body_rotation_matrix.transpose() *  // perdict hip pos at landing
                                  ( pos_yaw_corrected + vel_des_pf * swing_time_remain_( i ) );
        landing_pos_ctrl[ 0 ] = vel_des_robot_( 0 ) * ( .5 + landing_gain_params_( 0 ) ) * stance_time_( i )
                                + landing_gain_params_( 1 ) * ( state_est_.velocity_in_body_frame( 0 ) - vel_des_robot_( 0 ) )
                                + 0.5f * state_est_.position( 2 ) / GRAVITY * ( state_est_.velocity_in_body_frame( 1 ) * vel_des_robot_( 2 ) );
        landing_pos_ctrl[ 1 ] = vel_des_robot_( 1 ) * ( .5 + landing_gain_params_( 2 ) ) * stance_time_( i )
                                + landing_gain_params_( 3 ) * ( state_est_.velocity_in_body_frame( 1 ) - vel_des_robot_( 1 ) )
                                + 0.5f * state_est_.position( 2 ) / GRAVITY * ( -state_est_.velocity_in_body_frame( 0 ) * vel_des_robot_( 2 ) );
        landing_pos_ctrl[ 0 ]  = WrapRange( landing_pos_ctrl[ 0 ], -landing_pos_ctrl_max, landing_pos_ctrl_max );
        landing_pos_ctrl[ 1 ]  = WrapRange( landing_pos_ctrl[ 1 ], -landing_pos_ctrl_max, landing_pos_ctrl_max );
        landing_pos_ctrl[ 2 ]  = 0.0;
        landing_pos_ctrl_world = state_est_.world2body_rotation_matrix.transpose() * landing_pos_ctrl;
        landing_pos_[ i ]( 0 ) += landing_pos_ctrl_world[ 0 ];
        landing_pos_[ i ]( 1 ) += landing_pos_ctrl_world[ 1 ];
#ifdef USE_ABSOLUTE_ODOM_FOR_ALL
        landing_pos_[ i ]( 2 ) = user_params_->foot_final_height + state_est_.position( 2 ) - state_est_.height;
#else
        landing_pos_[ i ]( 2 ) = user_params_->foot_final_height;
#endif

#ifdef USE_TERRAIN_DETECTER
        if ( enable_terrain_comp_ ) {
            static int forward_id[ 2 ] = { 0, 1 };
            float      forward_divid   = rpy_des_with_comp_[ 1 ] / sqrt( rpy_des_with_comp_[ 0 ] * rpy_des_with_comp_[ 0 ] + rpy_des_with_comp_[ 1 ] * rpy_des_with_comp_[ 1 ] );
            float      right_divid     = rpy_des_with_comp_[ 0 ] / sqrt( rpy_des_with_comp_[ 0 ] * rpy_des_with_comp_[ 0 ] + rpy_des_with_comp_[ 1 ] * rpy_des_with_comp_[ 1 ] );
            // determine direction of moving and which pair of feet is upward on slop
            if ( forward_divid < -0.5 ) {
                heading_direction_on_slope_ = 1;
                forward_id[ 0 ]             = 0;
                forward_id[ 1 ]             = 1;
            }
            else if ( forward_divid < 0.5 ) {
                if ( right_divid > 0 ) {
                    heading_direction_on_slope_ = 2;
                    forward_id[ 0 ]             = 1;
                    forward_id[ 1 ]             = 3;
                }
                else {
                    heading_direction_on_slope_ = -2;
                    forward_id[ 0 ]             = 0;
                    forward_id[ 1 ]             = 2;
                }
            }
            else {
                heading_direction_on_slope_ = -1;
                forward_id[ 0 ]             = 2;
                forward_id[ 1 ]             = 3;
            }

            est_terrain_coef_ = state_est_.terrain_coefficient;
            if ( i == forward_id[ 0 ] || i == forward_id[ 1 ] )
                landing_pos_[ i ]( 2 ) += est_terrain_coef_( 0 ) + est_terrain_coef_( 1 ) * landing_pos_[ i ]( 0 ) + est_terrain_coef_( 2 ) * landing_pos_[ i ]( 1 );
            else
                landing_pos_[ i ]( 2 ) += 1.16 * ( est_terrain_coef_( 0 ) + est_terrain_coef_( 1 ) * landing_pos_[ i ]( 0 ) + est_terrain_coef_( 2 ) * landing_pos_[ i ]( 1 ) );
        }
#endif

        foot_swing_trajectory_[ i ].SetHeight( step_height_des_( i ) );
        foot_swing_trajectory_[ i ].SetFinalPosition( landing_pos_[ i ] );
    }

    // // DEBUG
    // for (int i = 0; i < 4; i++) {
    //   std::cout << "landing_pos_[" << i << "]: " << landing_pos_[ i ].transpose() << std::endl;
    // }
}

void ConvexMpcLocoGaits::SetTerrainComp() {

    static float filter = 1.0;
    slope_              = ( 1 - filter ) * slope_ + filter * sqrt( pow( state_est_.terrain_coefficient( 1 ), 2 ) + pow( state_est_.terrain_coefficient( 2 ), 2 ) );
    // set threshold of detecting slope
    if ( current_gait_ != GaitId::kTrot8v3 && current_gait_ != GaitId::kTrotFast && current_gait_ != GaitId::kTrot10v4 ) {
        slope_up_threshold_   = 0.08;
        slope_down_threshold_ = 0.05;
    }
    else {
        slope_up_threshold_   = 0.1;
        slope_down_threshold_ = 0.06;
    }
    // set two different threshold to avoid shocking nearby the threshold
    if ( slope_ >= slope_up_threshold_ || ( slope_ >= slope_down_threshold_ && enable_terrain_comp_ ) ) {
        slope_threshold_num_++;
        if ( slope_threshold_num_ > 60 )
            enable_terrain_comp_ = true;
    }
    else {
        slope_threshold_num_ = 0;
        enable_terrain_comp_ = false;
    }
}

void ConvexMpcLocoGaits::SetLandingOffsetByVel() {
    static float landing_pos_offset_ratio_y   = 1.0;
    static float landing_pos_offset_ratio_yaw = 1.0;

    if ( fabs( vel_cmd_max_( 1 ) ) > 1e-2 && fabs( vel_cmd_max_( 2 ) > 1e-2 ) ) {
        landing_pos_offset_ratio_y   = 1.0 - fmin( 1.0, fabs( 1.5 * vel_des_robot_( 1 ) / vel_cmd_max_( 1 ) ) );
        landing_pos_offset_ratio_yaw = 1.0 - fmin( 1.0, fabs( 0.5 * vel_des_robot_( 2 ) / vel_cmd_max_( 2 ) ) );
    }
    else {
        // STAND gait
        landing_pos_offset_ratio_y   = 1.0;
        landing_pos_offset_ratio_yaw = 1.0;
    }

    // select a smaller one between y and yaw
    for ( int i = 0; i < 4; i++ ) {
        if ( landing_pos_offset_ratio_y <= landing_pos_offset_ratio_yaw ) {
            landing_pos_offset_[ i ]( 1 ) = landing_pos_offset_ratio_y * landing_pos_offset_[ i ]( 1 );
        }
        else {
            landing_pos_offset_[ i ]( 1 ) = landing_pos_offset_ratio_yaw * landing_pos_offset_[ i ]( 1 );
        }
    }
    // push recovery disables lateral landing offsets
    if ( push_recovery_flag_ ) {
        for ( int i = 0; i < 4; i++ ) {
            landing_pos_offset_[ i ]( 1 ) = 0.0;
        }
    }

    // // DEBUG
    // for (int i = 0; i < 4; i++) {
    //   std::cout << "landing_pos_offset[" << i << "]: " << landing_pos_offset[i].transpose() << std::endl;
    // }
}

void ConvexMpcLocoGaits::CheckGaitTransition() {
    static Vec3< float > vel_xy_robot( state_est_.velocity_in_body_frame( 0 ), state_est_.velocity_in_body_frame( 1 ), 0.0 );
    static float         vel_ban_exit = 0.15;

    contact_states_        = gait_type_->GetContactState();
    swing_states_          = gait_type_->GetSwingState();
    gait_check_transition_ = false;
    gait_allow_transition_ = true;
    vel_xy_robot << state_est_.velocity_in_body_frame( 0 ), state_est_.velocity_in_body_frame( 1 ), 0.0;

    // allow transition in STAND
    if ( current_gait_ == GaitId::kStand || current_gait_ == GaitId::kStandNoPr || current_gait_ == GaitId::kStandPassive ) {
        gait_check_transition_ = true;
    }
    // trot gait enables transition only at FR touchdown
    // auto switch gaits don't consider vel_transition_
    else if ( trot_gait_ && auto_switch_gait_ ) {
        if ( swing_states_( 0 ) >= ( 1.0 - trans_state_ ) && swing_states_( 1 ) <= trans_state_ && swing_states_( 3 ) >= ( 1.0 - trans_state_ ) && swing_states_( 2 ) <= trans_state_ ) {
            gait_check_transition_ = true;
        }
    }
    // allow transition in low speed
    else if ( vel_xy_robot.norm() < vel_transition_ ) {
        if ( trot_gait_ ) {
            if ( swing_states_( 0 ) >= ( 1.0 - trans_state_ ) && swing_states_( 1 ) <= trans_state_ && swing_states_( 3 ) >= ( 1.0 - trans_state_ ) && swing_states_( 2 ) <= trans_state_ ) {
                gait_check_transition_ = true;
            }
        }
        else if ( current_gait_ != GaitId::kPronk ) {
            if ( ( swing_states_( 0 ) <= trans_state_ || swing_states_( 0 ) >= ( 1.0 - trans_state_ ) ) && ( swing_states_( 1 ) <= trans_state_ || swing_states_( 1 ) >= ( 1.0 - trans_state_ ) )
                 && ( swing_states_( 2 ) <= trans_state_ || swing_states_( 2 ) >= ( 1.0 - trans_state_ ) ) && ( swing_states_( 3 ) <= trans_state_ || swing_states_( 3 ) >= ( 1.0 - trans_state_ ) ) ) {
                gait_check_transition_ = true;
            }
        }
        else {
            if ( ( contact_states_[ 0 ] <= trans_state_ && contact_states_[ 0 ] >= ( 1.0 - trans_state_ ) )
                 && ( contact_states_[ 1 ] <= trans_state_ && contact_states_[ 1 ] >= ( 1.0 - trans_state_ ) )
                 && ( contact_states_[ 2 ] <= trans_state_ && contact_states_[ 2 ] >= ( 1.0 - trans_state_ ) )
                 && ( contact_states_[ 3 ] <= trans_state_ && contact_states_[ 3 ] >= ( 1.0 - trans_state_ ) ) ) {
                gait_check_transition_ = true;
            }
        }
    }
    else {
        gait_allow_transition_ = false;
    }

    // ban transition when velocity is too high
    if ( vel_est_avg_.norm() >= vel_ban_exit ) {
        ban_exit_loco_ = true;
    }
    else {
        ban_exit_loco_ = false;
    }

    // // DBEUG
    // if (gait_check_transition_) {
    //     std::cout << "loco_iter_: " << loco_iter_ << std::endl;
    //     std::cout << "gait_iter_: " << gait_iter_ << std::endl;
    //     std::cout << "ban_exit_loco_: " << ban_exit_loco_ << std::endl;
    // }
}

void ConvexMpcLocoGaits::ComputeVelAverage() {
    vel_est_sum_( 0 ) += state_est_.velocity_in_body_frame( 0 );
    vel_est_sum_( 1 ) += state_est_.velocity_in_body_frame( 1 );
    vel_est_sum_( 2 ) += state_est_.angular_velocity_in_body_frame( 2 );

    if ( gait_iter_ == 1 ) {
        // reset sum after gait changes
        vel_est_sum_.setZero();
    }

    if ( ( gait_iter_ % ( gait_period_ * iter_between_mpc_ ) ) == 0 ) {
        if ( current_gait_ != GaitId::kStand && current_gait_ != GaitId::kStandNoPr && current_gait_ != GaitId::kStandPassive ) {
            vel_est_avg_ = vel_est_sum_ / ( fabs( gait_period_ * iter_between_mpc_ ) > 1 ? fabs( gait_period_ * iter_between_mpc_ ) : 1 );
            vel_est_sum_.setZero();
        }
        else {
            // Stand average vel considers 150 steps
            vel_est_avg_ = vel_est_sum_ / ( fabs( 10 * iter_between_mpc_ ) > 1 ? fabs( 10 * iter_between_mpc_ ) : 1 );
            vel_est_sum_.setZero();
        }
    }

    // // DEBUG
    // std::cout << "vel_est_avg_: " << vel_est_avg_.transpose() << std::endl;
    // std::cout << "vel_est_sum_: " << vel_est_sum_.transpose() << std::endl;
}

void ConvexMpcLocoGaits::ComputeMpc() {
    static int* mpc_table = gait_type_->GetMpcTable();
    mpc_table             = gait_type_->GetMpcTable();
    start_solve_mpc_      = false;
    UpdateMpc( mpc_table );
}

void ConvexMpcLocoGaits::UpdateMpc( int* mpc_table ) {
    static Vec3< float > pos_init( 0.0, 0.0, 0.0 );
    static Vec3< float > v_robot( 0.0, 0.0, 0.0 );
    static Vec3< float > v_world( 0.0, 0.0, 0.0 );
    static float*        p = state_est_.position.data();

    if ( ( loco_iter_ % iter_between_mpc_ ) == 0 ) {
        pos_init = pos_des_;
        p        = state_est_.position.data();
        if ( pos_init[ 0 ] - p[ 0 ] > max_pos_err_mpc_( 0 ) )
            pos_init[ 0 ] = p[ 0 ] + max_pos_err_mpc_( 0 );

        if ( p[ 0 ] - pos_init[ 0 ] > max_pos_err_mpc_( 0 ) )
            pos_init[ 0 ] = p[ 0 ] - max_pos_err_mpc_( 0 );

        if ( pos_init[ 1 ] - p[ 1 ] > max_pos_err_mpc_( 1 ) )
            pos_init[ 1 ] = p[ 1 ] + max_pos_err_mpc_( 1 );

        if ( p[ 1 ] - pos_init[ 1 ] > max_pos_err_mpc_( 1 ) )
            pos_init[ 1 ] = p[ 1 ] - max_pos_err_mpc_( 1 );

        pos_des_ = pos_init;

        v_robot << vel_des_robot_( 0 ), vel_des_robot_( 1 ), 0.0;
        v_world = omni_cmd_ ? v_robot : CoordinateRotation( CoordinateAxis::Z, -rpy_des_( 2 ) ) * v_robot;

#ifndef USE_TIME_VARING_MPC
        float traj_init[ NUM_MPC_STATE_INPUT ] = {                           /*rpy_des_( 0 ) + rpy_comp_( 0 ),  // 0
                                                            rpy_des_( 1 ) + rpy_comp_( 1 ),  // 1 */
                                                   rpy_des_with_comp_[ 0 ],  // 0
                                                   rpy_des_with_comp_[ 1 ],  // 1
                                                   rpy_des_( 2 ),            // 2
                                                   pos_init[ 0 ],            // 3
                                                   pos_init[ 1 ],            // 4
                                                   pos_init[ 2 ],            // 5
                                                   0,                        // 6
                                                   0,                        // 7
                                                   vel_des_robot_( 2 ),      // 8
                                                   v_world[ 0 ],             // 9
                                                   v_world[ 1 ],             // 10
                                                   0
        };  // 11
        for ( int i = 0; i < horizon_length_; i++ ) {
            for ( int j = 0; j < NUM_MPC_STATE_INPUT; j++ )
                traj_all_[ NUM_MPC_STATE_INPUT * i + j ] = traj_init[ j ];

            if ( i > 0 ) {
                traj_all_[ NUM_MPC_STATE_INPUT * i + 2 ] = traj_all_[ NUM_MPC_STATE_INPUT * ( i - 1 ) + 2 ] + dt_mpc_ * vel_des_robot_( 2 );
                v_world                                  = omni_cmd_ ? v_robot : CoordinateRotation( CoordinateAxis::Z, -traj_all_[ NUM_MPC_STATE_INPUT * i + 2 ] ) * v_robot;

                traj_all_[ NUM_MPC_STATE_INPUT * i + 3 ]  = traj_all_[ NUM_MPC_STATE_INPUT * ( i - 1 ) + 3 ] + dt_mpc_ * v_world[ 0 ];  // vel_des[ 0 ];
                traj_all_[ NUM_MPC_STATE_INPUT * i + 4 ]  = traj_all_[ NUM_MPC_STATE_INPUT * ( i - 1 ) + 4 ] + dt_mpc_ * v_world[ 1 ];  // vel_des[ 1 ];
                traj_all_[ NUM_MPC_STATE_INPUT * i + 9 ]  = v_world[ 0 ];
                traj_all_[ NUM_MPC_STATE_INPUT * i + 10 ] = v_world[ 1 ];
            }
        }
#else
#endif
        if ( solve_mpc_by_single_thread_ ) {
            gait_mpc_table_                       = mpc_table;
            start_solve_mpc_                      = true;
            solver_mpc_interface_->solve_success_ = false;
        }
    }
}

/**
 * @brief solve mpc in another cpu thread
 *
 */
void ConvexMpcLocoGaits::SolveMpcAnotherThread() {
    static std::mutex opt_data_mutex;

    // calu xy drag
    static float         pz_err = state_est_.position( 2 ) - pos_des_( 2 );
    static Vec3< float > vxy( state_est_.velocity_in_body_frame( 0 ), state_est_.velocity_in_body_frame( 1 ), 0 );
    static float         xy_comp_drag[ 2 ] = { x_comp_integral_, y_comp_integral_ };
    static float         weight_state[ NUM_MPC_STATE_INPUT ], weight_ctrl[ NUM_MPC_CTRL_INPUT ];
    static float         alpha = 4e-5;
    static Vec3< float > com_offset( 0.0, 0.0, 0.0 );
    static float         r_body2foot[ 12 ];

    vxy << state_est_.velocity_in_body_frame( 0 ), state_est_.velocity_in_body_frame( 1 ), 0;
    pz_err = state_est_.position( 2 ) - pos_des_( 2 );
    if ( vxy[ 0 ] > 0.3 || vxy[ 0 ] < -0.3 ) {
        comp_integral_ += user_params_->cmpc_x_drag * pz_err * dt_mpc_ / vxy[ 0 ];
        x_comp_integral_ = std::cos( state_est_.rpy( 2 ) ) * comp_integral_;
        y_comp_integral_ = std::sin( state_est_.rpy( 2 ) ) * comp_integral_;
    }
    xy_comp_drag[ 0 ] = x_comp_integral_;
    xy_comp_drag[ 1 ] = y_comp_integral_;

    if ( alpha > 1e-4 ) {
        std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
        alpha = 1e-5;
    }
    // set weight
    for ( int i = 0; i < NUM_MPC_CTRL_INPUT; i++ ) {
        weight_state[ i ] = user_params_->mpc_task_weight[ i ];
    }
#ifndef USE_TIME_VARING_MPC
    for ( int i = 0; i < NUM_MPC_CTRL_INPUT; i++ )
        weight_ctrl[ i ] = alpha;
#else
#endif
    // get foot pos on absolute body frame

    com_offset = state_est_.world2body_rotation_matrix.transpose() * user_params_->mpc_com_offset.cast< float >();
    for ( int i = 0; i < 12; i++ )
        r_body2foot[ i ] = foot_pos_feedback_[ i % 4 ][ i / 4 ] - ( state_est_.position[ i / 4 ] + com_offset[ i / 4 ] );
    // update params and data collect
    solver_mpc_interface_->SetupGravityDirectionCompensation( xy_comp_drag );

    solver_mpc_interface_->SetupMpcSolverParams( dt_mpc_, horizon_length_, mu_for_mpc_, user_params_->mpc_force_max, user_params_->mpc_body_inertia.cast< float >(), user_params_->mpc_body_mass,
                                                 weight_state, weight_ctrl );

    solver_mpc_interface_->UpdateMpcSolverData( state_est_.position, state_est_.velocity_in_world_frame, state_est_.rpy, state_est_.angular_velocity_in_world_frame, state_est_.orientation,
                                                r_body2foot, traj_all_, gait_mpc_table_ );

    // solve dense mpc
    solver_mpc_interface_->SolveDenseMpc();
    // update force and com trajectory for wbc
    if ( solver_mpc_interface_->solve_success_ ) {
        opt_data_mutex.lock();
        for ( int leg = 0; leg < 4; leg++ ) {
            foot_force_result_[ leg ]     = solver_mpc_interface_->optimal_solution_.optimal_force[ leg ];
            foot_force_leg_result_[ leg ] = -state_est_.world2body_rotation_matrix * foot_force_result_[ leg ];

            foot_force_des_[ leg ] = foot_force_result_[ leg ];
        }
        opt_trajectory_next_ = solver_mpc_interface_->optimal_solution_.optimal_state_next_horizon;
        opt_data_mutex.unlock();
    }
}

void ConvexMpcLocoGaits::SetLegKpKd() {
    kd_joint_for_mpc_.setZero();
    kp_leg_swing_for_mpc_.setZero();
    kd_leg_swing_for_mpc_.setZero();
    kp_leg_stance_for_mpc_.setZero();
    kd_leg_stance_for_mpc_.setZero();
    kp_leg_stance_for_mpc_wbc_.setZero();
    kd_leg_stance_for_mpc_wbc_.setZero();

    kd_joint_for_mpc_.diagonal()          = user_params_->mpc_only_joint_kd.cast< float >();
    kp_leg_swing_for_mpc_.diagonal()      = user_params_->mpc_only_swing_cartesian_kp.cast< float >();
    kd_leg_swing_for_mpc_.diagonal()      = user_params_->mpc_only_swing_cartesian_kd.cast< float >();
    kp_leg_stance_for_mpc_.diagonal()     = user_params_->mpc_only_stance_cartesian_kp.cast< float >();
    kd_leg_stance_for_mpc_.diagonal()     = user_params_->mpc_only_stance_cartesian_kd.cast< float >();
    kp_leg_stance_for_mpc_wbc_.diagonal() = user_params_->mpc_wbc_stance_cartesian_kp.cast< float >();
    kd_leg_stance_for_mpc_wbc_.diagonal() = user_params_->mpc_wbc_stance_cartesian_kd.cast< float >();
}

void ConvexMpcLocoGaits::SetFootDesire() {
    se_contact_states_ << 0.0, 0.0, 0.0, 0.0;
    for ( int i = 0; i < 4; i++ ) {
        if ( swing_states_( i ) > 0.0 ) {  // foot is in swing phase
            if ( foot_first_swing_[ i ] ) {
                foot_first_swing_[ i ] = false;
                foot_swing_trajectory_[ i ].SetInitialPosition( foot_pos_feedback_[ i ] );
                if ( i == 0 ) {
                    gait_step_iter_++;
                }
            }
            ComputeSwingTrajectory( i );
            SetFootDesireSwing( i );
        }
        else {  // foot is in contact phase
            foot_first_swing_[ i ] = true;
            SetFootDesireStance( i );
        }
    }

    // // DEBUG
    // for (int i = 0; i < 4; i++) {
    //   // std::cout << "foot_pos_des_[" << i << "]: " << foot_pos_des_[ i ].transpose() << std::endl;
    //   // std::cout << "foot_vel_des_[" << i << "]: " << foot_vel_des_[ i ].transpose() << std::endl;

    //   // std::cout << "pDes[" << i << "]: " << leg_ctrl_->commands_[ i ].p_des.transpose() << std::endl;
    //   // std::cout << "vDes[" << i << "]: " << leg_ctrl_->commands_[ i ].v_des.transpose() << std::endl;
    // }
}

void ConvexMpcLocoGaits::ComputeSwingTrajectory( int foot_num ) {
    static int i = 0;
    i            = foot_num;
    if ( user_params_->mpc_use_bezier >= 0.9 )  // use bezier traj
        foot_swing_trajectory_[ i ].ComputeSwingTrajectoryBezier( swing_states_( i ), swing_time_( i ) );

    else if ( user_params_->mpc_use_bezier >= 0.0 ) {  // use Tstair traj
        if ( user_params_->mpc_use_bezier >= 0.5 )     // use cubicbezier to connect Tstair
            foot_swing_trajectory_[ i ].SetUseBezier( 1 );
        else  // use cyclodial to connect Tstair
            foot_swing_trajectory_[ i ].SetUseBezier( 0 );
        foot_swing_trajectory_[ i ].ComputeSwingTrajectoryTstair( swing_states_( i ), swing_time_( i ) );
    }
    else {  // use Tsimilar traj
        foot_swing_trajectory_[ i ].ComputeSwingTrajectoryTsimilar( swing_states_( i ), swing_time_( i ) );
    }
}

void ConvexMpcLocoGaits::SetFootDesireSwing( int foot_num ) {
    static int           i = 0;
    static Vec3< float > foot_pos_des;
    static Vec3< float > foot_vel_des;
    static Vec3< float > foot_acc_des;
    static Vec3< float > foot_pos_des_leg;  // in leg frame
    static Vec3< float > foot_vel_des_leg;

    i            = foot_num;
    foot_pos_des = foot_swing_trajectory_[ i ].GetPosition();
    foot_vel_des = foot_swing_trajectory_[ i ].GetVelocity();
    foot_acc_des = foot_swing_trajectory_[ i ].GetAcceleration();

    foot_pos_des_leg = state_est_.world2body_rotation_matrix * ( foot_pos_des - state_est_.position ) - robot_model_->GetHipLocation( i );
    foot_vel_des_leg = state_est_.world2body_rotation_matrix * ( foot_vel_des - state_est_.velocity_in_world_frame );

    // set desire for WBC
    foot_pos_des_[ i ] = foot_pos_des;
    foot_vel_des_[ i ] = foot_vel_des;
    foot_acc_des_[ i ] = foot_acc_des;

    // set desird for leg controller
    leg_ctrl_->commands_[ i ].p_des = foot_pos_des_leg;
    leg_ctrl_->commands_[ i ].v_des = foot_vel_des_leg;
    if ( user_params_->use_wbc < 0.9 ) {
        leg_ctrl_->commands_[ i ].kp_cartesian = kp_leg_swing_for_mpc_;
        leg_ctrl_->commands_[ i ].kd_cartesian = kd_leg_swing_for_mpc_;
    }
    else {
        leg_ctrl_->commands_[ i ].kp_cartesian = 0.0 * kp_leg_swing_for_mpc_;
        leg_ctrl_->commands_[ i ].kd_cartesian = 0.0 * kd_leg_swing_for_mpc_;
    }
}

void ConvexMpcLocoGaits::SetFootDesireStance( int foot_num ) {
    static int           i = 0;
    static Vec3< float > foot_pos_des;
    static Vec3< float > foot_vel_des;
    static Vec3< float > foot_acc_des;
    static Vec3< float > foot_pos_des_leg;  // in leg frame
    static Vec3< float > foot_vel_des_leg;

    i            = foot_num;
    foot_pos_des = foot_swing_trajectory_[ i ].GetPosition();
    foot_vel_des = foot_swing_trajectory_[ i ].GetVelocity();
    foot_acc_des = foot_swing_trajectory_[ i ].GetAcceleration();

    foot_pos_des_leg = state_est_.world2body_rotation_matrix * ( foot_pos_des - state_est_.position ) - robot_model_->GetHipLocation( i );
    foot_vel_des_leg = state_est_.world2body_rotation_matrix * ( foot_vel_des - state_est_.velocity_in_world_frame );

    // set desird for leg controller
    leg_ctrl_->commands_[ i ].p_des = foot_pos_des_leg;
    leg_ctrl_->commands_[ i ].v_des = foot_vel_des_leg;
    if ( user_params_->use_wbc < 0.9 ) {
        leg_ctrl_->commands_[ i ].kp_cartesian       = kp_leg_stance_for_mpc_;
        leg_ctrl_->commands_[ i ].kd_cartesian       = kd_leg_stance_for_mpc_;
        leg_ctrl_->commands_[ i ].force_feed_forward = foot_force_leg_result_[ i ];
        leg_ctrl_->commands_[ i ].kd_joint           = kd_joint_for_mpc_;
    }
    else {
        leg_ctrl_->commands_[ i ].kp_cartesian = kp_leg_stance_for_mpc_wbc_;
        leg_ctrl_->commands_[ i ].kd_cartesian = kd_leg_stance_for_mpc_wbc_;
    }

    se_contact_states_( i ) = contact_states_( i );
}

void ConvexMpcLocoGaits::SetBodyDesireTrajectory() {
    if ( current_gait_ != GaitId::kStand && current_gait_ != GaitId::kStandNoPr && current_gait_ != GaitId::kStandPassive ) {
        if ( user_params_->wbc_use_mpc_traj < 0.9 ) {  // traj before MPC
            body_pos_des_ = pos_des_;

            body_vel_des_[ 0 ] = vel_des_[ 0 ];
            body_vel_des_[ 1 ] = vel_des_[ 1 ];
            body_vel_des_[ 2 ] = 0.;
        }
        else {  // traj after MPC
            static Vec12< float > _opt_traj_inter;
            _opt_traj_inter = 1.0 / interpolate_max_num_ * interpolate_count_ * ( opt_trajectory_next_ - opt_trajectory_pre_ ) + opt_trajectory_pre_;

            body_pos_des_( 0 ) = _opt_traj_inter[ 3 ];
            body_pos_des_( 1 ) = _opt_traj_inter[ 4 ];
            body_pos_des_( 2 ) = pos_des_( 2 );

            body_vel_des_[ 0 ] = _opt_traj_inter[ 9 ];
            body_vel_des_[ 1 ] = _opt_traj_inter[ 10 ];
            body_vel_des_[ 2 ] = 0.;

            interpolate_count_++;
            // for first run , max is 9, and then is 10
            if ( interpolate_count_ > interpolate_max_num_ ) {
                interpolate_count_   = 1;
                interpolate_max_num_ = 15;  // 10
                opt_trajectory_pre_  = opt_trajectory_next_;
            }
        }
        body_acc_des_.setZero();
        body_rpy_des_ = rpy_des_with_comp_;
        body_omg_des_.setZero();
        body_omg_des_[ 2 ] = vel_des_robot_( 2 );

        // Set ini_pos/rpy_des_ for STAND
        init_pos_des_ = state_est_.position;
        init_rpy_des_ = state_est_.rpy;
    }
    else {
        pos_des_      = init_pos_des_;
        body_pos_des_ = pos_des_;
        body_vel_des_.setZero();
        body_acc_des_.setZero();
        body_rpy_des_ = init_rpy_des_;
        body_omg_des_.setZero();
    }

    // // DEBUG
    // std::cout << "body_pos_des_: " << body_pos_des_.transpose() << std::endl;
    // std::cout << "body_vel_des_: " << body_vel_des_.transpose() << std::endl;
    // std::cout << "body_rpy_des_: " << body_rpy_des_.transpose() << std::endl;
    // std::cout << "body_omg_des_: " << body_omg_des_.transpose() << std::endl;
    // std::cout << "state_est_.position: " << state_est_.position.transpose() << std::endl;
    // std::cout << "state_est_.rpy: " << state_est_.rpy.transpose() << std::endl;
}

#if ( defined DRAW_DEBUG_SWINGS ) || ( defined DRAW_DEBUG_PATH )
void ConvexMpcLocoGaits::DrawResult( ControlFsmData< float >& data ) {
#ifdef DRAW_DEBUG_SWINGS

    for ( int i = 0; i < 4; i++ ) {
        if ( swing_states_( i ) > 0.0 ) {  // foot is in swing phase
            DrawSwingPath( data, i );
        }
        else {  // foot is in contact phase
            DrawStancePath( data, i );
        }
    }

#endif

#ifdef DRAW_DEBUG_PATH

    DrawMpcPath( data );

#endif
}
#endif

void ConvexMpcLocoGaits::DrawSwingPath( ControlFsmData< float >& data, int foot_num ) {
    int foot        = 0;
    foot            = foot_num;
    auto* debugPath = data.visualization_data->AddPath();
    if ( debugPath ) {
        debugPath->num_points = 100;
        debugPath->color      = { 0.2, 1, 0.2, 0.5 };
        float step            = ( 1.f - swing_states_( foot ) ) / 100.f;
        for ( int i = 0; i < 100; i++ ) {
            if ( user_params_->mpc_use_bezier >= 0.9 )
                foot_swing_trajectory_[ foot ].ComputeSwingTrajectoryBezier( swing_states_( foot ) + i * step, swing_time_( foot ) );
            else if ( user_params_->mpc_use_bezier >= 0.0 )
                foot_swing_trajectory_[ foot ].ComputeSwingTrajectoryTstair( swing_states_( foot ) + i * step, swing_time_( foot ) );
            else
                foot_swing_trajectory_[ foot ].ComputeSwingTrajectoryTsimilar( swing_states_( foot ) + i * step, swing_time_( foot ) );
            debugPath->position[ i ] = foot_swing_trajectory_[ foot ].GetPosition();
        }
    }
    auto* finalSphere = data.visualization_data->AddSphere();
    if ( finalSphere ) {
        finalSphere->position = foot_swing_trajectory_[ foot ].GetPosition();
        finalSphere->radius   = 0.02;
        finalSphere->color    = { 0.6, 0.6, 0.2, 0.7 };
    }
    if ( user_params_->mpc_use_bezier >= 0.9 )
        foot_swing_trajectory_[ foot ].ComputeSwingTrajectoryBezier( swing_states_( foot ), swing_time_( foot ) );
    else if ( user_params_->mpc_use_bezier >= 0.0 )
        foot_swing_trajectory_[ foot ].ComputeSwingTrajectoryTstair( swing_states_( foot ), swing_time_( foot ) );
    else
        foot_swing_trajectory_[ foot ].ComputeSwingTrajectoryTsimilar( swing_states_( foot ), swing_time_( foot ) );
    auto* actualSphere     = data.visualization_data->AddSphere();
    auto* goalSphere       = data.visualization_data->AddSphere();
    goalSphere->position   = foot_swing_trajectory_[ foot ].GetPosition();
    actualSphere->position = foot_pos_feedback_[ foot ];
    goalSphere->radius     = 0.02;
    actualSphere->radius   = 0.02;
    goalSphere->color      = { 0.2, 1, 0.2, 0.7 };
    actualSphere->color    = { 0.8, 0.2, 0.2, 0.7 };
}

void ConvexMpcLocoGaits::DrawStancePath( ControlFsmData< float >& data, int foot_num ) {
    static int foot        = 0;
    foot                   = foot_num;
    auto* actualSphere     = data.visualization_data->AddSphere();
    actualSphere->position = foot_pos_feedback_[ foot ];
    actualSphere->radius   = 0.02;
    actualSphere->color    = { 0.2, 0.2, 0.8, 0.7 };
}

void ConvexMpcLocoGaits::DrawMpcPath( ControlFsmData< float >& data ) {
    auto* trajectory_debug = data.visualization_data->AddPath();
    if ( trajectory_debug ) {
        trajectory_debug->num_points = 10;
        trajectory_debug->color      = { 0.2, 0.2, 0.7, 0.5 };
        for ( int i = 0; i < 10; i++ ) {
            trajectory_debug->position[ i ][ 0 ] = traj_all_[ NUM_MPC_STATE_INPUT * i + 3 ];
            trajectory_debug->position[ i ][ 1 ] = traj_all_[ NUM_MPC_STATE_INPUT * i + 4 ];
            trajectory_debug->position[ i ][ 2 ] = traj_all_[ NUM_MPC_STATE_INPUT * i + 5 ];
            auto* ball                           = data.visualization_data->AddSphere();
            ball->radius                         = 0.01;
            ball->position                       = trajectory_debug->position[ i ];
            ball->color                          = { 1.0, 0.2, 0.2, 0.5 };
        }
    }
}
