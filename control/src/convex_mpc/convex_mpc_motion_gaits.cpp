#include <iostream>
#include <parameters.hpp>

#include "utilities/control_utilities.hpp"
#include "utilities/pseudoInverse.hpp"
#include "utilities/timer.hpp"
#include "utilities/utilities.hpp"

#include "convex_mpc/convex_mpc_motion_gaits.hpp"

// #define DRAW_DEBUG_SWINGS
// #define DRAW_DEBUG_PATH

static const int gait_period_base = 10;

/**
 * @brief Construct a new Convex Mpc Motion Gaits:: Convex Mpc Motion Gaits object
 *
 * @param dt control time step
 * @param iterations_between_mpc iteration between mpc updates
 */
ConvexMpcMotionGaits::ConvexMpcMotionGaits( float dt, int iterations_between_mpc )
    : iter_between_mpc_( iterations_between_mpc ), horizon_length_( 10 ), dt_( dt ),
      special_trot_( int( gait_period_base * 1.2 ), Vec4< int >( 0, 6, 6, 0 ), Vec4< int >( 5, 5, 5, 5 ), "SpecialTrot" ),
      special_pronk_( int( gait_period_base * 1.2 ), Vec4< int >( 0, 0, 0, 0 ), Vec4< int >( 6, 6, 6, 6 ), "SpecialPronk" ),
      mini_special_pronk_( int( gait_period_base * 1.0 ), Vec4< int >( 0, 0, 0, 0 ), Vec4< int >( 6, 6, 6, 6 ), "MiniSpecialPronk" ),
      diagonal_left_( int( gait_period_base * 1.6 ), Vec4< int >( 0, 0, 0, 0 ), Vec4< int >( 9, 16, 16, 9 ), "DiagonalLeft" ),
      diagonal_right_( int( gait_period_base * 1.6 ), Vec4< int >( 0, 0, 0, 0 ), Vec4< int >( 16, 9, 9, 16 ), "DiagonalRight" ),
      trot_swing_( int( gait_period_base * 2.0 ), Vec4< int >( 0, 10, 10, 0 ), Vec4< int >( 12, 12, 12, 12 ), "TrotSwing" ),
      trot_in_out_( int( gait_period_base * 1.8 ), Vec4< int >( 0, 9, 9, 0 ), Vec4< int >( 11, 11, 11, 11 ), "TrotInOut" ),
      trot_pitch_( int( gait_period_base * 1.8 ), Vec4< int >( 0, 9, 9, 0 ), Vec4< int >( 11, 11, 11, 11 ), "TrotPitch" ),
      front_lift_in_( int( gait_period_base * 1.6 ), Vec4< int >( 0, 0, 0, 0 ), Vec4< int >( 9, 9, 16, 16 ), "FrontLiftIn" ),
      front_lift_left_( int( gait_period_base * 1.6 ), Vec4< int >( 0, 0, 0, 0 ), Vec4< int >( 9, 9, 16, 16 ), "FrontLiftLeft" ),
      front_lift_switch_( int( gait_period_base * 2.8 ), Vec4< int >( 14, 0, 0, 0 ), Vec4< int >( 12, 12, 28, 28 ), "FrontLiftSwitch" ),
      rear_lift_in_( int( gait_period_base * 1.6 ), Vec4< int >( 0, 0, 0, 0 ), Vec4< int >( 16, 16, 9, 9 ), "RearLiftIn" ),
      rear_lift_left_( int( gait_period_base * 1.6 ), Vec4< int >( 0, 0, 0, 0 ), Vec4< int >( 16, 16, 9, 9 ), "RearLiftLeft" ),
      rear_lift_switch_( int( gait_period_base * 2.8 ), Vec4< int >( 0, 0, 0, 14 ), Vec4< int >( 28, 28, 12, 12 ), "RearLiftSwitch" ),
      ballet_( gait_period_base, Vec4< int >( 0, 5, 5, 0 ), Vec4< int >( 6, 6, 6, 6 ), "Ballet" ),
      ballet_trans_( gait_period_base, Vec4< int >( 0, 5, 5, 0 ), Vec4< int >( 6, 6, 6, 6 ), "BalletTrans" ),
      pitch_down_left_( int( gait_period_base * 1.8 ), Vec4< int >( 0, 0, 0, 0 ), Vec4< int >( 18, 11, 18, 18 ), "PitchDownLeft" ),
      pitch_down_right_( int( gait_period_base * 1.8 ), Vec4< int >( 0, 0, 0, 0 ), Vec4< int >( 11, 18, 18, 18 ), "PitchDownRight" ),
      walk_wave_( int( gait_period_base * 3.6 ), Vec4< int >( 18, 0, 9, 27 ), Vec4< int >( 27, 27, 27, 27 ), "WalkWave" ),
      pace_stride_left_( int( gait_period_base * 1.2 ), Vec4< int >( 0, 6, 0, 6 ), Vec4< int >( 7, 7, 7, 7 ), "PaceStrideLeft" ),
      pace_stride_right_( int( gait_period_base * 1.2 ), Vec4< int >( 6, 0, 6, 0 ), Vec4< int >( 7, 7, 7, 7 ), "PaceStrideRight" ),
      moonwalk_left_( gait_period_base, Vec4< int >( 16, 8, 8, 16 ), Vec4< float >( 0.375, 0.75, 0.75, 0.375 ), "MoonwalkLeft" ),
      moonwalk_right_( gait_period_base, Vec4< int >( 8, 16, 16, 8 ), Vec4< float >( 0.75, 0.375, 0.375, 0.75 ), "MoonwalkRight" ),
      moonwalk_switch_left_( gait_period_base, Vec4< int >( 12, 12, 12, 12 ), Vec4< float >( 0.5, 0.75, 0.75, 0.5 ), "MoonwalkSwitchLeft" ),
      moonwalk_switch_right_( gait_period_base, Vec4< int >( 12, 12, 12, 12 ), Vec4< float >( 0.75, 0.5, 0.5, 0.75 ), "MoonwalkSwitchRight" ),
      user_gait_( GetUserGaitDefinePath() + "user_gait.toml", horizon_length_, iter_between_mpc_, "UserGait" ),
      user_gait_00_( GetUserGaitDefinePath() + "user_gait_00.toml", horizon_length_, iter_between_mpc_, "UserGait00" ),
      user_gait_01_( GetUserGaitDefinePath() + "user_gait_01.toml", horizon_length_, iter_between_mpc_, "UserGait01" ),
      user_gait_02_( GetUserGaitDefinePath() + "user_gait_02.toml", horizon_length_, iter_between_mpc_, "UserGait02" ),
      user_gait_03_( GetUserGaitDefinePath() + "user_gait_03.toml", horizon_length_, iter_between_mpc_, "UserGait03" ),
      user_gait_04_( GetUserGaitDefinePath() + "user_gait_04.toml", horizon_length_, iter_between_mpc_, "UserGait04" ),
      user_gait_05_( GetUserGaitDefinePath() + "user_gait_05.toml", horizon_length_, iter_between_mpc_, "UserGait05" ),
      user_gait_06_( GetUserGaitDefinePath() + "user_gait_06.toml", horizon_length_, iter_between_mpc_, "UserGait06" ),
      user_gait_07_( GetUserGaitDefinePath() + "user_gait_07.toml", horizon_length_, iter_between_mpc_, "UserGait07" ),
      user_gait_08_( GetUserGaitDefinePath() + "user_gait_08.toml", horizon_length_, iter_between_mpc_, "UserGait08" ),
      user_gait_09_( GetUserGaitDefinePath() + "user_gait_09.toml", horizon_length_, iter_between_mpc_, "UserGait09" ),
      user_gait_10_( GetUserGaitDefinePath() + "user_gait_10.toml", horizon_length_, iter_between_mpc_, "UserGait10" ),
      user_gait_11_( GetUserGaitDefinePath() + "user_gait_11.toml", horizon_length_, iter_between_mpc_, "UserGait11" ),
      user_gait_12_( GetUserGaitDefinePath() + "user_gait_12.toml", horizon_length_, iter_between_mpc_, "UserGait12" ),
      user_gait_13_( GetUserGaitDefinePath() + "user_gait_13.toml", horizon_length_, iter_between_mpc_, "UserGait13" ),
      user_gait_14_( GetUserGaitDefinePath() + "user_gait_14.toml", horizon_length_, iter_between_mpc_, "UserGait14" ),
      user_gait_15_( GetUserGaitDefinePath() + "user_gait_15.toml", horizon_length_, iter_between_mpc_, "UserGait15" ),
      user_gait_16_( GetUserGaitDefinePath() + "user_gait_16.toml", horizon_length_, iter_between_mpc_, "UserGait16" ),
      user_gait_17_( GetUserGaitDefinePath() + "user_gait_17.toml", horizon_length_, iter_between_mpc_, "UserGait17" ),
      user_gait_18_( GetUserGaitDefinePath() + "user_gait_18.toml", horizon_length_, iter_between_mpc_, "UserGait18" ),
      user_gait_19_( GetUserGaitDefinePath() + "user_gait_19.toml", horizon_length_, iter_between_mpc_, "UserGait19" ),
      user_gait_20_( GetUserGaitDefinePath() + "user_gait_20.toml", horizon_length_, iter_between_mpc_, "UserGait20" ) {
    dt_mpc_ = dt_ * iter_between_mpc_;
    printf( "[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt_, iter_between_mpc_, dt_mpc_ );

    vel_cmd_.setZero();
    pos_cmd_ << 0.0, 0.0, 0.3;
    rpy_cmd_.setZero();
    step_height_cmd_.setConstant( 0.05 );
    gait_cmd_      = GaitId::kSpecialPronk;
    omni_cmd_      = false;
    duration_mode_ = false;

    body_pos_des_ << 0.0, 0.0, 0.3;
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
    vel_transition_        = 0.2;

    contact_states_.setOnes();
    swing_states_.setZero();

    floating_base_weight_wbc_.setZero();
    mu_for_wbc_   = 0.4;
    mu_for_mpc_   = mu_for_wbc_;
    landing_gain_ = 1.0;

    kp_body_for_wbc_.setConstant( 60.0 );
    kd_body_for_wbc_.setConstant( 6.0 );
    kp_ori_for_wbc_.setConstant( 60.0 );
    kd_ori_for_wbc_.setConstant( 6.0 );
    kp_foot_for_wbc_.setConstant( 300.0 );
    kd_foot_for_wbc_.setConstant( 36.0 );
    kp_joint_for_wbc_.setConstant( 3.0 );
    kd_joint_for_wbc_.setConstant( 1.0 );

    solver_mpc_interface_ = new SolveMpcInterface();

    solve_mpc_by_single_thread_ = false;
    start_solve_mpc_            = false;
}

/**
 * @brief Destroy the Convex Mpc Motion Gaits:: Convex Mpc Motion Gaits object
 *
 */
ConvexMpcMotionGaits::~ConvexMpcMotionGaits() {}

/**
 * @brief Initialize all variables
 *
 * @param data robot data
 */
void ConvexMpcMotionGaits::Initialize( ControlFsmData< float >& data ) {
    user_params_     = data.user_parameters;
    robot_params_    = data.control_parameters;
    state_estimator_ = data.state_estimator;
    ctrl_cmd_        = data.command;
    state_est_       = data.state_estimator->GetResult();
    robot_model_     = data.quadruped;
    gait_type_       = &special_trot_;
    leg_ctrl_        = data.leg_controller;

    se_ori_cali_offset_ = Vec3< float >::Zero();
    se_ori_cali_gain_   = Vec3< float >::Zero();

    gait_step_iter_ = 0;
    loco_iter_      = 0;
    gait_iter_      = 0;
    gait_step_iter_leg_.setZero();

    toggle_every_step_ = 1;

    gait_first_step_ = true;
    gait_first_run_  = true;

    step_height_des_ = step_height_cmd_;
    step_height_max_ = 0.08;
    step_height_min_ = 0.00;
    for ( int i = 0; i < 4; i++ ) {
        swing_mid_pos_[ i ] << 0.0, 0.0, 0.05;
    }
    use_swing_mid_pos_ = false;

    vel_des_.setZero();
    vel_des_robot_ = vel_des_;
    vel_des_last_  = vel_des_;
    vel_cmd_max_ << 2.0, 0.5, 0.5;
    vel_cmd_min_ = -1.0f * vel_cmd_max_;
    acc_cmd_max_ << 1.0, 1.0, 2.0;
    acc_cmd_min_ = -1.0f * acc_cmd_max_;
    pos_cmd_ << 0.0, 0.0, user_params_->des_roll_pitch_height_motion[ 2 ];
    pos_des_      = pos_cmd_;
    pos_des_last_ = state_est_.position;
    pos_cmd_max_ << 0.0, 0.0, user_params_->des_roll_pitch_height_motion[ 2 ] + 0.07;  // Only z is enabled
    pos_cmd_min_ << 0.0, 0.0, 0.16;
    rpy_des_.setZero();
    rpy_des_last_ = state_est_.rpy;
    rpy_cmd_max_ << 0.0, 0.2, 0.0;
    rpy_cmd_min_ = -1.0f * rpy_cmd_max_;

    yaw_integral_       = 0.0;
    yaw_des_last_       = 0.0;
    body_height_filter_ = 0.01;
    body_pitch_filter_  = 0.01;

    rpy_integral_.setZero();
    rpy_comp_.setZero();

    for ( int i = 0; i < 4; i++ ) {
        foot_pos_feedback_[ i ] = state_est_.position + state_est_.world2body_rotation_matrix.transpose() * ( robot_model_->GetHipLocation( i ) + leg_ctrl_->datas_[ i ].p );
        foot_first_swing_[ i ]  = false;
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

    se_contact_states_.setZero();
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

    wbc_user_mpc_traj_gait_select_ = false;
    wbc_use_vertical_traj_         = false;

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

    robot_type_ = data.quadruped->robot_type_;

    // // DEBUG
    // std::cout << "ConvexMpcMotionGaits initialization finished." << std::endl;
}

/**
 * @brief Initialized all user and nonperiodic gaits
 *
 * @param file_string a string that contains user gait defination
 */
void ConvexMpcMotionGaits::InitUserGaits( bool file_string ) {
    Timer t_user_gait;
    if ( file_string ) {
        user_gait_.InitGait( ctrl_cmd_->user_gait_file, horizon_length_, iter_between_mpc_, "UserGait" );
        std::cout << "[ConvexMpcMotionGaits] Initialized user_gait by LCM string data!" << std::endl;
        return;
    }
    user_gait_.InitGait( GetUserGaitDefinePath() + "user_gait.toml", horizon_length_, iter_between_mpc_, "UserGait" );
    user_gait_00_.InitGait( GetUserGaitDefinePath() + "user_gait_00.toml", horizon_length_, iter_between_mpc_, "UserGait00" );
    user_gait_01_.InitGait( GetUserGaitDefinePath() + "user_gait_01.toml", horizon_length_, iter_between_mpc_, "UserGait01" );
    user_gait_02_.InitGait( GetUserGaitDefinePath() + "user_gait_02.toml", horizon_length_, iter_between_mpc_, "UserGait02" );
    user_gait_03_.InitGait( GetUserGaitDefinePath() + "user_gait_03.toml", horizon_length_, iter_between_mpc_, "UserGait03" );
    user_gait_04_.InitGait( GetUserGaitDefinePath() + "user_gait_04.toml", horizon_length_, iter_between_mpc_, "UserGait04" );
    user_gait_05_.InitGait( GetUserGaitDefinePath() + "user_gait_05.toml", horizon_length_, iter_between_mpc_, "UserGait05" );
    user_gait_06_.InitGait( GetUserGaitDefinePath() + "user_gait_06.toml", horizon_length_, iter_between_mpc_, "UserGait06" );
    user_gait_07_.InitGait( GetUserGaitDefinePath() + "user_gait_07.toml", horizon_length_, iter_between_mpc_, "UserGait07" );
    user_gait_08_.InitGait( GetUserGaitDefinePath() + "user_gait_08.toml", horizon_length_, iter_between_mpc_, "UserGait08" );
    user_gait_09_.InitGait( GetUserGaitDefinePath() + "user_gait_09.toml", horizon_length_, iter_between_mpc_, "UserGait09" );
    user_gait_10_.InitGait( GetUserGaitDefinePath() + "user_gait_10.toml", horizon_length_, iter_between_mpc_, "UserGait10" );
    user_gait_11_.InitGait( GetUserGaitDefinePath() + "user_gait_11.toml", horizon_length_, iter_between_mpc_, "UserGait11" );
    user_gait_12_.InitGait( GetUserGaitDefinePath() + "user_gait_12.toml", horizon_length_, iter_between_mpc_, "UserGait12" );
    user_gait_13_.InitGait( GetUserGaitDefinePath() + "user_gait_13.toml", horizon_length_, iter_between_mpc_, "UserGait13" );
    user_gait_14_.InitGait( GetUserGaitDefinePath() + "user_gait_14.toml", horizon_length_, iter_between_mpc_, "UserGait14" );
    user_gait_15_.InitGait( GetUserGaitDefinePath() + "user_gait_15.toml", horizon_length_, iter_between_mpc_, "UserGait15" );
    user_gait_16_.InitGait( GetUserGaitDefinePath() + "user_gait_16.toml", horizon_length_, iter_between_mpc_, "UserGait16" );
    user_gait_17_.InitGait( GetUserGaitDefinePath() + "user_gait_17.toml", horizon_length_, iter_between_mpc_, "UserGait17" );
    user_gait_18_.InitGait( GetUserGaitDefinePath() + "user_gait_18.toml", horizon_length_, iter_between_mpc_, "UserGait18" );
    user_gait_19_.InitGait( GetUserGaitDefinePath() + "user_gait_19.toml", horizon_length_, iter_between_mpc_, "UserGait19" );
    user_gait_20_.InitGait( GetUserGaitDefinePath() + "user_gait_20.toml", horizon_length_, iter_between_mpc_, "UserGait20" );
    std::cout << "[ConvexMpcMotionGaits] Initialized all user gaits in " << t_user_gait.GetElapsedMilliseconds() << " ms." << std::endl;
}

/**
 * @brief Run mpc
 *
 * @param data robot data
 */
template <> void ConvexMpcMotionGaits::RunMpc( ControlFsmData< float >& data ) {
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

    SetRpyComp();
    SetLandingPos();
    CheckGaitTransition();
    ComputeMpc();

    SetLegKpKd();
    SetFootDesire();

    SetContactStatesForEstimator();

    SetBodyDesireTrajectory();

#if ( defined DRAW_DEBUG_SWINGS ) || ( defined DRAW_DEBUG_PATH )
    DrawResult( data );
#endif
}

void ConvexMpcMotionGaits::SetCurrentGait() {
    state_est_ = state_estimator_->GetResult();

    if ( gait_check_transition_ ) {
        gait_cmd_ = ctrl_cmd_->gait_id;
        omni_cmd_ = false;  // x y cmd are in current robot frame
    }

    if ( current_gait_ != gait_cmd_ ) {
        ResetIter();
    }

    current_gait_ = gait_cmd_;
}

void ConvexMpcMotionGaits::AccumulateIteration() {
    loco_iter_++;
    gait_iter_++;
}

void ConvexMpcMotionGaits::ResetIter() {
    // loco_iter_ = 0;
    gait_step_iter_ = 0;
    gait_iter_      = 0;
    gait_step_iter_leg_.setZero();
}

void ConvexMpcMotionGaits::SetGaitParams( const int gait_id ) {
    SetDefaultParams();
    switch ( gait_id ) {
    case GaitId::kSpecialPronk:
        SetSpecialPronkParams();
        break;
    case GaitId::kSpecialTrot:
        SetSpecialTrotParams();
        break;
    case GaitId::kDiagonalLeft:
        SetDiagonalLeftParams();
        break;
    case GaitId::kDiagonalRight:
        SetDiagonalRightParams();
        break;
    case GaitId::kTrotSwing:
        SetTrotSwingParams();
        break;
    case GaitId::kTrotInOut:
        SetTrotInOutParams();
        break;
    case GaitId::kTrotPitch:
        SetTrotPitchParams();
        break;
    case GaitId::kWalkWave:
        SetWalkWaveParams();
        break;
    case GaitId::kPaceStrideLeft:
        SetPaceStrideLeftParams();
        break;
    case GaitId::kPaceStrideRight:
        SetPaceStrideRightParams();
        break;
    case GaitId::kMoonwalkLeft:
        SetMoonwalkLeftParams();
        break;
    case GaitId::kMoonwalkRight:
        SetMoonwalkRightParams();
        break;
    case GaitId::kMoonSwitchLeft:
        SetMoonwalkSwitchLeftParams();
        break;
    case GaitId::kMoonSwitchRight:
        SetMoonwalkSwitchRightParams();
        break;
    case GaitId::kFrontLiftIn:
        SetFrontLiftInParams();
        break;
    case GaitId::kFrontLiftLeft:
        SetFrontLiftLeftParams();
        break;
    case GaitId::kFrontLiftSwitch:
        SetFrontLiftSwitchParams();
        break;
    case GaitId::kRearLiftIn:
        SetRearLiftInParams();
        break;
    case GaitId::kRearLiftLeft:
        SetRearLiftLeftParams();
        break;
    case GaitId::kRearLiftSwitch:
        SetRearLiftSwitchParams();
        break;
    case GaitId::kBallet:
        SetBalletParams();
        break;
    case GaitId::kBalletTrans:
        SetBalletTransParams();
        break;
    case GaitId::kPitchDownLeft:
        SetPitchDownLeftParams();
        break;
    case GaitId::kPitchDownRight:
        SetPitchDownRightParams();
        break;
    case GaitId::kUserGait:
    case GaitId::kUserGait00:
    case GaitId::kUserGait01:
    case GaitId::kUserGait02:
    case GaitId::kUserGait03:
    case GaitId::kUserGait04:
    case GaitId::kUserGait05:
    case GaitId::kUserGait06:
    case GaitId::kUserGait07:
    case GaitId::kUserGait08:
    case GaitId::kUserGait09:
    case GaitId::kUserGait10:
    case GaitId::kUserGait11:
    case GaitId::kUserGait12:
    case GaitId::kUserGait13:
    case GaitId::kUserGait14:
    case GaitId::kUserGait15:
    case GaitId::kUserGait16:
    case GaitId::kUserGait17:
    case GaitId::kUserGait18:
    case GaitId::kUserGait19:
    case GaitId::kUserGait20:
        SetUserGait( gait_id );
        break;
    default:
        SetDefaultParams();
        break;
    }
}

void ConvexMpcMotionGaits::SetDefaultParams() {
    gait_type_ = &trot_in_out_;

    vel_cmd_.setZero();

    for ( int i = 0; i < 3; i++ ) {
        vel_cmd_min_( i ) = user_params_->vel_xy_yaw_min_motion_default[ i ];
        vel_cmd_max_( i ) = user_params_->vel_xy_yaw_max_motion_default[ i ];
        acc_cmd_min_( i ) = user_params_->acc_xy_yaw_min[ i ];
        acc_cmd_max_( i ) = user_params_->acc_xy_yaw_max[ i ];
    }
    rpy_cmd_min_ << 0.0, -0.2, 0.0;  // only pitch is enabled
    rpy_cmd_max_ << 0.0, 0.2, 0.0;

    step_height_max_ = ( float )user_params_->step_height_max;
    step_height_min_ = 0.0;
    step_height_cmd_.setConstant( 0.05 );

    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ].setZero();
    }
    landing_pos_ratio_last_ = landing_pos_ratio_;
    landing_pos_ratio_.setOnes();

    landing_gain_params_( 0 ) = user_params_->cmpc_bonus_swing;  // x kp and kd
    landing_gain_params_( 1 ) = user_params_->swing_p_gain_motion( 0 );
    landing_gain_params_( 2 ) = user_params_->cmpc_bonus_swing;  // y kp and kd
    landing_gain_params_( 3 ) = user_params_->swing_p_gain_motion( 1 );

    trans_state_ = 0.01;

    max_pos_err_mpc_ << 0.1, 0.1, 0.0;

    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_motion_default( i );
    }

    rpy_cmd_[ 1 ]       = 0.0;
    body_pitch_filter_  = 0.01;
    pos_cmd_[ 2 ]       = user_params_->des_roll_pitch_height_motion[ 2 ];
    body_height_filter_ = 0.01;

    use_swing_mid_pos_ = false;

    vel_transition_ = 0.2;
    mu_for_wbc_     = user_params_->wbc_friction_default;
    mu_for_mpc_     = user_params_->mpc_friction_coef;
    landing_gain_   = 1.0;

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

    wbc_user_mpc_traj_gait_select_ = false;
    wbc_use_vertical_traj_         = false;
}

void ConvexMpcMotionGaits::SetSpecialTrotParams() {
    gait_type_ = &special_trot_;

    vel_cmd_max_ << 0.0, 0.0, 0.0;
    vel_cmd_min_ << 0.0, 0.0, 0.0;

    // step height
    if ( gait_first_step_ ) {
        step_height_cmd_.setConstant( 0.005 );
    }
    else {
        step_height_cmd_.setConstant( 0.05 );
    }

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        if ( fmod( gait_step_iter_, 4 ) <= 1 ) {
            if ( i == 0 ) {
                landing_pos_offset_[ i ] << user_params_->special_trot_landing_offset( 0 ), user_params_->special_trot_landing_offset( 1 ), 0.0;
            }
            else {
                landing_pos_offset_[ i ] << -user_params_->special_trot_landing_offset( 0 ), -user_params_->special_trot_landing_offset( 1 ), 0.0;
            }
        }
        else {
            if ( i == 1 ) {
                landing_pos_offset_[ i ] << user_params_->special_trot_landing_offset( 0 ), user_params_->special_trot_landing_offset( 1 ), 0.0;
            }
            else {
                landing_pos_offset_[ i ] << -user_params_->special_trot_landing_offset( 0 ), -user_params_->special_trot_landing_offset( 1 ), 0.0;
            }
        }
        landing_pos_offset_[ i ] << leg_sign_x_[ i ] * landing_pos_offset_[ i ][ 0 ], leg_sign_y_[ i ] * landing_pos_offset_[ i ][ 1 ], landing_pos_offset_[ i ][ 2 ];
    }

    trans_state_ = 0.1;
}

void ConvexMpcMotionGaits::SetSpecialPronkParams() {
    gait_type_ = robot_type_ == RobotType::CYBERDOG2 ? &mini_special_pronk_ : &special_pronk_;

    vel_cmd_max_ << 0.0, 0.0, 0.0;
    vel_cmd_min_ << 0.0, 0.0, 0.0;

    // step height
    for ( int i = 0; i < 4; i++ ) {
        if ( i == 0 || i == 3 ) {
            step_height_cmd_[ i ] = 0.01 + ( toggle_every_step_ > 0 ? user_params_->special_pronk_height : 0.0 );
        }
        else {
            step_height_cmd_[ i ] = 0.01 + ( toggle_every_step_ < 0 ? user_params_->special_pronk_height : 0.0 );
        }
    }

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        if ( !foot_first_swing_[ i ] ) {  // compute landing pos only in swing phase
            if ( i == 0 || i == 3 ) {
                if ( gait_step_iter_ == 1 ) {
                    landing_pos_offset_[ i ] << toggle_every_step_ * user_params_->special_pronk_size, 0.0, 0.0;
                }
                else {
                    landing_pos_offset_[ i ] << 2.0 * toggle_every_step_ * user_params_->special_pronk_size, 0.0, 0.0;
                }
            }
            else {
                if ( gait_step_iter_ == 1 ) {
                    landing_pos_offset_[ i ] << -toggle_every_step_ * user_params_->special_pronk_size, 0.0, 0.0;
                }
                else {
                    landing_pos_offset_[ i ] << -2.0 * toggle_every_step_ * user_params_->special_pronk_size, 0.0, 0.0;
                }
            }
        }
    }

    // torso height
    pos_cmd_[ 2 ]       = user_params_->des_roll_pitch_height_motion[ 2 ] + 0.03;
    body_height_filter_ = 0.01;

    trans_state_ = 0.55;
}

void ConvexMpcMotionGaits::SetDiagonalLeftParams() {
    gait_type_ = &diagonal_left_;

    // torso moves when leg swing, keeps steady when all legs are in support phase
    if ( fmod( gait_step_iter_leg_( 0 ), 2 ) == 1 && swing_states_( 0 ) > 0.01 ) {
        vel_cmd_( 2 ) = user_params_->diagonal_yaw_rate[ 0 ];
    }
    else if ( fmod( gait_step_iter_leg_( 0 ), 2 ) == 0 && swing_states_( 0 ) > 0.01 ) {
        vel_cmd_( 2 ) = user_params_->diagonal_yaw_rate[ 1 ];
    }
    else {
        vel_cmd_( 2 ) = 0.0;
    }

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ].setZero();
        if ( fmod( gait_step_iter_leg_( 0 ), 2 ) == 1 ) {
            if ( i == 0 || i == 3 )  // FR and RL stretch
                landing_pos_offset_[ i ] << user_params_->diagonal_landing_offset[ 0 ], user_params_->diagonal_landing_offset[ 1 ], 0.0;
            else
                landing_pos_offset_[ i ] << 0.0, 0.0, 0.0;
        }
        else {
            landing_pos_offset_[ i ] << 0.0, 0.0, 0.0;
        }
        landing_pos_offset_[ i ] << leg_sign_x_[ i ] * landing_pos_offset_[ i ][ 0 ], leg_sign_y_[ i ] * landing_pos_offset_[ i ][ 1 ], landing_pos_offset_[ i ][ 2 ];
    }

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_diagonal( i );
    }
}

void ConvexMpcMotionGaits::SetDiagonalRightParams() {
    gait_type_ = &diagonal_right_;

    // torso moves when leg swing, keeps steady when all legs are in support phase
    if ( fmod( gait_step_iter_leg_( 1 ), 2 ) == 0 && swing_states_( 1 ) > 0.01 ) {
        vel_cmd_( 2 ) = -user_params_->diagonal_yaw_rate[ 0 ];
    }
    else if ( fmod( gait_step_iter_leg_( 1 ), 2 ) == 1 && swing_states_( 1 ) > 0.01 ) {
        vel_cmd_( 2 ) = -user_params_->diagonal_yaw_rate[ 1 ];
    }
    else {
        vel_cmd_( 2 ) = 0.0;
    }

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ].setZero();
        if ( fmod( gait_step_iter_leg_( 1 ), 2 ) == 1 ) {
            if ( i == 1 || i == 2 )  // FL and RR stretch
                landing_pos_offset_[ i ] << user_params_->diagonal_landing_offset[ 0 ], user_params_->diagonal_landing_offset[ 1 ], 0.0;
            else
                landing_pos_offset_[ i ] << 0.0, 0.0, 0.0;
        }
        else {
            landing_pos_offset_[ i ] << 0.0, 0.0, 0.0;
        }
        landing_pos_offset_[ i ] << leg_sign_x_[ i ] * landing_pos_offset_[ i ][ 0 ], leg_sign_y_[ i ] * landing_pos_offset_[ i ][ 1 ], landing_pos_offset_[ i ][ 2 ];
    }

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_diagonal( i );
    }
}

void ConvexMpcMotionGaits::SetTrotSwingParams() {
    gait_type_ = &trot_swing_;

    for ( int i = 0; i < 4; i++ ) {
        if ( fmod( gait_step_iter_, 8 ) < 4 ) {
            swing_mid_pos_[ i ] << leg_sign_x_[ i ] * user_params_->trot_swing_middle_pos[ 0 ], leg_sign_y_[ i ] * user_params_->trot_swing_middle_pos[ 1 ], user_params_->trot_swing_middle_pos[ 2 ];
        }
        else {
            swing_mid_pos_[ i ] << leg_sign_x_[ i ] * user_params_->trot_swing_middle_pos[ 0 ], -leg_sign_y_[ i ] * user_params_->trot_swing_middle_pos[ 1 ], user_params_->trot_swing_middle_pos[ 2 ];
        }
        swing_mid_pos_[ i ] = state_est_.world2body_rotation_matrix.transpose() * swing_mid_pos_[ i ];
    }
    use_swing_mid_pos_ = true;

    wbc_user_mpc_traj_gait_select_ = true;
}

void ConvexMpcMotionGaits::SetTrotInOutParams() {
    gait_type_ = &trot_in_out_;

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        if ( fmod( gait_step_iter_, 4 ) == 0 || fmod( gait_step_iter_, 4 ) == 1 ) {
            landing_pos_offset_[ i ] << 0.0, -user_params_->trot_in_out_landing_offset[ 1 ], 0.0;
        }
        else {
            landing_pos_offset_[ i ] << user_params_->diagonal_landing_offset[ 0 ], user_params_->diagonal_landing_offset[ 1 ], 0.0;
        }
        landing_pos_offset_[ i ] << leg_sign_x_[ i ] * landing_pos_offset_[ i ][ 0 ], leg_sign_y_[ i ] * landing_pos_offset_[ i ][ 1 ], landing_pos_offset_[ i ][ 2 ];
    }
}

void ConvexMpcMotionGaits::SetTrotPitchParams() {
    gait_type_ = &trot_pitch_;

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        if ( ( fmod( gait_step_iter_, 4 ) == 1 || fmod( gait_step_iter_, 4 ) == 2 ) && ( i == 0 || i == 1 ) ) {
            landing_pos_offset_[ i ] << user_params_->trot_pitch_landing_offset[ 0 ] + user_params_->trot_pitch_landing_offset[ 1 ], 0.0, 0.0;
        }
        else {
            landing_pos_offset_[ i ] << user_params_->trot_pitch_landing_offset[ 0 ], 0.0, 0.0;
        }
        landing_pos_offset_[ i ] << leg_sign_x_[ i ] * landing_pos_offset_[ i ][ 0 ], leg_sign_y_[ i ] * landing_pos_offset_[ i ][ 1 ], landing_pos_offset_[ i ][ 2 ];
    }

    // torso pitch
    if ( fmod( gait_step_iter_, 4 ) == 1 || fmod( gait_step_iter_, 4 ) == 2 ) {
        rpy_cmd_[ 1 ] = user_params_->trot_pitch_up_down[ 1 ];
    }
    else {
        rpy_cmd_[ 1 ] = user_params_->trot_pitch_up_down[ 0 ];
    }
    body_pitch_filter_ = 0.01;
}

void ConvexMpcMotionGaits::SetWalkWaveParams() {
    gait_type_ = &walk_wave_;

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ] << user_params_->walk_wave_landing_offset( 0 ), user_params_->walk_wave_landing_offset( 1 ), 0.0;
        landing_pos_offset_[ i ] << leg_sign_x_[ i ] * landing_pos_offset_[ i ][ 0 ], leg_sign_y_[ i ] * landing_pos_offset_[ i ][ 1 ], landing_pos_offset_[ i ][ 2 ];
    }

    // torso pitch and height
    if ( fmod( gait_step_iter_, 2 ) == 1 ) {
        rpy_cmd_[ 1 ] = user_params_->walk_wave_pitch( 0 );
        pos_cmd_[ 2 ] = user_params_->des_roll_pitch_height_motion[ 2 ] + user_params_->walk_wave_height[ 0 ] + user_params_->walk_wave_height[ 1 ];
    }
    else {
        rpy_cmd_[ 1 ] = user_params_->walk_wave_pitch( 1 );
        pos_cmd_[ 2 ] = user_params_->des_roll_pitch_height_motion[ 2 ] + user_params_->walk_wave_height[ 0 ] + user_params_->walk_wave_height[ 2 ];
    }

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_walk_wave( i );
    }

    body_pitch_filter_  = 0.01;
    body_height_filter_ = 0.01;
}

void ConvexMpcMotionGaits::SetPaceStrideLeftParams() {
    gait_type_ = &pace_stride_left_;

    if ( swing_states_( 1 ) >= 0.0 ) {
        vel_cmd_( 1 ) = user_params_->pace_stride_vel_cmd( 1 );
        vel_cmd_( 2 ) = user_params_->pace_stride_vel_cmd( 2 );
    }
    else {
        vel_cmd_( 1 ) = -user_params_->pace_stride_vel_cmd( 1 );
        vel_cmd_( 2 ) = user_params_->pace_stride_vel_cmd( 2 );
    }

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ] << 0.0, user_params_->pace_stride_landing_offset_y( i ), 0.0;
    }

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_pace_stride( i );
    }

    vel_transition_ = 0.35;

    wbc_user_mpc_traj_gait_select_ = true;
}

void ConvexMpcMotionGaits::SetPaceStrideRightParams() {
    gait_type_ = &pace_stride_right_;

    vel_transition_ = 0.35;
}

void ConvexMpcMotionGaits::SetMoonwalkLeftParams() {
    gait_type_ = &moonwalk_left_;

    // step height
    step_height_max_ = ( float )user_params_->moonwalk_step_height[ 0 ];
    for ( int i = 0; i < 4; i++ ) {
        if ( i == 0 || i == 3 ) {
            step_height_cmd_[ i ] = user_params_->moonwalk_step_height[ 0 ];
        }
        else {
            step_height_cmd_[ i ] = user_params_->moonwalk_step_height[ 1 ];
        }
    }

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        if ( i == 0 || i == 3 ) {  // single-beat legs
            if ( fmod( gait_step_iter_leg_( 0 ), 2 ) == 1 ) {
                landing_pos_offset_[ i ] << user_params_->moonwalk_landing_offset[ 0 ] + user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
            else {
                landing_pos_offset_[ i ] << -user_params_->moonwalk_landing_offset[ 0 ] + user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
        }
        else {  // double-beat legs
            if ( fmod( gait_step_iter_leg_( 1 ), 4 ) == 1 ) {
                landing_pos_offset_[ i ] << user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
            else if ( fmod( gait_step_iter_leg_( 1 ), 4 ) == 2 ) {
                landing_pos_offset_[ i ] << -user_params_->moonwalk_landing_offset[ 0 ] + user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
            else if ( fmod( gait_step_iter_leg_( 1 ), 4 ) == 3 ) {
                landing_pos_offset_[ i ] << user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
            else {
                landing_pos_offset_[ i ] << user_params_->moonwalk_landing_offset[ 0 ] + user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
        }
    }

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_moonwalk( i );
    }
}

void ConvexMpcMotionGaits::SetMoonwalkRightParams() {
    gait_type_ = &moonwalk_right_;

    // step height
    step_height_max_ = ( float )user_params_->moonwalk_step_height[ 0 ];
    for ( int i = 0; i < 4; i++ ) {
        if ( i == 1 || i == 2 ) {
            step_height_cmd_[ i ] = user_params_->moonwalk_step_height[ 0 ];
        }
        else {
            step_height_cmd_[ i ] = user_params_->moonwalk_step_height[ 1 ];
        }
    }

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        if ( i == 1 || i == 2 ) {  // single-beat legs
            if ( fmod( gait_step_iter_leg_( 1 ), 2 ) == 1 ) {
                landing_pos_offset_[ i ] << user_params_->moonwalk_landing_offset[ 0 ] + user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
            else {
                landing_pos_offset_[ i ] << -user_params_->moonwalk_landing_offset[ 0 ] + user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
        }
        else {  // double-beat legs
            if ( fmod( gait_step_iter_leg_( 0 ), 4 ) == 1 ) {
                landing_pos_offset_[ i ] << user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
            else if ( fmod( gait_step_iter_leg_( 0 ), 4 ) == 2 ) {
                landing_pos_offset_[ i ] << -user_params_->moonwalk_landing_offset[ 0 ] + user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
            else if ( fmod( gait_step_iter_leg_( 0 ), 4 ) == 3 ) {
                landing_pos_offset_[ i ] << user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
            else {
                landing_pos_offset_[ i ] << user_params_->moonwalk_landing_offset[ 0 ] + user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
        }
    }

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_moonwalk( i );
    }
}

void ConvexMpcMotionGaits::SetMoonwalkSwitchLeftParams() {
    gait_type_ = &moonwalk_switch_left_;

    // step height
    step_height_max_ = ( float )user_params_->moonwalk_step_height[ 0 ];
    for ( int i = 0; i < 4; i++ ) {
        if ( i == 0 || i == 3 ) {
            step_height_cmd_[ i ] = 0.5 * user_params_->moonwalk_step_height[ 0 ];
        }
        else {
            step_height_cmd_[ i ] = 0.5 * user_params_->moonwalk_step_height[ 1 ];
        }
    }

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        if ( i == 0 || i == 3 ) {  // front rear switch
            if ( fmod( gait_step_iter_leg_( 0 ), 2 ) == 1 ) {
                landing_pos_offset_[ i ] << 0.5 * user_params_->moonwalk_landing_offset[ 0 ] + user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
            else {
                landing_pos_offset_[ i ] << -0.5 * user_params_->moonwalk_landing_offset[ 0 ] + user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
        }
        else {  // stepping
            landing_pos_offset_[ i ] << user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
        }
    }

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_moonwalk( i );
    }
}

void ConvexMpcMotionGaits::SetMoonwalkSwitchRightParams() {
    gait_type_ = &moonwalk_switch_right_;

    // step height
    step_height_max_ = ( float )user_params_->moonwalk_step_height[ 0 ];
    for ( int i = 0; i < 4; i++ ) {
        if ( i == 1 || i == 2 ) {
            step_height_cmd_[ i ] = 0.5 * user_params_->moonwalk_step_height[ 0 ];
        }
        else {
            step_height_cmd_[ i ] = 0.5 * user_params_->moonwalk_step_height[ 1 ];
        }
    }

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        if ( i == 1 || i == 2 ) {  // front rear switch
            if ( fmod( gait_step_iter_leg_( 1 ), 2 ) == 1 ) {
                landing_pos_offset_[ i ] << 0.5 * user_params_->moonwalk_landing_offset[ 0 ] + user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
            else {
                landing_pos_offset_[ i ] << -0.5 * user_params_->moonwalk_landing_offset[ 0 ] + user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
            }
        }
        else {  // stepping
            landing_pos_offset_[ i ] << user_params_->moonwalk_landing_offset[ 1 ], 0.0, 0.0;
        }
    }

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_moonwalk( i );
    }
}

void ConvexMpcMotionGaits::SetFrontLiftInParams() {
    gait_type_ = &front_lift_in_;

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        if ( i == 0 || i == 1 ) {
            if ( fmod( gait_step_iter_, 2 ) == 1 ) {
                landing_pos_offset_[ i ] << 0.0, user_params_->front_lift_landing_offset[ 2 ] + user_params_->front_lift_landing_offset[ 3 ], 0.0;
            }
            else {
                landing_pos_offset_[ i ] << 0.0, user_params_->front_lift_landing_offset[ 2 ] - user_params_->front_lift_landing_offset[ 3 ], 0.0;
            }
        }
        else {
            landing_pos_offset_[ i ] << 0.0, 0.0, 0.0;
        }
        landing_pos_offset_[ i ] << leg_sign_x_[ i ] * landing_pos_offset_[ i ][ 0 ], leg_sign_y_[ i ] * landing_pos_offset_[ i ][ 1 ], landing_pos_offset_[ i ][ 2 ];
    }

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_front_lift( i );
    }
    mu_for_wbc_ = user_params_->wbc_friction_front_lift;

    // pitch and height delta
    rpy_cmd_[ 1 ]       = user_params_->front_lift_height_pitch[ 1 ];
    body_pitch_filter_  = 0.9;
    pos_cmd_[ 2 ]       = user_params_->des_roll_pitch_height_motion[ 2 ] + user_params_->front_lift_height_pitch[ 0 ];
    body_height_filter_ = 0.9;
}

void ConvexMpcMotionGaits::SetFrontLiftLeftParams() {
    gait_type_ = &front_lift_left_;

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        if ( i == 0 ) {
            if ( fmod( gait_step_iter_, 2 ) == 1 ) {
                landing_pos_offset_[ i ] << user_params_->front_lift_landing_offset[ 0 ] + user_params_->front_lift_landing_offset[ 1 ], 0.0, 0.0;
            }
            else {
                landing_pos_offset_[ i ] << user_params_->front_lift_landing_offset[ 0 ] - user_params_->front_lift_landing_offset[ 1 ], 0.0, 0.0;
            }
        }
        else if ( i == 1 ) {
            if ( fmod( gait_step_iter_, 2 ) == 1 ) {
                landing_pos_offset_[ i ] << user_params_->front_lift_landing_offset[ 0 ] - user_params_->front_lift_landing_offset[ 1 ], 0.0, 0.0;
            }
            else {
                landing_pos_offset_[ i ] << user_params_->front_lift_landing_offset[ 0 ] + user_params_->front_lift_landing_offset[ 1 ], 0.0, 0.0;
            }
        }
        else {
            landing_pos_offset_[ i ] << 0.0, 0.0, 0.0;
        }
        landing_pos_offset_[ i ] << leg_sign_x_[ i ] * landing_pos_offset_[ i ][ 0 ], leg_sign_y_[ i ] * landing_pos_offset_[ i ][ 1 ], landing_pos_offset_[ i ][ 2 ];
    }

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_front_lift( i );
    }
    mu_for_wbc_ = user_params_->wbc_friction_front_lift;

    // pitch and height delta
    rpy_cmd_[ 1 ]       = user_params_->front_lift_height_pitch[ 1 ];
    body_pitch_filter_  = 0.9;
    pos_cmd_[ 2 ]       = user_params_->des_roll_pitch_height_motion[ 2 ] + user_params_->front_lift_height_pitch[ 0 ];
    body_height_filter_ = 0.9;
}

void ConvexMpcMotionGaits::SetFrontLiftSwitchParams() {
    gait_type_ = &front_lift_switch_;

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ] << user_params_->front_lift_switch_landing_offset[ 0 ], user_params_->front_lift_switch_landing_offset[ 1 ], 0.0;
        landing_pos_offset_[ i ] << leg_sign_x_[ i ] * landing_pos_offset_[ i ][ 0 ], leg_sign_y_[ i ] * landing_pos_offset_[ i ][ 1 ], landing_pos_offset_[ i ][ 2 ];
    }

    trans_state_ = 0.2;

    // swing middle position
    for ( int i = 0; i < 4; i++ ) {
        swing_mid_pos_[ i ] << leg_sign_x_[ i ] * 0.0, leg_sign_y_[ i ] * user_params_->front_lift_switch_middle_pos[ 1 ], user_params_->front_lift_switch_middle_pos[ 2 ];
        swing_mid_pos_[ i ] = state_est_.world2body_rotation_matrix.transpose() * swing_mid_pos_[ i ];
    }
    use_swing_mid_pos_ = true;

    vel_transition_ = 0.5;

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_front_lift( i );
    }

    // pitch and height delta
    rpy_cmd_[ 1 ]       = user_params_->front_lift_height_pitch[ 1 ];
    body_pitch_filter_  = 0.9;
    pos_cmd_[ 2 ]       = user_params_->des_roll_pitch_height_motion[ 2 ] + user_params_->front_lift_height_pitch[ 0 ];
    body_height_filter_ = 0.9;
}

void ConvexMpcMotionGaits::SetRearLiftInParams() {
    gait_type_ = &rear_lift_in_;

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        if ( i == 2 || i == 3 ) {
            if ( fmod( gait_step_iter_leg_( 2 ), 2 ) == 1 ) {
                landing_pos_offset_[ i ] << 0.0, user_params_->rear_lift_landing_offset[ 2 ] + user_params_->rear_lift_landing_offset[ 3 ], 0.0;
            }
            else {
                landing_pos_offset_[ i ] << 0.0, user_params_->rear_lift_landing_offset[ 2 ] - user_params_->rear_lift_landing_offset[ 3 ], 0.0;
            }
        }
        else {
            landing_pos_offset_[ i ] << 0.0, 0.0, 0.0;
        }
        landing_pos_offset_[ i ] << leg_sign_x_[ i ] * landing_pos_offset_[ i ][ 0 ], leg_sign_y_[ i ] * landing_pos_offset_[ i ][ 1 ], landing_pos_offset_[ i ][ 2 ];
    }

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_rear_lift( i );
    }
    mu_for_wbc_ = user_params_->wbc_friction_front_lift;

    // pitch and height delta
    rpy_cmd_[ 1 ]       = user_params_->rear_lift_height_pitch[ 1 ];
    body_pitch_filter_  = 0.9;
    pos_cmd_[ 2 ]       = user_params_->des_roll_pitch_height_motion[ 2 ] + user_params_->rear_lift_height_pitch[ 0 ];
    body_height_filter_ = 0.9;
}

void ConvexMpcMotionGaits::SetRearLiftLeftParams() {
    gait_type_ = &rear_lift_left_;

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        if ( i == 2 ) {
            if ( fmod( gait_step_iter_leg_( 2 ), 2 ) == 1 ) {
                landing_pos_offset_[ i ] << user_params_->rear_lift_landing_offset[ 0 ] + user_params_->rear_lift_landing_offset[ 1 ], 0.0, 0.0;
            }
            else {
                landing_pos_offset_[ i ] << user_params_->rear_lift_landing_offset[ 0 ] - user_params_->rear_lift_landing_offset[ 1 ], 0.0, 0.0;
            }
        }
        else if ( i == 3 ) {
            if ( fmod( gait_step_iter_leg_( 3 ), 2 ) == 1 ) {
                landing_pos_offset_[ i ] << user_params_->rear_lift_landing_offset[ 0 ] - user_params_->rear_lift_landing_offset[ 1 ], 0.0, 0.0;
            }
            else {
                landing_pos_offset_[ i ] << user_params_->rear_lift_landing_offset[ 0 ] + user_params_->rear_lift_landing_offset[ 1 ], 0.0, 0.0;
            }
        }
        else {
            landing_pos_offset_[ i ] << 0.0, 0.0, 0.0;
        }
        landing_pos_offset_[ i ] << leg_sign_x_[ i ] * landing_pos_offset_[ i ][ 0 ], leg_sign_y_[ i ] * landing_pos_offset_[ i ][ 1 ], landing_pos_offset_[ i ][ 2 ];
    }

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_rear_lift( i );
    }
    mu_for_wbc_ = user_params_->wbc_friction_front_lift;

    // pitch and height delta
    rpy_cmd_[ 1 ]       = user_params_->rear_lift_height_pitch[ 1 ];
    body_pitch_filter_  = 0.9;
    pos_cmd_[ 2 ]       = user_params_->des_roll_pitch_height_motion[ 2 ] + user_params_->rear_lift_height_pitch[ 0 ];
    body_height_filter_ = 0.9;
}

void ConvexMpcMotionGaits::SetRearLiftSwitchParams() {
    gait_type_ = &rear_lift_switch_;

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ] << user_params_->rear_lift_switch_landing_offset[ 0 ], user_params_->rear_lift_switch_landing_offset[ 1 ], 0.0;
        landing_pos_offset_[ i ] << leg_sign_x_[ i ] * landing_pos_offset_[ i ][ 0 ], leg_sign_y_[ i ] * landing_pos_offset_[ i ][ 1 ], landing_pos_offset_[ i ][ 2 ];
    }

    trans_state_ = 0.2;

    // swing middle position
    for ( int i = 0; i < 4; i++ ) {
        swing_mid_pos_[ i ] << leg_sign_x_[ i ] * 0.0, leg_sign_y_[ i ] * user_params_->rear_lift_switch_middle_pos[ 1 ], user_params_->rear_lift_switch_middle_pos[ 2 ];
        swing_mid_pos_[ i ] = state_est_.world2body_rotation_matrix.transpose() * swing_mid_pos_[ i ];
    }
    use_swing_mid_pos_ = true;

    vel_transition_ = 0.5;

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_rear_lift( i );
    }

    // pitch and height delta
    rpy_cmd_[ 1 ]       = user_params_->rear_lift_height_pitch[ 1 ];
    body_pitch_filter_  = 0.9;
    pos_cmd_[ 2 ]       = user_params_->des_roll_pitch_height_motion[ 2 ] + user_params_->rear_lift_height_pitch[ 0 ];
    body_height_filter_ = 0.9;
}

void ConvexMpcMotionGaits::SetBalletParams() {
    gait_type_ = &ballet_;

    // step height
    for ( int i = 0; i < 4; i++ ) {
        step_height_cmd_[ i ] = user_params_->ballet_step_height;
    }

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ] << leg_sign_y_[ i ] * user_params_->ballet_landing_offset[ 0 ], leg_sign_y_[ i ] * user_params_->ballet_landing_offset[ 1 ], 0.0;
    }

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_ballet( i );
    }

    // height
    pos_cmd_[ 2 ]       = user_params_->des_roll_pitch_height_motion[ 2 ] + user_params_->ballet_height;
    body_height_filter_ = 0.01;

    wbc_user_mpc_traj_gait_select_ = true;
}

void ConvexMpcMotionGaits::SetBalletTransParams() {
    gait_type_ = &ballet_trans_;

    // step height
    for ( int i = 0; i < 4; i++ ) {
        step_height_cmd_[ i ] = user_params_->ballet_trans_step_height;
    }
    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        landing_pos_offset_[ i ] << leg_sign_y_[ i ] * 0.0, leg_sign_y_[ i ] * 0.33 * user_params_->ballet_landing_offset[ 1 ], 0.0;
    }

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_ballet( i );
    }

    wbc_user_mpc_traj_gait_select_ = true;
}

void ConvexMpcMotionGaits::SetPitchDownLeftParams() {
    gait_type_ = &pitch_down_left_;

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        if ( fmod( gait_step_iter_leg_( 1 ), 2 ) == 1 && ( i == 1 ) )
            landing_pos_offset_[ i ] << user_params_->pitch_step_landing_offset[ 0 ], user_params_->pitch_step_landing_offset[ 1 ], 0.0;
        else {
            landing_pos_offset_[ i ] << 0.0, 0.0, 0.0;
        }
    }

    // pitch and height delta
    if ( fmod( gait_step_iter_leg_( 1 ), 2 ) == 1 ) {
        rpy_cmd_[ 1 ] = user_params_->pitch_step_pitch[ 0 ];
    }
    else {
        rpy_cmd_[ 1 ] = -user_params_->pitch_step_pitch[ 0 ];
    }
    body_pitch_filter_  = 0.1;
    pos_cmd_[ 2 ]       = user_params_->des_roll_pitch_height_motion[ 2 ] + user_params_->pitch_step_height[ 0 ];
    body_height_filter_ = 0.1;

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_pitch_step( i );
    }
}

void ConvexMpcMotionGaits::SetPitchDownRightParams() {
    gait_type_ = &pitch_down_right_;

    // landing offset
    for ( int i = 0; i < 4; i++ ) {
        if ( fmod( gait_step_iter_leg_( 0 ), 2 ) == 1 && ( i == 0 ) )
            landing_pos_offset_[ i ] << user_params_->pitch_step_landing_offset[ 0 ], -user_params_->pitch_step_landing_offset[ 1 ], 0.0;
        else {
            landing_pos_offset_[ i ] << 0.0, 0.0, 0.0;
        }
    }

    // pitch and height delta
    if ( fmod( gait_step_iter_leg_( 0 ), 2 ) == 1 ) {
        rpy_cmd_[ 1 ] = user_params_->pitch_step_pitch[ 0 ];
    }
    else {
        rpy_cmd_[ 1 ] = -user_params_->pitch_step_pitch[ 0 ];
    }
    body_pitch_filter_  = 0.1;
    pos_cmd_[ 2 ]       = user_params_->des_roll_pitch_height_motion[ 2 ] + user_params_->pitch_step_height[ 1 ];
    body_height_filter_ = 0.1;

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = user_params_->wbc_weight_pitch_step( i );
    }
}

void ConvexMpcMotionGaits::SetUserGait( const int gait_id ) {
    switch ( gait_id ) {
    case GaitId::kUserGait:
        gait_type_ = &user_gait_;
        break;
    case GaitId::kUserGait00:
        gait_type_ = &user_gait_00_;
        break;
    case GaitId::kUserGait01:
        gait_type_ = &user_gait_01_;
        break;
    case GaitId::kUserGait02:
        gait_type_ = &user_gait_02_;
        break;
    case GaitId::kUserGait03:
        gait_type_ = &user_gait_03_;
        break;
    case GaitId::kUserGait04:
        gait_type_ = &user_gait_04_;
        break;
    case GaitId::kUserGait05:
        gait_type_ = &user_gait_05_;
        break;
    case GaitId::kUserGait06:
        gait_type_ = &user_gait_06_;
        break;
    case GaitId::kUserGait07:
        gait_type_ = &user_gait_07_;
        break;
    case GaitId::kUserGait08:
        gait_type_ = &user_gait_08_;
        break;
    case GaitId::kUserGait09:
        gait_type_ = &user_gait_09_;
        break;
    case GaitId::kUserGait10:
        gait_type_ = &user_gait_10_;
        break;
    case GaitId::kUserGait11:
        gait_type_ = &user_gait_11_;
        break;
    case GaitId::kUserGait12:
        gait_type_ = &user_gait_12_;
        break;
    case GaitId::kUserGait13:
        gait_type_ = &user_gait_13_;
        break;
    case GaitId::kUserGait14:
        gait_type_ = &user_gait_14_;
        break;
    case GaitId::kUserGait15:
        gait_type_ = &user_gait_15_;
        break;
    case GaitId::kUserGait16:
        gait_type_ = &user_gait_16_;
        break;
    case GaitId::kUserGait17:
        gait_type_ = &user_gait_17_;
        break;
    case GaitId::kUserGait18:
        gait_type_ = &user_gait_18_;
        break;
    case GaitId::kUserGait19:
        gait_type_ = &user_gait_19_;
        break;
    case GaitId::kUserGait20:
        gait_type_ = &user_gait_20_;
        break;
    default:
        gait_type_ = &user_gait_00_;
        break;
    }

    // landing offset
    for ( int i = 0; i < 2; i++ ) {
        landing_pos_offset_[ 0 ]( i ) = ctrl_cmd_->foot_pose[ i ];
        landing_pos_offset_[ 1 ]( i ) = ctrl_cmd_->foot_pose[ i + 2 ];
        landing_pos_offset_[ 2 ]( i ) = ctrl_cmd_->foot_pose[ i + 4 ];
        landing_pos_offset_[ 3 ]( i ) = ctrl_cmd_->ctrl_point[ i ];
    }
    //     for ( int i = 0; i < 4; i++ ) {
    //     landing_pos_offset_[ i ] << 0.0, 0.0, 0.0;
    // }

    // step height
    step_height_cmd_( 0 ) = ( ( int )ctrl_cmd_->step_height[ 0 ] % 1000 ) * 1e-3;
    step_height_cmd_( 1 ) = ( ( int )ctrl_cmd_->step_height[ 0 ] / 1000 ) * 1e-3;
    step_height_cmd_( 2 ) = ( ( int )ctrl_cmd_->step_height[ 1 ] % 1000 ) * 1e-3;
    step_height_cmd_( 3 ) = ( ( int )ctrl_cmd_->step_height[ 1 ] / 1000 ) * 1e-3;
    // step_height_cmd_.setConstant( 0.02 );

    // pitch and height
    rpy_cmd_[ 1 ]       = ctrl_cmd_->rpy_des[ 1 ];
    body_pitch_filter_  = 0.1;
    pos_cmd_[ 2 ]       = user_params_->des_roll_pitch_height_motion[ 2 ] + ctrl_cmd_->pos_des[ 2 ];
    body_height_filter_ = 0.01;

    trans_state_    = 0.5;
    vel_transition_ = 2.0;

    // WBC weight
    for ( int i = 0; i < 6; i++ ) {
        floating_base_weight_wbc_( i ) = ctrl_cmd_->acc_des[ i ];
        if ( floating_base_weight_wbc_( i ) < 1e-5 ) {  // in case of error data
            floating_base_weight_wbc_( i ) = 1e-5;
        }
        // floating_base_weight_wbc_( i ) = user_params_->wbc_weight_user_gait( i );
    }

    wbc_user_mpc_traj_gait_select_ = ( bool )( ctrl_cmd_->value & 0x01 );
    mu_for_wbc_                    = ctrl_cmd_->ctrl_point[ 2 ];
    landing_gain_                  = ( ctrl_cmd_->contact % 100 ) * 1e-1;

    // in case of error data
    if ( mu_for_wbc_ < 0.1 * user_params_->wbc_friction_default || mu_for_wbc_ > 5.0 * user_params_->wbc_friction_default ) {
        mu_for_wbc_ = user_params_->wbc_friction_default;
    }
    if ( landing_gain_ < 1e-2 || landing_gain_ > 10.0 ) {
        landing_gain_ = 1.0;
    }

    // // DEBUG
    // for ( int i = 0; i < 4; i++ ) {
    //     std::cout << "landing_pos_offset_[" << i << "]: " << landing_pos_offset_[ i ].transpose() << std::endl;
    // }
    // std::cout << "ctrl_cmd_->step_height: " << ctrl_cmd_->step_height[ 0 ] << " " << ctrl_cmd_->step_height[ 1 ] << std::endl;
    // std::cout << "step_height_cmd_: " << step_height_cmd_.transpose() << std::endl;
    // std::cout << "floating_base_weight_wbc_: " << floating_base_weight_wbc_.transpose() << std::endl;
    // std::cout << "wbc_user_mpc_traj_gait_select_: " << wbc_user_mpc_traj_gait_select_ << std::endl;
    // std::cout << "mu: " << mu_for_wbc_ << std::endl;
    // std::cout << "landing_gain_: " << landing_gain_ << std::endl;
}

void ConvexMpcMotionGaits::SetCmd( ControlFsmData< float >& data ) {

    // backup des
    vel_des_last_ = vel_des_robot_;
    pos_des_last_ = pos_des_;
    rpy_des_last_ = rpy_des_;

    // set vel_des_
    vel_cmd_( 0 ) += ( ( duration_mode_ || ctrl_cmd_->cmd_source > kCyberdogLcmCmd ) ? 1.0 : vel_cmd_max_[ 0 ] ) * ctrl_cmd_->vel_des[ 0 ];  // RC and GAMEMPAD are same
    vel_cmd_( 1 ) += ( ( duration_mode_ || ctrl_cmd_->cmd_source > kCyberdogLcmCmd ) ? 1.0 : vel_cmd_max_[ 1 ] ) * ctrl_cmd_->vel_des[ 1 ];
    vel_cmd_( 2 ) += ( ( duration_mode_ || ctrl_cmd_->cmd_source > kCyberdogLcmCmd ) ? 1.0 : vel_cmd_max_[ 2 ] ) * ctrl_cmd_->vel_des[ 2 ];

    SetCmdOffset( data );

    for ( int i = 0; i < 3; i++ ) {
        // vel_des_robot_( i ) = ApplyVelocityMeetAccelationLimit( vel_des_last_( i ), vel_cmd_( i ), vel_cmd_min_( i ), vel_cmd_max_( i ), acc_cmd_min_( i ), acc_cmd_max_( i ), dt_ );
        vel_des_robot_( i ) = WrapRange( vel_cmd_( i ), vel_cmd_min_( i ), vel_cmd_max_( i ) );
    }

    // set step_height_des_
    for ( int i = 0; i < 4; i++ ) {
        step_height_des_( i ) = WrapRange( step_height_cmd_[ i ], step_height_min_, step_height_max_ );
    }

    // set rpy_des_
    SetYawDesire();
    rpy_des_( 0 ) = 0.0;
    rpy_des_( 1 ) = rpy_des_last_[ 1 ] * ( 1.0 - body_pitch_filter_ ) + rpy_cmd_[ 1 ] * body_pitch_filter_;
    rpy_des_( 1 ) = WrapRange( rpy_des_( 1 ), rpy_cmd_min_[ 1 ], rpy_cmd_max_[ 1 ] );

    if ( ctrl_cmd_->mode != MotionMode::kLocomotion ) // Hold pitch angle when exit locomotion
        rpy_des_( 1 ) = state_estimator_->GetResult().rpy( 1 );

    // set pos_des_
    pos_des_( 2 ) = pos_des_last_[ 2 ] * ( 1.0 - body_height_filter_ ) + pos_cmd_[ 2 ] * body_height_filter_;
    pos_des_( 2 ) = WrapRange( pos_des_( 2 ), pos_cmd_min_[ 2 ], pos_cmd_max_[ 2 ] );

    state_estimator_->SetRemoterVelocityResult( vel_des_robot_ );  // vel_des is used in lcm

    // // DEBUG
    // std::cout << "vel_des_robot_: " << vel_des_robot_.transpose() << std::endl;
    // std::cout << "pos_des_:"        << pos_des_.transpose() << std::endl;
    // std::cout << "step_height_des_: " << step_height_des_.transpose() << std::endl;
}

void ConvexMpcMotionGaits::SetCmdOffset( ControlFsmData< float >& data ) {
    switch ( gait_cmd_ ) {
    case GaitId::kBallet:
        vel_cmd_( 0 ) += data.control_parameters->speed_offset_ballet[ 0 ];
        vel_cmd_( 1 ) += data.control_parameters->speed_offset_ballet[ 1 ];
        vel_cmd_( 2 ) += data.control_parameters->speed_offset_ballet[ 2 ];
        break;
    default:
        break;
    }
}

void ConvexMpcMotionGaits::SetYawDesire() {
    static float yaw_dot_deadzone       = 1e-4;
    static float k_integral             = 1e-3;
    static float yaw_err_max            = 0.12;
    static float yaw_err_integral_reset = 0.01;

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

void ConvexMpcMotionGaits::CheckGaitFirstStep() {
    if ( gait_iter_ < horizon_length_ * iter_between_mpc_ ) {
        gait_first_step_ = true;
    }
    else {
        gait_first_step_ = false;
    }
}

void ConvexMpcMotionGaits::SetBodyWorldDesire() {
    static Vec3< float > v_robot_xyz( vel_des_robot_( 0 ), vel_des_robot_( 1 ), 0.0 );

    v_robot_xyz << vel_des_robot_( 0 ), vel_des_robot_( 1 ), 0.0;
    vel_des_ = omni_cmd_ ? v_robot_xyz : state_est_.world2body_rotation_matrix.transpose() * v_robot_xyz;
    pos_des_( 0 ) += dt_ * vel_des_( 0 );
    pos_des_( 1 ) += dt_ * vel_des_( 1 );
}

void ConvexMpcMotionGaits::GetFootPosFeedback() {
    for ( int i = 0; i < 4; i++ ) {
        foot_pos_feedback_[ i ] = state_est_.position + state_est_.world2body_rotation_matrix.transpose() * ( robot_model_->GetHipLocation( i ) + leg_ctrl_->datas_[ i ].p );
    }
}

void ConvexMpcMotionGaits::SetGaitFirstRunVars() {
    pos_des_( 0 ) = state_est_.position( 0 );
    pos_des_( 1 ) = state_est_.position( 1 );
    rpy_des_( 2 ) = state_est_.rpy( 2 );

    for ( int i = 0; i < 4; i++ ) {
        foot_swing_trajectory_[ i ].SetHeight( step_height_des_( i ) );
        foot_swing_trajectory_[ i ].SetInitialPosition( foot_pos_feedback_[ i ] );
        foot_swing_trajectory_[ i ].SetFinalPosition( foot_pos_feedback_[ i ] );
        foot_swing_trajectory_[ i ].SetMidPosition( swing_mid_pos_[ i ] );
    }
    gait_first_run_ = false;
}

void ConvexMpcMotionGaits::SetRpyComp() {
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

    // // DEBUG
    // std::cout << "rpy_comp_: " << rpy_comp_.transpose() << std::endl;
}

void ConvexMpcMotionGaits::SetLandingPos() {
    static float         side_sign[ 4 ]            = { -1.0f, 1.0f, -1.0f, 1.0f };  // dirction of abad_link_length_
    static float         landing_pos_y_offset[ 4 ] = { 0.0, 0.0, 0.0, -0.0 };       //{0.0, 0.0, 0.02, -0.02};
    static float         landing_pos_ctrl_max      = 0.35f;
    static Vec3< float > hip_pos_robot_frame( 0.0, 0.0, 0.0 );
    static Vec3< float > hip_foot_offset( 0.0, 0.0, 0.0 );
    static Vec3< float > pos_yaw_corrected( 0.0, 0.0, 0.0 );
    static Vec3< float > vel_des_pf( 0.0, 0.0, 0.0 );
    static Vec3< float > landing_pos_ctrl( 0.0, 0.0, 0.0 );
    static Vec3< float > landing_pos_ctrl_world( 0.0, 0.0, 0.0 );

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
        hip_pos_robot_frame = landing_pos_ratio_.cwiseProduct( robot_model_->GetHipLocation( i ) + hip_foot_offset + landing_pos_offset_[ i ] );  // feet landing pos withdraw or extend
        hip_pos_robot_frame[ 1 ] += landing_pos_y_offset[ i ] * fabs( vel_des_robot_( 0 ) );                                                      // rear feet land outside to avoid collision

        pos_yaw_corrected = CoordinateRotation( CoordinateAxis::Z, -vel_des_robot_( 2 ) * stance_time_( i ) / 2 ) * hip_pos_robot_frame;

        vel_des_pf[ 0 ] = vel_des_robot_( 0 );
        vel_des_pf[ 1 ] = vel_des_robot_( 1 );
        vel_des_pf[ 2 ] = 0.0;

        landing_pos_[ i ] = state_est_.position
                            + state_est_.world2body_rotation_matrix.transpose() *  // perdict hip pos at landing
                                  ( pos_yaw_corrected + vel_des_pf * swing_time_remain_( i ) );

        landing_pos_ctrl[ 0 ] = state_est_.velocity_in_body_frame( 0 ) * ( .5 + landing_gain_params_( 0 ) ) * stance_time_( i )
                                + landing_gain_ * landing_gain_params_( 1 ) * ( state_est_.velocity_in_body_frame( 0 ) - vel_des_robot_( 0 ) )
                                + ( 0.5f * state_est_.position( 2 ) / GRAVITY ) * ( state_est_.velocity_in_body_frame( 1 ) * vel_des_robot_( 2 ) );
        landing_pos_ctrl[ 1 ] = state_est_.velocity_in_body_frame( 1 ) * ( .5 + landing_gain_params_( 2 ) ) * stance_time_( i )
                                + landing_gain_ * landing_gain_params_( 3 ) * ( state_est_.velocity_in_body_frame( 1 ) - vel_des_robot_( 1 ) )
                                + ( 0.5f * state_est_.position( 2 ) / GRAVITY ) * ( -state_est_.velocity_in_body_frame( 0 ) * vel_des_robot_( 2 ) );
        landing_pos_ctrl[ 0 ]  = WrapRange( landing_pos_ctrl[ 0 ], -landing_pos_ctrl_max, landing_pos_ctrl_max );
        landing_pos_ctrl[ 1 ]  = WrapRange( landing_pos_ctrl[ 1 ], -landing_pos_ctrl_max, landing_pos_ctrl_max );
        landing_pos_ctrl[ 2 ]  = 0.0;
        landing_pos_ctrl_world = state_est_.world2body_rotation_matrix.transpose() * landing_pos_ctrl;
        landing_pos_[ i ]( 0 ) += landing_pos_ctrl_world[ 0 ];
        landing_pos_[ i ]( 1 ) += landing_pos_ctrl_world[ 1 ];
        landing_pos_[ i ]( 2 ) = user_params_->foot_final_height;

        foot_swing_trajectory_[ i ].SetHeight( step_height_des_( i ) );
        foot_swing_trajectory_[ i ].SetFinalPosition( landing_pos_[ i ] );
        foot_swing_trajectory_[ i ].SetMidPosition( swing_mid_pos_[ i ] );
    }

    // // DEBUG
    // for (int i = 0; i < 4; i++) {
    //   std::cout << "landing_pos_[" << i << "]: " << landing_pos_[i].transpose() << std::endl;
    // }
}

void ConvexMpcMotionGaits::CheckGaitTransition() {
    static Vec3< float > vel_xy_robot( state_est_.velocity_in_body_frame( 0 ), state_est_.velocity_in_body_frame( 1 ), 0.0 );

    contact_states_        = gait_type_->GetContactState();
    swing_states_          = gait_type_->GetSwingState();
    gait_check_transition_ = false;
    gait_allow_transition_ = true;
    vel_xy_robot << state_est_.velocity_in_body_frame( 0 ), state_est_.velocity_in_body_frame( 1 ), 0.0;

    if ( vel_xy_robot.norm() < vel_transition_ ) {
        if ( current_gait_ != GaitId::kSpecialPronk ) {
            if ( ( swing_states_( 0 ) <= trans_state_ || swing_states_( 0 ) >= ( 1.0 - trans_state_ ) ) && ( swing_states_( 1 ) <= trans_state_ || swing_states_( 1 ) >= ( 1.0 - trans_state_ ) )
                 && ( swing_states_( 2 ) <= trans_state_ || swing_states_( 2 ) >= ( 1.0 - trans_state_ ) ) && ( swing_states_[ 3 ] <= trans_state_ || swing_states_[ 3 ] >= ( 1.0 - trans_state_ ) ) ) {
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

    // // DBEUG
    // if (gait_check_transition_) {
    //   std::cout << "loco_iter_: " << loco_iter_ << std::endl;
    //   std::cout << "gait_iter_: " << gait_iter_ << std::endl;
    // }
}

void ConvexMpcMotionGaits::ComputeMpc() {
    static int* mpc_table = gait_type_->GetMpcTable();
    mpc_table             = gait_type_->GetMpcTable();
    start_solve_mpc_      = false;
    UpdateMpc( mpc_table );
}

void ConvexMpcMotionGaits::UpdateMpc( int* mpc_table ) {
    static Vec3< float > pos_init( 0.0, 0.0, 0.0 );
    static float*        p = state_est_.position.data();

    if ( ( loco_iter_ % iter_between_mpc_ ) == 0 ) {
        pos_init = pos_des_;
        p        = state_est_.position.data();
        if ( pos_init[ 0 ] - p[ 0 ] > max_pos_err_mpc_[ 0 ] )
            pos_init[ 0 ] = p[ 0 ] + max_pos_err_mpc_[ 0 ];

        if ( p[ 0 ] - pos_init[ 0 ] > max_pos_err_mpc_[ 0 ] )
            pos_init[ 0 ] = p[ 0 ] - max_pos_err_mpc_[ 0 ];

        if ( pos_init[ 1 ] - p[ 1 ] > max_pos_err_mpc_[ 1 ] )
            pos_init[ 1 ] = p[ 1 ] + max_pos_err_mpc_[ 1 ];

        if ( p[ 1 ] - pos_init[ 1 ] > max_pos_err_mpc_[ 1 ] )
            pos_init[ 1 ] = p[ 1 ] - max_pos_err_mpc_[ 1 ];

        pos_des_ = pos_init;

        float traj_init[ NUM_MPC_STATE_INPUT ] = { rpy_des_( 0 ) + rpy_comp_( 0 ),  // 0
                                                   rpy_des_( 1 ) + rpy_comp_( 1 ),  // 1
                                                   rpy_des_( 2 ),                   // 2
                                                   pos_init[ 0 ],                   // 3
                                                   pos_init[ 1 ],                   // 4
                                                   pos_init[ 2 ],                   // 5
                                                   0,                               // 6
                                                   0,                               // 7
                                                   vel_des_robot_( 2 ),             // 8
                                                   vel_des_( 0 ),                   // 9
                                                   vel_des_( 1 ),                   // 10
                                                   0 };                             // 11

        for ( int i = 0; i < horizon_length_; i++ ) {
            for ( int j = 0; j < NUM_MPC_STATE_INPUT; j++ )
                traj_all_[ NUM_MPC_STATE_INPUT * i + j ] = traj_init[ j ];

            if ( i > 0 ) {
                traj_all_[ NUM_MPC_STATE_INPUT * i + 3 ] = traj_all_[ NUM_MPC_STATE_INPUT * ( i - 1 ) + 3 ] + dt_mpc_ * vel_des_( 0 );
                traj_all_[ NUM_MPC_STATE_INPUT * i + 4 ] = traj_all_[ NUM_MPC_STATE_INPUT * ( i - 1 ) + 4 ] + dt_mpc_ * vel_des_( 1 );
                traj_all_[ NUM_MPC_STATE_INPUT * i + 2 ] = traj_all_[ NUM_MPC_STATE_INPUT * ( i - 1 ) + 2 ] + dt_mpc_ * vel_des_robot_( 2 );
            }
        }

        if ( solve_mpc_by_single_thread_ ) {
            gait_mpc_table_                       = mpc_table;
            start_solve_mpc_                      = true;
            solver_mpc_interface_->solve_success_ = false;
        }
        else {
            // Timer solveTimer;
            // SolveDenseMpc( mpc_table );
            // printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.GetElapsedMilliseconds());
        }
    }
}

/**
 * @brief solve mpc in another cpu thread
 *
 */
void ConvexMpcMotionGaits::SolveMpcAnotherThread() {
    static std::mutex _opt_data_mutex;

    // calu xy drag
    static float         pz_err = state_est_.position( 2 ) - pos_des_( 2 );
    static Vec3< float > vxy( state_est_.velocity_in_body_frame( 0 ), state_est_.velocity_in_body_frame( 1 ), 0 );
    static float         xy_comp_drag[ 2 ] = { x_comp_integral_, y_comp_integral_ };
    static float         weight_state[ NUM_MPC_STATE_INPUT ];
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
    // set weight
    for ( int i = 0; i < NUM_MPC_CTRL_INPUT; i++ ) {
        weight_state[ i ] = user_params_->mpc_task_weight[ i ];
    }
    if ( alpha > 1e-4 ) {
        std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
        alpha = 1e-5;
    }
    // get foot pos on absolute body frame

    com_offset = state_est_.world2body_rotation_matrix.transpose() * user_params_->mpc_com_offset.cast< float >();
    for ( int i = 0; i < 12; i++ )
        r_body2foot[ i ] = foot_pos_feedback_[ i % 4 ][ i / 4 ] - ( state_est_.position[ i / 4 ] + com_offset[ i / 4 ] );
    // update params and data collect
    solver_mpc_interface_->SetupGravityDirectionCompensation( xy_comp_drag );

    solver_mpc_interface_->SetupMpcSolverParams( dt_mpc_, horizon_length_, mu_for_mpc_, user_params_->mpc_force_max, user_params_->mpc_body_inertia.cast< float >(), user_params_->mpc_body_mass,
                                                 weight_state, alpha );

    solver_mpc_interface_->UpdateMpcSolverData( state_est_.position, state_est_.velocity_in_world_frame, state_est_.rpy, state_est_.angular_velocity_in_world_frame, state_est_.orientation,
                                                r_body2foot, traj_all_, gait_mpc_table_ );

    // solve dense mpc
    solver_mpc_interface_->SolveDenseMpc();
    // update force and com trajectory for wbc
    if ( solver_mpc_interface_->solve_success_ ) {
        _opt_data_mutex.lock();
        for ( int leg = 0; leg < 4; leg++ ) {
            foot_force_result_[ leg ]     = solver_mpc_interface_->optimal_solution_.optimal_force[ leg ];
            foot_force_leg_result_[ leg ] = -state_est_.world2body_rotation_matrix * foot_force_result_[ leg ];

            foot_force_des_[ leg ] = foot_force_result_[ leg ];
        }
        opt_trajectory_next_ = solver_mpc_interface_->optimal_solution_.optimal_state_next_horizon;
        _opt_data_mutex.unlock();
    }
}

void ConvexMpcMotionGaits::SetLegKpKd() {
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

void ConvexMpcMotionGaits::SetFootDesire() {
    se_contact_states_ << 0.0, 0.0, 0.0, 0.0;
    for ( int i = 0; i < 4; i++ ) {
        if ( swing_states_( i ) > 0.0 ) {  // foot is in swing phase
            if ( foot_first_swing_[ i ] ) {
                foot_first_swing_[ i ] = false;
                foot_swing_trajectory_[ i ].SetInitialPosition( foot_pos_feedback_[ i ] );
                if ( i == 0 ) {
                    toggle_every_step_ *= -1;
                    gait_step_iter_++;
                }
                gait_step_iter_leg_( i )++;
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
    //   // std::cout << "foot_pos_des_[" << i << "]: " << foot_pos_des_[i].transpose() << std::endl;
    //   // std::cout << "foot_vel_des_[" << i << "]: " << foot_vel_des_[i].transpose() << std::endl;
    //   // std::cout << "pDes[" << i << "]: " << leg_ctrl_->commands_[i].pDes.transpose() << std::endl;
    //   // std::cout << "vDes[" << i << "]: " << leg_ctrl_->commands_[i].vDes.transpose() << std::endl;
    // }
}

void ConvexMpcMotionGaits::ComputeSwingTrajectory( int foot_num ) {
    static int i = 0;
    i            = foot_num;
    if ( user_params_->mpc_use_bezier >= 0.9 ) {  // use bezier traj
        if ( !use_swing_mid_pos_ ) {
            foot_swing_trajectory_[ i ].ComputeSwingTrajectoryBezier( swing_states_( i ), swing_time_( i ) );
        }
        else {
            foot_swing_trajectory_[ i ].ComputeSwingTrajectoryMidPosBezier( swing_states_( i ), swing_time_( i ) );
        }
    }
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

void ConvexMpcMotionGaits::SetFootDesireSwing( int foot_num ) {
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

void ConvexMpcMotionGaits::SetFootDesireStance( int foot_num ) {
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

void ConvexMpcMotionGaits::SetContactStatesForEstimator() {
    if ( contact_states_( 0 ) >= 0.8 && contact_states_( 1 ) >= 0.8 && contact_states_( 2 ) >= 0.8 && contact_states_( 3 ) >= 0.8 )
        se_contact_states_ << 0.5, 0.5, 0.5, 0.5;

    state_estimator_->SetContactPhase( se_contact_states_ );
}

void ConvexMpcMotionGaits::SetBodyDesireTrajectory() {
    if ( user_params_->wbc_use_mpc_traj_motion < 0.9 && !wbc_user_mpc_traj_gait_select_ ) {  // traj before MPC
        body_pos_des_ = pos_des_;

        body_vel_des_( 0 ) = vel_des_( 0 );
        body_vel_des_( 1 ) = vel_des_( 1 );
        body_vel_des_( 2 ) = 0.;
    }
    else {  // traj after MPC
        static Vec12< float > _opt_traj_inter;
        _opt_traj_inter = 1.0 / interpolate_max_num_ * interpolate_count_ * ( opt_trajectory_next_ - opt_trajectory_pre_ ) + opt_trajectory_pre_;

        body_pos_des_( 0 ) = _opt_traj_inter[ 3 ];
        body_pos_des_( 1 ) = _opt_traj_inter[ 4 ];
        if ( wbc_use_vertical_traj_ ) {
            body_pos_des_( 2 ) = _opt_traj_inter[ 5 ];
        }
        else {
            body_pos_des_( 2 ) = pos_des_( 2 );
        }
        body_vel_des_( 0 ) = _opt_traj_inter[ 9 ];
        body_vel_des_( 1 ) = _opt_traj_inter[ 10 ];
        if ( wbc_use_vertical_traj_ ) {
            body_vel_des_( 2 ) = _opt_traj_inter[ 11 ];
        }
        else {
            body_vel_des_( 2 ) = 0.;
        }
        interpolate_count_++;
        // for first run , max is 9, and then is 10
        if ( interpolate_count_ > interpolate_max_num_ ) {
            interpolate_count_   = 1;
            interpolate_max_num_ = 15;  // 10
            opt_trajectory_pre_  = opt_trajectory_next_;
        }
    }

    body_acc_des_.setZero();
    body_rpy_des_ = rpy_des_ + rpy_comp_;
    body_omg_des_.setZero();
    body_omg_des_( 2 ) = vel_des_robot_( 2 );

    // // DEBUG
    // std::cout << "body_pos_des_: " << body_pos_des_.transpose() << std::endl;
    // std::cout << "body_vel_des_: " << body_vel_des_.transpose() << std::endl;
    // std::cout << "body_rpy_des_: " << body_rpy_des_.transpose() << std::endl;
    // std::cout << "body_omg_des_: " << body_omg_des_.transpose() << std::endl;
}

#if ( defined DRAW_DEBUG_SWINGS ) || ( defined DRAW_DEBUG_PATH )
void ConvexMpcMotionGaits::DrawResult( ControlFsmData< float >& data ) {
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

void ConvexMpcMotionGaits::DrawSwingPath( ControlFsmData< float >& data, int foot_num ) {
    int foot        = 0;
    foot            = foot_num;
    auto* debugPath = data.visualization_data->AddPath();
    if ( debugPath ) {
        debugPath->num_points = 100;
        debugPath->color      = { 0.2, 1, 0.2, 0.5 };
        float step            = ( 1.f - swing_states_( foot ) ) / 100.f;
        for ( int i = 0; i < 100; i++ ) {
            if ( data.user_parameters->mpc_use_bezier >= 0.9 ) {
                if ( !use_swing_mid_pos_ ) {
                    foot_swing_trajectory_[ foot ].ComputeSwingTrajectoryBezier( swing_states_( foot ) + i * step, swing_time_( foot ) );
                }
                else {
                    foot_swing_trajectory_[ foot ].ComputeSwingTrajectoryMidPosBezier( swing_states_( foot ) + i * step, swing_time_( foot ) );
                }
            }
            else if ( data.user_parameters->mpc_use_bezier >= 0.0 )
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
    if ( data.user_parameters->mpc_use_bezier >= 0.9 ) {
        if ( !use_swing_mid_pos_ ) {
            foot_swing_trajectory_[ foot ].ComputeSwingTrajectoryBezier( swing_states_( foot ), swing_time_( foot ) );
        }
        else {
            foot_swing_trajectory_[ foot ].ComputeSwingTrajectoryMidPosBezier( swing_states_( foot ), swing_time_( foot ) );
        }
    }
    else if ( data.user_parameters->mpc_use_bezier >= 0.0 )
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

void ConvexMpcMotionGaits::DrawStancePath( ControlFsmData< float >& data, int foot_num ) {
    static int foot        = 0;
    foot                   = foot_num;
    auto* actualSphere     = data.visualization_data->AddSphere();
    actualSphere->position = foot_pos_feedback_[ foot ];
    actualSphere->radius   = 0.02;
    actualSphere->color    = { 0.2, 0.2, 0.8, 0.7 };
}

void ConvexMpcMotionGaits::DrawMpcPath( ControlFsmData< float >& data ) {
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
    // draw  mpc optimized traj
    auto* trajectory_mpc_opt = data.visualization_data->AddPath();
    if ( trajectory_mpc_opt ) {
        trajectory_mpc_opt->num_points = horizon_length_;
        trajectory_mpc_opt->color      = { 0.2, 0.2, 0.6, 0.4 };
        for ( int i = 0; i < horizon_length_; i++ ) {
            trajectory_mpc_opt->position[ i ][ 0 ] = opt_trajectory_next_( 3 );
            trajectory_mpc_opt->position[ i ][ 1 ] = opt_trajectory_next_( 4 );
            trajectory_mpc_opt->position[ i ][ 2 ] = opt_trajectory_next_( 5 );
            auto* ball                             = data.visualization_data->AddSphere();
            ball->radius                           = 0.01;
            ball->position                         = trajectory_mpc_opt->position[ i ];
            ball->color                            = { 1.0, 0.2, 0.5, 0.5 };
        }
    }
}
