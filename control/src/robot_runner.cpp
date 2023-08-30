#include <chrono>
#include <unistd.h>

#include "ParamHandler.hpp"
#include "control_flags.hpp"
#include "dynamics/cyberdog.hpp"
#include "estimator/bat_estimator.hpp"
#include "estimator/contact_estimator.hpp"
#include "estimator/footforce_contact_estimator.hpp"
#include "estimator/new_bat_estimator.hpp"
#include "estimator/orientation_estimator.hpp"
#include "estimator/position_velocity_estimator.hpp"
// #include "estimator/absolute_position_velocity_estimator.hpp"
#include "estimator/terrain_estimator.hpp"
#include "robot_runner.hpp"
#include "utilities/timer.hpp"
#include "utilities/toolkit.hpp"

RobotRunner::RobotRunner( RobotController* robot_ctrl, PeriodicTaskManager* manager, float period, std::string name )
    : PeriodicTask( manager, period, name ), bms_status_( nullptr ), battery_soc_( nullptr ), state_estimate_result_(), lcm_( GetLcmUrl( 255 ) ), state_lcm_( GetLcmUrlWithPort( 7669, 255 ) ),
      global_to_robot_lcm_( GetLcmUrl( 255 ) ), send_to_ros_lcm_( GetLcmUrlWithPort( 7670, 255 ) ), send_to_ros_lcm_cyberdog_( GetLcmUrlWithPort( 7670, 255 ) ),
      send_to_ros_lcm_motor_( GetLcmUrlWithPort( 7670, 255 ) ) {
    robot_ctrl_ = robot_ctrl;
}

/**
 * @brief Initializes the robot model, state estimator, leg controller,
 * robot data, and any control logic specific data.
 */
void RobotRunner::InitTask() {
    printf( "[RobotRunner] initialize\n" );

    // Build the appropriate Quadruped object
    if ( robot_type_ == RobotType::CYBERDOG || robot_type_ == RobotType::CYBERDOG2 ) {
        quadruped_ = BuildCyberdog< float >( robot_type_, robot_appearance_type_ );
    }
    else {
        assert( false );
    }

    // Initialize the model and robot data
    model_ = quadruped_.BuildModel();

    // Always initialize the leg controller and state entimator
    leg_controller_  = new LegController< float >( quadruped_ );
    state_estimator_ = new StateEstimatorContainer< float >( cheater_state_, vector_nav_data_, leg_controller_->datas_, &state_estimate_result_, control_parameters_ );
    InitializeStateEstimator( false );

    // TODO: Init command interface
    // Controller initializations
    robot_ctrl_->robot_type_            = robot_type_;
    robot_ctrl_->model_                 = &model_;
    robot_ctrl_->quadruped_             = &quadruped_;
    robot_ctrl_->state_estimate_result_ = &state_estimate_result_;

    // Pointer passing
    robot_ctrl_->leg_controller_     = leg_controller_;
    robot_ctrl_->state_estimator_    = state_estimator_;
    robot_ctrl_->visualization_data_ = visualization_data_;
    robot_ctrl_->control_parameters_ = control_parameters_;
    robot_ctrl_->command_            = &cmd_interface_->GetCommand();

    robot_ctrl_->InitializeController();
}

/**
 * @brief Runs the overall robot control system by calling each of the major components
 * to run each of their respective steps.
 *
 */
void RobotRunner::RunTask() {

    //  Timer runController_timer;

    // record start timestamp first, in ns
    using clock        = std::chrono::high_resolution_clock;
    intmax_t timestamp = clock::now().time_since_epoch().count();
    // Run the state estimator step
    state_estimator_->Run();
    visualization_data_->clear();

    // publish state estimator fisrt, for SLAM system
    state_estimate_result_.setLcm( state_estimator_lcm_ );
    state_estimator_lcm_.timestamp  = timestamp;
    global_to_robot_lcmt_.timestamp = timestamp;
    /***publish by independent thread***/
    // lcm_.publish("state_estimator", &state_estimator_lcm_);

    // Update the data from the robot
    SetupStep();

    static int64_t count_ini( 0 );
    static int32_t reset_motor_error_cnt = 0;
    static bool    isMotorEnable         = false;

    ++count_ini;
    if ( !isMotorEnable && robot_ctrl_->GetLegMode( spi_data_->flags ) != 2 && count_ini < 1000 ) {
        if ( count_ini % 20 < 10 ) {
            leg_controller_->SetEnabled( false );
        }
        else {
            leg_controller_->SetEnabled( true );
        }
        if ( count_ini % 2000 == 0 ) {
            std::cout << "#### Motor not in motor mode !!!" << std::endl;
        }
    }
    else {
        Vec4< float > contactDefault( 0.5, 0.5, 0.5, 0.5 );
        state_estimator_->SetContactPhase( contactDefault );

        isMotorEnable = true;
        // leg_controller_->SetEnabled(true);

        leg_controller_->SetErrorClear( false );

        robot_ctrl_->HandleLegErrorAndWarn( spi_data_->flags );
        robot_ctrl_->HandleMotorsTemperature( spi_data_->tmp_abad, spi_data_->tmp_hip, spi_data_->tmp_knee );

        UnresponceSaftyCheck();
        auto cmd = cmd_interface_->GetCommand();

        if ( control_parameters_->use_rc == 1 and cmd.mode == MotionMode::kOff ) {
            if ( count_ini % 1000 == 0 ) {
                using clock = std::chrono::system_clock;
                auto tt     = clock::to_time_t( clock::now() );
                std::cout << "ESTOP " << ctime( &tt );
            }
            for ( int leg = 0; leg < 4; leg++ ) {
                leg_controller_->commands_[ leg ].Zero();
            }
            // clear error or warning manually
            robot_ctrl_->Estop();
            if ( robot_ctrl_->GetMotorErrorFlag() || robot_ctrl_->GetMotorWarnFlag() || robot_ctrl_->GetMotionModelag() ) {
                if ( robot_ctrl_->CheckMotorsOverHeat() ) {
                    if ( reset_motor_error_cnt >= 2000 && robot_ctrl_->CheckMotorsReturnSaftyTemperature() ) {
                        leg_controller_->SetErrorClear( true );
                        std::cout << "Clear Motor Error !!!" << std::endl;
                    }
                }
                else {
                    if ( reset_motor_error_cnt >= 2000 ) {
                        leg_controller_->SetErrorClear( true );
                        if ( reset_motor_error_cnt % 1000 == 0 ) {
                            std::cout << "Clear Motor Error !!!" << std::endl;
                        }
                    }
                }
                reset_motor_error_cnt++;
            }
            else {
                leg_controller_->SetEnabled( false );
                reset_motor_error_cnt = 0;
            }
        }
        else {
            // Controller
            // Run Control
            robot_ctrl_->RunController();
            // Update Visualization
            robot_ctrl_->UpdateVisualization();

            reset_motor_error_cnt = 0;
        }
    }

    // Visualization (will make this into a separate function later)
    for ( int leg = 0; leg < 4; leg++ ) {
        for ( int joint = 0; joint < 3; joint++ ) {
            cyberdog2_main_visualization_->q[ leg * 3 + joint ] = leg_controller_->datas_[ leg ].q[ joint ];
        }
    }
    cyberdog2_main_visualization_->p    = state_estimate_result_.position;
    cyberdog2_main_visualization_->quat = state_estimate_result_.orientation;

    // Sets the leg controller commands for the robot appropriate commands
    FinalizeStep();

    // std::cout<<"runController time cost (ms) : "<<runController_timer.GetElapsedMilliseconds()<<std::endl;

    // Set bms_status_ for Cyberdog2
    if ( robot_type_ == RobotType::CYBERDOG2 ) {
        state_estimator_->SetBmsStatus( bms_status_ );
        state_estimator_->SetBattSoc( battery_soc_ );
        if ( robot_ctrl_->GetFsmMode() == static_cast< int >( MotionMode::kOff )
             || ( robot_ctrl_->GetFsmMode() == static_cast< int >( MotionMode::kPureDamper ) && robot_ctrl_->GetFsmSwitchFlag() ) ) {
            enable_low_power_ = true;
        }
        else {
            enable_low_power_ = false;
        }
    }
}
void RobotRunner::UnresponceSaftyCheck() {
    auto cmd        = cmd_interface_->GetCommand();
    auto cur_status = robot_ctrl_->GetSwitchStatus();
    // GetSwitchStatus: kDone, kTransitioning, kEstop, kEdamp, kRobotLifted, kBanTrans, kOverHeat, kLowBat
    if ( ( cur_status == 2 && cmd.mode != MotionMode::kOff ) || ( cur_status == 0 && robot_ctrl_->GetFsmMode() == MotionMode::kOff && cmd.mode == MotionMode::kRecoveryStand ) ) {
        unresponce_count_++;
        if ( unresponce_count_ > 10 )
            cmd_interface_->SetCmdQueueClearFlag( MotionMode::kOff );
    }
    else if ( cur_status == 3 && cmd.mode != MotionMode::kPureDamper ) {
        unresponce_count_++;
        if ( unresponce_count_ > 25 )
            cmd_interface_->SetCmdQueueClearFlag( MotionMode::kPureDamper );
    }
    else {
        unresponce_count_ = 0;
        cmd_interface_->SetCmdQueueClearFlag( -1 );
    }
}
/**
 * @brief Publish lcm message by independent thread.
 *
 */
void RobotRunner::LCMPublishByThread() {
    int freq_div = 10;  // 2:250HZ  3:167hz  4:125HZ  5:100HZ  10:50HZ
    if ( control_parameters_->lcm_debug_switch == 1 )
        freq_div = 1;
    // stateEstimator publish
    if ( lcm_iterations_ % freq_div == 0 ) {
        state_lcm_.publish( "state_estimator", &state_estimator_lcm_ );
        global_to_robot_lcm_.publish( "global_to_robot", &global_to_robot_lcmt_ );
        // publish legs' cmd and data
        lcm_.publish( "leg_control_data", &leg_control_data_lcm_ );
    }
    if ( control_parameters_->lcm_debug_switch == 1 ) {
        lcm_.publish( "leg_control_command", &leg_control_command_lcm_ );
    }
    if ( robot_type_ == RobotType::CYBERDOG )
        PublishCyberdogLcmFeedback();
    // lcm_.publish("wbc_lcm_data", &(WBCtrl::wbc_data_lcm_) );
    // PublishLcmFeedback();
    PublishLcmMotorStates();
    lcm_iterations_++;
}

/**
 * @brief Before running user code, setup the leg control and estimators.
 *
 */
void RobotRunner::SetupStep() {
    // Update the leg data
    if ( robot_type_ == RobotType::CYBERDOG || robot_type_ == RobotType::CYBERDOG2 ) {
        leg_controller_->UpdateData( spi_data_ );
    }
    else {
        assert( false );
    }

    // Setup the leg controller for a new iteration
    // leg_controller_->ZeroCommand();
    // leg_controller_->SetEnabled( true );

    // state estimator
    // check transition to cheater mode:
    if ( !cheater_mode_enabled_ && control_parameters_->cheater_mode ) {
        printf( "[RobotRunner] Transitioning to Cheater Mode...\n" );
        InitializeStateEstimator( true );
        // todo any configuration
        cheater_mode_enabled_ = true;
    }

    // check transition from cheater mode:
    if ( cheater_mode_enabled_ && !control_parameters_->cheater_mode ) {
        printf( "[RobotRunner] Transitioning from Cheater Mode...\n" );
        InitializeStateEstimator( false );
        // todo any configuration
        cheater_mode_enabled_ = false;
    }

    // got cmd
    cmd_interface_->SetSwitchStaus( robot_ctrl_->GetSwitchStatus() );
    cmd_interface_->SetFsmState( robot_ctrl_->GetFsmMode() );
    cmd_interface_->PrepareCmd( control_parameters_->use_rc, &control_parameters_->control_mode, &control_parameters_->gait_id, robot_type_ );
    if ( control_parameters_->lcm_debug_switch == 1 )
        lcm_.publish( "motion_control_cmd", ( robot_control_cmd_lcmt* )&cmd_interface_->GetCommand() );
    // publish the state by lcm
    // publishLcmFeedback();

    // todo safety checks, sanity checks, etc...
}

/**
 * @brief After the user code, send leg commands, update state estimate, and publish debug data.
 *
 */
void RobotRunner::FinalizeStep() {
    if ( robot_type_ == RobotType::CYBERDOG || robot_type_ == RobotType::CYBERDOG2 ) {
        int mode = robot_ctrl_->GetFsmMode();
        if ( mode == static_cast< int >( MotionMode::kLocomotion ) || mode == static_cast< int >( MotionMode::kRlReset ) )
            leg_controller_->UpdateCommand( spi_command_, 1 );  // spi_cmd & pos_limit_methord
        else
            leg_controller_->UpdateCommand( spi_command_, 0 );  // spi_cmd & pos_limit_methord
    }
    else {
        assert( false );
    }
    leg_controller_->SetLcm( &leg_control_data_lcm_, &leg_control_command_lcm_ );
    /***publish by independent thread***/
    // lcm_.publish("leg_control_command", &leg_control_command_lcm_);
    // lcm_.publish("leg_control_data", &leg_control_data_lcm_);

    global_to_robot_lcmt_.rpy[ 0 ]       = state_estimator_lcm_.rpy[ 0 ];
    global_to_robot_lcmt_.rpy[ 1 ]       = state_estimator_lcm_.rpy[ 1 ];
    global_to_robot_lcmt_.rpy[ 2 ]       = state_estimator_lcm_.rpy[ 2 ];
    global_to_robot_lcmt_.omegaBody[ 0 ] = state_estimator_lcm_.omegaBody[ 0 ];
    global_to_robot_lcmt_.omegaBody[ 1 ] = state_estimator_lcm_.omegaBody[ 1 ];
    global_to_robot_lcmt_.omegaBody[ 2 ] = state_estimator_lcm_.omegaBody[ 2 ];
#if defined USE_ABSOLUTE_ODOM_ONLY_FOR_VISION
    global_to_robot_lcmt_.xyz[ 0 ]   = state_estimator_lcm_.p_abs[ 0 ];
    global_to_robot_lcmt_.xyz[ 1 ]   = state_estimator_lcm_.p_abs[ 1 ];
    global_to_robot_lcmt_.xyz[ 2 ]   = state_estimator_lcm_.p_abs[ 2 ];
    global_to_robot_lcmt_.vxyz[ 0 ]  = state_estimator_lcm_.vWorld_abs[ 0 ];
    global_to_robot_lcmt_.vxyz[ 1 ]  = state_estimator_lcm_.vWorld_abs[ 1 ];
    global_to_robot_lcmt_.vxyz[ 2 ]  = state_estimator_lcm_.vWorld_abs[ 2 ];
    global_to_robot_lcmt_.vBody[ 0 ] = state_estimator_lcm_.vBody_abs[ 0 ];
    global_to_robot_lcmt_.vBody[ 1 ] = state_estimator_lcm_.vBody_abs[ 1 ];
    global_to_robot_lcmt_.vBody[ 2 ] = state_estimator_lcm_.vBody_abs[ 2 ];
#else
    global_to_robot_lcmt_.xyz[ 0 ]   = state_estimator_lcm_.p[ 0 ];
    global_to_robot_lcmt_.xyz[ 1 ]   = state_estimator_lcm_.p[ 1 ];
    global_to_robot_lcmt_.xyz[ 2 ]   = state_estimator_lcm_.p[ 2 ];
    global_to_robot_lcmt_.vxyz[ 0 ]  = state_estimator_lcm_.vWorld[ 0 ];
    global_to_robot_lcmt_.vxyz[ 1 ]  = state_estimator_lcm_.vWorld[ 1 ];
    global_to_robot_lcmt_.vxyz[ 2 ]  = state_estimator_lcm_.vWorld[ 2 ];
    global_to_robot_lcmt_.vBody[ 0 ] = state_estimator_lcm_.vBody[ 0 ];
    global_to_robot_lcmt_.vBody[ 1 ] = state_estimator_lcm_.vBody[ 1 ];
    global_to_robot_lcmt_.vBody[ 2 ] = state_estimator_lcm_.vBody[ 2 ];
#endif
    /***publish by independent thread***/
    // lcm_.publish("global_to_robot", &global_to_robot_lcmt_);
    iterations_++;
    PublishLcmFeedback();
}

void RobotRunner::PublishCyberdogLcmFeedback() {
    // get pattern
    int8_t pattern;
    int8_t mode;
    auto   cmd = cmd_interface_->GetCommand();
    Mode2Pattern( robot_ctrl_->GetFsmMode(), cmd.gait_id, pattern );
    Order2Mode( cmd.gait_id, mode );
    motion_control_response_lcmt lcm_data;
    lcm_data.pattern           = robot_ctrl_->GetFsmSwitchFlag() ? pattern : 0;
    lcm_data.order             = robot_ctrl_->GetFsmMode() == mode ? cmd.gait_id : 0;
    lcm_data.order_process_bar = robot_ctrl_->GetFsmProcessBar();
    lcm_data.foot_contact      = 0;

    for ( int i = 0; i < 4; i++ ) {
        double fforce = leg_controller_->datas_[ i ].foot_force_actual.norm();
        if ( fforce > 5.0 )
            lcm_data.foot_contact |= ( 1 << i );
    }

    lcm_data.error_flag.exist_error   = robot_ctrl_->GetSafetyCheckErrorFlag() || robot_ctrl_->GetMotorErrorFlag();
    lcm_data.error_flag.ori_error     = robot_ctrl_->GetOriErrorFlag();
    lcm_data.error_flag.footpos_error = robot_ctrl_->GetFootPosError();
    for ( int i = 0; i < 12; i++ )
        lcm_data.error_flag.motor_error[ i ] = spi_data_->flags[ i ];
    /***publish by independent thread***/
    if ( iterations_ % 10 == 1 )
        send_to_ros_lcm_cyberdog_.publish( "exec_response", &lcm_data );
}

void RobotRunner::PublishLcmFeedback() {

    if ( robot_type_ == RobotType::CYBERDOG )
        return;
    robot_control_response_lcmt lcm_data;
    lcm_data.mode              = robot_ctrl_->GetFsmMode();
    lcm_data.gait_id           = robot_ctrl_->GetFsmGaitId();
    lcm_data.switch_status     = robot_ctrl_->GetSwitchStatus();
    lcm_data.order_process_bar = robot_ctrl_->GetFsmProcessBar();
    lcm_data.contact           = 0;
    lcm_data.footpos_error     = 0;

    for ( int i = 0; i < 4; i++ ) {
        double fforce = leg_controller_->datas_[ i ].foot_force_actual.norm();
        if ( fforce > 5.0 )
            lcm_data.contact |= ( 1 << i );
    }
    lcm_data.ori_error   = robot_ctrl_->GetOriErrorFlag();
    int32_t footpos_flag = robot_ctrl_->GetFootPosError();
    for ( int i = 0; i < 4; i++ )
        lcm_data.footpos_error |= ( ( footpos_flag & ( 0x07 << ( i * 8 ) ) ) >> ( i * 5 ) );  // Compress int32_t to int16_t;
    for ( int i = 0; i < 12; i++ )
        lcm_data.motor_error[ i ] = spi_data_->flags[ i ];
    if ( lcm_data_old_.switch_status != lcm_data.switch_status || ( iterations_ % 10 == 0 ) || ( lcm_data.order_process_bar >= 95 && lcm_data_old_.order_process_bar != 100 ) )
        send_to_ros_lcm_.publish( "robot_control_response", &lcm_data );
    lcm_data_old_ = lcm_data;
}

void RobotRunner::PublishLcmMotorStates() {
    danger_states_lcmt motor_tmp;
    for ( int leg = 0; leg < 4; leg++ ) {
        motor_tmp.motor_temperature[ 3 * leg ]     = 1.0 * spi_data_->tmp_abad[ leg ] / 10.0;
        motor_tmp.motor_temperature[ 3 * leg + 1 ] = 1.0 * spi_data_->tmp_hip[ leg ] / 10.0;
        motor_tmp.motor_temperature[ 3 * leg + 2 ] = 1.0 * spi_data_->tmp_knee[ leg ] / 10.0;
    }
    if ( lcm_iterations_ % 500 == 3 )
        send_to_ros_lcm_motor_.publish( "motor_temperature", &motor_tmp );
}

/**
 * @brief Reset the state estimator in the given mode.
 *
 * @param cheaterMode
 */
void RobotRunner::InitializeStateEstimator( bool cheaterMode ) {
    state_estimator_->RemoveAllEstimators();
    state_estimator_->AddEstimator< ContactEstimator< float > >();
    Vec4< float > contactDefault;
    contactDefault << 0.5, 0.5, 0.5, 0.5;
    state_estimator_->SetContactPhase( contactDefault );
    if ( cheaterMode ) {
        state_estimator_->AddEstimator< CheaterOrientationEstimator< float > >();
        state_estimator_->AddEstimator< CheaterPositionVelocityEstimator< float > >();
        state_estimator_->AddEstimator< CheaterBatEstimator< float > >();
    }
    else {
        state_estimator_->AddEstimator< VectorNavOrientationEstimator< float > >();
#ifdef USE_ABSOLUTE_ODOM_ONLY_FOR_VISION
        state_estimator_->AddEstimator< AbsolutePositionVelocityEstimator< float > >();
        state_estimator_->AddEstimator< LinearKFPositionVelocityEstimator< float > >();
#elif defined USE_ABSOLUTE_ODOM_FOR_ALL
        state_estimator_->AddEstimator< AbsolutePositionVelocityEstimator< float > >();
#else
        state_estimator_->AddEstimator< LinearKFPositionVelocityEstimator< float > >();
#endif
        if ( robot_type_ == RobotType::CYBERDOG2 )
            state_estimator_->AddEstimator< NewBatEstimator< float > >();
        else
            state_estimator_->AddEstimator< BatEstimator< float > >();
    }
    state_estimator_->AddEstimator< TerrainEstimator< float > >();
    state_estimator_->AddEstimator< FootForceContactEstimator< float > >();
}

RobotRunner::~RobotRunner() {
    delete leg_controller_;
    delete state_estimator_;
}

void RobotRunner::CleanUp() {}

void RobotRunner::Mode2Pattern( double mode, double gait_num, int8_t& pattern ) {
    if ( mode == kLocomotion ) {
        // in locomotion
        const std::map< int, int > gait_proj_tb = { { kWalk, LCM_GAIT_WALK },
                                                    { kTrotMedium, LCM_GAIT_TROT_MEDIUM },
                                                    { kTrot10v5, LCM_GAIT_TROT },
                                                    { kTrot10v4, LCM_GAIT_TROT_10_4 },
                                                    { kBound, LCM_GAIT_GALLOP },
                                                    { kPronk, LCM_GAIT_PRONK },
                                                    { kPassiveTrot, LCM_GAIT_PASSIVE_TROT },

                                                    { kStand, LCM_GAIT_STAND },
                                                    { kTrot24v16, LCM_GAIT_TROT_24_16 },

                                                    { kSpecialPronk, LCM_GAIT_SPEC_PRONK },
                                                    { kSpecialTrot, LCM_GAIT_SPEC_TROT }
        };
        if ( gait_proj_tb.find( ( int )gait_num ) == gait_proj_tb.end() ) {  // Error!
            pattern               = -1;
            static int error_iter = 0;
            if ( error_iter++ % 1000 == 0 ) {
                // std::cout << "[RobotRunner::Mode2Pattern] failed to find gaitNumber " << gait_num << " in convert table, set pattern to -1" << std::endl;
            }
        }
        else {  // we got the right pattern number
            pattern = gait_proj_tb.at( ( int )gait_num );
        }
    }
    else {
        // in other mode
        const std::map< int, int > mode_proj_tb = {
            { kOff, LCM_GAIT_PASSIVE }, { kQpStand, LCM_GAIT_STAND_B }, { kPureDamper, LCM_GAIT_KNEEL }, { kRecoveryStand, LCM_GAIT_STAND_R }, { kMotion, LCM_GAIT_STAND_R },
        };
        if ( mode_proj_tb.find( ( int )mode ) == mode_proj_tb.end() ) {
            pattern = ( char )233;
        }
        else {
            pattern = mode_proj_tb.at( ( int )mode );
        }
    }
}

void RobotRunner::Order2Mode( int order, int8_t& mode ) {
    const int order_proj_tb[ 5 ] = { kMotion, kMotion, kTwoLegStand, kRecoveryStand, kMotion };

    mode = -1;
    if ( order >= 14 && order <= 18 )
        mode = order_proj_tb[ order - 14 ];
}
