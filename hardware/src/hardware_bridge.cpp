#include <arpa/inet.h>
#include <cstring>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "Configuration.h"
#include "ParamHandler.hpp"
#include "control_flags.hpp"
#include "hardware_bridge.hpp"
#include "parameters.hpp"
#include "rt/handle_bms_client.hpp"
#include "rt/handle_new_board_bms_client.hpp"
#include "rt/imu_online_calibrate.h"
#include "rt/rt_sbus.h"
#include "utilities/timer.hpp"

#ifdef linux
#include <sys/stat.h>
#include <sys/timerfd.h>
#endif

#define DEBUG_PRINTF 0

/**
 * @brief If an error occurs during initialization, before motors are enabled, print
 *        error and exit.
 *
 * @param reason Error message string
 * @param print_errno If true, also print C errno
 */
void HardwareBridge::InitError( const char* reason, bool print_errno ) {
    printf( "FAILED TO INITIALIZE HARDWARE: %s\n", reason );

    if ( print_errno ) {
        printf( "Error: %s\n", strerror( errno ) );
    }

    exit( -1 );
}

/**
 * @brief Initialize all hardware communication.
 *
 */
void HardwareBridge::InitCommon() {
    printf( "[HardwareBridge] Init stack\n" );
    PrefaultStack();
    printf( "[HardwareBridge] Init scheduler\n" );
    SetupScheduler();
    if ( !interface_lcm_.good() )
        InitError( "interface_lcm_ failed to initialize\n", false );
    if ( !interface_lcm_r_.good() )
        InitError( "interface_lcm_r_ failed to initialize\n", false );
    if ( !recv_from_ros_lcm_.good() )
        InitError( "recv_from_ros_lcm_ failed to initialize\n", false );
    if ( !recv_from_cyberdog_ros_lcm_.good() )
        InitError( "recv_from_cyberdog_ros_lcm_ failed to initialize\n", false );
    if ( !motor_ctrl_lcm_.good() )
        InitError( "motor_ctrl_lcm_ failed to initialize\n\n" );
    printf( "[HardwareBridge] Subscribe LCM\n" );
    interface_lcm_.subscribe( "interface", &HardwareBridge::HandleGamepadLcm, this );
    interface_lcm_r_.subscribe( "interface_request", &HardwareBridge::HandleControlParameter, this );
    recv_from_ros_lcm_.subscribe( "robot_control_cmd", &HardwareBridge::LcmCmdCallback, this );
    user_gait_file_lcm_.subscribe( "user_gait_file", &HardwareBridge::UserGaitFileCallback, this );
    recv_from_cyberdog_ros_lcm_.subscribe( LCM_CMD_CHANNEL_NAME, &HardwareBridge::CyberdogLcmCmdCallback, this );
    motion_list_lcm_.subscribe( "motion-list", &HardwareBridge::LcmMotionCallback, this );
    motor_ctrl_lcm_.subscribe( "motor_ctrl", &HardwareBridge::LcmMotorCtrlCallback, this );

    printf( "[HardwareBridge] Start interface LCM handler\n" );
    interface_lcm_thread_              = std::thread( &HardwareBridge::HandleInterfaceLcm, this );
    interface_lcm_thread_r_            = std::thread( &HardwareBridge::HandleInterfaceLcmR, this );
    interface_lcm_thread_cmd_          = std::thread( &HardwareBridge::HandleInterfaceLcmCmd, this );
    interface_lcm_thread_cyberdog_cmd_ = std::thread( &HardwareBridge::HandleInterfaceLcmCyberdogCmd, this );
    interface_lcm_thread_usergait_     = std::thread( &HardwareBridge::HandleInterfaceLcmUsergaitFile, this );
    interface_lcm_thread_motion_list_  = std::thread( &HardwareBridge::HandleInterfaceLcmMotionList, this );
    interface_motor_ctrl_thread_       = std::thread( &HardwareBridge::HandleMotorCtrlLcmThread, this );
}

/**
 * @brief Run interface LCM.
 *
 */
void HardwareBridge::HandleInterfaceLcm() {
    while ( !interface_lcm_quit_ ) {
        interface_lcm_.handle();
    }
}

/**
 * @brief Run interface LCM.
 *
 */
void HardwareBridge::HandleInterfaceLcmR() {
    while ( !interface_lcm_quit_ ) {
        interface_lcm_r_.handle();
    }
}

/**
 * @brief Run interface LCM.
 *
 */
void HardwareBridge::HandleInterfaceLcmCmd() {
    while ( !interface_lcm_quit_ ) {
        recv_from_ros_lcm_.handle();
    }
}

/**
 * @brief Run interface LCM.
 *
 */
void HardwareBridge::HandleInterfaceLcmCyberdogCmd() {
    while ( !interface_lcm_quit_ ) {
        recv_from_cyberdog_ros_lcm_.handle();
    }
}

/**
 * @brief Run interface LCM.
 *
 */
void HardwareBridge::HandleInterfaceLcmMotionList() {
    while ( !interface_lcm_quit_ ) {
        motion_list_lcm_.handle();
    }
}

/**
 * @brief Run interface LCM.
 *
 */
void HardwareBridge::HandleInterfaceLcmUsergaitFile() {
    while ( !interface_lcm_quit_ ) {
        user_gait_file_lcm_.handle();
    }
}

/**
 * @brief Handles the motor control LCM thread.
 *
 */
void HardwareBridge::HandleMotorCtrlLcmThread() {
    while ( !interface_lcm_quit_ ) {
        motor_ctrl_lcm_.handle();
    }
}

/**
 * @brief Writes to a 16 KB buffer on the stack. If we are using 4K pages for our
 *        stack, this will make sure that we won't have a page fault when the stack
 *        grows.  Also mlock's all pages associated with the current process, which
 *        prevents the cyberdog2 software from being swapped out.  If we do run out of
 *        memory, the robot program will be killed by the OOM process killer (and
 *        leaves a log) instead of just becoming unresponsive.
 *
 */
void HardwareBridge::PrefaultStack() {
    printf( "[Init] Prefault stack...\n" );
    volatile char stack[ MAX_STACK_SIZE ];
    memset( const_cast< char* >( stack ), 0, MAX_STACK_SIZE );
    if ( mlockall( MCL_CURRENT | MCL_FUTURE ) == -1 ) {
        InitError( "mlockall failed.  This is likely because you didn't run robot as "
                   "root.\n",
                   true );
    }
}

/**
 * @brief Configures the scheduler for real time priority.
 *
 */
void HardwareBridge::SetupScheduler() {
    printf( "[Init] Setup RT Scheduler...\n" );
    struct sched_param params;
    params.sched_priority = TASK_PRIORITY;
    if ( sched_setscheduler( 0, SCHED_FIFO, &params ) == -1 ) {
        InitError( "sched_setscheduler failed.\n", true );
    }
}

/**
 * @brief LCM Handler for gamepad message.
 *
 * @param rbuf Pointer to the receive buffer
 * @param chan The channel name
 * @param msg Pointer to the gamepad_lcmt message received
 */
void HardwareBridge::HandleGamepadLcm( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const gamepad_lcmt* msg ) {
    ( void )rbuf;
    ( void )chan;
    cmd_interface_.ProcessGamepadCommand( msg );
}

/**
 * @brief LCM Handler for control parameters.
 *
 * @param rbuf Pointer to the receive buffer
 * @param chan The channel name
 * @param msg Pointer to the control_parameter_request_lcmt message
 */
void HardwareBridge::HandleControlParameter( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const control_parameter_request_lcmt* msg ) {
    ( void )rbuf;
    ( void )chan;
    if ( msg->requestNumber <= parameter_response_lcmt_.requestNumber ) {
        // nothing to do!
        printf( "[HardwareBridge] Warning: the interface has run a ControlParameter "
                "iteration, but there is no new request!\n" );
        std::cout << "the msg->requestNumber is: " << msg->requestNumber << " ;and the parameter_response_lcmt_.requestNumber is: " << parameter_response_lcmt_.requestNumber << std::endl;
        // return;
    }

    // sanity check
    s64 nRequests = msg->requestNumber - parameter_response_lcmt_.requestNumber;
    if ( nRequests != 1 ) {
        printf( "[ERROR] Hardware bridge: we've missed %ld requests\n", nRequests - 1 );
    }

    switch ( msg->requestKind ) {
    case ( s8 )ControlParameterRequestKind::kSET_USER_PARAM_BY_NAME: {
        if ( !user_control_parameters_ ) {
            printf( "[Warning] Got user param %s, but not using user parameters!\n", ( char* )msg->name );
        }
        else {
            std::string       name( ( char* )msg->name );
            ControlParameter& param = user_control_parameters_->collection_.LookUp( name );

            GetParaResponseLcm( msg, param );
        }
    } break;
    case ( s8 )ControlParameterRequestKind::kSET_ROBOT_PARAM_BY_NAME: {
        std::string       name( ( char* )msg->name );
        ControlParameter& param = robot_params_.collection_.LookUp( name );

        GetParaResponseLcm( msg, param );
    } break;
    case ( s8 )ControlParameterRequestKind::kSET_SPEED_CALIBRATE_PARAM_BY_NAME: {
        if ( SPEED_PARAMETER_CALIBRATE_CONTROL_DEBUG ) {
            std::string       name( ( char* )msg->name );
            ControlParameter& param = speed_param_.collection_.LookUp( name );

            GetParaResponseLcm( msg, param );

            robot_params_.speed_offset_trot_10_4   = speed_param_.speed_offset_trot_10_4;
            robot_params_.speed_offset_trot_follow = speed_param_.speed_offset_trot_follow;
            robot_params_.speed_offset_trot_medium = speed_param_.speed_offset_trot_medium;
            robot_params_.speed_offset_trot_24_16  = speed_param_.speed_offset_trot_24_16;
            robot_params_.speed_offset_trot_slow   = speed_param_.speed_offset_trot_slow;
            robot_params_.speed_offset_trot_fast   = speed_param_.speed_offset_trot_fast;
            robot_params_.speed_offset_trot_8_3    = speed_param_.speed_offset_trot_8_3;
            robot_params_.speed_offset_ballet      = speed_param_.speed_offset_ballet;
            robot_params_.speed_offset_bound       = speed_param_.speed_offset_bound;
            robot_params_.speed_offset_pronk       = speed_param_.speed_offset_pronk;
            robot_params_.se_ori_cali_offset       = speed_param_.se_ori_cali_offset;
            robot_params_.se_ori_cali_gain         = speed_param_.se_ori_cali_gain;

            std::cout << "get msg: kSET_SPEED_CALIBRATE_PARAM_BY_NAME, the parameters will be writed to " << SPEED_CALIBRATE_FILE_PATH << std::endl;

            speed_param_.LockMutex();
            speed_param_.WriteToYamlFile( SPEED_CALIBRATE_FILE_PATH );
            speed_param_.UnlockMutex();
        }
    } break;
    case ( s8 )ControlParameterRequestKind::kSET_IMU_CALIBRATE_PARAM_BY_NAME: {
        std::string       name( ( char* )msg->name );
        ControlParameter& param = imu_param_.collection_.LookUp( name );

        GetParaResponseLcm( msg, param );
        std::cout << "*******************GetParaResponseLcm::kSET_IMU_CALIBRATE_PARAM_BY_NAME,and break***********************" << std::endl;
    } break;
    case ( s8 )ControlParameterRequestKind::kGET_SPEED_CALIBRATE_PARAM_BY_NAME: {
        if ( SPEED_PARAMETER_CALIBRATE_CONTROL_DEBUG ) {
            std::string       name( ( char* )msg->name );
            ControlParameter& param = speed_param_.collection_.LookUp( name );

            GetParaResponseLcm( msg, param, false );
            std::cout << "*******************GetParaResponseLcm::kGET_SPEED_CALIBRATE_PARAM_BY_NAME,and break***********************" << std::endl;
        }
    } break;
    default: {
        throw std::runtime_error( "parameter type unsupported" );
    } break;
    }

    std::cout << "*******************publish interface_response***********************" << std::endl;
    interface_lcm_r_.publish( "interface_response", &parameter_response_lcmt_ );
}

/**
 * @brief Constructs a Cyberdog2HardwareBridge object.
 *
 * @param robot_ctrl Pointer to the RobotController object
 * @param load_parameters_from_file Boolean indicating whether to load parameters from a file
 * @param robot_type The type of the robot
 */
Cyberdog2HardwareBridge::Cyberdog2HardwareBridge( RobotController* robot_ctrl, bool load_parameters_from_file, const RobotType& robot_type )
#ifdef USE_EXTERNAL_IMU
    : HardwareBridge( robot_ctrl ), spi_lcm_( GetLcmUrl( 255 ) ), imu_lcm_( GetLcmUrl( 255 ) ), external_imu_lcm_( GetLcmUrl( 255 ) ) {
#else
    : HardwareBridge( robot_ctrl ), spi_lcm_( GetLcmUrl( 255 ) ), imu_lcm_( GetLcmUrl( 255 ) ) {
#endif
    load_parameters_from_file_ = load_parameters_from_file;
    with_imu_                  = true;
    speed_offset_flag_         = 1;
    is_imu_params_in_yaml_     = true;
    robot_type_                = robot_type;
}

/**
 * @brief Receive Rc socket msg from Uart_to_UDP module(USR-K7).
 *
 */
void Cyberdog2HardwareBridge::HandleInterfaceRcRec() {
    uint16_t channels[ 18 ];
    int      fd = socket( AF_INET, SOCK_DGRAM, 0 );
    if ( fd == -1 ) {
        printf( "Failed to create Socket for RC_UDP Receive\n" );
        exit( -1 );
    }
    struct sockaddr_in addr;
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons( 23 );
    addr.sin_addr.s_addr = inet_addr( "192.168.44.233" );

    int ret = bind( fd, ( struct sockaddr* )&addr, sizeof( addr ) );
    if ( ret == -1 ) {
        printf( "Failed to bind RC Socket\n" );
        exit( -1 );
    }
    char                   rec_msg[ 25 ];  //, ipbuf[ 64 ];
    std::vector< uint8_t > datas;
    struct sockaddr_in     cliaddr;
    uint                   len = sizeof( cliaddr );

    while ( true ) {
        memset( rec_msg, 0, sizeof( rec_msg ) );
        int rlen = recvfrom( fd, rec_msg, sizeof( rec_msg ), 0, ( struct sockaddr* )&cliaddr, &len );
        for ( int i = 0; i < rlen; i++ )
            datas.push_back( rec_msg[ i ] );
        // printf("IP:%s@%d Len:%d:",inet_ntop(AF_INET, &cliaddr.sin_addr.s_addr, ipbuf, sizeof(ipbuf)), ntohs(cliaddr.sin_port), rlen);
        // for (int j=0;j<25;j++)
        //     printf(" 0x%x",rec_msg[j]);
        // printf("\n");
        while ( datas.size() >= 25 ) {
            if ( ( datas[ 0 ] == 15 ) && ( datas[ 24 ] == 0 ) && ( ( datas[ 23 ] & 0x0C ) == 0 ) ) {
                for ( int i = 0; i < 25; i++ ) {
                    rec_msg[ i ] = datas[ 0 ];
                    datas.erase( datas.begin() );
                }
                UnpackSbusData( ( uint8_t* )rec_msg, channels );
                RcCommand* rc_udp_cmd = SbusPacketCompleteAt9s( channels );
                cmd_interface_.ProcessRcUdpCommand( rc_udp_cmd );
            }
            else
                datas.erase( datas.begin() );
        }
    }
    close( fd );
}

/**
 * @brief Main method for Cyberdog2 hardware.
 *
 */
void Cyberdog2HardwareBridge::Run() {
    InitCommon();

    if ( robot_type_ == RobotType::CYBERDOG2 )
        interface_rc_thread_ = std::thread( &Cyberdog2HardwareBridge::HandleInterfaceRcRec, this );

    if ( SPEED_PARAMETER_CALIBRATE_CONTROL_DEBUG ) {
        // Load Speed calibration result
        std::fstream speed_param_fs;
        speed_param_fs.open( SPEED_CALIBRATE_FILE_PATH, std::ios::in );
        if ( speed_param_fs )
            speed_param_.InitializeFromYamlFile( SPEED_CALIBRATE_FILE_PATH );
        if ( !speed_param_fs || !speed_param_.IsFullyInitialized() ) {
            int ret = system( "mount -o remount,rw /mnt/misc/" );
            if ( ret < 0 )
                exit( 1 );
            system( "cp -f ../common/config/speed_calibrate_param_v1.1.yaml /mnt/misc/" );
            system( "sync" );
            speed_param_.InitializeFromYamlFile( SPEED_CALIBRATE_FILE_PATH );
            if ( !speed_param_.IsFullyInitialized() ) {
                printf( "Failed to initialize all speed parameters\n" );
                exit( 1 );
            }
        }
        std::cout << "[HardwareBridge] load speed parameters" << std::endl;
        speed_thread_ = std::thread( &Cyberdog2HardwareBridge::RunSpeedConfig, this );
        usleep( 1000 );
        // end of tmp speed offset calibration
    }

    if ( load_parameters_from_file_ ) {
        printf( "[Hardware Bridge] Loading parameters from file...\n" );

        try {
            robot_params_.InitializeFromYamlFile( THIS_COM "common/config/robot-defaults.yaml" );

            if ( SPEED_PARAMETER_CALIBRATE_CONTROL_DEBUG ) {
                robot_params_.speed_offset_trot_10_4 += speed_param_.speed_offset_trot_10_4;
                robot_params_.speed_offset_trot_follow += speed_param_.speed_offset_trot_follow;
                robot_params_.speed_offset_trot_medium += speed_param_.speed_offset_trot_medium;
                robot_params_.speed_offset_trot_24_16 += speed_param_.speed_offset_trot_24_16;
                robot_params_.speed_offset_trot_slow += speed_param_.speed_offset_trot_slow;
                robot_params_.speed_offset_trot_fast += speed_param_.speed_offset_trot_fast;
                robot_params_.speed_offset_trot_8_3 += speed_param_.speed_offset_trot_8_3;
                robot_params_.speed_offset_ballet += speed_param_.speed_offset_ballet;
                robot_params_.speed_offset_bound += speed_param_.speed_offset_bound;
                robot_params_.speed_offset_pronk += speed_param_.speed_offset_pronk;
                robot_params_.se_ori_cali_offset += speed_param_.se_ori_cali_offset;
                robot_params_.se_ori_cali_gain += speed_param_.se_ori_cali_gain;
            }

            std::cout << "**robot_params_.speed_offset_trot_slow: **** " << robot_params_.speed_offset_trot_slow.transpose() << std::endl;
            std::cout << "**robot_params_.speed_offset_trot_medium: ** " << robot_params_.speed_offset_trot_medium.transpose() << std::endl;
            std::cout << "**robot_params_.speed_offset_trot_fast: **** " << robot_params_.speed_offset_trot_fast.transpose() << std::endl;
            std::cout << "**robot_params_.speed_offset_trot_follow: ** " << robot_params_.speed_offset_trot_follow.transpose() << std::endl;
            std::cout << "**robot_params_.speed_offset_ballet: ******* " << robot_params_.speed_offset_ballet.transpose() << std::endl;
            std::cout << "**robot_params_.speed_offset_bound: ******** " << robot_params_.speed_offset_bound.transpose() << std::endl;
            std::cout << "**robot_params_.speed_offset_pronk: ******** " << robot_params_.speed_offset_pronk.transpose() << std::endl;
            std::cout << "**robot_params_.speed_offset_trot_8_3: ***** " << robot_params_.speed_offset_trot_8_3.transpose() << std::endl;
            std::cout << "**robot_params_.speed_offset_trot_10_4: **** " << robot_params_.speed_offset_trot_10_4.transpose() << std::endl;
            std::cout << "**robot_params_.speed_offset_trot_24_16: *** " << robot_params_.speed_offset_trot_24_16.transpose() << std::endl;
            std::cout << "**robot_params_.se_ori_cali_offset: ******** " << robot_params_.se_ori_cali_offset.transpose() << std::endl;
            std::cout << "**robot_params_.se_ori_cali_gain: ********** " << robot_params_.se_ori_cali_gain.transpose() << std::endl;
        }
        catch ( std::exception& e ) {
            printf( "Failed to initialize robot parameters from yaml file: %s\n", e.what() );
            exit( 1 );
        }

        if ( !robot_params_.IsFullyInitialized() ) {
            printf( "Failed to initialize all robot parameters\n" );
            exit( 1 );
        }

        printf( "Loaded robot parameters\n" );

        if ( user_control_parameters_ ) {
            try {
                if ( robot_type_ == RobotType::CYBERDOG )
                    user_control_parameters_->InitializeFromYamlFile( THIS_COM "common/config/cyberdog-ctrl-user-parameters.yaml" );
                else if ( robot_type_ == RobotType::CYBERDOG2 )
                    user_control_parameters_->InitializeFromYamlFile( THIS_COM "common/config/cyberdog2-ctrl-user-parameters.yaml" );
            }
            catch ( std::exception& e ) {
                printf( "Failed to initialize user parameters from yaml file: %s\n", e.what() );
                exit( 1 );
            }

            if ( !user_control_parameters_->IsFullyInitialized() ) {
                printf( "Failed to initialize all user parameters\n" );
                exit( 1 );
            }

            printf( "Loaded user parameters\n" );
        }
        else {
            printf( "Did not load user parameters because there aren't any\n" );
        }
    }
    else {
        printf( "[Hardware Bridge] Loading parameters over LCM...\n" );
        while ( !robot_params_.IsFullyInitialized() ) {
            printf( "[Hardware Bridge] Waiting for robot parameters...\n" );
            usleep( 1000000 );
        }

        if ( user_control_parameters_ ) {
            while ( !user_control_parameters_->IsFullyInitialized() ) {
                printf( "[Hardware Bridge] Waiting for user parameters...\n" );
                usleep( 1000000 );
            }
        }
    }
    // select communicate type of imu by board id
    int ctrl_board_id = GetBoardId();
    if ( ctrl_board_id < 2 ) {
        use_imu_by_spi_ = false;
        std::cout << "[HardwareBridge] Read Imu Data By Serial!" << std::endl;
    }
    else {
        use_imu_by_spi_ = true;
        std::cout << "[HardwareBridge] Read Imu Data By Spi!" << std::endl;
    }

    // get robot type ( different outer shell ) by shell command
    appearance_type_ = GetRobotAppearanceType( ctrl_board_id );

    InitHardware();

    printf( "[Hardware Bridge] Got robot&user parameters, starting up!\n" );

    // init control thread
    robot_runner_ = new RobotRunnerInterface( controller_, &task_manager_, robot_params_.controller_dt, "robot-control" );

    robot_runner_->SetCommandInterface( &cmd_interface_ );
    robot_runner_->SetSpiData( &spi_data_ );
    robot_runner_->SetSpiCommand( &spi_command_ );
    robot_runner_->SetRobotType( robot_type_ );
    robot_runner_->SetRobotAppearanceType( appearance_type_ );
#ifdef USE_EXTERNAL_IMU
    robot_runner_->SetVectorNavData( &external_vector_nav_data_ );
#else
    robot_runner_->SetVectorNavData( &vector_nav_data_ );
#endif
    robot_runner_->SetRobotControlParameters( &robot_params_ );
    robot_runner_->SetVisualizationData( &visualization_data_ );
    robot_runner_->SetCyberdog2Visualization( &cyberdog2_visualization_ );

    // spi Task start
    spi_handler_.FlushSpiBuffer();
    PeriodicMemberFunction< Cyberdog2HardwareBridge > spiTask( &task_manager_, robot_params_.controller_dt, "spi", &Cyberdog2HardwareBridge::RunSpi, this );
    spiTask.StartTask();

    std::cout << "[HardwareBridge] with_imu: " << with_imu_ << std::endl;
    if ( with_imu_ ) {
#ifndef USE_EXTERNAL_IMU
        // Load IMU calibration result
        imu_param_.InitializeFromYamlFile( IMU_CALIBRATE_FILE_PATH );
        if ( !imu_param_.IsFullyInitialized() ) {
            printf( "Failed to initialize all IMU parameters from yaml\n" );
            is_imu_params_in_yaml_ = false;
        }
        std::cout << "[HardwareBridge] load IMU parameters" << std::endl;

        // Load Pose calibration result
        pos_param_.InitializeFromYamlFile( POSE_CALIBRATE_PARAM_FILE_PATH );
        if ( !pos_param_.IsFullyInitialized() ) {
            printf( "Failed to initialize all Pose parameters\n" );
            exit( 1 );
        }
        std::cout << "[HardwareBridge] load Pose parameters" << std::endl;
#endif
#ifdef USE_EXTERNAL_IMU
        if ( !use_imu_by_spi_ )
            external_imu_thread_ = std::thread( &Cyberdog2HardwareBridge::RunExternalImuSerial, this );
        else {
            spi_handler_for_imu_.FlushSpiBuffer();
            external_imu_thread_ = std::thread( &Cyberdog2HardwareBridge::RunSpiForImu, this );
            // PeriodicMemberFunction< Cyberdog2HardwareBridge > spiForImuTask( &task_manager_, robot_params_.controller_dt, "spiForImu", &Cyberdog2HardwareBridge::RunSpiForImu, this );
            // spiForImuTask.StartTask();
        }

        static int         policy;
        static sched_param sch_params;
        policy                    = SCHED_RR;
        sch_params.sched_priority = sched_get_priority_max( policy );
        pthread_setschedparam( external_imu_thread_.native_handle(), policy, &sch_params );
        int rc = pthread_getschedparam( external_imu_thread_.native_handle(), &policy, &sch_params );
        std::cout << "[check pthread]  "
                  << "external_imu_thread_"
                  << "   " << rc << "   " << policy << "   " << sch_params.sched_priority << std::endl;
#else
        imu_thread_ = std::thread( &Cyberdog2HardwareBridge::RunImuSerial, this );
#endif

        // waiting a little while for imu update
        Timer imu_ready;
        imu_ready.StartTimer();
        while ( first_imu_ ) {
            usleep( 100 );
            if ( imu_ready.GetElapsedSeconds() >= 2.0 ) {
                std::cout << "[check imu] imu do not commucinate successfully" << std::endl;
                imu_ready.StartTimer();
            }
        }
    }

    // visualization start
    PeriodicMemberFunction< Cyberdog2HardwareBridge > visualizationLCMTask( &task_manager_, .0167, "lcm-vis", &Cyberdog2HardwareBridge::PublishVisualizationLcm, this );
    visualizationLCMTask.StartTask();

    // rc controller
    port_ = InitSbus( false );
    PeriodicMemberFunction< HardwareBridge > sbusTask( &task_manager_, .005, "rc_controller", &HardwareBridge::RunSbus, this );
    sbusTask.StartTask();

    // robot controller start
    robot_runner_->Start();

    // publish lcm's message start
    PeriodicMemberFunction< Cyberdog2HardwareBridge > debugLCMTask( &task_manager_, robot_params_.controller_dt, "lcm-debug", &Cyberdog2HardwareBridge::PublishDebugLcm, this );
    debugLCMTask.StartTask();

    // Construct one BMS object
    // TODO: Cyberdog2 Nx application board will use can bus communicating with bms
#ifdef USE_NEW_BOARD
    auto sPtrBmsClient = std::make_shared< HandleNewBmsClient >();
    sPtrBmsClient->SetLowPowerEnable( robot_runner_->GetLowPowerEnable() );
    sPtrBmsClient->RunBmsClient( "" );
    robot_runner_->SetBmsStatus( ( sPtrBmsClient->GetBmsStatus() ) );
    robot_runner_->SetBattSoc( ( sPtrBmsClient->GetBatterySoc() ) );
#else
    auto sPtrBmsClient = std::make_shared< HandleBmsClient >();
    sPtrBmsClient->RunBmsClient( "" );
#endif

    if ( robot_type_ == RobotType::CYBERDOG ) {
        std::shared_ptr< MessageHandler > sPtrMsgHandler = std::make_shared< MessageHandler >();
        sPtrMsgHandler->Run();
    }

    for ( ;; ) {
        usleep( 1000000 );
        // printf("joy %f\n", robot_runner_->driverCommand->leftStickAnalog[0]);
    }
}

/**
 * @brief Publish debug lcm.
 *
 */
void HardwareBridge::PublishDebugLcm() {
    robot_runner_->LCMPublishByThread();
}

/**
 * @brief Receive RC with SBUS.
 *
 */
void HardwareBridge::RunSbus() {
    uint16_t channels[ 18 ];
    if ( port_ > 0 ) {
        int x = ReceiveSbus( port_, channels );

        if ( x == 1 ) {  // received sbus package
            RcCommand* rc_cmd = SbusPacketCompleteAt9s( channels );
            cmd_interface_.ProcessRcCommand( rc_cmd );
        }
    }
}

/**
 * @brief Process lcm cmd.
 *
 * @param buf Pointer to the receive buffer
 * @param channel The channel name
 * @param msg Pointer to the motion_control_request_lcmt message
 */
void HardwareBridge::CyberdogLcmCmdCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const motion_control_request_lcmt* msg ) {
    ( void )buf;
    ( void )channel;
    cmd_interface_.ProcessCyberdogLcmCommand( msg );
}

/**
 * @brief Callback function for user gait file reception.
 *
 * @param buf Pointer to the receive buffer
 * @param channel The channel name
 * @param msg Pointer to the file_send_lcmt message
 */
void HardwareBridge::UserGaitFileCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const file_send_lcmt* msg ) {
    ( void )buf;
    ( void )channel;
    file_recv_lcmt receive_msg;
    receive_msg.result = cmd_interface_.ProcessUserGaitFile( msg );
    user_gait_file_responce_lcm_.publish( "user_gait_result", &receive_msg );
}

/**
 * @brief Process lcm cmd.
 *
 * @param buf Pointer to the receive buffer
 * @param channel The channel name
 * @param msg Pointer to the robot_control_cmd_lcmt message
 */
void HardwareBridge::LcmCmdCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const robot_control_cmd_lcmt* msg ) {
    ( void )buf;
    ( void )channel;
    cmd_interface_.ProcessLcmCommand( msg );
}

/**
 * @brief Process lcm cmd.
 *
 * @param buf Pointer to the receive buffer
 * @param channel The channel name
 * @param msg Pointer to the trajectory_command_lcmt message
 */
void HardwareBridge::LcmMotionCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const trajectory_command_lcmt* msg ) {
    ( void )buf;
    ( void )channel;
    cmd_interface_.ProcessLcmMotionCommand( msg );
}

/**
 * @brief Process lcm cmd.
 *
 * @param buf Pointer to the receive buffer
 * @param channel The channel name
 * @param msg Pointer to the motor_ctrl_lcmt message
 */
void HardwareBridge::LcmMotorCtrlCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const motor_ctrl_lcmt* msg ) {
    ( void )buf;
    ( void )channel;
    cmd_interface_.ProcessLcmMotorCtrlCommand( msg );
}

/**
 * @brief Initialize Cyberdog2 specific hardware.
 *
 */
void Cyberdog2HardwareBridge::InitHardware() {
    vector_nav_data_.quat << 0, 0, 0, 1;
#ifdef USE_EXTERNAL_IMU
    external_vector_nav_data_.quat << 0, 0, 0, 1;
    if ( use_imu_by_spi_ )
        spi_handler_for_imu_.InitializeSpi();
#endif
    spi_handler_.InitializeSpi( robot_type_, appearance_type_ );
}

/**
 * @brief Factory Speed offset config set thread.
 *
 */
void Cyberdog2HardwareBridge::RunSpeedConfig() {
    while ( true ) {
        if ( cmd_interface_.GetSpeedOffsetTrigger() > 0 ) {
            auto cmd           = cmd_interface_.GetCommand();
            speed_offset_flag_ = 1;
            Eigen::Vector3d speed_diff( cmd.vel_des[ 0 ], cmd.vel_des[ 1 ], cmd.vel_des[ 2 ] );
            Eigen::Vector3d rpy_f_gain_diff( 0, cmd.rpy_des[ 2 ], 0 );
            Eigen::Vector3d rpy_b_gain_diff( 0, 0, cmd.rpy_des[ 2 ] );
            Eigen::Vector3d rpy_diff( cmd.rpy_des[ 0 ], cmd.rpy_des[ 1 ], 0 );

            // Speed Offset
            if ( cmd_interface_.GetSpeedOffsetTrigger() == 1 ) {
                if ( cmd.gait_id == kTrotSlow ) {
                    robot_params_.speed_offset_trot_slow += speed_diff;
                    speed_param_.speed_offset_trot_slow += speed_diff;
                }
                if ( cmd.gait_id == kTrotMedium ) {
                    robot_params_.speed_offset_trot_medium += speed_diff;
                    speed_param_.speed_offset_trot_medium += speed_diff;
                }
                if ( cmd.gait_id == kTrotFast ) {
                    robot_params_.speed_offset_trot_fast += speed_diff;
                    speed_param_.speed_offset_trot_fast += speed_diff;
                }
                if ( cmd.gait_id == kTrot20v12Follow ) {
                    robot_params_.speed_offset_trot_follow += speed_diff;
                    speed_param_.speed_offset_trot_follow += speed_diff;
                }
                if ( cmd.gait_id == kTrot24v16 ) {
                    robot_params_.speed_offset_trot_24_16 += speed_diff;
                    speed_param_.speed_offset_trot_24_16 += speed_diff;
                }
                if ( cmd.gait_id == kBallet ) {
                    robot_params_.speed_offset_ballet += speed_diff;
                    speed_param_.speed_offset_ballet += speed_diff;
                }
                if ( cmd.gait_id == kBound ) {
                    robot_params_.speed_offset_bound += speed_diff;
                    speed_param_.speed_offset_bound += speed_diff;
                }
                if ( cmd.gait_id == kPronk ) {
                    robot_params_.speed_offset_pronk += speed_diff;
                    speed_param_.speed_offset_pronk += speed_diff;
                }
                if ( cmd.gait_id == kTrot8v3 ) {
                    robot_params_.speed_offset_trot_8_3 += speed_diff;
                    speed_param_.speed_offset_trot_8_3 += speed_diff;
                }
                std::cout << "Get speed_diff:" << speed_diff.transpose() << std::endl;
            }  // RPY speed + gain Offset
            else if ( cmd_interface_.GetSpeedOffsetTrigger() == 2 ) {
                if ( cmd.gait_id == kTrotFast ) {
                    robot_params_.se_ori_cali_gain += rpy_f_gain_diff;
                    speed_param_.se_ori_cali_gain += rpy_f_gain_diff;
                }
                if ( speed_param_.se_ori_cali_gain( 1 ) > 0.1 || robot_params_.se_ori_cali_gain( 1 ) > 0.1 ) {
                    speed_param_.se_ori_cali_gain( 1 )  = 0.1;
                    robot_params_.se_ori_cali_gain( 1 ) = 0.1;
                }
                else if ( speed_param_.se_ori_cali_gain( 1 ) < 0 || robot_params_.se_ori_cali_gain( 1 ) < 0 ) {
                    speed_param_.se_ori_cali_gain( 1 )  = 0;
                    robot_params_.se_ori_cali_gain( 1 ) = 0;
                }
                std::cout << "Get rpy_f_gain_diff:" << rpy_f_gain_diff.transpose() << std::endl;
            }  // RPY speed - gain Offset
            else if ( cmd_interface_.GetSpeedOffsetTrigger() == 3 ) {
                if ( cmd.gait_id == kTrotFast ) {
                    robot_params_.se_ori_cali_gain += rpy_b_gain_diff;
                    speed_param_.se_ori_cali_gain += rpy_b_gain_diff;
                }
                if ( speed_param_.se_ori_cali_gain( 2 ) > 0.05 || robot_params_.se_ori_cali_gain( 2 ) > 0.05 ) {
                    speed_param_.se_ori_cali_gain( 2 )  = 0.05;
                    robot_params_.se_ori_cali_gain( 2 ) = 0.1;
                }
                else if ( speed_param_.se_ori_cali_gain( 2 ) < -0.02 || robot_params_.se_ori_cali_gain( 2 ) < -0.02 ) {
                    speed_param_.se_ori_cali_gain( 2 )  = -0.02;
                    robot_params_.se_ori_cali_gain( 2 ) = -0.02;
                }
                std::cout << "Get rpy_b_gain_diff:" << rpy_b_gain_diff.transpose() << std::endl;

            }  // RPY Offset
            else if ( cmd_interface_.GetSpeedOffsetTrigger() == 4 ) {
                if ( cmd.gait_id == kTrotSlow || kTrotMedium || kBallet || kTrot20v12Follow ) {
                    robot_params_.se_ori_cali_offset -= rpy_diff;
                    speed_param_.se_ori_cali_offset -= rpy_diff;
                }
                std::cout << "Get rpy_diff:" << rpy_diff.transpose() << std::endl;
            }

            std::cout << "**set robot_params_: " << std::endl;
            std::cout << "**set speed_offset_trot_follow: ** " << robot_params_.speed_offset_trot_follow.transpose() << std::endl;
            std::cout << "**set speed_offset_trot_24_16: *** " << robot_params_.speed_offset_trot_24_16.transpose() << std::endl;
            std::cout << "**set speed_offset_trot_medium: ** " << robot_params_.speed_offset_trot_medium.transpose() << std::endl;
            std::cout << "**set speed_offset_trot_slow: **** " << robot_params_.speed_offset_trot_slow.transpose() << std::endl;
            std::cout << "**set speed_offset_trot_fast: **** " << robot_params_.speed_offset_trot_fast.transpose() << std::endl;
            std::cout << "**set speed_offset_trot_8_3: ***** " << robot_params_.speed_offset_trot_8_3.transpose() << std::endl;
            std::cout << "**set speed_offset_ballet: ******* " << robot_params_.speed_offset_ballet.transpose() << std::endl;
            std::cout << "**set speed_offset_bound: ******** " << robot_params_.speed_offset_bound.transpose() << std::endl;
            std::cout << "**set speed_offset_pronk: ******** " << robot_params_.speed_offset_pronk.transpose() << std::endl;
            std::cout << "**set se_ori_cali_offset: ******** " << robot_params_.se_ori_cali_offset.transpose() << std::endl;
            std::cout << "**set se_ori_cali_gain: ********** " << robot_params_.se_ori_cali_gain.transpose() << std::endl;

            std::ofstream speed_res_f( SPEED_CALIBRATE_FILE_PATH );
            if ( !speed_res_f.is_open() ) {
                std::cout << "FAIL: failed to open speed_calib file:" << SPEED_CALIBRATE_FILE_PATH << std::endl;
                return;
            }

            Eigen::IOFormat CommaInitFmt( Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]" );
            speed_res_f << "__collection-name__: speed-parameters\n" << std::endl;
            speed_res_f << "speed_offset_trot_slow: " << speed_param_.speed_offset_trot_slow.format( CommaInitFmt ) << std::endl;
            speed_res_f << "speed_offset_trot_medium: " << speed_param_.speed_offset_trot_medium.format( CommaInitFmt ) << std::endl;
            speed_res_f << "speed_offset_trot_fast: " << speed_param_.speed_offset_trot_fast.format( CommaInitFmt ) << std::endl;
            speed_res_f << "speed_offset_trot_follow: " << speed_param_.speed_offset_trot_follow.format( CommaInitFmt ) << std::endl;
            speed_res_f << std::endl;
            speed_res_f << "speed_offset_trot_24_16: " << speed_param_.speed_offset_trot_24_16.format( CommaInitFmt ) << std::endl;
            speed_res_f << "speed_offset_trot_10_5: " << speed_param_.speed_offset_trot_10_5.format( CommaInitFmt ) << std::endl;
            speed_res_f << "speed_offset_trot_10_4: " << speed_param_.speed_offset_trot_10_4.format( CommaInitFmt ) << std::endl;
            speed_res_f << "speed_offset_trot_8_3: " << speed_param_.speed_offset_trot_8_3.format( CommaInitFmt ) << std::endl;
            speed_res_f << std::endl;
            speed_res_f << "speed_offset_ballet: " << speed_param_.speed_offset_ballet.format( CommaInitFmt ) << std::endl;
            speed_res_f << "speed_offset_pronk: " << speed_param_.speed_offset_pronk.format( CommaInitFmt ) << std::endl;
            speed_res_f << "speed_offset_bound: " << speed_param_.speed_offset_bound.format( CommaInitFmt ) << std::endl;
            speed_res_f << "speed_offset_walk: " << speed_param_.speed_offset_walk.format( CommaInitFmt ) << std::endl;
            speed_res_f << std::endl;
            speed_res_f << "se_ori_cali_offset: " << speed_param_.se_ori_cali_offset.format( CommaInitFmt ) << std::endl;
            speed_res_f << "se_ori_cali_gain: " << speed_param_.se_ori_cali_gain.format( CommaInitFmt ) << std::endl;
            speed_res_f << std::endl;
            speed_res_f << "speed_offset_gait1: " << speed_param_.speed_offset_gait1.format( CommaInitFmt ) << std::endl;
            speed_res_f << "speed_offset_gait2: " << speed_param_.speed_offset_gait2.format( CommaInitFmt ) << std::endl;
            speed_res_f << "speed_offset_gait3: " << speed_param_.speed_offset_gait3.format( CommaInitFmt ) << std::endl;
            speed_res_f << "speed_offset_gait4: " << speed_param_.speed_offset_gait4.format( CommaInitFmt ) << std::endl;

            speed_res_f.close();
            system( "sync" );
            while ( cmd_interface_.GetSpeedOffsetTrigger() > 0 )
                usleep( 10000 );
        }
        usleep( 10 * 1000 );
    }
}

/**
 * @brief Run SPI For IMU.
 *
 */
#ifdef USE_EXTERNAL_IMU
void Cyberdog2HardwareBridge::RunSpiForImu() {
    // set timer for loop
#ifdef linux
    auto timerFd = timerfd_create( CLOCK_MONOTONIC, 0 );
#endif
    int seconds     = ( int )1.0 * robot_params_.controller_dt;
    int nanoseconds = ( int )( 1e9 * std::fmod( 1.0 * robot_params_.controller_dt, 1.f ) );

    Timer t;

#ifdef linux
    itimerspec timerSpec;
    timerSpec.it_interval.tv_sec  = seconds;
    timerSpec.it_value.tv_sec     = seconds;
    timerSpec.it_value.tv_nsec    = nanoseconds;
    timerSpec.it_interval.tv_nsec = nanoseconds;

    timerfd_settime( timerFd, 0, &timerSpec, nullptr );
#endif

    printf( "[PeriodicTask] Start Reading IMU Data By SPI (%d s, %d ns)\n", seconds, nanoseconds );

    float _lastPeriodTime = 0;
    float _lastRuntime    = 0;
    ( void )_lastPeriodTime;
    ( void )_lastRuntime;

    // imu_cost_log_f_.open("/tmp/log/time_log_imu.txt");
    // if (!imu_cost_log_f_.is_open()) {
    // std::cout << "Failed to open period log file time_log.txt, exit"
    // << std::endl;
    // exit(-1);
    // }
    while ( true ) {
        _lastPeriodTime = ( float )t.GetElapsedSeconds();
        t.StartTimer();

        static ImuProtocolHeader _imu_norminal_spi_cmd;
        static ImuNorminalData   _imu_norminal_data;

        _imu_norminal_spi_cmd.frame_header_first  = 0xAA;
        _imu_norminal_spi_cmd.frame_header_second = 0x55;
        _imu_norminal_spi_cmd.seq                 = 0x00;
        _imu_norminal_spi_cmd.board_id            = 0x02;
        _imu_norminal_spi_cmd.system_id           = 0x02;
        _imu_norminal_spi_cmd.sensor_id           = 0x04;
        _imu_norminal_spi_cmd.frame_type          = 0x00;
        _imu_norminal_spi_cmd.command             = 0x02;
        _imu_norminal_spi_cmd.payload_length      = 0x34;

        spi_handler_for_imu_.SetSpiCommandForImu( _imu_norminal_spi_cmd );
        spi_handler_for_imu_.DriverRun();
        spi_handler_for_imu_.GetImuDataBySpi( _imu_norminal_data );
        // update for imu data lcmt
        memcpy( external_imu_data_.omega, _imu_norminal_data.gyro, sizeof( float[ 3 ] ) );
        memcpy( external_imu_data_.acc, _imu_norminal_data.acc, sizeof( float[ 3 ] ) );
        if ( robot_params_.complementaryfilter_source > 0.5 ) {
            memcpy( external_imu_data_.quat, _imu_norminal_data.quaternions, sizeof( float[ 4 ] ) );
            Vec3< float > rpy_tmp = QuatToRPY( Vec4< float >( external_imu_data_.quat[ 0 ], external_imu_data_.quat[ 1 ], external_imu_data_.quat[ 2 ], external_imu_data_.quat[ 3 ] ) );
            for ( int i = 0; i < 3; i++ )
                external_imu_data_.rpy[ i ] = rpy_tmp[ i ];
            external_imu_data_.temp = 0;
            // std::cerr << "external_imu_data_.rpy: " << external_imu_data_.rpy[ 0 ] << " " << external_imu_data_.rpy[ 1 ] << " " << external_imu_data_.rpy[ 2 ] << std::endl;
            {
                std::lock_guard< std::mutex > guard( imu_mtx_ );
                external_vector_nav_data_.accelerometer = Vec3< float >( external_imu_data_.acc[ 0 ], external_imu_data_.acc[ 1 ], external_imu_data_.acc[ 2 ] );
                external_vector_nav_data_.gyro          = Vec3< float >( external_imu_data_.omega[ 0 ], external_imu_data_.omega[ 1 ], external_imu_data_.omega[ 2 ] );
                external_vector_nav_data_.quat          = Quat< float >( external_imu_data_.quat[ 1 ], external_imu_data_.quat[ 2 ], external_imu_data_.quat[ 3 ], external_imu_data_.quat[ 0 ] );
            }
        }
        // if ( robot_params_.lcm_debug_switch == 1 )
        if ( true ) {
            using Clock                  = std::chrono::high_resolution_clock;
            external_imu_data_.timestamp = Clock::now().time_since_epoch().count();
            external_imu_lcm_.publish( "external_imu", &external_imu_data_ );
        }
        _lastRuntime = ( float )t.GetElapsedSeconds();
#ifdef linux
        unsigned long long missed = 0;
        int                m      = read( timerFd, &missed, sizeof( missed ) );
        ( void )m;
        // log timeCost
        // if ( imu_cost_log_f_ ) {
        //     imu_cost_log_f_ << _lastPeriodTime << " " << _lastRuntime << std::endl;
        // }
        if ( spi_handler_for_imu_.GetSpiSuccessIterations() > 10 )
            first_imu_ = false;
#endif
    }
}
#endif

/**
 * @brief Run Cyberdog2 SPI.
 *
 */
void Cyberdog2HardwareBridge::RunSpi() {
    // if (robot_runner_->robot_ctrl_->GetMotorErrorFlag())
    if ( controller_->GetMotorErrorFlag() ) {
        spi_handler_.ResetSpi();
    }
    spi_handler_.SetSpiCommand( spi_command_ );
    spi_handler_.DriverRun();
    spi_handler_.GetSpiData( spi_data_ );
    if ( robot_params_.lcm_debug_switch == 1 )
        spi_handler_.PublishSpi( spi_lcm_ );
}

/**
 * @brief IMU serial port read thread.
 *
 */
void Cyberdog2HardwareBridge::RunImuSerial() {
    Serial serial_port;
    int    fd = serial_port.OpenPort( IMU_SERIAL_COM );
    if ( fd < 0 ) {
        fprintf( stderr, "[Serial] Open Serial <%s> failed!\n", IMU_SERIAL_COM );
        exit( -1 );
    }

    serial_port.SetPara( 6 );
    printf( "Open IMU serial succeed!\n" );

    // get parameters
    imu_filter_.SetDoBiasEstimation( robot_params_.imu_bias_estimation );
    if ( robot_params_.imu_bias_estimation ) {
        imu_filter_.SetBiasAlpha( 0.01 );
    }
    imu_filter_.SetDoAdaptiveGain( robot_params_.imu_adaptive_gain );
    imu_filter_.SetDoFreeAccPred( true );
    imu_filter_.SetGainAcc( robot_params_.gain_acc );

    Vec3< float > a_bias;
    Vec3< float > g_bias;
    Mat3< float > Ta_1;
    Mat3< float > Tg_1;
    if ( is_imu_params_in_yaml_ ) {
        int64_t             id_low = imu_param_.id_low, id_high = imu_param_.id_high;
        int64_t             id_low_curr, id_high_curr;
        std::vector< char > imu_id_curr( GetMcuInfo( serial_port, IMU_NUM, MCU_ID_NUMBER_REQ ) );
        if ( !Id2Int( imu_id_curr, id_high_curr, id_low_curr ) ) {
            std::cerr << "[IMU Serial] IMU MCU ID convert fail, exit program!" << std::endl;
            exit( -1 );
        }
        if ( !( id_low_curr == id_low && id_high_curr == id_high ) ) {
            std::cerr << std::hex;
            std::cerr << "[IMU Serial] IMU MCU ID mismatch, expect(low, hight): (0x" << id_low << ", 0x" << id_high << "), got: (0x" << id_low_curr << ", 0x" << id_high_curr << "), exit program!"
                      << std::endl;
            std::cerr << std::dec;
            exit( -1 );
        }
        else {
            std::cout << "[IMU Serial] IMU MCU ID matched, start sample IMU data" << std::endl;
        }

        a_bias << imu_param_.a_bias.cast< float >();
        g_bias << imu_param_.w_bias.cast< float >();
        Ta_1 << imu_param_.Ta.inverse().cast< float >();
        Tg_1 << imu_param_.Tg.inverse().cast< float >();
    }
    else {
        if ( !GetImuParamsFromMcu( a_bias, g_bias, Ta_1, Tg_1, serial_port ) ) {
            std::cout << "Failed to get IMU params from MCU" << std::endl;
            exit( -1 );
        }
        std::cout << "Ta: " << Ta_1 << std::endl;
        std::cout << "Tg: " << Tg_1 << std::endl;
        std::cout << "a_bias: " << a_bias << std::endl;
        std::cout << "g_bias: " << g_bias << std::endl;
    }

    Vec3< float > init_acc = pos_param_.init_acc.cast< float >();
    init_acc               = Ta_1 * ( init_acc - a_bias );

    float deltPitch = -std::atan2( init_acc( 0 ), init_acc( 2 ) );
    float deltRoll  = std::atan2( init_acc( 1 ), init_acc( 2 ) );

    int iter_t = 0;

    Timer t;
    t.StartTimer();
    RawImuData imu_data;
    ImuResult  imu_res;

    while ( true ) {
        static int iCountNumber = 0;

        // read data in 1000Hz
        memset( &imu_data, 0, sizeof( RawImuData ) );
        memset( &imu_res, 0, sizeof( ImuResult ) );
        GetImuData( imu_data, serial_port, imu_res );

        iter_t++;
        if ( iter_t < 1000 ) {
            continue;
        }  // discard the first 100 data

        Vec3< float > acc_vec( imu_res.acc[ 0 ], imu_res.acc[ 1 ], imu_res.acc[ 2 ] );
        Vec3< float > gyro_vec( imu_res.gyro[ 0 ], imu_res.gyro[ 1 ], imu_res.gyro[ 2 ] );

        acc_vec  = Ta_1 * ( acc_vec - a_bias ) * GRAVITY;
        gyro_vec = Tg_1 * ( gyro_vec - g_bias );

        // Transform from IMU coordinate to cyberdog coordinate
        Mat3< float > R_IMU = ori::RpyToRotMat( Vec3< float >( 0. + robot_params_.delt_roll + deltRoll, 0. + robot_params_.delt_pitch + deltPitch, -M_PI ) );
        acc_vec             = R_IMU * acc_vec;   // Vec3<float>(-acc_vec(0), -acc_vec(1), acc_vec(2));
        gyro_vec            = R_IMU * gyro_vec;  // Vec3<float>(-gyro_vec(0), -gyro_vec(1), gyro_vec(2));

        // update gyro_vec after online calibration
        gyro_vec -= imu_param_.inrun_w_bias.cast< float >();

        if ( DEBUG_PRINTF ) {
            if ( iCountNumber++ % 10000 == 0 ) {
                std::cout << "the value of imu_param_.inrun_w_bias.cast<float>() is: " << imu_param_.inrun_w_bias.cast< float >() << std::endl;
                iCountNumber = 0;
            }
        }

        for ( int i = 0; i < 3; i++ ) {
            my_imu_data_.acc[ i ]   = acc_vec[ i ];
            my_imu_data_.omega[ i ] = gyro_vec[ i ];
        }
        my_imu_data_.temp = imu_res.temperature;

        FilterImu( acc_vec, gyro_vec );

        auto&         q = vector_nav_data_.quat;
        Quat< float > q_right( q[ 3 ], q[ 0 ], q[ 1 ], q[ 2 ] );
        Vec3< float > rpy = ori::QuatToRPY( q_right );
        for ( u32 i = 0; i < 3; i++ )
            my_imu_data_.rpy[ i ] = rpy[ i ];

        Eigen::Map< Quat< float > >( my_imu_data_.quat ) = vector_nav_data_.quat;
        imu_lcm_.publish( "myIMU", &my_imu_data_ );

        first_imu_ = false;  // we have got the first IMU data
        usleep( 500 );
    }
}

#ifdef USE_EXTERNAL_IMU

#ifdef USE_NEW_BOARD

uint8_t receive_message[ sizeof( Bmi088MsgCmdData ) ] = { 0 };

/**
 * @brief Runs the external IMU serial communication.
 *
 */
void Cyberdog2HardwareBridge::RunExternalImuSerial() {
    Serial   serial_port;
    long int iter     = 0;
    int      freq_div = 5;
    int      fd       = serial_port.OpenPort( EXTERNAL_IMU_SERIAL_COM );
    if ( fd < 0 ) {
        fprintf( stderr, "[Serial] Open Serial <%s> failed!\n", EXTERNAL_IMU_SERIAL_COM );
        exit( -1 );
    }

    serial_port.SetPara( 6 );
    fprintf( stdout, "[IMU Serial] Open External_IMU serial succeed!\n" );

    using Clock = std::chrono::high_resolution_clock;
    intmax_t timestamp;
    Timer    t;
    t.StartTimer();
    uint8_t  msg[ 12 ] = { 0xaa, 0x55, 0x00, 0x02, 0x02, 0x04, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00 };
    uint16_t sum       = 0;
    for ( uint8_t i = 0; i < 10; i++ )
        sum += msg[ i ];
    msg[ 10 ] = sum & 0x00FF;
    msg[ 11 ] = sum >> 8;
    tcflush( serial_port.fd(), TCIOFLUSH );
    serial_port.WriteData( ( const char* )msg, 12 );
    while ( true ) {
        usleep( 1000 );
        if ( GetExternalImuData( receive_message, serial_port, true ) ) {
            first_imu_ = false;  // we have got the first IMU data
            std::cout << "[IMU Serial] Receive First Right Data! " << std::endl;
            break;
        }
        else {
            std::cerr << "[IMU Serial] Receive Wrong Data!" << std::endl;
        }
    }
    tcflush( serial_port.fd(), TCIOFLUSH );
    // save time cost as file
    // std::ofstream imu_log_f;
    // imu_log_f.open("/tmp/log/time_log_imu.txt");
    // if (!imu_log_f.is_open()) {
    // std::cout << "Failed to open period log file time_log.txt, exit"
    // << std::endl;
    // exit(-1);
    // }
    while ( true ) {
        // static Timer t;
        // static float timeCost;
        // timeCost = t.GetElapsedSeconds();
        // t.StartTimer();
        // read out invalid data
        if ( !GetExternalImuData( receive_message, serial_port, true ) ) {
            std::cerr << "[IMU Serial]  Wrong Data" << std::endl;
            continue;
        }
        Bmi088MsgCmdData msg;
        memcpy( &msg, receive_message, sizeof( Bmi088MsgCmdData ) );
        memcpy( external_imu_data_.omega, msg.gyro, sizeof( float[ 3 ] ) );
        memcpy( external_imu_data_.acc, msg.acc, sizeof( float[ 3 ] ) );
        if ( robot_params_.complementaryfilter_source > 0.5 ) {
            memcpy( external_imu_data_.quat, msg.quaternions, sizeof( float[ 4 ] ) );
            // external_imu_data_.rpy[2] = msg->yaw;
            Vec3< float > rpy_tmp = QuatToRPY( Vec4< float >( external_imu_data_.quat[ 0 ], external_imu_data_.quat[ 1 ], external_imu_data_.quat[ 2 ], external_imu_data_.quat[ 3 ] ) );
            for ( int i = 0; i < 3; i++ )
                external_imu_data_.rpy[ i ] = rpy_tmp[ i ];
            external_imu_data_.temp = 0;
            // std::cerr << "external_imu_data_.rpy: " << external_imu_data_.rpy[ 0 ] << " " << external_imu_data_.rpy[ 1 ] << " " << external_imu_data_.rpy[ 2 ] << std::endl;
            {
                std::lock_guard< std::mutex > guard( imu_mtx_ );
                external_vector_nav_data_.accelerometer = Vec3< float >( external_imu_data_.acc[ 0 ], external_imu_data_.acc[ 1 ], external_imu_data_.acc[ 2 ] );
                external_vector_nav_data_.gyro          = Vec3< float >( external_imu_data_.omega[ 0 ], external_imu_data_.omega[ 1 ], external_imu_data_.omega[ 2 ] );
                external_vector_nav_data_.quat          = Quat< float >( external_imu_data_.quat[ 1 ], external_imu_data_.quat[ 2 ], external_imu_data_.quat[ 3 ], external_imu_data_.quat[ 0 ] );
            }
        }
        else {
            // get parameters
            imu_filter_.SetDoBiasEstimation( robot_params_.imu_bias_estimation );
            if ( robot_params_.imu_bias_estimation ) {
                imu_filter_.SetBiasAlpha( 0.01 );
            }
            imu_filter_.SetDoAdaptiveGain( robot_params_.imu_adaptive_gain );
            imu_filter_.SetDoFreeAccPred( true );
            imu_filter_.SetGainAcc( robot_params_.gain_acc );
            Vec3< float > acc_tmp  = Vec3< float >( external_imu_data_.acc[ 0 ], external_imu_data_.acc[ 1 ], external_imu_data_.acc[ 2 ] );
            Vec3< float > gyro_tmp = Vec3< float >( external_imu_data_.omega[ 0 ], external_imu_data_.omega[ 1 ], external_imu_data_.omega[ 2 ] );
            FilterImu( acc_tmp, gyro_tmp );
            external_imu_data_.quat[ 0 ] = external_vector_nav_data_.quat[ 3 ];
            external_imu_data_.quat[ 1 ] = external_vector_nav_data_.quat[ 0 ];
            external_imu_data_.quat[ 2 ] = external_vector_nav_data_.quat[ 1 ];
            external_imu_data_.quat[ 3 ] = external_vector_nav_data_.quat[ 2 ];
            Vec3< float > rpy_tmp        = QuatToRPY( Vec4< float >( external_imu_data_.quat[ 0 ], external_imu_data_.quat[ 1 ], external_imu_data_.quat[ 2 ], external_imu_data_.quat[ 3 ] ) );
            for ( int i = 0; i < 3; i++ )
                external_imu_data_.rpy[ i ] = rpy_tmp[ i ];
            rpy_tmp = QuatToRPY( Vec4< float >( msg.quaternions[ 0 ], msg.quaternions[ 1 ], msg.quaternions[ 2 ], msg.quaternions[ 3 ] ) );
            for ( int i = 0; i < 3; i++ )
                external_imu_data_.rpy_imu[ i ] = rpy_tmp[ i ];
            external_imu_data_.temp = 0;
        }
        iter++;
        if ( robot_params_.lcm_debug_switch == 1 )
            freq_div = 1;  // 1000HZ
        else
            freq_div = 5;  // 200HZ
        if ( iter % freq_div == 0 ) {
            timestamp                    = Clock::now().time_since_epoch().count();
            external_imu_data_.timestamp = timestamp;
            external_imu_lcm_.publish( "external_imu", &external_imu_data_ );
        }
        // std::cerr << "external_imu_data_.rpy: " << external_imu_data_.rpy[ 0 ] << " " << external_imu_data_.rpy[ 1 ] << " " << external_imu_data_.rpy[ 2 ] << std::endl;
        usleep( 250 );
        // if ( imu_log_f )
        //     imu_log_f << timeCost<<std::endl;
    }
}

#else

// Short integer byte swapping
#define Swap16( A ) ( ( ( ( uint16_t )( A )&0xff00 ) >> 8 ) | ( ( ( uint16_t )( A )&0x00ff ) << 8 ) )

// Long integer byte swapping
#define Swap32( A ) ( ( ( ( uint32_t )( A )&0xff000000 ) >> 24 ) | ( ( ( uint32_t )( A )&0x00ff0000 ) >> 8 ) | ( ( ( uint32_t )( A )&0x0000ff00 ) << 8 ) | ( ( ( uint32_t )( A )&0x000000ff ) << 24 ) )

#pragma pack( 1 )
typedef struct {
    uint8_t pkg_id1;
    uint8_t pkg_id2;
    uint8_t pkg_len;
} pkg_head_def;

typedef struct {
    pkg_head_def quat_head;
    float        data[ 4 ];
} quat_msg;

typedef struct {
    pkg_head_def acc_gyro_head;
    float        data[ 3 ];
} acc_gyro_msg;

typedef struct {
    uint8_t      Preamble;
    uint8_t      BID;
    uint8_t      MID;
    uint8_t      Len;
    quat_msg     quat;
    acc_gyro_msg acc;
    acc_gyro_msg gyro;
    acc_gyro_msg euler;
    uint8_t      SC;
} xsens_message_def;
#pragma pack( 0 )

#define XSENS_BUF_LEN ( sizeof( xsens_message_def ) )
static uint8_t head_board_uart_buffer[ XSENS_BUF_LEN ] = { 0 };

/**
 * @brief Checks the integrity of an Xsens message
 *
 * @param message The Xsens message to be checked
 * @return true If the message is valid
 * @return false If the message is invalid
 */
bool XsensCheck( xsens_message_def* message ) {
    bool res = false;

    uint8_t* temp = ( uint8_t* )message;
    uint8_t  SC   = 0;
    uint8_t  len  = message->Len + 5;
    temp += 1;

    for ( uint8_t i = 1; i < len; i++ ) {
        SC += ( *temp++ );
    }

    if ( SC == 0 ) {
        res = true;
    }

    return res;
}

/**
 * @brief Converts the byte order of Xsens message data from network to host byte order.
 *
 * @param message Pointer to the Xsens message structure
 */
void XsensNtohl( xsens_message_def* message ) {
    uint32_t temp = 0;

    for ( uint8_t i = 0; i < 4; i++ ) {
        uint32_t toConvert      = *( ( uint32_t* )( &( message->quat.data[ i ] ) ) );
        temp                    = Swap32( toConvert );
        message->quat.data[ i ] = *( ( float* )&temp );
    }

    for ( uint8_t i = 0; i < 3; i++ ) {
        uint32_t toConvert     = *( ( uint32_t* )( &( message->acc.data[ i ] ) ) );
        temp                   = Swap32( toConvert );
        message->acc.data[ i ] = *( ( float* )&temp );

        toConvert               = *( ( uint32_t* )( &( message->gyro.data[ i ] ) ) );
        temp                    = Swap32( toConvert );
        message->gyro.data[ i ] = *( ( float* )&temp );

        toConvert                = *( ( uint32_t* )( &( message->euler.data[ i ] ) ) );
        temp                     = Swap32( toConvert );
        message->euler.data[ i ] = *( ( float* )&temp );
    }
}

/**
 * @brief Runs the external IMU serial communication.
 *
 */
void Cyberdog2HardwareBridge::RunExternalImuSerial() {
    Serial serial_port;
    int    fd = serial_port.OpenPort( EXTERNAL_IMU_SERIAL_COM );
    if ( fd < 0 ) {
        fprintf( stderr, "[Serial] Open Serial <%s> failed!\n", EXTERNAL_IMU_SERIAL_COM );
        exit( -1 );
    }

    serial_port.SetPara( 6 );
    printf( "Open External_IMU serial succeed!\n" );

    Timer t;
    t.StartTimer();

    while ( true ) {
        // std::cout << "==========================================" << std::endl;
        serial_port.ReadData( &( head_board_uart_buffer[ 0 ] ), sizeof( unsigned char ), true );
        // std::cout << "!!! head_board_uart_buffer[0] = " << std::hex << (unsigned int)head_board_uart_buffer[0] << std::endl;
        if ( head_board_uart_buffer[ 0 ] == 0xFA ) {
            serial_port.ReadData( &( head_board_uart_buffer[ 1 ] ), sizeof( unsigned char ), true );
            // std::cout << "!!! head_board_uart_buffer[1] = " << std::hex << (unsigned int)head_board_uart_buffer[1] << std::endl;
            if ( head_board_uart_buffer[ 1 ] == 0xFF ) {
                serial_port.ReadData( &( head_board_uart_buffer[ 2 ] ), sizeof( unsigned char ), true );
                // std::cout << "!!! head_board_uart_buffer[2] = " << std::hex << (unsigned int)head_board_uart_buffer[2] << std::endl;
                if ( head_board_uart_buffer[ 2 ] == 0x36 ) {
                    serial_port.ReadData( &( head_board_uart_buffer[ 3 ] ), sizeof( xsens_message_def ) - 3, true );
                    xsens_message_def* message = ( xsens_message_def* )head_board_uart_buffer;
                    {
                        if ( XsensCheck( message ) ) {
                            XsensNtohl( message );
                            memcpy( external_imu_data_.quat, message->quat.data, sizeof( float[ 4 ] ) );
                            memcpy( external_imu_data_.omega, message->gyro.data, sizeof( float[ 3 ] ) );
                            memcpy( external_imu_data_.acc, message->acc.data, sizeof( float[ 3 ] ) );
                            memcpy( external_imu_data_.rpy, message->euler.data, sizeof( float[ 3 ] ) );
                            external_imu_data_.temp                 = 0;
                            external_vector_nav_data_.accelerometer = Vec3< float >( external_imu_data_.acc[ 0 ], external_imu_data_.acc[ 1 ], external_imu_data_.acc[ 2 ] );
                            external_vector_nav_data_.gyro          = Vec3< float >( external_imu_data_.omega[ 0 ], external_imu_data_.omega[ 1 ], external_imu_data_.omega[ 2 ] );
                            external_vector_nav_data_.quat = Quat< float >( external_imu_data_.quat[ 1 ], external_imu_data_.quat[ 2 ], external_imu_data_.quat[ 3 ], external_imu_data_.quat[ 0 ] );
                            external_imu_lcm_.publish( "external_imu", &external_imu_data_ );
                        }
                    }
                }
            }
        }

        memset( head_board_uart_buffer, 0, XSENS_BUF_LEN );
        usleep( 250 );
    }
}
#endif
#endif

/**
 * @brief Filters the IMU data.
 *
 * @param acc_correct The corrected accelerometer data
 * @param gyro_correct The corrected gyroscope data
 */
void Cyberdog2HardwareBridge::FilterImu( Vec3< float > acc_correct, Vec3< float > gyro_correct ) {
    imu_filter_.SetDoBiasEstimation( robot_params_.imu_bias_estimation );
    imu_filter_.SetGainAcc( robot_params_.gain_acc );

    float dt = 0.001;
    // fuse IMU data
    double q[ 4 ];
    if ( robot_params_.filter_type == 0 ) {
        imu_filter_.Update( acc_correct[ 0 ], acc_correct[ 1 ], acc_correct[ 2 ], gyro_correct[ 0 ], gyro_correct[ 1 ], gyro_correct[ 2 ], dt );

        imu_filter_.GetOrientation( q[ 0 ], q[ 1 ], q[ 2 ], q[ 3 ] );  // w x y z
    }
    else {
        imu_mh_filter_.Update( acc_correct[ 0 ], acc_correct[ 1 ], acc_correct[ 2 ], gyro_correct[ 0 ], gyro_correct[ 1 ], gyro_correct[ 2 ], dt );

        imu_mh_filter_.GetOrientation( q[ 0 ], q[ 1 ], q[ 2 ], q[ 3 ] );
    }

    Quat< float > cur_quat = Quat< float >( q[ 0 ], q[ 1 ], q[ 2 ], q[ 3 ] );  // w x y z
    Vec3< float > rpy_cur  = ori::QuatToRPY( cur_quat );

    Quat< float > quat_filter = ori::RpyToQuat( rpy_cur );
    for ( int i = 0; i < 4; i++ ) {
        q[ i ] = quat_filter[ i ];
    }

    {
        std::lock_guard< std::mutex > guard( imu_mtx_ );
#ifndef USE_NEW_BOARD
        vector_nav_data_.accelerometer = Vec3< float >( acc_correct[ 0 ], acc_correct[ 1 ], acc_correct[ 2 ] );
        vector_nav_data_.gyro          = Vec3< float >( gyro_correct[ 0 ], gyro_correct[ 1 ], gyro_correct[ 2 ] );
        vector_nav_data_.quat          = Quat< float >( q[ 1 ], q[ 2 ], q[ 3 ], q[ 0 ] );
#else
        external_vector_nav_data_.accelerometer = Vec3< float >( acc_correct[ 0 ], acc_correct[ 1 ], acc_correct[ 2 ] );
        external_vector_nav_data_.gyro          = Vec3< float >( gyro_correct[ 0 ], gyro_correct[ 1 ], gyro_correct[ 2 ] );
        external_vector_nav_data_.quat          = Quat< float >( q[ 1 ], q[ 2 ], q[ 3 ], q[ 0 ] );
#endif
    }
}

/**
 * @brief Send LCM visualization data.
 *
 */
void HardwareBridge::PublishVisualizationLcm() {
    visualization_lcmt visualization_data;
    for ( int i = 0; i < 3; i++ ) {
        visualization_data.x[ i ] = cyberdog2_visualization_.p[ i ];
    }

    for ( int i = 0; i < 4; i++ ) {
        visualization_data.quat[ i ] = cyberdog2_visualization_.quat[ i ];
        visualization_data.rgba[ i ] = cyberdog2_visualization_.color[ i ];
    }

    for ( int i = 0; i < 12; i++ ) {
        visualization_data.q[ i ] = cyberdog2_visualization_.q[ i ];
    }

    visualization_lcm_.publish( "main_visualization", &visualization_data );
}

/**
 * @brief Retrieves the parameter response from LCM.
 *
 * @param ptr_msg Pointer to the control_parameter_request_lcmt message
 * @param control_para ControlParameter object to store the parameter
 * @param is_set_flag Flag indicating whether it's a set or get operation
 * @return true if the operation is successful, false otherwise
 */
bool HardwareBridge::GetParaResponseLcm( const control_parameter_request_lcmt* ptr_msg, ControlParameter& control_para, bool is_set_flag ) {
    std::string name( ( char* )ptr_msg->name );

    // type check
    if ( ( s8 )control_para.kind_ != ptr_msg->parameterKind ) {
        throw std::runtime_error( "type mismatch for parameter " + name + ", robot thinks it is " + ControlParameterValueKindToString( control_para.kind_ ) + " but received a command to set it to "
                                  + ControlParameterValueKindToString( ( ControlParameterValueKind )ptr_msg->parameterKind ) );
    }

    // do the actual set
    ControlParameterValue v;

    if ( is_set_flag ) {
        memcpy( &v, ptr_msg->value, sizeof( v ) );
        control_para.Set( v, ( ControlParameterValueKind )ptr_msg->parameterKind );
        std::cout << "Receive kSET_SPEED_CALIBRATE_PARAM_BY_NAME msg ,and the value will be set!" << std::endl;
    }
    else {
        v = control_para.Get( ( ControlParameterValueKind )ptr_msg->parameterKind );
        std::cout << "Receive kGET_SPEED_CALIBRATE_PARAM_BY_NAME msg ,and the value will be read from speed_param_!" << std::endl;
    }

    // respond:
    parameter_response_lcmt_.requestNumber = ptr_msg->requestNumber;  // acknowledge that the set has happened
    parameter_response_lcmt_.parameterKind = ptr_msg->parameterKind;  // just for debugging print statements

    if ( is_set_flag ) {
        memcpy( parameter_response_lcmt_.value, ptr_msg->value, 64 );
        printf( "[HardwareBridge] Control Parameter set %s to %s\n", name.c_str(), ControlParameterValueToString( v, ( ControlParameterValueKind )ptr_msg->parameterKind ).c_str() );
    }
    else {
        memcpy( parameter_response_lcmt_.value, &v, sizeof( v ) );
        printf( "[HardwareBridge] Control Parameter get %s is : %s\n", name.c_str(), ControlParameterValueToString( v, ( ControlParameterValueKind )ptr_msg->parameterKind ).c_str() );
    }

    // parameter_response_lcmt_.value = _parameter_request_lcmt.value; // just
    // for debugging print statements
    strcpy( ( char* )parameter_response_lcmt_.name, name.c_str() );  // just for debugging print statements
    parameter_response_lcmt_.requestKind = ptr_msg->requestKind;

    return true;
}

/**
 * @brief Retrieves byte parameters from the IMU via serial communication.
 *
 * @param serial_port The serial port object used for communication
 * @param rec_data The buffer to store the received data
 * @param start_pos The starting position of the parameters to retrieve
 * @param size The size of the parameters to retrieve
 * @return true if the parameters are successfully retrieved, false otherwise
 */
bool Cyberdog2HardwareBridge::GetImuByteParams( Serial& serial_port, unsigned char* rec_data, unsigned char start_pos, unsigned char size ) {
    // discard some data to flush buffer
    usleep( 10000 );
    tcflush( serial_port.fd(), TCIOFLUSH );

    // ask mcu send params back
    unsigned char sendData[ 8 ] = { 0xAA, 0x55, 0x03, 0x0A, start_pos, ( unsigned char )( size - 2 ), 0x00, 0x00 };
    int16_t       checksum      = 0;
    for ( int i = 2; i < 6; i++ ) {
        checksum += sendData[ i ];
    }

    sendData[ 6 ] = checksum & 0XFF;
    sendData[ 7 ] = ( checksum >> 8 ) & 0XFF;

    serial_port.WriteData( ( char* )sendData, sizeof( sendData ) );

    unsigned char head1;
    unsigned char head2;
    unsigned char len;
    unsigned char id;
    int           err_num = 0;
    int           iter    = 0;

    while ( true ) {  // wait for the data head
        if ( err_num > 5 && err_num % 5 == 0 ) {
            std::cerr << "[IMU_TEST Serial] too many err char when waiting for header" << std::endl;
        }
        serial_port.ReadData( &head1, sizeof( unsigned char ), true );
        if ( head1 == 0x5A ) {
            serial_port.ReadData( &head2, sizeof( unsigned char ), true );
            if ( head2 == 0xA5 ) {
                serial_port.ReadData( &len, sizeof( unsigned char ), true );
                serial_port.ReadData( &id, sizeof( unsigned char ), true );
                if ( id == 0x0A && len == size - 1 ) {
                    serial_port.ReadData( rec_data, size, true );
                    checksum = 0;
                    checksum = len + id;
                    for ( int i = 0; i < len - 1; i++ ) {
                        checksum += rec_data[ i ];
                    }
                    if ( ( ( checksum & 0XFF ) == rec_data[ len - 1 ] ) && ( ( ( checksum >> 8 ) & 0XFF ) == rec_data[ len ] ) ) {
                        break;
                    }
                    else {
                        if ( iter > 100 ) {
                            return false;
                        }
                        usleep( 10000 );
                        tcflush( serial_port.fd(), TCIOFLUSH );
                        printf( "request imu params again\n" );
                        serial_port.WriteData( ( char* )sendData, sizeof( sendData ) );
                        iter++;
                    }
                }
            }
            else {
                err_num += 1;
            }
        }
        else {
            err_num += 1;
        }
        usleep( 1 );
    }
    return true;
}

/**
 * @brief Retrieves IMU parameters from MCU.
 *
 * @param a_bias The accelerometer bias vector
 * @param g_bias The gyroscope bias vector
 * @param Ta_1 The accelerometer calibration matrix
 * @param Tg_1 The gyroscope calibration matrix
 * @param serial_port The serial port object for communication
 * @return true if successful, false otherwise
 */
bool Cyberdog2HardwareBridge::GetImuParamsFromMcu( Vec3< float >& a_bias, Vec3< float >& g_bias, Mat3< float >& Ta_1, Mat3< float >& Tg_1, Serial& serial_port ) {
    unsigned char resrecData[ 3 ];
    if ( !GetImuByteParams( serial_port, resrecData, 216, sizeof( resrecData ) ) ) {
        return false;
    }
    else {
        if ( resrecData[ 0 ] == 0 ) {
            return false;
        }
    }

    unsigned char accrecData[ 98 ];
    if ( !GetImuByteParams( serial_port, accrecData, 0, sizeof( accrecData ) ) ) {
        return false;
    }

    Byte2Double data;

    for ( int i = 0; i < 72; ) {
        for ( int j = 0; j < 9; ++j ) {
            for ( uint k = 0; k < sizeof( double ); ++k, ++i ) {
                data.bytes[ k ] = accrecData[ i ];
            }
            Ta_1( j / 3, j % 3 ) = ( float )data.value;
        }
    }
    std::cout << "Get Ta Params from MCU" << std::endl;
    for ( int i = 72; i < 96; ) {
        for ( int j = 0; j < 3; ++j ) {
            for ( uint k = 0; k < sizeof( double ); ++k, ++i ) {
                data.bytes[ k ] = accrecData[ i ];
            }
            a_bias( j ) = ( float )data.value;
        }
    }
    std::cout << "Get a_bias Params from MCU" << std::endl;
    unsigned char gyrorecData[ 98 ];
    if ( !GetImuByteParams( serial_port, gyrorecData, 96, sizeof( gyrorecData ) ) ) {
        return false;
    }
    for ( int i = 0; i < 24; ) {
        for ( int j = 0; j < 3; ++j ) {
            for ( uint k = 0; k < sizeof( double ); ++k, ++i ) {
                data.bytes[ k ] = gyrorecData[ i ];
            }
            g_bias( j ) = ( float )data.value;
        }
    }
    std::cout << "Get g_bias Params from MCU" << std::endl;
    for ( int i = 24; i < 96; ) {
        for ( int j = 0; j < 9; ++j ) {
            for ( uint k = 0; k < sizeof( double ); ++k, ++i ) {
                data.bytes[ k ] = gyrorecData[ i ];
            }
            Tg_1( j / 3, j % 3 ) = ( float )data.value;
        }
    }
    std::cout << "Get Tg Params from MCU" << std::endl;
    return true;
}
