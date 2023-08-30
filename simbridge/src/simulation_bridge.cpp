/*! @file simulation_bridge.cpp
 *  @brief  The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the
 * HardwareBridge.
 */

#include <iostream>

#include "simulation_bridge.hpp"
#include "utilities/segfault_handler.hpp"

/**
 * @brief Connect to a simulation.
 *
 */
void SimulationBridge::Run() {
    // init shared memory:
    shared_memory_.Attach( DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME );
    shared_memory_.Init( false );

    InstallSegfaultHandler( shared_memory_().robotToSim.errorMessage );

    // init Quadruped Controller
    try {
        printf( "[Simulation Driver] Starting main loop...\n" );
        bool first_run = true;
        for ( ;; ) {
            // wait for our turn to access the shared memory
            // on the first loop, this gives the simulator a chance to put stuff in
            // shared memory before we start
            shared_memory_.WaitForSimulator();

            if ( first_run ) {
                first_run = false;
                // check that the robot type is correct:
                if ( robot_ != shared_memory_().simToRobot.robotType ) {
                    printf( "simulator and simulatorDriver don't agree on which robot we are "
                            "simulating (robot %d, sim %d)\n",
                            ( int )robot_, ( int )shared_memory_().simToRobot.robotType );
                    throw std::runtime_error( "robot mismatch!" );
                }
            }

            // the simulator tells us which mode to run in
            sim_mode_ = shared_memory_().simToRobot.mode;
            switch ( sim_mode_ ) {
            case SimulatorMode::kRunContorlParameters:  // there is a new control
                // parameter request
                HandleControlParameters();
                break;
            case SimulatorMode::kRunContorller:  // the simulator is ready for the
                // next robot controller run
                iterations_++;
                RunRobotControl();
                break;
            case SimulatorMode::kDoNothing:  // the simulator is just checking to see
                // if we are alive yet
                break;
            case SimulatorMode::kExit:  // the simulator is done with us
                printf( "[Simulation Driver] Transitioned to exit mode\n" );
                return;
                break;
            default:
                throw std::runtime_error( "unknown simulator mode" );
            }

            // tell the simulator we are done
            shared_memory_.RobotIsDone();
        }
    }
    catch ( std::exception& e ) {
        strncpy( shared_memory_().robotToSim.errorMessage, e.what(), sizeof( shared_memory_().robotToSim.errorMessage ) - 1 );
        shared_memory_().robotToSim.errorMessage[ sizeof( shared_memory_().robotToSim.errorMessage ) - 1 ] = '\0';
        std::cout << "the error is: " << e.what() << std::endl;
        throw e;
    }
}

/**
 * @brief This function handles a control parameter message from the simulator.
 *
 */
void SimulationBridge::HandleControlParameters() {
    ControlParameterRequest&  request  = shared_memory_().simToRobot.controlParameterRequest;
    ControlParameterResponse& response = shared_memory_().robotToSim.controlParameterResponse;
    if ( request.requestNumber <= response.requestNumber ) {
        // nothing to do!
        printf( "[SimulationBridge] Warning: the simulator has run a ControlParameter "
                "iteration, but there is no new request!\n" );
        return;
    }

    // sanity check
    u64 num_requests = request.requestNumber - response.requestNumber;
    assert( num_requests == 1 );

    response.nParameters = robot_params_->collection_.map_.size();  // todo don't do this every single time?

    switch ( request.requestKind ) {
    case ControlParameterRequestKind::kSET_ROBOT_PARAM_BY_NAME: {
        std::string       name( request.name );
        ControlParameter& param = robot_params_->collection_.LookUp( name );

        // type check
        if ( param.kind_ != request.parameterKind ) {
            throw std::runtime_error( "type mismatch for parameter " + name + ", robot thinks it is " + ControlParameterValueKindToString( param.kind_ ) + " but received a command to set it to "
                                      + ControlParameterValueKindToString( request.parameterKind ) );
        }

        // do the actual set
        param.Set( request.value, request.parameterKind );

        // respond:
        response.requestNumber = request.requestNumber;  // acknowledge that the set has happened
        response.parameterKind = request.parameterKind;  // just for debugging print statements
        response.value         = request.value;          // just for debugging print statements
        strcpy( response.name,
                name.c_str() );  // just for debugging print statements
        response.requestKind = request.requestKind;

        printf( "%s\n", response.ToString().c_str() );

    } break;

    case ControlParameterRequestKind::kSET_USER_PARAM_BY_NAME: {
        std::string name( request.name );
        if ( !user_params_ ) {
            printf( "[Simulation Bridge] Warning: tried to set user parameter, but the robot does not have any!\n" );
        }
        else {
            ControlParameter& param = user_params_->collection_.LookUp( name );

            // type check
            if ( param.kind_ != request.parameterKind ) {
                throw std::runtime_error( "type mismatch for parameter " + name + ", robot thinks it is " + ControlParameterValueKindToString( param.kind_ ) + " but received a command to set it to "
                                          + ControlParameterValueKindToString( request.parameterKind ) );
            }

            // do the actual set
            param.Set( request.value, request.parameterKind );
        }

        // respond:
        response.requestNumber = request.requestNumber;  // acknowledge that the set has happened
        response.parameterKind = request.parameterKind;  // just for debugging print statements
        response.value         = request.value;          // just for debugging print statements
        strcpy( response.name,
                name.c_str() );  // just for debugging print statements
        response.requestKind = request.requestKind;

        printf( "%s\n", response.ToString().c_str() );

    } break;

    case ControlParameterRequestKind::kGET_ROBOT_PARAM_BY_NAME: {
        std::string       name( request.name );
        ControlParameter& param = robot_params_->collection_.LookUp( name );

        // type check
        if ( param.kind_ != request.parameterKind ) {
            throw std::runtime_error( "type mismatch for parameter " + name + ", robot thinks it is " + ControlParameterValueKindToString( param.kind_ ) + " but received a command to set it to "
                                      + ControlParameterValueKindToString( request.parameterKind ) );
        }

        // respond
        response.value         = param.Get( request.parameterKind );
        response.requestNumber = request.requestNumber;  // acknowledge
        response.parameterKind = request.parameterKind;  // just for debugging print statements
        strcpy( response.name,
                name.c_str() );                      // just for debugging print statements
        response.requestKind = request.requestKind;  // just for debugging print statements

        printf( "%s\n", response.ToString().c_str() );
    } break;
    default:
        throw std::runtime_error( "unhandled get/set" );
    }
}

/**
 * @brief Run the robot controller.
 *
 */
void SimulationBridge::RunRobotControl() {
    if ( first_controller_run_ ) {
        printf( "[Simulator Driver] First run of robot controller...\n" );
        if ( robot_params_->IsFullyInitialized() ) {
            printf( "\tAll %ld control parameters are initialized\n", robot_params_->collection_.map_.size() );
        }
        else {
            printf( "\tbut not all control parameters were initialized. Missing:\n%s\n", robot_params_->GenerateUnitializedList().c_str() );
            throw std::runtime_error( "not all parameters initialized when going into kRunContorller" );
        }

        // auto* userControlParameters = robot_runner_->robot_ctrl_->GetUserControlParameters();
        auto* userControlParameters = robot_runner_->GetUserControlParameters();
        if ( userControlParameters ) {
            if ( userControlParameters->IsFullyInitialized() ) {
                printf( "\tAll %ld user parameters are initialized\n", userControlParameters->collection_.map_.size() );
                sim_mode_ = SimulatorMode::kRunContorller;
            }
            else {
                printf( "\tbut not all control parameters were initialized. Missing:\n%s\n", userControlParameters->GenerateUnitializedList().c_str() );
                throw std::runtime_error( "not all parameters initialized when going into kRunContorller" );
            }
        }
        else {
            sim_mode_ = SimulatorMode::kRunContorller;
        }

        interface_lcm_thread_cmd_          = std::thread( &SimulationBridge::HandleInterfaceLCM_cmd, this );
        interface_lcm_thread_cyberdog_cmd_ = std::thread( &SimulationBridge::HandleInterfaceLCM_cyberdog_cmd, this );
        interface_lcm_thread_motion_list_  = std::thread( &SimulationBridge::HandleInterfaceLCM_motion_list, this );
        interface_lcm_thread_usergait_     = std::thread( &SimulationBridge::HandleInterfaceLCM_usergait_file, this );
        interface_motor_control_thread_    = std::thread( &SimulationBridge::HandleMotorCtrlLcmThread, this );

        robot_runner_->SetCommandInterface( &cmd_interface_ );
        robot_runner_->SetSpiData( &shared_memory_().simToRobot.spiData );
        robot_runner_->SetSpiCommand( &shared_memory_().robotToSim.spiCommand );
        robot_runner_->SetRobotType( robot_ );
        robot_runner_->SetRobotAppearanceType( RobotAppearanceType::CURVED );
        robot_runner_->SetVectorNavData( &shared_memory_().simToRobot.vectorNav );
        robot_runner_->SetCheaterState( &shared_memory_().simToRobot.cheater_state );
        robot_runner_->SetRobotControlParameters( robot_params_ );
        robot_runner_->SetVisualizationData( &shared_memory_().robotToSim.visualizationData );
        robot_runner_->SetCyberdog2Visualization( &shared_memory_().robotToSim.mainCyberdog2Visualization );

        robot_runner_->Init();
        first_controller_run_ = false;
    }
    cmd_interface_.ProcessGamepadCommand( shared_memory_().simToRobot.gamepadCommand );

    robot_runner_->Run();
    robot_runner_->LCMPublishByThread();
}

void SimulationBridge::CyberdogLcmCmdCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const motion_control_request_lcmt* msg ) {
    ( void )buf;
    ( void )channel;
    ( void )msg;
    cmd_interface_.ProcessCyberdogLcmCommand( msg );
}

void SimulationBridge::UserGaitFileCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const file_send_lcmt* msg ) {
    ( void )buf;
    ( void )channel;
    file_recv_lcmt receive_msg_;
    receive_msg_.result = cmd_interface_.ProcessUserGaitFile( msg );
    user_gait_file_responce_lcm_.publish( "user_gait_result", &receive_msg_ );
}

void SimulationBridge::LcmCmdCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const robot_control_cmd_lcmt* msg ) {
    ( void )buf;
    ( void )channel;
    ( void )msg;
    cmd_interface_.ProcessLcmCommand( msg );
}

void SimulationBridge::LcmMotionCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const trajectory_command_lcmt* msg ) {
    ( void )buf;
    ( void )channel;
    cmd_interface_.ProcessLcmMotionCommand( msg );
}

void SimulationBridge::LcmMotorCtrlCallback( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const motor_ctrl_lcmt* msg ) {
    ( void )rbuf;
    ( void )chan;
    cmd_interface_.ProcessLcmMotorCtrlCommand( msg );
}

void SimulationBridge::HandleInterfaceLCM_cmd() {
    while ( !interface_lcm_quit_ ) {
        lcm_.handle();
    }
}

void SimulationBridge::HandleInterfaceLCM_cyberdog_cmd() {
    while ( !interface_lcm_quit_ ) {
        cyberdog_lcm_.handle();
    }
}

void SimulationBridge::HandleInterfaceLCM_motion_list() {
    while ( !interface_lcm_quit_ ) {
        motion_list_lcm_.handle();
    }
}

void SimulationBridge::HandleInterfaceLCM_usergait_file() {
    while ( !interface_lcm_quit_ ) {
        user_gait_file_lcm_.handle();
    }
}
void SimulationBridge::HandleMotorCtrlLcmThread() {
    while ( !interface_lcm_quit_ ) {
        motor_control_lcm_.handle();
    }
}