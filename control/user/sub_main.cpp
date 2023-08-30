#include <cassert>
#include <iostream>

#include "robot_controller.hpp"
#include "sub_main.hpp"

#if ( ONBOARD_BUILD == 1 )
#include "hardware_bridge_interface.hpp"
#else
#include "simulation_bridge_interface.hpp"
#endif

MasterConfig gMasterConfig;

/**
 * @brief Print a message describing the command line flags for the robot program
 *
 */
void printUsage() {
    printf( "Usage: robot [robot-id] [sim-or-robot] [parameters-from-file]\n"
            "\twhere robot-id:     c for cyberdog, m for cyberdog2\n"
            "\t      sim-or-robot: s for sim, r for robot\n"
            "\t      param-file:   f for loading parameters from file, l (or nothing) for LCM,\n"
            "\t                    this option can only be used in robot mode\n" );
}

/**
 * @brief Setup and run the given robot controller, parses command line arguments and starts the appropriate
 * driver.
 *
 * @param argc
 * @param argv
 * @param ctrl parent class pointer of the main controller
 * @param with_imu if use imu
 * @return int
 */
int SubMain( int argc, char** argv, RobotController* ctrl, bool with_imu ) {
    if ( argc != 3 && argc != 4 ) {
        printUsage();
        return EXIT_FAILURE;
    }
    // select robot type
    if ( argv[ 1 ][ 0 ] == 'c' ) {
        gMasterConfig.robot = RobotType::CYBERDOG;
    }
    else if ( argv[ 1 ][ 0 ] == 'm' ) {
        gMasterConfig.robot = RobotType::CYBERDOG2;
    }
    else {
        printUsage();
        return EXIT_FAILURE;
    }
    // select usage occasion, sim or real-bot
    if ( argv[ 2 ][ 0 ] == 's' ) {
        gMasterConfig.simulated = true;
    }
    else if ( argv[ 2 ][ 0 ] == 'r' ) {
        gMasterConfig.simulated = false;
    }
    else {
        printUsage();
        return EXIT_FAILURE;
    }
    // select source of parameters
    if ( argc == 4 && argv[ 3 ][ 0 ] == 'f' ) {
        gMasterConfig.load_from_file = true;
    }
    else {
        gMasterConfig.load_from_file = false;
    }

    // suppress with_imu unused error in linux, because we donot select it in simulation
    ( void )with_imu;

    // printf result of the options
    printf( "[Quadruped] Legged Robots Control Software\n" );
    printf( "        Quadruped: %s\n", gMasterConfig.robot == RobotType::CYBERDOG ? "Cyberdog" : gMasterConfig.robot == RobotType::CYBERDOG2 ? "Cyberdog2" : "Other Dog" );
    printf( "        Driver: %s\n", gMasterConfig.simulated ? "Development Simulation Driver" : "Quadruped Hardware Driver" );
    printf( "        Parameters Source: %s\n", gMasterConfig.load_from_file ? "Load parameters from file" : "Load parameters from network" );

    // dispatch the appropriate driver
    if ( gMasterConfig.simulated ) {
#if ( ONBOARD_BUILD == 1 )
        fprintf( stderr, "[ERROR] Can not run simulation mode on robot, exit!\n" );
#else
        if ( argc != 3 ) {
            printUsage();
            return EXIT_FAILURE;
        }
        if ( gMasterConfig.robot == RobotType::CYBERDOG || gMasterConfig.robot == RobotType::CYBERDOG2 ) {
            SimulationBridgeInterface simulationBridge( gMasterConfig.robot, ctrl );
            simulationBridge.Run();
            printf( "[Quadruped] Simulation Driver Run() has finished!\n" );
        }
        else {
            printf( "[ERROR] Unknown robot\n" );
            assert( false );
        }
#endif
    }
    else {
#if ( ONBOARD_BUILD == 1 )
        if ( gMasterConfig.robot == RobotType::CYBERDOG || gMasterConfig.robot == RobotType::CYBERDOG2 ) {
            HardwareBridgeInterface hw( ctrl, gMasterConfig.load_from_file, gMasterConfig.robot );
            hw.SetWithImu( with_imu );
            hw.Run();
            printf( "[Quadruped] Hardware Driver Run() has finished!\n" );
        }
        else {
            printf( "[ERROR] Unknown robot\n" );
            assert( false );
        }
#else
        fprintf( stderr, "[ERROR] Can not run real robot mode in PC, exit!\n" );
#endif
    }

    return 0;
}
