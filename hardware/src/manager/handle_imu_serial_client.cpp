#include <algorithm>
#include <iostream>
#include <memory>
#include <signal.h>
#include <sys/wait.h>
#include <vector>

#include "fork_program.hpp"
#include "handle_imu_serial_client.hpp"
#include "manager_parameters.hpp"
#include "manager_utils.hpp"
#include "parse_json_on_rork_para_conf.hpp"
#include "serial.h"
#include "utilities/dog_toolkit.hpp"

#define OK_REPLY "OK"

int MkdirPath( char* file_path, mode_t mode ) {
    char* p = file_path;
    do {
        p = strchr( p + 1, '/' );
        if ( p )
            *p = '\0';
        if ( mkdir( file_path, mode ) == -1 ) {
            if ( errno != EEXIST ) {
                *p = '/';
                return -1;
            }
        }
        if ( p )
            *p = '/';
    } while ( p );
    return 0;
}

void CreateImuLogDir( const char* log_root_path, const char* name ) {
    struct stat st;  // = { 0 };
    memset( &st, 0, sizeof( struct stat ) );

    std::string log_dir = std::string( log_root_path ) + "/" + name;
    std::cout << "log_dir: " << log_dir << std::endl;
    const char* log_dir_ptr = log_dir.c_str();
    char*       log_dir_cpy = new char[ strlen( log_dir_ptr ) + 1 ];
    strcpy( log_dir_cpy, log_dir_ptr );
    MkdirPath( log_dir_cpy, 0700 );
    std::string std_log_dir( log_dir_ptr );
    std::string std_err_dir( log_dir_ptr );
    std_log_dir += "/stdout";
    std_err_dir += "/stderr";
    if ( stat( std_log_dir.c_str(), &st ) == -1 ) {
        mkdir( std_log_dir.c_str(), 0700 );
    }
    if ( stat( std_err_dir.c_str(), &st ) == -1 ) {
        mkdir( std_err_dir.c_str(), 0700 );
    }

    delete[] log_dir_cpy;
}

void HandleIMUSerialClient( std::vector< pid_t >& childs_pid, const pid_t extprog_pid ) {
    Serial ctrl_serial_port;
    if ( ctrl_serial_port.OpenPort( "/dev/ttyS2" ) < 0 ) {
        fprintf( stderr, "[IMU Serial] Open Serial <%s> failed!\n", "/dev/ttyS2" );
        return;
    }
    ctrl_serial_port.SetPara( 2 );  // baudrate 9600
    printf( "HandleIMUSerialClient: Open Control serial succeed!\n" );

    while ( true ) {
        char start_cmd[ 512 ] = { 0 };
        int  read_len         = ctrl_serial_port.ReadData( ( unsigned char* )start_cmd, sizeof( start_cmd ) - 1, false );
        if ( read_len >= static_cast< int >( strlen( "imu_calibrate" ) ) ) {
            if ( strstr( start_cmd, "imu_calibrate" ) != NULL ) {
                std::cout << "[imu_calibrate] cmd received" << std::endl;

                /* kill cyberdog_control process */
                if ( extprog_pid != -1 ) {
                    if ( kill( extprog_pid, SIGKILL ) < 0 ) {
                        sleep( 1 );
                        if ( kill( extprog_pid, SIGKILL ) < 0 ) {
                            if ( errno != ESRCH ) {
                                std::cerr << "Failed to kill cyberdog_control" << std::endl;
                                exit( EXIT_FAILURE );
                            }
                        }
                    }
                    RemoveVecElement( childs_pid, extprog_pid );
                }

                /*    prepare log directories   */
                CreateImuLogDir( IMU_CALIB_LOG_PATH, BIAS_SAMPLE_COMMAND );
                /*    fork IMU calib process    */
                pid_t             calib_pid;
                std::string       imu_calibrate_command = BIAS_SAMPLE_COMMAND;
                std::string       imu_test_real_path    = exe_path + IMU_TEST_RELATIVE_PATH;
                const char* const imu_calib_argv[]      = { imu_calibrate_command.c_str(), "--serial", NULL }; /* arguments */
                if ( ForkProgram( &calib_pid, imu_calibrate_command.c_str(), imu_test_real_path.c_str(), IMU_CALIB_LOG_PATH, ( char* const* )imu_calib_argv ) < 0 ) {
                    fprintf( stderr, "\nfailed to launch %s, quitting...\n", imu_calibrate_command.c_str() );
                    exit( EXIT_FAILURE );
                }
                else {
                    ctrl_serial_port.WriteData( OK_REPLY, strlen( OK_REPLY ) );
                    childs_pid.push_back( calib_pid );
                    waitpid( calib_pid, NULL, 0 );
                    RemoveVecElement( childs_pid, calib_pid );
                }
            }
            else {
                std::cout << "Invalid cmd received: " << std::endl;
                PrintHex( std::cout, ( int8_t* )start_cmd, read_len );
            }
        }

        sleep( 2 );
    }
}
