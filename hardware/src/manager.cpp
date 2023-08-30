#include "utilities/dog_toolkit.hpp"
#include "fork_program.hpp"
#include "handle_client_initial.hpp"
#include "handle_imu_serial_client.hpp"
#include "libraries.h"
#include "manager_parameters.hpp"
#include "parse_json_on_rork_para_conf.hpp"
#include "rt/spi_handler.hpp"
#include "serial.h"

#include <utilities/timer.hpp>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <stdio.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <vector>

#include <document.h>
#include <istreamwrapper.h>
#include <ostreamwrapper.h>
#include <prettywriter.h>
#include <stringbuffer.h>

#define Hardware_id "/sys/firmware/devicetree/base/soc@03000000/board_id"

std::vector< pid_t >            childs_pid;
std::string                     exe_path;
std::vector< ForkParaConfJson > g_fork_json_datas;

bool reqMCUVersion( Serial& serial );

bool getAllMcuVersionInfo();

/* clean up zombie child process */
void cleanChild( int signo ) {
    ( void )signo;
    int stat;
    while ( waitpid( -1, &stat, WNOHANG ) > 0 )
        ;
}

/* capture signals for quitting program */
void quitProg( int signo ) {
    fprintf( stdout, "Received signal %d\nkilling child processes...\n", signo );
    for ( pid_t pid : childs_pid ) {
        kill( pid, SIGTERM );
    }
    exit( 0 );
}

/* main function */
int main( int argc, char** argv ) {

    time_t start_time = time( NULL );
    fprintf( stdout, "Program started on %s\n", ctime( &start_time ) );
    /* capture signals */
    signal( SIGCHLD, cleanChild );
    signal( SIGTERM, quitProg );
    signal( SIGINT, quitProg );

    if ( argc >= 2 ) {
        exe_path = argv[ 1 ];
    }
    else {
        exe_path = "/robot/";
    }

    // parse json data from json file on execute command
    auto sPtrParseJson = std::make_shared< ParseJsonOnForkParaConf >();
    g_fork_json_datas    = sPtrParseJson->ParseJsonData( ( std::string( exe_path ) + FORK_CONFIGURE_JSON_PATH ).c_str() );

    RobotType     robot_type;
    std::string   hardware_id;
    std::ifstream f( Hardware_id, std::ios::in );
    if ( f.good() ) {
        f >> hardware_id;
        std::cout << "CYBERDOG2 soc@03000000/board_id:" << hardware_id << std::endl;
        robot_type = RobotType::CYBERDOG2;
        f.close();
    }
    else
        robot_type = RobotType::CYBERDOG;

    // get mcu version  information from mcu
    bool ret = getAllMcuVersionInfo();
    if ( ret ) {
        std::cout << "[GET ALL MCU INFO] Get mcu info Success!" << std::endl;
    }
    else {
        std::cerr << "[GET ALL MCU INFO] Get mcu info Failed!" << std::endl;
    }

    /*fork all programs in json file*/
    pid_t mit_ctrl_pid = 0;
    for ( auto iter = g_fork_json_datas.begin(); iter != g_fork_json_datas.end(); iter++ ) {
        pid_t       exe_pid        = -1;
        std::string exe_real_path  = exe_path + iter->object_path;
        size_t      paraNum        = iter->para_values.size();
        char*       exe_argv[ 50 ] = {};
        exe_argv[ 0 ]              = ( char* )iter->name.c_str();
        exe_argv[ paraNum + 1 ]    = NULL;

        for ( size_t argv_num = 0; argv_num < paraNum; argv_num++ ) {
            exe_argv[ argv_num + 1 ] = ( char* )iter->para_values[ argv_num ].c_str();
        }
        if ( iter->name == "cyberdog_control" ) {
            if ( *exe_argv[ 1 ] == 'a' ) {
                if ( robot_type == RobotType::CYBERDOG2 )
                    *exe_argv[ 1 ] = 'm';  // cyberdog2
                else
                    *exe_argv[ 1 ] = 'c';  // cyberdog
            }
        }

        if ( ForkProgram( &exe_pid, iter->name.c_str(), exe_real_path.c_str(), iter->log_path.c_str(), ( char* const* )exe_argv ) < 0 ) {
            fprintf( stderr, "\nfailed to launch %s, quitting...\n", iter->name.c_str() );
            exit( EXIT_FAILURE );
        }

        if ( iter->name == "cyberdog_control" ) {
            mit_ctrl_pid = exe_pid;
        }

        childs_pid.push_back( exe_pid );
    }

    /* launch a thread for IMU serial call */
    // std::thread imu_serial( HandleIMUSerialClient, std::ref( childs_pid ), mit_ctrl_pid );

    /* below setup tcp socket server */
    char ip[ 50 ]   = "";
    char port[ 50 ] = "";
    /* use default ip and port */
    if ( argc != 4 ) {
        strncpy( ip, IP, sizeof( ip ) );
        strncpy( port, PORT, sizeof( port ) );
        fprintf( stdout, "IP or Port not specified, set to default %s:%s\n", ip, port );
        fprintf( stdout,
                 "You may specify custom ip and port by using: %s "
                 "<cyberdog_control path> <bind ip> <bind port>\n",
                 argv[ 0 ] );
        fprintf( stdout, "Use 0.0.0.0 to bind to all available interfaces\n\n" );
        /* use specified ip and port */
    }
    else {
        strncpy( ip, argv[ 2 ], sizeof( ip ) );
        strncpy( port, argv[ 3 ], sizeof( port ) );
    }

    int                listenfd, connfd;
    pid_t              child_pid;
    socklen_t          cli_len;
    struct sockaddr_in cli_addr, serv_addr;

    /* create socket */
    if ( ( listenfd = socket( AF_INET, SOCK_STREAM, 0 ) ) < 0 ) {
        perror( "Socket Error" );
        exit( EXIT_FAILURE );
    }

    /* set socket options */
    int optval = 1;
    if ( setsockopt( listenfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof( optval ) ) < 0 ) {
        perror( "Set Socket Options Error" );
        exit( EXIT_FAILURE );
    }

    bzero( &serv_addr, sizeof( serv_addr ) );
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port   = htons( strtol( port, NULL, 10 ) );
    inet_pton( AF_INET, ip, &serv_addr.sin_addr );

    /* bind to ip and port */
    if ( ::bind( listenfd, ( struct sockaddr* )&serv_addr, sizeof( serv_addr ) ) < 0 ) {
        perror( "Bind Error" );
        exit( EXIT_FAILURE );
    }

    /* start listening */
    if ( listen( listenfd, 100 ) < 0 ) {
        perror( "Listen Error" );
        exit( EXIT_FAILURE );
    }

    fprintf( stdout, "Server listening on %s:%s\n", ip, port );

    while ( 1 ) {
        cli_len = sizeof( cli_addr );
        /* Accept incoming connections */
        if ( ( connfd = accept( listenfd, ( struct sockaddr* )&cli_addr, &cli_len ) ) < 0 ) {
            if ( errno != EINTR )
                perror( "Accept Connection Error" );
        }
        else {
            char client[ 20 ] = {};
            fprintf( stdout, "Incoming connection from %s:%d  ", inet_ntop( AF_INET, &( cli_addr.sin_addr ), client, 20 ), ntohs( cli_addr.sin_port ) );
            fflush( stdout );

            /* fork a child to process the client, this block executed by child only */
            if ( ( child_pid = fork() ) == 0 ) {
                // TODO add the pid into child_pids
                close( listenfd );                           /* close listening socket */
                HandleClientInitialize( connfd, mit_ctrl_pid ); /* handle client, never return */
                exit( EXIT_FAILURE );                        /* should not reach here */
            }

            fprintf( stdout, "Allocated PID: %d\n", child_pid );
            fflush( stdout );
            close( connfd ); /* parent closes connected socket */
        }
    }

    exit( EXIT_FAILURE ); /* should not reach here */
}

bool reqMCUVersion( Serial& serial ) {
    unsigned char req_data[ 7 ] = { 0xAA, 0x55, 0x02, 0x0B, 0x01, 0x00, 0x00 };
    // Get checksum
    int16_t checksum = 0;
    for ( int i = 2; i < 5; i++ ) {
        checksum += req_data[ i ];
    }
    req_data[ 5 ] = 0xff & checksum;
    req_data[ 6 ] = ( ( 0xff << 8 ) & checksum ) >> 8;

    for ( int i = 0; i < 4; i++ ) {
        tcflush( serial.fd(), TCIOFLUSH );
        serial.WriteData( ( char* )req_data, 7 );
        Timer t;
        while ( t.GetElapsedSeconds() < 1 ) {  // timeout 2s
            RawImuData imu_data;
            ImuResult  imu_res;
            GetImuData( imu_data, serial, imu_res );
            if ( imu_data.type == 0x0B ) {
                char buff[ 3 ] = { 0 };
                memcpy( buff, ( char* )&imu_data, sizeof( buff ) );
                if ( buff[ 2 ] == 1 ) {
                    return true;
                }
                else {
                    std::cerr << "[MCU INFO] Failed to require MCU version" << std::endl;
                    continue;
                }
            }
            else {
                static int unrelated_iter = 0;
                if ( unrelated_iter % 300 == 0 ) {
                    std::cout << "[MCU INFO] Received an unrelated type: " << ( int )imu_data.type << std::endl;
                }
                unrelated_iter++;
            }
        }
    }
    std::cerr << "[MCU INFO] Get MCU INFO TimeOut in 4s!" << std::endl;
    return false;
}
#ifndef USE_NEW_BOARD
bool getAllMcuVersionInfo() {
    bool   isUSB_IMU_EXIST = true;
    Serial uart_serial_port;
    if ( uart_serial_port.OpenPort( "/dev/ttyS1" ) < 0 ) {
        fprintf( stderr, "[Serial] Open Serial <%s> failed!\n", "/dev/ttyS1" );
        return false;
    }
    uart_serial_port.SetPara( 6 );  // baudrate 921600
    printf( "Open onboard IMU serial succeed!\n" );

    Serial bms_serial_port;
    if ( bms_serial_port.OpenPort( BMS_SERIAL_PORT ) < 0 ) {
        fprintf( stderr, "[Serial] Open Serial <%s> failed!\n", BMS_SERIAL_PORT );
        return false;
    }
    bms_serial_port.SetPara( 7 );  // baudrate 115200
    printf( "Open BMS serial succeed!\n" );

    Serial usb_serial_port;
    if ( usb_serial_port.OpenPort( "/dev/ttyUSB0" ) < 0 ) {
        fprintf( stderr, "[Serial] Open Serial <%s> failed!\n", "/dev/ttyUSB0" );
        isUSB_IMU_EXIST = false;
        // return false;
    }
    else {
        usb_serial_port.SetPara( 6 );  // baudrate 921600
        printf( "Open usb IMU serial succeed!\n" );
    }

    std::vector< std::string > mcu_string{
        "ONBOARD IMU",  // 0x00
        "SPINE1",       // 0x01
        "SPINE2",       // 0x02
        "LEG1M1",       // 0x03
        "LEG1M2",       // 0x04
        "LEG1M3",       // 0x05
        "LEG0M1",       // 0x06
        "LEG0M2",       // 0x07
        "LEG0M3",       // 0x08
        "LEG3M1",       // 0x09
        "LEG3M2",       // 0x0a
        "LEG3M3",       // 0x0b
        "LEG2M1",       // 0x0c
        "LEG2M2",       // 0x0d
        "LEG2M3",       // 0x0e
    };

    std::vector< std::string > info_string{
        "ID_NUMBER",        // 0x00
        "BOOT_VERSION",     // 0x01
        "BOOT_DATE",        // 0x02
        "BOOT_TIME",        // 0x03
        "APP_VERSION",      // 0x04
        "APP_GIT_VERSION",  // 0x05
        "APP_DATE",         // 0x06
        "APP_TIME",         // 0x07
        "APP_NAME",         // 0x08
    };

    std::map< std::string, std::string > mcu_app_versions;
    std::map< std::string, std::string > mcu_boot_versions;

    std::vector< char > mcu_info;
    std::string         mcu_type_str;

    for ( int i = 1; i < 8; i++ ) {
        for ( size_t mcu = 0; mcu < mcu_string.size(); mcu++ ) {
            mcu_info     = GetMcuInfo( uart_serial_port, mcu, i );
            mcu_type_str = mcu_string[ mcu ];

            bool ret = false;
            for ( auto iter = mcu_info.begin(); iter != mcu_info.end(); iter++ ) {
                if ( *iter != '\0' ) {
                    ret = true;
                }
            }

            if ( mcu_info.empty() || !ret ) {  // can not get target MCU INFO
                std::cerr << "[GET ALL MCU INFO] Can not get " << info_string[ i ] << " of " << mcu_type_str << std::endl;
                mcu_app_versions[ mcu_type_str ] = "NONE";
            }
            else {
                mcu_app_versions[ mcu_type_str ] = ( ( unsigned char )mcu_info[ 0 ] == 0xff ? "undefined" : std::string( mcu_info.data() ) );
                std::cout << "[GET ALL MCU INFO] Got " << mcu_type_str << " " << info_string[ i ] << ": " << mcu_app_versions[ mcu_type_str ] << std::endl;
            }
        }

        if ( isUSB_IMU_EXIST ) {
            mcu_info     = GetMcuInfo( usb_serial_port, 0, i );
            mcu_type_str = "USB IMU";
            if ( mcu_info.empty() ) {  // can not get target MCU INFO
                std::cerr << "[GET ALL MCU INFO] Can not get " << info_string[ i ] << " of " << mcu_type_str << std::endl;
                mcu_app_versions[ mcu_type_str ] = "NONE";
            }
            else {
                mcu_app_versions[ mcu_type_str ] = ( ( unsigned char )mcu_info[ 0 ] == 0xff ? "undefined" : std::string( mcu_info.data() ) );
                std::cout << "[MCU INFO] Got " << mcu_type_str << " " << info_string[ i ] << ": " << mcu_app_versions[ mcu_type_str ] << std::endl;
            }
        }
        else {
            mcu_app_versions[ "USB IMU" ] = "NONE";
            std::cout << "[WARNING]:check the serial imu-usb is not existed,so set the \"NONE\" on \"mcu_app_versions[\"USB IMU\"]\" ！！！" << std::endl;
        }

        for ( size_t num = 0; num < 5; num++ ) {
            mcu_info     = GetMcuInfo( bms_serial_port, 15, i );
            mcu_type_str = "BMS";
            if ( mcu_info.empty() ) {  // can not get target GET ALL MCU INFO
                std::cerr << "[GET ALL MCU INFO] Can not get " << info_string[ i ] << " of " << mcu_type_str << std::endl;
                mcu_app_versions[ mcu_type_str ] = "NONE";
                std::cout << "the num of mcu_info.empty() is : " << num + 1 << std::endl;
            }
            else {
                mcu_app_versions[ mcu_type_str ] = ( ( unsigned char )mcu_info[ 0 ] == 0xff ? "undefined" : std::string( mcu_info.data() ) );
                std::cout << "[GET ALL MCU INFO] Got " << mcu_type_str << " " << info_string[ i ] << ": " << mcu_app_versions[ mcu_type_str ] << std::endl;
                break;
            }
        }

        for ( auto& pair : mcu_app_versions ) {
            mcu_boot_versions[ pair.first ] = mcu_boot_versions[ pair.first ] + pair.second + "|";
        }
    }

    // get all softWare version
    std::string   img_result;
    std::ifstream img_version_file( "/etc/os-release" );
    if ( img_version_file.is_open() ) {
        rapidjson::IStreamWrapper isw{ img_version_file };
        rapidjson::Document       doc{};
        doc.ParseStream( isw );
        if ( doc.HasMember( "fw_arm_ver" ) && doc[ "fw_arm_ver" ].IsString() ) {
            img_result = doc[ "fw_arm_ver" ].GetString();
        }
    }

    mcu_boot_versions.insert( std::make_pair( "Img_Version", img_result ) );

    std::string   cheetah_buffer;
    std::ifstream cheetah_version_file( "/robot/robot-software/version.txt" );
    if ( cheetah_version_file.is_open() ) {
        getline( cheetah_version_file, cheetah_buffer );
    }

    mcu_boot_versions.insert( std::make_pair( "Cheetah_Version", cheetah_buffer ) );

    std::string   manager_buffer;
    std::ifstream manager_version_file( "/robot/manager_config/version.txt" );
    if ( manager_version_file.is_open() ) {
        getline( manager_version_file, manager_buffer );
    }

    mcu_boot_versions.insert( std::make_pair( "Manager_Version", manager_buffer ) );

    std::string   imu_test_buffer;
    std::ifstream imu_test_version_file( "/robot/imu-test/version.txt" );
    if ( imu_test_version_file.is_open() ) {
        getline( imu_test_version_file, imu_test_buffer );
    }

    mcu_boot_versions.insert( std::make_pair( "Imu_test_Version", imu_test_buffer ) );

    // close all serial port
    // uart_serial_port.ClosePort();
    // bms_serial_port.ClosePort();
    // usb_serial_port.ClosePort();

    // write mcu info to json file
    rapidjson::Document                 document;
    rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
    rapidjson::Value                    root( rapidjson::kObjectType );
    rapidjson::Value                    key( rapidjson::kStringType );
    rapidjson::Value                    value( rapidjson::kStringType );

    std::ofstream             ofs( "/mnt/misc/soft_hardware_version_info.json" );
    rapidjson::OStreamWrapper osw( ofs );

    for ( std::map< std::string, std::string >::const_iterator it = mcu_boot_versions.begin(); it != mcu_boot_versions.end(); ++it ) {
        key.SetString( it->first.c_str(), allocator );
        value.SetString( it->second.c_str(), allocator );
        root.AddMember( key, value, allocator );
    }

    rapidjson::PrettyWriter< rapidjson::OStreamWrapper > writer( osw );
    // d.Accept(writer2);
    root.Accept( writer );

    return true;
}
#else
bool getAllMcuVersionInfo() {
    Timer timeCost;
    std::cout << "[IMU-SPINE-MOTOR VERSION] start getting versions by mcu and spi" << std::endl;
    Serial imu_serial_port;
    if ( imu_serial_port.OpenPort( "/dev/ttyS1" ) < 0 ) {
        fprintf( stderr, "[Serial] Open Serial <%s> failed!\n", "/dev/ttyS1" );
        return false;
    }
    imu_serial_port.SetPara( 6 );  // baudrate 921600

    std::vector< std::string > app_versions;
    std::vector< std::string > boot_versions;
    std::vector< std::string > fsns;
    std::vector< std::string > imu_spine_motor_info_name = {
        "L_SYSTEM_IMAGE",
        "L_IMU",     // 0x00
        "L_SPIE1",   // 0x01
        "L_SPIE2",   // 0x02
        "MOTOR_1",   // 0x03
        "MOTOR_7",   // 0x04
        "MOTOR_2",   // 0x05
        "MOTOR_8",   // 0x06
        "MOTOR_3",   // 0x07
        "MOTOR_9",   // 0x08
        "MOTOR_4",   // 0x09
        "MOTOR_10",  // 0x0a
        "MOTOR_5",   // 0x0b
        "MOTOR_11",  // 0x0c
        "MOTOR_6",   // 0x0d
        "MOTOR_12"   // 0x0e
    };

    std::vector< std::string > version_fsn_name = { "APP", "BOOT", "FSN" };
    usleep( 2 * 1000 );
    // get app+boot version of imu
    std::vector< char > mcu_info;
    int                 read_loop = 0;
    mcu_info.clear();
    while ( mcu_info.empty() && read_loop < 3 ) {
        mcu_info = GetMcuInfo( imu_serial_port, 0x06, 0x31, 0x04 );
        read_loop++;
    }
    if ( mcu_info.empty() ) {  // can not get target MCU INFO
        app_versions.push_back( "FAIL" );
        boot_versions.push_back( "FAIL" );
    }
    else {
        app_versions.push_back( Conversion( ( char* )&mcu_info[ 0 ] ) );
        boot_versions.push_back( Conversion( ( char* )&mcu_info[ 16 ] ) );
    }
    // TODO: decrease time dura, 20ms is ok
    usleep( 60 * 1000 );
    // get fsn of imu
    read_loop = 0;
    mcu_info.clear();
    while ( mcu_info.empty() && read_loop < 3 ) {
        mcu_info = GetMcuInfo( imu_serial_port, 0x06, 0x31, 0x05 );
        read_loop++;
    }
    if ( mcu_info.empty() ) {
        fsns.push_back( "FAIL" );
    }
    else {
        std::string imu_fsn;
        for ( uint16_t i = 0; i < 12; i++ )
            imu_fsn.push_back( mcu_info.at( i ) );
        fsns.push_back( imu_fsn );
    }
    // TODO: decrease time dura, 20ms is ok
    usleep( 60 * 1000 );
    // set imu serial to be 0x02， select to communicate by spi or serial
    int           ctrl_board_id = 1;
    std::string   board_id_data;
    std::ifstream board_id_file( "/sys/firmware/devicetree/base/soc@03000000/board_id" );
    if ( board_id_file.is_open() ) {
        getline( board_id_file, board_id_data );
        ctrl_board_id = std::atoi( board_id_data.c_str() );
    }
    if ( ctrl_board_id < 2 ) {
        GetMcuInfo( imu_serial_port, 0x02, 0x01, 0x00 );
        std::cout << "[IMU COMMUNICATE METHOD] Communicate By Serial!" << std::endl;
    }
    else {
        GetMcuInfo( imu_serial_port, 0x02, 0x01, 0x01 );
        std::cout << "[IMU COMMUNICATE METHOD] Communicate By Spi!" << std::endl;
    }
    usleep( 2 * 1000 );

    // open spiHandle and get spine and motor's info
    SpiHandler spiHandler;
    spiHandler.InitializeSpi();

    SpineCommand read_version_cmd;
    memset( &read_version_cmd, 0, sizeof( SpineCommand ) );
    read_version_cmd.flags[ 0 ] = 2;
    spiHandler.SetSpineCommand( read_version_cmd, 0 );
    spiHandler.SetSpineCommand( read_version_cmd, 1 );
    // send zero command to disable motor
    for ( int i = 0; i < 5; i++ ) {
        spiHandler.DriverRunTest( 3 );
        usleep( 1000 * 2 );
    }
    spiHandler.FlushSpiBuffer();
    usleep( 2000 );
    // set read_version flag to spine
    Timer    t;
    float    time_delay         = 10.0;
    uint16_t idSum              = 0;
    read_version_cmd.flags[ 0 ] = 103;
    bool      read_succeed[ 2 ] = { false };
    char      version[ 2 ][ 32 ];
    SpineDate read_version_data[ 2 ];
    int       read_succeed_iter[ 2 ] = { 0 };
    int       min_iter               = 1;
    while ( t.GetElapsedMilliseconds() < time_delay ) {
        idSum = 0;
        for ( uint16_t id = 0; id < 2; id++ ) {
            if ( !read_succeed[ id ] ) {
                spiHandler.SetSpineCommand( read_version_cmd, id );
                idSum += ( id + 1 );
            }
        }

        spiHandler.DriverRunTest( idSum );

        char* data_tmp;
        for ( int16_t id = 0; id < 2; id++ ) {
            if ( !read_succeed[ id ] ) {
                spiHandler.GetSpineData( read_version_data[ id ], id );
                data_tmp = ( char* )&read_version_data[ id ];

                if ( read_succeed_iter[ id ] >= min_iter && data_tmp[ 0 ] == 0 && data_tmp[ 1 ] != 0 ) {
                    read_succeed[ id ] = true;
                    for ( uint16_t i = 0; i < 32; i++ ) {
                        version[ id ][ i ] = data_tmp[ i + 1 ];
                    }
                }
            }
        }
        if ( read_succeed[ 0 ] && read_succeed[ 1 ] ) {
            break;
        }
        usleep( 2000 );
        read_succeed_iter[ 0 ]++;
        read_succeed_iter[ 1 ]++;
    }

    for ( int16_t id = 0; id < 2; id++ ) {
        if ( read_succeed[ id ] ) {
            app_versions.push_back( Conversion( &version[ id ][ 16 ] ) );
            boot_versions.push_back( Conversion( &version[ id ][ 0 ] ) );
        }
        else {
            app_versions.push_back( "FAIL" );
            boot_versions.push_back( "FAIL" );
        }
    }

    t.StartTimer();
    SpineCommand read_sn_cmd;
    memset( ( uint8_t* )&read_sn_cmd, 0, sizeof( SpineCommand ) );
    read_sn_cmd.flags[ 0 ] = 102;

    read_succeed[ 0 ]      = false;
    read_succeed[ 1 ]      = false;
    read_succeed_iter[ 0 ] = 0;
    read_succeed_iter[ 1 ] = 0;
    std::string spi_sn[ 2 ];
    SpineDate   read_sn_data[ 2 ];
    while ( t.GetElapsedMilliseconds() < time_delay ) {
        idSum = 0;
        for ( uint16_t id = 0; id < 2; id++ ) {
            if ( !read_succeed[ id ] ) {
                spiHandler.SetSpineCommand( read_sn_cmd, id );
                idSum += ( id + 1 );
            }
        }

        spiHandler.DriverRunTest( idSum );

        char* data_tmp;
        for ( int16_t id = 0; id < 2; id++ ) {
            if ( !read_succeed[ id ] ) {
                spiHandler.GetSpineData( read_sn_data[ id ], id );
                data_tmp = ( char* )&read_sn_data[ id ];

                if ( read_succeed_iter[ id ] >= min_iter && data_tmp[ 0 ] == 0 && data_tmp[ 1 ] != 0 ) {
                    read_succeed[ id ] = true;
                    for ( uint16_t i = 0; i < 12; i++ )
                        spi_sn[ id ].push_back( data_tmp[ i + 1 ] );
                }
            }
        }

        if ( read_succeed[ 0 ] && read_succeed[ 1 ] ) {
            break;
        }
        usleep( 2000 );
        read_succeed_iter[ 0 ]++;
        read_succeed_iter[ 1 ]++;
    }
    for ( int16_t id = 0; id < 2; id++ ) {
        if ( read_succeed[ id ] ) {
            fsns.push_back( spi_sn[ id ] );
        }
        else {
            fsns.push_back( "FAIL" );
        }
    }

    // get motor version info
    SpineCommand read_motor_cmd;
    memset( ( uint8_t* )&read_motor_cmd, 0, sizeof( SpineCommand ) );
    unsigned char* cmd_tmp = ( unsigned char* )&read_motor_cmd;

    SpineDate   read_motor_data[ 2 ];
    bool        read_motor_succeed[ 2 ] = { false };
    std::string motor_data[ 2 ];

    time_delay = 600;
    // read app+boot version
    for ( int i = 0; i < 6; i++ ) {
        t.StartTimer();
        motor_data[ 0 ].clear();
        motor_data[ 1 ].clear();
        read_motor_succeed[ 0 ] = false;
        read_motor_succeed[ 1 ] = false;
        read_succeed_iter[ 0 ]  = 0;
        read_succeed_iter[ 1 ]  = 0;
        // set cmd
        read_motor_cmd.flags[ 0 ] = 105;

        // get data
        memset( &read_motor_data[ 0 ], 0, sizeof( SpineDate ) );
        memset( &read_motor_data[ 1 ], 0, sizeof( SpineDate ) );
        while ( t.GetElapsedMilliseconds() < time_delay ) {
            idSum = 0;
            for ( uint16_t id = 0; id < 2; id++ ) {
                if ( !read_motor_succeed[ id ] ) {
                    read_motor_cmd.flags[ 0 ] = 105;
                    cmd_tmp[ 0 ]              = i + 1 + 6 * id;
                    spiHandler.SetSpineCommand( read_motor_cmd, id );
                    idSum += ( id + 1 );
                }
                else {
                    read_motor_cmd.flags[ 0 ] = 2;
                    cmd_tmp[ 0 ]              = 0;
                    spiHandler.SetSpineCommand( read_motor_cmd, id );
                }
            }

            spiHandler.DriverRunTest( idSum );

            for ( uint16_t id = 0; id < 2; id++ ) {
                if ( !read_motor_succeed[ id ] ) {
                    spiHandler.GetSpineData( read_motor_data[ id ], id );
                    char* data_tmp = ( char* )&read_motor_data[ id ];

                    if ( read_succeed_iter[ id ] >= min_iter
                         && ( ( data_tmp[ 0 ] == 0 && data_tmp[ 1 ] != 0 )
                              /* || ( data_tmp[ 0 ] == 1 && data_tmp[ 1 ] == 0 ) */ ) ) {
                        read_motor_succeed[ id ] = true;
                        for ( int num = 0; num < 32; num++ ) {
                            motor_data[ id ].push_back( data_tmp[ num + 1 ] );
                        }
                    }
                }
            }

            if ( read_motor_succeed[ 0 ] && read_motor_succeed[ 1 ] ) {
                break;
            }
            usleep( 2000 );
            read_succeed_iter[ 0 ]++;
            read_succeed_iter[ 1 ]++;
        }
        for ( uint16_t id = 0; id < 2; id++ ) {
            if ( read_motor_succeed[ id ] ) {
                app_versions.push_back( Conversion( &motor_data[ id ][ 16 ] ) );
                boot_versions.push_back( Conversion( &motor_data[ id ][ 0 ] ) );
            }
            else {
                app_versions.push_back( "FAIL" );
                boot_versions.push_back( "FAIL" );
            }
        }
    }
    usleep( 2000 );
    // read motor's fsn
    min_iter = 1;
    for ( int i = 0; i < 6; i++ ) {
        t.StartTimer();
        motor_data[ 0 ].clear();
        motor_data[ 1 ].clear();
        read_motor_succeed[ 0 ] = false;
        read_motor_succeed[ 1 ] = false;
        read_succeed_iter[ 0 ]  = 0;
        read_succeed_iter[ 1 ]  = 0;
        // set cmd
        read_motor_cmd.flags[ 0 ] = 104;
        cmd_tmp[ 0 ]              = i + 1;
        // get data
        memset( &read_motor_data[ 0 ], 0, sizeof( SpineDate ) );
        memset( &read_motor_data[ 1 ], 0, sizeof( SpineDate ) );
        while ( t.GetElapsedMilliseconds() < time_delay ) {
            idSum = 0;
            for ( int id = 0; id < 2; id++ ) {
                if ( !read_motor_succeed[ id ] ) {
                    read_motor_cmd.flags[ 0 ] = 104;
                    cmd_tmp[ 0 ]              = i + 1 + 6 * id;
                    spiHandler.SetSpineCommand( read_motor_cmd, id );
                    idSum += ( id + 1 );
                }
                else {
                    read_motor_cmd.flags[ 0 ] = 2;
                    cmd_tmp[ 0 ]              = 0;
                    spiHandler.SetSpineCommand( read_motor_cmd, id );
                }
            }

            spiHandler.DriverRunTest( idSum );

            for ( uint16_t id = 0; id < 2; id++ ) {
                if ( !read_motor_succeed[ id ] ) {
                    spiHandler.GetSpineData( read_motor_data[ id ], id );
                    char* data_tmp = ( char* )&read_motor_data[ id ];

                    if ( read_succeed_iter[ id ] >= min_iter
                         && ( ( data_tmp[ 0 ] == 0 && data_tmp[ 1 ] != 0 )
                              /* || ( data_tmp[ 0 ] == 1 && data_tmp[ 1 ] == 0 ) */ ) ) {
                        read_motor_succeed[ id ] = true;
                        for ( int num = 0; num < 12; num++ ) {
                            motor_data[ id ].push_back( data_tmp[ num + 1 ] );
                        }
                    }
                }
            }

            if ( read_motor_succeed[ 0 ] && read_motor_succeed[ 1 ] ) {
                break;
            }
            usleep( 2000 );
            read_succeed_iter[ 0 ]++;
            read_succeed_iter[ 1 ]++;
        }
        for ( uint16_t id = 0; id < 2; id++ ) {
            if ( read_motor_succeed[ id ] ) {
                fsns.push_back( motor_data[ id ] );
            }
            else {
                fsns.push_back( "FAIL" );
            }
        }
    }

    // get Mr813's image version and fsn
    std::string   img_app;
    std::ifstream img_version_file( "/etc/os-release" );
    if ( img_version_file.is_open() ) {
        rapidjson::IStreamWrapper isw{ img_version_file };
        rapidjson::Document       doc{};
        doc.ParseStream( isw );
        if ( doc.HasMember( "fw_arm_ver" ) && doc[ "fw_arm_ver" ].IsString() ) {
            img_app = doc[ "fw_arm_ver" ].GetString();
        }
        img_app = "V" + img_app;
    }

    FILE* stream;
    char  buff[ 13 ] = { '\0' };
    stream           = popen( "mikey get sn", "r" );
    fread( buff, 1, 12, stream );
    pclose( stream );
    std::string img_fsn = buff;

    rapidjson::Document document;
    document.SetObject();
    rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
    rapidjson::Value                    device_name( rapidjson::kStringType );

    std::ofstream             ofs( "/var/soft_hardware_version_info.json" );
    rapidjson::OStreamWrapper osw( ofs );
    // write system image version
    device_name.SetString( imu_spine_motor_info_name.at( 0 ).c_str(), allocator );
    // document.AddMember( device_name, rapidjson::StringRef( img_app.c_str() ), allocator );
    rapidjson::Value device_awary( rapidjson::kArrayType );
    rapidjson::Value obj( rapidjson::kObjectType );

    obj.AddMember( rapidjson::StringRef( version_fsn_name.at( 0 ).c_str() ), rapidjson::StringRef( img_app.c_str() ), allocator );

    obj.AddMember( rapidjson::StringRef( version_fsn_name.at( 2 ).c_str() ), rapidjson::StringRef( img_fsn.c_str() ), allocator );

    device_awary.PushBack( obj, allocator );

    document.AddMember( device_name, device_awary, allocator );
    // write imu and spie's info
    for ( uint16_t num = 1; num < imu_spine_motor_info_name.size(); num++ ) {
        int id = 0;
        if ( num < 4 )
            id = num - 1;
        else if ( num < 10 )
            id = 2 * num - 5;
        else
            id = 2 * num - 16;
        device_name.SetString( imu_spine_motor_info_name.at( id + 1 ).c_str(), allocator );
        rapidjson::Value device_awary( rapidjson::kArrayType );
        rapidjson::Value obj( rapidjson::kObjectType );

        obj.AddMember( rapidjson::StringRef( version_fsn_name.at( 0 ).c_str() ), rapidjson::StringRef( app_versions.at( id ).c_str() ), allocator );

        obj.AddMember( rapidjson::StringRef( version_fsn_name.at( 1 ).c_str() ), rapidjson::StringRef( boot_versions.at( id ).c_str() ), allocator );

        obj.AddMember( rapidjson::StringRef( version_fsn_name.at( 2 ).c_str() ), rapidjson::StringRef( fsns.at( id ).c_str() ), allocator );

        device_awary.PushBack( obj, allocator );

        document.AddMember( device_name, device_awary, allocator );
    }

    rapidjson::PrettyWriter< rapidjson::OStreamWrapper > writer( osw );
    document.Accept( writer );

    std::cout << "[TIME COST] to get all version info : " << timeCost.GetElapsedMilliseconds() << "  ms" << std::endl;

    return true;
}
#endif
