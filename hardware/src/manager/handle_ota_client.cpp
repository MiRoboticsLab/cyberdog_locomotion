#include <document.h>
#include <exception>
#include <filereadstream.h>
#include <fstream>
#include <iostream>
#include <istreamwrapper.h>
#include <lcm/lcm-cpp.hpp>
#include <ostreamwrapper.h>
#include <prettywriter.h>
#include <sstream>
#include <stringbuffer.h>
#include <writer.h>

#include "calc_md5.hpp"
#include "control_flags.hpp"
#include "handle_ota_client.hpp"
#include "header/lcm_type/robot_control_response_lcmt.hpp"
#include "manager_parameters.hpp"
#include "raw_socket.hpp"
#include "utilities/dog_toolkit.hpp"
#include "utilities/shm_help.hpp"

void HandleOtaClient( const int connfd, const pid_t extprog_pid ) {
    char buf[ BUF_LEN + 1 ];
    char buf_uppercase[ BUF_LEN + 1 ];

    /* process commands */
    while ( 1 ) {
        ReadCommandByNewLine( connfd, buf, BUF_LEN );
        strncpy( buf_uppercase, buf, BUF_LEN );
        // String2Upper( buf_uppercase );

        /* client upload new firmware */
        if ( strncmp( buf_uppercase, "PUSH", 4 ) == 0 )
            Push( connfd, extprog_pid );

        /* start flashing firmware */
        else if ( strncmp( buf_uppercase, "FLASH", 5 ) == 0 )
            Flash( connfd, buf_uppercase );

        /* client query firmware version */
        else if ( strncmp( buf_uppercase, "QUERY", 5 ) == 0 )
            Query( connfd );

        /* token write from nx to mr813*/
        else if ( strncmp( buf_uppercase, "TOKEN_WRITE", 11 ) == 0 )
            // TokenWrite( connfd, buf_uppercase );
            TokenWriteExpand( connfd, buf_uppercase );

        /* token read from mr 813 to nx*/
        else if ( strncmp( buf_uppercase, "TOKEN_READ", 10 ) == 0 )
            // TokenRead( connfd );
            TokenReadExpand( connfd, buf_uppercase );

        /* upload log from mr 813 to nx*/
        else if ( strncmp( buf_uppercase, "GET_LOG", 7 ) == 0 )
            LogUpload( connfd );

        /* reboot controller board */
        else if ( strncmp( buf_uppercase, "REBOOT", 6 ) == 0 )
            RebootOperateSystem( connfd );

        /* client help */
        else if ( strncmp( buf_uppercase, "HELP", 4 ) == 0 ) {
            WriteCommandByNewLine( connfd, "201 OK AVAILABLE COMMAND: PUSH FLASH QUERY REBOOT HELP QUIT" );

            /* client quit */
        }
        else if ( strncmp( buf_uppercase, "QUIT", 4 ) == 0 || strncmp( buf_uppercase, "EXIT", 4 ) == 0 ) {
            WriteCommandByNewLine( connfd, "200 GOODBYE" );
            fprintf( stdout, "Client on PID %d quitted normally.\n", getpid() );
            exit( 0 );

            /* invalid command */
        }
        else {
            WriteCommandByNewLine( connfd, "500 INVALID COMMAND" );
        }
    }

    exit( EXIT_FAILURE ); /* should not reach here */
}

class MotionStatus {
public:
    int    called = 0;
    int8_t status = -1;
};

void LcmHandler( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const robot_control_response_lcmt* msg, MotionStatus* ms ) {
    ( void )rbuf;
    ( void )chan;
    ms->status = msg->mode;
    ms->called = 1;
}

/**
 * @brief Client upload new firmware.
 *
 */
void Push( const int connfd, const pid_t extprog_pid ) {

    /* initialize a lcm_t instance */
    lcm::LCM lc( GetLcmUrlWithPort( 7670, 255 ) );
    if ( !lc.good() ) {
        WriteCommandByNewLine( connfd, "511 CANNOT CREATE LCM INSTANCE" );
        return;
    }

    /* receive message */
    MotionStatus ms;
    auto         sub_h = lc.subscribeFunction< robot_control_response_lcmt, MotionStatus* >( "robot_control_response", LcmHandler, &ms );

    time_t begin = time( NULL );
    while ( !ms.called ) {
        if ( time( NULL ) - begin > 5 ) {
            fprintf( stderr, "Read timeout, set robot status to 0\n" );
            ms.status = 0;
            ms.called = 1;
        }
        lc.handleTimeout( 1000 );
    }
    lc.unsubscribe( sub_h );
    ms.called = 0;

    fprintf( stdout, "Robot Status: %d\n", ms.status );

    /* if unsafe to do OTA update now, reject update, 0 = K_PASSIVE */
    if ( ms.status != kOff && ms.status != kPureDamper ) {
        WriteCommandByNewLine( connfd, "510 TRY LATER" );
        return;
    }

    /* kill cyberdog_control process */
    if ( extprog_pid != -1 ) {
        if ( kill( extprog_pid, SIGKILL ) < 0 ) {
            sleep( 1 );
            if ( kill( extprog_pid, SIGKILL ) < 0 ) {
                if ( errno != ESRCH ) {
                    WriteCommandByNewLine( connfd, "513 CANNOT KILL CYBERDOG_CONTROL PROCESS" );
                    return;
                }
            }
        }
    }

    WriteCommandByNewLine( connfd, "210 OK PLEASE UPLOAD FILE" );
}

/**
 * @brief Start flashing firmware.
 *
 */
void Flash( const int connfd, const char* buf ) {
    char input_md5[ 33 ]  = "";
    char result_md5[ 33 ] = "";
    // 1.0.0.1.20220510.095113：23
    // .img：4
    char        input_file_name[ 129 ] = { '\0' };
    std::string cmd_0                  = "tftp -r ";
    std::string cmd_2                  = " -l /tmp/fw.bin -g 192.168.44.1 -b 65536";
    /* parse md5 string */
    if ( sscanf( buf, "FLASH %32s %s\n", input_md5, input_file_name ) != 2 ) {
        WriteCommandByNewLine( connfd, "521 PARSE ERROR" );
        return;
    }
    std::string cmd_1;
    for ( int i = 0; input_file_name[ i ] != '\0' && i < 128; i++ ) {
        if ( input_file_name[ i ] <= 'Z' && input_file_name[ i ] >= 'A' )
            input_file_name[ i ] += 32;
        cmd_1.push_back( input_file_name[ i ] );
    }
    // fprintf( stderr, "[Flash] input_file_name: %s\n" , input_file_name );
    std::string tftp_cmd = cmd_0 + cmd_1 + cmd_2;
    // fprintf( stderr, "[Flash] input_file_name: %s\n" , tftp_cmd.c_str() );
    system( tftp_cmd.c_str() );
    /* open firmware file */
    FILE* firmware = fopen( "/tmp/fw.bin", "rb" );
    if ( firmware == NULL ) {
        WriteCommandByNewLine( connfd, "522 CANNOT OPEN FILE" );
        return;
    }

    WriteCommandByNewLine( connfd, "223 CHECKING" );

    /* calculate md5 */
    unsigned char digest[ 16 ];
    CalcMd5( digest, firmware );
    fclose( firmware );

    /* convert result hex to string */
    for ( int i = 0; i < 16; i++ )
        sprintf( result_md5 + i * 2, "%.2X", digest[ i ] );
    fprintf( stdout, "MD5 Hash of file is: %s\n", result_md5 );
    fflush( stdout );

    /* check if ota file is damaged by comparing md5 hash */
    if ( strncmp( input_md5, result_md5, 32 ) != 0 ) {
        WriteCommandByNewLine( connfd, "520 MD5 INVALID" );
        return;
    }

    // /* abort the BMS serial port to prevent ota communicate conflict */
    // uint16_t* bms_abort = NULL;
    // if ( ( bms_abort = ( uint16_t* )AttachShm( BMS_SHM_NAME, BMS_SHM_SIZE ) ) == NULL ) {
    //     std::cout << "[OTA Handler] failed to open bms abort shared memory, exit ota" << std::endl;
    //     WriteCommandByNewLine( connfd, "555 SOMETHING WRONG HAPPEND" );
    //     return;
    // }
    // *bms_abort = BMS_ABORT_REQUEST;
    // while ( *bms_abort != BMS_ABORT_FINISH ) {
    //     static int wait_iter;
    //     if ( wait_iter++ > 200 ) {  // wait for 20s
    //         std::cout << "[OTA Handler] BMS thread cannot stoped, exit ota" << std::endl;
    //         WriteCommandByNewLine( connfd, "555 SOMETHING WRONG HAPPEND" );
    //         return;
    //     }
    //     usleep( 100000 );
    // }
    // std::cout << "[OTA Handler] BMS serial port closed, start ota Flash now" << std::endl;

    /* start flashing procedure */
    int ota_output = system( "/robot/ota/ota_unpack.sh" );
    if ( ota_output == -1 || WIFEXITED( ota_output ) == 0 || WEXITSTATUS( ota_output ) != 0 ) {
        std::string ota_unpack_err;
        if ( ota_output == -1 ) {
            ota_unpack_err = std::to_string( -1 );
            fprintf( stdout, "ota_unpack.sh fail running\n" );
        }
        if ( WIFEXITED( ota_output ) == 0 ) {
            ota_unpack_err = std::to_string( 0 );
            fprintf( stdout, "ota_unpack.sh exit: %d\n", WIFEXITED( ota_output ) );
        }
        if ( WEXITSTATUS( ota_output ) != 0 ) {
            ota_unpack_err = std::to_string( WEXITSTATUS( ota_output ) );
            fprintf( stdout, "ota_unpack.sh return: %d\n", WEXITSTATUS( ota_output ) );
        }
        ota_unpack_err = "523 OTA UNPACK ERROR: " + ota_unpack_err;
        WriteCommandByNewLine( connfd, ota_unpack_err.c_str() );
        return;
    }

    /* read status and report progress, reboot */
    int         state_sent[ 4 ] = { 0, 0, 0, 0 };
    const char* state_msg[ 4 ]  = { "220 OK START FLASHING", "221 MCU FLASH OK", "222 SYSTEM FLASH OK", "300 REBOOT SYSTEM" };
    while ( true ) {  // 1
        usleep( 5000 );
        FILE* file = fopen( "/mnt/UDISK/ota_state", "rb" );
        if ( file == NULL )
            continue;
        int status = fgetc( file ) - 0x31;
        fclose( file );
        if ( status < 0 || 4 <= status ) {
            static int err_iter = 0;
            if ( err_iter % 500 == 0 ) {
                fprintf( stderr, "Invalid ota_state read: %d\n", status );
            }
            err_iter++;
            continue;
        }

        /* send progress */
        for ( int i = 0; i <= status; i++ ) {
            if ( state_sent[ i ] == 0 ) {
                WriteCommandByNewLine( connfd, state_msg[ i ] );
                state_sent[ i ] = 1;
            }
        }
        static bool report_send = false;
        if ( status == 2 && report_send == false ) {
            report_send        = true;
            std::string report = "400 " + FetchOtaReport();
            WriteCommandByNewLine( connfd, report.c_str() );
        }
        else if ( status == 3 ) {
            fprintf( stdout, "OTA finished, reboot\n" );
            break;
        }
    }

    /* reboot system */
    while ( 1 ) {
        sleep( 1 );
        system( "reboot -n -f" );
    }
}

/**
 * @brief Client query firmware version.
 *
 */
void Query( const int connfd ) {
    /* check last Flash successful */

    std::ifstream             ifs( "/var/soft_hardware_version_info.json" );
    rapidjson::IStreamWrapper isw( ifs );
    rapidjson::Document       doc;
    doc.ParseStream( isw );

    rapidjson::StringBuffer                      buffer;
    rapidjson::Writer< rapidjson::StringBuffer > writer( buffer );
    doc.Accept( writer );

    std::string jsonStr( buffer.GetString() );
    std::string sendMsg = "230 OK " + jsonStr;

    std::cout << "the msg to client is: " << sendMsg << std::endl;

    WriteCommandByNewLine( connfd, sendMsg.c_str() );
}

void RebootOperateSystem( const int connfd ) {
    WriteCommandByNewLine( connfd, "300 REBOOT SYSTEM" );
    while ( 1 ) {
        system( "reboot -n -f" );
        sleep( 1 );
    }
}

void TokenWrite( int connfd, const char* buf ) {
    char token[ 17 ] = { '\0' };
    if ( sscanf( buf, "TOKEN_WRITE %16s", token ) != 1 ) {
        std::string tmp = buf;
        std::cout << tmp << std::endl;
        WriteCommandByNewLine( connfd, "521 PARSE ERROR " );
        return;
    }
    std::string cmd        = "sec_port set ";
    std::string params     = token;
    std::string cmd_params = cmd + params;
    // std::cout << cmd_params << std::endl;
    system( cmd_params.c_str() );
    WriteCommandByNewLine( connfd, "310 OK" );
}

void TokenWriteExpand( int connfd, const char* buf ) {
    std::vector< std::string > dest;
    // split command string, saved on vector
    SplitString( buf, " ", dest );

    if ( dest.size() == 2 ) {
        std::string cmd_params = "sec_port set " + dest.at( 1 );
        system( cmd_params.c_str() );
        WriteCommandByNewLine( connfd, "310 OK" );
    }
    else if ( dest.size() == 3 ) {
        std::string cmd_params = "mikey set " + dest.at( 1 ) + " " + dest.at( 2 );  // mikey set name value
        system( cmd_params.c_str() );
        WriteCommandByNewLine( connfd, "310 OK" );
    }
    else {
        std::string tmp = buf;
        std::cout << tmp << std::endl;
        WriteCommandByNewLine( connfd, "521 PARSE ERROR " );
        return;
    }
}

void TokenRead( int connfd ) {
    FILE* stream;
    char  buff[ 17 ] = { '\0' };
    stream           = popen( "sec_port get", "r" );
    fread( buff, 1, 16, stream );
    pclose( stream );
    std::string token = buff;
    std::string msg   = "311 OK" + token;
    // std::cout << msg << std::endl;
    WriteCommandByNewLine( connfd, msg.c_str() );
}

void TokenReadExpand( int connfd, const char* buf ) {
    std::vector< std::string > dest;
    // split command string， saved on vector
    SplitString( buf, " ", dest );

    FILE* stream;                 // data stream
    char  buff[ 17 ] = { '\0' };  // maximum length

    if ( dest.size() == 1 ) {
        stream = popen( "sec_port get", "r" );
        fread( buff, 1, 16, stream );
        pclose( stream );
        std::string token = buff;
        std::string msg   = "311 OK " + token;
        WriteCommandByNewLine( connfd, msg.c_str() );
    }
    else if ( dest.size() == 2 ) {
        std::string cmd_params = "mikey get " + dest.at( 1 );  // mikey get name
        stream                 = popen( cmd_params.c_str(), "r" );
        fread( buff, 1, 16, stream );
        pclose( stream );
        std::string token = buff;
        std::string msg   = "311 OK " + token;
        WriteCommandByNewLine( connfd, msg.c_str() );
    }
    else {
        std::string tmp = buf;
        std::cout << tmp << std::endl;
        WriteCommandByNewLine( connfd, "521 PARSE ERROR " );
        return;
    }
}

void LogUpload( int connfd ) {
    if ( 0 > system( "cd /robot && tftp -l manager.log -p 192.168.44.1 -b 65536" )
         || 0 > system( "tar zcf /tmp/syslog.tar.gz /data/syslog/ /robot/manager*log && cd /tmp && tftp -l syslog.tar.gz -p 192.168.44.1 -b 65536" ) )
        WriteCommandByNewLine( connfd, "331 GET LOG FAIL" );
    else
        WriteCommandByNewLine( connfd, "330 OK" );
}

std::string FetchOtaReport() {
    rapidjson::Document doc{};
    doc.SetObject();
    auto& allocator = doc.GetAllocator();

    // Set default values

    // there is 1 in-board IMU MCU (0), 2 SPINE (1-2), 12 motors (3-14), 1 imuExt (15),
    // 1 bms (16)
    rapidjson::Value mcu_profiles( rapidjson::kObjectType );
    for ( int i = 0; i < 17; i++ ) {
        rapidjson::Value mcu_ota_state( rapidjson::kArrayType );  // default ota_state: [0, "0.0.0"]
        mcu_ota_state.PushBack( 0, allocator );
        mcu_ota_state.PushBack( "0.0.0", allocator );
        rapidjson::Value mcu_num;
        std::string      num( std::to_string( i ) );
        mcu_num.SetString( num.c_str(), num.length(), allocator );

        mcu_profiles.AddMember( mcu_num, mcu_ota_state, allocator );
    }
    doc.AddMember( "os_version", "0.0.0_0000", allocator );
    doc.AddMember( "mcu_profile", mcu_profiles, allocator );

    try {
        // fetch the new os version
        FILE*                     fp = fopen( "/mnt/exUDISK/etc/os-release", "r" );
        char                      read_buf[ 1024 ];
        rapidjson::FileReadStream is( fp, read_buf, sizeof( read_buf ) );

        rapidjson::Document os_ver_doc;
        os_ver_doc.ParseStream( is );
        if ( os_ver_doc.HasMember( "fw_arm_ver" ) && os_ver_doc[ "fw_arm_ver" ].IsString() ) {
            const char* os_ver    = os_ver_doc[ "fw_arm_ver" ].GetString();
            auto        os_ver_it = doc.FindMember( "os_version" );
            os_ver_it->value.SetString( os_ver, allocator );
        }
        else {
            std::cout << "[FetchOtaReport] exUDISK os-release file broken!" << std::endl;
        }

        // fetch the mcu report
        std::ifstream spine_motor_f( "/mnt/UDISK/ota_report/spineMotor_ota_report.txt" );
        std::ifstream imuExt_f( "/mnt/UDISK/ota_report/imuExt_ota_report.txt" );
        std::ifstream bms_f( "/mnt/UDISK/ota_report/bms_ota_report.txt" );
        // SpineMotor
        int         mcu_num = -1, ota_state = -1;
        std::string version_str;
        auto&       mcu_profile_obj = doc.FindMember( "mcu_profile" )->value;
        for ( int i = 0; i < 15; i++ ) {
            spine_motor_f >> mcu_num >> ota_state >> version_str;
            std::string      mcu_num_str( std::to_string( mcu_num ) );
            rapidjson::Value mcu_num_val, version_str_val;
            mcu_num_val.SetString( mcu_num_str.c_str(), mcu_num_str.length(), allocator );
            version_str_val.SetString( version_str.c_str(), version_str.length(), allocator );

            rapidjson::Value ota_state_arr( rapidjson::kArrayType );
            ota_state_arr.PushBack( ota_state, allocator );
            ota_state_arr.PushBack( version_str_val, allocator );
            if ( 0 <= mcu_num && mcu_num < 15 ) {
                auto mcu_it   = mcu_profile_obj.FindMember( mcu_num_val );
                mcu_it->value = ota_state_arr;
            }
            else {
                std::cerr << "[FetchOtaReport] expect mcu num in ota_report between [0 - 14], "
                          << "but got: " << mcu_num << std::endl;
            }
        }

        // imuExt
        imuExt_f >> mcu_num >> ota_state >> version_str;
        rapidjson::Value mcu_num_val( "15" );  // hard coded imuExt number
        rapidjson::Value version_str_val( version_str.c_str(), version_str.length() );
        rapidjson::Value imuext_ota_state_arr( rapidjson::kArrayType );
        imuext_ota_state_arr.PushBack( ota_state, allocator );
        imuext_ota_state_arr.PushBack( version_str_val, allocator );
        auto imuext_obj   = mcu_profile_obj.FindMember( mcu_num_val );
        imuext_obj->value = imuext_ota_state_arr;

        // bms
        bms_f >> mcu_num >> ota_state >> version_str;
        mcu_num_val = "16";
        version_str_val.SetString( version_str.c_str(), version_str.length() );
        rapidjson::Value bms_ota_state_arr( rapidjson::kArrayType );
        bms_ota_state_arr.PushBack( ota_state, allocator );
        bms_ota_state_arr.PushBack( version_str_val, allocator );
        auto bms_obj   = mcu_profile_obj.FindMember( mcu_num_val );
        bms_obj->value = bms_ota_state_arr;
    }
    catch ( std::exception& ex ) {
        std::cerr << "[FetchOtaReport] error while fetch ota report: " << ex.what() << std::endl;
        rapidjson::Value err_msg;
        err_msg.SetString( ex.what(), allocator );
        doc.AddMember( "error", err_msg, allocator );
    }

    std::stringstream                              ss;
    rapidjson::OStreamWrapper                      osw( ss );
    rapidjson::Writer< rapidjson::OStreamWrapper > writer( osw );
    doc.Accept( writer );
    std::cout << "[FetchOtaReport] json str: " << ss.str() << std::endl;

    return ss.str();
}

/**
 * @brief Split string into some substring by the given separator chars, saved on vector.
 *
 * @param src source string
 * @param separator split chars
 * @param dest collecter of splited string
 */
void SplitString( const std::string& src, const std::string& separator, std::vector< std::string >& dest ) {
    std::string            source = src;
    std::string            sub_string;
    std::string::size_type start = 0, index;
    dest.clear();
    do {
        index = source.find( separator, start );
        if ( index != std::string::npos ) {
            sub_string = source.substr( start, index - start );
            if ( !sub_string.empty() )
                dest.push_back( sub_string );
            start = index + separator.size();
        }
    } while ( index != std::string::npos && start != std::string::npos );

    sub_string = source.substr( start );
    if ( !sub_string.empty() )
        dest.push_back( sub_string );
}