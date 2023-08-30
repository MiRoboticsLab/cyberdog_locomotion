#include <cstdlib>
#include <iostream>
#include <memory>

#include "document.h"      // rapidjson's DOM-style API
#include "prettywriter.h"  // for stringify JSON

#include "fork_para_conf_json.hpp"
#include "fork_program.hpp"
#include "handle_leg_test_client.hpp"
#include "manager_parameters.hpp"
#include "parse_json_on_execute_command.hpp"
#include "raw_socket.hpp"
#include "utilities/shm_help.hpp"

extern std::vector< pid_t >            childs_pid;
extern std::vector< ForkParaConfJson > g_fork_json_datas;
bool                                   g_restart_flag = false;

void HandleLegTestClient( const int connfd, const pid_t extprog_pid, const char* file_name ) {

    char                buf[ BUF_LEN + 1 ];
    rapidjson::Document json_received;

    // parse json data from json file on execute command
    auto ptr_parse_json = std::make_shared< ParseJsonOnExecuteCommand >();
    auto json_datas     = ptr_parse_json->ParseJsonData( file_name );

    /* reset signal handler to system default (for later use of waitpid) */
    signal( SIGCHLD, SIG_DFL );

    /* process commands */
    while ( 1 ) {
        ReadCommandByLength( connfd, buf, BUF_LEN );
        pid_t testprog_pid = -1;

        /* decode json string */
        if ( json_received.ParseInsitu( buf ).HasParseError() ) {
            WriteCommandByLength( connfd, "{\"code\": -1, \"message\": \"not in correct json format\"}" );
            continue;
        }

        /* check parameters */
        if ( !json_received.IsObject() || !json_received.HasMember( "cmd" ) || !json_received[ "cmd" ].IsString() || !json_received.HasMember( "data" ) || !json_received[ "data" ].IsArray() ) {
            WriteCommandByLength( connfd, "{\"code\": -2, \"message\": \"json parameters not correct\"}" );
            continue;
        }

        /* read command */
        std::string command( json_received[ "cmd" ].GetString() );

        /* find Test by json cmd */
        auto it = std::find_if( json_datas.begin(), json_datas.end(), [ &command ]( const ExecuteCommandJson& t ) { return t.cmd == command; } );

        /* Test found */
        if ( it != json_datas.end() ) {
            if ( it->pc_cmd == "2" ) {
                /* kill cyberdog_control process */
                if ( extprog_pid != -1 ) {
                    if ( kill( extprog_pid, SIGKILL ) < 0 ) {
                        sleep( 1 );
                        if ( kill( extprog_pid, SIGKILL ) < 0 ) {
                            if ( errno != ESRCH ) {
                                WriteCommandByLength( connfd, "{\"code\": -10, \"message\": \"cannot kill cyberdog_control process, quitting\"}" );
                                exit( EXIT_FAILURE );
                            }
                        }
                    }
                    auto pid_iter = std::find( childs_pid.begin(), childs_pid.end(), extprog_pid );
                    if ( pid_iter != childs_pid.end() ) {
                        childs_pid.erase( pid_iter );
                    }
                    else {
                        std::cerr << "extprog_pid: " << extprog_pid << " does not exist in childs_pid, exit" << std::endl;
                    }
                }

                g_restart_flag = true;
                std::cout << "the value of pc_cmd is " << it->pc_cmd << " ,cyberdog_control will be killed and will be forked for meet certain conditions!" << std::endl;
            }
            else if ( it->pc_cmd == "1" ) {
                /* kill cyberdog_control process */
                if ( extprog_pid != -1 ) {
                    if ( kill( extprog_pid, SIGKILL ) < 0 ) {
                        sleep( 1 );
                        if ( kill( extprog_pid, SIGKILL ) < 0 ) {
                            if ( errno != ESRCH ) {
                                WriteCommandByLength( connfd, "{\"code\": -10, \"message\": \"cannot kill cyberdog_control process, quitting\"}" );
                                exit( EXIT_FAILURE );
                            }
                        }
                    }
                    auto pid_iter = std::find( childs_pid.begin(), childs_pid.end(), extprog_pid );
                    if ( pid_iter != childs_pid.end() ) {
                        childs_pid.erase( pid_iter );
                    }
                    else {
                        std::cerr << "extprog_pid: " << extprog_pid << " does not exist in childs_pid, exit" << std::endl;
                    }
                }

                std::cout << "the value of pc_cmd is " << it->pc_cmd << " ,cyberdog_control will be killed and not will be automatic forked!" << std::endl;
            }
            else {
                std::cout << "the value of pc_cmd is " << it->pc_cmd << " ,and cyberdog_control will not be killed!" << std::endl;
            }

            /* setup a reference for array object */
            const rapidjson::Value& array = json_received[ "data" ];

            /* setup arguments list */
            const char** test_prog_argv = new const char*[ array.Size() + 3 ];

            /* executable name */
            test_prog_argv[ 0 ] = it->name.c_str();

            /* fd */
            int fd[ 2 ];
            pipe( fd );
            char fd_str[ 20 ];
            snprintf( fd_str, 20, "%d", fd[ 1 ] );
            test_prog_argv[ 1 ] = fd_str;

            /* add remaining arguments from json */
            for ( rapidjson::SizeType i = 0; i < array.Size(); i++ )
                test_prog_argv[ i + 2 ] = array[ i ].GetString();

            /* add NULL terminator */
            test_prog_argv[ array.Size() + 2 ] = NULL;

            /* fork program */
            std::string path = ( it->path.empty() ? ( exe_path + "/robot-software/build/" ) : ( exe_path + it->path ) );
            if ( ForkProgram( &testprog_pid, it->name.c_str(), path.c_str(), TEST_PROG_LOG_PATH, ( char* const* )test_prog_argv ) < 0 ) {
                WriteCommandByLength( connfd, "{\"code\": -5, \"message\": \"cannot fork Test program\"}" ); /* fork failed */

                /* fork success, wait for Test program ends */
            }
            else {
                close( fd[ 1 ] );
                waitpid( testprog_pid, NULL, 0 );
                FILE* result_f = fdopen( fd[ 0 ], "r" );
                char  result_line[ 2048 ];
                fgets( result_line, sizeof( result_line ), result_f );
                std::string responce = "{\"cmd\": \"" + it->cmd + "\", \"result\": \"" + result_line + "\"}";

                std::cout << "g_restart_flag: " << g_restart_flag << "; result_line: " << result_line << std::endl;

                if ( g_restart_flag == true and std::string( result_line ).substr( 0, 2 ) == "OK" ) {
                    /*fork all programs in json file*/
                    // pid_t mit_ctrl_pid;
                    for ( auto iter = g_fork_json_datas.begin(); iter != g_fork_json_datas.end(); iter++ ) {
                        if ( iter->name == "cyberdog_control" ) {  // only fork cyberdog_control
                            pid_t       exe_pid        = -1;
                            std::string exe_real_path  = exe_path + iter->object_path;
                            size_t      paraNum        = iter->para_values.size();
                            const char* exe_argv[ 50 ] = {};
                            exe_argv[ 0 ]              = iter->name.c_str();
                            exe_argv[ paraNum + 1 ]    = NULL;

                            for ( size_t argv_num = 0; argv_num < paraNum; argv_num++ ) {
                                exe_argv[ argv_num + 1 ] = iter->para_values[ argv_num ].c_str();
                            }

                            if ( ForkProgram( &exe_pid, iter->name.c_str(), exe_real_path.c_str(), iter->log_path.c_str(), ( char* const* )exe_argv ) < 0 ) {
                                fprintf( stderr, "\nfailed to launch %s, quitting...\n", iter->name.c_str() );
                                exit( EXIT_FAILURE );
                            }

                            childs_pid.push_back( exe_pid );
                            usleep( 1000 * 1 );
                        }
                    }

                    g_restart_flag = false;
                    std::cout << "cyberdog_control meet certain conditions,and the progress will be forked!" << std::endl;
                }

                WriteCommandByNewLine( connfd, responce.c_str() );
                usleep( 1000 * 1500 );
                close( connfd );
            }

            delete[] test_prog_argv;

            /* Test not found, inv+alid cmd */
        }
        else {
            WriteCommandByLength( connfd, "{\"code\": -3, \"message\": \"cmd value not valid\"}" );
        }
    }

    exit( EXIT_FAILURE ); /* should not reach here */
}
