#include "handle_client_initial.hpp"
#include "handle_leg_test_client.hpp"
#include "handle_ota_client.hpp"
#include "manager_parameters.hpp"
#include "raw_socket.hpp"

/**
 * @brief Close connection on receiving SIGPIPE
 * 
 */
void EndChild( int signo ) {
    ( void )signo;
    fprintf( stderr, "Client on PID %d connection broken.\n", getpid() );
    exit( EXIT_FAILURE );
}

void HandleClientInitialize( const int connfd, const pid_t extprog_pid ) {
    /* capture SIGPIPE signal */
    signal( SIGPIPE, EndChild );

    /* HandShake confirm client */
    int client_type = HandShake( connfd, BUF_LEN );
    if ( client_type < 0 )
        fprintf( stderr, "Client on PID %d HandShake failed due to invalid response. Connection closed\n", getpid() );

    /* ota client */
    else if ( client_type == 1 )
        HandleOtaClient( connfd, extprog_pid ); /* never return */

    /* leg Test client */
    else if ( client_type == 2 ) {
        std::string test_json_fn( exe_path + TEST_EXE_COMMAND_JSON_PATH );
        HandleLegTestClient( connfd, extprog_pid, test_json_fn.c_str() ); /* never return */
    }

    exit( EXIT_FAILURE ); /* should not reach here */
}
