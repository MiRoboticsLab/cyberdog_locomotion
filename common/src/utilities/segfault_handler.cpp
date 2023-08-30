#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <execinfo.h>
#include <unistd.h>

#include "utilities/segfault_handler.hpp"
#include "utilities/utilities_print.hpp"

static char* error_message_buffer;

/**
 * @brief Called when has segfault. Prints stack trace, flushes output, and sends error code to simulator
 *
 * @param sig signal (should be SIGSEGV = 11)
 */
static void SegfaultHandler( int sig ) {
    void* stack_frames[ 200 ];
    int   size = backtrace( stack_frames, 200 );
    FprintfColor( PrintColor::kRed, stderr, "[Segfault] Crash, caught %d (%s)\n", sig, strsignal( sig ) );
    backtrace_symbols_fd( stack_frames, size, STDERR_FILENO );

    fflush( stderr );
    fflush( stdout );

    if ( error_message_buffer )
        strcpy( error_message_buffer, "[Segfault] Check the robot controller output for more information." );
    exit( 1 );
}

/**
 * @brief Called when has sigint. Prints the error message.
 *
 * @param sig signal (should be SIGINT)
 */
static void SigintHandler( int sig ) {
    if ( sig == SIGINT && error_message_buffer ) {
        strcpy( error_message_buffer, "[Segfault] Robot program has been killed by SIGINT" );
    }
    exit( 0 );
}

/**
 * @brief This will register signal and execute corresponding progress, so that the simulator can provide a reasonable error message for why the
 * robot code disappears.
 *
 * @param error_message char pointer
 */
void InstallSegfaultHandler( char* error_message ) {
    // Register signal and connected to handler functions
    signal( SIGSEGV, SegfaultHandler );
    signal( SIGINT, SigintHandler );
    error_message_buffer = error_message;
}