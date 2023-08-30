#include <iostream>

#include "fork_program.hpp"

int ForkProgram( pid_t* pid, const char* name, const char* path, const char* log_path, char* const argv[] ) {

    fprintf( stdout, "Starting %s in %s ...\ncheck %s for logs\n", name, path, log_path );
    fflush( stdout );
    *pid = fork();

    fprintf( stdout, "pid: %d\n", *pid );
    fflush( stdout );
    /* executed by child */
    if ( *pid == 0 ) {
        /* setup environment */
        int res = chdir( path );
        std::cout << "chdir res: " << res << std::endl;
        char env_string[ 100 ] = "";
        snprintf( env_string, 100, "LD_LIBRARY_PATH=%s", path );
        fflush( stdout );
        putenv( env_string );

        /* create log files */
        char       stdout_path[ 100 ] = "", stderr_path[ 100 ] = "", time_string[ 20 ];
        time_t     starttime = time( NULL );
        struct tm* timeinfo  = localtime( &starttime );
        strftime( time_string, sizeof( time_string ), "%Y%m%d_%H%M%S", timeinfo );
        snprintf( stdout_path, 100, "%s/%s/stdout/%s.log", log_path, name, time_string );
        snprintf( stderr_path, 100, "%s/%s/stderr/%s.log", log_path, name, time_string );
        int stdout_fd = open( stdout_path, O_WRONLY | O_CREAT, S_IRUSR | S_IWUSR );
        int stderr_fd = open( stderr_path, O_WRONLY | O_CREAT, S_IRUSR | S_IWUSR );

        if ( stdout_fd < 0 || stderr_fd < 0 ) {
            std::cout << "Cannot create log file in stdout_path: " << stdout_path << ", stderr_path: " << stderr_path << std::endl;
            /* redirect program's stdout and stderr to our custom file descriptor */
        }
        else {
            fflush( stdout );
            dup2( stdout_fd, STDOUT_FILENO );
            dup2( stderr_fd, STDERR_FILENO );
            close( stdout_fd );
            close( stderr_fd );
        }

        /* execute program */
        std::cout << "name: " << name << ", argv: " << argv[ 1 ] << std::endl;
        ;
        fflush( stdout );

        execv( name, argv );
        perror( "Cannot execute program" );
        exit( EXIT_FAILURE );

        /* executed by parent */
    }
    else if ( *pid < 0 ) { /* fork failed */
        perror( "Cannot fork a new process" );
        return -1;
    }

    return 0;
}
