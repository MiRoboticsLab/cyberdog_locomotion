#include <iostream>

#include "cyberdog_controller.hpp"
#include "sub_main.hpp"

/**
 * @brief The main function parses command line arguments and starts the appropriate
 * driver.
 *
 * @param argc number of parameters
 * @param argv parameters
 * @return int
 */
int main( int argc, char** argv ) {
    // printf date and time
    time_t t = time( 0 );
    char   dateTime[ 64 ];
    strftime( dateTime, sizeof( dateTime ), "%Y/%m/%d/ %X %A", localtime( &t ) );

    std::cout << "-----------------------------------------------------------------------------" << std::endl;
    std::cout << "[Main Controller] Begin to execute, and the current time is: " << dateTime << std::endl;
    std::cout << "-----------------------------------------------------------------------------" << std::endl;

    SubMain( argc, argv, new CyberdogController(), true );
    return 0;
}
