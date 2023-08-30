#include <ctime>
#include <iomanip>
#include <iostream>

#include "Configuration.h"
#include "utilities/utilities.hpp"

/**
 * @brief Write std::string to file with given name
 *
 * @param fileName
 * @param fileData
 */
void WriteStringToFile( const std::string& fileName, const std::string& fileData ) {
    FILE* fp = fopen( fileName.c_str(), "w" );
    if ( !fp ) {
        printf( "Failed to fopen %s\n", fileName.c_str() );
        throw std::runtime_error( "Failed to open file" );
    }
    fprintf( fp, "%s", fileData.c_str() );
    fclose( fp );
}

/**
 * @brief Get the current time and return date object as a string
 *
 * @return std::string
 */
std::string GetCurrentTimeAndDate() {
    auto               t  = std::time( nullptr );
    auto               tm = *std::localtime( &t );
    std::ostringstream ss;
    ss << std::put_time( &tm, "%c" );
    return ss.str();
}

/**
 * @brief Return the path of config directory
 *
 * @return std::string
 */
std::string GetConfigDirectoryPath() {
    return std::string( THIS_COM ) + "common/config/";
}
/**
 * @brief Get the Preinstalled Motion Path For CyberDog object
 *
 * @return std::string
 */
std::string GetPreinstalledMotionPathForCyberDog() {
    return std::string( THIS_COM ) + "control/motion_list/cyberdog/preinstalled/";
}

/**
 * @brief Get the Preinstalled Motion Path For CyberDog2 object
 *
 * @return std::string
 */
std::string GetPreinstalledMotionPathForCyberDog2() {
    return std::string( THIS_COM ) + "control/motion_list/cyberdog2/preinstalled/";
}

/**
 * @brief Get the User Gait Define Path object
 *
 * @return std::string
 */
std::string GetUserGaitDefinePath() {
    return std::string( THIS_COM ) + "control/motion_list/cyberdog2/user/define/";
}

/**
 * @brief Get the User Gait Parameter Path object
 *
 * @return std::string
 */
std::string GetUserGaitParameterPath() {
    return std::string( THIS_COM ) + "control/motion_list/cyberdog2/user/parameter/";
}

/**
 * @brief Get the Nonperiodic Gait Define Path object
 * 
 * @return std::string
 */
std::string GetNonperiodicGaitDefinePath() {
    return std::string( THIS_COM ) + "control/motion_list/cyberdog2/nonperiodic/define/";
}

/**
 * @brief Get the Nonperiodic Gait Parameter Path object
 *
 * @return std::string
 */
std::string GetNonperiodicGaitParameterPath() {
    return std::string( THIS_COM ) + "control/motion_list/cyberdog2/nonperiodic/parameter/";
}

/**
 * @brief Get the Offline Trajectory Path object
 *
 * @return std::string
 */
std::string GetJumpTrajectoryPath() {
    return std::string( THIS_COM ) + "control/offline_trajectory/";
}

/**
 * @brief Get the Jump3d Trajecoty Path object
 *
 * @return std::string
 */
std::string GetJump3dTrajecotyPath() {
    return std::string( THIS_COM ) + "control/offline_trajectory/motion_lib/";
}
