#ifndef SUB_MAIN_HPP_
#define SUB_MAIN_HPP_

#include "robot_controller.hpp"

extern MasterConfig gMasterConfig;

/**
 * @brief Function which should be called in main to start your robot control code
 *
 * @param argc number of parameters
 * @param argv parameters
 * @param ctrl obot controller instance
 * @param with_imu if the controller need IMU, it should be setted as true, else false,
 *                 if false is set, the IMU thread with not start, and IMU calib param
 *                 will not load ether.
 * @return int
 */
int SubMain( int argc, char** argv, RobotController* ctrl, bool with_imu = false );

#endif  // SUB_MAIN_HPP_
