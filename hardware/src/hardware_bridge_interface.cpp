#include "hardware_bridge_interface.hpp"
#include "hardware_bridge.hpp"

/**
 * @brief Construct a new Hardware Bridge Interface:: Hardware Bridge Interface object.
 *
 * @param rc Pointer to the RobotController object
 * @param load_parameters_from_file Boolean indicating whether to load parameters from a file
 * @param robot_type The type of the robot
 */
HardwareBridgeInterface::HardwareBridgeInterface( RobotController* rc, bool load_parameters_from_file, const RobotType& robot_type ) {
    hb_ = new Cyberdog2HardwareBridge( rc, load_parameters_from_file, robot_type );
}

/**
 * @brief Destroy the Hardware Bridge Interface:: Hardware Bridge Interface object.
 *
 */
HardwareBridgeInterface::~HardwareBridgeInterface() {
    delete hb_;
}

/**
 * @brief Runs the hardware bridge interface.
 *
 */
void HardwareBridgeInterface::Run() {
    hb_->Run();
}

/**
 * @brief Sets the flag indicating whether to use IMU data.
 *
 * @param with_imu Flag indicating whether to use IMU data.
 */
void HardwareBridgeInterface::SetWithImu( bool with_imu ) {
    hb_->SetWithImu( with_imu );
}
