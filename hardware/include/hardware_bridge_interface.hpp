#ifndef HARDWARE_BRIDGE_INTERFACE_HPP_
#define HARDWARE_BRIDGE_INTERFACE_HPP_

#include "robot_controller.hpp"
#include "robot_runner_Interface.hpp"

class Cyberdog2HardwareBridge;

class HardwareBridgeInterface {
public:
    HardwareBridgeInterface( RobotController* rc, bool load_parameters_from_file, const RobotType& robot_type );
    ~HardwareBridgeInterface();

    void Run();
    void SetWithImu( bool with_imu );

private:
    Cyberdog2HardwareBridge* hb_;
};

#endif  // HARDWARE_BRIDGE_INTERFACE_HPP_
