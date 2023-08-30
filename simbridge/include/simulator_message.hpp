#ifndef SIMULATOR_MESSAGE_HPP_
#define SIMULATOR_MESSAGE_HPP_

#include "command_interface/gamepad_command.hpp"
#include "control_parameters/control_parameter_interface.hpp"
#include "shared_memory.hpp"
#include "sim_utilities/imu_types.hpp"
#include "sim_utilities/spine_board.hpp"
#include "sim_utilities/visualization_data.hpp"

/**
 * @brief These messsages contain all data that is exchanged between the robot program
 * and the simulator using shared memory. This is basically everything except
 * for debugging logs, which are handled by LCM instead.
 *
 */

/**
 * @brief The mode for the simulator.
 */
enum class SimulatorMode {
    kRunContorlParameters,  // don't run the robot controller, just process
                            // Control Parameters
    kRunContorller,         // run the robot controller
    kDoNothing,             // just to check connection
    kExit                   // quit!
};

/**
 * @brief A plain message from the simulator to the robot.
 *
 */
struct SimulatorToRobotMessage {
    GamepadCommand gamepadCommand;  // joystick
    RobotType      robotType;       // which robot the simulator thinks we are simulating

    // imu data
    VectorNavData          vectorNav;
    CheaterState< double > cheater_state;

    // leg data
    SpiData spiData;
    // TODO: remove tiboard related later
    // TiBoardData tiBoardData[4];
    ControlParameterRequest controlParameterRequest;

    SimulatorMode mode;
};

/**
 * @brief A plain message from the robot to the simulator.
 */
struct RobotToSimulatorMessage {
    RobotType  robotType;
    SpiCommand spiCommand;
    // TiBoardCommand tiBoardCommand[4];

    VisualizationData        visualizationData;
    Cyberdog2Visualization   mainCyberdog2Visualization;
    ControlParameterResponse controlParameterResponse;

    char errorMessage[ 2056 ];
};

/**
 * @brief All the data shared between the robot and the simulator.
 *
 */
struct SimulatorMessage {
    RobotToSimulatorMessage robotToSim;
    SimulatorToRobotMessage simToRobot;
};

#endif  // SIMULATOR_MESSAGE_HPP_
