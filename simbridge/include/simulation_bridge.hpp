#ifndef SIMULATION_BRIDGE_HPP_
#define SIMULATION_BRIDGE_HPP_

#include <lcm/lcm-cpp.hpp>
#include <thread>

#include "parameters/robot_parameters.hpp"
#include "simulator_message.hpp"
#include "utilities/periodic_task.hpp"
#include "shared_memory.hpp"
#include "control_flags.hpp"
#include "command_interface/command_interface.hpp"
#include "robot_controller.hpp"
#include "robot_runner_Interface.hpp"
#include "header/lcm_type/file_recv_lcmt.hpp"
#include "header/lcm_type/file_send_lcmt.hpp"
#include "header/lcm_type/motion_control_request_lcmt.hpp"
#include "header/lcm_type/robot_control_cmd_lcmt.hpp"
#include "header/lcm_type/trajectory_command_lcmt.hpp"
#include "header/lcm_type/motor_ctrl_lcmt.hpp"

/**
 * @brief The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the HardwareBridge.
 * 
 */
class SimulationBridge {
public:
    explicit SimulationBridge( RobotType robot, RobotController* robot_ctrl )
        : robot_( robot ), lcm_( "udpm://239.255.76.67:7671?ttl=255" ), cyberdog_lcm_( "udpm://239.255.76.67:7671?ttl=255" ), motion_list_lcm_( "udpm://239.255.76.67:7671?ttl=255" ),
          user_gait_file_lcm_( "udpm://239.255.76.67:7671?ttl=255" ), user_gait_file_responce_lcm_( "udpm://239.255.76.67:7671?ttl=255" ), motor_control_lcm_( "udpm://239.255.76.67:7667?ttl=255" ) {
        fake_task_manager_ = new PeriodicTaskManager();
        // robot_runner_ = new RobotRunner(robot_ctrl, fake_task_manager_, 0, "robot-task");
        robot_runner_ = new RobotRunnerInterface( robot_ctrl, fake_task_manager_, 0, "robot-task" );
        user_params_  = robot_ctrl->GetUserControlParameters();
        robot_params_ = new RobotControlParameters();
        lcm_.subscribe( "robot_control_cmd", &SimulationBridge::LcmCmdCallback, this );
        user_gait_file_lcm_.subscribe( "user_gait_file", &SimulationBridge::UserGaitFileCallback, this );
        cyberdog_lcm_.subscribe( LCM_CMD_CHANNEL_NAME, &SimulationBridge::CyberdogLcmCmdCallback, this );
        motion_list_lcm_.subscribe( "motion-list", &SimulationBridge::LcmMotionCallback, this );
        motor_control_lcm_.subscribe( "motor_ctrl", &SimulationBridge::LcmMotorCtrlCallback, this );
    }
    void Run();
    void HandleControlParameters();
    void RunRobotControl();
    ~SimulationBridge() {
        delete fake_task_manager_;
        delete robot_runner_;
        delete robot_params_;
    }

private:
    void LcmCmdCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const robot_control_cmd_lcmt* msg );
    void CyberdogLcmCmdCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const motion_control_request_lcmt* msg );
    void LcmMotionCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const trajectory_command_lcmt* msg );
    void UserGaitFileCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const file_send_lcmt* msg );
    void LcmMotorCtrlCallback( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const motor_ctrl_lcmt* msg );

    bool                 first_controller_run_ = true;
    PeriodicTaskManager* fake_task_manager_    = nullptr;
    RobotType            robot_;
    lcm::LCM             lcm_;
    lcm::LCM             cyberdog_lcm_;
    lcm::LCM             motion_list_lcm_;
    lcm::LCM             user_gait_file_lcm_;
    lcm::LCM             user_gait_file_responce_lcm_;
    lcm::LCM             motor_control_lcm_;
    std::thread          interface_lcm_thread_cmd_;
    std::thread          interface_lcm_thread_cyberdog_cmd_;
    std::thread          interface_lcm_thread_motion_list_;
    std::thread          interface_lcm_thread_usergait_;
    std::thread          interface_motor_control_thread_;

    void                 HandleInterfaceLCM_cmd();
    void                 HandleInterfaceLCM_cyberdog_cmd();
    void                 HandleInterfaceLCM_motion_list();
    void                 HandleInterfaceLCM_usergait_file();
    void                 HandleMotorCtrlLcmThread();
    volatile bool        interface_lcm_quit_ = false;

    // RobotRunner*                         robot_runner_ = nullptr;
    RobotRunnerInterface*                  robot_runner_ = nullptr;
    SimulatorMode                          sim_mode_;
    SharedMemoryObject< SimulatorMessage > shared_memory_;
    RobotControlParameters*                robot_params_ = nullptr;
    ControlParameters*                     user_params_  = nullptr;
    u64                                    iterations_  = 0;

    CommandInterface cmd_interface_;
};

#endif  // SIMULATION_BRIDGE_HPP_
