#ifndef COMMAND_INTERFACE_HPP_
#define COMMAND_INTERFACE_HPP_

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "cpptoml.h"
#include "dynamics/cyberdog.hpp"
#include "dynamics/quadruped.hpp"
#include "utilities/timer.hpp"
#include "command_interface/gamepad_command.hpp"
#include "command_interface/rc_command.hpp"
#include "header/lcm_type/file_recv_lcmt.hpp"
#include "header/lcm_type/file_send_lcmt.hpp"
#include "header/lcm_type/gamepad_lcmt.hpp"
#include "header/lcm_type/motion_control_request_lcmt.hpp"
#include "header/lcm_type/motor_ctrl_lcmt.hpp"
#include "header/lcm_type/robot_control_cmd_lcmt.hpp"
#include "header/lcm_type/trajectory_command_lcmt.hpp"

#define CMD_TIMEOUT 0.5  // cmd timeout time in second

/**
 * @brief data structure of MotionControlCommand
 * 
 */
struct MotionControlCommand {
    int8_t  mode;
    int8_t  gait_id;
    int8_t  contact;
    int8_t  cmd_source;
    float   vel_des[ 3 ];     // x y yaw
    float   rpy_des[ 3 ];     // roll pitch yaw
    float   pos_des[ 3 ];     // x y z
    float   ctrl_point[ 3 ];  // pose ctrl point
    float   acc_des[ 6 ];
    float   foot_pose[ 6 ];  // front/back foot pose x,y,z
    float   step_height[ 2 ];
    int32_t value;
    int32_t duration;

    int16_t     motion_id;
    int16_t     motion_trigger;
    int16_t     motion_process_bar;
    std::string user_gait_file;

    float           cmd_time_delay;
    motor_ctrl_lcmt motor_ctrl;
};

typedef enum {
    kGamepadCmd      = 7,
    kRcCmd           = 8,
    kCyberdogLcmCmd  = 9,
    kCyberdog2LcmCmd = 10,
    kMotorCmd        = 11,
} CmdSource;

/**
 * @brief Unified command interface for different input sources
 * 
 */
class CommandInterface {
public:

    CommandInterface();

    void PrepareCmd( int use_rc, long int* control_mode, long int* gait_id, const RobotType& robotType );

    const MotionControlCommand& GetCommand() {
        return command_;
    }
    void ZeroCmd( MotionControlCommand& cmd );

    void SetFsmState( int fsm_state ) {
        current_fsm_state_ = fsm_state;
    }
    void SetSwitchStaus( int switch_status ) {
        current_switch_status_ = switch_status;
    }
    void SetCmdQueueClearFlag( int mode ) {
        cmd_queue_clear_flag_ = mode;
    }

    
     // Command Process function group, work in bridge layer
    void ProcessGamepadCommand( const GamepadCommand& gamepad_cmd );
    void ProcessGamepadCommand( const gamepad_lcmt* gamepad_cmd );
    void ProcessRcCommand( const RcCommand* rc_cmd );
    void ProcessRcUdpCommand( const RcCommand* rc_cmd );
    void ProcessLcmCommand( const robot_control_cmd_lcmt* lcm_cmd );
    void ProcessCyberdogLcmCommand( const motion_control_request_lcmt* lcm_cmd );
    void ProcessLcmMotionCommand( const trajectory_command_lcmt* lcm_cmd );
    void ProcessLcmMotorCtrlCommand( const motor_ctrl_lcmt* ctrl_cmd );
    int  ProcessUserGaitFile( const file_send_lcmt* user_gait_msg );
    int  GetSpeedOffsetTrigger() {
        return speed_offset_rc_trigger_;
    }

private:
    void Gamepad2Cmd( long int* control_mode, long int* gait_id, const RobotType& robotType );
    void Rc2Cmd( const RobotType& robotType );
    void MotorLcm2Cmd();
    void Lcm2Cmd();
    void CyberdogLcm2Cmd();
    void Default2Cmd( int control_mode, int cmpc_gait );
    int  MotionListTxtRead( std::string file_name );
    int  MotionListTomlRead( std::string file_name );
    int  MotionListCppTomlRead( std::string file_name );
    int  MotionListFullTomlRead( std::string file_string );
    int  MotionListFullCppTomlRead( std::string file_string );
    void Pattern2Mode( int pattern, int order, int8_t& control_mode, int8_t& gait_num );

    template < size_t n > void CpptomlGetVector( const std::shared_ptr< cpptoml::table >& table, const std::string& name, float ( &value )[ n ] ) {
        auto tmp = table->get_array_of< double >( name );
        if ( tmp && tmp->size() == n ) {
            std::copy( tmp->begin(), tmp->end(), value );
        }
    }

    MotionControlCommand               command_;
    MotionControlCommand               cmd_cur_, cmd_tmp_;
    MotionControlCommand               pose_empty_cmd_, qp_empty_cmd_, loco_empty_cmd_, loco_stand_cmd_, sit_down_trans_cmd_;
    std::queue< MotionControlCommand > cmd_list_;
    std::vector< std::string >         Split( const std::string& str, const std::string& pattern );
    std::vector< std::string >         Get( const std::string& str, const std::string& target );

    GamepadCommand gamepad_cmd_;
    Timer          gamepad_timer_;
    int16_t        motion_id_;
    int16_t        motion_trigger_;
    int16_t        cmd_queue_clear_flag_;

    RcCommand                   rc_cmd_;
    RcCommand                   rc_cmd_old_;
    Timer                       rc_timer_;
    Timer                       rc_udp_timer_;
    int32_t                     interface_iter_;
    int16_t                     life_count_;
    int16_t                     motion_list_step_;
    int16_t                     motion_list_size_;
    std::string                 user_gait_file_;
    std::string                 user_gait_list_file_;
    motion_control_request_lcmt cyberdog_lcm_cmd_;
    robot_control_cmd_lcmt      lcm_cmd_;
    motor_ctrl_lcmt             motor_ctrl_cmd_;
    Timer                       cyberdog_lcm_timer_;
    Timer                       lcm_timer_;
    Timer                       lcm_duration_timer_;
    Timer                       MotorCtrl_LCM_timer_;
    bool                        MotorCtrl_mode_flag_;
    int                         log_count_;
    int                         switch_tri_flag_;
    bool                        speed_offset_rc_flag_;
    int                         speed_offset_rc_trigger_;
    CmdSource                   last_mode_ = kRcCmd;
    int                         current_fsm_state_;
    int                         current_switch_status_;
    u64                         speed_offset_count_;
};

#endif  // COMMAND_INTERFACE_HPP_
