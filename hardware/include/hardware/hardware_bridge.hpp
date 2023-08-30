#ifndef HARDWARE_BRIDGE_HPP_
#define HARDWARE_BRIDGE_HPP_

#include <string>

#include <lcm/lcm-cpp.hpp>

#include "command_interface/command_interface.hpp"
#include "control_parameters/control_parameter_interface.hpp"
#include "header/lcm_type/visualization_lcmt.hpp"
#include "header/lcm_type/control_parameter_request_lcmt.hpp"
#include "header/lcm_type/control_parameter_respones_lcmt.hpp"
#include "header/lcm_type/gamepad_lcmt.hpp"
#include "header/lcm_type/microstrain_lcmt.hpp"
#include "header/lcm_type/motor_ctrl_lcmt.hpp"
#include "header/lcm_type/robot_control_cmd_lcmt.hpp"
#include "header/lcm_type/trajectory_command_lcmt.hpp"
#include "math/orientation_tools.hpp"
#include "parameters/calibrate_parameters.hpp"
#include "parameters/robot_parameters.hpp"
#include "robot_controller.hpp"
#include "robot_runner_Interface.hpp"
#include "rt/rt_rc_interface.h"
#include "rt/spi_handler.hpp"
#include "rt/spi_handler_for_imu.hpp"
#include "sim_utilities/imu_types.hpp"
#include "sim_utilities/visualization_data.hpp"
#include "utilities/complementary_filter.hpp"
#include "utilities/dog_toolkit.hpp"
#include "utilities/filters.hpp"
#include "utilities/mahony_filter.hpp"
#include "utilities/periodic_task.hpp"

#ifdef BUILD_CYBERDOG2
#define USE_EXTERNAL_IMU
#endif

union Byte2Double {
    double value;
    char   bytes[ sizeof( double ) ];
};

/**
 * @brief Interface between robot and hardware.
 *
 */
class HardwareBridge {
public:
    /**
     * @brief Construct a new Hardware Bridge object.
     *
     * @param robot_ctrl Robot Controller
     */
    HardwareBridge( RobotController* robot_ctrl )
        : status_task_( &task_manager_, 0.5f ), interface_lcm_( GetLcmUrl( 255 ) ), interface_lcm_r_( GetLcmUrlWithPort( 7668, 255 ) ), recv_from_ros_lcm_( GetLcmUrlWithPort( 7671, 255 ) ),
          recv_from_cyberdog_ros_lcm_( GetLcmUrlWithPort( 7671, 255 ) ), visualization_lcm_( GetLcmUrl( 255 ) ), lcm_( GetLcmUrl( 255 ) ), motion_list_lcm_( GetLcmUrlWithPort( 7671, 255 ) ),
          motor_ctrl_lcm_( GetLcmUrlWithPort( 7667, 255 ) ), user_gait_file_lcm_( GetLcmUrlWithPort( 7671, 255 ) ), user_gait_file_responce_lcm_( GetLcmUrlWithPort( 7671, 255 ) ), spi_command_() {
        controller_              = robot_ctrl;
        user_control_parameters_ = robot_ctrl->GetUserControlParameters();
    }

    /**
     * @brief Destroy the Hardware Bridge object.
     *
     */
    ~HardwareBridge() {
        delete robot_runner_;
    }

    void PrefaultStack();
    void SetupScheduler();
    void InitError( const char* reason, bool print_errno = false );
    void InitCommon();
    void HandleGamepadLcm( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const gamepad_lcmt* msg );

    void HandleInterfaceLcm();
    void HandleInterfaceLcmR();
    void HandleInterfaceLcmCmd();
    void HandleInterfaceLcmCyberdogCmd();
    void HandleInterfaceLcmMotionList();
    void HandleInterfaceLcmUsergaitFile();
    void HandleMotorCtrlLcmThread();
    void HandleControlParameter( const lcm::ReceiveBuffer* rbuf, const std::string& chan, const control_parameter_request_lcmt* msg );

    void PublishVisualizationLcm();
    void PublishDebugLcm();
    void RunSbus();
    void LcmCmdCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const robot_control_cmd_lcmt* msg );
    void CyberdogLcmCmdCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const motion_control_request_lcmt* msg );
    void LcmMotionCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const trajectory_command_lcmt* msg );
    void LcmMotorCtrlCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const motor_ctrl_lcmt* msg );
    void UserGaitFileCallback( const lcm::ReceiveBuffer* buf, const std::string& channel, const file_send_lcmt* msg );
    bool GetParaResponseLcm( const control_parameter_request_lcmt* ptr_msg, ControlParameter& control_para, bool is_set_flag = true );

protected:
    PeriodicTaskManager             task_manager_;
    PrintTaskStatus                 status_task_;
    VisualizationData               visualization_data_;
    Cyberdog2Visualization          cyberdog2_visualization_;
    lcm::LCM                        interface_lcm_;
    lcm::LCM                        interface_lcm_r_;
    lcm::LCM                        recv_from_ros_lcm_;
    lcm::LCM                        recv_from_cyberdog_ros_lcm_;
    lcm::LCM                        visualization_lcm_;
    lcm::LCM                        lcm_;
    lcm::LCM                        motion_list_lcm_;
    lcm::LCM                        motor_ctrl_lcm_;
    lcm::LCM                        user_gait_file_lcm_;
    lcm::LCM                        user_gait_file_responce_lcm_;
    control_parameter_respones_lcmt parameter_response_lcmt_;
    SpiData                         spi_data_;
    SpiCommand                      spi_command_;

    RobotRunnerInterface*  robot_runner_ = nullptr;
    RobotControlParameters robot_params_;
    u64                    iterations_ = 0;
    std::thread            interface_lcm_thread_;
    std::thread            interface_lcm_thread_r_;
    std::thread            interface_lcm_thread_cyberdog_cmd_;
    std::thread            interface_lcm_thread_cmd_;
    std::thread            interface_lcm_thread_motion_list_;
    std::thread            interface_lcm_thread_usergait_;
    std::thread            interface_motor_ctrl_thread_;

    volatile bool      interface_lcm_quit_      = false;
    RobotController*   controller_              = nullptr;
    ControlParameters* user_control_parameters_ = nullptr;

    int                    port_;
    IMUCalibrateParameters imu_param_;
    PosCalibrateParameters pos_param_;

    CommandInterface         cmd_interface_;
    SpeedCalibrateParameters speed_param_;
};

/**
 * @brief Interface between robot and hardware specialized for Cyberdog2.
 *
 */
class Cyberdog2HardwareBridge : public HardwareBridge {
public:
    Cyberdog2HardwareBridge( RobotController* rc, bool load_parameters_from_file, const RobotType& robot_type );
    void RunSpi();
    void RunImuSerial();
    void RunSpeedConfig();
    void InitHardware();
    void Run();
    void RunMicrostrain();
    void Abort( const std::string& reason );
    void Abort( const char* reason );
    void SetWithImu( bool with_imu ) {
        with_imu_ = with_imu;
    }
#ifdef USE_EXTERNAL_IMU
    void RunExternalImuSerial();

    void RunSpiForImu();
#endif

private:
    void FilterImu( Vec3< float > acc_correct, Vec3< float > gyro_correct );

    bool GetImuByteParams( Serial& serial_port, unsigned char* rec_data, unsigned char start_pos, unsigned char size );

    bool GetImuParamsFromMcu( Vec3< float >& a_bias, Vec3< float >& g_bias, Mat3< float >& Ta_1, Mat3< float >& Tg_1, Serial& serial_port );

    void HandleInterfaceRcRec();

private:
    std::thread speed_thread_;
    std::thread interface_rc_thread_;

    int speed_offset_flag_;

    ComplementaryFilter imu_filter_;
    MahonyFilter        imu_mh_filter_;
    VectorNavData       vector_nav_data_;

    std::mutex imu_mtx_;
    lcm::LCM   spi_lcm_;
    lcm::LCM   imu_lcm_;

    bool       load_parameters_from_file_;
    SpiHandler spi_handler_;

    // IMU hardware realated
    bool             with_imu_;
    bool             is_imu_params_in_yaml_;
    std::thread      imu_thread_;
    bool             first_imu_ = true;
    microstrain_lcmt my_imu_data_;
    RobotType        robot_type_;
    SpiHandlerForImu spi_handler_for_imu_;

    RobotAppearanceType appearance_type_;

#ifdef USE_EXTERNAL_IMU
    lcm::LCM         external_imu_lcm_;
    std::thread      external_imu_thread_;
    microstrain_lcmt external_imu_data_;
    VectorNavData    external_vector_nav_data_;
#endif
    std::ofstream imu_cost_log_f_;
    bool          use_imu_by_spi_;
};

#endif  // HARDWARE_BRIDGE_HPP_
