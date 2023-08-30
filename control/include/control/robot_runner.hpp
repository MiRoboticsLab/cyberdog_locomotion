#ifndef ROBOT_RUNNER_HPP_
#define ROBOT_RUNNER_HPP_

#include "control_parameters/control_parameter_interface.hpp"
#include "controllers/state_estimator_container.hpp"
#include "sim_utilities/imu_types.hpp"
#include "parameters/robot_parameters.hpp"
// #include "rt/rt_rc_interface.h"
#include "controllers/leg_controller.hpp"
#include "dynamics/quadruped.hpp"
#include "command_interface/command_interface.hpp"

#include "sim_utilities/visualization_data.hpp"
#include "utilities/periodic_task.hpp"
#include "command_interface/gamepad_command.hpp"
#include "fsm_states/robot_current_state.hpp"
#include "header/lcm_type/visualization_lcmt.hpp"
#include "header/lcm_type/danger_states_lcmt.hpp"
#include "header/lcm_type/localization_lcmt.hpp"
#include "header/lcm_type/motion_control_request_lcmt.hpp"
#include "header/lcm_type/motion_control_response_lcmt.hpp"
#include "header/lcm_type/robot_control_response_lcmt.hpp"
#include "header/lcm_type/state_estimator_lcmt.hpp"
#include "robot_controller.hpp"
#include <lcm/lcm-cpp.hpp>

class RobotRunner : public PeriodicTask {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RobotRunner( RobotController*, PeriodicTaskManager*, float, std::string );
    using PeriodicTask::PeriodicTask;
    void InitTask() override;
    void RunTask() override;
    void CleanUp() override;

    // Initialize the state estimator with default no cheaterMode
    void InitializeStateEstimator( bool cheaterMode = false );
    virtual ~RobotRunner();

    RobotController* robot_ctrl_;

    CommandInterface*       cmd_interface_;
    RobotType               robot_type_;
    RobotAppearanceType     robot_appearance_type_;
    VectorNavData*          vector_nav_data_;
    CheaterState< double >* cheater_state_;
    SpiData*                spi_data_;
    SpiCommand*             spi_command_;
    RobotControlParameters* control_parameters_;
    VisualizationData*      visualization_data_;
    Cyberdog2Visualization*   cyberdog2_main_visualization_;
    int8_t*                 bms_status_;
    int8_t*                 battery_soc_;
    bool                    enable_low_power_ = false;
    void                    LCMPublishByThread();

private:
    int iter_             = 0;
    int unresponce_count_ = 0;

    void SetupStep();
    void FinalizeStep();

    void PublishCyberdogLcmFeedback();
    void PublishLcmFeedback();
    void PublishLcmMotorStates();
    void UnresponceSaftyCheck();

    void Mode2Pattern( double mode, double gait_num, int8_t& pattern );
    void Order2Mode( int order, int8_t& mode );

    Quadruped< float >            quadruped_;
    LegController< float >*       leg_controller_ = nullptr;
    StateEstimatorResult< float > state_estimate_result_;

    StateEstimatorContainer< float >* state_estimator_;

    bool cheater_mode_enabled_ = false;

    robot_control_response_lcmt lcm_data_old_;

    lcm::LCM                 lcm_;
    lcm::LCM                 state_lcm_;
    lcm::LCM                 global_to_robot_lcm_;
    leg_control_command_lcmt leg_control_command_lcm_;
    state_estimator_lcmt     state_estimator_lcm_;
    localization_lcmt        global_to_robot_lcmt_;
    leg_control_data_lcmt    leg_control_data_lcm_;
    // Contact Estimator to calculate estimated forces and contacts

    FloatingBaseModel< float > model_;
    u64                        iterations_ = 0;

    lcm::LCM             send_to_ros_lcm_;
    lcm::LCM             send_to_ros_lcm_cyberdog_;
    lcm::LCM             send_to_ros_lcm_motor_;
    int                  cmd_lcm_timeout_     = 0;
    static constexpr int cmd_lcm_timeout_max_ = 500;
    int                  lcm_iterations_      = 0;
};

#endif  // ROBOT_RUNNER_HPP_
