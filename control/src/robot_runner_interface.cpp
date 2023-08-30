#include "robot_runner_Interface.hpp"
#include "robot_controller.hpp"
#include "robot_runner.hpp"
#include "utilities/periodic_task.hpp"

RobotRunnerInterface::RobotRunnerInterface( RobotController* controller, PeriodicTaskManager* taskManager, float period, std::string name ) {
    robot_runner_ = new RobotRunner( controller, taskManager, period, name );
}

RobotRunnerInterface::~RobotRunnerInterface() {
    delete robot_runner_;
}

void RobotRunnerInterface::SetCommandInterface( CommandInterface* value ) {
    robot_runner_->cmd_interface_ = value;
}
void RobotRunnerInterface::SetSpiData( SpiData* value ) {
    robot_runner_->spi_data_ = value;
}
void RobotRunnerInterface::SetSpiCommand( SpiCommand* value ) {
    robot_runner_->spi_command_ = value;
}
void RobotRunnerInterface::SetRobotType( RobotType value ) {
    robot_runner_->robot_type_ = value;
}
void RobotRunnerInterface::SetRobotAppearanceType( RobotAppearanceType value ) {
    robot_runner_->robot_appearance_type_ = value;
}
void RobotRunnerInterface::SetVectorNavData( VectorNavData* value ) {
    robot_runner_->vector_nav_data_ = value;
}
void RobotRunnerInterface::SetRobotControlParameters( RobotControlParameters* value ) {
    robot_runner_->control_parameters_ = value;
}
void RobotRunnerInterface::SetVisualizationData( VisualizationData* value ) {
    robot_runner_->visualization_data_ = value;
}
void RobotRunnerInterface::SetCyberdog2Visualization( Cyberdog2Visualization* value ) {
    robot_runner_->cyberdog2_main_visualization_ = value;
}
void RobotRunnerInterface::SetCheaterState( CheaterState< double >* value ) {
    robot_runner_->cheater_state_ = value;
}
void RobotRunnerInterface::SetBmsStatus( int8_t* value ) {
    robot_runner_->bms_status_ = value;
}

void RobotRunnerInterface::SetBattSoc( int8_t* value ) {
    robot_runner_->battery_soc_ = value;
}

void RobotRunnerInterface::Init() {
    robot_runner_->InitTask();
}

void RobotRunnerInterface::Start() {
    robot_runner_->StartTask();
}

void RobotRunnerInterface::Run() {
    robot_runner_->RunTask();
}

void RobotRunnerInterface::Cleanup() {
    robot_runner_->CleanUp();
}

ControlParameters* RobotRunnerInterface::GetUserControlParameters() {
    return robot_runner_->robot_ctrl_->GetUserControlParameters();
}

bool* RobotRunnerInterface::GetLowPowerEnable() {
    return &( robot_runner_->enable_low_power_ );
}

/**
 * @brief Initialize the state estimator with default no cheaterMode
 *
 **/
void RobotRunnerInterface::InitializeStateEstimator( bool cheaterMode ) {
    robot_runner_->InitializeStateEstimator( cheaterMode );
}

void RobotRunnerInterface::LCMPublishByThread() {
    robot_runner_->LCMPublishByThread();
}
