#ifndef ROBOT_RUNNER_INTERFACE_HPP_
#define ROBOT_RUNNER_INTERFACE_HPP_

#include "robot_controller.hpp"

class RobotRunner;
class PeriodicTaskManager;

class RobotRunnerInterface {
public:
    RobotRunnerInterface( RobotController* controller, PeriodicTaskManager* taskManager, float period, std::string name );
    ~RobotRunnerInterface();

    void Init();
    void Start();
    void Run();
    void Cleanup();

    void InitializeStateEstimator( bool cheaterMode = false );

    ControlParameters* GetUserControlParameters();
    bool*              GetLowPowerEnable();

    void LCMPublishByThread();

    void SetCommandInterface( CommandInterface* value );
    void SetSpiData( SpiData* value );
    void SetSpiCommand( SpiCommand* value );
    void SetRobotType( RobotType value );
    void SetRobotAppearanceType( RobotAppearanceType value );
    void SetVectorNavData( VectorNavData* value );
    void SetRobotControlParameters( RobotControlParameters* value );
    void SetVisualizationData( VisualizationData* value );
    void SetCyberdog2Visualization( Cyberdog2Visualization* value );
    void SetCheaterState( CheaterState< double >* value );
    void SetBmsStatus( int8_t* value );
    void SetBattSoc( int8_t* value );

private:
    RobotRunner* robot_runner_;
};

#endif  // ROBOT_RUNNER_INTERFACE_HPP_
