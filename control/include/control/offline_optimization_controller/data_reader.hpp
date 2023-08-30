#ifndef DATA_READER_HPP_
#define DATA_READER_HPP_

#include "cpp_types.hpp"
#include "fsm_states/fsm_state.hpp"

enum OptimalizedVariablesOffset {
    kGeneralPositionOffset = 0,   // x, z, yaw, front hip, front knee, rear hip, rear knee
    kGeneralVelocityOffset = 7,   // x, z, yaw, front hip, front knee, rear hip, rear knee
    kJointTorqueOffset     = 14,  // front hip, front knee, rear hip, rear knee
    kReactionForceOffset   = 18   // front x, front z, rear x, rear z
};

typedef Eigen::Matrix< float, 7, 1 > Vector7f;

/**
 * @brief Get offline optimized trajectory by reading binary files
 * the coloum presents all optimized variables that described at OptimalizedVariablesOffset
 * the row presents all time steps on planning horizon, time interval is one millisecond in general
 *
 */
class DataReader {
public:
    DataReader( const RobotType& type, FsmStateName fsm_state_name, const int gait_id );
    ~DataReader();

    float* GetInitialConfiguration();
    float* GetTrajectoryAtOnePoint( int time_step );

    int GetDataTimeSteps() {
        return data_time_steps_;
    }
    int GetDataCols() {
        return data_cols_;
    }

    RobotType robot_type_;

private:
    float* data_location_;
    bool   data_loaded_ = false;

    int data_time_steps_ = -1;
    int data_cols_       = 22;

    void LoadOptimalizedTrajectory( const char* filename );
    void UnLoadOptimalizedTrajectory();
};

#endif  // DATA_READER_HPP_
