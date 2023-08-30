#ifndef ROBOT_CONTROLLER_HPP_
#define ROBOT_CONTROLLER_HPP_

#include "command_interface/command_interface.hpp"
#include "command_interface/gamepad_command.hpp"
#include "controllers/leg_controller.hpp"
#include "controllers/state_estimator_container.hpp"
#include "dynamics/floating_base_model.hpp"
#include "sim_utilities/visualization_data.hpp"

/**
 * @brief Parent class of user robot controllers.
 * This is an interface between the control code and the common hardware code.
 *
 */
class RobotController {
    friend class RobotRunner;

public:
    RobotController() {}
    virtual ~RobotController() {}

    /**
     * @brief handle motor errors, this function will be called before `RunController`
     *        in every control loop
     */
    virtual void HandleLegErrorAndWarn( int32_t flags[ 12 ] ) = 0;

    virtual int GetLegMode( int32_t flags[ 12 ] ) = 0;

    /**
     * @brief handle motor temperature
     *
     * @param tmp_abad
     * @param tmp_hip
     * @param tmp_knee
     */
    virtual void HandleMotorsTemperature( int16_t tmp_abad[ 4 ], int16_t tmp_hip[ 4 ], int16_t tmp_knee[ 4 ] ) {
        for ( int8_t i( 0 ); i < 4; i++ ) {
            ( void )tmp_abad[ i ];
            ( void )tmp_hip[ i ];
            ( void )tmp_knee[ i ];
        }
    };

    virtual bool CheckMotorsOverHeat() {
        return false;
    };

    virtual bool CheckMotorsReturnSaftyTemperature() {
        return true;
    };

    virtual bool GetMotionModelag() {
        return false;
    }

    virtual void InitializeController() = 0;

    // Called one time every control loop
    virtual void               RunController()            = 0;
    virtual void               UpdateVisualization()      = 0;
    virtual ControlParameters* GetUserControlParameters() = 0;
    virtual void               Estop() {}

    virtual int GetFsmMode() {
        return 0;
    }

    virtual int GetFsmGaitId() {
        return 0;
    }

    virtual bool GetFsmSwitchFlag() {
        return false;
    }

    virtual int GetSwitchStatus() {
        return 0;
    }

    virtual int GetFsmProcessBar() {
        return 0;
    }

    virtual bool GetMotorErrorFlag() {
        return false;
    }

    virtual bool GetSafetyCheckErrorFlag() {
        return false;
    }

    virtual bool GetOriErrorFlag() {
        return false;
    }

    virtual int32_t GetFootPosError() {
        return 0;
    }

    virtual bool GetMotorWarnFlag() {
        return false;
    }

protected:
    Quadruped< float >*               quadruped_             = nullptr;
    FloatingBaseModel< float >*       model_                 = nullptr;
    LegController< float >*           leg_controller_        = nullptr;
    StateEstimatorContainer< float >* state_estimator_       = nullptr;
    StateEstimatorResult< float >*    state_estimate_result_ = nullptr;
    RobotControlParameters*           control_parameters_    = nullptr;
    const MotionControlCommand*       command_               = nullptr;

    VisualizationData* visualization_data_ = nullptr;
    RobotType          robot_type_;
};

#endif  // ROBOT_CONTROLLER_HPP_
