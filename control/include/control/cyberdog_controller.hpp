#ifndef CYBERDOG_CONTROLLER_HPP_
#define CYBERDOG_CONTROLLER_HPP_

#include "user_parameters.hpp"
#include "fsm_states/control_fsm.hpp"
#include "robot_controller.hpp"
#include <fstream>

enum SwitchStatus { kDone, kTransitioning, kEstop, kEdamp, kRobotLifted, kBanTrans, kOverHeat, kLowBat };

class CyberdogController : public RobotController {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CyberdogController();
    virtual ~CyberdogController() {}

    virtual void HandleLegErrorAndWarn( int32_t flags[ 12 ] ) override;

    virtual int GetLegMode( int32_t flags[ 12 ] ) override;

    void HandleMotorsTemperature( int16_t tmp_abad[ 4 ], int16_t tmp_hip[ 4 ], int16_t tmp_knee[ 4 ] ) override {
        for ( int8_t leg( 0 ); leg < 4; leg++ ) {
            motor_tmp_[ 3 * leg ]     = 0.1 * tmp_abad[ leg ];
            motor_tmp_[ 3 * leg + 1 ] = 0.1 * tmp_hip[ leg ];
            motor_tmp_[ 3 * leg + 2 ] = 0.1 * tmp_knee[ leg ];
        }
    }

    bool CheckMotorsOverHeat() override {
        return motor_over_heat_;
    }

    bool CheckMotorsReturnSaftyTemperature() override {
        for ( int i( 0 ); i < 12; i++ ) {
            if ( motor_tmp_[ i ] > motor_safety_tmp_ ) {
                return false;
            }
        }
        return true;
    }

    virtual bool GetMotionModelag() override {
        return motor_mode_error_;
    }

    virtual void               InitializeController() override;
    virtual void               RunController() override;
    virtual void               UpdateVisualization() override {}
    virtual ControlParameters* GetUserControlParameters() override {
        return &user_parameters_;
    }
    virtual void Estop() override {
        control_fsm_->Initialize();
    }

    virtual int GetFsmMode() override {
        if ( control_fsm_ && control_fsm_->current_state_ ) {
            if ( control_fsm_->current_state_->state_name_ != FsmStateName::kOff && control_fsm_->current_state_->state_name_ != FsmStateName::kPureDamper
                 && control_fsm_->data.command->motion_trigger > 0 )
                return static_cast< int >( FsmStateName::kMotion );
            else
                return static_cast< int >( control_fsm_->current_state_->state_name_ );
        }
        else
            return 0;
    }
    virtual int GetFsmGaitId() override {
        if ( control_fsm_ && control_fsm_->current_state_ )
            if ( GetFsmMode() == static_cast< int >( FsmStateName::kMotion ) )
                return static_cast< int >( control_fsm_->data.command->motion_id );
            else if ( GetFsmMode() == static_cast< int >( FsmStateName::kLocomotion ) )
                return static_cast< int >( robot_current_state_.gait_cmd_used );
            else
                return static_cast< int >( robot_current_state_.gait_id );
        else
            return 0;
    }
    virtual int GetSwitchStatus() override {
        int ban_flag = control_fsm_->current_state_->transition_data_.ban_trans_flag;
        if ( ban_flag > 0 )
            return static_cast< int >( ( SwitchStatus )ban_flag );
        int OperationMode = static_cast< int >( control_fsm_->GetOperatingMode() );
        if ( OperationMode == 0 && ( GetFsmGaitId() != control_fsm_->data.command->gait_id ) && ( GetFsmMode() != static_cast< int >( FsmStateName::kMotion ) ) )
            return static_cast< int >( SwitchStatus::kTransitioning );
        return OperationMode;
    }

    virtual bool GetFsmSwitchFlag() override {
        if ( control_fsm_ && control_fsm_->current_state_ )
            return static_cast< bool >( control_fsm_->current_state_->ready_for_switch_ );
        else
            return false;
    }

    virtual int GetFsmProcessBar() override {
        if ( control_fsm_ && control_fsm_->current_state_ ) {
            if ( GetFsmMode() == static_cast< int >( FsmStateName::kMotion ) )
                return control_fsm_->data.command->motion_process_bar;
            else
                return control_fsm_->current_state_->motion_progress_bar_;
        }
        else
            return 0;
    }

    virtual bool GetMotorErrorFlag() override {
        return motor_error_;
    }

    virtual bool GetSafetyCheckErrorFlag() override {
        return control_fsm_->safety_check_error_;
    }

    virtual bool GetOriErrorFlag() override {
        return control_fsm_->orientation_error_;
    }

    virtual int32_t GetFootPosError() override {
        return control_fsm_->foot_pos_error_;
    }

    virtual bool GetMotorWarnFlag() override {
        return motor_warn_;
    }

protected:
    bool                       motor_error_;
    bool                       motor_warn_;
    bool                       motor_mode_error_;
    float                      motor_tmp_[ 12 ];
    bool                       motor_over_heat_;
    const float                motor_safety_tmp_ = 74.0;
    long                       motor_mode_err_count_[ 12 ];
    ControlFsm< float >*       control_fsm_;
    UserParameters         user_parameters_;
    RobotCurrentState< float > robot_current_state_;
};

#endif  // CYBERDOG_CONTROLLER_HPP_
