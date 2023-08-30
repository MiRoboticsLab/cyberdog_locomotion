#ifndef FSM_STATE_MOTOR_CTRL_HPP_
#define FSM_STATE_MOTOR_CTRL_HPP_

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <string>
#include <thread>
#include <vector>

#include "fsm_state.hpp"
#include "header/lcm_type/motor_ctrl_lcmt.hpp"
#include "header/lcm_type/motor_ctrl_state_lcmt.hpp"
#include "utilities/timer.hpp"
#include "utilities/toolkit.hpp"

template < typename T > class FsmStateMotorCtrl : public FsmState< T > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FsmStateMotorCtrl( ControlFsmData< T >* control_fsm_data );
    void                OnEnter();
    void                OnExit();
    void                Run();
    bool                CheckZero();
    FsmStateName        CheckTransition();
    TransitionData< T > Transition();

private:
    lcm::LCM motor_ctrl_state_lcm_;

    bool                  firstRun_;
    motor_ctrl_lcmt       ctrl_, ctrl_zero_;
    motor_ctrl_state_lcmt ctrl_state_;
    int16_t               err_flag_;
    uint64_t              wait_reconnect_;
};

#endif  // FSM_STATE_MOTOR_CTRL_HPP_
