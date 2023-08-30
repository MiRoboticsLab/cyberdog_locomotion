#ifndef CONTROL_FSM_HPP_
#define CONTROL_FSM_HPP_

#include <iostream>

// Contains all of the control related data
#include "control_fsm_data.hpp"

// Checks the robot state and commands for safety
#include "safety_checker.hpp"

#include "parameters.hpp"

// FSM States
#include "fsm_states/fsm_state.hpp"
#include "fsm_states/fsm_state_balancestand.hpp"
#include "fsm_states/fsm_state_forcejump.hpp"
#include "fsm_states/fsm_state_locomotion.hpp"
#include "fsm_states/fsm_state_passive.hpp"
#include "fsm_states/fsm_state_posectrl.hpp"
#include "fsm_states/fsm_state_pure_damper.hpp"
#include "fsm_states/fsm_state_recoverystand.hpp"

// new gait
#include "fsm_states/fsm_state_jump_3d.hpp"
#include "fsm_states/fsm_state_lifted.hpp"
#include "fsm_states/fsm_state_motor_ctrl.hpp"
#include "fsm_states/fsm_state_two_leg_stand.hpp"

// rl gait
#include "fsm_states/fsm_state_rl_rapid.hpp"
#include "fsm_states/fsm_state_rl_reset.hpp"

/**
 * Enumerate all of the operating modes
 */
enum class FsmOperatingMode { kNormal, kTransitioning, kEstop, kEdamp, kRobotLifted };

/**
 * @brief Contains all Fsm states
 *
 */
template < typename T > struct FsmStateList {
    FsmState< T >*             invalid;
    FsmStatePassive< T >*      passive;
    FsmStatePoseCtrl< T >*     posectrl;
    FsmStateBalanceStand< T >* balance_stand;
    FsmStateLocomotion< T >*    locomotion;
    FSMStateRecoveryStand< T >* recoveryStand;
    FsmStatePureDamper< T >*    pure_damper;  // TODO refactor it
    // new gait
    FsmStateTwoLegStand< T >* two_leg_stand;
    FsmStateJump3d< T >*      jump_3d;
    FsmStateForceJump< T >*   forcejump;
    FsmStateMotorCtrl< T >*   motor_ctrl;
    FsmStateLifted< T >*      lifted;

    // rl gait
    FsmStateRlReset< T >* rl_reset;
    FsmStateRlRapid< T >* rl_rapid;
};

/**
 * @brief Control FSM handles the FSM states from a higher level
 * The Finite State Machine that manages the robot's controls. Handles
 * calls to the FSM State functions and manages transitions between all
 * of the states.
 *
 */
template < typename T > class ControlFsm {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ControlFsm( Quadruped< T >* quadruped, StateEstimatorContainer< T >* state_estimator, LegController< T >* leg_controller, const MotionControlCommand* cmd,
                RobotControlParameters* control_parameters, VisualizationData* visualization_data, UserParameters* user_parameters, RobotCurrentState< T >* robot_current_state );

    // Initializes the Control FSM instance
    void Initialize();

    // Runs the FSM logic and handles the state transitions and normal runs
    void RunFsm();

    void SetOperatingMode( FsmOperatingMode operating_mode ) {
        operating_mode_ = operating_mode;
    }

    FsmOperatingMode GetOperatingMode() {
        return operating_mode_;
    }

    FsmOperatingMode SafetyPreCheck();
    FsmOperatingMode SafetyPostCheck();

    // Gets the next FsmState from the list of created states when requested
    FsmState< T >* GetNextState( FsmStateName state_name );

    // Prints the current FSM status
    void PrintInfo( int opt );

    // Contains all of the control related data
    ControlFsmData< T > data;

    // FSM state information
    FsmStateList< T > states_list_;      // holds all of the FSM States
    FsmState< T >*    current_state_;    // current FSM state
    FsmState< T >*    next_state_;       // next FSM state
    FsmStateName      next_state_name_;  // next FSM state name

    // Checks all of the inputs and commands for safety
    SafetyChecker< T >* safety_checker_;

    TransitionData< T > transition_data_;
    bool                safety_check_error_;
    int32_t             foot_pos_error_    = 0;
    bool                orientation_error_ = false;  // record the error of orientation
    bool                robotlifted_error_ = false;  // record the error of robot lifted
private:
    void CheckBatteryAndMotor();

    // Operating mode of the FSM
    FsmOperatingMode operating_mode_;

    // Choose how often to print info, every N iterations
    int print_num_ = 10000;  // N*(0.001s) in simulation time

    // Track the number of iterations since last info print
    int       print_iter_    = 0;  // make larger than print_num_ to not print
    int       warn_log_      = 0;
    int       iter_          = 0;
    float     max_temp_      = 0;
    bool      over_heat_     = false;
    bool      low_bat_       = false;
    int       low_bat_iter_  = 0;
    int       charging_iter_ = 0;
    Vec3< T > max_foot_pos_for_lift_, min_joint_torque_for_lift_, max_omega_for_lift_, max_rpy_for_lift_;  // For robot lifted
    T         max_leg_length_for_lift_;
    RobotType robot_type_;
    // lcm::LCM state_estimator_lcm;
    // state_estimator_lcmt _state_estimator;
};

#endif  // CONTROL_FSM_HPP_
