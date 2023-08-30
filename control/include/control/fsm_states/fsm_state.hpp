#ifndef FSM_STATE_HPP_
#define FSM_STATE_HPP_

#include <stdio.h>

#include "control_fsm_data.hpp"
#include "robot_runner.hpp"
#include "transition_data.hpp"
#include "parameters.hpp"
#include "control_flags.hpp"


/**
 * Enumerate all of the FSM states so we can keep track of them.
 * All states have the same value with MotionMode
 */
using FsmStateName = MotionMode;

/**
 * @brief parent class implementing the state of finite state machine(FSM)
 *
 * @tparam T
 */
template < typename T > class FsmState {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Generic constructor for all states
    FsmState( ControlFsmData< T >* control_fsm_data, FsmStateName state_name_input, std::string state_string_input );

    // Behavior to be carried out when entering a state
    virtual void OnEnter() = 0;  // {}

    // Run the normal behavior for the state
    virtual void Run() = 0;  //{}

    // Manages state specific transitions
    virtual FsmStateName CheckTransition() {
        return FsmStateName::kInvalid;
    }

    // Runs the transition behaviors and returns true when done transitioning
    virtual TransitionData< T > Transition() {
        return transition_data_;
    }

    // Behavior to be carried out when exiting a state
    virtual void OnExit() = 0;  // {}

    // Leg controller
    void JointPdControl( int leg, Vec3< T > q_des, Vec3< T > qd_des );
    void JointSafetyControl( int leg, Vec3< T > qd_des );
    void JointDamperControl( int leg, Vec3< T > qd_des );
    void CartesianImpedanceControl( int leg, Vec3< T > p_des, Vec3< T > v_des, Vec3< double > kp_cartesian, Vec3< double > kd_cartesian );

    // Safety check
    void TurnOnAllSafetyChecks();
    void TurnOffAllSafetyChecks();

    // Holds all of the relevant control data
    ControlFsmData< T >* data_;

    // FSM State info
    FsmStateName state_name_;       // enumerated name of the current state
    FsmStateName next_state_name_;  // enumerated name of the next state
    std::string  state_string_;     // state name string

    // Transition parameters
    T                   transition_duration_;  // transition duration time
    TransitionData< T > transition_data_;

    // Pre controls safety checks
    bool check_safe_orientation_ = false;  // check roll and pitch
    bool check_robot_lifted_     = false;  // check robot lifted

    // Post control safety checks
    bool check_desired_foot_position_ = false;  // do not command footsetps too far
    bool check_feed_forward_force_    = false;  // do not command huge forces
    bool check_leg_singularity_       = false;  // do not let leg

    bool ready_for_switch_    = false;
    int  motion_progress_bar_ = 0;

protected:
    // Iteration for printf
    int iter_printf_       = 2500;
    int iter_printf_reset_ = 2500;

private:
    // Create the cartesian P gain matrix
    Mat3< float > kp_mat_;

    // Create the cartesian D gain matrix
    Mat3< float > kd_mat_;
};

#endif  // FSM_STATE_HPP_
