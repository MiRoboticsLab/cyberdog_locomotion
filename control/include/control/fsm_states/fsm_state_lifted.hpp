#ifndef FSM_STATE_LIFTED_HPP_
#define FSM_STATE_LIFTED_HPP_

#include "fsm_state.hpp"

/**
 * @brief This state machine is used for lifted protection
 * It holds a constant joint position and waitting for landing
 *
 */
template < typename T > class FsmStateLifted : public FsmState< T > {
public:
    FsmStateLifted( ControlFsmData< T >* control_fsm_data );

    // Behavior to be carried out when entering a state
    void OnEnter();

    // Run the normal behavior for the state
    void Run();

    // Checks for any transition triggers
    FsmStateName CheckTransition();

    // Manages state specific transitions
    TransitionData< T > Transition();

    // Behavior to be carried out when exiting a state
    void OnExit();

private:
    int         iter_                 = 0;      // add one per timestep
    int         iter_landing_         = 0;      // iteration for landing state
    int         iter_falling_         = 0;      // iteration for falling state
    const int   duration_for_tau_avg_ = 500;    // compute tau_lift_avg_ in this duration
    const int   duration_for_landing_ = 500;    // duration to trigger ready_for_switch_ for landing state
    const int   duration_for_falling_ = 250;    // duration to trigger ready_for_switch_ for falling state
    const float falling_threshold_    = 1.0;    // threshold for detecting falling
    bool        landing_flag_         = false;  // is true if landing is detected
    bool        falling_flag_         = false;
    Vec3< T >   tau_lift_avg_[ 4 ];    // computes tau average after robot is lifted
    Vec3< T >   joint_pos_lifted_;     // hold this joint position after robot is lifted
    Vec3< T >   tau_trigger_landing_;  // a threshold that triggers landing
};

#endif  // FSM_STATE_LIFTED_HPP_
