#ifndef FSM_STATE_PURE_DAMPER_HPP_
#define FSM_STATE_PURE_DAMPER_HPP_

#include "fsm_state.hpp"

/**
 * @brief FSM state for pure damping control.
 *
 */
template < typename T > class FsmStatePureDamper : public FsmState< T > {
public:
    FsmStatePureDamper( ControlFsmData< T >* control_fsm_data );

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

    TransitionData< T > TestTransition();

private:
    int       iter_    = 0;
    const int pd_iter_ = 3000;  // increase from 1500 to 3000 to ensure the damping motion is fully done when switching from sit-down
    Vec3< T > zero_vec3_;
    Vec3< T > initial_jpos_[ 4 ];
    Vec3< T > target_jpos_[ 4 ];
};

#endif  // FSM_STATE_PURE_DAMPER_HPP_
