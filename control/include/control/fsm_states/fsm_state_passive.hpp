#ifndef FSM_STATE_PASSIVE_HPP_
#define FSM_STATE_PASSIVE_HPP_

#include "fsm_state.hpp"

/**
 * @brief FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 *
 */
template < typename T > class FsmStatePassive : public FsmState< T > {
public:
    FsmStatePassive( ControlFsmData< T >* control_fsm_data );

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
    // Keep track of the control iterations
    int iter_ = 0;
};

#endif  // FSM_STATE_PASSIVE_HPP_
