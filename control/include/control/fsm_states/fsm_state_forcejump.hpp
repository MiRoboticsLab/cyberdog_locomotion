#ifndef FSM_STATE_FORCE_JUMP_HPP_
#define FSM_STATE_FORCE_JUMP_HPP_

#include "balance_controller/balance_controller_lsm.hpp"
#include "fsm_state.hpp"

template < typename T > class FsmStateForceJump : public FsmState< T > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FsmStateForceJump( ControlFsmData< T >* control_fsm_data );

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
    BalanceControllerLSM* lsm_;

    bool       jump_first_run_;
    Vec12< T > leg_angle_init12_;
    Vec12< T > leg_angle_goal12_;
    Vec12< T > leg_angle_des12_;
    Vec3< T >  jump_x_acc_cmd_;
    Vec3< T >  jump_w_acc_cmd_;
    Vec4< T >  jump_contact_;
    Vec4< T >  jump_foot_support_;
    Vec3< T >  public_q_cmd_[ 4 ];
    bool       jump_isjump_;
    int        jump_time_;
    int        public_iter_;
    int        public_iter_jump_;

    void JumpStep();

    int SaftyCheck();
};

#endif  // FSM_STATE_FORCE_JUMP_HPP_
