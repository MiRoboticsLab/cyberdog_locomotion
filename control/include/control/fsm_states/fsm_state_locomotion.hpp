#ifndef FSM_STATE_LOCOMOTION_HPP_
#define FSM_STATE_LOCOMOTION_HPP_

#include <semaphore.h>

#include "convex_mpc/convex_mpc_loco_gaits.hpp"
#include "convex_mpc/convex_mpc_motion_gaits.hpp"
#include "fsm_state.hpp"
#include "qpOASES/Utils.hpp"

#define SEPERATE_MPC_ANOTHER_THREAD

template < typename T > class WbcCtrl;
template < typename T > class LocomotionCtrlData;

/**
 * @brief This state machine runs all locomotion gaits
 * and some MPC are inculuded in this state machine
 *
 */
template < typename T > class FsmStateLocomotion : public FsmState< T > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Construct a new fsm state locomotion object
    FsmStateLocomotion( ControlFsmData< T >* control_fsm_data );

    // Behavior to be carried out when entering a state
    void OnEnter();

    // Run the normal behavior for the state
    void Run();

    // Checks for any transition triggers
    FsmStateName CheckTransition();

    // Manages state specific transitions
    TransitionData< T > Transition();

    // Behavior to be carried out when exiting a state.
    void OnExit();

    // Initialize user gaits.
    void InitUserGaits();

#ifdef SEPERATE_MPC_ANOTHER_THREAD
    std::thread*  solve_mpc_thread_;
    std::ofstream mpc_thread_log_f_;
    sem_t         solve_mpc_flag_;

    void RunSolveMpcAnotherThread();
#endif

private:
    // Keep track of the control iterations
    int                      iter_       = 0;
    bool                     trans_flag_ = false;
    bool                     first_locomotion_run_;
    int                      public_iter_;
    int                      convex_switch_index_      = 1;  // 1: Loco; 2: Motion; 3: Stair
    int                      convex_switch_index_last_ = 1;
    DVec< T >                weight_vec_;
    ConvexMpcLocoGaits*      convex_mpc_loco_gaits_;
    ConvexMpcMotionGaits*    convex_mpc_motion_gaits_;
    WbcCtrl< T >*            wbc_ctrl_;
    LocomotionCtrlData< T >* wbc_data_;
    float                    duration_ = 0;

    // Parses contact specific controls to the leg controller
    void LocomotionControlStep();

    // Judges whether the robot is in good condition
    bool LocomotionSafe();
};

#endif  // FSM_STATE_LOCOMOTION_HPP_
