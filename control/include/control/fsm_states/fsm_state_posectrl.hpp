#ifndef FSM_STATE_POSE_CTRL_HPP_
#define FSM_STATE_POSE_CTRL_HPP_

#include "fsm_state.hpp"
#include "position_controller/position_controller.hpp"

template < typename T > class FsmStatePoseCtrl : public FsmState< T > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FsmStatePoseCtrl( ControlFsmData< T >* control_fsm_data );

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
    PositionController* p_ctrl_;
    int                 trans_iter_;
    Vec3< T >           ini_body_pos_;
    Vec3< T >           ini_body_ori_rpy_;

    bool          poseFirstRun_;
    bool          transFirshRun_;
    Vec3< float > public_q_cmd_[ 4 ];
    Vec3< float > trans_start_q_[ 4 ];
    Vec3< float > trans_end_q_[ 4 ];
    float         height_;

    Vec6< float > pose_body_cmd_;
    Vec3< float > pose_foot_cmd_;
    Vec4< float > pose_ctrl_point_;
    Vec4< float > pose_foot_support_;

    bool dampSwingLeg_;
    void PoseStep( const bool SwinglegFlag );

    int SaftyCheck();
};

#endif  // FSM_STATE_POSE_CTRL_HPP_
