#ifndef FSM_STATE_BALANCESTAND_HPP_
#define FSM_STATE_BALANCESTAND_HPP_

#include "fsm_state.hpp"

template < typename T > class WbcCtrl;
template < typename T > class LocomotionCtrlData;

/**
 * @brief FSM State that forces all legs to be on the ground and uses the QP
 * Balance controller for instantaneous balance control.
 */
template < typename T > class FsmStateBalanceStand : public FsmState< T > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FsmStateBalanceStand( ControlFsmData< T >* control_fsm_data );

    // Behavior to be carried out when entering a state
    void OnEnter();  // override;

    // Run the normal behavior for the state
    void Run();

    // Checks for any transition triggers
    FsmStateName CheckTransition();

    // Manages state specific transitions
    TransitionData< T > Transition();

    // Behavior to be carried out when exiting a state
    void OnExit();

private:
    // Keep track of the control iterations
    int iter_ = 0;

    // Parses contact specific controls to the leg controller
    void BalanceStandStep();
    // Check if the state of leg is safe
    bool LegSafetyChecker();
    // Tune the scale of cmd value based on the priority of cmd types( height > yaw > roll == pitch )
    void RpyCmdRescale( const T& body_height_lim_des, Vec3< T >& rpy_lim_des, const RobotType& robot_type );

    WbcCtrl< T >*           wbc_ctrl_;
    LocomotionCtrlData< T >* wbc_data_;

    Vec3< T >      ini_body_pos_;
    Vec3< T >      ini_body_ori_rpy_;
    Vec3< T >      rpy_lim_des_;
    Vec3< double > rpy_cmd_last_;
    Vec3< T >      omega_cur_;
    T              body_weight_;

    bool      first_torctrlposture_;
    Vec9< T > body_pos_des_;
    Vec9< T > body_rpy_des_;
    Vec9< T > foot_pos_des_[ 4 ];
    Vec3< T > init_body_pos_;
    Vec3< T > init_body_rpy_;
    Vec3< T > init_foot_pos_[ 4 ];
    Vec3< T > init_foot_pos_bodyframe_[ 4 ];
    Vec3< T > body_pos_cmd_pre_;
    Vec3< T > body_rpy_cmd_pre_;
    Vec3< T > foot_pos_cmd_pre_[ 4 ];
    Vec4< T > pose_foot_support_;
    Vec3< T > public_q_cmd_[ 4 ];

    int  public_iter_            = 0;
    bool contact_state_old_[ 4 ] = { true, true, true, true };
    bool contact_transition_     = false;

    double    body_height_last_;
    T         body_height_lim_des_;
    T         body_height_vel_cur_;
    Vec9< T > QuinticPolySpline( const Vec3< T > trajec_cmd_pre, const Vec3< T > trajec_cmd, const double iter_T, double iter_t );

    Vec3< T > ini_pfoot_;

    T              safe_height_ = 0;
    Vec3< double > safe_rpy_;
    bool           is_unsafe_ = false;

    uint32_t no_move_cnt_ = 0;
    int32_t  duration_    = 0;
};

#endif  // FSM_STATE_BALANCESTAND_HPP_
