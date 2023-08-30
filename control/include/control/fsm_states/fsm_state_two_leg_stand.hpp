#ifndef FSM_STATE_TWO_LEG_STAND_HPP_
#define FSM_STATE_TWO_LEG_STAND_HPP_

#include <math.h>
#include <vector>

#include "fsm_state.hpp"

template < typename T > class WbcCtrl;
template < typename T > class LocomotionCtrlData;

/**
 * @brief FSM state for Two Leg Stand (front-back)
 *
 * Transitionary state that is called for the robot to stand by two legs
 * action is similar to “New year's greeting"
 */
template < typename T > class FsmStateTwoLegStand : public FsmState< T > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructor for the FSM State that passes in state specific info to the generic FSM State constructor.
    FsmStateTwoLegStand( ControlFsmData< T >* control_fsm_data );

    // Behavior to be carried out when entering a state
    void OnEnter();

    // Run the normal behavior for the state.
    void Run();

    // Checks for any transition triggers.
    FsmStateName CheckTransition();

    // Manages state specific transitions.
    TransitionData< T > Transition();

    // Behavior to be carried out when exiting a state.
    void OnExit();

private:
    // Keep track of the control iterations
    int iter_ = 0;

    // Each flag corresponds to an action stage like "kneel down" or "pushup".
    int jump_flag_ = 0;

    // in each step jump_iter_+1, and the product of dt and jump_iter is taken as current_time, but note that it can be set to 0 or 1 to control when the time clock starts
    int jump_iter_ = 0;

    // if the action is done,the value = 1
    int front_leg_jump_ok_[ 2 ];

    // flag to make diference between cyberdog & cyberdog2
    int is_cyberdog2_ = 0;

    // Record the leg angle to generate control command
    Vec12< T > leg_angle_init_;
    Vec12< T > leg_angle_goal_;
    Vec12< T > leg_angle_des_;
    Vec12< T > leg_angle_temp_;
    Vec12< T > leg_angle_log_;
    Vec12< T > foot_pos_init_;
    Vec12< T > foot_pos_goal_;
    Vec12< T > foot_pos_des_;
    Vec12< T > leg_angle_cmd_init_;
    // Initial body state
    Vec3< T > ini_body_pos_;
    Vec3< T > ini_body_ori_rpy_;

    // Record control mode, which should be force_control when contact ground, and pos_control when leave ground
    bool front_leg_force_to_pos_flag_{ false };

    // Record the body state during the motion
    double jump_pitch_{ 0.0 };
    double jump_pitch_vel_{ 0.0 };
    double jump_pitch_ang_tmp_{ 0.0 };
    double jump_pitch_vel_tmp_{ 0.0 };

    // Pass the leg angle between adjacent action stages
    Vec3< T > rear_leg_angle_1_;
    Vec3< T > rear_leg_angle_2_;
    Vec3< T > rear_leg_angle_3_;
    Vec3< T > rear_leg_angle_4_;

    // Time keypoints of action stages switch
    double t_1_{ 0.0 };
    double t_2_{ 0.0 };

    // joint angle limit check
    float safe_angle_low_;
    float safe_angle_upp_;

    // joint angle when stand
    Vec3< T > stand_j_pos_[ 4 ];

    // value = 0.5 when contact, otherwise 0
    Vec4< T > contact_states_;

    LegControllerCommand< T > leg_cmd_old_[ 4 ];

    // passive swing flag
    bool          stop_swing_[ 2 ];
    Vec4< float > joint_torque_filter_;

    // Calculate the commands for support legs(WBC) and swing legs(Joint PD).
    void TwoLegStandStep();

    void CheckContactObstacle( int8_t leg_id, float current_time, float unit_period, bool rebound_after_contact = false );
};
#endif  // FSM_STATE_TWO_LEG_STAND_HPP_
