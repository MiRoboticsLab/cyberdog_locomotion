#ifndef FSM_STATE_RECOVERYSTAND_HPP_
#define FSM_STATE_RECOVERYSTAND_HPP_

#include "fsm_state.hpp"

template < typename T > class FSMStateRecoveryStand : public FsmState< T > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FSMStateRecoveryStand( ControlFsmData< T >* control_fsm_data );

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
    unsigned long long motion_start_iter_ = 0;

    static constexpr int StandUp_            = 0;
    static constexpr int FoldLegs_           = 1;
    static constexpr int LeftRollOver_       = 2;
    static constexpr int SideLying_          = 3;
    static constexpr int LieDown_            = 4;
    static constexpr int LieSide_            = 5;
    static constexpr int LookUpOrDown_       = 6;
    static constexpr int PrepareForFoldLegs_ = 7;
    static constexpr int RightRollOver_      = 8;

    unsigned long long state_iter_;
    unsigned long long gesture_err_iter_;
    int                flag_;

    Vec4< T > RightLegConfigFlag;

    // JPos
    Vec3< T > fold_jpos[ 4 ];
    Vec3< T > side_jpose[ 4 ];
    Vec3< T > stand_jpos[ 4 ];
    Vec3< T > rolling_jpos[ 4 ];
    Vec3< T > initial_jpos[ 4 ];
    Vec3< T > zero_vec3;
    Vec3< T > foot_pos[ 4 ];
    Vec3< T > initial_foot_pos[ 4 ];

    int LieDown_flag_     = 0;
    int fold_ramp_iter_   = 1000;
    int fold_settle_iter_ = 100;

    int prepare_ramp_iter_   = 1500;
    int prepare_settle_iter_ = 100;

    int damp_ramp_iter_   = 1000;
    int damp_settle_iter_ = 1000;

    int  side_ramp_iter_   = 1000;
    int  side_settle_iter_ = 500;
    int  drop_safety_      = 0;
    int  drop_flag_        = 0;
    int  stuck_count_      = 0;
    int  soft_limimt_      = 0;
    char which_side_       = 'l';
    bool liedown_ok_       = false;
    bool first_stand_      = true;
    int  stand_waitstable_ = 50;

    const int standup_ramp_iter_ = 350;

    void LeftRollOver( const int& iter );
    void RightRollOver( const int& iter );
    void StandUp( const int& iter );
    void FoldLegs( const int& iter );
    void SideLying( const int& curr_iter );
    void LieDown( const int& iter );
    void LieSide( const int& iter );
    void SwitchSIde();
    int  SaftyCheck();
    void PrepareForFoldLegs( const int& curr_iter );
    void LookUpOrDown( const int& curr_iter );

    bool OnRightLegConfig();
    bool UpsideDown();
    int  BeLookUpOrDown();
    int  SideLyingState();
    void SetJPosInterPts( const size_t& curr_iter, size_t max_iter, int leg, const Vec3< T >& ini, const Vec3< T >& fin );
    void JointPdControl( int leg, Vec3< T > qDes, Vec3< T > qdDes );
    void JointPdControl( int leg, Vec3< T > kd );
    void SoftLimitation( T& value, T min, T max );
};

#endif  // FSM_STATE_RECOVERYSTAND_HPP_
