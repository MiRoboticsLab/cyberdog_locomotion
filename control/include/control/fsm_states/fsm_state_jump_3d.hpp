#ifndef FSM_STATE_JUMP3D_H_
#define FSM_STATE_JUMP3D_H_

#include "fsm_state.hpp"
#include "offline_optimization_controller/data_reader.hpp"
#include "offline_optimization_controller/offline_data_ctrl.hpp"
#include "wbc_ctrl/locomotion_ctrl/locomotion_ctrl.hpp"

/**
 * @brief FSM state for dynamic jump with offline data
 *
 */
template < typename T > class FsmStateJump3d : public FsmState< T > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FsmStateJump3d( ControlFsmData< T >* control_fsm_data );

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
    WbcCtrl< T >*            wbc_ctrl_;
    LocomotionCtrlData< T >* wbc_data_;
    OfflineOptCtrlData< T >* offline_opt_data_;

    // Keep track of the control iterations
    Vec3< T >   initial_jpos_[ 4 ]        = { Vec3< T >::Zero() };
    RotMat< T > body_rot_matrix_yaw_init_ = RotMat< T >::Zero();
    bool        use_wbc_                  = true;  // true: WBC, false: joint control
    bool        first_visit_              = true;
    float       curr_time_                = 0;
    int         pre_mode_duration_        = 100;
    int         landing_delay_            = 100;
    int         landing_count_            = 0;
    float       landing_filter_           = 0.995;
    int         touch_down_delay_         = 0;
    const int   touhc_down_delay_thresh_  = 150;
    bool        data_end_                 = false;  // whether it comes to end of data
    bool        height_good_for_trans_    = false;  // whether it height is good for transition
    bool        use_height_good_          = true;   // whether considers height_good_for_trans_ in transition
    const float height_min_thresh         = 0.2;    // min height for transition
    bool        flying_phase_             = false;  // whether all contact_state are zero
    bool        touch_down_               = false;  // whether robot touches down
    int         touch_down_count_         = 0;      // a counter that triggers touch_down
    T           touch_down_threh_         = 10.0;   // threshold of contact force that triggers touch down
    Vec3< T >   joint_pos_end_pos_[ 4 ]   = { Vec3< T >::Zero() };
    DVec< T >   weight_vec_               = DVec< T >::Zero( 6, 1 );

    DataReader* data_reader_yaw_p90_;
    DataReader* data_reader_x_p60_;
    DataReader* data_reader_yaw_n90_;
    DataReader* data_reader_z_p30_;
    DataReader* data_reader_down_stair_;
    DataReader* data_reader_y_p20_;
    DataReader* data_reader_x_p30_;
    DataReader* data_reader_y_n20_;

    OfflineDataCtrl< T >* jump_ctrl_yaw_p90_;
    OfflineDataCtrl< T >* jump_ctrl_x_p60_;
    OfflineDataCtrl< T >* jump_ctrl_yaw_n90_;
    OfflineDataCtrl< T >* jump_ctrl_z_p30_;
    OfflineDataCtrl< T >* jump_ctrl_down_stair_;
    OfflineDataCtrl< T >* jump_ctrl_y_p20_;
    OfflineDataCtrl< T >* jump_ctrl_x_p30_;
    OfflineDataCtrl< T >* jump_ctrl_y_n20_;
    OfflineDataCtrl< T >* jump_ctrl_;

    Vec3< T > kp_swing_        = Vec3< T >::Constant( 40.0 );
    Vec3< T > kd_swing_        = Vec3< T >::Constant( 2.5 );
    Vec3< T > kp_support_      = Vec3< T >::Constant( 30.0 );
    Vec3< T > kd_support_      = Vec3< T >::Constant( 1.0 );
    Vec3< T > kp_land_         = Vec3< T >::Constant( 40.0 );
    Vec3< T > kd_land_         = Vec3< T >::Constant( 2.5 );
    Vec3< T > kp_end_          = Vec3< T >::Constant( 60.0 );
    Vec3< T > kd_end_          = Vec3< T >::Constant( 2.5 );
    Vec3< T > kp_trans_        = Vec3< T >::Constant( 60.0 );
    Vec3< T > kd_trans_        = Vec3< T >::Constant( 2.5 );
    Vec3< T > tau_front_ratio_ = Vec3< T >::Constant( 1.0 );
    Vec3< T > tau_rear_ratio_  = Vec3< T >::Constant( 1.0 );

    // Set Jump ID based on command
    void SetJumpId();
    // Compute joint command
    void ComputeCommand();
    // Reset joint commnand in case of error
    void SafeCommand();
    // Detect touch event
    void TounchDownDetection();
    // Convert offline data to wbc data
    void ConvertData();
};

#endif  // FSM_STATE_JUMP3D_H_
