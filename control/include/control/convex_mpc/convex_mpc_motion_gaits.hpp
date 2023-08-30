#ifndef CONVEX_MPC_MOTION_GAITS_HPP_
#define CONVEX_MPC_MOTION_GAITS_HPP_

#include <cstdio>

#include "control_flags.hpp"
#include "convex_mpc/solve_mpc_interface.hpp"
#include "cpp_types.hpp"
#include "fsm_states/control_fsm_data.hpp"
#include "gait.hpp"
#include "robot_runner.hpp"
#include "trajectory/foot_swing_trajectory.hpp"

using Eigen::Array4f;
using Eigen::Array4i;

class ConvexMpcMotionGaits {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConvexMpcMotionGaits( float dt, int iterations_between_mpc );
    ~ConvexMpcMotionGaits();

    void Initialize( ControlFsmData< float >& data );
    void InitUserGaits( bool file_string );

    template < typename T > void RunMpc( ControlFsmData< T >& data );

    void SolveMpcAnotherThread();

    RobotType robot_type_;

    Vec3< float > vel_cmd_;          // x y yaw
    Vec3< float > pos_cmd_;          // x y z
    Vec3< float > rpy_cmd_;          // roll pitch yaw
    Vec4< float > step_height_cmd_;  // xxx_cmd are command raw data, can be set by RC or motion list

    int  gait_cmd_;               // traget GaitId
    bool omni_cmd_      = false;  // false: vel_cmd_ is with respect to current robot frame
    bool duration_mode_ = false;  // true: cmd comes from motion list

    Vec3< float > body_pos_des_;  // mpc output for wbc
    Vec3< float > body_vel_des_;
    Vec3< float > body_acc_des_;
    Vec3< float > body_rpy_des_;
    Vec3< float > body_omg_des_;

    Vec3< float > foot_pos_des_[ 4 ];
    Vec3< float > foot_vel_des_[ 4 ];
    Vec3< float > foot_acc_des_[ 4 ];
    Vec3< float > foot_force_des_[ 4 ];

    int   current_gait_;
    bool  gait_check_transition_ = true;  // true: gait can be changed
    bool  gait_allow_transition_ = true;  // true: allow gait transition
    float vel_transition_;                // threshold for gait transition

    Vec4< float > contact_states_;  // 0->1 from Gaits.cpp
    Vec4< float > swing_states_;

    Vec6< float > floating_base_weight_wbc_;  // weight of torso in WBC
    float         mu_for_wbc_;
    float         mu_for_mpc_;
    float         landing_gain_;

    Vec3< float > kp_body_for_wbc_;
    Vec3< float > kd_body_for_wbc_;
    Vec3< float > kp_ori_for_wbc_;
    Vec3< float > kd_ori_for_wbc_;
    Vec3< float > kp_foot_for_wbc_;
    Vec3< float > kd_foot_for_wbc_;
    Vec3< float > kp_joint_for_wbc_;
    Vec3< float > kd_joint_for_wbc_;

    SolveMpcInterface* solver_mpc_interface_;
    bool               solve_mpc_by_single_thread_;
    int*               gait_mpc_table_;
    bool               start_solve_mpc_;

private:
    UserParameters*               user_params_;   // read user panel
    RobotControlParameters*       robot_params_;  // params for imu and vel calibration
    const MotionControlCommand*   ctrl_cmd_;      // cmd from RC or motion list
    StateEstimatorResult< float > state_est_;     // state estimation date
    Quadruped< float >*           robot_model_;   // robot kinematics and dynamics params
    Gait*                         gait_type_;     // define gait type
    LegController< float >*       leg_ctrl_;      // leg controller

    StateEstimatorContainer< float >* state_estimator_;
    Vec3< float >                     se_ori_cali_offset_;
    Vec3< float >                     se_ori_cali_gain_;

    int   iter_between_mpc_;  // iterations between two MPC
    int   horizon_length_;    // MPC horizon is horizon_length_ * dt_mpc_
    float dt_;                // time step of WBC
    float dt_mpc_;            // time step of MPC

    int         gait_step_iter_;      // increase every gait period, reset when gait changes
    int         loco_iter_;           // increase every time step, reset when FSM_State_Locomoion enters
    int         gait_iter_;           // increase every time step, reset when gait changes
    Vec4< int > gait_step_iter_leg_;  // increase at leg first swing, rest when gait changes

    int toggle_every_step_;  // toggle between + and - every gait period

    bool gait_first_step_;  // whether it is first step of current gait
    bool gait_first_run_;   // whether it is first time step of locomotion, only set in Initialize()

    Vec4< float > step_height_des_;  // xxx_des are desire for mpc
    float         step_height_max_;
    float         step_height_min_;
    Vec3< float > swing_mid_pos_[ 4 ];  // middle position of feet in swing offset with respect to lifting position
    bool          use_swing_mid_pos_;

    Vec3< float > vel_des_;        // x y z
    Vec3< float > vel_des_robot_;  // x y yaw
    Vec3< float > vel_des_last_;
    Vec3< float > vel_cmd_max_;  // x y yaw
    Vec3< float > vel_cmd_min_;
    Vec3< float > acc_cmd_max_;  // x y yaw
    Vec3< float > acc_cmd_min_;
    Vec3< float > pos_des_;  // x y z
    Vec3< float > pos_des_last_;
    Vec3< float > pos_cmd_max_;
    Vec3< float > pos_cmd_min_;
    Vec3< float > rpy_des_;  // roll pitch yaw
    Vec3< float > rpy_des_last_;
    Vec3< float > rpy_cmd_max_;
    Vec3< float > rpy_cmd_min_;

    float yaw_integral_;  // intergrate yaw error
    float yaw_des_last_;
    float body_height_filter_;
    float body_pitch_filter_;

    Vec3< float > rpy_integral_;  // rpy intergration to eliminate error
    Vec3< float > rpy_comp_;

    Vec3< float > foot_pos_feedback_[ 4 ];  // feedback feet position
    bool          foot_first_swing_[ 4 ];   // whether it is first swing time step of each foot

    Vec4< float > swing_time_;   // total swing time
    Vec4< float > stance_time_;  // total stance time
    Vec4< float > swing_time_remain_;
    Vec3< float > landing_pos_offset_[ 4 ];  // feet landing offset with respect to normal pos
    Vec3< float > landing_pos_ratio_;        // multiply hip position to make feet land inside
    Vec3< float > landing_pos_ratio_last_;
    Vec3< float > landing_pos_[ 4 ];                            // foot landing pos in world frame
    Vec4< float > landing_gain_params_;                         // used in computing landing_ctrl
    const float   leg_sign_x_[ 4 ] = { 1.0, 1.0, -1.0, -1.0 };  // legs have different offset sign
    const float   leg_sign_y_[ 4 ] = { -1.0, 1.0, -1.0, 1.0 };

    Vec4< float > se_contact_states_;  // used in state estimation
    double        trans_state_;        // a threshold to set transition flag

    Mat3< float > kd_joint_for_mpc_;  // mpc leg and body PD
    Mat3< float > kp_leg_swing_for_mpc_, kd_leg_swing_for_mpc_;
    Mat3< float > kp_leg_stance_for_mpc_, kd_leg_stance_for_mpc_;
    Mat3< float > kp_leg_stance_for_mpc_wbc_, kd_leg_stance_for_mpc_wbc_;

    Vec3< float > foot_force_leg_result_[ 4 ];  // contact force mpc result in leg frame
    Vec3< float > foot_force_result_[ 4 ];      // contact force mpc result in world frame

    Vec3< float > max_pos_err_mpc_;  // maximum pos error for mpc

    FootSwingTrajectory< float > foot_swing_trajectory_[ 4 ];
    OffsetDurationGait           special_trot_, special_pronk_, mini_special_pronk_, diagonal_left_, diagonal_right_, trot_swing_, trot_in_out_, trot_pitch_, front_lift_in_, front_lift_left_,
        front_lift_switch_, rear_lift_in_, rear_lift_left_, rear_lift_switch_, ballet_, ballet_trans_, pitch_down_left_, pitch_down_right_, walk_wave_, pace_stride_left_, pace_stride_right_;
    MixedFrequncyGait moonwalk_left_, moonwalk_right_, moonwalk_switch_left_, moonwalk_switch_right_;
    NonPeriodicGait   user_gait_, user_gait_00_, user_gait_01_, user_gait_02_, user_gait_03_, user_gait_04_, user_gait_05_, user_gait_06_, user_gait_07_, user_gait_08_, user_gait_09_, user_gait_10_,
        user_gait_11_, user_gait_12_, user_gait_13_, user_gait_14_, user_gait_15_, user_gait_16_, user_gait_17_, user_gait_18_, user_gait_19_, user_gait_20_;

    bool wbc_user_mpc_traj_gait_select_ = false;
    bool wbc_use_vertical_traj_         = false;

    float comp_integral_;  // variables used in SolveDenseMpc
    float x_comp_integral_;
    float y_comp_integral_;

    float traj_all_[ NUM_MPC_STATE_INPUT * MAX_HORIZON ] = { 0.0 };  // traj used in mpc

    Vec12< float > opt_trajectory_next_;
    Vec12< float > opt_trajectory_pre_;
    int            interpolate_count_;
    int            interpolate_max_num_;

    void SetCurrentGait();
    void AccumulateIteration();

    void ResetIter();  // reset iter

    void SetGaitParams( const int gait_id );  // switch && case for set params
    void SetDefaultParams();
    void SetSpecialTrotParams();
    void SetSpecialPronkParams();
    void SetDiagonalLeftParams();
    void SetDiagonalRightParams();
    void SetTrotSwingParams();
    void SetTrotInOutParams();
    void SetTrotPitchParams();
    void SetWalkWaveParams();
    void SetPaceStrideLeftParams();
    void SetPaceStrideRightParams();
    void SetMoonwalkLeftParams();
    void SetMoonwalkRightParams();
    void SetMoonwalkSwitchLeftParams();
    void SetMoonwalkSwitchRightParams();
    void SetFrontLiftInParams();
    void SetFrontLiftLeftParams();
    void SetFrontLiftSwitchParams();
    void SetRearLiftInParams();
    void SetRearLiftLeftParams();
    void SetRearLiftSwitchParams();
    void SetBalletParams();
    void SetBalletTransParams();
    void SetPitchDownLeftParams();
    void SetPitchDownRightParams();
    void SetUserGait( const int gait_id );

    void SetYawDesire();                                 // set yaw desire based on yaw_dot and integral command to eliminate error
    void SetCmd( ControlFsmData< float >& data );        // set xxx_cmd
    void SetCmdOffset( ControlFsmData< float >& data );  // set cmd offset for gaits

    void CheckGaitFirstStep();  // whether the first step of current gait

    void SetBodyWorldDesire();   // set pos_des_ and vel_des_
    void GetFootPosFeedback();   // to be used in following funs
    void SetGaitFirstRunVars();  // the first time step of locomotion set pos_des_ as state_est_.position
    void SetRpyComp();           // use integration to eliminate roll and pitch errors

    void SetLandingPos();

    void CheckGaitTransition();  // check gait_check_transition_ based on contact_states_ and swing_states_

    void ComputeMpc();
    void UpdateMpc( int* mpc_table );

    void SetLegKpKd();
    void SetFootDesire();  // set swing foot des
    void ComputeSwingTrajectory( int foot_num );
    void SetFootDesireSwing( int foot_num );  // get feet desired pos vel and acc
    void SetFootDesireStance( int foot_num );

    void SetContactStatesForEstimator();

    void SetBodyDesireTrajectory();  // set wbc des

    void DrawResult( ControlFsmData< float >& data );  // draw swing and body traj
    void DrawSwingPath( ControlFsmData< float >& data, int foot_num );
    void DrawStancePath( ControlFsmData< float >& data, int foot_num );
    void DrawMpcPath( ControlFsmData< float >& data );
};

#endif  // CONVEX_MPC_MOTION_GAITS_HPP__
