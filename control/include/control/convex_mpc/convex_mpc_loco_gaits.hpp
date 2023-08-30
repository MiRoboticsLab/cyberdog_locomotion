#ifndef CONVEX_MPC_LOCO_GAITS_HPP_
#define CONVEX_MPC_LOCO_GAITS_HPP_

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

#define TROT_AUTO_SWITCH_NUM 10

class ConvexMpcLocoGaits {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConvexMpcLocoGaits( float dt, int iterations_between_mpc );
    ~ConvexMpcLocoGaits();

    void Initialize( ControlFsmData< float >& data );

    template < typename T > void RunMpc( ControlFsmData< T >& data );
    void                         RunStand();

    void SolveMpcAnotherThread();

    void GetSlopeState( bool& flag ) {
        flag = enable_terrain_comp_;
    };

    RobotType robot_type_;

    Vec3< float > vel_cmd_;  // x y yaw
#ifdef USE_ABSOLUTE_ODOM_FOR_ALL
    Vec3< float > pos_cmd_rel_;  // x y z
#else
    Vec3< float > pos_cmd_;  // x y z
#endif
    Vec3< float > rpy_cmd_;            // roll pitch yaw
    Vec4< float > step_height_cmd_;    // xxx_cmd are command raw data, can be set by RC or motion list
    float         step_height_ratio_;  // Some gaits need large step height

    int  gait_cmd_;               // traget GaitId
    bool omni_cmd_      = false;  // false: vel_cmd is with respect to current robot frame
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
    bool  gait_check_transition_ = true;   // true: gait can be changed
    bool  gait_allow_transition_ = true;   // true: allow gait transition
    bool  ban_exit_loco_         = false;  // ban exiting locomotion because of velocity
    float vel_transition_;                 // threshold for gait transition

    Vec4< float > contact_states_;  // 0->1 from Gaits.cpp
    Vec4< float > swing_states_;

    Vec6< float > floating_base_weight_wbc_;  // weight of torso in WBC

    bool  push_recovery_flag_;  // whether push recovery is needed
    float mu_for_wbc_;
    float mu_for_mpc_;

    Vec3< float > kp_body_for_wbc_;
    Vec3< float > kd_body_for_wbc_;
    Vec3< float > kp_ori_for_wbc_;
    Vec3< float > kd_ori_for_wbc_;
    Vec3< float > kp_foot_for_wbc_;
    Vec3< float > kd_foot_for_wbc_;
    Vec3< float > kp_joint_for_wbc_;
    Vec3< float > kd_joint_for_wbc_;

    bool transform_last_steps_;  // transition motion, for last two steps
    bool transform_first_run_;   // start transition motion, the first run
    int  gait_period_;           // gait period (segmentation)

    SolveMpcInterface* solver_mpc_interface_;
    bool               solve_mpc_by_single_thread_;  // true: solve mpc in another thread
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
    int   horizon_length_;    // MPC horizon is horizon_length_ * dt_mpc
    float dt_;                // time step of WBC
    float dt_mpc_;            // time step of MPC

    float dev_mode_scale_;  // true: enlarge vel_cmd

    bool          gait_cmd_is_passive_trot_;  // whether the gait_cmd is passive trot
    bool          trot_gait_;                 // whether it is a trot gait, used in check gait transition
    bool          auto_switch_gait_;          // wheterh it is a auto switch gait
    bool          auto_switch_enabled_;       // whether apply auto swich
    bool          push_recovery_enabled_;     // whether enable push recovery
    bool          push_recovery_flag_stand_;
    bool          push_recovery_flag_trot_;
    bool          push_recovery_vel_update_;    // vel_cmd_max_/min_ can update
    int           push_recovery_trigger_iter_;  // iter when push recovery is triggered
    Vec3< float > vel_max_ratio_;               // enlarge vel_max for auto switch and push recovery

    Vec3< double > vel_max_for_switch_[ TROT_AUTO_SWITCH_NUM ];  // vel for TrotAutoSwitch
    Vec3< double > vel_min_for_switch_[ TROT_AUTO_SWITCH_NUM ];
    Vec3< double > vel_max_for_switch_cmd_;  // vel for set vel_cmd in auto switch gaits
    Vec3< double > vel_min_for_switch_cmd_;
    const int      gait_id_all_auto_switch_[ TROT_AUTO_SWITCH_NUM ] = { GaitId::kTrot8v3,   GaitId::kTrot10v4,  GaitId::kTrot12v6,  GaitId::kTrot14v8,  GaitId::kTrot16v10,
                                                                   GaitId::kTrot18v11, GaitId::kTrot20v12, GaitId::kTrot22v14, GaitId::kTrot24v16, GaitId::kStand };
    const int      gait_id_all_follow_[ TROT_AUTO_SWITCH_NUM ] = { GaitId::kTrot8v3Follow,   GaitId::kTrot10v4Follow,  GaitId::kTrot12v6Follow,  GaitId::kTrot14v8Follow,  GaitId::kTrot16v10Follow,
                                                              GaitId::kTrot18v11Follow, GaitId::kTrot20v12Follow, GaitId::kTrot20v12Follow, GaitId::kTrot20v12Follow, GaitId::kStand };
    const int*     gait_id_all_;

    int gait_step_iter_;  // increase every gait period, rest when gait changes
    int loco_iter_;       // increase every time step, reset when FSM_State_Locomoion enters
    int gait_iter_;       // increase every time step, reset when gait changes

    bool gait_first_step_;  // whether it is first step of current gait
    bool gait_first_run_;   // whether it is first time step of locomotion, only set in initialize()

    float         step_height_scale_;  // xxx_scale are shrink or enlarge ratio xxx_cmd for xxx_des
    Vec4< float > step_height_des_;    // xxx_des are desire for mpc
    float         step_height_max_;
    float         step_height_min_;

    Vec3< float > vel_des_;        // x y z
    Vec3< float > vel_des_robot_;  // x y yaw
    Vec3< float > vel_des_last_;
    Vec3< float > vel_cmd_scale_;
    Vec3< float > vel_cmd_max_;  // x y yaw
    Vec3< float > vel_cmd_min_;
    Vec3< float > acc_cmd_scale_;  // x y yaw
    Vec3< float > acc_cmd_max_;
    Vec3< float > acc_cmd_min_;
    Vec3< float > vel_cmd_max_last_;  // record last vel_cmd_max_/min_ and is used for push recovery
    Vec3< float > vel_cmd_min_last_;
    Vec3< float > vel_cmd_zero_offset_;  // x y yaw, velocity offset to keep it stay in place
#ifdef USE_ABSOLUTE_ODOM_FOR_ALL
    Vec3< float > pos_des_rel_;  // x y z
    Vec3< float > pos_des_rel_last_;
    Vec3< float > pos_cmd_rel_scale_;
    Vec3< float > pos_cmd_rel_max_;
    Vec3< float > pos_cmd_rel_min_;
    Vec3< float > pos_des_;  // x y z global
#else
    Vec3< float > pos_des_;  // x y z
    Vec3< float > pos_des_last_;
    Vec3< float > pos_cmd_max_;
    Vec3< float > pos_cmd_min_;
#endif
    Vec3< float > init_pos_des_;  // initial pos/rpy for STAND
    Vec3< float > init_rpy_des_;
    Vec3< float > rpy_des_;  // roll pitch yaw
    Vec3< float > rpy_cmd_scale_;
    Vec3< float > rpy_cmd_max_;
    Vec3< float > rpy_cmd_min_;
    Vec3< float > vel_est_avg_;  // x y yaw, average/sum of estimated vel
    Vec3< float > vel_est_sum_;

    bool yaw_absolute_ = false;  // whether yaw desire is absoulte or relative

    float vel_buffer_in_x_[ 3 ];  // used in passive trot
    float vel_buffer_in_y_[ 3 ];
    float vel_buffer_in_yaw_[ 3 ];
    float vel_buffer_out_x_[ 3 ];
    float vel_buffer_out_y_[ 3 ];
    float vel_buffer_out_yaw_[ 3 ];

    float yaw_integral_;  // intergrate yaw error
    float yaw_des_last_;
    float body_height_delta_max_;      // body height changes based on x vel
    float body_height_delta_vel_max_;  // the max vel for to body_height_delta_max_
    float body_height_filter_;

    Vec3< float > rpy_integral_;  // rpy intergration to eliminate error
    Vec3< float > rpy_comp_;

    Vec3< float > foot_pos_feedback_[ 4 ];  // feedback feet position
    bool          foot_first_swing_[ 4 ];   // whether it is first swing time step of each foot

    Vec4< float > swing_time_;               // total swing time
    Vec4< float > stance_time_;              // total stance time
    Vec4< float > swing_time_remain_;        // used in computing landing position
    Vec3< float > landing_pos_offset_[ 4 ];  // feet landing offset with respect to normal pos
    Vec3< float > landing_pos_ratio_;        // multiply hip position to make feet land inside
    Vec3< float > landing_pos_ratio_last_;
    Vec3< float > landing_pos_[ 4 ];          // foot landing pos in world frame
    Vec4< float > landing_gain_params_;       // used in computing landing_ctrl
    float         landing_pos_offset_scale_;  // reset landing_pos_offset_ in transition()

    Vec4< float > se_contact_states_;  // used in state estimation
    Vec4< float > se_contact_states_stand_;
    double        trans_state_;  // a threshold to set transition flag

    Mat3< float > kd_joint_for_mpc_;  // those leg cartesian PD are used for only mpc without wbc
    Mat3< float > kp_leg_swing_for_mpc_, kd_leg_swing_for_mpc_;
    Mat3< float > kp_leg_stance_for_mpc_, kd_leg_stance_for_mpc_;
    Mat3< float > kp_leg_stance_for_mpc_wbc_, kd_leg_stance_for_mpc_wbc_;  // those leg cartesianl PD are used for mpc with wbc

    Vec3< float > foot_force_leg_result_[ 4 ];  // contact force mpc result in leg frame
    Vec3< float > foot_force_result_[ 4 ];      // contact force mpc result in world frame

    Vec3< float > max_pos_err_mpc_;  // maximum pos error for mpc

    FootSwingTrajectory< float > foot_swing_trajectory_[ 4 ];
    OffsetDurationGait trot_medium_, trot_fast_, trot_slow_, bound_, pronk_, walk_forward_, walk_backward_, pace_, passive_trot_, trot_24_16_, trot_22_14_, trot_20_12_, trot_18_11_, trot_16_10_,
        trot_14_8_, trot_12_6_, trot_10_5_, trot_10_4_, trot_8_3_, stand_, stand_no_pr_, mini_pronk_;

    float comp_integral_;  // intergrate position error to make robot step in place
    float x_comp_integral_;
    float y_comp_integral_;

    float traj_all_[ NUM_MPC_STATE_INPUT * MAX_HORIZON ] = { 0.0 };  // traj used in mpc

    Vec12< float > opt_trajectory_next_;  // next and previous mpc opt trajectory
    Vec12< float > opt_trajectory_pre_;
    int            interpolate_count_;  // interpolate mpc trajectory for wbc
    int            interpolate_max_num_;

    Vec3< float > rpy_des_with_comp_;
    Vec3< float > est_terrain_coef_;  // terrain equation coefficients from state estimatior
    Mat3< float > est_terrain_rot_matrix_;
    bool          enable_terrain_comp_;                        // true: enable terrain compensation, false: disable
    float         slope_up_threshold_, slope_down_threshold_;  // threshold to trigger terrain compensation
    int           slope_threshold_num_;
    int           heading_direction_on_slope_;  // 1 forward, -1 backward, 2 right, -2 left
    float         slope_;                       // slope of surface

    bool use_quick_break_mode_;  // true is use, flase is not

    int walk_direction_;  // 0 forward, 1 backward

    void SetCurrentGait();
    void AccumulateIteration();

    void SetPushRecoveryFlag();  // check whether push revoery is triggered
    void TrotAutoSwitch();       // gait auto switch based on vel and push_recovery_flag_, loco_iter_ is reset here
    void PassiveTrotAutoSwitch();
    void SwitchByPercentage();  // switch trot based on velocity percentage
    void SwitchByAbsolute();    // switch trot based on velocity absolute value
    bool VelVectorCompare( const int gait_id, Vec3< float > vel_cmd, Vec3< double > vel_max1, Vec3< double > vel_max2, Vec3< double > vel_min1, Vec3< double > vel_min2 );
    int  SearchGaitIdAutoSwitch( const int gait_id, const int gait_id_all[ TROT_AUTO_SWITCH_NUM ] );
    int  GaitCmdSelect( const int gait_id, const int current_gait, Vec3< double > vel_max, Vec3< double > vel_min );  // select gait between current_gait and target gait
    int  GaitCmdSelectFromAll( const int gait_id, const int gait_id_all[ TROT_AUTO_SWITCH_NUM ], Vec3< double > vel_max[ TROT_AUTO_SWITCH_NUM ], Vec3< double > vel_min[ TROT_AUTO_SWITCH_NUM ] );
    void SetVelMaxMinForSwitch();       // set vel_max/min_for_switch
    void SwitchByAbsoluteAndGaitCmd();  // consider current gait_cmd_ based on SwitchByAbsolute()
    bool VelVectorCompare( const int gait_id, const int gait_cmd, Vec3< float > vel_cmd, Vec3< double > vel_max1, Vec3< double > vel_max2, Vec3< double > vel_min1, Vec3< double > vel_min2 );
    void WalkAutoSwitch();            // set different gait scheduler according to motion direction
    void ExitLocomotionAutoSwitch();  // before exiting locomotion, reset landing offset to avoid feet sliding on ground
    void ResetIter();                 // reset iter

    void SetGaitParams( const int gait_id );  // switch && case for set params
    void SetDefaultParams();
    void SetTrotMediumParams();
    void SetTrotFastParams();
    void SetTrotSlowParams();
    void SetBoundParams();
    void SetPronkParams();
    void SetWalkParams();
    void SetPaceParams();
    void SetPassiveTrotParams();
    void SetStandParams();
    void SetStandNoPushRecoveryParams();
    void SetTrot24v16Params();
    void SetTrot22v14Params();
    void SetTrot20v12Params();
    void SetTrot18v11Params();
    void SetTrot16v10Params();
    void SetTrot14v8Params();
    void SetTrot12v6Params();
    void SetTrot10v5Params();
    void SetTrot10v4Params();
    void SetTrot8v3Params();
    void SetTrot24v16FollowParams();
    void SetTrot22v14FollowParams();
    void SetTrot20v12FollowParams();
    void SetTrot18v11FollowParams();
    void SetTrot16v10FollowParams();
    void SetTrot14v8FollowParams();
    void SetTrot12v6FollowParams();
    void SetTrot10v4FollowParams();
    void SetTrot8v3FollowParams();

    void SetCmd( ControlFsmData< float >& data );        // set xxx_cmd
    void SetCmdOffset( ControlFsmData< float >& data );  // set cmd offset for gaits
    void SetCmdScale();                                  // set xxx_scale
    void SetAccCmdScale();                               // set acc cmd scale
    void SetVelDesireRobot();                            // set vel_des_robot_ based on GaitId
    void SetYawDesire();                                 // set yaw desire based on yaw_dot and integration to eliminate error
    void SetVelCompOnSlope();                            // set literal vel compensation on slope

    void CheckGaitFirstStep();  // whether the first step of current gait

    void SetBodyWorldDesire();   // set pos_des_ and vel_des_
    void GetFootPosFeedback();   // to be used in following funs
    void SetGaitFirstRunVars();  // the first time step of locomotion set pos_des_ as state_est_.position
    void SetRpyComp();           // use integration to eliminate roll and pitch errors

    void SetTerrainComp();  // set rpy compensation based on terrain information

    void SetLandingPos();
    void SetLandingOffsetByVel();  // large y and yaw vel disables lateral landing offsets

    void CheckGaitTransition();  // check gait_check_transition_ based on contact_states_ and swing_states_

    void ComputeVelAverage();  // compute average velocity in one step

    void ComputeMpc();
    void UpdateMpc( int* mpc_table );

    void SetLegKpKd();
    void SetFootDesire();  // set swing foot des
    void ComputeSwingTrajectory( int foot_num );
    void SetFootDesireSwing( int foot_num );  // get feet desired pos vel and acc
    void SetFootDesireStance( int foot_num );

    void SetBodyDesireTrajectory();  // set wbc des

    void DrawResult( ControlFsmData< float >& data );  // draw swing and body traj
    void DrawSwingPath( ControlFsmData< float >& data, int foot_num );
    void DrawStancePath( ControlFsmData< float >& data, int foot_num );
    void DrawMpcPath( ControlFsmData< float >& data );
};

#endif  // CONVEX_MPC_LOCO_GAITS_HPP_
