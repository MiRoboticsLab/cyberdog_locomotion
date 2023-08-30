#ifndef USER_PARAMETERS_HPP_
#define USER_PARAMETERS_HPP_

#include "control_parameters/control_parameters.hpp"

class UserParameters : public ControlParameters {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    UserParameters()
        : ControlParameters( "user-parameters" ),

          INIT_PARAMETER( mpc_body_inertia ), INIT_PARAMETER( mpc_body_mass ), INIT_PARAMETER( mpc_task_weight ), INIT_PARAMETER( mpc_friction_coef ), INIT_PARAMETER( mpc_friction_coef_bound ),
          INIT_PARAMETER( mpc_friction_coef_follow ), INIT_PARAMETER( mpc_force_max ), INIT_PARAMETER( mpc_com_offset ),

          INIT_PARAMETER( mpc_only_joint_kd ), INIT_PARAMETER( mpc_only_swing_cartesian_kp ), INIT_PARAMETER( mpc_only_swing_cartesian_kd ), INIT_PARAMETER( mpc_only_stance_cartesian_kp ),
          INIT_PARAMETER( mpc_only_stance_cartesian_kd ),

          INIT_PARAMETER( mpc_wbc_stance_cartesian_kp ), INIT_PARAMETER( mpc_wbc_stance_cartesian_kd ),

          INIT_PARAMETER( mpc_velocity_filter ), INIT_PARAMETER( mpc_use_bezier ),

          INIT_PARAMETER( cmpc_x_drag ), INIT_PARAMETER( cmpc_bonus_swing ), INIT_PARAMETER( cmpc_bonus_swing_large ), INIT_PARAMETER( cmpc_bonus_swing_follow ),

          INIT_PARAMETER( stance_legs ), INIT_PARAMETER( use_jcqp ), INIT_PARAMETER( jcqp_max_iter ), INIT_PARAMETER( jcqp_rho ), INIT_PARAMETER( jcqp_sigma ), INIT_PARAMETER( jcqp_alpha ),
          INIT_PARAMETER( jcqp_terminate ),

          INIT_PARAMETER( use_wbc ), INIT_PARAMETER( wbc_body_kp ), INIT_PARAMETER( wbc_body_kd ), INIT_PARAMETER( wbc_orient_kp ), INIT_PARAMETER( wbc_orient_kd ), INIT_PARAMETER( wbc_foot_kp ),
          INIT_PARAMETER( wbc_foot_kd ), INIT_PARAMETER( wbc_joint_kp ), INIT_PARAMETER( wbc_joint_kd ), INIT_PARAMETER( wbc_use_mpc_traj ), INIT_PARAMETER( wbc_use_mpc_traj_motion ),
          INIT_PARAMETER( wbc_friction_default ), INIT_PARAMETER( wbc_friction_bound ), INIT_PARAMETER( wbc_friction_front_lift ), INIT_PARAMETER( wbc_friction_follow ),

          INIT_PARAMETER( wbc_weight ), INIT_PARAMETER( wbc_weight_default ), INIT_PARAMETER( wbc_weight_walk ), INIT_PARAMETER( wbc_weight_passive_trot ), INIT_PARAMETER( wbc_weight_motion_default ),
          INIT_PARAMETER( wbc_weight_ballet ), INIT_PARAMETER( wbc_weight_pitch_step ), INIT_PARAMETER( wbc_weight_moonwalk ), INIT_PARAMETER( wbc_weight_diagonal ),
          INIT_PARAMETER( wbc_weight_front_lift ), INIT_PARAMETER( wbc_weight_rear_lift ), INIT_PARAMETER( wbc_weight_walk_wave ), INIT_PARAMETER( wbc_weight_jump ),
          INIT_PARAMETER( wbc_weight_pace_stride ), INIT_PARAMETER( wbc_weight_user_gait ),

          INIT_PARAMETER( gait_type ), INIT_PARAMETER( gait_period_time ), INIT_PARAMETER( gait_switching_phase ), INIT_PARAMETER( gait_override ), INIT_PARAMETER( gait_max_leg_angle ),
          INIT_PARAMETER( gait_max_stance_time ), INIT_PARAMETER( gait_min_stance_time ), INIT_PARAMETER( des_roll_pitch_height ), INIT_PARAMETER( des_roll_pitch_height_motion ),
          INIT_PARAMETER( des_roll_pitch_height_stair ), INIT_PARAMETER( des_vel ), INIT_PARAMETER( swing_p_gain ), INIT_PARAMETER( swing_p_gain_large ), INIT_PARAMETER( swing_p_gain_motion ),
          INIT_PARAMETER( swing_p_gain_follow ), INIT_PARAMETER( foot_final_height ),

          INIT_PARAMETER( contact_threshold ), INIT_PARAMETER( detect_terrain ), INIT_PARAMETER( body_size_port ),

          INIT_PARAMETER( jump_initial_pos ), INIT_PARAMETER( jump_action_order ), INIT_PARAMETER( jump_height ), INIT_PARAMETER( jump_vel ), INIT_PARAMETER( jump_rot ), INIT_PARAMETER( jump_qkp ),
          INIT_PARAMETER( jump_qkd ),

          INIT_PARAMETER( jump_force ), INIT_PARAMETER( knee_angle_adj ),

          INIT_PARAMETER( action_pause ),

          INIT_PARAMETER( rpy_acc_max ), INIT_PARAMETER( rpy_w_max ), INIT_PARAMETER( rpy_min ), INIT_PARAMETER( rpy_max ),

          INIT_PARAMETER( enable_push_recovery_vel_trot ), INIT_PARAMETER( enable_push_recovery_vel_stand ), INIT_PARAMETER( enable_push_recovery_pos_stand ),
          INIT_PARAMETER( enable_push_recovery_delay ), INIT_PARAMETER( disable_push_recovery_vel_trot ), INIT_PARAMETER( disable_push_recovery_vel_stand ),
          INIT_PARAMETER( disable_push_recovery_delay ), INIT_PARAMETER( vel_max_ratio ),

          INIT_PARAMETER( x_effect_scale_pos ), INIT_PARAMETER( x_effect_scale_neg ), INIT_PARAMETER( y_effect_scale ), INIT_PARAMETER( yaw_effect_scale ), INIT_PARAMETER( centrifugal_thresh ),
          INIT_PARAMETER( vel_scale_limit_type ),

          INIT_PARAMETER( vel_switch ), INIT_PARAMETER( auto_switch_enabled ), INIT_PARAMETER( push_recovery_enabled ),

          INIT_PARAMETER( vel_xy_yaw_max_stair ), INIT_PARAMETER( vel_xy_yaw_min_stair ), INIT_PARAMETER( vel_xy_yaw_max_default ), INIT_PARAMETER( vel_xy_yaw_min_default ),
          INIT_PARAMETER( vel_xy_yaw_max_trot_fast ), INIT_PARAMETER( vel_xy_yaw_min_trot_fast ), INIT_PARAMETER( vel_xy_yaw_max_trot_medium ), INIT_PARAMETER( vel_xy_yaw_min_trot_medium ),
          INIT_PARAMETER( vel_xy_yaw_max_trot_slow ), INIT_PARAMETER( vel_xy_yaw_min_trot_slow ), INIT_PARAMETER( vel_xy_yaw_max_bound ), INIT_PARAMETER( vel_xy_yaw_min_bound ),
          INIT_PARAMETER( vel_xy_yaw_max_pronk ), INIT_PARAMETER( vel_xy_yaw_min_pronk ), INIT_PARAMETER( vel_xy_yaw_max_walk ), INIT_PARAMETER( vel_xy_yaw_min_walk ),
          INIT_PARAMETER( vel_xy_yaw_max_passive_trot ), INIT_PARAMETER( vel_xy_yaw_min_passive_trot ), INIT_PARAMETER( vel_xy_yaw_max_trot_8_3 ), INIT_PARAMETER( vel_xy_yaw_min_trot_8_3 ),
          INIT_PARAMETER( vel_xy_yaw_max_trot_10_4 ), INIT_PARAMETER( vel_xy_yaw_min_trot_10_4 ), INIT_PARAMETER( vel_xy_yaw_max_trot_10_5 ), INIT_PARAMETER( vel_xy_yaw_min_trot_10_5 ),
          INIT_PARAMETER( vel_xy_yaw_max_trot_12_6 ), INIT_PARAMETER( vel_xy_yaw_min_trot_12_6 ), INIT_PARAMETER( vel_xy_yaw_max_trot_14_8 ), INIT_PARAMETER( vel_xy_yaw_min_trot_14_8 ),
          INIT_PARAMETER( vel_xy_yaw_max_trot_16_10 ), INIT_PARAMETER( vel_xy_yaw_min_trot_16_10 ), INIT_PARAMETER( vel_xy_yaw_max_trot_18_11 ), INIT_PARAMETER( vel_xy_yaw_min_trot_18_11 ),
          INIT_PARAMETER( vel_xy_yaw_max_trot_20_12 ), INIT_PARAMETER( vel_xy_yaw_min_trot_20_12 ), INIT_PARAMETER( vel_xy_yaw_max_trot_22_14 ), INIT_PARAMETER( vel_xy_yaw_min_trot_22_14 ),
          INIT_PARAMETER( vel_xy_yaw_max_trot_24_16 ), INIT_PARAMETER( vel_xy_yaw_min_trot_24_16 ), INIT_PARAMETER( vel_xy_yaw_max_motion_default ), INIT_PARAMETER( vel_xy_yaw_min_motion_default ),
          INIT_PARAMETER( vel_xy_yaw_max_vision ), INIT_PARAMETER( vel_xy_yaw_min_vision ),

          INIT_PARAMETER( vel_xy_yaw_zero_bound ), INIT_PARAMETER( vel_xy_yaw_zero_pronk ),

          INIT_PARAMETER( acc_xy_yaw_max ), INIT_PARAMETER( acc_xy_yaw_min ), INIT_PARAMETER( acc_xy_yaw_max_bound ), INIT_PARAMETER( acc_xy_yaw_min_bound ), INIT_PARAMETER( acc_xy_yaw_max_follow ),
          INIT_PARAMETER( acc_xy_yaw_min_follow ), INIT_PARAMETER( vel_xy_yaw_scale ), INIT_PARAMETER( step_height_max ), INIT_PARAMETER( x_offset_pronk ), INIT_PARAMETER( x_offset_bound ),
          INIT_PARAMETER( x_offset_default ), INIT_PARAMETER( x_offset_trot_10_4 ), INIT_PARAMETER( y_offset_trot_10_4 ), INIT_PARAMETER( y_offset_trot ),

          INIT_PARAMETER( walk_offset_x ), INIT_PARAMETER( walk_offset_y ),

          INIT_PARAMETER( diagonal_landing_offset ), INIT_PARAMETER( diagonal_yaw_rate ),

          INIT_PARAMETER( trot_swing_middle_pos ),

          INIT_PARAMETER( trot_in_out_landing_offset ),

          INIT_PARAMETER( trot_pitch_landing_offset ), INIT_PARAMETER( trot_pitch_up_down ),

          INIT_PARAMETER( moonwalk_step_height ), INIT_PARAMETER( moonwalk_landing_offset ),

          INIT_PARAMETER( front_lift_landing_offset ), INIT_PARAMETER( front_lift_height_pitch ), INIT_PARAMETER( front_lift_switch_landing_offset ), INIT_PARAMETER( front_lift_switch_middle_pos ),

          INIT_PARAMETER( rear_lift_landing_offset ), INIT_PARAMETER( rear_lift_height_pitch ), INIT_PARAMETER( rear_lift_switch_landing_offset ), INIT_PARAMETER( rear_lift_switch_middle_pos ),

          INIT_PARAMETER( ballet_step_height ), INIT_PARAMETER( ballet_landing_offset ), INIT_PARAMETER( ballet_height ), INIT_PARAMETER( ballet_trans_step_height ),

          INIT_PARAMETER( pitch_step_landing_offset ), INIT_PARAMETER( pitch_step_pitch ), INIT_PARAMETER( pitch_step_height ),

          INIT_PARAMETER( walk_wave_landing_offset ), INIT_PARAMETER( walk_wave_pitch ), INIT_PARAMETER( walk_wave_height ),

          INIT_PARAMETER( pace_stride_vel_cmd ), INIT_PARAMETER( pace_stride_landing_offset_y ),

          INIT_PARAMETER( special_pronk_height ), INIT_PARAMETER( special_pronk_size ), INIT_PARAMETER( special_trot_landing_offset ), INIT_PARAMETER( upstairs_height_cmd ),
          INIT_PARAMETER( downstairs_height_cmd ), INIT_PARAMETER( rc_delta_step_height ), INIT_PARAMETER( downstairs_depth ), INIT_PARAMETER( jump_time_vector ), INIT_PARAMETER( jump_front_hip ),
          INIT_PARAMETER( jump_front_knee ), INIT_PARAMETER( jump_hind_hip ), INIT_PARAMETER( jump_hind_knee ),

          INIT_PARAMETER( use_energy_saving_mode ), INIT_PARAMETER( motor_sdk_position_mutation_limit ),

          INIT_PARAMETER( vel_xy_yaw_max_pronk_rl ), INIT_PARAMETER( vel_xy_yaw_min_pronk_rl ), INIT_PARAMETER( acc_xy_yaw_max_pronk_rl ), INIT_PARAMETER( acc_xy_yaw_min_pronk_rl ),
          INIT_PARAMETER( x_offset_pronk_rl ), INIT_PARAMETER( y_offset_pronk_rl ), INIT_PARAMETER( yaw_offset_pronk_rl ),

          INIT_PARAMETER( vel_xy_yaw_max_rapid_rl ), INIT_PARAMETER( vel_xy_yaw_min_rapid_rl ), INIT_PARAMETER( acc_xy_yaw_max_rapid_rl ), INIT_PARAMETER( acc_xy_yaw_min_rapid_rl ),
          INIT_PARAMETER( x_offset_rapid_rl ), INIT_PARAMETER( y_offset_rapid_rl ), INIT_PARAMETER( yaw_offset_rapid_rl ),

          INIT_PARAMETER( skateboard_height ), INIT_PARAMETER( skateboard_velocity ), INIT_PARAMETER( skateboard_body_pos_balance ), INIT_PARAMETER( skateboard_body_pos_skate ),
          INIT_PARAMETER( skateboard_body_pos_swing_in_skate ), INIT_PARAMETER( skateboard_thrust_force ), INIT_PARAMETER( skateboard_thrust_force_first ),
          INIT_PARAMETER( skateboard_foot_swing_final_offset )

    {}
    // MPC related parameters
    DECLARE_PARAMETER( Vec3< double >, mpc_body_inertia );
    DECLARE_PARAMETER( double, mpc_body_mass );
    DECLARE_PARAMETER( Vec12< double >, mpc_task_weight );
    DECLARE_PARAMETER( double, mpc_friction_coef );
    DECLARE_PARAMETER( double, mpc_friction_coef_bound );
    DECLARE_PARAMETER( double, mpc_friction_coef_follow );
    DECLARE_PARAMETER( double, mpc_force_max );
    DECLARE_PARAMETER( Vec3< double >, mpc_com_offset );

    DECLARE_PARAMETER( Vec3< double >, mpc_only_joint_kd );
    DECLARE_PARAMETER( Vec3< double >, mpc_only_swing_cartesian_kp );
    DECLARE_PARAMETER( Vec3< double >, mpc_only_swing_cartesian_kd );
    DECLARE_PARAMETER( Vec3< double >, mpc_only_stance_cartesian_kp );
    DECLARE_PARAMETER( Vec3< double >, mpc_only_stance_cartesian_kd );

    DECLARE_PARAMETER( Vec3< double >, mpc_wbc_stance_cartesian_kp );
    DECLARE_PARAMETER( Vec3< double >, mpc_wbc_stance_cartesian_kd );

    DECLARE_PARAMETER( double, mpc_velocity_filter );
    DECLARE_PARAMETER( double, mpc_use_bezier );

    DECLARE_PARAMETER( double, cmpc_x_drag );
    DECLARE_PARAMETER( double, cmpc_bonus_swing );
    DECLARE_PARAMETER( double, cmpc_bonus_swing_large );
    DECLARE_PARAMETER( double, cmpc_bonus_swing_follow );

    DECLARE_PARAMETER( double, stance_legs );

    DECLARE_PARAMETER( double, use_jcqp );
    DECLARE_PARAMETER( double, jcqp_max_iter );
    DECLARE_PARAMETER( double, jcqp_rho );
    DECLARE_PARAMETER( double, jcqp_sigma );
    DECLARE_PARAMETER( double, jcqp_alpha );
    DECLARE_PARAMETER( double, jcqp_terminate );

    // WBC related Kp, Kd
    DECLARE_PARAMETER( double, use_wbc );
    DECLARE_PARAMETER( Vec3< double >, wbc_body_kp );
    DECLARE_PARAMETER( Vec3< double >, wbc_body_kd );
    DECLARE_PARAMETER( Vec3< double >, wbc_orient_kp );
    DECLARE_PARAMETER( Vec3< double >, wbc_orient_kd );
    DECLARE_PARAMETER( Vec3< double >, wbc_foot_kp );
    DECLARE_PARAMETER( Vec3< double >, wbc_foot_kd );
    DECLARE_PARAMETER( Vec3< double >, wbc_joint_kp );
    DECLARE_PARAMETER( Vec3< double >, wbc_joint_kd );
    DECLARE_PARAMETER( double, wbc_use_mpc_traj );
    DECLARE_PARAMETER( double, wbc_use_mpc_traj_motion );
    DECLARE_PARAMETER( double, wbc_friction_default );
    DECLARE_PARAMETER( double, wbc_friction_bound );
    DECLARE_PARAMETER( double, wbc_friction_front_lift );
    DECLARE_PARAMETER( double, wbc_friction_follow );

    DECLARE_PARAMETER( Vec6< double >, wbc_weight );
    DECLARE_PARAMETER( Vec6< double >, wbc_weight_default );
    DECLARE_PARAMETER( Vec6< double >, wbc_weight_walk );
    DECLARE_PARAMETER( Vec6< double >, wbc_weight_passive_trot );
    DECLARE_PARAMETER( Vec6< double >, wbc_weight_motion_default );
    DECLARE_PARAMETER( Vec6< double >, wbc_weight_ballet );
    DECLARE_PARAMETER( Vec6< double >, wbc_weight_pitch_step );
    DECLARE_PARAMETER( Vec6< double >, wbc_weight_moonwalk );
    DECLARE_PARAMETER( Vec6< double >, wbc_weight_diagonal );
    DECLARE_PARAMETER( Vec6< double >, wbc_weight_front_lift );
    DECLARE_PARAMETER( Vec6< double >, wbc_weight_rear_lift );
    DECLARE_PARAMETER( Vec6< double >, wbc_weight_walk_wave );
    DECLARE_PARAMETER( Vec6< double >, wbc_weight_jump );
    DECLARE_PARAMETER( Vec6< double >, wbc_weight_pace_stride );
    DECLARE_PARAMETER( Vec6< double >, wbc_weight_user_gait );

    // Gait Scheduler
    DECLARE_PARAMETER( double, gait_type );
    DECLARE_PARAMETER( double, gait_period_time );
    DECLARE_PARAMETER( double, gait_switching_phase );
    DECLARE_PARAMETER( double, gait_override );
    DECLARE_PARAMETER( double, gait_max_leg_angle );
    DECLARE_PARAMETER( double, gait_max_stance_time );
    DECLARE_PARAMETER( double, gait_min_stance_time );
    DECLARE_PARAMETER( Vec3< double >, des_roll_pitch_height );
    DECLARE_PARAMETER( Vec3< double >, des_roll_pitch_height_motion );
    DECLARE_PARAMETER( Vec3< double >, des_roll_pitch_height_stair );
    DECLARE_PARAMETER( Vec3< double >, des_vel );
    DECLARE_PARAMETER( Vec3< double >, swing_p_gain );
    DECLARE_PARAMETER( Vec3< double >, swing_p_gain_large );
    DECLARE_PARAMETER( Vec3< double >, swing_p_gain_motion );
    DECLARE_PARAMETER( Vec3< double >, swing_p_gain_follow );
    DECLARE_PARAMETER( double, foot_final_height );

    DECLARE_PARAMETER( Vec3< double >, contact_threshold );
    DECLARE_PARAMETER( double, detect_terrain );
    DECLARE_PARAMETER( double, body_size_port );

    DECLARE_PARAMETER( Vec3< double >, jump_initial_pos );
    DECLARE_PARAMETER( Vec3< double >, jump_action_order );
    DECLARE_PARAMETER( Vec3< double >, jump_height );
    DECLARE_PARAMETER( Vec3< double >, jump_vel );
    DECLARE_PARAMETER( Vec3< double >, jump_rot );
    DECLARE_PARAMETER( Vec3< double >, jump_qkp );
    DECLARE_PARAMETER( Vec3< double >, jump_qkd );

    DECLARE_PARAMETER( double, jump_force );
    DECLARE_PARAMETER( double, knee_angle_adj );

    DECLARE_PARAMETER( double, action_pause );

    // For QP Stand rpy limit
    DECLARE_PARAMETER( Vec3< double >, rpy_acc_max );
    DECLARE_PARAMETER( Vec3< double >, rpy_w_max );
    DECLARE_PARAMETER( Vec3< double >, rpy_min );
    DECLARE_PARAMETER( Vec3< double >, rpy_max );

    DECLARE_PARAMETER( Vec2< double >, enable_push_recovery_vel_trot );
    DECLARE_PARAMETER( Vec2< double >, enable_push_recovery_vel_stand );
    DECLARE_PARAMETER( Vec2< double >, enable_push_recovery_pos_stand );
    DECLARE_PARAMETER( double, enable_push_recovery_delay );
    DECLARE_PARAMETER( Vec2< double >, disable_push_recovery_vel_trot );
    DECLARE_PARAMETER( Vec2< double >, disable_push_recovery_vel_stand );
    DECLARE_PARAMETER( double, disable_push_recovery_delay );
    DECLARE_PARAMETER( Vec3< double >, vel_max_ratio );

    DECLARE_PARAMETER( double, x_effect_scale_pos );
    DECLARE_PARAMETER( double, x_effect_scale_neg );
    DECLARE_PARAMETER( double, y_effect_scale );
    DECLARE_PARAMETER( Vec3< double >, yaw_effect_scale );
    DECLARE_PARAMETER( double, centrifugal_thresh );
    DECLARE_PARAMETER( double, vel_scale_limit_type );

    DECLARE_PARAMETER( Vec12< double >, vel_switch );
    DECLARE_PARAMETER( double, auto_switch_enabled );
    DECLARE_PARAMETER( double, push_recovery_enabled );

    // all limits about locomotion
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_stair );  // in order: [x, y, yaw]
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_stair );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_default );  // in order: [x, y, yaw]
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_default );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_trot_fast );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_trot_fast );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_trot_medium );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_trot_medium );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_trot_slow );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_trot_slow );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_bound );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_bound );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_pronk );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_pronk );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_walk );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_walk );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_passive_trot );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_passive_trot );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_trot_8_3 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_trot_8_3 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_trot_10_4 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_trot_10_4 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_trot_10_5 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_trot_10_5 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_trot_12_6 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_trot_12_6 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_trot_14_8 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_trot_14_8 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_trot_16_10 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_trot_16_10 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_trot_18_11 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_trot_18_11 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_trot_20_12 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_trot_20_12 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_trot_22_14 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_trot_22_14 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_trot_24_16 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_trot_24_16 );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_motion_default );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_motion_default );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_vision );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_vision );

    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_zero_bound );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_zero_pronk );

    DECLARE_PARAMETER( Vec3< double >, acc_xy_yaw_max );
    DECLARE_PARAMETER( Vec3< double >, acc_xy_yaw_min );
    DECLARE_PARAMETER( Vec3< double >, acc_xy_yaw_max_bound );
    DECLARE_PARAMETER( Vec3< double >, acc_xy_yaw_min_bound );
    DECLARE_PARAMETER( Vec3< double >, acc_xy_yaw_max_follow );
    DECLARE_PARAMETER( Vec3< double >, acc_xy_yaw_min_follow );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_scale );

    DECLARE_PARAMETER( double, step_height_max );

    DECLARE_PARAMETER( double, x_offset_pronk );
    DECLARE_PARAMETER( Vec4< double >, x_offset_bound );
    DECLARE_PARAMETER( Vec4< double >, x_offset_default );
    DECLARE_PARAMETER( Vec4< double >, x_offset_trot_10_4 );
    DECLARE_PARAMETER( Vec4< double >, y_offset_trot_10_4 );
    DECLARE_PARAMETER( Vec4< double >, y_offset_trot );

    DECLARE_PARAMETER( Vec4< double >, walk_offset_x );
    DECLARE_PARAMETER( Vec4< double >, walk_offset_y );

    DECLARE_PARAMETER( Vec2< double >, diagonal_landing_offset );
    DECLARE_PARAMETER( Vec2< double >, diagonal_yaw_rate );

    DECLARE_PARAMETER( Vec3< double >, trot_swing_middle_pos );

    DECLARE_PARAMETER( Vec2< double >, trot_in_out_landing_offset );

    DECLARE_PARAMETER( Vec2< double >, trot_pitch_landing_offset );
    DECLARE_PARAMETER( Vec2< double >, trot_pitch_up_down );

    DECLARE_PARAMETER( Vec2< double >, moonwalk_step_height );
    DECLARE_PARAMETER( Vec2< double >, moonwalk_landing_offset );

    DECLARE_PARAMETER( Vec4< double >, front_lift_landing_offset );
    DECLARE_PARAMETER( Vec2< double >, front_lift_height_pitch );
    DECLARE_PARAMETER( Vec2< double >, front_lift_switch_landing_offset );
    DECLARE_PARAMETER( Vec3< double >, front_lift_switch_middle_pos );

    DECLARE_PARAMETER( Vec4< double >, rear_lift_landing_offset );
    DECLARE_PARAMETER( Vec2< double >, rear_lift_height_pitch );
    DECLARE_PARAMETER( Vec2< double >, rear_lift_switch_landing_offset );
    DECLARE_PARAMETER( Vec3< double >, rear_lift_switch_middle_pos );

    DECLARE_PARAMETER( double, ballet_step_height );
    DECLARE_PARAMETER( Vec2< double >, ballet_landing_offset );
    DECLARE_PARAMETER( double, ballet_height );
    DECLARE_PARAMETER( double, ballet_trans_step_height );

    DECLARE_PARAMETER( Vec2< double >, pitch_step_landing_offset );
    DECLARE_PARAMETER( Vec2< double >, pitch_step_pitch );
    DECLARE_PARAMETER( Vec2< double >, pitch_step_height );

    DECLARE_PARAMETER( Vec2< double >, walk_wave_landing_offset );
    DECLARE_PARAMETER( Vec2< double >, walk_wave_pitch );
    DECLARE_PARAMETER( Vec3< double >, walk_wave_height );

    DECLARE_PARAMETER( Vec3< double >, pace_stride_vel_cmd );
    DECLARE_PARAMETER( Vec4< double >, pace_stride_landing_offset_y );

    DECLARE_PARAMETER( double, special_pronk_height );
    DECLARE_PARAMETER( double, special_pronk_size );
    DECLARE_PARAMETER( Vec3< double >, special_trot_landing_offset );
    DECLARE_PARAMETER( double, upstairs_height_cmd );
    DECLARE_PARAMETER( double, downstairs_height_cmd );
    DECLARE_PARAMETER( double, rc_delta_step_height );
    DECLARE_PARAMETER( double, downstairs_depth );

    DECLARE_PARAMETER( Vec4< double >, jump_time_vector );
    DECLARE_PARAMETER( Vec4< double >, jump_front_hip );
    DECLARE_PARAMETER( Vec4< double >, jump_front_knee );
    DECLARE_PARAMETER( Vec4< double >, jump_hind_hip );
    DECLARE_PARAMETER( Vec4< double >, jump_hind_knee );

    DECLARE_PARAMETER( double, use_energy_saving_mode );
    DECLARE_PARAMETER( double, motor_sdk_position_mutation_limit );

    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_pronk_rl );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_pronk_rl );
    DECLARE_PARAMETER( Vec3< double >, acc_xy_yaw_max_pronk_rl );
    DECLARE_PARAMETER( Vec3< double >, acc_xy_yaw_min_pronk_rl );
    DECLARE_PARAMETER( double, x_offset_pronk_rl );
    DECLARE_PARAMETER( double, y_offset_pronk_rl );
    DECLARE_PARAMETER( double, yaw_offset_pronk_rl );

    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_max_rapid_rl );
    DECLARE_PARAMETER( Vec3< double >, vel_xy_yaw_min_rapid_rl );
    DECLARE_PARAMETER( Vec3< double >, acc_xy_yaw_max_rapid_rl );
    DECLARE_PARAMETER( Vec3< double >, acc_xy_yaw_min_rapid_rl );
    DECLARE_PARAMETER( double, x_offset_rapid_rl );
    DECLARE_PARAMETER( double, y_offset_rapid_rl );
    DECLARE_PARAMETER( double, yaw_offset_rapid_rl );

    DECLARE_PARAMETER( double, skateboard_height );
    DECLARE_PARAMETER( Vec2< double >, skateboard_velocity );
    DECLARE_PARAMETER( Vec3< double >, skateboard_body_pos_balance );
    DECLARE_PARAMETER( Vec3< double >, skateboard_body_pos_skate );
    DECLARE_PARAMETER( Vec3< double >, skateboard_body_pos_swing_in_skate );
    DECLARE_PARAMETER( Vec3< double >, skateboard_thrust_force );
    DECLARE_PARAMETER( Vec3< double >, skateboard_thrust_force_first );
    DECLARE_PARAMETER( Vec3< double >, skateboard_foot_swing_final_offset );
};

#endif  // USER_PARAMETERS_HPP_
