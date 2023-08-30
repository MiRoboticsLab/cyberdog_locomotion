/*=========================== Balance Stand ===========================*/
/**
 * FSM State that forces all legs to be on the ground and uses the QP
 * Balance controller for instantaneous balance control.
 */

#include "fsm_states/fsm_state_balancestand.hpp"
#include "cpp_types.hpp"
#include "utilities/control_utilities.hpp"
#include "wbc_ctrl/locomotion_ctrl/locomotion_ctrl.hpp"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param control_fsm_data holds all of the relevant control data
 */
template < typename T > FsmStateBalanceStand< T >::FsmStateBalanceStand( ControlFsmData< T >* control_fsm_data ) : FsmState< T >( control_fsm_data, FsmStateName::kQpStand, "qp_stand" ) {
    // Set the pre controls safety checks
    this->TurnOnAllSafetyChecks();
    // Turn off Foot pos command since it is set in WBC as operational task
    this->check_desired_foot_position_ = true;

    wbc_ctrl_ = new LocomotionCtrl< T >( control_fsm_data->quadruped->BuildModel() );
    wbc_data_ = new LocomotionCtrlData< T >();

    wbc_ctrl_->SetFloatingBaseWeight( 1000. );
}

template < typename T > void FsmStateBalanceStand< T >::OnEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    // Reset the transition data
    this->transition_data_.ResetTransitionDone();

    this->ready_for_switch_    = true;
    this->motion_progress_bar_ = 0;

    ini_body_pos_ = ( this->data_->state_estimator->GetResult() ).position;
#ifdef USE_ABSOLUTE_ODOM_FOR_ALL
    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        if ( ( this->data_->state_estimator->GetResult() ).height < 0.14 ) {
            ini_body_pos_[ 2 ] += 0.2 - ( this->data_->state_estimator->GetResult() ).height;
        }
    }
    else {
        if ( ( this->data_->state_estimator->GetResult() ).height < 0.2 ) {
            ini_body_pos_[ 2 ] += 0.25 - ( this->data_->state_estimator->GetResult() ).height;
        }
    }
#else
    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        if ( ini_body_pos_[ 2 ] < 0.14 ) {
            ini_body_pos_[ 2 ] = 0.20;
        }
    }
    else {
        if ( ini_body_pos_[ 2 ] < 0.2 ) {
            ini_body_pos_[ 2 ] = 0.25;
        }
    }
#endif
    //   _ini_body_pos[2]=0.26;

    is_unsafe_ = false;

    body_height_last_ = ini_body_pos_[ 2 ];

    ini_body_ori_rpy_ = ( this->data_->state_estimator->GetResult() ).rpy;
    safe_rpy_         = ini_body_ori_rpy_.template cast< double >();
    rpy_cmd_last_.setZero();
    rpy_lim_des_.setZero();
    omega_cur_.setZero();
    body_height_lim_des_ = ini_body_pos_[ 2 ];
    body_height_vel_cur_ = 0;
    body_weight_         = this->data_->quadruped->body_mass_ * 9.81;

    for ( size_t i( 0 ); i < 4; ++i ) {
        wbc_data_->reaction_force_des[ i ].setZero();
        wbc_data_->reaction_force_des[ i ][ 2 ] = body_weight_ / 4.;
    }

    no_move_cnt_ = 0;

    first_torctrlposture_ = true;
    public_iter_          = 1;  // OnEnter will occupy one cycle
    for ( int i = 0; i < 4; i++ ) {
        init_foot_pos_bodyframe_[ i ] =
            this->data_->state_estimator->GetResult().world2body_rotation_matrix.transpose() * ( this->data_->quadruped->GetHipLocation( i ) + this->data_->leg_controller->datas_[ i ].p );
    }

    if ( this->data_->command->gait_id == 1 || this->data_->command->gait_id == 3 )
        this->data_->robot_current_state->gait_id = this->data_->command->gait_id;
    else
        this->data_->robot_current_state->gait_id = 0;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template < typename T > void FsmStateBalanceStand< T >::Run() {
    this->data_->leg_controller->ZeroCommand();
    this->data_->leg_controller->legs_enabled_ = true;

    BalanceStandStep();
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template < typename T > FsmStateName FsmStateBalanceStand< T >::CheckTransition() {
    // Get the next state
    iter_++;
    auto& cmd = this->data_->command;

    // Switch FSM control mode
    switch ( cmd->mode ) {
    case MotionMode::kQpStand:
        break;

    case MotionMode::kPureDamper:
    case MotionMode::kRecoveryStand:
    case MotionMode::kRlReset:
    case MotionMode::kOff:
    case MotionMode::kLocomotion:
    case MotionMode::kPoseCtrl:
    case MotionMode::kJump3d:
        this->next_state_name_     = ( FsmStateName )( cmd->mode );
        this->transition_duration_ = 0.;
        break;

    default:
        if ( this->iter_printf_ >= this->iter_printf_reset_ ) {
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << ( int )MotionMode::kQpStand << " to " << ( int )this->data_->command->mode << std::endl;
            this->iter_printf_ = 0;
        }
        this->iter_printf_++;
    }

    // Return the next state name to the FSM
    return this->next_state_name_;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template < typename T > TransitionData< T > FsmStateBalanceStand< T >::Transition() {
    // Switch FSM control mode
    switch ( this->next_state_name_ ) {
    case FsmStateName::kLocomotion:
        BalanceStandStep();

        iter_++;
        if ( iter_ >= this->transition_duration_ * 1000 ) {
            this->transition_data_.done = true;
        }
        else {
            this->transition_data_.done = false;
        }

        break;

    case FsmStateName::kOff:
        this->TurnOffAllSafetyChecks();
        this->transition_data_.done = true;
        break;

    case FsmStateName::kPureDamper:
    case FsmStateName::kRecoveryStand:
    case FsmStateName::kRlReset:
    case FsmStateName::kPoseCtrl:
    case FsmStateName::kJump3d:
        this->transition_data_.done = true;
        break;

    default:
        std::cout << "[CONTROL FSM] Something went wrong in transition" << std::endl;
    }

    // Return the transition data to the FSM
    return this->transition_data_;
}

/**
 * Cleans up the state information on exiting the state.
 */
template < typename T > void FsmStateBalanceStand< T >::OnExit() {
    iter_ = 0;
    rpy_cmd_last_.setZero();
    rpy_lim_des_.setZero();
    omega_cur_.setZero();
    body_height_lim_des_ = 0;
    body_height_vel_cur_ = 0;
    body_height_last_    = 0;

    body_pos_cmd_pre_.setZero();
    body_rpy_cmd_pre_.setZero();
    foot_pos_cmd_pre_[ 0 ].setZero();
    foot_pos_cmd_pre_[ 1 ].setZero();
    foot_pos_cmd_pre_[ 2 ].setZero();
    foot_pos_cmd_pre_[ 3 ].setZero();
}

/**
 * Calculate the commands for the leg controllers for each of the feet.
 */
template < typename T > void FsmStateBalanceStand< T >::BalanceStandStep() {
    Vec4< T > contact_state_vec;
    contact_state_vec << 0.5, 0.5, 0.5, 0.5;
    auto cmd  = this->data_->command;
    duration_ = cmd->duration;

    // Absolute force control attitude
    if ( this->data_->command->gait_id != 1 && this->data_->command->gait_id != 3 ) {
        this->data_->robot_current_state->gait_id = 0;

        wbc_data_->body_pos_des = ini_body_pos_;
        wbc_data_->body_vel_des.setZero();
        wbc_data_->body_acc_des.setZero();

        wbc_data_->body_rpy_des = ini_body_ori_rpy_;

        Vec3< T > rpy_des_cmd_scaled( cmd->rpy_des[ 0 ], cmd->rpy_des[ 1 ], cmd->rpy_des[ 2 ] );
        RpyCmdRescale( ( T )cmd->pos_des[ 2 ], rpy_des_cmd_scaled, this->data_->quadruped->robot_type_ );
        Vec3< double > rpy_des_cmd( rpy_des_cmd_scaled[ 0 ], rpy_des_cmd_scaled[ 1 ], rpy_des_cmd_scaled[ 2 ] );

        // Apply Orientation limit
        auto   user_param = this->data_->user_parameters;
        double dt         = this->data_->control_parameters->controller_dt;
        for ( int i = 0; i < 3; i++ ) {
            rpy_lim_des_[ i ] = ApplyPoseMeetVelocityAccelationLimits( ( double )rpy_cmd_last_[ i ], ( double )omega_cur_[ i ], rpy_des_cmd[ i ], user_param->rpy_min[ i ], user_param->rpy_max[ i ],
                                                                       -user_param->rpy_w_max[ i ], user_param->rpy_w_max[ i ], -user_param->rpy_acc_max[ i ], user_param->rpy_acc_max[ i ], dt );
            omega_cur_[ i ]   = ( rpy_lim_des_[ i ] - rpy_cmd_last_[ i ] ) / dt;
        }

        // Apply Orientation limit
        double body_height_cmd = cmd->pos_des[ 2 ];
#ifdef USE_ABSOLUTE_ODOM_FOR_ALL
        auto& se_result      = this->data_->state_estimator->GetResult();
        auto& quadruped      = *this->data_->quadruped;
        auto& leg_controller = *this->data_->leg_controller;
        float ground_height  = 0;

        Vec3< T > pos_foot[ 4 ];
        for ( int i = 0; i < 4; ++i ) {
            pos_foot[ i ] = se_result.position + se_result.world2body_rotation_matrix.transpose() * ( quadruped.GetHipLocation( i ) + leg_controller.datas_[ i ].p );
            ground_height += pos_foot[ i ][ 2 ];
        }

        body_height_cmd += ground_height / 4.0;
        body_height_lim_des_ = ApplyPoseMeetVelocityAccelationLimits( ( double )body_height_last_, ( double )body_height_vel_cur_, body_height_cmd, -std::numeric_limits< double >::infinity(),
                                                                      std::numeric_limits< double >::infinity(), -0.5, 0.5, -3., 3., dt );
#else
        body_height_lim_des_ = ApplyPoseMeetVelocityAccelationLimits( ( double )body_height_last_, ( double )body_height_vel_cur_, body_height_cmd, 0.15, 0.35, -0.5, 0.5, -3., 3., dt );
#endif
        body_height_vel_cur_ = ( body_height_lim_des_ - body_height_last_ ) / dt;

        // rpyCmdRescale( _body_height_lim_des, _rpy_lim_des, this->data_->quadruped->robot_type_ );
        // _is_unsafe : first-time action in if( !leg_safe ){}else{}
        bool leg_safe = LegSafetyChecker();

        if ( !leg_safe ) {
            if ( !is_unsafe_ ) {
                safe_height_ = body_height_last_;
                safe_rpy_    = rpy_cmd_last_.cast< double >();
            }
            rpy_cmd_last_[ 0 ] = rpy_lim_des_[ 0 ];
            rpy_cmd_last_[ 1 ] = rpy_lim_des_[ 1 ];
            rpy_cmd_last_[ 2 ] = rpy_lim_des_[ 2 ];
            body_height_last_  = body_height_lim_des_;

            rpy_lim_des_[ 0 ]    = safe_rpy_[ 0 ];
            rpy_lim_des_[ 1 ]    = safe_rpy_[ 1 ];
            rpy_lim_des_[ 2 ]    = safe_rpy_[ 2 ];
            body_height_lim_des_ = safe_height_;

            is_unsafe_ = true;
        }
        else {
            if ( is_unsafe_ ) {
                rpy_lim_des_[ 0 ]    = 0.1 * rpy_lim_des_[ 0 ] + 0.9 * safe_rpy_[ 0 ];
                rpy_lim_des_[ 1 ]    = 0.1 * rpy_lim_des_[ 1 ] + 0.9 * safe_rpy_[ 1 ];
                rpy_lim_des_[ 2 ]    = 0.1 * rpy_lim_des_[ 2 ] + 0.9 * safe_rpy_[ 2 ];
                body_height_lim_des_ = 0.1 * body_height_lim_des_ + 0.9 * safe_height_;
            }
            is_unsafe_ = false;
        }

        wbc_data_->body_pos_des[ 2 ] = body_height_lim_des_;
        wbc_data_->body_rpy_des[ 0 ] = rpy_lim_des_[ 0 ];
        wbc_data_->body_rpy_des[ 1 ] = rpy_lim_des_[ 1 ];
        if ( fabs( rpy_lim_des_[ 2 ] ) < 0.01 ) {
            if ( no_move_cnt_ > 500 ) {
                wbc_data_->body_rpy_des[ 2 ] = ( this->data_->state_estimator->GetResult() ).rpy[ 2 ];
                ini_body_ori_rpy_[ 2 ]       = wbc_data_->body_rpy_des[ 2 ];
            }
            no_move_cnt_++;
        }
        else {
            no_move_cnt_ = 0;
        }

        wbc_data_->body_rpy_des[ 2 ] += rpy_lim_des_[ 2 ];

        wbc_data_->body_omg_des.setZero();

        Vec3< T > foot_pos_bodyframe[ 4 ];
        for ( size_t i( 0 ); i < 4; ++i ) {
            foot_pos_bodyframe[ i ] =
                this->data_->state_estimator->GetResult().world2body_rotation_matrix.transpose() * ( this->data_->quadruped->GetHipLocation( i ) + this->data_->leg_controller->datas_[ i ].p );
        }
        for ( size_t i( 0 ); i < 4; ++i ) {
            wbc_data_->foot_pos_des[ i ].setZero();
            wbc_data_->foot_vel_des[ i ].setZero();
            wbc_data_->foot_acc_des[ i ].setZero();
            wbc_data_->reaction_force_des[ i ][ 0 ] = -100 * ( init_foot_pos_bodyframe_[ i ][ 0 ] - foot_pos_bodyframe[ i ][ 0 ] );
            wbc_data_->reaction_force_des[ i ][ 1 ] = -100 * ( init_foot_pos_bodyframe_[ i ][ 1 ] - foot_pos_bodyframe[ i ][ 1 ] );
            wbc_data_->reaction_force_des[ i ][ 2 ] = body_weight_ / 4.;
            wbc_data_->contact_state[ i ]           = true;
        }

        if ( leg_safe ) {
            rpy_cmd_last_     = rpy_lim_des_.template cast< double >();
            body_height_last_ = body_height_lim_des_;
        }
        first_torctrlposture_      = true;
        this->motion_progress_bar_ = 100;
        wbc_ctrl_->Run( wbc_data_, *this->data_ );
    }
    // Incremental force control attitude
    else if ( cmd->gait_id == 3 || cmd->gait_id == 1 ) {
        this->data_->robot_current_state->gait_id = this->data_->command->gait_id;
        if ( duration_ <= 0 )
            duration_ = 1000;
        if ( first_torctrlposture_ == true ) {  // torctrlposture
            body_pos_cmd_pre_.setZero();
            body_rpy_cmd_pre_.setZero();
            foot_pos_cmd_pre_[ 0 ].setZero();
            foot_pos_cmd_pre_[ 1 ].setZero();
            foot_pos_cmd_pre_[ 2 ].setZero();
            foot_pos_cmd_pre_[ 3 ].setZero();
            first_torctrlposture_ = false;
            init_body_pos_        = this->data_->state_estimator->GetResult().position;
            init_body_rpy_        = this->data_->state_estimator->GetResult().rpy;
            wbc_ctrl_->SetFloatingBaseWeight( 1000. );
            for ( int i = 0; i < 4; i++ ) {
                init_foot_pos_[ i ] =
                    init_body_pos_
                    + this->data_->state_estimator->GetResult().world2body_rotation_matrix.transpose() * ( this->data_->quadruped->GetHipLocation( i ) + this->data_->leg_controller->datas_[ i ].p );
            }
        }

        float         offset_yaw = this->data_->state_estimator->GetResult().rpy[ 2 ];
        Mat3< float > R_yaw;
        R_yaw.setZero();
        R_yaw( 0, 0 ) = cos( offset_yaw );
        R_yaw( 0, 1 ) = -sin( offset_yaw );
        R_yaw( 1, 0 ) = sin( offset_yaw );
        R_yaw( 1, 1 ) = cos( offset_yaw );
        R_yaw( 2, 2 ) = 1;
        for ( int k = 0; k < 4; k++ ) {
            pose_foot_support_( k ) = ( cmd->contact >> k ) & 0x01;
        }
        this->motion_progress_bar_ = ( int32_t )( public_iter_ < duration_ ? public_iter_ / ( float )duration_ * 100 : 100 );

        public_iter_++;
        if ( public_iter_ < duration_ ) {
            body_pos_des_ = QuinticPolySpline( body_pos_cmd_pre_, body_pos_cmd_pre_ + R_yaw * Vec3< T >( cmd->pos_des[ 0 ], cmd->pos_des[ 1 ], cmd->pos_des[ 2 ] ), duration_, public_iter_ );
            body_rpy_des_ = QuinticPolySpline( body_rpy_cmd_pre_, body_rpy_cmd_pre_ + Vec3< T >( cmd->rpy_des[ 0 ], cmd->rpy_des[ 1 ], cmd->rpy_des[ 2 ] ), duration_, public_iter_ );
            //  one legs swings
            for ( int leg( 0 ); leg < 4; leg++ ) {
                if ( pose_foot_support_[ leg ] < 0.01 )
                    foot_pos_des_[ leg ] = QuinticPolySpline( foot_pos_cmd_pre_[ leg ], foot_pos_cmd_pre_[ leg ] + R_yaw * Vec3< T >( cmd->foot_pose[ 0 ], cmd->foot_pose[ 1 ], cmd->foot_pose[ 2 ] ),
                                                              duration_, public_iter_ );
                else
                    foot_pos_des_[ leg ].segment( 6, 3 ) = foot_pos_cmd_pre_[ leg ];
            }
        }
        if ( public_iter_ >= duration_ ) {
            // transfer to world frame
            body_pos_cmd_pre_ += R_yaw * Vec3< T >( cmd->pos_des[ 0 ], cmd->pos_des[ 1 ], cmd->pos_des[ 2 ] );
            body_rpy_cmd_pre_ += Vec3< T >( cmd->rpy_des[ 0 ], cmd->rpy_des[ 1 ], cmd->rpy_des[ 2 ] );
            for ( int leg( 0 ); leg < 4; leg++ ) {
                if ( pose_foot_support_[ leg ] < 0.01 )
                    foot_pos_cmd_pre_[ leg ] += R_yaw * Vec3< T >( cmd->foot_pose[ 0 ], cmd->foot_pose[ 1 ], cmd->foot_pose[ 2 ] );
            }
            // First_Torctrlposture = true; //Shielding to support continuous execution
            public_iter_ = 0;
        }
        wbc_data_->body_acc_des = body_pos_des_.segment( 0, 3 );
        wbc_data_->body_vel_des = body_pos_des_.segment( 3, 3 );
        wbc_data_->body_pos_des = init_body_pos_ + body_pos_des_.segment( 6, 3 );

        wbc_data_->body_omg_des = body_rpy_des_.segment( 3, 3 );
        wbc_data_->body_rpy_des = init_body_rpy_ + body_rpy_des_.segment( 6, 3 );

        int  contact_num = 0;
        bool contact_state[ 4 ];
        for ( int leg( 0 ); leg < 4; leg++ ) {
            if ( pose_foot_support_( leg ) > 0 ) {
                contact_num++;
                contact_state[ leg ] = true;
                // set contact state for  stateEstimator
                contact_state_vec[ leg ] = 0.5;
            }
            else {
                contact_state[ leg ]     = false;
                contact_state_vec[ leg ] = 0;
            }
        }
        // check transtion
        int leg = 0;
        while ( !contact_transition_ && leg < 4 ) {
            if ( contact_state[ leg ] != contact_state_old_[ leg ] )
                contact_transition_ = true;
            leg++;
        }
        Mat3< T > kp_backup, kd_backup;
        kp_backup.setZero();
        kd_backup.setZero();
        kp_backup.diagonal() << this->data_->user_parameters->mpc_wbc_stance_cartesian_kp[ 0 ], this->data_->user_parameters->mpc_wbc_stance_cartesian_kp[ 1 ],
            this->data_->user_parameters->mpc_wbc_stance_cartesian_kp[ 2 ];
        kd_backup.diagonal() << this->data_->user_parameters->mpc_wbc_stance_cartesian_kd[ 0 ], this->data_->user_parameters->mpc_wbc_stance_cartesian_kd[ 1 ],
            this->data_->user_parameters->mpc_wbc_stance_cartesian_kd[ 2 ];
        // Kp_backup.diagonal() << this->data_->user_parameters->mpc_wbc_stance_cartesian_kp.cast<T>();
        // Kd_backup.diagonal() << this->data_->user_parameters->mpc_wbc_stance_cartesian_kd.cast<T>();
        // add  transition of reaction force
        Vec3< T > fr_des_cmd[ 4 ];
        double    filter = 0.1;
        for ( size_t i( 0 ); i < 4; ++i ) {
            wbc_data_->contact_state[ i ] = contact_state_vec[ i ];
            if ( contact_state_vec[ i ] > 0 ) {
                wbc_data_->foot_pos_des[ i ] = init_foot_pos_[ i ] + foot_pos_des_[ i ].segment( 6, 3 );
                wbc_data_->foot_vel_des[ i ].setZero();
                wbc_data_->foot_acc_des[ i ].setZero();
                fr_des_cmd[ i ] = this->data_->quadruped->body_mass_ * body_pos_des_.segment( 0, 3 ) / contact_num;
                fr_des_cmd[ i ][ 2 ] += body_weight_ / contact_num;
            }
            else {
                wbc_data_->foot_pos_des[ i ] = init_foot_pos_[ i ] + foot_pos_des_[ i ].segment( 6, 3 );
                wbc_data_->foot_vel_des[ i ] = foot_pos_des_[ i ].segment( 3, 3 );
                wbc_data_->foot_acc_des[ i ] = foot_pos_des_[ i ].segment( 0, 3 );
                fr_des_cmd[ i ] << 0.0, 0.0, 0.0;
            }
            wbc_data_->reaction_force_des[ i ] = wbc_data_->reaction_force_des[ i ] * ( 1 - filter ) + fr_des_cmd[ i ] * filter;
            // std::cout << "initial foot pos  is :  " << i << std::endl;
        }
        wbc_ctrl_->Run( wbc_data_, *this->data_ );
        // record joint pos for pos contrller, after  wbc->run!!!
        for ( int i( 0 ); i < 4; i++ ) {
            public_q_cmd_[ i ] = this->data_->leg_controller->datas_[ i ].q;
            // Cartesian pd  feedforward compensation
            // if(contact_state[i] <= 0)
            // {
            //     Vec3<float> pDesLeg = this->data_->state_estimator->GetResult().world2body_rotation_matrix * (wbc_data_->foot_pos_des[i] - this->data_->state_estimator->GetResult().position) -
            //     this->data_->quadruped->GetHipLocation(i); Vec3<float> vDesLeg = this->data_->state_estimator->GetResult().world2body_rotation_matrix * (wbc_data_->foot_vel_des[i] -
            //     this->data_->state_estimator->GetResult().velocity_in_world_frame); this->data_->leg_controller->commands_[i].p_des = pDesLeg; this->data_->leg_controller->commands_[i].v_des =
            //     vDesLeg; this->data_->leg_controller->commands_[i].kp_cartesian = Kp_backup; this->data_->leg_controller->commands_[i].kd_cartesian = Kd_backup;
            // }
        }
    }
    this->data_->state_estimator->SetContactPhase( contact_state_vec );
}

template < typename T > bool FsmStateBalanceStand< T >::LegSafetyChecker() {
    bool       is_safe        = true;
    static int iter           = 0;
    auto&      se_result      = this->data_->state_estimator->GetResult();
    auto&      quadruped      = *this->data_->quadruped;
    auto&      leg_controller = *this->data_->leg_controller;

    Vec3< T > pos_foot[ 4 ];
    for ( int i = 0; i < 4; ++i ) {
        pos_foot[ i ] = se_result.position + se_result.world2body_rotation_matrix.transpose() * ( quadruped.GetHipLocation( i ) + leg_controller.datas_[ i ].p );
    }

    Vec3< T >                rpy_des( rpy_lim_des_[ 0 ], rpy_lim_des_[ 1 ], ini_body_ori_rpy_[ 2 ] - rpy_lim_des_[ 2 ] );
    Eigen::Matrix< T, 3, 3 > rpydes = RpyToRotMat( rpy_des );
    Vec3< T >                posdes = Vec3< T >( se_result.position[ 0 ], se_result.position[ 1 ], body_height_lim_des_ );
    Vec3< T >                phipdes[ 4 ];
    for ( int i = 0; i < 4; ++i ) {
        phipdes[ i ] = posdes + rpydes.transpose() * quadruped.GetHipLocation( i );
    }

    Vec3< T > plegdes[ 4 ];
    for ( int i = 0; i < 4; ++i ) {
        plegdes[ i ] = rpydes * ( phipdes[ i ] - pos_foot[ i ] );

        if ( plegdes[ i ].norm() > 1.0 * quadruped.max_leg_length_ || plegdes[ i ].norm() < 0.35 * quadruped.max_leg_length_ || fabs( plegdes[ i ][ 2 ] ) < 0.11 ) {
            is_safe = false;
        }
    }

    iter++;

    return is_safe;
}
template < typename T > Vec9< T > FsmStateBalanceStand< T >::QuinticPolySpline( const Vec3< T > trajec_cmd_pre, const Vec3< T > trajec_cmd, const double iter_T, double iter_t ) {
    // quintic polynomial
    Vec9< T > trajec_out;  // acc-vel-pos
    Vec3< T > pos_ini;
    pos_ini << trajec_cmd_pre[ 0 ], trajec_cmd_pre[ 1 ], trajec_cmd_pre[ 2 ];
    Vec3< T > pos_fin;
    pos_fin << trajec_cmd[ 0 ], trajec_cmd[ 1 ], trajec_cmd[ 2 ];
    Vec3< T > vel_ini;
    vel_ini.setZero();
    Vec3< T > vel_fin;
    vel_fin.setZero();
    Vec3< T > acc_ini;
    acc_ini.setZero();
    Vec3< T > acc_fin;
    acc_fin.setZero();
    // define params
    Vec3< T > a1;
    Vec3< T > a2;
    Vec3< T > a3;
    Vec3< T > a4;
    Vec3< T > a5;
    Vec3< T > a6;
    a1 = acc_fin / ( 2 * pow( iter_T, 3 ) ) - acc_ini / ( 2 * pow( iter_T, 3 ) ) - ( 6 * pos_ini ) / pow( iter_T, 5 ) + ( 6 * pos_fin ) / pow( iter_T, 5 ) - ( 3 * vel_ini ) / pow( iter_T, 4 )
         - ( 3 * vel_fin ) / pow( iter_T, 4 );
    a2 = ( 3 * acc_ini ) / ( 2 * pow( iter_T, 2 ) ) - acc_fin / pow( iter_T, 2 ) + ( 15 * pos_ini ) / pow( iter_T, 4 ) - ( 15 * pos_fin ) / pow( iter_T, 4 ) + ( 8 * vel_ini ) / pow( iter_T, 3 )
         + ( 7 * vel_fin ) / pow( iter_T, 3 );
    a3 = acc_fin / ( 2 * iter_T ) - ( 3 * acc_ini ) / ( 2 * iter_T ) - ( 10 * pos_ini ) / pow( iter_T, 3 ) + ( 10 * pos_fin ) / pow( iter_T, 3 ) - ( 6 * vel_ini ) / pow( iter_T, 2 )
         - ( 4 * vel_fin ) / pow( iter_T, 2 );
    a4 = acc_ini / 2;
    a5 = vel_ini;
    a6 = pos_ini;
    // cal
    if ( iter_t <= iter_T ) {
        trajec_out.segment( 6, 3 ) = a1 * pow( iter_t, 5 ) + a2 * pow( iter_t, 4 ) + a3 * pow( iter_t, 3 ) + a4 * pow( iter_t, 2 ) + a5 * pow( iter_t, 1 ) + a6;
        trajec_out.segment( 3, 3 ) = 5 * a1 * pow( iter_t, 4 ) + 4 * a2 * pow( iter_t, 3 ) + 3 * a3 * pow( iter_t, 2 ) + 2 * a4 * pow( iter_t, 1 ) + a5;
        trajec_out.segment( 0, 3 ) = 20 * a1 * pow( iter_t, 3 ) + 12 * a2 * pow( iter_t, 2 ) + 6 * a3 * pow( iter_t, 1 ) + 2 * a4;
    }
    else {
        trajec_out.segment( 6, 3 ) = pos_fin;
        trajec_out.segment( 3, 3 ) = vel_fin;
        trajec_out.segment( 0, 3 ) = acc_fin;
    }
    return trajec_out;
}

template < typename T > void FsmStateBalanceStand< T >::RpyCmdRescale( const T& body_height_lim_des, Vec3< T >& rpy_lim_des, const RobotType& robot_type ) {
    // _body_height_lim_des
    // _rpy_lim_des
    if ( robot_type != RobotType::CYBERDOG2 )
        return;
    float rpy_lim_des_damp = 1.0;
    if ( body_height_lim_des < 0.26 && body_height_lim_des > 0.15 ) {
        if ( body_height_lim_des > 0.235 && body_height_lim_des < 0.26 )
            rpy_lim_des_damp = ( 0.26 - body_height_lim_des ) / 0.025;
        else if ( body_height_lim_des > 0.15 && body_height_lim_des < 0.18 )
            rpy_lim_des_damp = ( body_height_lim_des - 0.15 ) / 0.03;
        else
            rpy_lim_des_damp = 1.0;
    }
    else {
        rpy_lim_des_damp = 0.0;
    }
    rpy_lim_des *= rpy_lim_des_damp;

    auto  user_param      = this->data_->user_parameters;
    float rp_lim_des_damp = 1.0;
    if ( rpy_lim_des[ 2 ] > user_param->rpy_max[ 2 ] / 3 && rpy_lim_des[ 2 ] <= user_param->rpy_max[ 2 ] )
        rp_lim_des_damp = 1 - ( rpy_lim_des[ 2 ] - user_param->rpy_max[ 2 ] / 3 ) / ( user_param->rpy_max[ 2 ] * 2 / 3 );
    else if ( rpy_lim_des[ 2 ] < user_param->rpy_min[ 2 ] / 3 && rpy_lim_des[ 2 ] >= user_param->rpy_min[ 2 ] )
        rp_lim_des_damp = 1 - ( rpy_lim_des[ 2 ] - user_param->rpy_min[ 2 ] / 3 ) / ( user_param->rpy_min[ 2 ] * 2 / 3 );
    else if ( rpy_lim_des[ 2 ] > user_param->rpy_max[ 2 ] || rpy_lim_des[ 2 ] < user_param->rpy_min[ 2 ] )
        rp_lim_des_damp = 0;
    rpy_lim_des.head( 2 ) *= rp_lim_des_damp;

    float r_lim_des_damp = 1.0;
    if ( rpy_lim_des[ 1 ] > user_param->rpy_max[ 1 ] / 3 && rpy_lim_des[ 1 ] <= user_param->rpy_max[ 1 ] )
        r_lim_des_damp = 1 - ( rpy_lim_des[ 1 ] - user_param->rpy_max[ 1 ] / 3 ) / ( user_param->rpy_max[ 1 ] * 2 / 3 );
    else if ( rpy_lim_des[ 1 ] < user_param->rpy_min[ 1 ] / 3 && rpy_lim_des[ 1 ] >= user_param->rpy_min[ 1 ] )
        r_lim_des_damp = 1 - ( rpy_lim_des[ 1 ] - user_param->rpy_min[ 1 ] / 3 ) / ( user_param->rpy_min[ 1 ] * 2 / 3 );
    else if ( rpy_lim_des[ 1 ] > user_param->rpy_max[ 1 ] || rpy_lim_des[ 1 ] < user_param->rpy_min[ 1 ] )
        r_lim_des_damp = 0;

    float p_lim_des_damp = 1.0;
    if ( rpy_lim_des[ 0 ] > user_param->rpy_max[ 0 ] / 3 && rpy_lim_des[ 0 ] <= user_param->rpy_max[ 0 ] )
        p_lim_des_damp = 1 - ( rpy_lim_des[ 0 ] - user_param->rpy_max[ 0 ] / 3 ) / ( user_param->rpy_max[ 0 ] * 2 / 3 );
    else if ( rpy_lim_des[ 0 ] < user_param->rpy_min[ 0 ] / 3 && rpy_lim_des[ 1 ] >= user_param->rpy_min[ 0 ] )
        p_lim_des_damp = 1 - ( rpy_lim_des[ 0 ] - user_param->rpy_min[ 0 ] / 3 ) / ( user_param->rpy_min[ 0 ] * 2 / 3 );
    else if ( rpy_lim_des[ 0 ] > user_param->rpy_max[ 0 ] || rpy_lim_des[ 0 ] < user_param->rpy_min[ 0 ] )
        p_lim_des_damp = 0;

    rpy_lim_des[ 1 ] *= p_lim_des_damp;
    rpy_lim_des[ 0 ] *= r_lim_des_damp;
}

// template class FSM_State_BalanceStand<double>;
template class FsmStateBalanceStand< float >;
