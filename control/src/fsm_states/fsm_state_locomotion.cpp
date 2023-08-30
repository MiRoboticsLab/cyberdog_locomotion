#include "fsm_states/fsm_state_locomotion.hpp"
#include "utilities/timer.hpp"
#include "wbc_ctrl/locomotion_ctrl/locomotion_ctrl.hpp"
#include <fstream>
#include <unistd.h>

/**
 * @brief Construct a new Fsm State Locomotion<  T >:: Fsm State Locomotion object.
 *
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 *
 * @param control_fsm_data holds all of the relevant control data
 */
template < typename T > FsmStateLocomotion< T >::FsmStateLocomotion( ControlFsmData< T >* control_fsm_data ) : FsmState< T >( control_fsm_data, FsmStateName::kLocomotion, "locomotion" ) {
    if ( control_fsm_data->quadruped->robot_type_ == RobotType::CYBERDOG || control_fsm_data->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        convex_mpc_loco_gaits_   = new ConvexMpcLocoGaits( control_fsm_data->control_parameters->controller_dt, 30 / ( 1000. * control_fsm_data->control_parameters->controller_dt ) );
        convex_mpc_motion_gaits_ = new ConvexMpcMotionGaits( control_fsm_data->control_parameters->controller_dt, 30 / ( 1000. * control_fsm_data->control_parameters->controller_dt ) );
    }
    else {
        assert( false );
    }

    //  this->TurnOnAllSafetyChecks();
    this->TurnOffAllSafetyChecks();  ////////////////////////////////////////////////////////////////////////////
    // Turn off Foot pos command since it is set in WBC as operational task
    this->check_desired_foot_position_ = true;
    this->check_safe_orientation_      = true;
    this->check_robot_lifted_          = true;
    wbc_ctrl_                          = new LocomotionCtrl< T >( control_fsm_data->quadruped->BuildModel() );
    wbc_data_                          = new LocomotionCtrlData< T >();
#ifdef SEPERATE_MPC_ANOTHER_THREAD
    // creat new thread to solve mpc
    sem_init( &solve_mpc_flag_, 0, 0 );
    solve_mpc_thread_ = new std::thread( &FsmStateLocomotion::RunSolveMpcAnotherThread, this );

    static int  policy = SCHED_RR;
    sched_param sch_params;
    sch_params.sched_priority = sched_get_priority_max( policy );
    pthread_setschedparam( solve_mpc_thread_->native_handle(), policy, &sch_params );
    int rc = pthread_getschedparam( solve_mpc_thread_->native_handle(), &policy, &sch_params );
    ( void )rc;
    std::cout << "[FSM LOCOMOTION] solve-mpc  " << policy << "   " << sch_params.sched_priority << std::endl;

    // record time cost
    // mpc_thread_log_f_.open("/tmp/log/time_log_mpc.txt");
    // if (!mpc_thread_log_f_.is_open()) {
    // std::cout << "Failed to open period log file time_log.txt, exit"
    // << std::endl;
    // exit(-1);
    // }
    convex_mpc_loco_gaits_->solve_mpc_by_single_thread_   = true;
    convex_mpc_motion_gaits_->solve_mpc_by_single_thread_ = true;
#else
    convex_mpc_loco_gaits_->solve_mpc_by_single_thread_   = false;
    convex_mpc_motion_gaits_->solve_mpc_by_single_thread_ = false;
#endif
}

/**
 * @brief Behavior to be carried out when entering a state.
 *
 */
template < typename T > void FsmStateLocomotion< T >::OnEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    first_locomotion_run_ = true;
    public_iter_          = 0;
    duration_             = 0;
    iter_                 = 0;

    this->data_->robot_current_state->gait_id       = this->data_->command->gait_id;
    this->data_->robot_current_state->gait_cmd_used = this->data_->command->gait_id;

    convex_mpc_loco_gaits_->transform_last_steps_ = false;
    convex_mpc_loco_gaits_->transform_first_run_  = true;
    // Reset the transition data
    this->transition_data_.ResetTransitionDone();
    this->ready_for_switch_    = true;
    this->motion_progress_bar_ = 0;
    convex_mpc_loco_gaits_->Initialize( *this->data_ );
    convex_mpc_loco_gaits_->duration_mode_         = false;
    convex_mpc_loco_gaits_->gait_check_transition_ = true;
    convex_mpc_motion_gaits_->Initialize( *this->data_ );
    convex_mpc_motion_gaits_->duration_mode_         = false;
    convex_mpc_motion_gaits_->gait_check_transition_ = true;
    printf( "[FSM LOCOMOTION] On Enter\n" );
}

/**
 * @brief Calls the functions to be executed on each control loop iteration.
 *
 */
template < typename T > void FsmStateLocomotion< T >::Run() {
    // Call the locomotion control logic for this iteration
    this->data_->leg_controller->ZeroCommand();
    this->data_->leg_controller->legs_enabled_ = true;
    LocomotionControlStep();
}

/**
 * @brief  Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template < typename T > FsmStateName FsmStateLocomotion< T >::CheckTransition() {
    // Get the next state
    auto& cmd = this->data_->command;

    // Switch FSM control mode
    if ( LocomotionSafe() ) {
        switch ( cmd->mode ) {
        case MotionMode::kLocomotion:
            break;

        case MotionMode::kRecoveryStand:
        case MotionMode::kOff:
        case MotionMode::kQpStand:
        case MotionMode::kPureDamper:
        case MotionMode::kPoseCtrl:
        case MotionMode::kJump3d:
        case MotionMode::kRlReset:
        case MotionMode::kRlRapid:
            this->next_state_name_     = ( FsmStateName )cmd->mode;
            this->transition_duration_ = 0.;
            break;

        default:
            if ( this->iter_printf_ >= this->iter_printf_reset_ ) {
                std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << ( int )MotionMode::kLocomotion << " to " << ( int )this->data_->command->mode << std::endl;
                this->iter_printf_ = 0;
            }
            this->iter_printf_++;
        }
    }
    else {
        this->next_state_name_     = FsmStateName::kRecoveryStand;
        this->transition_duration_ = 0.;

        printf( "[FSM LOCOMOTION] locomotion force to kRecoveryStand\n" );
        return FsmStateName::kRecoveryStand;
    }

    // Return the next state name to the FSM
    return this->next_state_name_;
}

/**
 * @brief Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template < typename T > TransitionData< T > FsmStateLocomotion< T >::Transition() {
    if ( ( convex_mpc_loco_gaits_->gait_check_transition_ && !convex_mpc_loco_gaits_->ban_exit_loco_ ) || convex_mpc_motion_gaits_->gait_check_transition_ ) {
        trans_flag_                                   = true;
        convex_mpc_loco_gaits_->transform_last_steps_ = true;
    }
    else {
        trans_flag_ = false;
    }

    // Switch FSM control mode
    switch ( this->next_state_name_ ) {
    case FsmStateName::kQpStand:
    case FsmStateName::kPoseCtrl:
    case FsmStateName::kRecoveryStand:
    case FsmStateName::kPureDamper:
    case FsmStateName::kRlReset:
    case FsmStateName::kRlRapid:
        static const int kIterMpc   = 30 / ( 1000. * this->data_->control_parameters->controller_dt );
        static int       max_period = 10;
        if ( convex_switch_index_ != 1 )
            max_period = 0;
        else
            max_period = ( int )convex_mpc_loco_gaits_->gait_period_;
        LocomotionControlStep();
        if ( convex_mpc_loco_gaits_->transform_last_steps_ ) {
            iter_++;
            if ( iter_ % 60 == 0 )
                printf( "[FSM LOCOMOTION] current transitionDuration iter: %d\n", iter_ );
        }
        if ( trans_flag_ && iter_ > max_period * kIterMpc ) {
            this->transition_data_.done = true;
            printf( "[FSM LOCOMOTION] transitionDuration iter: %d\t%d\n", iter_, int( this->transition_duration_ * 1000 ) );
        }
        else {
            this->transition_data_.done = false;
        }
        break;

    case FsmStateName::kJump3d:
        LocomotionControlStep();
        if ( trans_flag_ ) {
            this->transition_data_.done = true;
            printf( "[FSM LOCOMOTION] transition_data_.done = true \n" );
        }
        break;

    case FsmStateName::kOff:
        this->TurnOffAllSafetyChecks();
        this->transition_data_.done = true;
        break;

    default:
        std::cout << "[CONTROL FSM] Something went wrong in transition" << std::endl;
    }

    // Return the transition data to the FSM
    return this->transition_data_;
}

/**
 * @brief Judges whether the robot is in good condition.
 *
 * If this function returns false, robot goes to recoverystand.
 *
 * @return true when robot is in good condition
 * @return false when robot has some errors
 */
template < typename T > bool FsmStateLocomotion< T >::LocomotionSafe() {
    auto& seResult = this->data_->state_estimator->GetResult();

    static const T kMaxRoll  = 80;  // 40;
    static const T kMaxPitch = 80;  // 40;

    if ( std::fabs( seResult.rpy[ 0 ] ) > ori::Deg2Rad( kMaxRoll ) ) {
        printf( "[FSM LOCOMOTION] Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::Rad2Deg( seResult.rpy[ 0 ] ), kMaxRoll );
        return false;
    }

    if ( std::fabs( seResult.rpy[ 1 ] ) > ori::Deg2Rad( kMaxPitch ) ) {
        printf( "[FSM LOCOMOTION] Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::Rad2Deg( seResult.rpy[ 1 ] ), kMaxPitch );
        return false;
    }

    for ( int leg = 0; leg < 4; leg++ ) {
        auto p_leg = this->data_->leg_controller->datas_[ leg ].p;
        if ( p_leg[ 2 ] > 0 ) {
            printf( "[FSM LOCOMOTION] Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[ 2 ] );
            return false;
        }

        if ( std::fabs( p_leg[ 1 ] ) > 0.28 ) {  // 0.18))
            printf( "[FSM LOCOMOTION] Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[ 1 ] );
            return false;
        }

        auto v_leg = this->data_->leg_controller->datas_[ leg ].v.norm();
        if ( std::fabs( v_leg ) > 19. ) {
            printf( "[FSM LOCOMOTION] Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg );
            return false;
        }
    }

    return true;
}

/**
 * @brief Behavior to be carried out when exiting a state.
 *
 * Cleans up the state information on exiting the state.
 */
template < typename T > void FsmStateLocomotion< T >::OnExit() {
    // Nothing to clean up when exiting
    iter_ = 0;
}

/**
 * @brief Initialize user gaits.
 *
 */
template < typename T > void FsmStateLocomotion< T >::InitUserGaits() {
    convex_mpc_motion_gaits_->InitUserGaits( false );
}

/**
 * @brief Parses contact specific controls to the leg controller.
 *
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 *
 * Select MPC based on gaits ID and get all gait information
 *
 */
template < typename T > void FsmStateLocomotion< T >::LocomotionControlStep() {
    auto             cmd                     = this->data_->command;
    static int       gait_id_cmd_last        = 0;
    static Vec3< T > leg_pos_des_backup[ 4 ] = { Vec3< T >::Zero() };
    static Vec3< T > leg_vel_des_backup[ 4 ] = { Vec3< T >::Zero() };
    static Mat3< T > leg_kp_des_backup[ 4 ]  = { Mat3< T >::Zero() };
    static Mat3< T > leg_kd_des_backup[ 4 ]  = { Mat3< T >::Zero() };

    duration_ = ( float )( cmd->duration );
    // set motionProgressBar
    if ( cmd->duration == 0 ) {
        duration_                  = 1800000;
        this->motion_progress_bar_ = 100;
    }
    else if ( cmd->gait_id == GaitId::kUserGait && cmd->cmd_source == kCyberdog2LcmCmd ) {
        // Set ProgressBar by the value of user gait section's count
        if ( cmd->motion_process_bar > 0 )
            this->motion_progress_bar_ = cmd->motion_process_bar;
        else
            this->motion_progress_bar_ = 100;
    }
    else {
        this->motion_progress_bar_ = public_iter_ < cmd->duration ? ( int )( public_iter_ / duration_ * 100 ) : 100;
    }

    if ( first_locomotion_run_ ) {
        if ( cmd->duration != 0 ) {
            convex_mpc_loco_gaits_->duration_mode_   = true;
            convex_mpc_motion_gaits_->duration_mode_ = true;
        }
        else {
            convex_mpc_loco_gaits_->duration_mode_   = false;
            convex_mpc_motion_gaits_->duration_mode_ = false;
        }

        first_locomotion_run_ = false;
    }

    if ( public_iter_ >= ( int )duration_ ) {
        first_locomotion_run_ = true;
        public_iter_          = 0;
    }
    public_iter_++;

    // Contact state logic
    // estimateContact();
    // double startTime = getCPUtime();

    // Reset weight vector
    weight_vec_.setZero( 6, 1 );

    // Check trans_flag_
    if ( convex_mpc_loco_gaits_->gait_check_transition_ || convex_mpc_motion_gaits_->gait_check_transition_ ) {
        trans_flag_ = true;
    }
    else {
        trans_flag_ = false;
    }

    // Choose controller based on GaitId
    convex_switch_index_last_ = convex_switch_index_;
    if ( trans_flag_ || convex_mpc_loco_gaits_->push_recovery_flag_ ) {
        if ( this->data_->command->mode != kLocomotion ) {
            convex_switch_index_ = convex_switch_index_last_;
        }
        else if ( this->data_->command->gait_id <= GaitId::kSpecialPronk - 1 ) {
            convex_switch_index_ = 1;
        }
        else {
            convex_switch_index_ = 2;
        }
    }
    // Run initialization when cMPC changes
    if ( convex_switch_index_ != convex_switch_index_last_ ) {
        convex_mpc_loco_gaits_->Initialize( *this->data_ );
        convex_mpc_loco_gaits_->gait_check_transition_ = true;
        convex_mpc_motion_gaits_->Initialize( *this->data_ );
        convex_mpc_motion_gaits_->gait_check_transition_ = true;
    }

    // wbc_ctrl_->SetFriction( this->data_->user_parameters->wbc_friction_default );
    switch ( convex_switch_index_ ) {
    case 1:  // LocoGaits
    {
        if ( cmd->cmd_source == kCyberdog2LcmCmd || cmd->cmd_source == kCyberdogLcmCmd ) {
            if ( cmd->value & 0x02 )
                this->data_->user_parameters->use_energy_saving_mode = 0;
            else
                this->data_->user_parameters->use_energy_saving_mode = 1;
        }
        if ( convex_mpc_loco_gaits_->current_gait_ != GaitId::kStand && convex_mpc_loco_gaits_->current_gait_ != GaitId::kStandNoPr
             && convex_mpc_loco_gaits_->current_gait_ != GaitId::kStandPassive ) {
            convex_mpc_motion_gaits_->gait_check_transition_ = false;
            convex_mpc_loco_gaits_->RunMpc< T >( *this->data_ );
#ifdef SEPERATE_MPC_ANOTHER_THREAD
            // set signal of solve mpc
            if ( convex_mpc_loco_gaits_->start_solve_mpc_ )
                sem_post( &solve_mpc_flag_ );
#endif

            for ( int i = 0; i < 6; i++ )
                weight_vec_[ i ] = convex_mpc_loco_gaits_->floating_base_weight_wbc_[ i ];
            wbc_ctrl_->SetFloatingBaseWeight( weight_vec_ );
            wbc_ctrl_->SetFriction( convex_mpc_loco_gaits_->mu_for_wbc_ );
            wbc_ctrl_->SetTaskPD( convex_mpc_loco_gaits_->kp_body_for_wbc_, convex_mpc_loco_gaits_->kd_body_for_wbc_, convex_mpc_loco_gaits_->kp_ori_for_wbc_, convex_mpc_loco_gaits_->kd_ori_for_wbc_,
                                  convex_mpc_loco_gaits_->kp_foot_for_wbc_, convex_mpc_loco_gaits_->kd_foot_for_wbc_, convex_mpc_loco_gaits_->kp_joint_for_wbc_,
                                  convex_mpc_loco_gaits_->kd_joint_for_wbc_ );

            for ( int leg( 0 ); leg < 4; ++leg ) {
                leg_pos_des_backup[ leg ] = this->data_->leg_controller->commands_[ leg ].p_des;
                leg_vel_des_backup[ leg ] = this->data_->leg_controller->commands_[ leg ].v_des;
                leg_kp_des_backup[ leg ]  = this->data_->leg_controller->commands_[ leg ].kp_cartesian;
                leg_kd_des_backup[ leg ]  = this->data_->leg_controller->commands_[ leg ].kd_cartesian;
            }

            if ( this->data_->user_parameters->use_wbc > 0.9 ) {
                wbc_data_->body_pos_des = convex_mpc_loco_gaits_->body_pos_des_;
                wbc_data_->body_vel_des = convex_mpc_loco_gaits_->body_vel_des_;
                wbc_data_->body_acc_des = convex_mpc_loco_gaits_->body_acc_des_;

                wbc_data_->body_rpy_des = convex_mpc_loco_gaits_->body_rpy_des_;
                wbc_data_->body_omg_des = convex_mpc_loco_gaits_->body_omg_des_;

                for ( size_t i( 0 ); i < 4; ++i ) {
                    wbc_data_->foot_pos_des[ i ]       = convex_mpc_loco_gaits_->foot_pos_des_[ i ];
                    wbc_data_->foot_vel_des[ i ]       = convex_mpc_loco_gaits_->foot_vel_des_[ i ];
                    wbc_data_->foot_acc_des[ i ]       = convex_mpc_loco_gaits_->foot_acc_des_[ i ];
                    wbc_data_->reaction_force_des[ i ] = convex_mpc_loco_gaits_->foot_force_des_[ i ];
                }
                wbc_data_->contact_state = convex_mpc_loco_gaits_->contact_states_;
                wbc_ctrl_->Run( wbc_data_, *this->data_ );
            }
            for ( int leg( 0 ); leg < 4; ++leg ) {
                // this->data_->leg_controller->commands_[leg].p_des = leg_pos_des_backup[leg];
                this->data_->leg_controller->commands_[ leg ].v_des = leg_vel_des_backup[ leg ];
                // this->data_->leg_controller->commands_[leg].kp_cartesian = leg_kp_des_backup[leg];
                this->data_->leg_controller->commands_[ leg ].kd_cartesian = leg_kd_des_backup[ leg ];
            }
        }
        else {
            convex_mpc_motion_gaits_->gait_check_transition_ = false;
            convex_mpc_loco_gaits_->RunStand();
            wbc_ctrl_->SetFloatingBaseWeight( 1000. );
            wbc_data_->body_pos_des = convex_mpc_loco_gaits_->body_pos_des_;
            wbc_data_->body_vel_des.setZero();
            wbc_data_->body_acc_des.setZero();
            wbc_data_->body_rpy_des      = convex_mpc_loco_gaits_->body_rpy_des_;
            wbc_data_->body_rpy_des[ 2 ] = ( this->data_->state_estimator->GetResult() ).rpy[ 2 ];
            wbc_data_->body_omg_des.setZero();

            for ( size_t i( 0 ); i < 4; ++i ) {
                wbc_data_->foot_pos_des[ i ].setZero();
                wbc_data_->foot_vel_des[ i ].setZero();
                wbc_data_->foot_acc_des[ i ].setZero();
                wbc_data_->reaction_force_des[ i ].setZero();
                wbc_data_->reaction_force_des[ i ][ 2 ] = this->data_->quadruped->body_mass_ * 0.25;
                wbc_data_->contact_state[ i ]           = 0.5;
            }
            wbc_ctrl_->Run( wbc_data_, *this->data_ );
        }

        // Update gait infro to ControlFsm
        this->data_->robot_current_state->gait_id          = convex_mpc_loco_gaits_->current_gait_;
        this->data_->robot_current_state->gait_check_trans = convex_mpc_loco_gaits_->gait_check_transition_;
        this->data_->robot_current_state->gait_allow_trans = convex_mpc_loco_gaits_->gait_allow_transition_;
        convex_mpc_loco_gaits_->GetSlopeState( this->data_->robot_current_state->step_on_slope );

        break;
    }
    case 2:  // MotionGaits
    {
        // initial usergait
        if ( gait_id_cmd_last != GaitId::kUserGait && cmd->gait_id == GaitId::kUserGait ) {
            convex_mpc_motion_gaits_->InitUserGaits( true );
        }
        convex_mpc_loco_gaits_->gait_check_transition_ = false;
        convex_mpc_motion_gaits_->RunMpc< T >( *this->data_ );
#ifdef SEPERATE_MPC_ANOTHER_THREAD
        // set signal of solve mpc
        if ( convex_mpc_motion_gaits_->start_solve_mpc_ )
            sem_post( &solve_mpc_flag_ );
#endif

        for ( int i = 0; i < 6; i++ )
            weight_vec_[ i ] = convex_mpc_motion_gaits_->floating_base_weight_wbc_[ i ];
        wbc_ctrl_->SetFloatingBaseWeight( weight_vec_ );
        wbc_ctrl_->SetFriction( convex_mpc_motion_gaits_->mu_for_wbc_ );
        wbc_ctrl_->SetTaskPD( convex_mpc_motion_gaits_->kp_body_for_wbc_, convex_mpc_motion_gaits_->kd_body_for_wbc_, convex_mpc_motion_gaits_->kp_ori_for_wbc_,
                              convex_mpc_motion_gaits_->kd_ori_for_wbc_, convex_mpc_motion_gaits_->kp_foot_for_wbc_, convex_mpc_motion_gaits_->kd_foot_for_wbc_,
                              convex_mpc_motion_gaits_->kp_joint_for_wbc_, convex_mpc_motion_gaits_->kd_joint_for_wbc_ );

        for ( int leg( 0 ); leg < 4; ++leg ) {
            leg_pos_des_backup[ leg ] = this->data_->leg_controller->commands_[ leg ].p_des;
            leg_vel_des_backup[ leg ] = this->data_->leg_controller->commands_[ leg ].v_des;
            leg_kp_des_backup[ leg ]  = this->data_->leg_controller->commands_[ leg ].kp_cartesian;
            leg_kd_des_backup[ leg ]  = this->data_->leg_controller->commands_[ leg ].kd_cartesian;
        }

        if ( this->data_->user_parameters->use_wbc > 0.9 ) {
            wbc_data_->body_pos_des = convex_mpc_motion_gaits_->body_pos_des_;
            wbc_data_->body_vel_des = convex_mpc_motion_gaits_->body_vel_des_;
            wbc_data_->body_acc_des = convex_mpc_motion_gaits_->body_acc_des_;

            wbc_data_->body_rpy_des = convex_mpc_motion_gaits_->body_rpy_des_;
            wbc_data_->body_omg_des = convex_mpc_motion_gaits_->body_omg_des_;

            for ( size_t i( 0 ); i < 4; ++i ) {
                wbc_data_->foot_pos_des[ i ]       = convex_mpc_motion_gaits_->foot_pos_des_[ i ];
                wbc_data_->foot_vel_des[ i ]       = convex_mpc_motion_gaits_->foot_vel_des_[ i ];
                wbc_data_->foot_acc_des[ i ]       = convex_mpc_motion_gaits_->foot_acc_des_[ i ];
                wbc_data_->reaction_force_des[ i ] = convex_mpc_motion_gaits_->foot_force_des_[ i ];
            }
            wbc_data_->contact_state = convex_mpc_motion_gaits_->contact_states_;
            wbc_ctrl_->Run( wbc_data_, *this->data_ );
        }
        for ( int leg( 0 ); leg < 4; ++leg ) {
            // this->data_->leg_controller->commands_[leg].p_des = leg_pos_des_backup[leg];
            this->data_->leg_controller->commands_[ leg ].v_des = leg_vel_des_backup[ leg ];
            // this->data_->leg_controller->commands_[leg].kp_cartesian = leg_kp_des_backup[leg];
            this->data_->leg_controller->commands_[ leg ].kd_cartesian = leg_kd_des_backup[ leg ];
        }
        // Update gait infro to ControlFsm
        this->data_->robot_current_state->gait_id          = convex_mpc_motion_gaits_->current_gait_;
        this->data_->robot_current_state->gait_check_trans = convex_mpc_motion_gaits_->gait_check_transition_;
        this->data_->robot_current_state->gait_allow_trans = convex_mpc_motion_gaits_->gait_allow_transition_;

        break;
    }
    }
    if ( this->data_->robot_current_state->gait_check_trans )
        this->data_->robot_current_state->gait_cmd_used = cmd->gait_id;
    gait_id_cmd_last = cmd->gait_id;

    // double stopTime = getCPUtime();
    // std::cout<<"time cost(ms) : "<<1000*(stopTime - startTime)<<std::endl;
    // static std::ofstream log_SimTau("../log_SimTau.csv", std::ios::out);
    //  log_SimTau<< 1000*(stopTime - startTime)<<",";
    //  log_SimTau<< std::endl;
}

#ifdef SEPERATE_MPC_ANOTHER_THREAD
/**
 * @brief Run MPC in another thread to avoid WBC thread timeout.
 *
 */
template < typename T > void FsmStateLocomotion< T >::RunSolveMpcAnotherThread() {
    while ( true ) {
        sem_wait( &solve_mpc_flag_ );
        static Timer solve_mpc_time_cost;
        solve_mpc_time_cost.StartTimer();
        switch ( convex_switch_index_ ) {
        case 1:
            convex_mpc_loco_gaits_->SolveMpcAnotherThread();
            break;
        case 2:
            convex_mpc_motion_gaits_->SolveMpcAnotherThread();
            break;
        }
        if ( mpc_thread_log_f_ )
            mpc_thread_log_f_ << solve_mpc_time_cost.GetElapsedSeconds() << std::endl;
        usleep( 100 );
    }
}
#endif

template class FsmStateLocomotion< float >;
