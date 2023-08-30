#include "fsm_states/control_fsm.hpp"
#include "control_flags.hpp"
// #include <rt/rt_rc_interface.h>

/**
 * @brief Construct a new Control Fsm<  T >:: Control Fsm object
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 *
 * @param quadruped the quadruped information
 * @param state_estimator contains the estimated states
 * @param leg_controller interface to the leg controllers
 * @param cmd get the desired control cmd
 * @param control_parameters passes in the control parameters from the GUI
 * @param visualization_data data for visualization
 * @param user_parameters passes in the user parameters from the GUI
 * @param robot_current_state passes the robot current state
 */
template < typename T >
ControlFsm< T >::ControlFsm( Quadruped< T >* quadruped, StateEstimatorContainer< T >* state_estimator, LegController< T >* leg_controller, const MotionControlCommand* cmd,
                             RobotControlParameters* control_parameters, VisualizationData* visualization_data, UserParameters* user_parameters, RobotCurrentState< T >* robot_current_state ) {
    // Add the pointers to the ControlFsmData struct
    data.quadruped           = quadruped;
    data.state_estimator     = state_estimator;
    data.leg_controller      = leg_controller;
    data.command             = cmd;
    data.control_parameters  = control_parameters;
    data.visualization_data  = visualization_data;
    data.user_parameters     = user_parameters;
    data.robot_current_state = robot_current_state;

    // Initialize and add all of the FSM States to the state list
    states_list_.invalid       = nullptr;
    states_list_.passive       = new FsmStatePassive< T >( &data );
    states_list_.posectrl      = new FsmStatePoseCtrl< T >( &data );
    states_list_.balance_stand = new FsmStateBalanceStand< T >( &data );
    states_list_.locomotion    = new FsmStateLocomotion< T >( &data );
    states_list_.recoveryStand = new FSMStateRecoveryStand< T >( &data );

    // TODO: remove it or add corresponding logic
    states_list_.pure_damper     = new FsmStatePureDamper< T >( &data );
    states_list_.two_leg_stand   = new FsmStateTwoLegStand< T >( &data );
    states_list_.jump_3d         = new FsmStateJump3d< T >( &data );
    states_list_.motor_ctrl      = new FsmStateMotorCtrl< T >( &data );
    states_list_.forcejump       = new FsmStateForceJump< T >( &data );
    states_list_.lifted          = new FsmStateLifted< T >( &data );

    states_list_.rl_reset = new FsmStateRlReset< T >( &data );
    states_list_.rl_rapid = new FsmStateRlRapid< T >( &data );

    safety_checker_ = new SafetyChecker< T >( &data );

    // Initialize the FSM with the Passive FSM State
    Initialize();
}

/**
 * @brief  * Initialize the Control FSM with the default settings. Should be set to
 * Passive state and Normal operation mode.
 *
 */
template < typename T > void ControlFsm< T >::Initialize() {
    // Initialize a new FSM State with the control data
    current_state_ = states_list_.passive;

    // Enter the new current state cleanly
    current_state_->OnEnter();

    // Initialize to not be in transition
    next_state_ = current_state_;

    // Initialize FSM mode to normal operation
    operating_mode_ = FsmOperatingMode::kNormal;

    iter_ = 0;

    safety_check_error_ = false;

    max_foot_pos_for_lift_ << 0.15, 0.15, 0.245;  // For robot lifted
    min_joint_torque_for_lift_ << 0.0, 0.0, -0.5;
    max_omega_for_lift_ << 0.0, 0.5, 0.0;
    max_rpy_for_lift_ << 0.0, 0.1, 0.0;
    max_leg_length_for_lift_ = 0.285;

    robot_type_ = this->data.quadruped->robot_type_;

    static Vec3< float > se_ori_cali_offset = Vec3< float >::Zero();
    static Vec3< float > se_ori_cali_gain   = Vec3< float >::Zero();
    for ( int i = 0; i < 3; i++ ) {
        se_ori_cali_offset( i ) = data.control_parameters->se_ori_cali_offset( i );
        se_ori_cali_gain( i )   = data.control_parameters->se_ori_cali_gain( i );
    }
    this->data.state_estimator->SetOriCaliOffset( se_ori_cali_offset );
    this->data.state_estimator->SetOriCaliGain( se_ori_cali_gain );
}

/**
 * @brief Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 *
 */
template < typename T > void ControlFsm< T >::RunFsm() {
    static int estop_iter = 0;
    static int edamp_iter = 0;
    static int elift_iter = 0;
    // Check the robot state for safe operation
    operating_mode_ = SafetyPreCheck();
    if ( operating_mode_ == FsmOperatingMode::kEstop || edamp_iter >= 1200 ) {
        safety_check_error_ = true;
    }
    else {
        safety_check_error_ = false;
    }
    // Run the robot control code if operating mode is not unsafe
    if ( operating_mode_ != FsmOperatingMode::kEstop ) {
        estop_iter = 0;
        // Run normal controls if no transition is detected
        if ( operating_mode_ == FsmOperatingMode::kNormal || ( ( FsmStateName )this->data.command->mode != next_state_name_ && operating_mode_ == FsmOperatingMode::kTransitioning ) ) {
            // Check the current state for any transition
            next_state_name_ = current_state_->CheckTransition();

            CheckBatteryAndMotor();

            // Detect a commanded transition
            if ( next_state_name_ != current_state_->state_name_ ) {
                // Set the FSM operating mode to transitioning
                operating_mode_ = FsmOperatingMode::kTransitioning;

                // Get the next FSM State by name
                next_state_ = GetNextState( next_state_name_ );
                std::cout << "[CONTROL FSM] transition fsm from " << current_state_->state_string_.c_str() << " to " << next_state_->state_string_.c_str() << std::endl;
                // Print transition initialized info
                // PrintInfo(1);
                current_state_->transition_data_.ban_trans_flag = 0;
            }
            else {
                if ( low_bat_ )
                    current_state_->transition_data_.ban_trans_flag = 7;
                else if ( over_heat_ )
                    current_state_->transition_data_.ban_trans_flag = 6;
                else if ( ( int )next_state_name_ != this->data.command->mode )
                    current_state_->transition_data_.ban_trans_flag = 5;
                else
                    current_state_->transition_data_.ban_trans_flag = 0;
                // Run the iteration for the current state normally
                current_state_->Run();
            }
        }

        // Run the transition code while transition is occuring
        if ( operating_mode_ == FsmOperatingMode::kTransitioning ) {
            transition_data_ = current_state_->Transition();

            // Check the robot state for safe operation
            SafetyPostCheck();

            // Run the state transition
            if ( transition_data_.done ) {
                // Exit the current state cleanly
                current_state_->OnExit();

                // Initialize user gaits when exits passive
                if ( current_state_->state_name_ == FsmStateName::kOff && robot_type_ != RobotType::CYBERDOG ) {
                    states_list_.locomotion->InitUserGaits();
                }

                // Print finalizing transition info
                // PrintInfo(2);

                // Complete the transition
                current_state_ = next_state_;

                // Enter the new current state cleanly
                current_state_->OnEnter();

                // Return the FSM to normal operation mode
                operating_mode_ = FsmOperatingMode::kNormal;
            }
            else {
                next_state_name_ = current_state_->CheckTransition();
                if ( next_state_name_ == current_state_->state_name_ ) {
                    // Set the FSM operating mode to transitioning
                    operating_mode_ = FsmOperatingMode::kNormal;
                }
            }
        }
        else {
            // Check the robot state for safe operation
            SafetyPostCheck();
        }

        if ( operating_mode_ == FsmOperatingMode::kEdamp ) {
            if ( current_state_->state_name_ != FsmStateName::kPureDamper ) {
                if ( edamp_iter < 1 ) {
                    current_state_->OnExit();
                }

                current_state_ = states_list_.pure_damper;
                if ( edamp_iter < 1 ) {
                    current_state_->OnEnter();
                }
            }
            if ( edamp_iter > 1550 || current_state_->CheckTransition() == FsmStateName::kRlReset ) {
                // Check the current state for any transition
                next_state_name_ = current_state_->CheckTransition();
            }
            else {
                usleep( 100 );
                next_state_name_ = current_state_->state_name_;
            }

            // Detect a commanded transition
            if ( next_state_name_ != current_state_->state_name_ ) {
                // Set the FSM operating mode to transitioning
                operating_mode_ = FsmOperatingMode::kTransitioning;

                // Get the next FSM State by name
                next_state_ = GetNextState( next_state_name_ );
                if ( edamp_iter % 50 == 0 )
                    std::cout << "[CONTROL FSM] transition fsm from " << current_state_->state_string_.c_str() << " to " << next_state_->state_string_.c_str() << std::endl;
            }
            else {
                // Run the iteration for the current state normally
                current_state_->Run();
            }

            edamp_iter++;
        }
        else {
            edamp_iter = 0;
        }

        if ( operating_mode_ == FsmOperatingMode::kRobotLifted ) {
            if ( elift_iter < 1 ) {
                current_state_->OnExit();
            }

            if ( robotlifted_error_ ) {  // robotlifted_error_ triggers lifted protection
                current_state_ = states_list_.lifted;
            }
            else {
                current_state_ = states_list_.pure_damper;
            }

            if ( elift_iter < 1 ) {
                current_state_->OnEnter();
            }

            if ( elift_iter > 1000 ) {
                // Check the current state for any transition
                next_state_name_ = current_state_->CheckTransition();
            }
            else {
                usleep( 100 );
                next_state_name_ = current_state_->state_name_;
            }

            // Detect a commanded transition
            if ( next_state_name_ != current_state_->state_name_ ) {
                // Set the FSM operating mode to transitioning
                operating_mode_ = FsmOperatingMode::kTransitioning;

                // Get the next FSM State by name
                next_state_ = GetNextState( next_state_name_ );
                std::cout << "[CONTROL FSM] transition fsm from " << current_state_->state_string_.c_str() << " to " << next_state_->state_string_.c_str() << std::endl;
            }
            else {
                // Run the iteration for the current state normally
                current_state_->Run();
            }

            elift_iter++;
        }
        else {
            elift_iter = 0;
        }
    }
    else {  // if ESTOP
        if ( estop_iter++ % 2000 == 0 ) {
            printf( "[CONTROL FSM] Robot is in ESTOP \n" );
        }

        current_state_ = states_list_.passive;
        current_state_->OnEnter();
        next_state_name_ = current_state_->state_name_;

        if ( estop_iter >= 2000 )
            operating_mode_ = FsmOperatingMode::kNormal;
    }

    PrintInfo( 0 );
    iter_++;
}

/**
 * @brief Checks the robot state for safe operation conditions. If it is in
 * an unsafe state, it will not run the normal control code until it
 * is safe to operate again.
 *
 * @return FsmOperatingMode the appropriate operating mode
 */
template < typename T > FsmOperatingMode ControlFsm< T >::SafetyPreCheck() {
    // Check for safe orientation if the current state requires it
    auto& cmd = this->data.command;
    if ( current_state_->check_safe_orientation_ && cmd->mode != MotionMode::kRecoveryStand ) {
        if ( !safety_checker_->CheckSafeOrientation() ) {
            operating_mode_    = FsmOperatingMode::kEstop;
            orientation_error_ = true;
            std::cout << "[CONTROL FSM] Safety: Orientation Safety Ceck FAIL" << std::endl;
        }
        else {
            orientation_error_ = false;
        }
    }

    // Check robot lifted
    if ( current_state_->check_robot_lifted_ ) {
        if ( robot_type_ == RobotType::CYBERDOG2 ) {
            if ( current_state_ == states_list_.balance_stand ) {
                if ( this->data.user_parameters->use_energy_saving_mode > 0.9 ) {
                    max_foot_pos_for_lift_ << 0.0, 0.0, 0.255;
                    min_joint_torque_for_lift_ << 0.0, 0.0, -0.25;
                    max_omega_for_lift_ << 0.0, 0.5, 0.0;
                    max_rpy_for_lift_ << 0.0, 0.2, 0.0;
                    max_leg_length_for_lift_ = 0.255 + 0.018;
                }
                else {
                    max_foot_pos_for_lift_ << 0.0, 0.0, 0.230;
                    min_joint_torque_for_lift_ << 0.0, 0.0, -0.25;
                    max_omega_for_lift_ << 0.0, 0.5, 0.0;
                    max_rpy_for_lift_ << 0.0, 0.2, 0.0;
                    max_leg_length_for_lift_ = 0.255;
                }
            }
            else if ( current_state_ == states_list_.locomotion ) {
                if ( this->data.user_parameters->use_energy_saving_mode > 0.9 ) {
                    max_foot_pos_for_lift_ << 0.0, 0.0, 0.260;
                    min_joint_torque_for_lift_ << 0.0, 0.0, -0.40;
                    max_omega_for_lift_ << 0.0, 0.3, 0.0;
                    max_rpy_for_lift_ << 0.0, 0.12, 0.0;
                    max_leg_length_for_lift_ = 0.1;  // max_leg_length_for_lift_ is not considered
                }
                else {
                    max_foot_pos_for_lift_ << 0.0, 0.0, 0.230;
                    min_joint_torque_for_lift_ << 0.0, 0.0, -0.40;
                    max_omega_for_lift_ << 0.0, 0.3, 0.0;
                    max_rpy_for_lift_ << 0.0, 0.12, 0.0;
                    max_leg_length_for_lift_ = 0.1;
                }
            }
        }
        else {
            if ( current_state_ == states_list_.balance_stand ) {
                max_foot_pos_for_lift_ << 0.0, 0.0, 0.31;
                min_joint_torque_for_lift_ << 0.0, 0.0, -0.5;
                max_omega_for_lift_ << 0.0, 0.5, 0.0;
                max_rpy_for_lift_ << 0.0, 0.2, 0.0;
                max_leg_length_for_lift_ = 0.33;
            }
            else if ( current_state_ == states_list_.locomotion ) {
                max_foot_pos_for_lift_ << 0.0, 0.0, 0.33;
                min_joint_torque_for_lift_ << 0.0, 0.0, -0.5;
                max_omega_for_lift_ << 0.0, 0.5, 0.0;
                max_rpy_for_lift_ << 0.0, 0.2, 0.0;
                max_leg_length_for_lift_ = 0.36;
            }
        }
        if ( !safety_checker_->CheckRobotLifted( max_foot_pos_for_lift_, max_leg_length_for_lift_, min_joint_torque_for_lift_, max_omega_for_lift_, max_rpy_for_lift_ ) ) {
            // operating_mode_ = FsmOperatingMode::kEstop;
            operating_mode_    = FsmOperatingMode::kRobotLifted;
            robotlifted_error_ = true;
            std::cout << "[CONTROL FSM] Safety: Robot is lifted" << std::endl;
            std::cout << "[CONTROL FSM] current_gait: " << this->data.robot_current_state->gait_id << std::endl;
            std::cout << "[CONTROL FSM] velocity_in_body_frame.norm(): " << this->data.state_estimator->GetResult().velocity_in_body_frame.norm() << std::endl;
        }
        else {
            robotlifted_error_ = false;
        }
    }

    auto& state_estimator_result = this->data.state_estimator->GetResult();

    // if ( state_estimator_result.is_battery_low ) { // NX board will handle is_battery_low case, unnecessary here anymore
    //     static int batIter = 0;
    //     if ( batIter % 1000 == 0 ) {
    //         // std::cout << "Battery less than 19.5v && 15%, ESTOP!!! " << std::endl;
    //         std::cout << "[CONTROL FSM] Battery too low, ESTOP!!! " << std::endl;
    //     }
    //     batIter++;

    //     if ( current_state_ == states_list_.passive ) {
    //         operating_mode_ = FsmOperatingMode::kEstop;
    //     }
    //     else {
    //         if ( operating_mode_ != FsmOperatingMode::kEstop ) {
    //             operating_mode_ = FsmOperatingMode::kEdamp;
    //         }

    //         if ( low_bat_iter_ > 1500 ) {
    //             operating_mode_ = FsmOperatingMode::kEstop;
    //         }
    //         low_bat_iter_++;
    //     }
    // }
    // else {
    //     low_bat_iter_ = 0;
    // }

    if ( state_estimator_result.is_charging ) {
        static int batIter = 0;
        if ( batIter++ % 60000 == 0 ) {
            std::cout << "[CONTROL FSM] Battery is charging, ESTOP!!! " << std::endl;
        }

        if ( current_state_ == states_list_.passive ) {
            operating_mode_ = FsmOperatingMode::kEstop;
        }
        else {
            if ( operating_mode_ != FsmOperatingMode::kEstop ) {
                operating_mode_ = FsmOperatingMode::kEdamp;
            }

            if ( charging_iter_ > 1250 ) {
                operating_mode_ = FsmOperatingMode::kEstop;
            }
            charging_iter_++;
        }
    }
    else {
        charging_iter_ = 0;
    }

    // Default is to return the current operating mode
    return operating_mode_;
}

/**
 * @brief Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * @return FsmOperatingMode
 */
template < typename T > FsmOperatingMode ControlFsm< T >::SafetyPostCheck() {
    // Check for safe desired foot positions
    if ( current_state_->check_desired_foot_position_ ) {
        T max_leg_percent     = 6;
        T max_hip_pitch_angle = Deg2Rad( 60. );  // 60 degree
        // T max_hip_roll_angle = Deg2Rad( 39. ); // smaller than 39 degree for safety and adapt to new mechanic design
        T max_hip_roll_angle;
        if ( abs( this->data.quadruped->abad_lower_bound_ ) >= abs( this->data.quadruped->abad_upper_bound_ ) )
            max_hip_roll_angle = abs( this->data.quadruped->abad_upper_bound_ );
        else
            max_hip_roll_angle = abs( this->data.quadruped->abad_lower_bound_ );
        if ( this->data.command->mode == MotionMode::kQpStand ) {
            max_leg_percent = 12;
        }
        else if ( this->data.command->mode == MotionMode::kLocomotion ) {
            if ( this->data.command->gait_id == GaitId::kBound ) {
                max_leg_percent     = 12;
                max_hip_pitch_angle = Deg2Rad( 45. );  // 45 degree
            }
            else {
                max_leg_percent     = 6;
                max_hip_pitch_angle = Deg2Rad( 60. );  // 55 degree
            }
        }
        if ( !safety_checker_->CheckLegWorkspace( max_hip_roll_angle, max_hip_pitch_angle, max_leg_percent ) ) {
            operating_mode_ = FsmOperatingMode::kEdamp;
            foot_pos_error_ = safety_checker_->foot_pos_error_flag_;
        }
    }
    else {
        foot_pos_error_ = 0;
    }

    // Check for safe desired feedforward forces
    // if ( current_state_->check_feed_forward_force_ ) {
    //     safety_checker_->CheckForceFeedForward();
    // }

    // Default is to return the current operating mode
    return operating_mode_;
}

/**
 * @brief Returns the approptiate next FSM State when commanded.
 *
 * @param state_name commanded enumerated state name
 * @return FsmState< T >* next FSM state
 */
template < typename T > FsmState< T >* ControlFsm< T >::GetNextState( FsmStateName state_name ) {
    // TODO: remove FsmStateName
    // Choose the correct FSM State by enumerated state name
    switch ( state_name ) {
    case FsmStateName::kInvalid:
        return states_list_.invalid;

    case FsmStateName::kOff:
        return states_list_.passive;

    case FsmStateName::kPoseCtrl:
        return states_list_.posectrl;

    case FsmStateName::kQpStand:
        return states_list_.balance_stand;


    case FsmStateName::kLocomotion:
        return states_list_.locomotion;

    case FsmStateName::kRecoveryStand:
        return states_list_.recoveryStand;

    case FsmStateName::kPureDamper:
        return states_list_.pure_damper;

    case FsmStateName::kTwoLegStand:
        return states_list_.two_leg_stand;

    case FsmStateName::kJump3d:
        return states_list_.jump_3d;

    case FsmStateName::kMotorCtrl:
        return states_list_.motor_ctrl;

    case FsmStateName::kForceJump:
        return states_list_.forcejump;


    case FsmStateName::kLifted:
        return states_list_.lifted;

    case FsmStateName::kRlReset:
        return states_list_.rl_reset;

    case FsmStateName::kRlRapid:
        return states_list_.rl_rapid;


    default:
        return states_list_.invalid;
    }
}

/**
 * @brief Prints Control FSM info at regular intervals and on important events
 * such as transition initializations and finalizations. Separate function
 * to not clutter the actual code.
 *
 * @param opt
 */
template < typename T > void ControlFsm< T >::PrintInfo( int opt ) {
    switch ( opt ) {
    case 0:  // Normal printing case at regular intervals
        // Increment printing iteration
        print_iter_++;

        // Print at commanded frequency
        if ( print_iter_ == print_num_ ) {
            // std::cout << "[CONTROL FSM] Printing FSM Info...\n";
            // std::cout << "---------------------------------------------------------\n";
            // std::cout << "Iteration: " << iter_ << "\n";
            if ( operating_mode_ == FsmOperatingMode::kNormal ) {
                std::cout << "[CONTROL FSM] Operating Mode: NORMAL in " << current_state_->state_string_ << "\n";
            }
            else if ( operating_mode_ == FsmOperatingMode::kTransitioning ) {
                std::cout << "[CONTROL FSM] Operating Mode: kTransitioning from " << current_state_->state_string_ << " to " << next_state_->state_string_ << "\n";
            }
            else if ( operating_mode_ == FsmOperatingMode::kEstop ) {
                std::cout << "[CONTROL FSM] Operating Mode: ESTOP\n";
            }
            std::cout << std::endl;

            // Reset iteration counter
            print_iter_ = 0;
        }

        break;

    case 1:  // Initializing FSM State transition
        std::cout << "[CONTROL FSM] Transition initialized from " << current_state_->state_string_ << " to " << next_state_->state_string_ << "\n" << std::endl;

        break;

    case 2:  // Finalizing FSM State transition
        std::cout << "[CONTROL FSM] Transition finalizing from " << current_state_->state_string_ << " to " << next_state_->state_string_ << "\n" << std::endl;

        break;
    }
}

/**
 * @brief Check battery and motor states
 *
 */
template < typename T > void ControlFsm< T >::CheckBatteryAndMotor() {
    auto& state_estimator_result = this->data.state_estimator->GetResult();
    max_temp_                    = 0;
    for ( int i( 0 ); i < 12; i++ ) {
        float Temper = this->data.leg_controller->datas_[ i / 3 ].tmp_actual( i % 3 );
        if ( Temper > max_temp_ )
            max_temp_ = Temper;
    }
    over_heat_ = false;
    low_bat_   = false;
    if ( over_heat_ || low_bat_ ) {
        next_state_name_ = current_state_->state_name_;  // Ban trans cause over heat
        if ( !warn_log_ ) {
            warn_log_ = 1;
            std::cout << "[Contrl_FSM]: Ban Flip, Cause Bat_soc:" << ( int )state_estimator_result.battery_soc << "  Motor_Tmp:" << max_temp_ << std::endl;
        }
    }
    else
        warn_log_ = 0;
}

// template class ControlFsm<double>; This should be fixed... need to make
// RobotRunner a template
template class ControlFsm< float >;
