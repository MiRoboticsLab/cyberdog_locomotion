/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "fsm_states/fsm_state_recoverystand.hpp"
#include "utilities/utilities_print.hpp"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param control_fsm_data holds all of the relevant control data
 */
template < typename T > FSMStateRecoveryStand< T >::FSMStateRecoveryStand( ControlFsmData< T >* control_fsm_data ) : FsmState< T >( control_fsm_data, FsmStateName::kRecoveryStand, "recovery_stand" ) {
    // Do nothing
    // Set the pre controls safety checks
    this->check_safe_orientation_ = false;
    this->check_robot_lifted_     = false;

    // Post control safety checks
    this->check_desired_foot_position_ = false;
    this->check_feed_forward_force_    = false;

    zero_vec3.setZero();
    // goal configuration
    // Folding
    for ( size_t i( 0 ); i < 4; ++i )
        if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 )
            fold_jpos[ i ] << -0.0f, -1.38f, 2.5f;
        else
            fold_jpos[ i ] << -0.0f, -1.4f, 2.4f;

    T L1           = this->data_->quadruped->hip_link_length_;
    T L2           = this->data_->quadruped->knee_link_length_;
    T com_offset_x = 0.01;
    T H            = L1 * cos( -0.69 ) + L2 * cos( 0.7 );  // cyberdog2: 0.24, cyberdog: 0.32
    T D            = std::sqrt( H * H + com_offset_x * com_offset_x );
    T alpha2       = 3.141592653589793 - std::acos( ( L1 * L1 + L2 * L2 - D * D ) / ( 2.0 * L1 * L2 ) );
    T alpha1       = std::atan( com_offset_x / H ) - std::acos( ( L1 * L1 + D * D - L2 * L2 ) / ( 2.0 * L1 * D ) );
    // Stand Up
    for ( size_t i( 0 ); i < 4; ++i ) {
        stand_jpos[ i ] << 0.f, alpha1, alpha2;
    }
    flag_ = FoldLegs_;
}

template < typename T > void FSMStateRecoveryStand< T >::OnEnter() {
    // Default is to not transition
    this->next_state_name_ = this->state_name_;

    // Reset the transition data
    this->transition_data_.ResetTransitionDone();

    this->ready_for_switch_ = false;

    this->motion_progress_bar_ = 0;
    // Reset iteration counter
    state_iter_  = 0;
    drop_safety_ = 0;
    first_stand_ = true;
    liedown_ok_  = false;

    gesture_err_iter_ = 0;

    if ( this->data_->command->gait_id == 17 || this->data_->command->gait_id == 18 )
        this->data_->robot_current_state->gait_id = this->data_->command->gait_id;
    else
        this->data_->robot_current_state->gait_id = 0;

    // initial configuration, position
    for ( size_t i( 0 ); i < 4; ++i ) {
        Vec3< T > data_q = this->data_->leg_controller->datas_[ i ].q;
        Vec3< T > cmd_q  = this->data_->leg_controller->commands_[ i ].q_des;
        for ( size_t j( 0 ); j < 3; ++j ) {
            if ( fabs( data_q[ j ] - cmd_q[ j ] ) > 4 / 57.3 )
                initial_jpos[ i ][ j ] = data_q[ j ];
            else
                initial_jpos[ i ][ j ] = cmd_q[ j ];
        }
    }
#ifdef USE_ABSOLUTE_ODOM_FOR_ALL
    T body_height = this->data_->state_estimator->GetResult().height;
#else
    T body_height = this->data_->state_estimator->GetResult().position[ 2 ];
#endif
    if ( OnRightLegConfig() )
        flag_ = FoldLegs_;
    else
        flag_ = PrepareForFoldLegs_;
    if ( !UpsideDown() ) {  // Proper orientation
#ifdef USE_ABSOLUTE_ODOM_FOR_ALL
        if ( true ) {
#else
        // Cyberdog2: 0.43*(0.12+0.174)=0.123
        if ( ( 0.42 * ( this->data_->quadruped->hip_link_length_ + this->data_->quadruped->knee_link_length_ ) < body_height ) && ( body_height < 0.45 ) ) {
#endif
            printf( "[RecoveryStand] body height is %f; Stand Up \n", body_height );
            flag_ = StandUp_;
        }
        else {
            printf( "[RecoveryStand] body height is %f; Folding legs \n", body_height );
        }
    }
    else {
        printf( "[RecoveryStand] UpsideDown (%d) \n", UpsideDown() );
    }
    motion_start_iter_ = 0;
}

template < typename T > bool FSMStateRecoveryStand< T >::OnRightLegConfig() {
    Vec3< T > jointPos;
    for ( int leg( 0 ); leg < 4; leg++ ) {
        jointPos = this->data_->leg_controller->datas_[ leg ].q;
        if ( ( jointPos[ 1 ] < -0.5 * M_PI && jointPos[ 2 ] < 0.73 * M_PI ) || jointPos[ 1 ] < -0.73 * M_PI )
            RightLegConfigFlag[ leg ] = false;
        else
            RightLegConfigFlag[ leg ] = true;
    }
    std::cout << "[RecoveryStand] legs can foldlegs: " << RightLegConfigFlag.transpose() << std::endl;
    if ( RightLegConfigFlag[ 0 ] && RightLegConfigFlag[ 1 ] && RightLegConfigFlag[ 2 ] && RightLegConfigFlag[ 3 ] )
        return true;
    else
        return false;
}

template < typename T > bool FSMStateRecoveryStand< T >::UpsideDown() {
    if ( this->data_->state_estimator->GetResult().world2body_rotation_matrix( 2, 2 ) < -0.15 ) {
        return true;
    }
    return false;
}

template < typename T > int FSMStateRecoveryStand< T >::BeLookUpOrDown() {
    auto& datas = this->data_->leg_controller->datas_;
    float force[ 4 ];
    for ( int i = 0; i < 4; i++ )
        force[ i ] = sqrt( datas[ i ].foot_force_actual( 0 ) * datas[ i ].foot_force_actual( 0 ) + datas[ i ].foot_force_actual( 1 ) * datas[ i ].foot_force_actual( 1 )
                           + datas[ i ].foot_force_actual( 2 ) * datas[ i ].foot_force_actual( 2 ) );
    if ( this->data_->state_estimator->GetResult().world2body_rotation_matrix( 0, 2 ) > 0.6 && force[ 2 ] > 10 && force[ 3 ] > 10 )
        // look up
        return -1;
    else if ( this->data_->state_estimator->GetResult().world2body_rotation_matrix( 0, 2 ) < -0.6 && force[ 0 ] > 10 && force[ 1 ] > 10 )
        // look down
        return 1;
    return 0;
}

template < typename T > int FSMStateRecoveryStand< T >::SideLyingState() {
    // PrettyPrint( this->data_->state_estimator->GetResult().world2body_rotation_matrix, std::cout, "Rot" );
    // if(this->data_->state_estimator->GetResult().acceleration_in_body_frame[2] < 0){
    if ( this->data_->state_estimator->GetResult().world2body_rotation_matrix( 2, 2 ) >= -0.2 && this->data_->state_estimator->GetResult().world2body_rotation_matrix( 2, 2 ) <= 0.2 ) {
        if ( this->data_->state_estimator->GetResult().world2body_rotation_matrix( 1, 2 ) > 0.5 )
            return 1;
        else if ( this->data_->state_estimator->GetResult().world2body_rotation_matrix( 1, 2 ) < -0.5 )
            return -1;
    }
    return 0;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template < typename T > void FSMStateRecoveryStand< T >::Run() {
    this->data_->leg_controller->ZeroCommand();
    this->data_->leg_controller->legs_enabled_ = true;
    if ( motion_start_iter_ > state_iter_ )
        motion_start_iter_ = state_iter_;

    if ( this->data_->command->gait_id == 17 || this->data_->command->gait_id == 18 )
        this->data_->robot_current_state->gait_id = this->data_->command->gait_id;
    else
        this->data_->robot_current_state->gait_id = 0;

    switch ( flag_ ) {
    case StandUp_:
        StandUp( state_iter_ - motion_start_iter_ );
        break;
    case FoldLegs_:
        FoldLegs( state_iter_ - motion_start_iter_ );
        break;
    case LeftRollOver_:  // Recover from face the sky to stand
        LeftRollOver( state_iter_ - motion_start_iter_ );
        break;
    case RightRollOver_:  // Recover from face the sky to stand
        RightRollOver( state_iter_ - motion_start_iter_ );
        break;
    case SideLying_:  // When Fall on one side， recovery to FoldLegs_
        SideLying( state_iter_ - motion_start_iter_ );
        break;
    case LieDown_:  // Lie down from the left side to face the sky, Swinging leg, then RollOver
        LieDown( state_iter_ - motion_start_iter_ );
        break;
    case LieSide_:  // Lie down to the left side, Swinging leg, then recovery to FoldLegs_
        LieSide( state_iter_ - motion_start_iter_ );
        break;
    case PrepareForFoldLegs_:
        PrepareForFoldLegs( state_iter_ - motion_start_iter_ );
        break;
    case LookUpOrDown_:
        LookUpOrDown( state_iter_ - motion_start_iter_ );
        break;
    }
    if ( SaftyCheck() )
        ++state_iter_;
    // safty check for gesture, roll over and look up/down
    if ( UpsideDown() || BeLookUpOrDown() || SideLyingState() ) {
        this->ready_for_switch_ = false;
        gesture_err_iter_++;
        if ( gesture_err_iter_ % 30000 == 0 )
            std::cout << "[RecoveryStand] error gesture, please make robot stand correctly!" << std::endl;
    }
}
template < typename T > int FSMStateRecoveryStand< T >::SaftyCheck() {
    float t            = 0;
    float diff         = 0;
    int   err          = -1;
    int   hang_flag    = 0;
    float safety_time1 = 500;
    float safety_time2 = 450;

    Vec3< T > foot_force;

    for ( size_t i( 0 ); i < 4; ++i ) {
        if ( this->data_->leg_controller->datas_[ i ].foot_force_actual( 2 ) < 3 )
            hang_flag++;
    }
    if ( this->data_->state_estimator->GetResult().acceleration_in_world_frame( 2 ) > 16 && this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        drop_flag_++;
    }
    else {
        drop_flag_ = 0;
    }

    if ( ( drop_flag_ > 10 ) && ( hang_flag == 4 ) && flag_ != FoldLegs_ && flag_ != LeftRollOver_ && flag_ != RightRollOver_ && flag_ != SideLying_ && flag_ != LieSide_ && flag_ != LookUpOrDown_ ) {
        drop_safety_ = safety_time1;  // 1s for safety
        std::cout << "[RecoveryStand] Dropping Waring! low pd value for safy! " << std::endl;
    }
    if ( drop_safety_ > 0 ) {
        drop_safety_--;
        t = drop_safety_ > safety_time2 ? 0 : ( safety_time2 - drop_safety_ ) / safety_time2;
        for ( size_t i( 0 ); i < 4; ++i ) {
            this->data_->leg_controller->commands_[ i ].kp_joint << 10 + t * 70, 0, 0, 0, 10 + t * 70, 0, 0, 0, 10 + t * 50;
            this->data_->leg_controller->commands_[ i ].kd_joint << 1 + t * 1, 0, 0, 0, 1 + t * 0.5, 0, 0, 0, 0.2 + t * 1.3;
        }
    }

    for ( size_t i( 0 ); i < 4; ++i )
        for ( size_t j( 0 ); j < 3; ++j ) {
            diff = fabs( this->data_->leg_controller->commands_[ i ].q_des[ j ] - this->data_->leg_controller->datas_[ i ].q[ j ] );
            if ( j == 0 ) {
                if ( diff > 8 / 57.3 )
                    err = i * 3 + j;
            }
            if ( j == 1 ) {
                if ( diff > 8 / 57.3 )
                    err = i * 3 + j;
            }
            if ( j == 2 ) {
                if ( diff > 10 / 57.3 )
                    err = i * 3 + j;
            }
        }
    if ( err != -1 && ( drop_safety_ == 0 ) && flag_ != LeftRollOver_ && flag_ != RightRollOver_ && flag_ != LookUpOrDown_ ) {
        if ( stuck_count_++ % 501 == 500 )
            printf( "[RecoveryStand] SaftyCheck Warning lasting for %.1fs at Motor%d!\n", stuck_count_ * 0.002, err );
        if ( stuck_count_ > 501 ) {
            motion_start_iter_ = state_iter_ + 1;
            for ( size_t i( 0 ); i < 4; ++i )
                for ( size_t j( 0 ); j < 3; ++j )
                    initial_jpos[ i ][ j ] = this->data_->leg_controller->datas_[ i ].q( j );
        }
        if ( stuck_count_ > 5 )
            return 0;
    }
    else
        stuck_count_ = 0;
    return 1;
}

template < typename T > void FSMStateRecoveryStand< T >::SetJPosInterPts( const size_t& curr_iter, size_t max_iter, int leg, const Vec3< T >& ini, const Vec3< T >& fin ) {

    float a( 0.f );
    float b( 1.f );

    // if we're done interpolating
    if ( curr_iter <= max_iter ) {
        b = ( float )curr_iter / ( float )max_iter;
        a = 1.f - b;
    }
    // compute setpoints
    Vec3< T > inter_pos = a * ini + b * fin;

    // do control
    JointPdControl( leg, inter_pos, zero_vec3 );
}

template < typename T > void FSMStateRecoveryStand< T >::RightRollOver( const int& curr_iter ) {
    if ( curr_iter == 1 )
        std::cout << "Start rollover to RecoveryStand" << std::endl;

    // leg motor limit angle
    // 3: 44.47 -46.9    -201.1  116.8    29.5~142.8
    // 1: 43.57 - 47.0   -251.1~ 72.8     28.5~143.7
    // leg motor limit angle for mini of new motor structrue
    // 1: -0.68~0.68(38.9)    -2.792（-159.9）~1.326(75.9)    0.523(29.96)~2.53(144.9)
    // 3: -0.68~0.68(38.9)    -3.141(-179.9)~0.977(55.9)    0.523(29.96)~2.53(144.9)
    rolling_jpos[ 0 ] << -45 / 57.3, -100 / 57.3, 140 / 57.3;
    rolling_jpos[ 1 ] << 0.0, -110 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << -45 / 57.3, -100 / 57.3, 140 / 57.3;
    rolling_jpos[ 3 ] << 0.0, -110 / 57.3, 140 / 57.3;
    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        rolling_jpos[ 0 ] << 0.0, -140 / 57.3, 140 / 57.3;
        rolling_jpos[ 1 ] << -30 / 57.3, -140 / 57.3, 140 / 57.3;
        rolling_jpos[ 2 ] << 0.0, -177.5 / 57.3, 120 / 57.3;
        rolling_jpos[ 3 ] << -30 / 57.3, -177.5 / 57.3, 120 / 57.3;
    }
    if ( curr_iter > 0 && curr_iter <= 200 )
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 0, 200, i, initial_jpos[ i ], rolling_jpos[ i ] );
    if ( curr_iter == 200 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage2:
    rolling_jpos[ 0 ] << -45 / 57.3, -100 / 57.3, 140 / 57.3;
    rolling_jpos[ 1 ] << 30 / 57.3, -180 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << -45 / 57.3, -100 / 57.3, 140 / 57.3;
    rolling_jpos[ 3 ] << 30 / 57.3, -180 / 57.3, 140 / 57.3;
    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        rolling_jpos[ 0 ] << 0.0, -157 / 57.3, 70 / 57.3;
        rolling_jpos[ 1 ] << 0.0, -150 / 57.3, 140 / 57.3;
        rolling_jpos[ 2 ] << 0.0, -177.5 / 57.3, 100 / 57.3;
        rolling_jpos[ 3 ] << 0.0, -177.5 / 57.3, 100 / 57.3;
    }
    if ( curr_iter > 200 && curr_iter <= 500 )
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 200, 300, i, initial_jpos[ i ], rolling_jpos[ i ] );
    if ( curr_iter == 500 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage3:
    rolling_jpos[ 0 ] << -45 / 57.3, -160 / 57.3, 120 / 57.3;
    rolling_jpos[ 1 ] << 30 / 57.3, -180 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << -45 / 57.3, -160 / 57.3, 120 / 57.3;
    rolling_jpos[ 3 ] << 30 / 57.3, -180 / 57.3, 140 / 57.3;
    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        rolling_jpos[ 0 ] << 0.0, -157 / 57.3, 70 / 57.3;
        rolling_jpos[ 1 ] << 0.0, -150 / 57.3, 140 / 57.3;
        rolling_jpos[ 2 ] << 0.0, -177.5 / 57.3, 100 / 57.3;
        rolling_jpos[ 3 ] << 0.0, -177.5 / 57.3, 100 / 57.3;
    }
    if ( curr_iter > 500 && curr_iter <= 800 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 500, 300, i, initial_jpos[ i ], rolling_jpos[ i ] );
    }
    if ( curr_iter == 800 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage4:
    rolling_jpos[ 0 ] << -45 / 57.3, -160 / 57.3, 120 / 57.3;
    rolling_jpos[ 1 ] << -10 / 57.3, -90 / 57.3, 120 / 57.3;
    rolling_jpos[ 2 ] << -45 / 57.3, -160 / 57.3, 120 / 57.3;
    rolling_jpos[ 3 ] << -10 / 57.3, -90 / 57.3, 120 / 57.3;
    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        rolling_jpos[ 0 ] << -38 / 57.3, -157 / 57.3, 32 / 57.3;
        rolling_jpos[ 1 ] << 0 / 57.3, -150 / 57.3, 140 / 57.3;
        rolling_jpos[ 2 ] << -38 / 57.3, -177.5 / 57.3, 32 / 57.3;
        rolling_jpos[ 3 ] << 0 / 57.3, -177.5 / 57.3, 100 / 57.3;
    }
    if ( curr_iter > 800 && curr_iter <= 900 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 800, this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ? 50 : 100, i, initial_jpos[ i ], rolling_jpos[ i ] );
    }
    if ( curr_iter == 900 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage5:
    rolling_jpos[ 0 ] << 42 / 57.3, -150 / 57.3, 140 / 57.3;
    rolling_jpos[ 1 ] << -10 / 57.3, -90 / 57.3, 120 / 57.3;
    rolling_jpos[ 2 ] << 42 / 57.3, -150 / 57.3, 140 / 57.3;
    rolling_jpos[ 3 ] << -10 / 57.3, -90 / 57.3, 120 / 57.3;
    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        rolling_jpos[ 0 ] << -38 / 57.3, -157 / 57.3, 32 / 57.3;
        rolling_jpos[ 1 ] << 0 / 57.3, -150 / 57.3, 140 / 57.3;
        rolling_jpos[ 2 ] << -38 / 57.3, -177.5 / 57.3, 32 / 57.3;
        rolling_jpos[ 3 ] << 0 / 57.3, -177.5 / 57.3, 100 / 57.3;
    }
    if ( curr_iter > 900 && curr_iter <= 1100 ) {
        for ( size_t i( 0 ); i < 4; ++i ) {
            SetJPosInterPts( curr_iter - 900, 200, i, initial_jpos[ i ], rolling_jpos[ i ] );
            if ( ( i == 1 || i == 3 ) && this->data_->quadruped->robot_type_ != RobotType::CYBERDOG2 ) {
                this->data_->leg_controller->commands_[ i ].kp_joint << 50, 0, 0, 0, 50, 0, 0, 0, 60;
                ;
                this->data_->leg_controller->commands_[ i ].kd_joint << 4, 0, 0, 0, 4, 0, 0, 0, 4;
            }
        }
    }
    if ( curr_iter == 1100 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage6:
    rolling_jpos[ 0 ] << -0 / 57.3, -80 / 57.3, 140 / 57.3;
    rolling_jpos[ 1 ] << 0 / 57.3, -80 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << -0 / 57.3, -80 / 57.3, 140 / 57.3;
    rolling_jpos[ 3 ] << 0 / 57.3, -80 / 57.3, 140 / 57.3;
    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        rolling_jpos[ 0 ] << 0.0, -157 / 57.3, 140 / 57.3;
        rolling_jpos[ 1 ] << 0.0, -150 / 57.3, 140 / 57.3;
        rolling_jpos[ 2 ] << 0.0, -177.5 / 57.3, 140 / 57.3;
        rolling_jpos[ 3 ] << 0.0, -177.5 / 57.3, 140 / 57.3;
    }
    if ( curr_iter > 1100 && curr_iter <= 1500 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 1100, 200, i, initial_jpos[ i ], rolling_jpos[ i ] );
    }
    if ( curr_iter == 1500 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage7:
    if ( curr_iter >= 1500 ) {
        if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
            if ( SideLyingState() != 0 )
                flag_ = SideLying_;
            else
                flag_ = FoldLegs_;
        }
        else {
            flag_ = StandUp_;
        }

        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
        motion_start_iter_ = state_iter_ + 1;
    }
}

template < typename T > void FSMStateRecoveryStand< T >::LeftRollOver( const int& curr_iter ) {
    if ( curr_iter == 1 )
        std::cout << "Start rollover to RecoveryStand" << std::endl;

    // leg motor limit angle
    // 3: 44.47 -46.9    -201.1  116.8    29.5~142.8
    // 1: 43.57 - 47.0   -251.1~ 72.8     28.5~143.7
    // leg motor limit angle for mini of new motor structrue
    // 1: -0.68~0.68(38.9)    -2.792（-159.9）~1.326(75.9)    0.523(29.96)~2.53(144.9)
    // 3: -0.68~0.68(38.9)    -3.141(-179.9)~0.977(55.9)    0.523(29.96)~2.53(144.9)
    rolling_jpos[ 0 ] << -45 / 57.3, -100 / 57.3, 140 / 57.3;
    rolling_jpos[ 1 ] << 0.0, -110 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << -45 / 57.3, -100 / 57.3, 140 / 57.3;
    rolling_jpos[ 3 ] << 0.0, -110 / 57.3, 140 / 57.3;
    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        rolling_jpos[ 0 ] << 30 / 57.3, -140 / 57.3, 140 / 57.3;
        rolling_jpos[ 1 ] << 0.0, -140 / 57.3, 140 / 57.3;
        rolling_jpos[ 2 ] << 30 / 57.3, -177.5 / 57.3, 120 / 57.3;
        rolling_jpos[ 3 ] << 0.0, -177.5 / 57.3, 120 / 57.3;
    }
    if ( curr_iter > 0 && curr_iter <= 200 )
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 0, 200, i, initial_jpos[ i ], rolling_jpos[ i ] );
    if ( curr_iter == 200 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage2:
    rolling_jpos[ 0 ] << -45 / 57.3, -100 / 57.3, 140 / 57.3;
    rolling_jpos[ 1 ] << 30 / 57.3, -180 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << -45 / 57.3, -100 / 57.3, 140 / 57.3;
    rolling_jpos[ 3 ] << 30 / 57.3, -180 / 57.3, 140 / 57.3;
    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        rolling_jpos[ 1 ] << 0.0, -157 / 57.3, 70 / 57.3;
        rolling_jpos[ 0 ] << 0.0, -150 / 57.3, 140 / 57.3;
        rolling_jpos[ 2 ] << 0.0, -177.5 / 57.3, 100 / 57.3;
        rolling_jpos[ 3 ] << 0.0, -177.5 / 57.3, 100 / 57.3;
    }
    if ( curr_iter > 200 && curr_iter <= 500 )
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 200, 300, i, initial_jpos[ i ], rolling_jpos[ i ] );
    if ( curr_iter == 500 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage3:
    rolling_jpos[ 0 ] << -45 / 57.3, -160 / 57.3, 120 / 57.3;
    rolling_jpos[ 1 ] << 30 / 57.3, -180 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << -45 / 57.3, -160 / 57.3, 120 / 57.3;
    rolling_jpos[ 3 ] << 30 / 57.3, -180 / 57.3, 140 / 57.3;
    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        rolling_jpos[ 1 ] << 0.0, -157 / 57.3, 70 / 57.3;
        rolling_jpos[ 0 ] << 0.0, -150 / 57.3, 140 / 57.3;
        rolling_jpos[ 2 ] << 0.0, -177.5 / 57.3, 100 / 57.3;
        rolling_jpos[ 3 ] << 0.0, -177.5 / 57.3, 100 / 57.3;
    }
    if ( curr_iter > 500 && curr_iter <= 800 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 500, 300, i, initial_jpos[ i ], rolling_jpos[ i ] );
    }
    if ( curr_iter == 800 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage4:
    rolling_jpos[ 0 ] << -45 / 57.3, -160 / 57.3, 120 / 57.3;
    rolling_jpos[ 1 ] << -10 / 57.3, -90 / 57.3, 120 / 57.3;
    rolling_jpos[ 2 ] << -45 / 57.3, -160 / 57.3, 120 / 57.3;
    rolling_jpos[ 3 ] << -10 / 57.3, -90 / 57.3, 120 / 57.3;
    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        rolling_jpos[ 1 ] << 38 / 57.3, -157 / 57.3, 32 / 57.3;
        rolling_jpos[ 0 ] << 0 / 57.3, -150 / 57.3, 140 / 57.3;
        rolling_jpos[ 3 ] << 38 / 57.3, -177.5 / 57.3, 32 / 57.3;
        rolling_jpos[ 2 ] << 0 / 57.3, -177.5 / 57.3, 100 / 57.3;
    }
    if ( curr_iter > 800 && curr_iter <= 900 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 800, this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ? 50 : 100, i, initial_jpos[ i ], rolling_jpos[ i ] );
    }
    if ( curr_iter == 900 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage5:
    rolling_jpos[ 0 ] << 42 / 57.3, -150 / 57.3, 140 / 57.3;
    rolling_jpos[ 1 ] << -10 / 57.3, -90 / 57.3, 120 / 57.3;
    rolling_jpos[ 2 ] << 42 / 57.3, -150 / 57.3, 140 / 57.3;
    rolling_jpos[ 3 ] << -10 / 57.3, -90 / 57.3, 120 / 57.3;
    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        rolling_jpos[ 1 ] << 38 / 57.3, -157 / 57.3, 32 / 57.3;
        rolling_jpos[ 0 ] << 0 / 57.3, -150 / 57.3, 140 / 57.3;
        rolling_jpos[ 3 ] << 38 / 57.3, -177.5 / 57.3, 32 / 57.3;
        rolling_jpos[ 2 ] << 0 / 57.3, -177.5 / 57.3, 100 / 57.3;
    }
    if ( curr_iter > 900 && curr_iter <= 1100 ) {
        for ( size_t i( 0 ); i < 4; ++i ) {
            SetJPosInterPts( curr_iter - 900, 200, i, initial_jpos[ i ], rolling_jpos[ i ] );
            if ( ( i == 1 || i == 3 ) && this->data_->quadruped->robot_type_ != RobotType::CYBERDOG2 ) {
                this->data_->leg_controller->commands_[ i ].kp_joint << 50, 0, 0, 0, 50, 0, 0, 0, 60;
                ;
                this->data_->leg_controller->commands_[ i ].kd_joint << 4, 0, 0, 0, 4, 0, 0, 0, 4;
            }
        }
    }
    if ( curr_iter == 1100 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage6:
    rolling_jpos[ 0 ] << -0 / 57.3, -80 / 57.3, 140 / 57.3;
    rolling_jpos[ 1 ] << 0 / 57.3, -80 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << -0 / 57.3, -80 / 57.3, 140 / 57.3;
    rolling_jpos[ 3 ] << 0 / 57.3, -80 / 57.3, 140 / 57.3;
    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        rolling_jpos[ 1 ] << 0.0, -157 / 57.3, 140 / 57.3;
        rolling_jpos[ 0 ] << 0.0, -150 / 57.3, 140 / 57.3;
        rolling_jpos[ 3 ] << 0.0, -177.5 / 57.3, 140 / 57.3;
        rolling_jpos[ 2 ] << 0.0, -177.5 / 57.3, 140 / 57.3;
    }
    if ( curr_iter > 1100 && curr_iter <= 1500 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 1100, 200, i, initial_jpos[ i ], rolling_jpos[ i ] );
    }
    if ( curr_iter == 1500 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage7:
    if ( curr_iter >= 1500 ) {
        if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
            if ( SideLyingState() != 0 )
                flag_ = SideLying_;
            else
                flag_ = FoldLegs_;
        }
        else {
            flag_ = StandUp_;
        }

        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
        motion_start_iter_ = state_iter_ + 1;
    }
}

template < typename T > void FSMStateRecoveryStand< T >::StandUp( const int& curr_iter ) {
#ifdef USE_ABSOLUTE_ODOM_FOR_ALL
    T body_height = this->data_->state_estimator->GetResult().height;
#else
    T body_height = this->data_->state_estimator->GetResult().position[ 2 ];
#endif
    bool something_wrong( false );

    if ( UpsideDown() || ( body_height < 0.1 ) || this->data_->state_estimator->GetResult().world2body_rotation_matrix( 0, 2 ) < -0.5 ) {
        something_wrong = true;
    }
    if ( this->data_->command->gait_id == 0 )
        LieDown_flag_ = 0;

    if ( ( curr_iter > floor( standup_ramp_iter_ * 0.7 ) ) && something_wrong && ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG ) ) {
        // If body height is too low because of some reason
        // even after the stand up motion is almost over
        // (Can happen when E-Stop is engaged in the middle of Other state)
        // Cyberdog2 don't redo foldlegs for hand safety
        for ( size_t i( 0 ); i < 4; ++i ) {
            initial_jpos[ i ] = this->data_->leg_controller->datas_[ i ].q;
        }
        flag_              = FoldLegs_;
        motion_start_iter_ = state_iter_ + 1;

        printf( "[RecoveryStand] Warning: body height is still too low (%f) or UpsideDown (%d); Folding legs \n", body_height, UpsideDown() );
    }
    else {
        if ( curr_iter <= 1 ) {
            if ( this->data_->state_estimator->GetResult().rpy[ 1 ] < -0.3 )
                stand_waitstable_ = 600;
            else
                stand_waitstable_ = 50;
        }

        for ( size_t leg( 0 ); leg < 4; ++leg ) {
            SetJPosInterPts( curr_iter, standup_ramp_iter_, leg, initial_jpos[ leg ], stand_jpos[ leg ] );
        }

        // Block the progress bar feedback of a short standing time before the single lieside request
        if ( this->data_->command->gait_id == 0 && this->data_->command->duration != 4 ) {
            first_stand_ = true;
            liedown_ok_  = false;
        }
        else
            first_stand_ = false;

        if ( curr_iter > standup_ramp_iter_ + stand_waitstable_ ) {
            if ( !this->ready_for_switch_ )
                printf( "[RecoveryStand] Stand finish! first_stand_=%d liedown_ok_=%d gait_id=%d LieDown_flag_=%d\n", ( int )first_stand_, ( int )liedown_ok_, this->data_->command->gait_id,
                        LieDown_flag_ );
            this->ready_for_switch_ = true;
            for ( size_t i( 0 ); i < 4; ++i ) {
                initial_jpos[ i ] = this->data_->leg_controller->datas_[ i ].q;
            }
            if ( liedown_ok_ || first_stand_ ) {
                this->motion_progress_bar_ = 99;
                if ( curr_iter > ( standup_ramp_iter_ + stand_waitstable_ + 5 ) )
                    this->motion_progress_bar_ = 100;
            }
        }

        if ( this->ready_for_switch_ == true && LieDown_flag_ == 0 ) {
            int _gait_id = this->data_->command->gait_id;
            if ( _gait_id == 17 || _gait_id == 18 ) {
                if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 )
                    LieDown_flag_ = 2;
                else
                    LieDown_flag_ = 1;
                if ( _gait_id == 18 )
                    which_side_ = 'r';
                else
                    which_side_ = 'l';
            }
            if ( LieDown_flag_ == 1 ) {
                this->ready_for_switch_ = false;
                for ( size_t i( 0 ); i < 4; ++i )
                    initial_jpos[ i ] = this->data_->leg_controller->commands_[ i ].q_des;
                motion_start_iter_         = state_iter_ + 1;
                flag_                      = LieDown_;
                this->motion_progress_bar_ = 0;
            }
            else if ( LieDown_flag_ == 2 ) {
                this->ready_for_switch_ = false;
                for ( size_t i( 0 ); i < 4; ++i )
                    initial_jpos[ i ] = this->data_->leg_controller->commands_[ i ].q_des;
                motion_start_iter_         = state_iter_ + 1;
                flag_                      = LieSide_;
                this->motion_progress_bar_ = 0;
            }
        }

        if ( this->ready_for_switch_ && BeLookUpOrDown() ) {
            for ( size_t i( 0 ); i < 4; ++i )
                initial_jpos[ i ] = this->data_->leg_controller->datas_[ i ].q;
            motion_start_iter_         = state_iter_ + 1;
            flag_                      = LookUpOrDown_;
            this->motion_progress_bar_ = 0;
            this->ready_for_switch_    = false;
        }
    }
    Vec4< T > se_contactState( 0.5, 0.5, 0.5, 0.5 );
    this->data_->state_estimator->SetContactPhase( se_contactState );
}

template < typename T > void FSMStateRecoveryStand< T >::LookUpOrDown( const int& curr_iter ) {
    if ( curr_iter <= 0 )
        std::cout << "[RecoveryStand] Damping Fold Legs" << std::endl;
    Vec3< T > joint_kd;
    if ( curr_iter < damp_ramp_iter_ ) {
        for ( int leg( 0 ); leg < 4; leg++ ) {
            joint_kd << 10, 4, 4;
            JointPdControl( leg, joint_kd );
        }
    }
    else if ( curr_iter < damp_ramp_iter_ + damp_settle_iter_ ) {
        for ( int leg( 0 ); leg < 4; leg++ ) {
            joint_kd << 2, 0.5, 0.5;
            JointPdControl( leg, joint_kd );
        }
    }
    else {
        if ( OnRightLegConfig() )
            flag_ = FoldLegs_;
        else
            flag_ = PrepareForFoldLegs_;
        for ( int leg( 0 ); leg < 4; leg++ )
            initial_jpos[ leg ] = this->data_->leg_controller->datas_[ leg ].q;
        motion_start_iter_ = state_iter_ + 1;
    }
}

template < typename T > void FSMStateRecoveryStand< T >::PrepareForFoldLegs( const int& curr_iter ) {
    if ( curr_iter <= 0 )
        std::cout << "[RecoveryStand] Prepare For Fold Legs" << std::endl;
    int       _iner_iter       = 0;
    int       _first_step_iter = prepare_ramp_iter_ / 2;
    Vec3< T > right_fold_pos[ 4 ];
    if ( curr_iter < _first_step_iter ) {
        _iner_iter = curr_iter;
        // first step
        for ( int leg( 0 ); leg < 4; leg++ ) {
            if ( _iner_iter <= 0 )
                initial_jpos[ leg ] = this->data_->leg_controller->datas_[ leg ].q;
            if ( !RightLegConfigFlag[ leg ] ) {
                if ( leg == 0 || leg == 2 )
                    right_fold_pos[ leg ] << this->data_->leg_controller->q_abad_lowerbound_ + 0.07, -2.6, 2.45;
                else
                    right_fold_pos[ leg ] << - ( this->data_->leg_controller->q_abad_lowerbound_ + 0.07 ), -2.6, 2.45;
            }
            else {
                right_fold_pos[ leg ] = initial_jpos[ leg ];
            }
            SetJPosInterPts( _iner_iter, _first_step_iter, leg, initial_jpos[ leg ], right_fold_pos[ leg ] );
        }
    }
    else {
        // second step
        _iner_iter = curr_iter - _first_step_iter;

        for ( int leg( 0 ); leg < 4; leg++ ) {
            if ( _iner_iter <= 0 )
                initial_jpos[ leg ] = this->data_->leg_controller->datas_[ leg ].q;
            if ( !RightLegConfigFlag[ leg ] ) {
                right_fold_pos[ leg ]      = initial_jpos[ leg ];
                right_fold_pos[ leg ][ 1 ] = -1.3;
            }
            else {
                right_fold_pos[ leg ] = initial_jpos[ leg ];
            }
            SetJPosInterPts( _iner_iter, prepare_ramp_iter_ - _first_step_iter, leg, initial_jpos[ leg ], right_fold_pos[ leg ] );
        }
    }
    if ( curr_iter > prepare_ramp_iter_ + prepare_settle_iter_ ) {
        if ( OnRightLegConfig() )
            flag_ = FoldLegs_;
        else
            flag_ = PrepareForFoldLegs_;
        for ( int leg( 0 ); leg < 4; leg++ )
            initial_jpos[ leg ] = this->data_->leg_controller->datas_[ leg ].q;
        motion_start_iter_ = state_iter_ + 1;
    }
}

template < typename T > void FSMStateRecoveryStand< T >::FoldLegs( const int& curr_iter ) {
    float     diff = 0, max_diff = 0;
    Vec3< T > fold_jpos_tmp[ 4 ], fold_jpos_tmp1[ 4 ];
    if ( this->data_->command->gait_id == 0 )
        LieDown_flag_ = 0;

    if ( curr_iter <= 0 ) {
        std::cout << "[RecoveryStand] Fold Legs" << std::endl;
        for ( size_t i( 0 ); i < 4; ++i )
            for ( size_t j( 0 ); j < 3; ++j ) {
                diff = fabs( fold_jpos[ i ]( j ) - initial_jpos[ i ]( j ) );
                if ( diff > max_diff )
                    max_diff = diff;
            }
        if ( max_diff < 30 / 57.3 )
            fold_ramp_iter_ = 501;
        else if ( max_diff < 100 / 57.3 )
            fold_ramp_iter_ = 801;
        else if ( max_diff < 180 / 57.3 )
            fold_ramp_iter_ = 999;
    }

    for ( int i( 0 ); i < 4; ++i ) {
        fold_jpos_tmp[ i ]( 0 ) = initial_jpos[ i ]( 0 );
        fold_jpos_tmp[ i ]( 1 ) = initial_jpos[ i ]( 1 ) + ( fold_jpos[ i ]( 1 ) + Deg2Rad( 30. ) - initial_jpos[ i ]( 1 ) ) * 2.0 / 3.0;
        fold_jpos_tmp[ i ]( 2 ) = initial_jpos[ i ]( 2 ) + ( fold_jpos[ i ]( 2 ) - initial_jpos[ i ]( 2 ) ) * 2.0 / 3.0;

        fold_jpos_tmp1[ i ]( 0 ) = 0;
        fold_jpos_tmp1[ i ]( 1 ) = initial_jpos[ i ]( 1 ) + ( fold_jpos[ i ]( 1 ) + Deg2Rad( 30. ) - initial_jpos[ i ]( 1 ) ) * 2.0 / 3.0;
        fold_jpos_tmp1[ i ]( 2 ) = initial_jpos[ i ]( 2 ) + ( fold_jpos[ i ]( 2 ) - initial_jpos[ i ]( 2 ) ) * 2.0 / 3.0;

        if ( curr_iter < fold_ramp_iter_ / 3 ) {
            SetJPosInterPts( fabs( curr_iter ), fold_ramp_iter_ / 3, i, initial_jpos[ i ], fold_jpos_tmp[ i ] );
        }
        else if ( curr_iter < 2 * fold_ramp_iter_ / 3 ) {
            SetJPosInterPts( fabs( curr_iter ) - fold_ramp_iter_ / 3, fold_ramp_iter_ / 3, i, fold_jpos_tmp[ i ], fold_jpos_tmp1[ i ] );
        }
        else {
            SetJPosInterPts( curr_iter - 2 * fold_ramp_iter_ / 3, fold_ramp_iter_ / 3, i, fold_jpos_tmp1[ i ], fold_jpos[ i ] );
        }
    }
    if ( curr_iter >= fold_ramp_iter_ + fold_settle_iter_ ) {
        if ( UpsideDown() ) {
            if ( this->data_->state_estimator->GetResult().rpy[ 0 ] > M_PI * 4 / 6 && this->data_->state_estimator->GetResult().rpy[ 0 ] < M_PI )
                flag_ = LeftRollOver_;
            else if ( this->data_->state_estimator->GetResult().rpy[ 0 ] > -M_PI && this->data_->state_estimator->GetResult().rpy[ 0 ] < -M_PI * 4 / 6 )
                flag_ = RightRollOver_;
            else
                flag_ = LeftRollOver_;
            for ( size_t i( 0 ); i < 4; ++i )
                initial_jpos[ i ] = fold_jpos[ i ];
        }
        else if ( SideLyingState() != 0 ) {
            flag_ = SideLying_;
            for ( size_t i( 0 ); i < 4; ++i )
                initial_jpos[ i ] = fold_jpos[ i ];
        }
        else {
            flag_                      = StandUp_;
            this->motion_progress_bar_ = 80;
            for ( size_t i( 0 ); i < 4; ++i )
                initial_jpos[ i ] = fold_jpos[ i ];
        }
        motion_start_iter_ = state_iter_ + 1;
    }
}

template < typename T > void FSMStateRecoveryStand< T >::SideLying( const int& curr_iter ) {
    if ( curr_iter == 0 )
        std::cout << "[RecoveryStand] SideLying recovery " << std::endl;
    if ( SideLyingState() == 1 ) {
        // fold legs
        if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
            side_jpose[ 0 ] << 0.52, -2.5, 2.4;
            side_jpose[ 2 ] << 0.52, -3.05, 2.4;
        }
        else {
            side_jpose[ 0 ] << 0.72, -2.5, 2.4;
            side_jpose[ 2 ] << 0.72, -2.5, 2.4;
        }
        // no fold legs
        side_jpose[ 1 ] << 0.0, -1.4, 2.4;
        side_jpose[ 3 ] << 0.0, -1.4, 2.4;
    }
    else if ( SideLyingState() == -1 ) {
        // no fold legs
        side_jpose[ 0 ] << 0.0, -1.4, 2.4;
        side_jpose[ 2 ] << 0.0, -1.4, 2.4;
        // fold legs
        if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
            side_jpose[ 1 ] << -0.52, -2.5, 2.4;
            side_jpose[ 3 ] << -0.52, -3.05, 2.4;
        }
        else {
            side_jpose[ 1 ] << -0.72, -2.5, 2.4;
            side_jpose[ 3 ] << -0.72, -2.5, 2.4;
        }
    }

    for ( size_t i( 0 ); i < 4; ++i ) {
        SetJPosInterPts( curr_iter, side_ramp_iter_, i, initial_jpos[ i ], side_jpose[ i ] );
    }

    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        if ( curr_iter >= side_ramp_iter_ && curr_iter <= side_ramp_iter_ + 70 ) {
            this->data_->leg_controller->commands_[ 0 ].tau_feed_forward[ 0 ] = side_jpose[ 0 ][ 0 ] > 0.5 ? 5 : 0;
            this->data_->leg_controller->commands_[ 2 ].tau_feed_forward[ 0 ] = side_jpose[ 2 ][ 0 ] > 0.5 ? 5 : 0;
            this->data_->leg_controller->commands_[ 1 ].tau_feed_forward[ 0 ] = side_jpose[ 1 ][ 0 ] < -0.5 ? -5 : 0;
            this->data_->leg_controller->commands_[ 3 ].tau_feed_forward[ 0 ] = side_jpose[ 3 ][ 0 ] < -0.5 ? -5 : 0;
        }
    }

    if ( curr_iter >= side_ramp_iter_ + side_settle_iter_ ) {
        if ( UpsideDown() ) {
            if ( this->data_->state_estimator->GetResult().rpy[ 0 ] > M_PI * 4 / 6 && this->data_->state_estimator->GetResult().rpy[ 0 ] < M_PI )
                flag_ = LeftRollOver_;
            else if ( this->data_->state_estimator->GetResult().rpy[ 0 ] > -M_PI && this->data_->state_estimator->GetResult().rpy[ 0 ] < -M_PI * 4 / 6 )
                flag_ = RightRollOver_;
            else
                flag_ = LeftRollOver_;
            for ( size_t i( 0 ); i < 4; ++i )
                initial_jpos[ i ] = side_jpose[ i ];
        }
        else if ( SideLyingState() == 0 ) {
            flag_ = FoldLegs_;
            for ( size_t i( 0 ); i < 4; ++i )
                initial_jpos[ i ] = side_jpose[ i ];
        }
        motion_start_iter_ = state_iter_ + 1;
    }
}
template < typename T > void FSMStateRecoveryStand< T >::LieDown( const int& curr_iter ) {
    if ( curr_iter == 0 )
        std::cout << "LieDown" << std::endl;
    // leg motor limit angle
    // 3: 44.47 -46.9    -201.1  116.8    29.5~142.8
    // 1: 43.57 - 47.0   -251.1~ 72.8     28.5~143.7

    rolling_jpos[ 0 ] << -0.0f, -1.4f, 2.4f;
    rolling_jpos[ 1 ] << 0.0f, -1.4f, 2.4f;
    rolling_jpos[ 2 ] << -0.0f, -1.4f, 2.4f;
    rolling_jpos[ 3 ] << 0.0f, -1.4f, 2.4f;
    if ( curr_iter > 0 && curr_iter <= 200 )
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 0, 200, i, initial_jpos[ i ], rolling_jpos[ i ] );
    if ( curr_iter == 200 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
        this->motion_progress_bar_ = 20;
    }

    rolling_jpos[ 0 ] << 0.0, -45 / 57.3, 90 / 57.3;
    rolling_jpos[ 1 ] << -40 / 57.3, -80 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << 0.0, -45 / 57.3, 90 / 57.3;
    rolling_jpos[ 3 ] << -40 / 57.3, -80 / 57.3, 140 / 57.3;
    if ( curr_iter > 200 && curr_iter <= 500 )
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 200, 300, i, initial_jpos[ i ], rolling_jpos[ i ] );
    if ( curr_iter == 500 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
        this->motion_progress_bar_ = 40;
    }

    rolling_jpos[ 0 ] << 0.0, 70 / 57.3, 125 / 57.3;
    rolling_jpos[ 1 ] << 45 / 57.3, -80 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << 0.0, 90 / 57.3, 135 / 57.3;
    rolling_jpos[ 3 ] << 45 / 57.3, -80 / 57.3, 140 / 57.3;
    if ( curr_iter > 500 && curr_iter <= 800 ) {
        for ( size_t i( 0 ); i < 4; ++i ) {
            SetJPosInterPts( curr_iter - 500, 100, i, initial_jpos[ i ], rolling_jpos[ i ] );
            this->data_->leg_controller->commands_[ i ].kp_joint << 50, 0, 0, 0, 50, 0, 0, 0, 60;
            ;
            this->data_->leg_controller->commands_[ i ].kd_joint << 4, 0, 0, 0, 4, 0, 0, 0, 4;
        }
    }
    if ( curr_iter == 800 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
        this->motion_progress_bar_ = 60;
    }

    rolling_jpos[ 0 ] << -0.0f, -1.4f, 2.4f;
    rolling_jpos[ 1 ] << 0.0f, -1.4f, 2.4f;
    rolling_jpos[ 2 ] << -0.0f, -1.4f, 2.4f;
    rolling_jpos[ 3 ] << 0.0f, -1.4f, 2.4f;
    if ( curr_iter > 800 && curr_iter <= 1000 + 100 )
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 800, 200, i, initial_jpos[ i ], rolling_jpos[ i ] );
    if ( curr_iter == 1100 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
        this->motion_progress_bar_ = 80;
    }

    if ( curr_iter > 1100 && curr_iter <= 1900 + 100 ) {
        T t = ( curr_iter - 1100 ) / 400.0;
        if ( t > 2 )
            t = 2;
        for ( size_t i( 0 ); i < 4; ++i ) {
            int sig = ( i % 2 ) * 2 - 1;
            foot_pos[ i ] << 0.15 * sin( t * 2 * M_PI ), 0.1 * sig, -0.25 + 0.1 * cos( t * 2 * M_PI );
            if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 )
                foot_pos[ i ] *= 0.7;
            rolling_jpos[ i ] = InverseKinematic< T >( *this->data_->quadruped, i, foot_pos[ i ] );
            JointPdControl( i, rolling_jpos[ i ], zero_vec3 );
        }
    }
    if ( curr_iter == 2000 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
        this->motion_progress_bar_ = 100;
    }
    if ( curr_iter > 2000 ) {
        if ( this->data_->state_estimator->GetResult().rpy[ 0 ] > M_PI * 4 / 6 && this->data_->state_estimator->GetResult().rpy[ 0 ] < M_PI )
            flag_ = LeftRollOver_;
        else if ( this->data_->state_estimator->GetResult().rpy[ 0 ] > -M_PI && this->data_->state_estimator->GetResult().rpy[ 0 ] < -M_PI * 4 / 6 )
            flag_ = RightRollOver_;
        else
            flag_ = LeftRollOver_;
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
        motion_start_iter_ = state_iter_ + 1;
    }
    Vec4< T > se_contactState( 0.5, 0.5, 0.5, 0.5 );
    this->data_->state_estimator->SetContactPhase( se_contactState );
}

template < typename T > void FSMStateRecoveryStand< T >::SwitchSIde() {
    Vec3< T > tmp_jpos;
    if ( which_side_ == 'r' ) {
        tmp_jpos          = rolling_jpos[ 0 ];
        rolling_jpos[ 0 ] = rolling_jpos[ 1 ];
        rolling_jpos[ 1 ] = tmp_jpos;
        tmp_jpos          = rolling_jpos[ 2 ];
        rolling_jpos[ 2 ] = rolling_jpos[ 3 ];
        rolling_jpos[ 3 ] = tmp_jpos;
        for ( int s = 0; s < 4; s++ )
            rolling_jpos[ s ]( 0 ) = -rolling_jpos[ s ]( 0 );
    }
}

template < typename T > void FSMStateRecoveryStand< T >::LieSide( const int& curr_iter ) {
    if ( curr_iter == 0 )
        std::cout << "[RecoveryStand] LieSide to the " << ( which_side_ == 'r' ? "right" : "left" ) << " side" << std::endl;
    // leg motor limit angle
    // 3: 44.47 -46.9    -201.1  116.8    29.5~142.8
    // 1: 43.57 - 47.0   -251.1~ 72.8     28.5~143.7

    rolling_jpos[ 0 ] << -0.0f, -1.4f, 2.4f;
    rolling_jpos[ 1 ] << 0.0f, -1.4f, 2.4f;
    rolling_jpos[ 2 ] << -0.0f, -1.4f, 2.4f;
    rolling_jpos[ 3 ] << 0.0f, -1.4f, 2.4f;
    if ( curr_iter >= 0 && curr_iter <= 200 )
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 0, 200, i, initial_jpos[ i ], rolling_jpos[ i ] );
    if ( curr_iter == 200 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
        this->motion_progress_bar_ = 20;
    }

    rolling_jpos[ 0 ] << 0.0, -45 / 57.3, 90 / 57.3;
    rolling_jpos[ 1 ] << -40 / 57.3, -70 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << 0.0, -45 / 57.3, 90 / 57.3;
    rolling_jpos[ 3 ] << -40 / 57.3, -70 / 57.3, 140 / 57.3;
    SwitchSIde();
    if ( curr_iter > 200 && curr_iter <= 500 )
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 200, 300, i, initial_jpos[ i ], rolling_jpos[ i ] );
    if ( curr_iter == 500 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
        this->motion_progress_bar_ = 40;
    }

    rolling_jpos[ 0 ] << 0.0, -45 / 57.3, 90 / 57.3;
    rolling_jpos[ 1 ] << -25 / 57.3, -80 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << 0.0, -45 / 57.3, 90 / 57.3;
    rolling_jpos[ 3 ] << -25 / 57.3, -80 / 57.3, 140 / 57.3;
    SwitchSIde();
    if ( curr_iter > 500 && curr_iter <= 1000 ) {
        for ( size_t i( 0 ); i < 4; ++i ) {
            SetJPosInterPts( curr_iter - 500, 500, i, initial_jpos[ i ], rolling_jpos[ i ] );
            this->data_->leg_controller->commands_[ i ].kp_joint << 50, 0, 0, 0, 50, 0, 0, 0, 60;
            this->data_->leg_controller->commands_[ i ].kd_joint << 2, 0, 0, 0, 2, 0, 0, 0, 2;
        }
    }
    if ( curr_iter == 1000 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
        this->motion_progress_bar_ = 50;
    }

    rolling_jpos[ 0 ] << -0.0f, -1.4f, 2.4f;
    rolling_jpos[ 1 ] << -25 / 57.3, -1.4f, 2.4f;
    rolling_jpos[ 2 ] << -0.0f, -1.4f, 2.4f;
    rolling_jpos[ 3 ] << -25 / 57.3f, -1.4f, 2.4f;
    SwitchSIde();
    if ( curr_iter > 1000 && ( curr_iter <= 1200 + 100 ) )
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 1000, 200, i, initial_jpos[ i ], rolling_jpos[ i ] );
    if ( curr_iter == 1300 ) {
        for ( size_t i( 0 ); i < 4; ++i ) {
            initial_jpos[ i ]     = rolling_jpos[ i ];
            initial_foot_pos[ i ] = ForwardKinematic( *this->data_->quadruped, i, rolling_jpos[ i ] );
        }
        this->motion_progress_bar_ = 60;
    }

    if ( curr_iter > 1300 && ( curr_iter <= 2500 + 100 ) ) {
        T t = ( curr_iter - 1300 ) / 600.0;
        if ( t > 2 )
            t = 2;
        for ( size_t i( 0 ); i < 4; ++i ) {
            size_t side = ( which_side_ == 'r' ? 1 : 0 );
            if ( i == ( 0 + side ) || i == ( 2 + side ) ) {
                foot_pos[ i ]     = initial_foot_pos[ i ] + Vec3< T >( 0.1 * sin( t * 2 * M_PI ), 0, -0.06 * ( 1 + sin( t * 2 * M_PI - M_PI / 2.0 ) ) );
                rolling_jpos[ i ] = InverseKinematic< T >( *this->data_->quadruped, i, foot_pos[ i ] );
            }
            JointPdControl( i, rolling_jpos[ i ], zero_vec3 );
        }
        if ( curr_iter == 2600 ) {
            for ( size_t i( 0 ); i < 4; ++i ) {
                initial_jpos[ i ] = rolling_jpos[ i ];
            }
            this->motion_progress_bar_ = 70;
        }
    }

    rolling_jpos[ 0 ] << 0.0, -1.4, 2.4;
    rolling_jpos[ 1 ] << -0.52, -2.9, 2.4;
    rolling_jpos[ 2 ] << 0.0, -1.4, 2.4;
    rolling_jpos[ 3 ] << -0.52, -2.9, 2.4;
    SwitchSIde();
    if ( curr_iter > 2600 && ( curr_iter <= 3400 ) )
        for ( size_t i( 0 ); i < 4; ++i )
            SetJPosInterPts( curr_iter - 2600, 800, i, initial_jpos[ i ], rolling_jpos[ i ] );
    if ( curr_iter >= 3400 ) {
        for ( size_t i( 0 ); i < 4; ++i ) {
            initial_jpos[ i ] = rolling_jpos[ i ];
        }
        flag_                      = FoldLegs_;
        motion_start_iter_         = state_iter_ + 1;
        first_stand_               = false;
        liedown_ok_                = true;
        this->motion_progress_bar_ = 80;
    }

    Vec4< T > se_contactState( 0.5, 0.5, 0.5, 0.5 );
    this->data_->state_estimator->SetContactPhase( se_contactState );
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template < typename T > FsmStateName FSMStateRecoveryStand< T >::CheckTransition() {
    this->next_state_name_ = this->state_name_;
    auto& cmd              = this->data_->command;

    // Switch FSM control mode
    switch ( cmd->mode ) {
    case MotionMode::kRecoveryStand:
        break;

    case MotionMode::kPoseCtrl:
    case MotionMode::kPureDamper:
    case MotionMode::kQpStand:
    case MotionMode::kOff:  // normal c
    case MotionMode::kLocomotion:
    case MotionMode::kTwoLegStand:
    case MotionMode::kJump3d:
    case MotionMode::kForceJump:
    case MotionMode::kLifted:
    case MotionMode::kRlReset:
    case MotionMode::kRlRapid:
        this->next_state_name_ = ( FsmStateName )cmd->mode;
        break;

    default:
        if ( this->iter_printf_ >= this->iter_printf_reset_ ) {
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << ( int )MotionMode::kRecoveryStand << " to " << ( int )this->data_->command->mode << std::endl;
            this->iter_printf_ = 0;
        }
        this->iter_printf_++;
    }

    // Get the next state
    return this->next_state_name_;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template < typename T > TransitionData< T > FSMStateRecoveryStand< T >::Transition() {
    // Finish Transition
    switch ( this->next_state_name_ ) {
    case FsmStateName::kOff:  // normal
    case FsmStateName::kPureDamper:
    case FsmStateName::kRlReset:
    case FsmStateName::kRlRapid:
        this->transition_data_.done = true;
        break;
    case FsmStateName::kTwoLegStand:
    case FsmStateName::kJump3d:
    case FsmStateName::kForceJump:
    case FsmStateName::kLifted:
    case FsmStateName::kPoseCtrl:
    case FsmStateName::kQpStand:
    case FsmStateName::kLocomotion:
        Run();
        if ( this->ready_for_switch_ == true )
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
template < typename T > void FSMStateRecoveryStand< T >::OnExit() {
    // Nothing to clean up when exiting
}
/**
 * Leg Joint Control.
 */
template < typename T > void FSMStateRecoveryStand< T >::JointPdControl( int leg, Vec3< T > qDes, Vec3< T > qdDes ) {
    auto& leg_ctrl = this->data_->leg_controller;

    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        leg_ctrl->commands_[ leg ].kp_joint << 60, 0, 0, 0, 80, 0, 0, 0, 60;
        leg_ctrl->commands_[ leg ].kd_joint << 2.5, 0, 0, 0, 2, 0, 0, 0, 1.5;
    }
    else {
        leg_ctrl->commands_[ leg ].kp_joint << 100, 0, 0, 0, 100, 0, 0, 0, 120;
        leg_ctrl->commands_[ leg ].kd_joint << 2, 0, 0, 0, 2, 0, 0, 0, 2;
    }
    soft_limimt_ = 0;
    SoftLimitation( qDes( 0 ), leg_ctrl->q_abad_lowerbound_ + 0.05, leg_ctrl->q_abad_upperbound_ - 0.05 );
    if ( soft_limimt_ == 1 )
        leg_ctrl->commands_[ leg ].kp_joint( 0 ) *= 0.3;
    leg_ctrl->commands_[ leg ].q_des  = qDes;
    leg_ctrl->commands_[ leg ].qd_des = qdDes;
}
template < typename T > void FSMStateRecoveryStand< T >::SoftLimitation( T& value, T min, T max ) {
    if ( value < min ) {
        value        = min;
        soft_limimt_ = 1;
    }
    else if ( value > max ) {
        value        = max;
        soft_limimt_ = 1;
    }
}

template < typename T > void FSMStateRecoveryStand< T >::JointPdControl( int leg, Vec3< T > kd ) {

    if ( this->data_->quadruped->robot_type_ == RobotType::CYBERDOG2 ) {
        this->data_->leg_controller->commands_[ leg ].kp_joint << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        this->data_->leg_controller->commands_[ leg ].kd_joint << kd[ 0 ], 0, 0, 0, kd[ 1 ], 0, 0, 0, kd[ 2 ];
    }
    else {
        this->data_->leg_controller->commands_[ leg ].kp_joint << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        this->data_->leg_controller->commands_[ leg ].kd_joint << kd[ 0 ], 0, 0, 0, kd[ 1 ], 0, 0, 0, kd[ 2 ];
    }
    // this->data_->leg_controller->commands_[ leg ].q_des  = qDes;
    // this->data_->leg_controller->commands_[ leg ].qd_des = qdDes;
}

// template class FSMStateRecoveryStand<double>;
template class FSMStateRecoveryStand< float >;
