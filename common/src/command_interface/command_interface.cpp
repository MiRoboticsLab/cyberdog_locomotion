#include <iostream>

#include "command_interface/command_interface.hpp"
#include "control_flags.hpp"
#include "utilities/utilities.hpp"

CommandInterface::CommandInterface() {
    motion_id_      = 0;
    motion_trigger_ = 0;
    interface_iter_ = 0;
    life_count_     = -1;
    ZeroCmd( pose_empty_cmd_ );
    ZeroCmd( qp_empty_cmd_ );
    ZeroCmd( loco_empty_cmd_ );
    ZeroCmd( loco_stand_cmd_ );
    ZeroCmd( sit_down_trans_cmd_ );
    pose_empty_cmd_.mode             = MotionMode::kPoseCtrl;
    pose_empty_cmd_.gait_id          = 3;
    pose_empty_cmd_.contact          = 0x0F;
    pose_empty_cmd_.duration         = 200;
    sit_down_trans_cmd_.mode         = MotionMode::kPoseCtrl;
    sit_down_trans_cmd_.gait_id      = 5;
    sit_down_trans_cmd_.pos_des[ 2 ] = 0.225;
    sit_down_trans_cmd_.contact      = 0x0F;
    sit_down_trans_cmd_.duration     = 400;
    qp_empty_cmd_.mode               = MotionMode::kQpStand;
    qp_empty_cmd_.gait_id            = 1;
    qp_empty_cmd_.contact            = 0x0F;
    qp_empty_cmd_.duration           = 200;
    qp_empty_cmd_.step_height[ 0 ]   = 0.05;
    qp_empty_cmd_.step_height[ 1 ]   = 0.05;
    loco_empty_cmd_.mode             = MotionMode::kLocomotion;
    loco_empty_cmd_.gait_id          = 1;
    loco_empty_cmd_.contact          = 0x0F;
    loco_empty_cmd_.duration         = 200;
    loco_empty_cmd_.step_height[ 0 ] = 0.05;
    loco_empty_cmd_.step_height[ 1 ] = 0.05;
    loco_stand_cmd_.mode             = MotionMode::kLocomotion;
    loco_stand_cmd_.gait_id          = 31;
    loco_stand_cmd_.contact          = 0x0F;
    loco_stand_cmd_.duration         = 200;
    loco_stand_cmd_.step_height[ 0 ] = 0.05;
    loco_stand_cmd_.step_height[ 1 ] = 0.05;
    switch_tri_flag_                 = 0;
    MotorCtrl_mode_flag_             = false;
    motion_list_size_                = 0;
    motion_list_step_                = 0;
    user_gait_file_                  = "";
    user_gait_list_file_             = "";
    current_fsm_state_               = MotionMode::kOff;
    speed_offset_count_              = 0;
}
void CommandInterface::ProcessGamepadCommand( const GamepadCommand& gamepad_cmd ) {
    gamepad_cmd_ = gamepad_cmd;
    gamepad_timer_.StartTimer();
}

void CommandInterface::ProcessGamepadCommand( const gamepad_lcmt* gamepad_cmd ) {
    gamepad_cmd_.set( gamepad_cmd );
    gamepad_timer_.StartTimer();
}

void CommandInterface::ProcessRcCommand( const RcCommand* rc_cmd ) {
    memcpy( &rc_cmd_, rc_cmd, sizeof( RcCommand ) );
    rc_timer_.StartTimer();
}

void CommandInterface::ProcessRcUdpCommand( const RcCommand* rc_udp_cmd ) {
    if ( rc_timer_.GetElapsedSeconds() > CMD_TIMEOUT * 1.5 ) {
        memcpy( &rc_cmd_, rc_udp_cmd, sizeof( RcCommand ) );
    }
    rc_udp_timer_.StartTimer();
}

void CommandInterface::ProcessCyberdogLcmCommand( const motion_control_request_lcmt* lcm_cmd ) {
    memcpy( &cyberdog_lcm_cmd_, lcm_cmd, sizeof( motion_control_request_lcmt ) );
    cyberdog_lcm_timer_.StartTimer();
}

void CommandInterface::ProcessLcmCommand( const robot_control_cmd_lcmt* lcm_cmd ) {
    memcpy( &lcm_cmd_, lcm_cmd, sizeof( robot_control_cmd_lcmt ) );
    lcm_timer_.StartTimer();
}
void CommandInterface::ProcessLcmMotorCtrlCommand( const motor_ctrl_lcmt* ctrl_cmd ) {
    memcpy( &motor_ctrl_cmd_, ctrl_cmd, sizeof( motor_ctrl_lcmt ) );
    MotorCtrl_mode_flag_ = true;
    MotorCtrl_LCM_timer_.StartTimer();
}

int CommandInterface::ProcessUserGaitFile( const file_send_lcmt* user_gait_msg ) {
    std::cout << "[CommandInterface] Get user gait string " << user_gait_msg->data.substr( 0, 11 ) << " " << user_gait_msg->data.length() << " Bytes!" << std::endl;
    if ( user_gait_msg->data.substr( 0, 10 ) == "# Gait Def" ) {
        user_gait_file_ = user_gait_msg->data;
        return 0;
    }
    else if ( user_gait_msg->data.substr( 0, 13 ) == "# Gait Params" ) {
        user_gait_list_file_ = user_gait_msg->data;
        return 0;
    }
    return 1;
}

void CommandInterface::ZeroCmd( MotionControlCommand& cmd ) {
    int i          = 0;
    cmd.mode       = 0;
    cmd.gait_id    = 0;
    cmd.contact    = 0;
    cmd.cmd_source = 0;
    for ( i = 0; i < 3; i++ ) {
        cmd.vel_des[ i ]    = 0;
        cmd.rpy_des[ i ]    = 0;
        cmd.pos_des[ i ]    = 0;
        cmd.ctrl_point[ i ] = 0;
    }
    for ( i = 0; i < 6; i++ ) {
        cmd.acc_des[ i ]   = 0;
        cmd.foot_pose[ i ] = 0;
    }
    cmd.step_height[ 0 ] = 0.03;
    cmd.step_height[ 1 ] = 0.03;
    cmd.value            = 0;
    cmd.duration         = 0;
}

/**
 * @brief Called in RobotRunner every loop, to get command by priority
 *
 * @param use_rc if 0 use gamepad command, elseif 1 select rc_command and lcm_cmmand by priority
 * @param control_mode if use gamepad as input source, the control_mode will be used
 * @param gait_id gait id
 * @param robotType robot type, cyberdog or cyberdog2
 */
void CommandInterface::PrepareCmd( int use_rc, long int* control_mode, long int* gait_id, const RobotType& robotType ) {
    static std::map< int, std::string > to_str{
        { kGamepadCmd, "kGamepadCmd" }, { kRcCmd, "kRcCmd" }, { kCyberdogLcmCmd, "kCyberdogLcmCmd" }, { kCyberdog2LcmCmd, "kCyberdog2LcmCmd" }, { kMotorCmd, "kMotorCmd" },
    };

    int current_mode = 0;
    if ( !use_rc ) {
        if ( gamepad_timer_.GetElapsedSeconds() < CMD_TIMEOUT ) {
            current_mode = kGamepadCmd;
        }
        else if ( gamepad_timer_.GetElapsedSeconds() < 2 * CMD_TIMEOUT ) {
            *control_mode = MotionMode::kPureDamper;
            *gait_id      = 0;
            current_mode  = 0;
        }
    }
    else {  // rc & lcm
        if ( rc_timer_.GetElapsedSeconds() < CMD_TIMEOUT || rc_udp_timer_.GetElapsedSeconds() < CMD_TIMEOUT ) {
            current_mode = kRcCmd;
        }
        else if ( MotorCtrl_mode_flag_ ) {
            current_mode = kMotorCmd;
        }
        else if ( lcm_timer_.GetElapsedSeconds() < CMD_TIMEOUT ) {
            current_mode = kCyberdog2LcmCmd;
        }
        else if ( cyberdog_lcm_timer_.GetElapsedSeconds() < CMD_TIMEOUT ) {
            current_mode = kCyberdogLcmCmd;
        }
        else {
            // all source timeout, make the dog puredamper
            if ( command_.mode != MotionMode::kOff && current_fsm_state_ != static_cast< int >( MotionMode::kOff ) )
                *control_mode = MotionMode::kPureDamper;
            else
                *control_mode = MotionMode::kOff;
            current_mode = 0;
            *gait_id     = 0;
        }
    }

    if ( current_mode != last_mode_ ) {
        std::cout << "[CommandInterface::PrepareCmd] ";
        if ( current_mode == 0 )
            std::cout << "all command input TIMEOUT!" << std::endl;
        else
            std::cout << "switch from " << to_str[ last_mode_ ] << "to " << to_str[ current_mode ] << std::endl;
        last_mode_ = ( CmdSource )current_mode;
    }

    ZeroCmd( command_ );
    ZeroCmd( cmd_cur_ );
    switch ( current_mode ) {
    case kGamepadCmd:
        Gamepad2Cmd( control_mode, gait_id, robotType );
        break;
    case kRcCmd:
        Rc2Cmd( robotType );
        break;
    case kCyberdog2LcmCmd:
        Lcm2Cmd();
        break;
    case kCyberdogLcmCmd:
        CyberdogLcm2Cmd();
        break;
    case kMotorCmd:
        MotorLcm2Cmd();
        break;
    default:
        Default2Cmd( *control_mode, *gait_id );
        break;
    }

    // Analysis control command list
    if ( cmd_list_.size() > 0 ) {
        command_ = cmd_list_.front();
        // When robot enter OFF/PUREDAMPER unexpectly， empty the cmd list
        if ( cmd_queue_clear_flag_ == MotionMode::kOff || cmd_queue_clear_flag_ == MotionMode::kPureDamper ) {
            while ( !cmd_list_.empty() )
                cmd_list_.pop();
            std::cout << "[CommandInterface] Robot unresponse! Cmd queue set to mode=" << cmd_queue_clear_flag_ << std::endl;
            lcm_cmd_.mode         = cmd_queue_clear_flag_;
            lcm_cmd_.duration     = 0;
            lcm_cmd_.gait_id      = 0;
            command_.mode         = cmd_queue_clear_flag_;
            command_.duration     = 0;
            command_.gait_id      = 0;
            cmd_queue_clear_flag_ = -1;
            cmd_list_.push( command_ );
        }
        if ( interface_iter_ >= command_.duration && command_.duration != 0 ) {
            cmd_list_.pop();
            if ( ( command_.mode == MotionMode::kLocomotion && command_.gait_id == kUserGait ) || motion_trigger_ > 0 ) {
                if ( motion_list_step_ > 0 )
                    motion_list_step_--;
            }
            // Empty stack: insert empty task according to the previous task type (position control or force control)
            if ( cmd_list_.size() == 0 ) {
                if ( command_.cmd_source == kCyberdogLcmCmd || command_.cmd_source == kCyberdog2LcmCmd )
                    motion_trigger_ = 0;  // For RC cmd, only trigger once when switch to Motion mode
                if ( command_.mode == MotionMode::kQpStand ) {
                    command_ = qp_empty_cmd_;
                }
                else if ( command_.mode == MotionMode::kLocomotion && command_.gait_id <= 30 ) {
                    command_ = loco_empty_cmd_;
                }
                else if ( command_.mode == MotionMode::kLocomotion && command_.gait_id > 30 ) {
                    command_ = loco_stand_cmd_;
                }
                else {
                    command_ = pose_empty_cmd_;
                }
                cmd_list_.push( command_ );
            }
            else {
                command_ = cmd_list_.front();
            }
            interface_iter_ = 0;
        }
        if ( command_.mode == MotionMode::kMotion ) {
            if ( motion_trigger_ == 0 && command_.cmd_source != kCyberdogLcmCmd ) {
                while ( !cmd_list_.empty() )
                    cmd_list_.pop();
                // Add sit_down_trans_cmd_ after kSitDown
                if ( ( motion_id_ == MotionId::kSitDown || ( motion_id_ >= MotionId::kSitDownLeft && motion_id_ <= MotionId::kSitDownShake ) )
                     && ( command_.gait_id != MotionId::kSitDown && ( command_.gait_id < MotionId::kSitDownLeft || command_.gait_id > MotionId::kSitDownShake ) ) )
                    cmd_list_.push( sit_down_trans_cmd_ );
                if ( robotType == RobotType::CYBERDOG2 ) {
                    motion_id_ = command_.gait_id;  // For gait_id responce
                    switch ( command_.gait_id ) {
                    case MotionId::kMotionAllInOne:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_motion.toml" );
                        break;
                    case MotionId::kHiFiveLeft:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_hiFive_LFleg_posCtrl.toml" );
                        break;
                    case MotionId::kHiFiveRight:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_hiFive_RFleg_posCtrl.toml" );
                        break;
                    case MotionId::kSitDown:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_sitDown_posCtrl.toml" );
                        break;
                    case MotionId::kSwingHip:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_swing_hip_posCtrl.toml" );
                        break;
                    case MotionId::kSwingHead:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_swing_head_posCtrl.toml" );
                        break;
                    case MotionId::kStretchBody:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_bodystretch.toml" );
                        break;
                    case MotionId::kSitDownLeft:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_sitDown_posCtrl_left.toml" );
                        break;
                    case MotionId::kSitDownRight:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_sitDown_posCtrl_right.toml" );
                        break;
                    case MotionId::kSitDownShake:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_sitDown_posCtrl_shake.toml" );
                        break;
                    case MotionId::kMotionBallet:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_ballet.toml" );
                        break;
                    case MotionId::kMotionMoonwalk:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_moonwalk.toml" );
                        break;
                    case MotionId::kMotionFrontLift:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_front_lift_out_in.toml" );
                        break;
                    case MotionId::kMotionRearLift:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_rear_lift_out_in.toml" );
                        break;
                    case MotionId::kMotionPitchLeft:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_pitch_down_left.toml" );
                        break;
                    case MotionId::kMotionPitchRight:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_pitch_down_right.toml" );
                        break;
                    case MotionId::kMotionDiagonalRight:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_diagonal_right.toml" );
                        break;
                    case MotionId::kMotionDiagonalLeft:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_diagonal_left.toml" );
                        break;
                    case MotionId::kMotionWalkWave:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_walk_wave.toml" );
                        break;
                    case MotionId::kMotionTrotInOut:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_trot_in_out.toml" );
                        break;
                    case MotionId::kMotionTrotPitch:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_trot_pitch.toml" );
                        break;
                    case MotionId::kMotionFrontLiftForward:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_front_lift_left_right.toml" );
                        break;
                    case MotionId::kMotionRearLiftForward:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_rear_lift_left_right.toml" );
                        break;
                    case MotionId::kMotionFrontSwitch:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_front_lift_switch.toml" );
                        break;
                    case MotionId::kMotionRearSwitch:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_rear_lift_switch.toml" );
                        break;
                    case MotionId::kMotionJump3d:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_jump3D.toml" );
                        break;
                    case MotionId::kMotionTrotSwing:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_trot_swing.toml" );
                        break;
                    case MotionId::kMotionSpecialPronk:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_special_pronk.toml" );
                        break;
                    case MotionId::kMotionSpecialTrot:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_special_trot.toml" );
                        break;
                    case MotionId::kMotionMoonwalkLeft:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_moonwalk_switch_left.toml" );
                        break;
                    case MotionId::kMotionMoonwalkRight:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_moonwalk_switch_right.toml" );
                        break;
                    case MotionId::kMotionMoonwalkBack:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_moonwalk_backward.toml" );
                        break;
                    case MotionId::kMotionUpDown:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_up_down.toml" );
                        break;
                    case MotionId::kMotionPushUp:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_push_up.toml" );
                        break;
                    case MotionId::kMotionPitchGainCalibration:
                        if ( MotionListCppTomlRead( "/mnt/misc/cyberdog2_pitch_gain_factory_calibrate.toml" ) < 0 )
                            MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_pitch_gain_factory_calibrate.toml" );
                        break;
                    case MotionId::kMotionUser00:
                        MotionListCppTomlRead( GetUserGaitParameterPath() + "cyberdog2_user80_ballet.toml" );
                        break;
                    case MotionId::kMotionUser01:
                        MotionListCppTomlRead( GetUserGaitParameterPath() + "cyberdog2_user81_front_lift.toml" );
                        break;
                    case MotionId::kMotionUser02:
                        MotionListCppTomlRead( GetUserGaitParameterPath() + "cyberdog2_user82_rear_lift.toml" );
                        break;
                    case MotionId::kMotionUser03:
                        MotionListCppTomlRead( GetUserGaitParameterPath() + "cyberdog2_user83_walk_wave.toml" );
                        break;
                    case MotionId::kMotionUser04:
                        MotionListCppTomlRead( GetUserGaitParameterPath() + "cyberdog2_user84_trot_in_out.toml" );
                        break;
                    case MotionId::kMotionUser05:
                        MotionListCppTomlRead( GetUserGaitParameterPath() + "cyberdog2_user85_diagonal_FR.toml" );
                        break;
                    case MotionId::kMotionUser06:
                        MotionListCppTomlRead( GetUserGaitParameterPath() + "cyberdog2_user86_diagonal_FL.toml" );
                        break;
                    case MotionId::kMotionUser07:
                        MotionListCppTomlRead( GetUserGaitParameterPath() + "cyberdog2_user87_pronk_front_back.toml" );
                        break;
                    case MotionId::kMotionUser08:
                        MotionListCppTomlRead( GetUserGaitParameterPath() + "cyberdog2_user88_FR_lift.toml" );
                        break;
                    case MotionId::kMotionUser09:
                        MotionListCppTomlRead( GetUserGaitParameterPath() + "cyberdog2_user89_FL_lift.toml" );
                        break;
                    case MotionId::kMotionUser10:
                        MotionListCppTomlRead( GetUserGaitParameterPath() + "cyberdog2_user90_moonwalk.toml" );
                        break;
                    case MotionId::kMotionUser11:
                        MotionListCppTomlRead( GetUserGaitParameterPath() + "cyberdog2_user91_amble.toml" );
                        break;
                    case MotionId::kMotionUser16:
                        MotionListCppTomlRead( GetUserGaitParameterPath() + "cyberdog2_user96_moonwalk_backward.toml" );
                        break;
                    case MotionId::kMotionUserGait:
                        MotionListFullCppTomlRead( user_gait_list_file_ );
                        break;
                    case MotionId::kDanceSet1:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_short_dance_1.toml" );
                        break;
                    case MotionId::kDanceSet2:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_short_dance_2.toml" );
                        break;
                    default:
                        MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog2() + "cyberdog2_motion.toml" );
                        break;
                    }
                    std::cout << "[CommandInterface] Load toml " << ( int )command_.gait_id << " with size=" << motion_list_step_ << std::endl;
                }
                else
                    MotionListCppTomlRead( GetPreinstalledMotionPathForCyberDog() + "toml/motion.toml" );
                motion_trigger_ = command_.gait_id != 0 ? command_.gait_id : 1;
                interface_iter_ = 0;
                command_        = cmd_list_.front();
            }
            else if ( motion_trigger_ == 0 && command_.cmd_source == kCyberdogLcmCmd ) {
                cmd_list_.pop();
                command_ = pose_empty_cmd_;
                cmd_list_.push( command_ );
                interface_iter_ = 0;
                motion_trigger_ = 1;
            }
        }
        else if ( command_.duration == 0 ) {
            motion_id_      = 0;
            motion_trigger_ = 0;
        }
        if ( current_switch_status_ != 1 )  // Waiting for TRANSITIONING
            interface_iter_++;
    }
    else {
        command_ = cmd_cur_;
    }

    // MotionMode::kMotion & UserGait process bar calculate
    if ( motion_list_size_ != 0 ) {
        // Avoid motion_list_size_ and  motion_process_bar over 100
        command_.motion_process_bar = ( int16_t )( ( motion_list_size_ - motion_list_step_ ) * 100.0 / motion_list_size_ ) + 1;
        if ( motion_list_step_ == 1 && interface_iter_ > command_.duration * 9.0 / 10.0 )
            command_.motion_process_bar = 100;
        if ( command_.motion_process_bar > 100 )
            command_.motion_process_bar = 100;
        if ( motion_list_step_ == 0 )
            motion_list_size_ = 0;
    }
    command_.motion_id      = motion_id_;
    command_.motion_trigger = motion_trigger_;  // Flag of motion gait_id, used to overwrite lcm cmd resoponce
    command_.user_gait_file = user_gait_file_;

    static int mode_old = 0, gaitid_old = 0, duration_old = 0;
    using Clock = std::chrono::system_clock;
    auto tt     = Clock::to_time_t( Clock::now() );
    if ( command_.mode != mode_old || command_.gait_id != gaitid_old || command_.duration != duration_old )
        printf( "[CommandInterface]Run: mode=%d gait_id=%d dura=%d motion_step=%d at %s", command_.mode, command_.gait_id, command_.duration, motion_list_step_, ctime( &tt ) );
    mode_old     = command_.mode;
    gaitid_old   = command_.gait_id;
    duration_old = command_.duration;
}

void CommandInterface::Gamepad2Cmd( long int* control_mode, long int* gait_id, const RobotType& robotType ) {

    if ( gamepad_cmd_.back ) {
        *control_mode = MotionMode::kOff;
    }
    if ( gamepad_cmd_.a ) {
        *control_mode = MotionMode::kPureDamper;
    }
    if ( gamepad_cmd_.b ) {
        *control_mode = MotionMode::kRecoveryStand;
    }
    if ( gamepad_cmd_.x ) {
        *control_mode = MotionMode::kQpStand;
    }
    if ( gamepad_cmd_.y ) {
        *control_mode = MotionMode::kLocomotion;
    }
    if ( gamepad_cmd_.rightBumper ) {
        *gait_id = GaitId::kTrot10v4;
    }
    if ( gamepad_cmd_.leftBumper ) {
        *gait_id = GaitId::kTrotFast;
    }
    // default is trot
    if ( *gait_id == 0 && *control_mode == MotionMode::kLocomotion )
        *gait_id = GaitId::kTrot10v5;

    cmd_cur_.mode       = *control_mode;
    cmd_cur_.gait_id    = *gait_id;
    cmd_cur_.cmd_source = kGamepadCmd;

    if ( cmd_cur_.mode == MotionMode::kQpStand ) {  // QP stand
        // rpy desired
        // TODO: deadband
        cmd_cur_.vel_des[ 0 ] = 0;
        cmd_cur_.vel_des[ 1 ] = 0;
        cmd_cur_.vel_des[ 2 ] = 0;
        cmd_cur_.rpy_des[ 0 ] = gamepad_cmd_.leftStickAnalog[ 0 ] * 0.6;   // roll
        cmd_cur_.rpy_des[ 1 ] = gamepad_cmd_.leftStickAnalog[ 1 ] * 0.6;   // pitch
        cmd_cur_.rpy_des[ 2 ] = gamepad_cmd_.rightStickAnalog[ 1 ] * 0.6;  // yaw
        if ( cmd_cur_.gait_id == 1 || cmd_cur_.gait_id == 3 )
            cmd_cur_.pos_des[ 2 ] = 0.1 * gamepad_cmd_.rightStickAnalog[ 0 ];
        else
            cmd_cur_.pos_des[ 2 ] = ( ( robotType == RobotType::CYBERDOG2 ) ? 0.24 : 0.32 ) + 0.1 * gamepad_cmd_.rightStickAnalog[ 0 ];
        cmd_cur_.contact = 0x0F;
    }
    else if ( cmd_cur_.mode == MotionMode::kLocomotion || cmd_cur_.mode == MotionMode::kRlRapid ) {
        // x,y, yaw velocity command
        cmd_cur_.vel_des[ 0 ]     = gamepad_cmd_.leftStickAnalog[ 1 ];
        cmd_cur_.vel_des[ 1 ]     = -gamepad_cmd_.leftStickAnalog[ 0 ];
        cmd_cur_.vel_des[ 2 ]     = -gamepad_cmd_.rightStickAnalog[ 0 ];
        cmd_cur_.rpy_des[ 0 ]     = 0;
        cmd_cur_.rpy_des[ 2 ]     = 0;
        cmd_cur_.rpy_des[ 1 ]     = gamepad_cmd_.rightStickAnalog[ 1 ] * 0.4;
        cmd_cur_.pos_des[ 2 ]     = ( ( robotType == RobotType::CYBERDOG2 ) ? 0.24 : 0.32 );
        cmd_cur_.step_height[ 0 ] = ( ( robotType == RobotType::CYBERDOG2 ) ? 0.04 : 0.06 );
    }

    if ( cmd_cur_.duration == 0 ) {
        if ( cmd_cur_.mode == MotionMode::kMotion && motion_trigger_ > 0 ) {
            return;
        }
        else {
            while ( !cmd_list_.empty() )
                cmd_list_.pop();
        }
    }
    interface_iter_ = 0;
    cmd_list_.push( cmd_cur_ );
}

void CommandInterface::Rc2Cmd( const RobotType& robotType ) {
    // AT9S
    auto estop_switch         = rc_cmd_.SWE;
    auto QP_Locomotion_switch = rc_cmd_.SWA;
    auto left_select          = rc_cmd_.SWC;
    auto right_select         = rc_cmd_.SWD;
    auto jump_flip_switch     = rc_cmd_.SWG;
    auto step_height          = ( rc_cmd_.varB + 1.0 ) * ( ( robotType == RobotType::CYBERDOG2 ) ? 0.09 : 0.1 );
    // T8S
    auto left_tri_switch  = rc_cmd_.CH7;
    auto right_tri_switch = rc_cmd_.CH5;
    auto right_buttion    = rc_cmd_.CH6;

    cmd_cur_.cmd_source = kRcCmd;
    if ( rc_cmd_.rc_type == 2 ) {  // T8S
        switch ( left_tri_switch ) {
        case T8S_TRI_UP:
            switch ( right_tri_switch ) {
            case T8S_TRI_MIDDLE:
                cmd_cur_.mode    = MotionMode::kPureDamper;
                cmd_cur_.gait_id = 1;
                break;
            case T8S_TRI_UP:
                cmd_cur_.mode = MotionMode::kOff;
                break;
            default:
                cmd_cur_.mode    = MotionMode::kPureDamper;
                cmd_cur_.gait_id = 1;
                break;
            }
            break;
        case T8S_TRI_MIDDLE:
            switch ( right_tri_switch ) {
            case T8S_TRI_UP:
                cmd_cur_.mode    = MotionMode::kMotion;
                cmd_cur_.gait_id = MotionId::kHiFiveLeft;
                break;
            case T8S_TRI_MIDDLE:
                cmd_cur_.mode    = MotionMode::kRecoveryStand;
                switch_tri_flag_ = -1;
                break;
            case T8S_TRI_DOWN:
                if ( switch_tri_flag_ == -1 ) {
                    if ( rc_cmd_.left_stick_x > 0.8 && rc_cmd_.left_stick_y < -0.8 )
                        switch_tri_flag_ = 1;
                    else if ( rc_cmd_.left_stick_x > 0.8 && rc_cmd_.left_stick_y > 0.8 )
                        switch_tri_flag_ = 2;
                    else if ( rc_cmd_.left_stick_x < -0.8 && rc_cmd_.left_stick_y < -0.8 )
                        switch_tri_flag_ = 3;
                    else if ( rc_cmd_.left_stick_x < -0.8 && rc_cmd_.left_stick_y > 0.8 )
                        switch_tri_flag_ = 4;
                    else
                        switch_tri_flag_ = 0;
                    std::cout << "function switch_tri_flag_=" << switch_tri_flag_ << std::endl;
                }
                switch ( switch_tri_flag_ ) {
                case 0:
                    cmd_cur_.mode = MotionMode::kTwoLegStand;
                    break;
                case 1:
                    cmd_cur_.mode = MotionMode::kJump3d;
                    break;
                default:
                    cmd_cur_.mode = MotionMode::kRecoveryStand;
                }
                break;
            }
            speed_offset_rc_flag_ = false;
            break;
        case T8S_TRI_DOWN:
            switch ( right_tri_switch ) {
                // cmd_cur_.mode = MotionMode::kLocomotion;
            case T8S_TRI_UP:
                cmd_cur_.mode = MotionMode::kRecoveryStand;
                // cmd_cur_.gait_id = GaitId::kTrotFast;
                break;
            case T8S_TRI_MIDDLE:
                cmd_cur_.gait_id = GaitId::kTrotMedium;
                // // cmd_cur_.gait_id = GaitId::kTrot24v16;
                // // cmd_cur_.gait_id = GaitId::kTrot20v12Follow;
                break;
            case T8S_TRI_DOWN:
                cmd_cur_.gait_id = GaitId::kTrotSlow;  // For factory calibrate
                // // cmd_cur_.gait_id = GaitId::kBound;
                break;
            }
#if ( ONBOARD_BUILD == 1 )
            int ret = 1;
            if ( rc_cmd_old_.CH7 != rc_cmd_.CH7 ) {
                if ( right_buttion == T8S_BOOL_DOWN ) {
                    ret = system( "mount -o remount,rw /mnt/misc/" );
                    if ( ret < 0 )
                        printf( "[FACTORY] remount /mnt/misc/ Failed!\n" );
                    else
                        speed_offset_rc_flag_ = true;  // For factory speed offset set
                    printf( "[FACTORY] Enter factory offset mode! varB=%.2f\n", rc_cmd_.varB );
                }
            }
#endif
            break;
        }
        // For factory speed offset trigger in runSpeedConfig()
        if ( right_buttion == T8S_BOOL_DOWN && speed_offset_rc_flag_ ) {
            if ( rc_cmd_.varB > 0 )
                speed_offset_rc_trigger_ = 1;
            else if ( rc_cmd_.varB < -0.5 )
                speed_offset_rc_trigger_ = 4;
            else if ( right_tri_switch == T8S_TRI_MIDDLE )
                speed_offset_rc_trigger_ = 2;  // front gain
            else if ( right_tri_switch == T8S_TRI_DOWN )
                speed_offset_rc_trigger_ = 3;  // back gain
            else
                speed_offset_rc_trigger_ = 0;
        }
        else
            speed_offset_rc_trigger_ = 0;
    }
    else {  // AT9S
        switch ( estop_switch ) {
        case AT9S_TRI_UP:
            cmd_cur_.mode = MotionMode::kOff;
            break;

        case AT9S_TRI_MIDDLE:
            switch ( jump_flip_switch ) {
            case AT9S_TRI_UP:
                if ( QP_Locomotion_switch == AT9S_BOOL_DOWN ) {
                    cmd_cur_.mode = MotionMode::kMotion;
                    switch ( right_select ) {
                    case AT9S_BOOL_UP:
                        switch ( left_select ) {
                        case AT9S_TRI_UP:
                            // cmd_cur_.gait_id = MotionId::kMotionAllInOne;
                            // cmd_cur_.gait_id = MotionId::kHiFiveLeft;
                            // cmd_cur_.gait_id = MotionId::kHiFiveRight;
                            // cmd_cur_.gait_id = MotionId::kMotionBallet;
                            cmd_cur_.gait_id = MotionId::kMotionMoonwalk;
                            break;
                        case AT9S_TRI_MIDDLE:
                            cmd_cur_.gait_id = MotionId::kMotionFrontLift;
                            // cmd_cur_.gait_id = MotionId::kMotionRearLift;
                            // cmd_cur_.gait_id = MotionId::kMotionPitchLeft;
                            // cmd_cur_.gait_id = MotionId::kMotionPitchRight;
                            // cmd_cur_.gait_id = MotionId::kMotionDiagonalLeft;
                            // cmd_cur_.gait_id = MotionId::kMotionDiagonalRight;
                            break;
                        case AT9S_TRI_DOWN:
                            cmd_cur_.gait_id = MotionId::kMotionWalkWave;
                            // cmd_cur_.gait_id = MotionId::kMotionTrotInOut;
                            // cmd_cur_.gait_id = MotionId::kMotionTrotPitch;
                            // cmd_cur_.gait_id = MotionId::kMotionFrontLiftForward;
                            // cmd_cur_.gait_id = MotionId::kMotionRearLiftForward;
                            break;
                        }
                        break;
                    case AT9S_BOOL_DOWN:
                        switch ( left_select ) {
                        case AT9S_TRI_UP:
                            // cmd_cur_.gait_id = MotionId::kMotionFrontSwitch;
                            // cmd_cur_.gait_id = MotionId::kMotionRearSwitch;
                            cmd_cur_.gait_id = MotionId::kMotionJump3d;
                            // cmd_cur_.gait_id = MotionId::kMotionTrotSwing;
                            break;
                        case AT9S_TRI_MIDDLE:
                            // cmd_cur_.gait_id = MotionId::kMotionSpecialPronk;
                            // cmd_cur_.gait_id = MotionId::kMotionSpecialTrot;
                            // cmd_cur_.gait_id = MotionId::kMotionMoonwalkLeft;
                            // cmd_cur_.gait_id = MotionId::kMotionMoonwalkRight;
                            cmd_cur_.gait_id = MotionId::kMotionMoonwalkBack;
                            break;
                        case AT9S_TRI_DOWN:
                            // cmd_cur_.gait_id = MotionId::kMotionFrontLiftForward;
                            cmd_cur_.gait_id = MotionId::kMotionBallet;
                            break;
                        }
                        break;
                    }
                }
                break;
            case AT9S_TRI_MIDDLE:
                if ( QP_Locomotion_switch == AT9S_BOOL_UP ) {
                    cmd_cur_.mode = MotionMode::kPureDamper;
                }
                else if ( QP_Locomotion_switch == AT9S_BOOL_DOWN ) {
                    cmd_cur_.mode = MotionMode::kRecoveryStand;
                    // if ( rc_cmd_old_.SWD != rc_cmd_.SWD )
                    //     cmd_cur_.gait_id = 17;
                    // else
                    //     cmd_cur_.gait_id = 0;
                }
                break;
            case AT9S_TRI_DOWN:
                if ( QP_Locomotion_switch == AT9S_BOOL_DOWN ) {
                    // cmd_cur_.mode = MotionMode::kTwoLegStand;
                    cmd_cur_.mode = MotionMode::kJump3d;
                    switch ( right_select ) {
                    case AT9S_BOOL_UP:
                        switch ( left_select ) {
                        case AT9S_TRI_UP:
                            cmd_cur_.gait_id = JumpId::kJumpPosYaw90;
                            break;
                        case AT9S_TRI_MIDDLE:
                            cmd_cur_.gait_id = JumpId::kJumpPosZ30;
                            // cmd_cur_.gait_id = JumpId::kJumpPosX60;
                            break;
                        case AT9S_TRI_DOWN:
                            cmd_cur_.gait_id = JumpId::kJumpDownStair;
                            break;
                        }
                        break;
                    case AT9S_BOOL_DOWN:
                        switch ( left_select ) {
                        case AT9S_TRI_UP:
                            cmd_cur_.gait_id = JumpId::kJumpNegYaw90;
                            // cmd_cur_.gait_id = JumpId::kJumpDonwTrunk;
                            break;
                        case AT9S_TRI_MIDDLE:
                            cmd_cur_.gait_id = JumpId::kJumpPosX30;
                            break;
                        case AT9S_TRI_DOWN:
                            cmd_cur_.gait_id = JumpId::kJumpNegY20;
                            break;
                        }
                        break;
                    }
                }
                break;
            }
            break;

        case AT9S_TRI_DOWN:
            switch ( jump_flip_switch ) {
            case AT9S_TRI_UP:
                cmd_cur_.mode = MotionMode::kPureDamper;
                break;

            case AT9S_TRI_MIDDLE:
                if ( QP_Locomotion_switch == AT9S_BOOL_UP ) {
                    cmd_cur_.mode = MotionMode::kLocomotion;
                }
                else if ( QP_Locomotion_switch == AT9S_BOOL_DOWN ) {
                    cmd_cur_.mode = MotionMode::kQpStand;
                }
                break;

            case AT9S_TRI_DOWN:
                cmd_cur_.mode = MotionMode::kPureDamper;
                break;
            }
            break;
        }

        // Get gait id
        if ( cmd_cur_.mode == MotionMode::kLocomotion ) {
            switch ( right_select ) {
            case AT9S_BOOL_UP:
                switch ( left_select ) {
                case AT9S_TRI_UP:
                    // cmd_cur_.gait_id = GaitId::kTrot10v5;
                    cmd_cur_.gait_id = GaitId::kTrotFast;
                    break;
                case AT9S_TRI_MIDDLE:
                    cmd_cur_.gait_id = GaitId::kTrotMedium;
                    // cmd_cur_.gait_id = GaitId::kStandNoPr;
                    break;
                case AT9S_TRI_DOWN:
                    // cmd_cur_.gait_id = GaitId::kPassiveTrot;
                    cmd_cur_.gait_id = GaitId::kTrotSlow;
                    break;
                }
                break;
            case AT9S_BOOL_DOWN:
                switch ( left_select ) {
                case AT9S_TRI_UP:
                    cmd_cur_.gait_id = GaitId::kTrot24v16;
                    // cmd_cur_.gait_id = GaitId::kTrot8v3;
                    break;
                case AT9S_TRI_MIDDLE:
                    cmd_cur_.gait_id = GaitId::kBound;
                    // cmd_cur_.gait_id = GaitId::kStand;
                    break;
                case AT9S_TRI_DOWN:
                    cmd_cur_.gait_id = GaitId::kPronk;
                    // cmd_cur_.gait_id = GaitId::kWalk;
                    break;
                }
                break;
            }
        }
    }

    // Get velocity & rpy cmd
    if ( cmd_cur_.mode == MotionMode::kLocomotion || cmd_cur_.mode == MotionMode::kRlRapid ) {
        //  unify scale of rc and ui
        cmd_cur_.vel_des[ 0 ] = ApplyDeadband( 1. * rc_cmd_.right_stick_x, 0.1 );
        cmd_cur_.vel_des[ 1 ] = ApplyDeadband( -1. * rc_cmd_.right_stick_y, 0.1 );
        cmd_cur_.vel_des[ 2 ] = ApplyDeadband( -1. * rc_cmd_.left_stick_y, 0.1 );

        cmd_cur_.rpy_des[ 0 ]     = 0;
        cmd_cur_.rpy_des[ 1 ]     = ApplyDeadband( 0.4 * rc_cmd_.left_stick_x, 0.1, -0.4, 0.4 );
        cmd_cur_.rpy_des[ 2 ]     = 0;
        cmd_cur_.pos_des[ 2 ]     = 0.3;
        cmd_cur_.step_height[ 0 ] = step_height;

        // For factory speed offset calibration
        if ( speed_offset_rc_flag_ ) {
            if ( rc_cmd_.varB > 0 ) {  // Speed Offset
                if ( rc_cmd_.varB < 0.5 )
                    switch ( right_tri_switch ) {
                    case T8S_TRI_UP:
                        cmd_cur_.gait_id = GaitId::kBallet;
                        break;
                    case T8S_TRI_MIDDLE:
                        cmd_cur_.gait_id = GaitId::kBound;
                        break;
                    case T8S_TRI_DOWN:
                        cmd_cur_.gait_id = GaitId::kTrot24v16;
                        break;
                    default:
                        break;
                    }
                cmd_cur_.vel_des[ 0 ]     = 0.1 * rc_cmd_.right_stick_x;
                cmd_cur_.vel_des[ 1 ]     = -0.1 * rc_cmd_.right_stick_y;
                cmd_cur_.vel_des[ 2 ]     = -0.1 * rc_cmd_.left_stick_y;  // Discard the calibration of the yaw speed
                cmd_cur_.rpy_des[ 1 ]     = 0;
                cmd_cur_.step_height[ 0 ] = 0.05;
            }
            else {  // Origin rpy and rpy gain Offset
                cmd_cur_.vel_des[ 0 ] = ApplyDeadband( 1.6 * rc_cmd_.right_stick_x, 0.1 );
                cmd_cur_.vel_des[ 1 ] = 0;
                cmd_cur_.vel_des[ 2 ] = ApplyDeadband( -1.6 * rc_cmd_.right_stick_y, 0.1 );
                cmd_cur_.rpy_des[ 0 ] = 0.1 * rc_cmd_.left_stick_y;  // roll
                cmd_cur_.rpy_des[ 1 ] = 0.1 * rc_cmd_.left_stick_x;
                cmd_cur_.rpy_des[ 2 ] = 0.05 * rc_cmd_.left_stick_x;
                if ( rc_cmd_.varB < -0.5 ) {
                    switch ( right_tri_switch ) {
                    case T8S_TRI_UP:
                        cmd_cur_.gait_id = GaitId::kBallet;
                        break;
                    case T8S_TRI_MIDDLE:
                        cmd_cur_.gait_id = GaitId::kTrotSlow;
                        break;
                    default:
                        cmd_cur_.gait_id = GaitId::kTrotMedium;
                        break;
                    }
                }
                else if ( right_tri_switch == T8S_TRI_UP ) {
                    cmd_cur_.mode    = MotionMode::kMotion;
                    cmd_cur_.gait_id = MotionId::kMotionPitchGainCalibration;
                }
                else
                    cmd_cur_.gait_id = GaitId::kTrotFast;
                // For pitch vel gain
                cmd_cur_.step_height[ 0 ] = 0.05;
            }
        }
    }
    else if ( cmd_cur_.mode == MotionMode::kQpStand ) {
        cmd_cur_.rpy_des[ 0 ] = ApplyDeadband( 1. * rc_cmd_.left_stick_y, 0.1 );
        cmd_cur_.rpy_des[ 1 ] = ApplyDeadband( 1. * rc_cmd_.left_stick_x, 0.1 );
        cmd_cur_.rpy_des[ 2 ] = ApplyDeadband( -1. * rc_cmd_.right_stick_y, 0.1 );
        cmd_cur_.vel_des[ 2 ] = 0;
        if ( robotType == RobotType::CYBERDOG )
            cmd_cur_.pos_des[ 2 ] = ApplyDeadband( 0.3 + 0.17 * rc_cmd_.right_stick_x, 0.05, 0.13, 0.47 );
        else
            cmd_cur_.pos_des[ 2 ] = ApplyDeadband( 0.235 + 0.12 * rc_cmd_.right_stick_x, 0.05, 0.11, 0.3 );
    }
    rc_cmd_old_ = rc_cmd_;

    if ( cmd_cur_.duration == 0 ) {
        if ( cmd_cur_.mode == MotionMode::kMotion && motion_trigger_ > 0 ) {
            return;
        }
        else {
            while ( !cmd_list_.empty() )
                cmd_list_.pop();
        }
    }
    interface_iter_ = 0;
    cmd_list_.push( cmd_cur_ );
}

void CommandInterface::MotorLcm2Cmd() {
    cmd_cur_.cmd_source     = kMotorCmd;
    cmd_cur_.mode           = MotionMode::kMotorCtrl;
    cmd_cur_.duration       = 0;
    cmd_cur_.motor_ctrl     = motor_ctrl_cmd_;
    cmd_cur_.cmd_time_delay = MotorCtrl_LCM_timer_.GetElapsedMilliseconds();
    interface_iter_         = 0;
    while ( !cmd_list_.empty() )
        cmd_list_.pop();
    cmd_list_.push( cmd_cur_ );
}

void CommandInterface::CyberdogLcm2Cmd() {
    cmd_cur_.cmd_source = kCyberdogLcmCmd;
    Pattern2Mode( cyberdog_lcm_cmd_.pattern, cyberdog_lcm_cmd_.order, cmd_cur_.mode, cmd_cur_.gait_id );
    cmd_cur_.vel_des[ 0 ] = cyberdog_lcm_cmd_.linear[ 0 ];
    cmd_cur_.vel_des[ 1 ] = cyberdog_lcm_cmd_.linear[ 1 ];
    if ( cmd_cur_.mode == MotionMode::kQpStand ) {
        for ( int i = 0; i < 3; i++ ) {
            cmd_cur_.rpy_des[ i ] = cyberdog_lcm_cmd_.angular[ i ];
        }
    }
    else if ( cmd_cur_.mode == MotionMode::kLocomotion ) {
        cmd_cur_.rpy_des[ 0 ] = cyberdog_lcm_cmd_.angular[ 0 ];
        cmd_cur_.rpy_des[ 1 ] = cyberdog_lcm_cmd_.angular[ 1 ];
        cmd_cur_.vel_des[ 2 ] = cyberdog_lcm_cmd_.angular[ 2 ];
    }
    cmd_cur_.pos_des[ 0 ]     = 0;
    cmd_cur_.pos_des[ 1 ]     = 0;
    cmd_cur_.pos_des[ 2 ]     = cyberdog_lcm_cmd_.body_height;
    cmd_cur_.step_height[ 0 ] = cyberdog_lcm_cmd_.gait_height;
    cmd_cur_.duration         = 0;

    if ( cmd_cur_.duration == 0 ) {
        if ( cmd_cur_.mode == MotionMode::kMotion && motion_trigger_ > 0 ) {
            return;
        }
        else {
            while ( !cmd_list_.empty() )
                cmd_list_.pop();
        }
    }
    else {
        while ( cmd_list_.size() > 0 ) {
            if ( cmd_list_.front().duration == 0 )
                cmd_list_.pop();
            else {
                break;
            }
        }
    }

    interface_iter_ = 0;
    cmd_list_.push( cmd_cur_ );
}
void CommandInterface::Lcm2Cmd() {
    int i;
    if ( lcm_cmd_.life_count == life_count_ || lcm_cmd_.mode == -1 ) {
        /*
        //连续动作超过2s不更新life count速度降为0,3S后站立，3.5s后趴下
        if ( lcm_cmd_.duration == 0 && lcm_cmd_.mode == MotionMode::kLocomotion && lcm_cmd_.gait_id != GaitId::kStand && lcm_cmd_.gait_id != GaitId::kStandNoPr ) {
            if ( lcm_duration_timer_.GetElapsedMilliseconds() > 2000 ) {
                for ( i = 0; i < 3; i++ )
                    cmd_cur_.vel_des[ i ] = 0;
            }
            else if ( lcm_duration_timer_.GetElapsedMilliseconds() > 3000 ) {
                cmd_cur_.mode = MotionMode::kRecoveryStand;
            }
            else if ( lcm_duration_timer_.GetElapsedMilliseconds() > 3500 ) {
                cmd_cur_.mode = MotionMode::kPureDamper;
            }
        }
        */
        return;
    }
    lcm_duration_timer_.StartTimer();

    life_count_               = lcm_cmd_.life_count;
    cmd_cur_.mode             = lcm_cmd_.mode;
    cmd_cur_.gait_id          = lcm_cmd_.gait_id;
    cmd_cur_.contact          = lcm_cmd_.contact;
    cmd_cur_.value            = lcm_cmd_.value;
    cmd_cur_.duration         = lcm_cmd_.duration / 2;
    cmd_cur_.cmd_source       = kCyberdog2LcmCmd;
    cmd_cur_.step_height[ 0 ] = lcm_cmd_.step_height[ 0 ];
    cmd_cur_.step_height[ 1 ] = lcm_cmd_.step_height[ 1 ];
    for ( i = 0; i < 3; i++ ) {
        cmd_cur_.vel_des[ i ]       = lcm_cmd_.vel_des[ i ];
        cmd_cur_.rpy_des[ i ]       = lcm_cmd_.rpy_des[ i ];
        cmd_cur_.pos_des[ i ]       = lcm_cmd_.pos_des[ i ];
        cmd_cur_.acc_des[ i ]       = lcm_cmd_.acc_des[ i ];
        cmd_cur_.acc_des[ i + 3 ]   = lcm_cmd_.acc_des[ i + 3 ];
        cmd_cur_.foot_pose[ i ]     = lcm_cmd_.foot_pose[ i ];
        cmd_cur_.foot_pose[ i + 3 ] = lcm_cmd_.foot_pose[ i + 3 ];
        cmd_cur_.ctrl_point[ i ]    = lcm_cmd_.ctrl_point[ i ];
    }

    // Don't clear the list when push kMotion cmd even the duration = 0
    if ( cmd_cur_.duration == 0 && cmd_cur_.mode != MotionMode::kMotion ) {
        while ( !cmd_list_.empty() )
            cmd_list_.pop();
        interface_iter_   = 0;
        motion_list_step_ = 0;
        motion_list_size_ = 0;
    }
    else {
        if ( cmd_cur_.mode == MotionMode::kMotion && motion_trigger_ > 0 )
            return;
        while ( cmd_list_.size() > 0 ) {
            if ( cmd_list_.front().duration == 0 ) {
                cmd_list_.pop();
                interface_iter_ = 0;
            }
            else {
                break;
            }
        }
    }
    // Insert transition command for lie down to side， only tirgger once
    if ( lcm_cmd_.mode == MotionMode::kRecoveryStand && ( lcm_cmd_.gait_id == 17 || lcm_cmd_.gait_id == 18 ) && lcm_cmd_.duration == 0 ) {
        cmd_cur_.gait_id  = 0;
        cmd_cur_.duration = 4;
        cmd_list_.push( cmd_cur_ );
        cmd_cur_.gait_id  = lcm_cmd_.gait_id;
        cmd_cur_.duration = 0;
    }
    if ( lcm_cmd_.mode == MotionMode::kLocomotion && lcm_cmd_.gait_id == kUserGait ) {
        motion_list_step_++;  // List remaining actions
        motion_list_size_++;  // The total list size
        // std::cout <<"start motion_list_step_ " << motion_list_step_ <<std::endl;
    }

    if ( cmd_cur_.mode == MotionMode::kLocomotion && cmd_cur_.value & 0x04 ) {
        int ret = system( "mount -o remount,rw /mnt/misc/" );  // For factory speed offset set
        printf( "[CommandInterface]Factory speed offset lcm set, Remount misc %d\n", ret );
        speed_offset_rc_flag_    = true;
        speed_offset_rc_trigger_ = 1;
        speed_offset_count_      = 0;
    }
    else {
        speed_offset_count_++;
        speed_offset_rc_trigger_ = 0;
    }

    cmd_list_.push( cmd_cur_ );
    static int lcm_mode_old = 0, lcm_gaitid_old = 0;
    if ( ( cmd_cur_.mode != lcm_mode_old ) || ( cmd_cur_.gait_id != lcm_gaitid_old ) )
        printf( "[CommandInterface]Push command_=%d %d vel=%.2f %.2f %.2f rpy=%.2f %.2f %.2f pos=%.2f %.2f %.2f dura=%d list_size=%ld \n", cmd_cur_.mode, cmd_cur_.gait_id, cmd_cur_.vel_des[ 0 ],
                cmd_cur_.vel_des[ 1 ], cmd_cur_.vel_des[ 2 ], cmd_cur_.rpy_des[ 0 ], cmd_cur_.rpy_des[ 1 ], cmd_cur_.rpy_des[ 2 ], cmd_cur_.pos_des[ 0 ], cmd_cur_.pos_des[ 1 ], cmd_cur_.pos_des[ 2 ],
                cmd_cur_.duration, cmd_list_.size() );
    lcm_mode_old   = cmd_cur_.mode;
    lcm_gaitid_old = cmd_cur_.gait_id;
}

void CommandInterface::Default2Cmd( int control_mode, int cmpc_gait ) {
    ZeroCmd( command_ );
    cmd_cur_.mode             = control_mode;
    cmd_cur_.gait_id          = cmpc_gait;
    cmd_cur_.pos_des[ 2 ]     = 0.24;
    cmd_cur_.step_height[ 0 ] = 0.07;
    while ( !cmd_list_.empty() )
        cmd_list_.pop();
    interface_iter_ = 0;
    cmd_list_.push( cmd_cur_ );
}

void CommandInterface::Pattern2Mode( int pattern, int order, int8_t& control_mode, int8_t& gait_num ) {
    const int8_t mode_proj_tb[ 13 ] = { // 0 - 4
                                        kOff, kOff, kPureDamper, kRecoveryStand, kQpStand,
                                        // 5 - 11
                                        kMotion, kLocomotion, kLocomotion, kLocomotion, kLocomotion, kLocomotion, kLocomotion, kLocomotion
    };
    const int8_t gait_proj_tb[ 8 ]  = { kWalk, kWalk, kTrotMedium, kTrot10v5, kTrot10v4, kBound, kPronk, kPassiveTrot };
    const int8_t order_proj_tb[ 5 ] = { kMotion, kMotion, kTwoLegStand, kRecoveryStand, kMotion };

    control_mode = kPureDamper;  // default is pure damper
    if ( order >= 14 && order <= 18 ) {
        control_mode = order_proj_tb[ order - 14 ];
        gait_num     = order;
    }
    else if ( 0 <= pattern && pattern <= 4 ) {  // not in locomotion, so without gait id
        control_mode = mode_proj_tb[ pattern ];
    }
    else if ( pattern <= 12 ) {
        control_mode = mode_proj_tb[ pattern ];
        gait_num     = gait_proj_tb[ pattern - 5 ];  // resolve gait number
    }
}

int CommandInterface::MotionListFullTomlRead( std::string file_string ) {
    MotionControlCommand       msg;
    std::string                buffer;
    std::stringstream          file_buffer;
    std::vector< std::string > elems;

    ZeroCmd( msg );
    msg.cmd_source = kCyberdog2LcmCmd;

    if ( file_string.substr( file_string.length() - 5, file_string.length() ) == ".toml" ) {  // Read file
        std::ifstream file( file_string );
        if ( !file.is_open() ) {
            std::cout << "[Commandinterface] Motion_list_full_toml Error opening " << file_string << std::endl;
            return 0;
        }
        file_buffer << file.rdbuf();
        file.clear();
        file.close();
    }
    else {
        file_buffer << file_string;
        std::cout << "[Gait] Get file string:" << file_buffer.str() << std::endl;
    }
    while ( std::getline( file_buffer, buffer, '\n' ) ) {
        if ( !buffer.empty() ) {
            elems.push_back( buffer );
        }
    }
    int step_nums = 0;
    for ( uint i = 0; i < elems.size() - 12; i++ ) {
        if ( elems[ i ].substr( 0, 1 ) == "#" || elems[ i ].substr( 0, 2 ) == " #" )
            continue;
        if ( elems[ i ].substr( 0, 8 ) == "[[step]]" && ( elems.size() - 13 >= i ) ) {
            msg.mode    = stoi( Get( elems[ i + 1 ], "mode" )[ 0 ] );
            msg.gait_id = stoi( Get( elems[ i + 2 ], "gait_id" )[ 0 ] );
            msg.contact = stoi( Get( elems[ i + 3 ], "contact" )[ 0 ] );
            // msg.life_count = stoi( Get( elems[ i + 4 ], "life_count" )[ 0 ] );
            for ( int j = 0; j < 3; j++ ) {
                msg.vel_des[ j ]       = stof( Get( elems[ i + 5 ], "vel_des" )[ j ] );
                msg.rpy_des[ j ]       = stof( Get( elems[ i + 6 ], "rpy_des" )[ j ] );
                msg.pos_des[ j ]       = stof( Get( elems[ i + 7 ], "pos_des" )[ j ] );
                msg.acc_des[ j ]       = stof( Get( elems[ i + 8 ], "acc_des" )[ j ] );
                msg.acc_des[ j + 3 ]   = stof( Get( elems[ i + 8 ], "acc_des" )[ j + 3 ] );
                msg.ctrl_point[ j ]    = stof( Get( elems[ i + 9 ], "ctrl_point" )[ j ] );
                msg.foot_pose[ j ]     = stof( Get( elems[ i + 10 ], "foot_pose" )[ j ] );
                msg.foot_pose[ j + 3 ] = stof( Get( elems[ i + 10 ], "foot_pose" )[ j + 3 ] );
            }
            msg.step_height[ 0 ] = stof( Get( elems[ i + 11 ], "step_height" )[ 0 ] );
            msg.step_height[ 1 ] = stof( Get( elems[ i + 11 ], "step_height" )[ 1 ] );
            msg.value            = stoi( Get( elems[ i + 12 ], "value" )[ 0 ] );
            msg.duration         = stoi( Get( elems[ i + 13 ], "duration" )[ 0 ] ) / 2;
            step_nums++;
            cmd_list_.push( msg );
        }
    }
    if ( step_nums == 0 )
        cmd_list_.push( pose_empty_cmd_ );
    motion_list_step_ = cmd_list_.size();  // List remaining actions
    motion_list_size_ = cmd_list_.size();  // The total list size
    return 0;
}

int CommandInterface::MotionListFullCppTomlRead( std::string file_string ) {
    MotionControlCommand       msg;
    std::string                buffer;
    std::stringstream          file_buffer;
    std::vector< std::string > elems;

    ZeroCmd( msg );
    msg.cmd_source = kCyberdog2LcmCmd;

    if ( file_string.substr( file_string.length() - 5, file_string.length() ) == ".toml" ) {  // Read file
        std::ifstream file( file_string );
        if ( !file.is_open() ) {
            std::cout << "[Commandinterface] Motion_list_full_toml Error opening " << file_string << std::endl;
            return 0;
        }
        file_buffer << file.rdbuf();
        file.clear();
        file.close();
    }
    else {
        file_buffer << file_string;
        std::cout << "[Gait] Get file string:" << file_buffer.str() << std::endl;
    }

    try {
        cpptoml::parser p{ file_buffer };
        auto            motion_list = p.parse();
        // std::cout << (*motion_list) << std::endl; // print for debugging

        if ( !motion_list->empty() ) {
            auto tarr = motion_list->get_table_array( "step" );

            for ( const auto& table : *tarr ) {
                // *table is a cpptoml::table
                msg.mode    = table->get_as< int >( "mode" ).value_or( msg.mode );
                msg.gait_id = table->get_as< int >( "gait_id" ).value_or( msg.gait_id );
                msg.contact = table->get_as< int >( "contact" ).value_or( msg.contact );
                // msg.life_count  = table->get_as< int >( "life_count" ).value_or( msg.life_count );
                msg.value    = table->get_as< int >( "value" ).value_or( msg.value );
                msg.duration = table->get_as< int >( "duration" ).value_or( msg.duration ) / 2;

                CpptomlGetVector( table, "vel_des", msg.vel_des );
                CpptomlGetVector( table, "rpy_des", msg.rpy_des );
                CpptomlGetVector( table, "pos_des", msg.pos_des );
                CpptomlGetVector( table, "acc_des", msg.acc_des );
                CpptomlGetVector( table, "ctrl_point", msg.ctrl_point );
                CpptomlGetVector( table, "foot_pose", msg.foot_pose );
                CpptomlGetVector( table, "step_height", msg.step_height );

                cmd_list_.push( msg );
            }
        }
        else {
            cmd_list_.push( pose_empty_cmd_ );
        }
        motion_list_step_ = cmd_list_.size();  // List remaining actions
        motion_list_size_ = cmd_list_.size();  // The total list size
        return 0;
    }
    catch ( const cpptoml::parse_exception& e ) {
        std::cerr << "Failed to parse full toml file"
                  << ": " << e.what() << std::endl;
        return 0;
    }
}

int CommandInterface::MotionListTomlRead( std::string file_name ) {

    MotionControlCommand msg;
    int                  i = 0;
    int                  support[ 4 ];
    std::string          type;
    std::string          buffer;
    std::ifstream        file( file_name );

    ZeroCmd( msg );
    msg.cmd_source = command_.cmd_source;

    if ( !file.is_open() ) {
        std::cout << "Error opening " << file_name << std::endl;
        return -1;
    }
    std::cout << "[CommandInterface] read motion list file:" << file_name << std::endl;
    while ( !file.eof() ) {
        getline( file, buffer );
        if ( buffer[ 0 ] == '#' || buffer.substr( 0, 2 ) == " #" )
            continue;
        if ( buffer.substr( 0, 8 ) == "[[step]]" ) {
            getline( file, buffer );

            if ( ( buffer.find( "pose" ) != std::string::npos ) || ( buffer.find( "swingleg" ) != std::string::npos ) ) {
                msg.mode    = MotionMode::kPoseCtrl;
                msg.gait_id = 1;
                if ( buffer.find( "swingleg" ) != std::string::npos )
                    msg.gait_id = 2;
                getline( file, buffer );
                for ( i = 0; i < 4; i++ ) {
                    support[ i ] = stof( Get( buffer, "foot_support" )[ i ] );
                }
                msg.contact = ( support[ 3 ] > 0 ? 1 : 0 ) * 8 + ( support[ 2 ] > 0 ? 1 : 0 ) * 4 + ( support[ 1 ] > 0 ? 1 : 0 ) * 2 + ( support[ 0 ] > 0 ? 1 : 0 );
                getline( file, buffer );
                for ( i = 0; i < 3; i++ ) {
                    msg.rpy_des[ i ] = stof( Get( buffer, "body_cmd" )[ i ] );
                    msg.pos_des[ i ] = stof( Get( buffer, "body_cmd" )[ i + 3 ] );
                }
                getline( file, buffer );
                for ( i = 0; i < 3; i++ )
                    msg.foot_pose[ i ] = stof( Get( buffer, "foot_pose" )[ i ] );
                getline( file, buffer );
                for ( i = 0; i < 3; i++ )
                    msg.ctrl_point[ i ] = stof( Get( buffer, "ctrl_point" )[ i ] );
                getline( file, buffer );
                msg.duration = stoi( Get( buffer, "duration" )[ 0 ] ) / 2;
                cmd_list_.push( msg );
            }
            else if ( buffer.find( "locomotion" ) != std::string::npos ) {
                msg.mode = MotionMode::kLocomotion;
                getline( file, buffer );
                for ( i = 0; i < 3; i++ ) {
                    msg.vel_des[ i ] = stof( Get( buffer, "vel_des" )[ i ] );
                    msg.rpy_des[ i ] = 0;
                    msg.pos_des[ i ] = 0;
                }
                getline( file, buffer );
                getline( file, buffer );
                msg.gait_id = stoi( Get( buffer, "gait_id" )[ 0 ] );
                getline( file, buffer );
                msg.duration         = stoi( Get( buffer, "duration" )[ 0 ] ) / 2;
                msg.step_height[ 0 ] = 0.06;
                msg.step_height[ 1 ] = 0.06;
                cmd_list_.push( msg );
            }
            else if ( buffer.find( "jumpacc" ) != std::string::npos ) {
                msg.mode    = MotionMode::kForceJump;
                msg.gait_id = 4;
                getline( file, buffer );
                for ( i = 0; i < 4; i++ )
                    support[ i ] = stof( Get( buffer, "foot_support" )[ i ] );
                msg.contact = ( support[ 3 ] > 0 ? 1 : 0 ) * 8 + ( support[ 2 ] > 0 ? 1 : 0 ) * 4 + ( support[ 1 ] > 0 ? 1 : 0 ) * 2 + ( support[ 0 ] > 0 ? 1 : 0 );
                getline( file, buffer );
                for ( i = 0; i < 3; i++ )
                    msg.acc_des[ i + 3 ] = stof( Get( buffer, "x_acc_cmd" )[ i ] );
                getline( file, buffer );
                for ( i = 0; i < 3; i++ )
                    msg.acc_des[ i ] = stof( Get( buffer, "w_acc_cmd" )[ i ] );
                getline( file, buffer );
                msg.duration = stoi( Get( buffer, "duration" )[ 0 ] ) / 2;
                cmd_list_.push( msg );
            }
            else if ( buffer.find( "transition" ) != std::string::npos ) {
                msg.mode    = MotionMode::kPoseCtrl;
                msg.gait_id = 5;
                getline( file, buffer );
                msg.pos_des[ 2 ] = stof( Get( buffer, "height" )[ 0 ] );
                getline( file, buffer );
                msg.duration = stoi( Get( buffer, "duration" )[ 0 ] ) / 2;
                cmd_list_.push( msg );
            }
            else if ( buffer.find( "recoverystand" ) != std::string::npos ) {
                msg.mode    = MotionMode::kRecoveryStand;
                msg.gait_id = 0;
                getline( file, buffer );
                msg.duration = stoi( Get( buffer, "duration" )[ 0 ] ) / 2;
                cmd_list_.push( msg );
            }
            else if ( buffer.find( "torctrlposture" ) != std::string::npos ) {
                msg.mode    = MotionMode::kQpStand;
                msg.gait_id = 3;
                getline( file, buffer );
                for ( i = 0; i < 4; i++ )
                    support[ i ] = stof( Get( buffer, "foot_support" )[ i ] );
                msg.contact = ( support[ 3 ] > 0 ? 1 : 0 ) * 8 + ( support[ 2 ] > 0 ? 1 : 0 ) * 4 + ( support[ 1 ] > 0 ? 1 : 0 ) * 2 + ( support[ 0 ] > 0 ? 1 : 0 );
                getline( file, buffer );
                for ( i = 0; i < 3; i++ ) {
                    msg.rpy_des[ i ] = stof( Get( buffer, "body_cmd" )[ i ] );
                    msg.pos_des[ i ] = stof( Get( buffer, "body_cmd" )[ i + 3 ] );
                }
                getline( file, buffer );
                for ( i = 0; i < 3; i++ )
                    msg.foot_pose[ i ] = stof( Get( buffer, "foot_pose" )[ i ] );
                getline( file, buffer );
                msg.duration = stoi( Get( buffer, "duration" )[ 0 ] ) / 2;
                cmd_list_.push( msg );
            }
            else if ( buffer.find( "usergait" ) != std::string::npos ) {
                msg.mode = MotionMode::kLocomotion;
                getline( file, buffer );  // body_vel_des: x y yaw
                for ( i = 0; i < 3; i++ ) {
                    msg.vel_des[ i ] = stof( Get( buffer, "body_vel_des" )[ i ] );
                }
                getline( file, buffer );  // body_pos_des : roll pitch yaw x y z
                for ( i = 0; i < 3; i++ ) {
                    msg.rpy_des[ i ] = stof( Get( buffer, "body_pos_des" )[ i ] );
                    msg.pos_des[ i ] = stof( Get( buffer, "body_pos_des" )[ i + 3 ] );
                }
                getline( file, buffer );  // landing_pos_des : x y
                for ( i = 0; i < 2; i++ ) {
                    msg.foot_pose[ i + 0 ]  = stof( Get( buffer, "landing_pos_des" )[ i ] );
                    msg.foot_pose[ i + 2 ]  = stof( Get( buffer, "landing_pos_des" )[ i + 3 ] );
                    msg.foot_pose[ i + 4 ]  = stof( Get( buffer, "landing_pos_des" )[ i + 6 ] );
                    msg.ctrl_point[ i + 0 ] = stof( Get( buffer, "landing_pos_des" )[ i + 9 ] );
                }
                getline( file, buffer );  // step_height : range [0.999, 0.001] with unit 0.001
                msg.step_height[ 0 ] = ceil( stof( Get( buffer, "step_height" )[ 0 ] ) * 1e3 ) + ceil( stof( Get( buffer, "step_height" )[ 1 ] ) * 1e3 ) * 1e3;
                msg.step_height[ 1 ] = ceil( stof( Get( buffer, "step_height" )[ 2 ] ) * 1e3 ) + ceil( stof( Get( buffer, "step_height" )[ 3 ] ) * 1e3 ) * 1e3;
                getline( file, buffer );  // WBC torso weight : roll pitch yaw x y z
                for ( i = 0; i < 6; i++ ) {
                    msg.acc_des[ i ] = stof( Get( buffer, "weight" )[ i ] );
                }
                getline( file, buffer );  // use MPC traj
                msg.value = stoi( Get( buffer, "use_mpc_traj" )[ 0 ] );
                getline( file, buffer );  // mu for WBC
                msg.ctrl_point[ 2 ] = stof( Get( buffer, "mu" )[ 0 ] );
                getline( file, buffer );  // landing_gait for swing_p_gain
                msg.contact = floor( stof( Get( buffer, "landing_gain" )[ 0 ] ) * 1e1 );
                getline( file, buffer );  // gait_id
                msg.gait_id = stoi( Get( buffer, "gait_id" )[ 0 ] );
                getline( file, buffer );  // duration : ms
                msg.duration = stoi( Get( buffer, "duration" )[ 0 ] ) / 2;
                cmd_list_.push( msg );
            }
            else if ( buffer.find( "jump3D" ) != std::string::npos ) {
                msg.mode = MotionMode::kJump3d;
                getline( file, buffer );  // gait_id
                msg.gait_id = stoi( Get( buffer, "gait_id" )[ 0 ] );
                getline( file, buffer );  // duration : ms
                msg.duration = stoi( Get( buffer, "duration" )[ 0 ] ) / 2;
                cmd_list_.push( msg );
            }
        }
    }
    motion_list_step_ = cmd_list_.size();  // List remaining actions
    motion_list_size_ = cmd_list_.size();  // The total list size
    file.clear();
    file.close();
    return 0;
}

int CommandInterface::MotionListCppTomlRead( std::string file_name ) {

    MotionControlCommand msg;
    ZeroCmd( msg );
    msg.cmd_source = command_.cmd_source;

    try {
        auto motion_list = cpptoml::parse_file( file_name );
        // std::cout << "[CommandInterface] read motion list file: " << file_name << std::endl;  // debug

        if ( !motion_list->empty() ) {
            auto tarr = motion_list->get_table_array( "step" );

            for ( const auto& table : *tarr ) {
                // *table is a cpptoml::table
                auto file = table->get_as< std::string >( "file" );
                if ( file ) {  // include file to define step
                    MotionListCppTomlRead( *file );
                    continue;
                }
                else {  // normal defintion
                    auto type = table->get_as< std::string >( "type" );
                    if ( *type == "pose" || *type == "swingleg" ) {
                        msg.mode = MotionMode::kPoseCtrl;

                        if ( *type == "swingleg" ) {
                            msg.gait_id = 2;
                        }
                        else {
                            msg.gait_id = 1;
                        }

                        auto support = table->get_array_of< double >( "foot_support" );
                        if ( support && support->size() == 4 ) {
                            msg.contact = ( ( *support )[ 3 ] > 0 ? 1 : 0 ) * 8 + ( ( *support )[ 2 ] > 0 ? 1 : 0 ) * 4 + ( ( *support )[ 1 ] > 0 ? 1 : 0 ) * 2 + ( ( *support )[ 0 ] > 0 ? 1 : 0 );
                        }

                        auto body_cmd = table->get_array_of< double >( "body_cmd" );
                        if ( body_cmd && body_cmd->size() == 6 ) {
                            std::copy( body_cmd->begin(), body_cmd->begin() + 3, msg.rpy_des );
                            std::copy( body_cmd->begin() + 3, body_cmd->begin() + 6, msg.pos_des );
                        }

                        auto foot_pose = table->get_array_of< double >( "foot_pose" );
                        if ( foot_pose && foot_pose->size() == 3 ) {
                            std::copy( foot_pose->begin(), foot_pose->end(), msg.foot_pose );
                        }

                        CpptomlGetVector( table, "ctrl_point", msg.ctrl_point );
                        msg.duration = table->get_as< int >( "duration" ).value_or( msg.duration ) / 2;

                        auto mirror = table->get_as< int >( "mirror" ).value_or( msg.value );
                        if ( mirror ) {
                            msg.value = mirror;
                        }

                        cmd_list_.push( msg );
                    }

                    if ( *type == "locomotion" ) {
                        msg.mode = MotionMode::kLocomotion;

                        CpptomlGetVector( table, "vel_des", msg.vel_des );
                        for ( int i = 0; i < 3; i++ ) {
                            msg.rpy_des[ i ] = 0;
                            msg.pos_des[ i ] = 0;
                        }
                        msg.gait_id  = table->get_as< int >( "gait_id" ).value_or( msg.gait_id );
                        msg.duration = table->get_as< int >( "duration" ).value_or( msg.duration ) / 2;

                        msg.step_height[ 0 ] = 0.06;
                        msg.step_height[ 1 ] = 0.06;
                        cmd_list_.push( msg );
                    }

                    if ( *type == "jumpacc" ) {
                        msg.mode    = MotionMode::kForceJump;
                        msg.gait_id = 4;

                        auto support = table->get_array_of< double >( "foot_support" );
                        if ( support && support->size() == 4 ) {
                            msg.contact = ( ( *support )[ 3 ] > 0 ? 1 : 0 ) * 8 + ( ( *support )[ 2 ] > 0 ? 1 : 0 ) * 4 + ( ( *support )[ 1 ] > 0 ? 1 : 0 ) * 2 + ( ( *support )[ 0 ] > 0 ? 1 : 0 );
                        }

                        auto x_acc_cmd = table->get_array_of< double >( "x_acc_cmd" );
                        if ( x_acc_cmd && x_acc_cmd->size() == 3 ) {
                            std::copy( x_acc_cmd->begin(), x_acc_cmd->end(), msg.acc_des + 3 );
                        }
                        auto w_acc_cmd = table->get_array_of< double >( "w_acc_cmd" );
                        if ( w_acc_cmd && w_acc_cmd->size() == 3 ) {
                            std::copy( w_acc_cmd->begin(), w_acc_cmd->end(), msg.acc_des );
                        }

                        msg.duration = table->get_as< int >( "duration" ).value_or( msg.duration ) / 2;
                        cmd_list_.push( msg );
                    }

                    if ( *type == "transition" ) {
                        msg.mode    = MotionMode::kPoseCtrl;
                        msg.gait_id = 5;

                        msg.pos_des[ 2 ] = table->get_as< double >( "height" ).value_or( msg.pos_des[ 2 ] );
                        msg.duration     = table->get_as< int >( "duration" ).value_or( msg.duration ) / 2;
                        cmd_list_.push( msg );
                    }

                    if ( *type == "recoverystand" ) {
                        msg.mode    = MotionMode::kRecoveryStand;
                        msg.gait_id = 0;

                        msg.duration = table->get_as< int >( "duration" ).value_or( msg.duration ) / 2;
                        cmd_list_.push( msg );
                    }

                    if ( *type == "torctrlposture" ) {
                        msg.mode    = MotionMode::kQpStand;
                        msg.gait_id = 3;

                        auto support = table->get_array_of< double >( "foot_support" );
                        if ( support && support->size() == 4 ) {
                            msg.contact = ( ( *support )[ 3 ] > 0 ? 1 : 0 ) * 8 + ( ( *support )[ 2 ] > 0 ? 1 : 0 ) * 4 + ( ( *support )[ 1 ] > 0 ? 1 : 0 ) * 2 + ( ( *support )[ 0 ] > 0 ? 1 : 0 );
                        }

                        auto body_cmd = table->get_array_of< double >( "body_cmd" );
                        if ( body_cmd && body_cmd->size() == 6 ) {
                            std::copy( body_cmd->begin(), body_cmd->begin() + 3, msg.rpy_des );
                            std::copy( body_cmd->begin() + 3, body_cmd->begin() + 6, msg.pos_des );
                        }

                        auto foot_pose = table->get_array_of< double >( "foot_pose" );
                        if ( foot_pose && foot_pose->size() == 3 ) {
                            std::copy( foot_pose->begin(), foot_pose->end(), msg.foot_pose );
                        }

                        msg.duration = table->get_as< int >( "duration" ).value_or( msg.duration ) / 2;
                        cmd_list_.push( msg );
                    }

                    if ( *type == "usergait" ) {
                        msg.mode = MotionMode::kLocomotion;

                        CpptomlGetVector( table, "body_vel_des", msg.vel_des );  // body_vel_des: x y yaw

                        auto body_pos_des = table->get_array_of< double >( "body_pos_des" );  // body_pos_des : roll pitch yaw x y z
                        if ( body_pos_des && body_pos_des->size() == 6 ) {
                            std::copy( body_pos_des->begin(), body_pos_des->begin() + 3, msg.rpy_des );
                            std::copy( body_pos_des->begin() + 3, body_pos_des->begin() + 6, msg.pos_des );
                        }

                        auto landing_pos_des = table->get_array_of< double >( "landing_pos_des" );  // landing_pos_des : x y
                        if ( landing_pos_des && landing_pos_des->size() == 12 ) {
                            for ( uint i = 0; i < 2; i++ ) {
                                msg.foot_pose[ i + 0 ]  = ( *landing_pos_des )[ i ];
                                msg.foot_pose[ i + 2 ]  = ( *landing_pos_des )[ i + 3 ];
                                msg.foot_pose[ i + 4 ]  = ( *landing_pos_des )[ i + 6 ];
                                msg.ctrl_point[ i + 0 ] = ( *landing_pos_des )[ i + 9 ];
                            }
                        }

                        auto step_height = table->get_array_of< double >( "step_height" );  // step_height : range [0.999, 0.001] with unit 0.001
                        if ( step_height && step_height->size() == 4 ) {
                            msg.step_height[ 0 ] = ceil( ( *step_height )[ 0 ] * 1e3 ) + ceil( ( *step_height )[ 1 ] * 1e3 ) * 1e3;
                            msg.step_height[ 1 ] = ceil( ( *step_height )[ 2 ] * 1e3 ) + ceil( ( *step_height )[ 3 ] * 1e3 ) * 1e3;
                        }

                        CpptomlGetVector( table, "weight", msg.acc_des );                                                                  // WBC torso weight : roll pitch yaw x y z
                        msg.value           = table->get_as< int >( "use_mpc_traj" ).value_or( msg.value );                                // use MPC traj
                        msg.ctrl_point[ 2 ] = table->get_as< double >( "mu" ).value_or( msg.ctrl_point[ 2 ] );                             // mu for WBC
                        msg.contact         = floor( table->get_as< double >( "landing_gain" ).value_or( double( msg.contact ) ) * 1e1 );  // mu for WBC
                        msg.gait_id         = table->get_as< int >( "gait_id" ).value_or( msg.gait_id );                                   // gait_id
                        msg.duration        = table->get_as< int >( "duration" ).value_or( msg.duration ) / 2;                             // duration : ms
                        cmd_list_.push( msg );
                    }

                    if ( *type == "jump3D" ) {
                        msg.mode = MotionMode::kJump3d;

                        msg.gait_id  = table->get_as< int >( "gait_id" ).value_or( msg.gait_id );
                        msg.duration = table->get_as< int >( "duration" ).value_or( msg.duration ) / 2;
                        cmd_list_.push( msg );
                    }
                    if ( *type == "rl_reset" ) {
                        msg.mode = MotionMode::kRlReset;

                        msg.gait_id  = table->get_as< int >( "gait_id" ).value_or( msg.gait_id );
                        msg.duration = table->get_as< int >( "duration" ).value_or( msg.duration ) / 2;
                        cmd_list_.push( msg );
                    }

                    if ( *type == "twoleg_stand" ) {
                        msg.mode     = MotionMode::kTwoLegStand;
                        msg.gait_id  = table->get_as< int >( "gait_id" ).value_or( msg.gait_id );
                        msg.duration = table->get_as< int >( "duration" ).value_or( msg.duration ) / 2;
                        cmd_list_.push( msg );
                    }

                    if ( *type == "rl_rapid" ) {
                        msg.mode = MotionMode::kRlRapid;

                        CpptomlGetVector( table, "vel_des", msg.vel_des );
                        msg.duration         = table->get_as< int >( "duration" ).value_or( msg.duration ) / 2;
                        msg.step_height[ 0 ] = 0.06;
                        msg.step_height[ 1 ] = 0.06;
                        cmd_list_.push( msg );
                    }

                }
            }
        }
        motion_list_step_ = cmd_list_.size();  // List remaining actions
        motion_list_size_ = cmd_list_.size();  // The total list size
        return 0;
    }
    catch ( const cpptoml::parse_exception& e ) {
        std::cerr << "Failed to parse " << file_name << ": " << e.what() << std::endl;
        return -1;
    }
}

int CommandInterface::MotionListTxtRead( std::string file_name ) {
    // using namespace std;
    while ( !cmd_list_.empty() )
        cmd_list_.pop();

    MotionControlCommand msg;
    int                  i = 0;
    int                  support[ 4 ];
    float                posture_des[ 6 ];
    std::string          buffer;
    std::ifstream        file( file_name );
    if ( !file.is_open() ) {
        std::cout << "Error opening " << file_name << std::endl;
        return -1;
    }
    while ( !file.eof() ) {
        getline( file, buffer );
        if ( buffer[ 0 ] == '#' || buffer.substr( 0, 2 ) == " #" )
            continue;
        if ( buffer.substr( 0, 4 ) == "pose" || buffer.substr( 0, 8 ) == "swingleg" ) {
            msg.mode    = MotionMode::kPoseCtrl;
            msg.gait_id = 1;
            getline( file, buffer );
            for ( i = 0; i < 4; i++ ) {
                support[ i ] = stof( Split( buffer, "," )[ i ] );
            }
            msg.contact = ( support[ 3 ] > 0 ? 1 : 0 ) * 8 + ( support[ 2 ] > 0 ? 1 : 0 ) * 4 + ( support[ 1 ] > 0 ? 1 : 0 ) * 2 + ( support[ 0 ] > 0 ? 1 : 0 );
            getline( file, buffer );
            for ( i = 0; i < 6; i++ )
                posture_des[ i ] = stof( Split( buffer, "," )[ i ] );
            for ( i = 0; i < 3; i++ ) {
                msg.rpy_des[ i ] = posture_des[ i ];
                msg.pos_des[ i ] = posture_des[ i + 3 ];
            }
            getline( file, buffer );
            for ( i = 0; i < 3; i++ )
                msg.foot_pose[ i ] = stof( Split( buffer, "," )[ i ] );
            getline( file, buffer );
            for ( i = 0; i < 3; i++ )
                msg.ctrl_point[ i ] = stof( Split( buffer, "," )[ i ] );
            getline( file, buffer );
            msg.duration = stoi( buffer ) / 2;
            cmd_list_.push( msg );

            // printf( " cmd_list_.push( msg );  \n" );
        }
        else if ( buffer.substr( 0, 10 ) == "locomotion" ) {
            msg.mode = MotionMode::kLocomotion;
            getline( file, buffer );
            for ( i = 0; i < 3; i++ ) {
                msg.vel_des[ i ] = stof( Split( buffer, "," )[ i ] );
                msg.rpy_des[ i ] = 0;
                msg.pos_des[ i ] = 0;
            }
            getline( file, buffer );
            // msg.locomotion_omni = stoi( buffer );
            getline( file, buffer );
            int locomotion_gait = stoi( buffer );
            getline( file, buffer );
            msg.duration = stoi( buffer ) / 2;
            msg.gait_id  = locomotion_gait;
            cmd_list_.push( msg );
        }
        else if ( buffer.substr( 0, 4 ) == "jump" ) {
            msg.mode    = MotionMode::kForceJump;
            msg.gait_id = 4;
            getline( file, buffer );
            for ( i = 0; i < 4; i++ )
                support[ i ] = stof( Split( buffer, "," )[ i ] );
            msg.contact = ( support[ 3 ] > 0 ? 1 : 0 ) * 8 + ( support[ 2 ] > 0 ? 1 : 0 ) * 4 + ( support[ 1 ] > 0 ? 1 : 0 ) * 2 + ( support[ 0 ] > 0 ? 1 : 0 );
            getline( file, buffer );
            for ( i = 0; i < 3; i++ )
                msg.acc_des[ i + 3 ] = stof( Split( buffer, "," )[ i ] );
            getline( file, buffer );
            for ( i = 0; i < 3; i++ )
                msg.acc_des[ i ] = stof( Split( buffer, "," )[ i ] );
            getline( file, buffer );
            msg.duration = stoi( buffer ) / 2;
            cmd_list_.push( msg );
        }
        else if ( buffer.substr( 0, 10 ) == "transition" ) {
            msg.mode    = MotionMode::kPoseCtrl;
            msg.gait_id = 5;
            getline( file, buffer );
            msg.pos_des[ 2 ] = stof( buffer );
            getline( file, buffer );
            msg.duration = stoi( buffer ) / 2;
            cmd_list_.push( msg );
        }
        else if ( buffer.substr( 0, 15 ) == "torctrlposture" ) {
            msg.mode    = MotionMode::kQpStand;
            msg.gait_id = 3;
            getline( file, buffer );
            for ( i = 0; i < 4; i++ )
                support[ i ] = stof( Split( buffer, "," )[ i ] );
            msg.contact = ( support[ 3 ] > 0 ? 1 : 0 ) * 8 + ( support[ 2 ] > 0 ? 1 : 0 ) * 4 + ( support[ 1 ] > 0 ? 1 : 0 ) * 2 + ( support[ 0 ] > 0 ? 1 : 0 );
            getline( file, buffer );
            for ( i = 0; i < 6; i++ )
                posture_des[ i ] = stof( Split( buffer, "," )[ i ] );
            for ( i = 0; i < 3; i++ ) {
                msg.rpy_des[ i ] = posture_des[ i ];
                msg.pos_des[ i ] = posture_des[ i + 3 ];
            }
            getline( file, buffer );
            for ( i = 0; i < 3; i++ )
                msg.foot_pose[ i ] = stof( Split( buffer, "," )[ i ] );
            getline( file, buffer );
            msg.duration = stoi( buffer ) / 2;
            cmd_list_.push( msg );
        }
    }
    file.clear();
    file.close();
    return 0;
}

std::vector< std::string > CommandInterface::Split( const std::string& str, const std::string& pattern ) {
    char* strc = new char[ strlen( str.c_str() ) + 1 ];
    strcpy( strc, str.c_str() );
    std::vector< std::string > resultVec;
    char*                      tmpStr = strtok( strc, pattern.c_str() );
    while ( tmpStr != NULL ) {
        resultVec.push_back( std::string( tmpStr ) );
        tmpStr = strtok( NULL, pattern.c_str() );
    }
    delete[] strc;
    return resultVec;
}

std::vector< std::string > CommandInterface::Get( const std::string& str, const std::string& target ) {
    // std::cout <<"\n get0  input: "<<str <<" target:" << target << std::endl;
    std::string value = "";
    if ( str.find( target ) != std::string::npos ) {
        if ( str.find( "[" ) == std::string::npos ) {
            std::vector< std::string > resultVec = Split( str, "=" );
            resultVec.erase( resultVec.begin() );
            // std::cout << "return1:" << resultVec[0] << std::endl;
            return resultVec;
        }
        value = Split( str, "[" )[ 1 ];
        value = Split( value, "]" )[ 0 ];
    }
    // std::cout << "return2:  " << Split(value,",")[0] << std::endl;
    return Split( value, "," );
}

void CommandInterface::ProcessLcmMotionCommand( const trajectory_command_lcmt* lcm_cmd ) {
    cmd_tmp_.cmd_source = kCyberdogLcmCmd;
    cmd_tmp_.mode       = MotionMode::kMotion;
    if ( lcm_cmd->motionType == "pose" || lcm_cmd->motionType == "swingleg" || lcm_cmd->motionType == "transition" ) {
        cmd_tmp_.mode = MotionMode::kPoseCtrl;
        if ( lcm_cmd->motionType == "transition" )
            cmd_tmp_.gait_id = 5;
        else
            cmd_tmp_.gait_id = 1;
    }
    else if ( lcm_cmd->motionType == "locomotion" ) {
        cmd_tmp_.mode    = MotionMode::kLocomotion;
        cmd_tmp_.gait_id = lcm_cmd->locomotion_gait;
    }
    else if ( lcm_cmd->motionType == "torctrlposture" ) {
        cmd_tmp_.mode    = MotionMode::kQpStand;
        cmd_tmp_.gait_id = 3;
    }
    else if ( lcm_cmd->motionType == "jump" ) {
        cmd_tmp_.mode    = MotionMode::kForceJump;
        cmd_tmp_.gait_id = 4;
    }
    else {
        cmd_tmp_.mode    = MotionMode::kPoseCtrl;
        cmd_tmp_.gait_id = 5;
    }
    for ( int i = 0; i < 3; i++ ) {
        cmd_tmp_.rpy_des[ i ]    = lcm_cmd->pose_body_cmd[ i ];
        cmd_tmp_.pos_des[ i ]    = lcm_cmd->pose_body_cmd[ i + 3 ];
        cmd_tmp_.foot_pose[ i ]  = lcm_cmd->pose_foot_cmd[ i ];
        cmd_tmp_.ctrl_point[ i ] = lcm_cmd->pose_ctrl_point[ i ];

        cmd_tmp_.vel_des[ i ]     = lcm_cmd->locomotion_vel[ i ];
        cmd_tmp_.acc_des[ i ]     = lcm_cmd->jump_w_acc[ i ];
        cmd_tmp_.acc_des[ i + 3 ] = lcm_cmd->jump_x_acc[ i ];
    }
    if ( lcm_cmd->motionType == "jump" )
        cmd_tmp_.contact = ( lcm_cmd->jump_contact[ 3 ] > 0 ? 1 : 0 ) * 8 + ( lcm_cmd->jump_contact[ 2 ] > 0 ? 1 : 0 ) * 4 + ( lcm_cmd->jump_contact[ 1 ] > 0 ? 1 : 0 ) * 2
                           + ( lcm_cmd->jump_contact[ 0 ] > 0 ? 1 : 0 );

    else
        cmd_tmp_.contact = ( lcm_cmd->pose_foot_support[ 3 ] > 0 ? 1 : 0 ) * 8 + ( lcm_cmd->pose_foot_support[ 2 ] > 0 ? 1 : 0 ) * 4 + ( lcm_cmd->pose_foot_support[ 1 ] > 0 ? 1 : 0 ) * 2
                           + ( lcm_cmd->pose_foot_support[ 0 ] > 0 ? 1 : 0 );
    if ( lcm_cmd->motionType == "transition" ) {
        cmd_tmp_.pos_des[ 2 ] = lcm_cmd->trans_height;
    }
    cmd_tmp_.duration   = lcm_cmd->duration / 2;
    cmd_tmp_.cmd_source = kCyberdogLcmCmd;

    motion_list_step_ = cmd_list_.size();  // List remaining actions
    motion_list_size_ = cmd_list_.size();  // The total list size

    cmd_list_.push( cmd_tmp_ );
    cyberdog_lcm_timer_.StartTimer();
}
