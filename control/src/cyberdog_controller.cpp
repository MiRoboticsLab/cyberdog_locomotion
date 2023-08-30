#include "cyberdog_controller.hpp"
#include "utilities/toolkit.hpp"

CyberdogController::CyberdogController() : RobotController() {
    motor_error_ = false;
    motor_warn_  = false;

    motor_mode_error_ = false;
    motor_over_heat_  = false;

    for ( int i = 0; i < 12; i++ )
        motor_mode_err_count_[ i ] = 0;
}

void CyberdogController::HandleLegErrorAndWarn( int32_t flags[ 12 ] ) {
    // resolve error code
    std::string leg_error_str = "";
    int32_t     flag_tmp      = 0;
    bool        spi_error     = false;
    bool        spi_warn      = false;

    bool spi_motor_mode_error = false;

    motor_over_heat_ = false;

    for ( int leg = 0; leg < 4; leg++ ) {
        for ( int motor = 0; motor < 3; ++motor ) {
            flag_tmp                  = flags[ leg * 3 + motor ];
            std::string motor_err_str = "";
            if ( !ResolveErrorFlags( motor, flag_tmp, motor_err_str ) ) {
                spi_error = ( spi_error || true );
            }
            if ( !ResolveWarnFlags( motor, flag_tmp, motor_err_str, motor_over_heat_ ) ) {
                spi_warn = ( spi_warn || true );
            }
#if ( ONBOARD_BUILD == 1 )
            // check motor mode
            if ( ( ( flag_tmp >> 30 ) & 0x03 ) != 0x02 && ( GetFsmMode() != static_cast< int >( FsmStateName::kOff ) ) && ( GetFsmMode() != static_cast< int >( FsmStateName::kPureDamper ) ) )
                motor_mode_err_count_[ leg * 3 + motor ]++;
            else
                motor_mode_err_count_[ leg * 3 + motor ] = 0;

            if ( motor_mode_err_count_[ leg * 3 + motor ] > 25 ) {
                spi_motor_mode_error = true;
                motor_err_str += "motor " + std::to_string( motor + 1 ) + ": error mode: " + std::to_string( ( flag_tmp >> 30 ) & 0x03 );
            }
#endif
            if ( !motor_err_str.empty() ) {
                leg_error_str += ( "leg " + std::to_string( leg ) + ": " + motor_err_str );
            }
        }
    }

    static int64_t error_iter = 0;
    if ( !leg_error_str.empty() ) {  // there is error or warning
        if ( error_iter++ % 2000 == 0 ) {
            std::cout << "[MOTOR ERROR&WARNING REPORT] " << leg_error_str;
            if ( spi_error ) {
                std::cout << ", ESTOP";
            }
            std::cout << std::endl;
        }
    }
    else {
        error_iter = 0;
    }
    if ( motor_error_ != spi_error && spi_error ) {  // print the exact disable dog motor err msg
        std::cout << "[MOTOR ERROR] Disable the dog, err msg: " << leg_error_str << std::endl;
    }
    motor_error_ = spi_error;  // if spi_error is occur, disable controller, else, enable
    motor_warn_  = spi_warn;

    motor_mode_error_ = spi_motor_mode_error;
}

int CyberdogController::GetLegMode( int32_t flags[ 12 ] ) {
    int32_t mode[ 12 ];

    for ( int i = 0; i < 12; ++i ) {
        mode[ i ] = flags[ i ];
        mode[ i ] = ( mode[ i ] >> 30 ) & 0x03;
        if ( mode[ i ] != mode[ 0 ] ) {
            return 4;
        }
    }

    return mode[ 0 ];
}

/**
 * @brief Initializes the Control FSM.
 *
 */
void CyberdogController::InitializeController() {

    // Initializes the Control FSM with all the required data
    control_fsm_ = new ControlFsm< float >( quadruped_, state_estimator_, leg_controller_, command_, control_parameters_, visualization_data_, &user_parameters_, &robot_current_state_ );
}

/**
 * @brief Calculate the commands for the leg controllers using the ControlFsm logic.
 *
 */
void CyberdogController::RunController() {
    static int edamp_iter            = 0;
    static int reset_motor_error_cnt = 0;
    if ( motor_error_ || motor_warn_ || motor_mode_error_ ) {
        if ( edamp_iter < 1560 )
            control_fsm_->SetOperatingMode( FsmOperatingMode::kEdamp );
        else {
            control_fsm_->SetOperatingMode( FsmOperatingMode::kEstop );
            // this->Estop();
            // clear error or warning automatically
            if ( motor_over_heat_ ) {
                if ( reset_motor_error_cnt >= 2000 && CheckMotorsReturnSaftyTemperature() ) {
                    leg_controller_->SetErrorClear( true );
                    std::cout << "Clear Motor Error !!!" << std::endl;
                }
            }
            else {
                if ( reset_motor_error_cnt >= 2000 ) {
                    leg_controller_->SetErrorClear( true );
                    if ( reset_motor_error_cnt % 1000 == 0 ) {
                        std::cout << "Clear Motor Error !!!" << std::endl;
                    }
                }
            }
            reset_motor_error_cnt++;
        }
        edamp_iter++;
    }
    else {
        edamp_iter            = 0;
        reset_motor_error_cnt = 0;
    }
    // Run the Control FSM code
    control_fsm_->RunFsm();
}
