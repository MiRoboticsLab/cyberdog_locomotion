#include <iostream>

#include "rt/handle_bms_client.hpp"

/**
 * @brief Construct a new Handle Bms Client:: Handle Bms Client object
 *
 */
HandleBmsClient::HandleBmsClient() : bms_lcm_( GetLcmUrlWithPort( 7672, 255 ) ), motion_response_lcm_( GetLcmUrlWithPort( 7670, 255 ) ) {
    if ( !bms_lcm_.good() ) {
        std::cout << "bms_lcm_ init failed,return!" << std::endl;
        return;
    }
    if ( !motion_response_lcm_.good() ) {
        std::cout << "motion_response_lcm_ init failed,return!" << std::endl;
        return;
    }
    serial_ptr_   = std::make_shared< Serial >();
    app_off_flag_ = false;
    std::cout << "HandleBmsClient() is invoked!" << std::endl;
}

/**
 * @brief Destroy the Handle Bms Client:: Handle Bms Client object
 *
 */
HandleBmsClient::~HandleBmsClient() {
    std::cout << "~HandleBmsClient() is invoked!" << std::endl;
}

/**
 * @brief Subscribe bms lcm and handle the  message
 *
 * @param str not use
 * @return true
 * @return false
 */
bool HandleBmsClient::RunBmsClient( const std::string& str ) {
    ( void )str;
    bms_lcm_.subscribe( "bms_command", &HandleBmsClient::HandleBmsLcm, this );
    motion_response_lcm_.subscribe( "exec_response", &HandleBmsClient::HandleMotionLcm, this );
    bms_client_thread_ = std::thread( &HandleBmsClient::HandleBmsData, this );
    return true;
}

/**
 * @brief Handle the bms lcm message
 *
 */
void HandleBmsClient::HandleBmsData() {

    const std::string log_head( std::string( "[" ) + typeid( this ).name() + "] " );
    const std::string log_function_name( "[" + std::string( const_cast< char* >( __FUNCTION__ ) ) + "] " );
    const char*       serial_port = BMS_SERIAL_PORT;
    static int        iteration   = 1;

    RawBmsData        bms_received_data;
    BmsSendData       bms_sent_data;
    bms_response_lcmt bms_response_msg;

    memset( &bms_received_data, 0, sizeof( RawBmsData ) );
    memset( &bms_sent_data, 0, sizeof( BmsSendData ) );
    memset( &bms_response_msg, 0, sizeof( bms_response_lcmt ) );

    int fd = serial_ptr_->OpenPort( serial_port );
    if ( fd < 0 ) {
        std::cout << log_function_name << "Open Serial " << serial_port << " failed,and return!";
        return;
    }

#ifdef USE_NEW_BOARD
    serial_ptr_->SetPara( 6 );
#else
    serial_ptr_->SetPara( 7 );
#endif
    std::cout << log_function_name << "Open BMS serial succeed!" << std::endl;

    ConstructBmsSendRawData( bms_sent_data );

    while ( 1 ) {

        static int count = 0;

        // Filter out the previous part of the data
        if ( count == 0 ) {
            usleep( 1000 * 5 );
        }

        if ( count > 1e5 ) {
            count = 1;
        }

        // discard some data to flush buffer
        tcflush( serial_ptr_->fd(), TCIOFLUSH );
        if ( !GetBmsData( serial_ptr_, bms_received_data ) and bms_received_data.communication_error == false ) {
            std::cout << log_function_name << "get data from BMS failed,and break!" << std::endl;
            break;
        }

        bms_response_msg.batt_volt        = bms_received_data.bms_volt * 100;
        bms_response_msg.batt_curr        = bms_received_data.bms_curr * 200;
        bms_response_msg.batt_temp        = bms_received_data.bms_temp;
        bms_response_msg.batt_soc         = bms_received_data.bms_soc;
        bms_response_msg.status           = bms_received_data.bms_status;
        bms_response_msg.key              = bms_received_data.bms_power_supply;
        bms_response_msg.batt_health      = bms_received_data.bms_health;
        bms_response_msg.batt_loop_number = bms_received_data.bms_loop_number;

        static int charging_count = 0;
        if ( bms_response_msg.status & 0x01 ) {
            is_low_bat_ = false;
            charging_count++;
            if ( charging_count >= 10 )
                is_charging_ = true;
        }
        else {
            charging_count = 0;
            is_charging_   = false;
        }

        static int low_bat_count = 0;
#ifdef USE_NEW_BOARD
        if ( bms_response_msg.batt_volt <= 19500 && bms_response_msg.batt_soc <= 15 ) {
#else
        if ( bms_response_msg.batt_volt <= 18000 && bms_response_msg.batt_soc <= 15 ) {
#endif
            if ( low_bat_count > 30 ) {
                is_low_bat_ = true;
            }
            low_bat_count++;
        }
        else {
            is_low_bat_   = false;
            low_bat_count = 0;
        }

        if ( is_low_bat_ ) {
            bms_response_msg.status |= 0x02;
        }

        if ( bms_received_data.communication_error == true ) {
            bms_response_msg.powerBoard_status = 1;
        }
        else {
            bms_response_msg.powerBoard_status = 0;
        }

        if ( is_charging_ ) {
            bms_sent_data.power_supply &= 0XFE;
        }
        else {
            bms_sent_data.power_supply |= 0X01;
        }
        static bool wifi_power_flag = false;
        ;
        static int loco_power_count = 0;
        if ( check_power_start_ ) {
            if ( !loco_board_check_power_ || loco_power_count < 10 )
                loco_power_count++;
            if ( loco_power_count >= 10 && loco_power_count <= 40 ) {
                if ( loco_board_check_power_ ) {
                    bms_response_msg.locomotion_bits &= 0XFE;
                    // printf("excuted here!\n");
                }
            }
            else if ( loco_power_count > 40 ) {
                bms_response_msg.locomotion_bits |= 0X01;
                bms_response_msg.key |= 0x04;
                state_terminate_ = true;
            }
        }
        else {
            loco_power_count = 0;
            bms_response_msg.key &= 0xFB;
            bms_response_msg.locomotion_bits &= 0XFE;
            bms_response_msg.power_boards_bits &= 0X1F;
            if ( !is_charging_ )
                bms_sent_data.power_supply |= 0X01;
            if ( !wifi_power_flag )
                system( "ifconfig wlan0 up" );
            wifi_power_flag  = true;
            state_terminate_ = false;
        }
        static int bms_power_count = 0;
        if ( check_power_start_ && loco_board_check_power_ && !state_terminate_ ) {
            bms_sent_data.power_supply &= 0XFE;
            if ( !( motor_offline_ && bms_response_msg.key & 0X02 ) || bms_power_count < 20 )
                bms_power_count++;
            if ( bms_power_count >= 20 && bms_power_count <= 80 ) {
                if ( motor_offline_ && bms_response_msg.key & 0X02 ) {
                    bms_response_msg.power_boards_bits &= 0X1F;
                    bms_response_msg.key |= 0x04;
                    if ( wifi_power_flag ) {
                        system( "wifi_disconnect_ap_test" );
                        system( "ifconfig wlan0 down" );
                    }
                    wifi_power_flag = false;
                }
            }
            else if ( bms_power_count > 80 ) {
                bms_response_msg.power_boards_bits |= 0XE0;
                bms_response_msg.key |= 0x04;
            }
        }
        else {
            bms_power_count = 0;
        }

        if ( BMS_DEBUG_PRINTF ) {
            if ( iteration % 10 == 0 || ( charging_count >= 1 && charging_count <= 5 ) ) {
                // bms_msg publish
                bms_lcm_.publish( "bms_data", &bms_response_msg );
                // std::cout << log_function_name << "bms_data will be sent to all subscribers!" << std::endl;
            }

            if ( iteration % 80 == 0 ) {
                std::cout << log_function_name << "bms_volt: " << std::dec << ( int16_t )bms_response_msg.batt_volt << " mv; " <<std::endl;
                std::cout << log_function_name << "batt_curr: " << std::dec << ( int16_t )bms_response_msg.batt_curr << " ma; " <<std::endl;
                std::cout << log_function_name << "batt_temp: " << std::dec << ( int16_t )bms_response_msg.batt_temp << " c; " <<std::endl;
                std::cout << log_function_name << "batt_soc: " << std::dec << ( int )bms_received_data.bms_soc << " %; " <<std::endl;
                std::cout << log_function_name << "status: " << std::dec << ( int )bms_received_data.bms_status << " ; " <<std::endl;
                std::cout << log_function_name << "power_supply: " << std::dec << ( int )bms_received_data.bms_power_supply << " ; " <<std::endl;
                std::cout << log_function_name << "batt_health: " << std::dec << ( int )bms_received_data.bms_health << " ; " <<std::endl;
                std::cout << log_function_name << "batt_loop_number: " << std::dec << ( int16_t )bms_received_data.bms_loop_number << " ; " <<std::endl;
                std::cout << log_function_name << "powerBoard_status: " << std::dec << ( int )bms_response_msg.powerBoard_status << " ; " <<std::endl;
                iteration = 0;
            }
        }

        // Detect that the shutdown button is pressed, shutdown processing
        static int count_down = 0;
        if ( app_off_flag_ ) {
            count_down++;
            if ( count_down >= 8 / 8 ) {
                std::cout << log_function_name
                          << "the bms_response_msg.key = 1,shutdown button has been pressed and receive three shutdown infos,"
                             "the system will be closed!!!"
                          << std::endl;

                if ( bms_lcm_interval_.GetElapsedSeconds() > 5 ) {

                    bms_sent_data.power_supply = 0X07;

                    for ( size_t powerOffNumm = 0; powerOffNumm < 5; powerOffNumm++ ) {
                        SetBmsData( serial_ptr_, bms_sent_data );
                        std::cout << "the number of power off command to bms is: " << powerOffNumm + 1 << std::endl;
                        usleep( 100 );
                    }

                    std::cout << log_function_name << "motion control board begin to power off!" << std::endl;
                    system( "poweroff" );
                }
            }
        }

        // Periodically send serial port data to the BMS
        SetBmsData( serial_ptr_, bms_sent_data );

        usleep( 1000 * 20 );

        // bms_msg handle
        bms_lcm_.handleTimeout( 100 );
        // motion_msg handle
        motion_response_lcm_.handleTimeout( 100 );

        count++;
        iteration++;
    }

    serial_ptr_->ClosePort();
}

/**
 * @brief Construct data struct of sending to bms
 *
 * @param bmsSendData
 * @return true
 * @return false
 */
bool HandleBmsClient::ConstructBmsSendRawData( BmsSendData& bmsSendData ) {

    unsigned char send_data[ 4 ] = { 0X03, 0X05, 0X01, 0X03 };

    int16_t check_sum = 0;
    for ( int i = 0; i < 4; i++ ) {
        check_sum += send_data[ i ];
    }

    bmsSendData.data_len      = send_data[ 0 ];
    bmsSendData.data_type     = send_data[ 1 ];
    bmsSendData.charge_enable = send_data[ 2 ];
    bmsSendData.power_supply  = send_data[ 3 ];
    bmsSendData.sum_check     = check_sum;

    return true;
}

/**
 * @brief Handle bms lcm message
 *
 * @param buf
 * @param channel
 * @param msg
 */
void HandleBmsClient::HandleBmsLcm( const lcm::ReceiveBuffer* buf, const std::string& channel, const bms_request_lcmt* msg ) {
    BmsSendData bms_sent_data;
    ( void )buf;

    if ( channel == "bms_command" ) {
        bms_sent_data.charge_enable = msg->charge_enable;
        bms_sent_data.power_supply  = msg->power_supply | 0x03;

        if ( IsAppPowerOff( bms_sent_data ) ) {
            app_off_flag_ = true;
        }
        if ( msg->power_supply & 0x08 ) {
            check_power_start_ = true;
        }
        if ( msg->power_supply & 0x10 ) {
            check_power_start_ = false;
        }
        bms_lcm_interval_.StartTimer();
        std::cout << "[" << __FUNCTION__ << "] "
                  << "charge_enable: " << ( int )bms_sent_data.charge_enable <<std::endl;
        std::cout << "[" << __FUNCTION__ << "] "
                  << "power_supply: " << ( int )bms_sent_data.power_supply <<std::endl;

        SetBmsData( serial_ptr_, bms_sent_data );
    }

    return;
}

/**
 * @brief Judge if app sends command of power-off
 *
 * @param bms_sent_data BmsSendData
 * @param name
 * @return true
 * @return false
 */
bool HandleBmsClient::IsAppPowerOff( BmsSendData& bms_sent_data, const std::string& name ) {

    if ( bms_sent_data.power_supply & ( 1 << 2 ) ) {
        std::cout << "[" << __FUNCTION__ << "] "
                  << "name: " << name << ", receive the msg on \"power off\" from APP,sleep some time and begin to power off on motion control board!";
        // usleep(1000 * BMS_WAIT_APP_DELAY_TIME);
        return true;
    }

    return false;
}

/**
 * @brief Handle motion lcm message
 *
 * @param buf
 * @param channel
 * @param msg
 */
void HandleBmsClient::HandleMotionLcm( const lcm::ReceiveBuffer* buf, const std::string& channel, const motion_control_response_lcmt* msg ) {
    ( void )buf;
    if ( channel == "exec_response" ) {
        int pattern = 0;
        // static int i       = 0;
        // if(i++ % 10 == 0)
        //     printf("msg->pattern: %d\n", (int)msg->pattern);
        pattern = ( int )msg->pattern;
        if ( check_power_start_ && ( pattern == 2 || pattern == 1 ) ) {
            loco_board_check_power_ = true;
        }
        else {
            loco_board_check_power_ = false;
        }
        if ( IsMotorOffline( msg->error_flag.motor_error ) )
            motor_offline_ = true;
        else
            motor_offline_ = false;
    }
}

/**
 * @brief Judge if motors are offline
 *
 * @param motor_error
 * @return true
 * @return false
 */
bool HandleBmsClient::IsMotorOffline( const int32_t* motor_error ) {

    for ( size_t i = 0; i < 12; i++ ) {
        if ( ( motor_error[ i ] & 0X0001 ) != 1 )
            return false;
    }
    return true;
}