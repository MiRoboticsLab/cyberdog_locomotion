#include <iostream>

#include "rt/handle_new_board_bms_client.hpp"

/**
 * @brief Construct a new Handle New Bms Client:: Handle New Bms Client object
 *
 */
HandleNewBmsClient::HandleNewBmsClient() : bms_lcm_( GetLcmUrlWithPort( 7672, 255 ) ) {
    serial_ptr_ = std::make_shared< Serial >();
    std::cout << "HandleNewBmsClient() is invoked!" << std::endl;
    if ( !bms_lcm_.good() ) {
        std::cout << "bms lcm init failed,return!" << std::endl;
        return;
    }
}

/**
 * @brief Destroy the Handle New Bms Client:: Handle New Bms Client object
 *
 */
HandleNewBmsClient::~HandleNewBmsClient() {
    std::cout << "~HandleNewBmsClient() is invoked!" << std::endl;
}

/**
 * @brief Subscribe bms lcm and handle the  message
 *
 * @param str not use
 * @return true
 * @return false
 */
bool HandleNewBmsClient::RunBmsClient( const std::string& str ) {
    ( void )str;
    bms_lcm_.subscribe( "POWER_CMD", &HandleNewBmsClient::HandleBmsLcm, this );
    bms_client_thread_ = std::thread( &HandleNewBmsClient::HandleBmsData, this );
    return true;
}

/**
 * @brief Handle the bms lcm message
 *
 */
void HandleNewBmsClient::HandleBmsData() {

    const std::string log_head( std::string( "[" ) + typeid( this ).name() + "] " );
    const std::string log_function_name( "[" + std::string( const_cast< char* >( __FUNCTION__ ) ) + "] " );
    const char*       serial_port = BMS_SERIAL_PORT;
    static int        iteration   = 1;

    RawBmsData  bms_received_data;
    BmsSendData bms_sent_data;

    memset( &bms_received_data, 0, sizeof( RawBmsData ) );
    memset( &bms_sent_data, 0, sizeof( BmsSendData ) );

    int fd = serial_ptr_->OpenPort( serial_port );
    if ( fd < 0 ) {
        std::cout << log_function_name << "Open Serial " << serial_port << " failed,and return!";
        return;
    }

    serial_ptr_->SetPara( 6 );

    std::cout << log_function_name << "Open BMS serial succeed!" << std::endl;

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

        int16_t batt_volt = bms_received_data.bms_volt * 100;
        int16_t batt_curr = bms_received_data.bms_curr * 200;
        int16_t batt_temp = bms_received_data.bms_temp;
        battery_soc_         = bms_received_data.bms_soc;
        bms_status_       = bms_received_data.bms_status;

        // int8_t  key              = bms_received_data.bms_power_supply;
        // int8_t  batt_health      = bms_received_data.bms_health;
        // int16_t batt_loop_number = bms_received_data.bms_loop_number;

        static int charging_count = 0;
        if ( bms_status_ & 0x01 ) {
            is_low_bat_ = false;
            charging_count++;
            if ( charging_count >= 10 )
                is_charging_ = true;
        }
        else {
            charging_count = 0;
            is_charging_   = false;
        }

        static int low_bat_count  = 0;
        static int high_bat_count = 0;

        if ( is_low_bat_ == false ) {
            if ( ( batt_volt >= 10000 && batt_volt <= 19500 ) && battery_soc_ <= 15 ) {
                if ( low_bat_count > 30 ) {
                    is_low_bat_ = true;
                }
                low_bat_count++;
            }
            else {
                is_low_bat_   = false;
                low_bat_count = 0;
            }
        }
        else {
            if ( batt_volt > 19900 ) {
                if ( high_bat_count > 30 ) {
                    is_low_bat_ = false;
                }
                high_bat_count++;
            }
            else {
                is_low_bat_    = true;
                high_bat_count = 0;
            }
        }

        if ( is_low_bat_ ) {
            bms_status_ |= 0x02;
        }
        int8_t power_board_status;
        if ( bms_received_data.communication_error == true ) {
            power_board_status = 1;
        }
        else {
            power_board_status = 0;
        }

        if ( is_charging_ ) {
            bms_sent_data.power_supply &= 0XFE;
        }
        else {
            bms_sent_data.power_supply |= 0X01;
        }

        if ( iteration % 80 == 0 ) {
            std::cout << log_function_name << "bms_volt: " << std::dec << ( int16_t )batt_volt << " mv; " <<std::endl;
            std::cout << log_function_name << "batt_curr: " << std::dec << ( int16_t )batt_curr << " ma; " <<std::endl;
            std::cout << log_function_name << "batt_temp: " << std::dec << ( int16_t )batt_temp << " c; " <<std::endl;
            std::cout << log_function_name << "batt_soc: " << std::dec << ( int )bms_received_data.bms_soc << " %; " <<std::endl;
            std::cout << log_function_name << "status: " << std::dec << ( int )bms_received_data.bms_status << " ; " <<std::endl;
            std::cout << log_function_name << "power_supply: " << std::dec << ( int )bms_received_data.bms_power_supply << " ; " <<std::endl;
            std::cout << log_function_name << "batt_health: " << std::dec << ( int )bms_received_data.bms_health << " ; " <<std::endl;
            std::cout << log_function_name << "batt_loop_number: " << std::dec << ( int16_t )bms_received_data.bms_loop_number << " ; " <<std::endl;
            std::cout << log_function_name << "power_board_status: " << std::dec << ( int )power_board_status << " ; " <<std::endl;
            std::cout << log_function_name << "bms_fault: " << std::dec << ( int16_t )bms_received_data.bms_fault << " ; " <<std::endl;
            iteration = 0;
        }

        if ( low_power_update_ ) {
            SetBmsData( serial_ptr_, bms_sent_data );
            new_bms_response_lcmt bms_response;
            memset( &bms_response, 0, sizeof( new_bms_response_lcmt ) );
            bms_response.lowpower_ack = low_power_flag_;
            bms_lcm_.publish( "POWER_STATUS", &bms_response );
        }

        if ( low_power_enable_ == nullptr || ( low_power_enable_ != nullptr && *low_power_enable_ ) ) {
            if ( low_power_flag_ == 0x01 )
                bms_sent_data.power_supply |= 0x08;
            else if ( low_power_flag_ == 0x02 ) {
                bms_sent_data.power_supply &= 0xFE;
                if ( low_power_update_ ) {
                    system( "echo mem > /sys/power/state" );
                }
            }
            else if ( low_power_flag_ == 0x03 ) {
                bms_sent_data.power_supply |= 0x04;
                if ( low_power_update_ )
                    system( "sync" );
            }
            low_power_update_ = false;
        }
        bms_lcm_.handleTimeout( 100 );

        usleep( 1000 * 20 );

        count++;
        iteration++;
    }

    serial_ptr_->ClosePort();
}

/**
 * @brief Handle low_power_flag of bms lcm message
 *
 * @param buf
 * @param channel
 * @param msg lcm message
 */
void HandleNewBmsClient::HandleBmsLcm( const lcm::ReceiveBuffer* buf, const std::string& channel, const new_bms_request_lcmt* msg ) {
    ( void )buf;
    printf( "%s\n", channel.c_str() );
    low_power_flag_   = msg->lowpower_flag;
    low_power_update_ = true;
    return;
}