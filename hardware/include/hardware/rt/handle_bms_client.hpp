#ifndef HANDLE_BMS_CLIENT_HPP_
#define HANDLE_BMS_CLIENT_HPP_

#include <lcm/lcm-cpp.hpp>
#include <string>
#include <thread>

#include "utilities/dog_toolkit.hpp"
#include "utilities/timer.hpp"
#include "control_flags.hpp"
#include "header/lcm_type/bms_request_lcmt.hpp"
#include "header/lcm_type/bms_response_lcmt.hpp"
#include "header/lcm_type/motion_control_response_lcmt.hpp"

#define BMS_SHM_NAME "bms_abort"
#define BMS_SERIAL_PORT "/dev/ttyS3"
#define BMS_ABORT_REQUEST ( 0xdead )
#define BMS_DEBUG_PRINTF 1  // debug printf control
#define BMS_ABORT_FINISH ( 0xffff )
#define BMS_SHM_SIZE 2

/**
 * @brief Handle bms for new board of cyberdog
 *
 */
class HandleBmsClient {
public:
    HandleBmsClient();
    virtual ~HandleBmsClient();
    bool RunBmsClient( const std::string& str = " " );

private:
    bool ConstructBmsSendRawData( BmsSendData& bmsRawData );
    void HandleBmsLcm( const lcm::ReceiveBuffer* buf, const std::string& channel, const bms_request_lcmt* msg );
    bool IsAppPowerOff( BmsSendData& bmsSendRawData, const std::string& name = "" );
    void HandleMotionLcm( const lcm::ReceiveBuffer* buf, const std::string& channel, const motion_control_response_lcmt* msg );
    bool IsMotorOffline( const int32_t* motor_error );
    void HandleBmsData();

private:
    lcm::LCM                  bms_lcm_;
    std::shared_ptr< Serial > serial_ptr_;
    std::thread               bms_client_thread_;
    bool                      app_off_flag_;
    Timer                     bms_lcm_interval_;
    bool                      is_low_bat_  = false;
    bool                      is_charging_ = false;
    // power-saving mode
    lcm::LCM motion_response_lcm_;
    bool     check_power_start_      = false;
    bool     loco_board_check_power_ = false;
    bool     bms_board_check_power_  = false;
    bool     motor_offline_          = false;
    bool     state_terminate_        = false;
};

#endif  // HANDLE_BMS_CLIENT_HPP_
