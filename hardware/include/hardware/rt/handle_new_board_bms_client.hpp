#ifndef HANDLE_NEW_BOARD_BMS_CLIENT_HPP_
#define HANDLE_NEW_BOARD_BMS_CLIENT_HPP_

#include <lcm/lcm-cpp.hpp>
#include <string>
#include <thread>

#include "utilities/dog_toolkit.hpp"
#include "utilities/timer.hpp"
#include "control_flags.hpp"
#include "header/lcm_type/new_bms_request_lcmt.hpp"
#include "header/lcm_type/new_bms_response_lcmt.hpp"

#define BMS_SHM_NAME "bms_abort"
#define BMS_SERIAL_PORT "/dev/ttyS3"
#define BMS_ABORT_REQUEST ( 0xdead )
#define BMS_ABORT_FINISH ( 0xffff )
#define BMS_SHM_SIZE 2

/**
 * @brief Handle bms for new board of cyberdog2
 *
 */
class HandleNewBmsClient {
public:
    HandleNewBmsClient();
    virtual ~HandleNewBmsClient();
    bool RunBmsClient( const std::string& str = " " );
    void SetLowPowerEnable( bool* enable ) {
        low_power_enable_ = enable;
    }
    int8_t* GetBmsStatus() {
        return &bms_status_;
    }

    int8_t* GetBatterySoc() {
        return &battery_soc_;
    }

private:
    void HandleBmsLcm( const lcm::ReceiveBuffer* buf, const std::string& channel, const new_bms_request_lcmt* msg );
    void HandleBmsData();

    std::shared_ptr< Serial > serial_ptr_;
    std::thread               bms_client_thread_;
    bool                      is_low_bat_  = false;
    bool                      is_charging_ = false;

    int8_t bms_status_ = 0;
    int8_t battery_soc_   = 0;

    lcm::LCM bms_lcm_;
    int8_t   low_power_flag_   = 0;
    bool     low_power_update_ = false;
    bool*    low_power_enable_ = nullptr;
};

#endif  // HANDLE_NEW_BOARD_BMS_CLIENT_HPP_
