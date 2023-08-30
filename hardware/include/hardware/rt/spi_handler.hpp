#ifndef SPI_HANDLER_HPP_
#define SPI_HANDLER_HPP_

#ifdef __cplusplus  // If this is a C++ compiler, use C linkage
extern "C" {
#endif

#include <linux/spi/spidev.h>

#ifdef __cplusplus  // If this is a C++ compiler, use C linkage
}
#endif

#include <lcm/lcm-cpp.hpp>

#include "header/lcm_type/spi_command_t.hpp"
#include "header/lcm_type/spi_data_t.hpp"
#include "parameters.hpp"
#include "parameters/calibrate_parameters.hpp"
#include "parameters/spi_parameters.hpp"
#include "sim_utilities/spine_board.hpp"

/**
 * @brief Spi command message, used to communicate with spine,
 * one spine connects with two legs.
 *
 */
typedef struct {
    float   q_des_abad[ 2 ];
    float   q_des_hip[ 2 ];
    float   q_des_knee[ 2 ];
    float   qd_des_abad[ 2 ];
    float   qd_des_hip[ 2 ];
    float   qd_des_knee[ 2 ];
    float   kp_abad[ 2 ];
    float   kp_hip[ 2 ];
    float   kp_knee[ 2 ];
    float   kd_abad[ 2 ];
    float   kd_hip[ 2 ];
    float   kd_knee[ 2 ];
    float   tau_abad_ff[ 2 ];
    float   tau_hip_ff[ 2 ];
    float   tau_knee_ff[ 2 ];
    int32_t flags[ 2 ];
    int32_t checksum;
} SpineCommand;

/**
 * @brief Spi data message, used to communicate with spine,
 * one spine connects with two legs.
 *
 */
typedef struct {
    float   q_abad[ 2 ];
    float   q_hip[ 2 ];
    float   q_knee[ 2 ];
    float   qd_abad[ 2 ];
    float   qd_hip[ 2 ];
    float   qd_knee[ 2 ];
    float   tau_abad[ 2 ];
    float   tau_hip[ 2 ];
    float   tau_knee[ 2 ];
    int16_t tmp_abad[ 2 ];
    int16_t tmp_hip[ 2 ];
    int16_t tmp_knee[ 2 ];
    uint8_t reserve[ 20 ];
    int32_t flags[ 6 ];
    int32_t checksum;
} SpineDate;

/**
 * @brief A handler class to control spi communication with motors
 *
 */
class SpiHandler {
public:
    SpiHandler();
    ~SpiHandler();

    void InitializeSpi( const RobotType& robot_type, const RobotAppearanceType& appearance_type = RobotAppearanceType::CURVED );
    void InitializeSpi();

    void SetSpiCommand( const SpiCommand& cmd );
    void GetSpiData( SpiData& data );

    void DriverRun();
    void FlushSpiBuffer();
    void PublishSpi( lcm::LCM& spi_lcm );

    void SetCommandZero();
    void ResetSpi();

    void SetMotorZeroFlag();
    void ResetMotorZeroFlag();

    void SpiDataToSpineData( const spi_data_t* data, SpineDate* spine_data, int leg_0 );
    void SpineCmdToSpiCmd( const SpineCommand* spine_cmd, spi_command_t* cmd, int leg_0 );

    // For reading sn or version of spine or motors
    void DriverRunTest( uint16_t idSum );
    void SetSpineCommand( const SpineCommand& cmd, int spine_board );
    void GetSpineData( SpineDate& data, int spine_board );

private:
    void SpiToSpine( spi_command_t& cmd, SpineCommand& spine_cmd, int leg_0 );
    void SpineToSpi( spi_data_t& data, SpineDate& spine_data, int leg_0 );

    uint32_t CheckSum( uint32_t* data, size_t len );
    bool     FindHipOffset( const Vec8< double >& hip_range, float q_hip, int leg );

    SpiParameters             spi_params_;
    JointsCalibrateParameters joints_params_;

    spi_command_t spi_command_;
    spi_data_t    spi_data_;
    int           iteration_;
    int           wrong_iteration_;
    int           spi_data_size_;
    RobotType     robot_type_;

    bool apply_hip_range_offset_[ 4 ];

    int spi_fd1_;
    int spi_fd2_;
    int spi_driver_iteration_;
    // For reading sn or version of spine or motors
    SpineDate    spine_data_[ 2 ];
    SpineCommand spine_command_[ 2 ];
};

#endif  // SPI_HANDLER_HPP_
