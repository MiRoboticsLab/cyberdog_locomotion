#ifndef SPI_HANDLER_FOR_IMU_HPP_
#define SPI_HANDLER_FOR_IMU_HPP_

#ifdef __cplusplus  // If this is a C++ compiler, use C linkage
extern "C" {
#endif

#include <linux/spi/spidev.h>

#ifdef __cplusplus  // If this is a C++ compiler, use C linkage
}
#endif
#include <string.h>

#include <lcm/lcm-cpp.hpp>

#include "parameters.hpp"

#define kMaxSizeOfPayload 54
#define kNumWordPerMessage ( kMaxSizeOfPayload + 11 )

/**
 * @brief Header of protocol packet of communicating with imu
 *
 */
typedef struct {
    uint8_t frame_header_first;
    uint8_t frame_header_second;
    uint8_t seq;
    uint8_t system_id;
    uint8_t board_id;
    uint8_t sensor_id;
    uint8_t frame_type;
    uint8_t command;
    uint8_t payload_length;
} ImuProtocolHeader;

/**
 * @brief Spi command message, used to send command to imu
 *
 */
typedef struct {
    ImuProtocolHeader header_cmd;
    uint8_t           payload[ kMaxSizeOfPayload ];
    uint8_t           sum_low_order;
    uint8_t           sum_high_order;
} SpiCommandForImu;

/**
 * @brief Spi data message, used to return imu data to controller
 *
 */
typedef struct SpiDataFromImu {
    ImuProtocolHeader header_data;
    uint8_t           payload[ kMaxSizeOfPayload ];
    uint8_t           sum_low_order;
    uint8_t           sum_high_order;
} SpiDataFromImu;

/**
 * @brief Data of imu
 *
 */
typedef struct {
    uint64_t timestamp;
    float    yaw;
    float    quaternions[ 4 ];
    float    acc[ 3 ];
    float    gyro[ 3 ];
    uint8_t  reserve[ 2 ];
} ImuNorminalData;

/**
 * @brief A handler class to control spi communication with imu.
 *
 */
class SpiHandlerForImu {
public:
    SpiHandlerForImu();
    ~SpiHandlerForImu();

    void InitializeSpi();

    void SetSpiCommandForImu( const ImuProtocolHeader& cmd );
    void GetImuDataBySpi( ImuNorminalData& data );

    void DriverRun();
    void FlushSpiBuffer();

    void PublishSpi( lcm::LCM& spi_lcm );

    void SetCommandZero();
    void ResetSpi();

    uint16_t CheckSum( uint8_t* message );

    int GetSpiSuccessIterations() {
        return spi_read_success_iteration_;
    }

private:
    SpiCommandForImu spi_command_;
    SpiDataFromImu   spi_data_;
    // hardware device
    int spi_fd_;
    int spi_driver_iteration_;
    int spi_read_success_iteration_;
};

#endif  // SPI_HANDLER_FOR_IMU_HPP_
