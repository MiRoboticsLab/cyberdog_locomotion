#ifndef DOG_TOOLKIT_HPP_
#define DOG_TOOLKIT_HPP_

#include <cassert>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "serial.h"
#include "utilities/toolkit.hpp"
#include "cpp_types.hpp"

#define MCU_GET_INFO_TYPE ( 0x04 )  // communicate type

#define MCU_ID_NUMBER_REQ ( 0x00 )
#define MCU_BOOT_VERSION_REQ ( 0x01 )
#define MCU_BOOT_COMPILE_DATE_REQ ( 0x02 )
#define MCU_BOOT_COMPILE_TIME_REQ ( 0x03 )

#define MCU_APP_VERSION_REQ ( 0x04 )
#define MCU_APP_GIT_VERSION_REQ ( 0x05 )
#define MCU_APP_COMPILE_DATE_REQ ( 0x06 )
#define MCU_APP_COMPILE_TIME_REQ ( 0x07 )
#define MCU_APP_NAME_REQ ( 0x08 )

#define MCU_ID_NUMBER_BUFF_LEN ( 12 )    // 12 bytes
#define MCU_INFO_RESULT_BUFF_LEN ( 18 )  // 12 bytes

#define IMU_NUM 0x00
#define SPINE1_NUM 0x01
#define SPINE2_NUM 0x02
#define LEG1M1_NUM 0x03
#define LEG1M2_NUM 0x04
#define LEG1M3_NUM 0x05
#define LEG0M1_NUM 0x06
#define LEG0M2_NUM 0x07
#define LEG0M3_NUM 0x08
#define LEG3M1_NUM 0x09
#define LEG3M2_NUM 0x0a
#define LEG3M3_NUM 0x0b
#define LEG2M1_NUM 0x0c
#define LEG2M2_NUM 0x0d
#define LEG2M3_NUM 0x0e

/**
 * @brief Raw IMU data send from MCU.
 *
 */
typedef struct {
    uint8_t  len;
    uint8_t  type;
    uint8_t  frameCount;
    uint32_t stampUs;
    uint16_t accRange;
    int16_t  acc_x;
    int16_t  acc_y;
    int16_t  acc_z;
    uint16_t gyroRange;
    int16_t  gyro_x;
    int16_t  gyro_y;
    int16_t  gyro_z;
    int16_t  temperature;
    int16_t  sum_check;
} RawImuData;

#ifdef BUILD_CYBERDOG2
#define USE_NEW_BOARD
#endif

typedef uint8_t SnType[ 32 ];
typedef uint8_t VersionType[ 32 ];

#define MAX_MSG_LEN 70

enum Bmi088FrameHead {
    kFrameHead1Upside   = 0x5A,
    kFrameHead2Upside   = 0xA5,
    kFrameHead1Downside = 0xAA,
    kFrameHead2Downside = 0x55,
};

enum Bmi088SysId {
    kSysIdNull = 0x00,
    kSysIdNx   = 0x01,
    kSysIdMr813,
    kSysIdPc,
    kSysIdImu = 0x10,
};

enum Bmi088SensorId {
    SensorIdMcu = 0x00,
    SensorIdAcc,
    SensorIdGyro,
    SensorIdMag,
    SensorIdImu,
};

enum Bmi088FrameType {
    kFrameTypeCtrl = 0x00,
    kFrameTypeData,
    kFrameTypeFile,
    kFrameTypeOta,
    kFrameTypeFactory,

    kFrameTypeMax,
};

enum Bmi088CmdCtrl {
    kCmdCtrlAck = 0x01,
    kCmdCtrlDataOutput,
    kCmdCtrlImuStatus,
    kCmdCtrlCalibration,
    kCmdCtrlBoardCheck,
    kCmdCtrlReadInfo,
    kCmdCtrlWriteInfo,

    kCmdCtrlMax,
};

enum Bmi088CmdData {
    kCmdDataDefaultOutput = 0x01,

    kCmdDataMax,
};

enum Bmi088CmdFile {
    kCmdFileNull,

    kCmdFileMax,
};

enum Bmi088CmdOta {
    kCmdOtaNull,

    kCmdOtaMax,
};

enum Bmi088CmdFactory {
    kCmdFactoryNull,

    kCmdFactoryMax,
};

enum Bmi088ImuStatus {
    kImuStatusError = 0x00,
    kImuStatusDataOutput,
    kImuStatusCalibration,
};

enum Bmi088ImuCalStatus {
    kImuCalError = 0x00,
    kImuCalReady,
    kImuCalCollectData,
    kImuCalComplete,
};

enum Bmi088ImuCalCollectStatus {
    kCollectStatusCollecting = 0x01,
    kCollectStatusCollectComplete,
    kCollectStatusCollected,
    kCollectStatusCollectFailed,
};

enum Bmi088ImuInfo {
    kImuInfoAccCalData = 0x01,
    kImuInfoGyroCalData,
    kImuInfoMagCalData,
    kImuInfoVersion,
    kImuInfoSn,
    kImuInfoUnsupportType,
};

enum Bmi088CtrlResult {
    kBmi088CtrlFailed = 0x00,
    kBmi088CtrlSuccess,
};

#pragma pack( 1 )
typedef struct {
    uint8_t frame_head1;
    uint8_t frame_head2;
    uint8_t seq;
    uint8_t sys_id;
    uint8_t board_id;
    uint8_t sensor_id;
    uint8_t frame_type;
    uint8_t cmd;
    uint8_t payload_len;
} Bmi088MsgHeadType;

typedef struct {
    uint8_t ack;
} Bmi088MsgCmdCtrlAck;

typedef struct {
    uint8_t imu_status;
    uint8_t sensor;
} Bmi088MsgCmdCtrlImuStatus;

typedef struct {
    uint8_t cal_status;
    uint8_t cal_sensor;
    union {
        struct {
            uint8_t collect_status;
            uint8_t axis_flag[ 6 ];
        } collect_info;

        struct {
            float   truth_value;
            uint8_t is_new_order;
        } collect_data;

        uint8_t cal_res;
    } cal_info;
} Bmi088MsgCmdCtrlCalImu;

typedef struct {
    uint8_t cal_status;
    uint8_t cal_res;
} Bmi088MsgCmdCtrlCalMag;

typedef struct {
    uint8_t board_check_res;
} Bmi088MsgCmdCtrlBoardCheck;

typedef struct {
    uint8_t info_type;
    union {
        struct {
            float acc_bias[ 3 ];
            float acc_R[ 3 ][ 3 ];
        } acc_cal_info;

        struct {
            float gyro_bias[ 3 ];
            float gyro_R[ 3 ][ 3 ];
        } gyro_cal_info;

        struct {
            float mag_bias[ 3 ];
            float mag_scale[ 3 ];
        } mag_cal_info;

        VersionType version;

        SnType sn;
    } info_data;
} Bmi088MsgCmdCtrlReadInfo;

typedef struct {
    union {
        uint8_t write_type;
        uint8_t write_res;
    } info_type;

    union {
        SnType sn;
    } info_data;
} Bmi088MsgCmdCtrlWriteInfo;

typedef struct {
    uint64_t timestamp;
    float    yaw;
    float    quaternions[ 4 ];
    float    acc[ 3 ];
    float    gyro[ 3 ];
} Bmi088MsgCmdData;
#pragma pack()

#define MAX_BMS_BUFF_LEN 23

typedef struct {
    int8_t  data_len;
    int8_t  data_type;
    int16_t bms_volt;
    int16_t bms_curr;
    int16_t bms_temp;
    int8_t  bms_soc;
    int8_t  bms_status;
    int8_t  bms_power_supply;
    int8_t  bms_health;
    int16_t bms_loop_number;
    int8_t  bms_fault;
    int16_t sum_check;
    bool    communication_error;
} RawBmsData;

typedef struct {
    int8_t  data_len;
    int8_t  data_type;
    int8_t  charge_enable;
    int8_t  power_supply;
    int16_t sum_check;
} BmsSendData;

bool ConvertBuff2ImuData( unsigned char* buff, RawImuData& data );

struct ImuResult {
    float acc[ 3 ];
    float gyro[ 3 ];
    float temperature;
    ImuResult() {
        for ( int i = 0; i < 3; i++ ) {
            acc[ i ]  = 0;
            gyro[ i ] = 0;
        }
    }
};

/**
 * @brief Print buff in Hex format.
 *
 */
template < typename T > void PrintHex( std::ostream& os, const T* buff, int size ) {
    int width = sizeof( T ) * 2;
    for ( int i = 0; i < size; i++ ) {
        os << "0x" << std::setfill( '0' ) << std::setw( width ) << std::hex << static_cast< int >( buff[ i ] ) << " ";
    }
    os << std::dec << std::endl;
}

// Parse serial data from socket
uint16_t SumCheck( uint8_t* message );
bool     CheckSum( uint8_t* message );
bool     FindNewestPacket( uint8_t* data_valid, uint16_t valid_len, uint8_t* packet_cmd, int& abandon_len );
bool     GetImuDataNotBlock( uint8_t* receive_message, Serial& serial_port );
bool     GetExternalImuData( uint8_t* receive_message, Serial& serial_port, bool isBlock );
bool     GetNewSerialData( uint8_t* receive_message, uint8_t& receive_cmd, uint8_t& receive_len, Serial& serial_port, bool isBlock );

/**
 * @brief Get one frame of IMU data frame socket.
 *
 */
bool GetImuData( RawImuData& imu_data, Serial& serial_port, ImuResult& result, bool isBlock = true );

/**
 * @brief Wait for IMU burn to desire temperature, if cannot get the temp in certain time, return false.
 *
 */
bool WaitImuTemp( Serial& imu_port );

bool ParseBmsData( unsigned char* buff, RawBmsData& bms_data );

bool GetBmsData( const std::shared_ptr< Serial >& serial_ptr, RawBmsData& bms_data );

bool SetBmsData( const std::shared_ptr< Serial >& serial_ptr, const BmsSendData& bms_data );

/**
 * @brief Get old MCU Info.
 */
std::vector< char > GetMcuInfo( Serial& mcu_port, unsigned char mcu_num, unsigned char data_type );

/**
 * @brief Get New MCU Info.
 */
std::vector< char > GetMcuInfo( Serial& mcu_port, uint8_t cmd, uint8_t dataLen, uint8_t dataType );

/**
 * @brief Convert the 96 bits MCU ID from vector to 2 int.
 *
 * @param ids source data
 * @param id_high save the low 64 bits
 * @param id_low save the high 32 bits
 * @return true if convert succeed, return true
 * @return false if convert failed, return false
 */
bool Id2Int( std::vector< char >& ids, int64_t& id_high, int64_t& id_low );

/**
 * @brief Convert the char sequence to string with fixed format.
 *
 * @param src source data
 * @return std::string string with fixed format
 */
std::string Conversion( const char* src );

/**
 * @brief Get the Board Id object
 *
 * @return int
 */
inline int GetBoardId() {
    int           board_id = 1;
    std::string   board_id_data;
    std::ifstream board_id_file( "/sys/firmware/devicetree/base/soc@03000000/board_id" );
    if ( board_id_file.is_open() ) {
        getline( board_id_file, board_id_data );
        board_id = std::atoi( board_id_data.c_str() );
    }
    return board_id;
}

/**
 * @brief Get the Robot Appearance Type object
 *
 * @param token
 * @return RobotAppearanceType
 */
inline RobotAppearanceType GetRobotAppearanceType( const int& board_id ) {
    RobotAppearanceType type;

    FILE*       stream;
    char        buff[ 17 ] = { '\0' };
    std::string name       = "ptype";
    std::string cmd_params = "mikey get " + name;  // mikey get name
    stream                 = popen( cmd_params.c_str(), "r" );
    fread( buff, 1, 16, stream );
    pclose( stream );
    std::string token = buff;

    if ( board_id < 4 ) {
        if ( strncmp( token.c_str(), "angular", 7 ) == 0 )
            type = RobotAppearanceType::ANGULAR;
        else
            type = RobotAppearanceType::CURVED;
        std::cout << "[ZeroTool] Board ID: " << board_id << ", Robot appearance: " << ( ( int )type ? "ANGULAR" : "CURVED" ) << std::endl;
    }
    else {
        type = RobotAppearanceType::ANGULAR;
        std::cout << "[ZeroTool] Board ID: " << board_id << ", Robot appearance: Default[ANGULAR]" << std::endl;
    }
    return type;
}
#endif  // DOG_TOOLKIT_HPP_
