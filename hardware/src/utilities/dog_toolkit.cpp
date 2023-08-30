#include "utilities/dog_toolkit.hpp"
#include "utilities/timer.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#define DEBUG_PRINTF 1

#define DESIRE_TEMP 50
#define MAX_ITER 100  // wait for 30s
#define GET16( ptr ) ( *( ( ptr ) + 1 ) << 8 | *( ( ptr ) + 0 ) )
// TODO: wrong code!
#define GET32( ptr ) ( *( ( ptr ) + 3 ) << 24 | *( ( ptr ) + 2 ) << 16 | *( ( ptr ) + 1 ) << 8 | *( ( ptr ) + 0 ) )

#define MCU_INFO_REQ_BUFF_LEN ( 2 + 1 + 3 + 2 )
#define MCU_INFO_REP_BUFF_LEN ( 1 + 19 )

bool ConvertBuff2ImuData( unsigned char* buff, RawImuData& data ) {
    data.len         = buff[ 0 ];
    data.type        = buff[ 1 ];
    data.frameCount  = buff[ 2 ];
    data.stampUs     = GET32( buff + 3 );
    data.accRange    = GET16( buff + 7 );
    data.acc_x       = GET16( buff + 9 );
    data.acc_y       = GET16( buff + 11 );
    data.acc_z       = GET16( buff + 13 );
    data.gyroRange   = GET16( buff + 15 );
    data.gyro_x      = GET16( buff + 17 );
    data.gyro_y      = GET16( buff + 19 );
    data.gyro_z      = GET16( buff + 21 );
    data.temperature = GET16( buff + 23 );
    data.sum_check   = GET16( buff + 25 );

    int16_t checksum = 0;
    for ( int i = 0; i < 25; i++ ) {
        checksum += buff[ i ];
    }
    if ( checksum != data.sum_check ) {
        fprintf( stderr, "[ERROR] IMU receive checksum is wrong, expect %#06X, got %#06X\n", checksum, data.sum_check );
        return false;
    }
    return true;
}

uint16_t SumCheck( uint8_t* message ) {
    Bmi088MsgHeadType* msg_head = ( Bmi088MsgHeadType* )message;
    uint16_t           AC       = 0;
    uint8_t            len      = msg_head->payload_len + sizeof( Bmi088MsgHeadType );

    while ( len-- )
        AC += ( *message++ );

    return AC;
}

bool CheckSum( uint8_t* message ) {
    Bmi088MsgHeadType* msg_head = ( Bmi088MsgHeadType* )message;
    uint16_t           AC       = 0;
    uint8_t            len      = msg_head->payload_len + sizeof( Bmi088MsgHeadType );
    // std::cout << "Len:" << ( uint16_t )len << std::endl;

    while ( len-- )
        AC += ( *message++ );

    uint16_t msg_AC = ( *( message + 1 ) << 8 ) | *( message );
    // std::cout << "msg_AC: " << msg_AC << std::endl;
    // std::cout << "AC: " << AC << std::endl;
    return AC == msg_AC;
}

bool GetImuDataNotBlock( uint8_t* receive_message, Serial& serial_port ) {
    static const int     packet_len              = 63;
    static const int     max_size                = packet_len * 3;
    static unsigned char data_read[ max_size ]   = { 0 };
    static unsigned char data_stream[ max_size ] = { 0 };
    static int           read_len                = 0;
    static int           remain_len              = 0;
    static int           abandon_len             = 0;
    static int           valid_len               = 0;
    static bool          findNewPacket;
    // static Timer tt;
    // tt.StartTimer();
    // read imu data by serial
    read_len = serial_port.ReadData( data_read, max_size, false );
    // if ( tt.GetElapsedMilliseconds() > 2 )
    //     std::cout<<"[check read cost]  "<<tt.GetElapsedMilliseconds()<<std::endl;
    // combine data
    memcpy( &data_stream[ remain_len ], data_read, read_len );
    valid_len = read_len + remain_len;
    if ( read_len != 126 && read_len != 63 && read_len != 189 && read_len != 0 )
        std::cout << "[imu read]  " << read_len << "  " << remain_len << "  " << valid_len << std::endl;
    // get newest packet
    findNewPacket = FindNewestPacket( data_stream, valid_len, receive_message, abandon_len );
    // abandon used data
    remain_len = valid_len - abandon_len;
    if ( remain_len > 0 ) {
        memcpy( data_stream, &data_stream[ abandon_len ], remain_len );
    }
    return findNewPacket;
}
bool FindNewestPacket( uint8_t* data_valid, uint16_t valid_len, uint8_t* packet_cmd, int& abandon_len ) {
    static std::vector< uint16_t > header_id;
    static std::vector< uint16_t > packet_start_id;
    header_id.clear();
    packet_start_id.clear();
    static uint16_t    header_type_len = sizeof( Bmi088MsgHeadType );
    Bmi088MsgHeadType* msg_head;
    static uint8_t     buffer[ MAX_MSG_LEN ] = { 0 };
    // find header of all integral packet
    for ( uint16_t i = 0; i < valid_len - 1; i++ ) {
        if ( data_valid[ i ] == 0x5a && data_valid[ i + 1 ] == 0xa5 && i + header_type_len - 1 < valid_len ) {
            header_id.push_back( i );
            memcpy( buffer, &data_valid[ i ], header_type_len );
            msg_head        = ( Bmi088MsgHeadType* )buffer;
            uint8_t msg_Len = msg_head->payload_len;
            if ( i + header_type_len + msg_Len + 1 < valid_len ) {
                packet_start_id.push_back( i );
            }
            i += header_type_len + msg_Len + 1;
        }
    }
    if ( packet_start_id.empty() ) {
        std::cout << "[IMU Serial] there are not integral packet !!!" << std::endl;
        abandon_len = 0;
        std::cout << "[check abandon len 0 ]  " << valid_len << "   " << abandon_len << std::endl;
        return false;
    }
    else {
        abandon_len = packet_start_id.at( packet_start_id.size() - 1 );
        if ( abandon_len != 0 && abandon_len != 63 && abandon_len != 126 )
            std::cout << "[check abandon len 1 ]  " << valid_len << "   " << abandon_len << std::endl;
        memcpy( buffer, &data_valid[ abandon_len ], header_type_len );
        msg_head              = ( Bmi088MsgHeadType* )buffer;
        uint8_t msg_FrameType = msg_head->frame_type;
        uint8_t msg_CMD       = msg_head->cmd;
        uint8_t msg_Len       = msg_head->payload_len;
        if ( msg_FrameType == 0x01 && msg_CMD == 0x01 ) {
            // if checksum is wrong, donot update imu data
            if ( !CheckSum( &data_valid[ abandon_len ] ) ) {
                std::cerr << "[IMU Serial] checksum is wrong " << std::endl;
                abandon_len = abandon_len + sizeof( Bmi088MsgHeadType ) + msg_Len + 2;
                return false;
            }
            else {
                memcpy( packet_cmd, &data_valid[ abandon_len + sizeof( Bmi088MsgHeadType ) ], msg_Len );
                abandon_len = abandon_len + sizeof( Bmi088MsgHeadType ) + msg_Len + 2;
                return true;
            }
        }
        else {
            std::cerr << "[IMU Serial] type is wrong " << ( int )msg_FrameType << ", " << ( int )msg_CMD << std::endl;
            abandon_len = abandon_len + sizeof( Bmi088MsgHeadType ) + msg_Len + 2;
            return false;
        }
    }
}

bool GetExternalImuData( uint8_t* receive_message, Serial& serial_port, bool isBlock ) {
    unsigned char buff_0[ MAX_MSG_LEN ] = { 0 };
    unsigned char head1, head2;

    int err_num = 0;
    while ( true ) {  // wait for the data head
        if ( err_num >= 1000 && err_num % 1000 == 0 ) {
            std::cerr << "[IMU Serial] too many err char when waiting for header " << ( int )head1 << ", " << ( int )head2 << std::endl;

            if ( isBlock == false ) {
                return false;
            }
        }
        serial_port.ReadData( &head1, sizeof( unsigned char ), isBlock );
        if ( head1 == 0x5a ) {
            serial_port.ReadData( &head2, sizeof( unsigned char ), isBlock );
            if ( head2 == 0xa5 ) {
                buff_0[ 0 ] = 0x5a;
                buff_0[ 1 ] = 0xa5;
                break;
            }
            else {
                err_num += 1;
            }
        }
        else {
            err_num += 1;
        }

        usleep( 1 );
    }

    serial_port.ReadData( ( unsigned char* )( &buff_0[ 2 ] ), sizeof( Bmi088MsgHeadType ) - 2, true );
    Bmi088MsgHeadType* msg_head = ( Bmi088MsgHeadType* )buff_0;

    uint8_t msg_FrameType = msg_head->frame_type;
    uint8_t msg_CMD       = msg_head->cmd;
    uint8_t msg_Len       = msg_head->payload_len;

    if ( msg_FrameType == 0x01 && msg_CMD == 0x01 && msg_Len == sizeof( Bmi088MsgCmdData ) ) {
        serial_port.ReadData( ( unsigned char* )&buff_0[ sizeof( Bmi088MsgHeadType ) ], msg_Len + 2, true );
        // if checksum is wrong, donot update imu data
        if ( !CheckSum( buff_0 ) ) {
            std::cerr << "[IMU Serial] checksum is wrong " << std::endl;
            return false;
        }
    }
    else {
        std::cerr << "[IMU Serial] type is wrong " << ( int )msg_FrameType << ", " << ( int )msg_CMD << std::endl;
        return false;
    }
    memcpy( receive_message, &buff_0[ sizeof( Bmi088MsgHeadType ) ], msg_Len );
    return true;
}

bool GetNewSerialData( uint8_t* receive_message, uint8_t& receive_cmd, uint8_t& receive_len, Serial& serial_port, bool isBlock ) {
    unsigned char buff_0[ MAX_MSG_LEN ] = { 0 };
    unsigned char head1, head2;

    int err_num = 0;
    while ( true ) {  // wait for the data head
        if ( err_num >= 40 && err_num % 40 == 0 ) {
            std::cerr << "[IMU Serial] too many err char when waiting for header " << ( int )head1 << ", " << ( int )head2 << std::endl;

            if ( isBlock == false ) {
                return false;
            }
        }
        serial_port.ReadData( &head1, sizeof( unsigned char ), isBlock );
        if ( head1 == 0x5a ) {
            serial_port.ReadData( &head2, sizeof( unsigned char ), isBlock );
            if ( head2 == 0xa5 ) {
                buff_0[ 0 ] = 0x5a;
                buff_0[ 1 ] = 0xa5;
                break;
            }
            else {
                err_num += 1;
            }
        }
        else {
            err_num += 1;
        }

        usleep( 1 );
    }

    serial_port.ReadData( ( unsigned char* )( &buff_0[ 2 ] ), sizeof( Bmi088MsgHeadType ) - 2, true );
    Bmi088MsgHeadType* msg_head = ( Bmi088MsgHeadType* )buff_0;

    uint8_t msg_FrameType = msg_head->frame_type;
    uint8_t msg_CMD       = msg_head->cmd;
    uint8_t msg_Len       = msg_head->payload_len;

    serial_port.ReadData( ( unsigned char* )&buff_0[ sizeof( Bmi088MsgHeadType ) ], msg_Len + 2, true );

    if ( msg_FrameType <= 0x04 && msg_CMD <= 0x07 ) {
        // if checksum is wrong, donot update imu data
        if ( !CheckSum( buff_0 ) ) {
            std::cerr << "[Serial] checksum is wrong " << std::endl;
            return false;
        }
    }
    else {
        std::cerr << "[Serial] type is wrong " << std::endl;
        return false;
    }
    receive_cmd = msg_CMD;
    receive_len = msg_Len;
    memcpy( receive_message, &buff_0[ sizeof( Bmi088MsgHeadType ) ], msg_Len );
    return true;
}

bool GetImuData( RawImuData& imu_data, Serial& serial_port, ImuResult& result, bool isBlock ) {
    unsigned char buff[ 27 ];
    unsigned char head1, head2;

    int err_num = 0;
    while ( true ) {  // wait for the data head
        if ( err_num >= 40 && err_num % 40 == 0 ) {
            std::cerr << "[IMU Serial] too many err char when waiting for header " << ( int )head1 << ", " << ( int )head2 << std::endl;

            if ( isBlock == false ) {
                return false;
            }
        }
        serial_port.ReadData( &head1, sizeof( unsigned char ), isBlock );
        if ( head1 == 0x5a ) {
            serial_port.ReadData( &head2, sizeof( unsigned char ), isBlock );
            if ( head2 == 0xa5 ) {
                break;
            }
            else {
                err_num += 1;
            }
        }
        else {
            err_num += 1;
        }

        usleep( 1 );
    }

    unsigned char& len = buff[ 0 ];
    serial_port.ReadData( ( unsigned char* )( &len ), sizeof( len ), true );

    int real_len = std::min( ( int )len, 200 ) + 2;
    serial_port.ReadData( ( unsigned char* )( buff + 1 ), real_len, true );
    unsigned char& type = buff[ 1 ];

    if ( type == 0x01 ) {
        // if checksum is wrong, donot update imu data
        if ( !ConvertBuff2ImuData( buff, imu_data ) ) {
            return false;
        }
    }
    else {  // save frame data directly into imu_data, donot analyze
        memcpy( ( char* )&imu_data, buff, sizeof( buff ) );
        std::cout << "save frame data directly into imu_data, do not analyze! " << std::endl;
        return false;
    }

    float accRange  = imu_data.accRange / 1000. / 32768.;
    float gyroRange = imu_data.gyroRange * M_PI / 180. / 32768.;

    float ax = imu_data.acc_x * accRange;
    float ay = imu_data.acc_y * accRange;
    float az = imu_data.acc_z * accRange;
    float gx = imu_data.gyro_x * gyroRange;
    float gy = imu_data.gyro_y * gyroRange;
    float gz = imu_data.gyro_z * gyroRange;

    result.acc[ 0 ]    = ax;
    result.acc[ 1 ]    = ay;
    result.acc[ 2 ]    = az;
    result.gyro[ 0 ]   = gx;
    result.gyro[ 1 ]   = gy;
    result.gyro[ 2 ]   = gz;
    result.temperature = imu_data.temperature * 0.1;
    return true;
}

bool WaitImuTemp( Serial& imu_port ) {
    RawImuData imu_data;
    ImuResult  imu_res;
    // discard some data to flush buffer
    tcflush( imu_port.fd(), TCIOFLUSH );

    int  temp_ready_count = 0;
    int  iter             = 0;
    bool temp_ready       = false;
    while ( true ) {
        tcflush( imu_port.fd(), TCIOFLUSH );
        GetImuData( imu_data, imu_port, imu_res );
        std::cout << "temp: " << imu_res.temperature << std::endl;

        if ( fabs( imu_res.temperature - DESIRE_TEMP ) < 2. ) {
            temp_ready_count++;
            if ( temp_ready_count >= 6 ) {  // if the temp in desired temperature for 1.8 s
                temp_ready = true;
                break;
            }
        }
        else {
            temp_ready_count = 0;
        }

        iter++;
        if ( iter > MAX_ITER ) {
            break;
        }

        usleep( 1000 * 300 );  // wait 300ms
    }
    return temp_ready;
}

#ifndef USE_NEW_BOARD
bool ParseBmsData( unsigned char* buff, RawBmsData& bms_data ) {
    bms_data.data_len         = buff[ 0 ];
    bms_data.data_type        = buff[ 1 ];
    bms_data.bms_volt         = GET16( buff + 2 );
    bms_data.bms_curr         = GET16( buff + 4 );
    bms_data.bms_temp         = GET16( buff + 6 );
    bms_data.bms_soc          = buff[ 8 ];
    bms_data.bms_status       = buff[ 9 ];
    bms_data.bms_power_supply = buff[ 10 ];
    bms_data.bms_health       = buff[ 11 ];
    bms_data.bms_loop_number  = GET16( buff + 12 );
    bms_data.sum_check        = GET16( buff + 14 );

    int16_t checksum = 0;
    for ( int i = 0; i < 14; i++ ) {
        checksum += buff[ i ];
    }
    if ( checksum != bms_data.sum_check ) {
        fprintf( stderr, "[ERROR] BMS receive checksum is wrong, expect %#06X, got %#06X\n", checksum, bms_data.sum_check );
        return false;
    }
    return true;
}

bool GetBmsData( const std::shared_ptr< Serial >& serial_ptr, RawBmsData& bms_data ) {
    unsigned char buff[ 13 ];
    unsigned char head1, head2;

    int err_num = 0;
    while ( true ) {  // wait for the data head
        if ( err_num > 5 ) {
            if ( err_num % 100 == 0 ) {
                std::cout << "[" << __FUNCTION__ << "] : "
                          << "Too many err when waiting for header!!!" << std::endl;
            }

            if ( err_num > 200 ) {
                std::cout << "[" << __FUNCTION__ << "] : "
                          << "Can not read anything data from serial--dev/ttyS3 after err_num > 200,and return false!!!" << std::endl;
                bms_data.communication_error = true;
                return false;
            }
        }

        serial_ptr->ReadData( &head1, sizeof( unsigned char ), true );
        if ( head1 == 0x5a ) {
            serial_ptr->ReadData( &head2, sizeof( unsigned char ), true );
            if ( head2 == 0xa5 ) {
                break;
            }
            else {
                err_num += 1;
            }
        }
        else {
            err_num += 1;
        }
        usleep( 1 );
    }

    unsigned char& dataLen = buff[ 0 ];
    serial_ptr->ReadData( ( unsigned char* )( &dataLen ), sizeof( dataLen ), true );

    serial_ptr->ReadData( ( unsigned char* )( buff + 1 ), std::max( ( int )dataLen, 13 ) + 2, true );

    bms_data.communication_error = false;

    //  for(std::size_t i = 0; i < sizeof(buff); i++){
    //    std::cout << "the data is: " << "--dec: " << dec <<  (int)buff[i] << " ; --hex: " << setiosflags(ios::uppercase) << hex << (int)buff[i]<<std::endl;
    //    }

    // if checksum is wrong, or the type is not data type do not update imu data
    if ( !( ParseBmsData( buff, bms_data ) && bms_data.data_type == 0x05 ) ) {
        std::cout << "ParseBmsData() failed,return false!" << std::endl;
        return false;
    }

    return true;
}
#else
bool ParseBmsData( unsigned char* buff, RawBmsData& bms_data ) {
    bms_data.bms_soc  = buff[ sizeof( Bmi088MsgHeadType ) ];
    bms_data.bms_volt = buff[ sizeof( Bmi088MsgHeadType ) + 1 ];
    bms_data.bms_curr = ( int8_t )buff[ sizeof( Bmi088MsgHeadType ) + 2 ];
    bms_data.bms_temp = buff[ sizeof( Bmi088MsgHeadType ) + 3 ];

    bms_data.bms_loop_number = GET16( buff + sizeof( Bmi088MsgHeadType ) + 4 );
    bms_data.bms_health      = buff[ sizeof( Bmi088MsgHeadType ) + 6 ];
    bms_data.bms_fault       = buff[ sizeof( Bmi088MsgHeadType ) + 7 ];
    bms_data.sum_check       = GET16( buff + sizeof( Bmi088MsgHeadType ) + 8 );

    if ( ( bms_data.data_type & 0x06 ) == 0x02 )
        bms_data.bms_status |= 0x05;
    else if ( ( bms_data.data_type & 0x06 ) == 0x06 )
        bms_data.bms_status |= 0x09;
    else
        bms_data.bms_status &= 0xF2;

    if ( ( bms_data.data_type & 0x10 ) == 0x10 )
        bms_data.bms_power_supply |= 0x01;
    else
        bms_data.bms_power_supply &= 0xFE;

    uint16_t checksum = 0;
    for ( uint16_t i = 0; i < sizeof( Bmi088MsgHeadType ) + bms_data.data_len; i++ ) {
        checksum += buff[ i ];
    }
    if ( checksum != bms_data.sum_check ) {
        fprintf( stderr, "[ERROR] BMS receive checksum is wrong, expect %#06X, got %#06X\n", checksum, bms_data.sum_check );
        return false;
    }
    return true;
}

bool GetBmsData( const std::shared_ptr< Serial >& serial_ptr, RawBmsData& bms_data ) {
    unsigned char buff[ MAX_BMS_BUFF_LEN ];
    unsigned char head1, head2;

    int err_num = 0;
    while ( true ) {  // wait for the data head
        if ( err_num > 5 ) {
            if ( err_num % 100 == 0 ) {
                std::cout << "[" << __FUNCTION__ << "] : "
                          << "Too many err when waiting for header!!!" << std::endl;
            }

            if ( err_num > 200 ) {
                std::cout << "[" << __FUNCTION__ << "] : "
                          << "Can not read anything data from serial--dev/ttyS3 after err_num > 200,and return false!!!" << std::endl;
                bms_data.communication_error = true;
                return false;
            }
        }

        serial_ptr->ReadData( &head1, sizeof( unsigned char ), true );
        buff[ 0 ] = head1;
        if ( head1 == 0x5a ) {
            serial_ptr->ReadData( &head2, sizeof( unsigned char ), true );
            if ( head2 == 0xa5 ) {
                buff[ 1 ] = head2;
                break;
            }
            else {
                err_num += 1;
            }
        }
        else {
            err_num += 1;
        }
        usleep( 1 );
    }

    serial_ptr->ReadData( ( unsigned char* )( &buff[ 2 ] ), sizeof( Bmi088MsgHeadType ) - 2, true );
    Bmi088MsgHeadType* msg_head = ( Bmi088MsgHeadType* )buff;

    uint8_t msg_FrameType = msg_head->frame_type;
    uint8_t msg_CMD       = msg_head->cmd;
    uint8_t msg_Len       = msg_head->payload_len;

    if ( msg_FrameType > 0x04 ) {
        std::cout << "BMS  receives wrong type or wrong cmd!  " << std::endl;
        return false;
    }

    serial_ptr->ReadData( ( unsigned char* )&buff[ sizeof( Bmi088MsgHeadType ) ], msg_Len + 2, true );

    bms_data.communication_error = false;
    bms_data.data_type           = msg_CMD;
    bms_data.data_len            = msg_Len;

    //  for(std::size_t i = 0; i < sizeof(buff); i++){
    //    std::cout << "the data is: " << "--dec: " << dec <<  (int)buff[i] << " ; --hex: " << setiosflags(ios::uppercase) << hex << (int)buff[i]<<std::endl;
    //    }

    // if checksum is wrong, or the type is not data type do not update imu data
    if ( !ParseBmsData( buff, bms_data ) ) {
        std::cout << "ParseBmsData() failed,return false!" << std::endl;
        return false;
    }

    return true;
}
#endif

#ifndef USE_NEW_BOARD
bool SetBmsData( const std::shared_ptr< Serial >& serial_ptr, const BmsSendData& bms_data ) {

    unsigned char sendData[ 8 ] = { 0XAA, 0X55, 0X03, 0X05, 0X01, 0X03, 0X00, 0X00 };
    static int    iCountNumber  = 1;

    sendData[ 4 ] = bms_data.charge_enable;
    sendData[ 5 ] = bms_data.power_supply;

    int16_t checksum = 0;
    for ( int i = 2; i < 6; i++ ) {
        checksum += sendData[ i ];
    }

    sendData[ 6 ] = checksum & 0XFF;
    sendData[ 7 ] = ( checksum >> 8 ) & 0XFF;

    if ( DEBUG_PRINTF ) {
        if ( iCountNumber % 1000 == 0 ) {
            std::cout << "SetBmsData() is invoked,and BMS data will be send to Serial Common!" << std::endl;
            iCountNumber = 0;
        }
    }

    serial_ptr->WriteData( ( char* )sendData, sizeof( sendData ) );

    iCountNumber++;

    return true;
}
#else
bool SetBmsData( const std::shared_ptr< Serial >& serial_ptr, const BmsSendData& bms_data ) {

    unsigned char sendData[ 11 ] = { 0XAA, 0X55, 0X00, 0X02, 0X01, 0x00, 0x00, 0X01, 0X00, 0X00, 0x00 };
    static int    iCountNumber   = 1;

    (void) bms_data;

    // if ( ( bms_data.power_supply & 0x01 ) == 0 ) {  // motor off
    //     sendData[ 7 ] &= 0xFE;
    //     sendData[ 7 ] |= 0x02;
    // }

    // if ( ( bms_data.power_supply & 0x04 ) == 0x04 ) {  // power off
    //     sendData[ 7 ] &= 0xFE;
    //     sendData[ 7 ] |= 0x08;
    // }

    // if ( ( bms_data.power_supply & 0x08 ) == 0x08 ) {  // power save
    //     sendData[ 7 ] &= 0xFE;
    //     sendData[ 7 ] |= 0x04;
    // }

    int16_t checksum = 0;
    for ( int i = 0; i < 9; i++ ) {
        checksum += sendData[ i ];
    }

    sendData[ 9 ]  = checksum & 0XFF;
    sendData[ 10 ] = ( checksum >> 8 ) & 0XFF;

    if ( DEBUG_PRINTF ) {
        if ( iCountNumber % 1000 == 0 ) {
            std::cout << "SetBmsData() is invoked,and BMS data will be send to Serial Common!" << std::endl;
            iCountNumber = 0;
        }
    }

    serial_ptr->WriteData( ( char* )sendData, sizeof( sendData ) );

    iCountNumber++;

    return true;
}
#endif

std::vector< char > GetMcuInfo( Serial& mcu_port, unsigned char mcu_num, unsigned char data_type ) {
    unsigned char version_req_data[ MCU_INFO_REQ_BUFF_LEN ] = { 0xAA, 0x55, 0x03, MCU_GET_INFO_TYPE, mcu_num, data_type, 0x00, 0x00 };
    // Get checksum
    int16_t checksum = 0;
    for ( int i = 2; i < 6; i++ ) {
        checksum += version_req_data[ i ];
    }
    version_req_data[ 6 ] = 0xff & checksum;
    version_req_data[ 7 ] = ( ( 0xff << 8 ) & checksum ) >> 8;
    for ( int i = 0; i < 4; i++ ) {

        if ( mcu_num == 15 && i < 3 ) {
            std::cout << "current mcu_num = 15 and i < 3,so do nothing and continue!" << std::endl;
            continue;
        }

        tcflush( mcu_port.fd(), TCIOFLUSH );
        mcu_port.WriteData( ( char* )version_req_data, MCU_INFO_REQ_BUFF_LEN );
        Timer t;
        while ( t.GetElapsedSeconds() < 1 ) {  // timeout 2s
            RawImuData imu_data;
            ImuResult  imu_res;
            bool       isBlock = true;

            // if the mcu is bms,get data in non-block module
            if ( mcu_num == 15 ) {
                isBlock = false;
                std::cout << "the current mcu name is bms,and the isBlock=false,the progress will not be blocked even if no serial port is connected!" << std::endl;
            }

            bool ret = GetImuData( imu_data, mcu_port, imu_res, isBlock );
            if ( !ret ) {
                std::cout << "[Warning]:[GetMcuInfo]:GetImuData() failed,but no problem!" << std::endl;
            }

            if ( imu_data.type == MCU_GET_INFO_TYPE ) {
                char buff[ MCU_INFO_REP_BUFF_LEN ] = { 0 };
                memcpy( buff, ( char* )&imu_data, sizeof( buff ) );
                int mcu_rep_num       = ( int )buff[ 2 ];
                int mcu_rep_info_type = ( int )buff[ 3 ];
                if ( mcu_rep_num != mcu_num ) {
                    std::cerr << "[MCU INFO] received invalid mcu number: " << mcu_rep_num << ", expect: " << mcu_num << std::endl;
                    continue;
                }
                else if ( mcu_rep_info_type != data_type ) {
                    std::cerr << "[MCU INFO] received invalid mcu info type: " << mcu_rep_info_type << ", expect: " << mcu_rep_info_type << std::endl;
                    continue;
                }
                else {  // All number check is OK, get target Info now
                    std::vector< char > info;
                    int                 buff_len = ( mcu_rep_num == MCU_ID_NUMBER_REQ ? MCU_ID_NUMBER_BUFF_LEN : MCU_INFO_RESULT_BUFF_LEN );
                    info.resize( buff_len );
                    memcpy( info.data(), &buff[ 4 ], buff_len );
                    return info;
                }
            }
            else {
                static int unrelated_iter = 0;
                if ( unrelated_iter % 300 == 0 ) {
                    std::cout << "[MCU INFO] Received an unrelated type: " << ( int )imu_data.type << std::endl;
                }
                unrelated_iter++;
            }
        }
    }
    std::cerr << "[MCU INFO] Get MCU INFO TimeOut in 4s!" << std::endl;
    return std::vector< char >();
}

std::vector< char > GetMcuInfo( Serial& mcu_port, uint8_t cmd, uint8_t dataLen, uint8_t dataType ) {
    Bmi088MsgHeadType msg_head;
    // imu info
    msg_head.frame_head1 = 0xAA;
    msg_head.frame_head2 = 0x55;
    msg_head.seq         = 0x00;
    msg_head.board_id    = 0x02;
    msg_head.sys_id      = 0x02;
    msg_head.sensor_id   = 0x04;
    msg_head.frame_type  = 0x00;
    msg_head.cmd         = cmd;
    msg_head.payload_len = dataLen;

    int           msg_total_len = sizeof( Bmi088MsgHeadType ) + msg_head.payload_len + 2;
    unsigned char buff[ msg_total_len ];
    memcpy( buff, &msg_head, sizeof( Bmi088MsgHeadType ) );
    if ( msg_head.payload_len > 0 ) {
        buff[ sizeof( Bmi088MsgHeadType ) ] = dataType;
        for ( uint16_t i = 0; i < msg_head.payload_len - 1; i++ )
            buff[ sizeof( Bmi088MsgHeadType ) + 1 + i ] = 0x00;
        // memcpy( &(buff[ sizeof( Bmi088MsgHeadType ) + 1 ]), 0, 48 );
    }
    uint16_t CheckSum = 0;
    for ( int i = 0; i < msg_total_len - 2; i++ )
        CheckSum += buff[ i ];
    buff[ msg_total_len - 2 ] = CheckSum & 0x00ff;
    buff[ msg_total_len - 1 ] = CheckSum >> 8 & 0x00ff;

    tcflush( mcu_port.fd(), TCIOFLUSH );
    mcu_port.WriteData( ( char* )buff, msg_total_len );

    std::vector< char > version_number = {};
    if ( cmd != 0x02 ) {
        unsigned char version_buff[ msg_head.payload_len ];
        unsigned char receive_cmd = 0, receive_len = 0;
        uint16_t      readLen = 16;
        if ( cmd == 0x06 && dataType == 0x04 )
            readLen = 32;
        else if ( cmd == 0x06 && dataType == 0x05 )
            readLen = 16;
        bool read_succeed = false;
        ( void )read_succeed;
        int read_iter = 0;
        do {
            read_succeed = GetNewSerialData( version_buff, receive_cmd, receive_len, mcu_port, false );
            usleep( 1000 );
            read_iter++;
        } while ( !( ( receive_cmd == cmd && receive_len == dataLen && version_buff[ 0 ] == dataType ) || read_iter > 60 ) );
        if ( receive_cmd == cmd && receive_len == dataLen && version_buff[ 0 ] == dataType ) {
            for ( uint16_t i = 1; i < readLen + 1; i++ ) {
                version_number.push_back( version_buff[ i ] );
            }
        }
    }

    return version_number;
}

bool Id2Int( std::vector< char >& ids, int64_t& id_high, int64_t& id_low ) {
    if ( ids.size() != MCU_ID_NUMBER_BUFF_LEN ) {
        std::cerr << "[IMU ID Convert] Fatal err, ids size mismatch! expect: " << MCU_ID_NUMBER_BUFF_LEN << ", got: " << ids.size() << std::endl;
        return false;
    }
    id_high = 0;
    id_low  = 0;
    for ( int i = 0; i < MCU_ID_NUMBER_BUFF_LEN; i++ ) {
        unsigned char curr_id    = ids[ i ];
        int           shift_i    = ( i % 8 );
        int64_t&      target_int = ( i >= 8 ? id_high : id_low );
        target_int |= ( static_cast< uint64_t >( curr_id ) << ( shift_i * 8 ) );
    }
    return true;
}

std::string Conversion( const char* src ) {
    std::string des( 23, '\0' );
    uint16_t    year = 0;
    for ( int i = 0; i < 12; i++ ) {
        if ( i <= 0 ) {
            des[ 0 ] = src[ i ];
        }
        else if ( i <= 4 ) {
            des[ 2 * i - 1 ] = src[ i ] + 0x30;
            if ( i != 4 )
                des[ 2 * i ] = 0x2E;
            else
                des[ 2 * i ] = 0x5F;
        }
        else if ( i <= 6 ) {
            if ( i == 5 )
                year = 256 * src[ i ];
            else
                year += src[ i ];
            des[ 9 ]  = year / 1000 + 0x30;
            des[ 10 ] = year % 1000 / 100 + 0x30;
            des[ 11 ] = year % 1000 % 100 / 10 + 0x30;
            des[ 12 ] = year % 1000 % 100 % 10 + 0x30;
        }
        else {
            des[ 2 * i - 1 ] = src[ i ] / 10 + 0x30;
            des[ 2 * i ]     = src[ i ] % 10 + 0x30;
        }
    }
    return des;
}