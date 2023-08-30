#include <fcntl.h>
#include <iostream>
#include <linux/spi/spidev.h>
#include <string>
#include <sys/ioctl.h>

#include "Configuration.h"
#include "rt/spi_handler_for_imu.hpp"

/**
 * @brief Construct a new Spi Handler For Imu:: Spi Handler For Imu object
 *
 */
SpiHandlerForImu::SpiHandlerForImu() {
    memset( &spi_command_, 0, sizeof( SpiCommandForImu ) );
    memset( &spi_data_, 0, sizeof( SpiDataFromImu ) );

    spi_fd_                     = -1;
    spi_driver_iteration_       = 0;
    spi_read_success_iteration_ = 0;
}

/**
 * @brief Destroy the Spi Handler For Imu:: Spi Handler For Imu object
 *
 */
SpiHandlerForImu::~SpiHandlerForImu() {}

/**
 * @brief Initialize spi, read-in joint parameters and open spi device
 *
 */
void SpiHandlerForImu::InitializeSpi() {
    unsigned char spi_mode          = SPI_MODE_0;
    unsigned char spi_bits_per_word = 8;
    unsigned int  spi_speed         = 10000000;
    uint8_t       lsb               = 0x01;

    int rv  = 0;
    spi_fd_ = open( "/dev/spidev2.0", O_RDWR );
    if ( spi_fd_ < 0 )
        perror( "[ERROR] Couldn't open spidev 2.0" );

    rv = ioctl( spi_fd_, SPI_IOC_WR_MODE, &spi_mode );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_wr_mode (1)" );

    rv = ioctl( spi_fd_, SPI_IOC_RD_MODE, &spi_mode );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_mode (1)" );

    rv = ioctl( spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_wr_bits_per_word (1)" );

    rv = ioctl( spi_fd_, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_bits_per_word (1)" );

    rv = ioctl( spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_wr_max_speed_hz (1)" );

    rv = ioctl( spi_fd_, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_max_speed_hz (1)" );

    rv = ioctl( spi_fd_, SPI_IOC_RD_LSB_FIRST, &lsb );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_lsb_first (1)" );

    printf( "[SpiHandlerForImu] Open spi device successfully!\n" );
}

/**
 * @brief Input spi command for imu
 *
 * @param cmd
 */
void SpiHandlerForImu::SetSpiCommandForImu( const ImuProtocolHeader& cmd ) {
    memcpy( &spi_command_, &cmd, sizeof( ImuProtocolHeader ) );
    // set zero for payload
    memset( spi_command_.payload, 0x00, kMaxSizeOfPayload );
    // CheckSum
    uint16_t sum                = CheckSum( ( uint8_t* )&spi_command_ );
    spi_command_.sum_low_order  = sum & 0x00ff;
    spi_command_.sum_high_order = sum >> 8 && 0x00ff;
}

/**
 * @brief Output data from imu
 *
 * @param data
 */
void SpiHandlerForImu::GetImuDataBySpi( ImuNorminalData& data ) {
    ImuProtocolHeader* header = &spi_data_.header_data;
    // judge whether valid data
    if ( header->frame_header_first == 0x5A && header->frame_header_second == 0xA5 ) {
        if ( header->command == 0x01 && header->frame_type == 0x01 ) {
            memcpy( &data, spi_data_.payload, header->payload_length );
            spi_read_success_iteration_++;
        }
    }
    // memset( &data, 0, sizeof( ImuNorminalData ) );
}

/**
 * @brief Communicate with spi device
 *
 */
void SpiHandlerForImu::DriverRun() {
    // update driver status flag
    spi_driver_iteration_++;

    unsigned char spi_bits_per_word = 8;
    // transmit and receive buffers
    uint8_t tx_buf[ kNumWordPerMessage ];
    uint8_t rx_buf[ kNumWordPerMessage ];

    // pointers to command/data spine array
    uint8_t* cmd_d  = ( uint8_t* )&spi_command_;
    uint8_t* data_d = ( uint8_t* )&spi_data_;

    // zero rx buffer
    memset( rx_buf, 0, kNumWordPerMessage * sizeof( uint8_t ) );

    // copy into tx buffer flipping bytes
    for ( int i = 0; i < kNumWordPerMessage; i++ )
        tx_buf[ i ] = cmd_d[ i ];

    // each word is one bytes long
    size_t word_len = 1;  // 8 bit word

    // spi message struct
    struct spi_ioc_transfer spi_message;

    // zero message struct.
    memset( &spi_message, 0, 1 * sizeof( struct spi_ioc_transfer ) );

    // set up message struct
    spi_message.bits_per_word = spi_bits_per_word;
    spi_message.cs_change     = 1;
    spi_message.delay_usecs   = 0;
    spi_message.len           = word_len * kNumWordPerMessage;
    spi_message.rx_buf        = ( uint64_t )rx_buf;
    spi_message.tx_buf        = ( uint64_t )tx_buf;

    // do spi communication
    int rv = ioctl( spi_fd_, SPI_IOC_MESSAGE( 1 ), &spi_message );
    ( void )rv;

    for ( int i = 0; i < kNumWordPerMessage; i++ ) {
        data_d[ i ] = rx_buf[ i ];
    }
}

/**
 * @brief Clear up buffer memory of spi device
 *
 */
void SpiHandlerForImu::FlushSpiBuffer() {
    // transmit and receive buffers
    uint8_t       tx_buf[ kNumWordPerMessage ];
    uint8_t       rx_buf[ kNumWordPerMessage ];
    unsigned char spi_bits_per_word = 8;

    // flush the can for 20 times to discard buff pos
    for ( int k = 0; k < 20; k++ ) {
        // zero rx/tx buffer
        memset( rx_buf, 0, kNumWordPerMessage * sizeof( uint8_t ) );
        memset( tx_buf, 0, kNumWordPerMessage * sizeof( uint8_t ) );
        // spi message struct
        struct spi_ioc_transfer spi_message[ 1 ];

        // zero message struct.
        memset( spi_message, 0, 1 * sizeof( struct spi_ioc_transfer ) );

        size_t word_len = 1;  // 8 bit word

        // set up message struct
        for ( int i = 0; i < 1; i++ ) {
            spi_message[ i ].bits_per_word = spi_bits_per_word;
            spi_message[ i ].cs_change     = 1;
            spi_message[ i ].delay_usecs   = 0;
            spi_message[ i ].len           = word_len * kNumWordPerMessage;
            spi_message[ i ].rx_buf        = ( uint64_t )rx_buf;
            spi_message[ i ].tx_buf        = ( uint64_t )tx_buf;
        }

        errno = 0;
        // do spi communication
        int rv = ioctl( spi_fd_, SPI_IOC_MESSAGE( 1 ), &spi_message );
        ( void )rv;
        if ( errno != 0 ) {
            printf( "[SpiHandlerForImu] Flush_spi_buffer, spi ioctl ret(%d)error(%d) %s\n", rv, errno, strerror( errno ) );
        }
    }
}

/**
 * @brief Publish spi command and data by lcm, not work
 *
 * @param spi_lcm
 */
void SpiHandlerForImu::PublishSpi( lcm::LCM& spi_lcm ) {
    ( void )spi_lcm;
    // spi_lcm.publish( "spi_data", &spi_data_ );
    // spi_lcm.publish( "spi_command", &spi_command_ );
}

/**
 * @brief Set spi command zero
 *
 */
void SpiHandlerForImu::SetCommandZero() {
    memset( &spi_command_, 0, sizeof( SpiCommandForImu ) );
}

/**
 * @brief Reset some logic variables, not work
 *
 */
void SpiHandlerForImu::ResetSpi() {}

/**
 * @brief Compute checksum of spi message
 *
 * @param message input data
 * @return uint16_t
 */
uint16_t SpiHandlerForImu::CheckSum( uint8_t* message ) {
    ImuProtocolHeader* msg_head = ( ImuProtocolHeader* )message;
    uint16_t           AC       = 0;
    uint8_t            len      = msg_head->payload_length + sizeof( ImuProtocolHeader );

    while ( len-- )
        AC += ( *message++ );

    return AC;
}