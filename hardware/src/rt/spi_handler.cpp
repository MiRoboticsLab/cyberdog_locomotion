#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <linux/spi/spidev.h>
#include <string>
#include <sys/ioctl.h>

#include "Configuration.h"
#include "rt/spi_handler.hpp"

/**
 * @brief Construct a new Spi Handler:: Spi Handler object
 *
 */
SpiHandler::SpiHandler() {
    memset( &spi_command_, 0, sizeof( spi_command_t ) );
    memset( &spi_data_, 0, sizeof( spi_data_t ) );
    iteration_            = 0;
    spi_fd1_              = -1;
    spi_fd2_              = -1;
    wrong_iteration_      = 0;
    spi_driver_iteration_ = 0;
    robot_type_           = RobotType::CYBERDOG2;
    for ( int i = 0; i < 4; i++ ) {
        apply_hip_range_offset_[ i ] = false;
    }
}

/**
 * @brief Destroy the Spi Handler:: Spi Handler object
 *
 */
SpiHandler::~SpiHandler() {}

/**
 * @brief Initialize spi, read-in joint parameters and open spi device
 *
 * @param robot_type robot type, cyberdog or cyberdog2
 */
void SpiHandler::InitializeSpi( const RobotType& robot_type, const RobotAppearanceType& appearance_type ) {
    try {
        if ( robot_type == RobotType::CYBERDOG2 ) {
            if ( appearance_type == RobotAppearanceType::CURVED )
                spi_params_.InitializeFromYamlFile( THIS_COM "common/config/cyberdog2-spi.yaml" );
            else
                spi_params_.InitializeFromYamlFile( THIS_COM "common/config/cyberdog2-angular-spi.yaml" );
        }
        else
            spi_params_.InitializeFromYamlFile( THIS_COM "common/config/cyberdog-spi.yaml" );
        std::fstream file;
        file.open( JOINTS_CALIBRATE_FILE_PATH, std::ios::in );
        if ( file.good() )
            joints_params_.InitializeFromYamlFile( JOINTS_CALIBRATE_FILE_PATH );
        else
            joints_params_.InitializeFromYamlFile( THIS_COM "common/config/joints_calibrate_param.yaml" );

        robot_type_    = robot_type;
        spi_data_size_ = ( robot_type == RobotType::CYBERDOG2 ) ? 66 : 50;
    }
    catch ( std::exception& e ) {
        printf( "[SpiHandler] Failed to initialize robot parameters from yaml file: %s\n", e.what() );
        exit( 1 );
    }

    unsigned char spi_mode          = SPI_MODE_0;
    unsigned char spi_bits_per_word = 8;
    unsigned int  spi_speed         = 10000000;
    uint8_t       lsb               = 0x01;

    int rv   = 0;
    spi_fd1_ = open( "/dev/spidev0.0", O_RDWR );
    if ( spi_fd1_ < 0 )
        perror( "[ERROR] Couldn't open spidev 0.0" );
    spi_fd2_ = open( "/dev/spidev0.1", O_RDWR );
    if ( spi_fd2_ < 0 )
        perror( "[ERROR] Couldn't open spidev 0.1" );

    rv = ioctl( spi_fd1_, SPI_IOC_WR_MODE, &spi_mode );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_wr_mode (1)" );

    rv = ioctl( spi_fd2_, SPI_IOC_WR_MODE, &spi_mode );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_wr_mode (2)" );

    rv = ioctl( spi_fd1_, SPI_IOC_RD_MODE, &spi_mode );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_mode (1)" );

    rv = ioctl( spi_fd2_, SPI_IOC_RD_MODE, &spi_mode );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_mode (2)" );

    rv = ioctl( spi_fd1_, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_wr_bits_per_word (1)" );

    rv = ioctl( spi_fd2_, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_wr_bits_per_word (2)" );

    rv = ioctl( spi_fd1_, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_bits_per_word (1)" );

    rv = ioctl( spi_fd2_, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_bits_per_word (2)" );

    rv = ioctl( spi_fd1_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_wr_max_speed_hz (1)" );
    rv = ioctl( spi_fd2_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_wr_max_speed_hz (2)" );

    rv = ioctl( spi_fd1_, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_max_speed_hz (1)" );
    rv = ioctl( spi_fd2_, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_max_speed_hz (2)" );

    rv = ioctl( spi_fd1_, SPI_IOC_RD_LSB_FIRST, &lsb );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_lsb_first (1)" );

    rv = ioctl( spi_fd2_, SPI_IOC_RD_LSB_FIRST, &lsb );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_lsb_first (2)" );

    printf( "[SpiHandler] Open spi device successfully!\n" );
}

/**
 * @brief Initialize spi, read-in joint parameters and open spi device
 *
 */
void SpiHandler::InitializeSpi() {
    try {
        spi_params_.InitializeFromYamlFile( THIS_COM "common/config/cyberdog2-spi.yaml" );
    }
    catch ( std::exception& e ) {
        printf( "[SpiHandler] Failed to initialize robot parameters from yaml file: %s\n", e.what() );
        exit( 1 );
    }

    unsigned char spi_mode          = SPI_MODE_0;
    unsigned char spi_bits_per_word = 8;
    unsigned int  spi_speed         = 10000000;
    uint8_t       lsb               = 0x01;

    int rv   = 0;
    spi_fd1_ = open( "/dev/spidev0.0", O_RDWR );
    if ( spi_fd1_ < 0 )
        perror( "[ERROR] Couldn't open spidev 0.0" );
    spi_fd2_ = open( "/dev/spidev0.1", O_RDWR );
    if ( spi_fd2_ < 0 )
        perror( "[ERROR] Couldn't open spidev 0.1" );

    rv = ioctl( spi_fd1_, SPI_IOC_WR_MODE, &spi_mode );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_wr_mode (1)" );

    rv = ioctl( spi_fd2_, SPI_IOC_WR_MODE, &spi_mode );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_wr_mode (2)" );

    rv = ioctl( spi_fd1_, SPI_IOC_RD_MODE, &spi_mode );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_mode (1)" );

    rv = ioctl( spi_fd2_, SPI_IOC_RD_MODE, &spi_mode );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_mode (2)" );

    rv = ioctl( spi_fd1_, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_wr_bits_per_word (1)" );

    rv = ioctl( spi_fd2_, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_wr_bits_per_word (2)" );

    rv = ioctl( spi_fd1_, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_bits_per_word (1)" );

    rv = ioctl( spi_fd2_, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_bits_per_word (2)" );

    rv = ioctl( spi_fd1_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_wr_max_speed_hz (1)" );
    rv = ioctl( spi_fd2_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_wr_max_speed_hz (2)" );

    rv = ioctl( spi_fd1_, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_max_speed_hz (1)" );
    rv = ioctl( spi_fd2_, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_max_speed_hz (2)" );

    rv = ioctl( spi_fd1_, SPI_IOC_RD_LSB_FIRST, &lsb );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_lsb_first (1)" );

    rv = ioctl( spi_fd2_, SPI_IOC_RD_LSB_FIRST, &lsb );
    if ( rv < 0 )
        perror( "[ERROR] ioctl spi_ioc_rd_lsb_first (2)" );

    printf( "[SpiHandler] Open spi device successfully!\n" );
}

/**
 * @brief Input spi command
 *
 * @param cmd desired command
 */
void SpiHandler::SetSpiCommand( const SpiCommand& cmd ) {
    // NOTE: currently SpiCommand and spi_command_t have exactly
    // same structure, so we can copy them byte by byte
    memcpy( &spi_command_, &cmd, sizeof( spi_command_t ) );
}

/**
 * @brief Output spi data
 *
 * @param data current data
 */
void SpiHandler::GetSpiData( SpiData& data ) {
    // NOTE: currently SpiData and spi_data_t have exactly
    // same structure, so we can copy them byte by byte
    memcpy( &data, &spi_data_, sizeof( spi_data_t ) );
}

/**
 * @brief Communicate with spi device
 *
 */
void SpiHandler::DriverRun() {
    // update driver status flag
    spi_driver_iteration_++;
    spi_data_.spi_driver_status = spi_driver_iteration_ << 16;

    unsigned char spi_bits_per_word = 8;
    // transmit and receive buffers
    uint16_t tx_buf[ K_WORDS_PER_MESSAGE ];
    uint16_t rx_buf[ K_WORDS_PER_MESSAGE ];

    for ( int spi_board = 0; spi_board < 2; spi_board++ ) {
        SpineCommand spine_cmd;
        SpineDate    spine_data;
        // copy command into spine type:
        SpiToSpine( spi_command_, spine_cmd, spi_board * 2 );

        // pointers to command/data spine array
        uint16_t* cmd_d  = ( uint16_t* )&spine_cmd;
        uint16_t* data_d = ( uint16_t* )&spine_data;

        // zero rx buffer
        memset( rx_buf, 0, K_WORDS_PER_MESSAGE * sizeof( uint16_t ) );

        // copy into tx buffer flipping bytes
        for ( int i = 0; i < K_WORDS_PER_MESSAGE; i++ )
            tx_buf[ i ] = cmd_d[ i ];

        // each word is two bytes long
        size_t word_len = 2;  // 16 bit word

        // spi message struct
        struct spi_ioc_transfer spi_message[ 1 ];

        // zero message struct.
        memset( spi_message, 0, 1 * sizeof( struct spi_ioc_transfer ) );

        // set up message struct
        for ( int i = 0; i < 1; i++ ) {
            spi_message[ i ].bits_per_word = spi_bits_per_word;
            spi_message[ i ].cs_change     = 1;
            spi_message[ i ].delay_usecs   = 0;
            spi_message[ i ].len           = word_len * K_WORDS_PER_MESSAGE;
            spi_message[ i ].rx_buf        = ( uint64_t )rx_buf;
            spi_message[ i ].tx_buf        = ( uint64_t )tx_buf;
        }

        // do spi communication
        int rv = ioctl( spi_board == 0 ? spi_fd1_ : spi_fd2_, SPI_IOC_MESSAGE( 1 ), &spi_message );
        ( void )rv;

        for ( int i = 0; i < K_WORDS_PER_MESSAGE; i++ ) {
            if ( robot_type_ == RobotType::CYBERDOG ) {
                if ( i < spi_data_size_ + 2 )
                    data_d[ i ] = rx_buf[ i ];
                else
                    data_d[ i ] = rx_buf[ i - 16 ];
            }
            else
                data_d[ i ] = rx_buf[ i ];
        }
        // copy back to data
        SpineToSpi( spi_data_, spine_data, spi_board * 2 );
    }
}

/**
 * @brief Transfer command from spi to spine
 *
 * @param cmd spi command
 * @param spine_cmd spine command
 * @param leg_0 start leg id for two spines
 */
void SpiHandler::SpiToSpine( spi_command_t& cmd, SpineCommand& spine_cmd, int leg_0 ) {
    auto& abad_side_sign = spi_params_.abad_side_sign;
    auto& hip_side_sign  = spi_params_.hip_side_sign;
    auto& knee_side_sign = spi_params_.knee_side_sign;
    auto& abad_offset    = spi_params_.abad_offset;
    auto& hip_offset     = spi_params_.hip_offset;
    auto& knee_offset    = spi_params_.knee_offset;

    for ( int i = 0; i < 2; i++ ) {
        int spine_i                     = ( leg_0 == 2 ? 1 - i : i );  // for the none cross wire design
        spine_cmd.q_des_abad[ spine_i ] = ( cmd.q_des_abad[ i + leg_0 ] * abad_side_sign[ i + leg_0 ] ) + abad_offset[ i + leg_0 ] + joints_params_.abad_calibrate[ i + leg_0 ];
        spine_cmd.q_des_hip[ spine_i ]  = ( cmd.q_des_hip[ i + leg_0 ] * hip_side_sign[ i + leg_0 ] ) + hip_offset[ i + leg_0 ] + joints_params_.hip_calibrate[ i + leg_0 ];
        spine_cmd.q_des_knee[ spine_i ] = ( cmd.q_des_knee[ i + leg_0 ] * knee_side_sign[ i + leg_0 ] ) + knee_offset[ i + leg_0 ] + joints_params_.knee_calibrate[ i + leg_0 ];

        // for motor startup [-pi, pi] feature
        if ( apply_hip_range_offset_[ i + leg_0 ] ) {
            spine_cmd.q_des_hip[ spine_i ] -= spi_params_.hip_range_offset[ i + leg_0 ];
        }

        spine_cmd.qd_des_abad[ spine_i ] = cmd.qd_des_abad[ i + leg_0 ] * abad_side_sign[ i + leg_0 ];
        spine_cmd.qd_des_hip[ spine_i ]  = cmd.qd_des_hip[ i + leg_0 ] * hip_side_sign[ i + leg_0 ];
        spine_cmd.qd_des_knee[ spine_i ] = cmd.qd_des_knee[ i + leg_0 ] * knee_side_sign[ i + leg_0 ];

        spine_cmd.kp_abad[ spine_i ] = cmd.kp_abad[ i + leg_0 ];
        spine_cmd.kp_hip[ spine_i ]  = cmd.kp_hip[ i + leg_0 ];
        spine_cmd.kp_knee[ spine_i ] = cmd.kp_knee[ i + leg_0 ];

        spine_cmd.kd_abad[ spine_i ] = cmd.kd_abad[ i + leg_0 ];
        spine_cmd.kd_hip[ spine_i ]  = cmd.kd_hip[ i + leg_0 ];
        spine_cmd.kd_knee[ spine_i ] = cmd.kd_knee[ i + leg_0 ];

        spine_cmd.tau_abad_ff[ spine_i ] = cmd.tau_abad_ff[ i + leg_0 ] * abad_side_sign[ i + leg_0 ];
        spine_cmd.tau_hip_ff[ spine_i ]  = cmd.tau_hip_ff[ i + leg_0 ] * hip_side_sign[ i + leg_0 ];
        spine_cmd.tau_knee_ff[ spine_i ] = cmd.tau_knee_ff[ i + leg_0 ] * knee_side_sign[ i + leg_0 ];

        spine_cmd.flags[ spine_i ] = cmd.flags[ i + leg_0 ];
    }
    spine_cmd.checksum = CheckSum( ( uint32_t* )&spine_cmd, 32 );
}

/**
 * @brief Whether need to compensate position offset for hip joints
 *
 * @param hip_range
 * @param q_hip
 * @param leg
 * @return true
 * @return false
 */
bool SpiHandler::FindHipOffset( const Vec8< double >& hip_range, float q_hip, int leg ) {
    if ( hip_range[ 2 * leg ] < q_hip && q_hip < hip_range[ 2 * leg + 1 ] ) {
        apply_hip_range_offset_[ leg ] = false;
    }
    else {
        apply_hip_range_offset_[ leg ] = true;
    }
    return apply_hip_range_offset_[ leg ];
}

/**
 * @brief Transfer data from spie to spi
 *
 * @param data spi data
 * @param spine_data spine data
 * @param leg_0 start leg id for two spines
 */
void SpiHandler::SpineToSpi( spi_data_t& data, SpineDate& spine_data, int leg_0 ) {
    auto& abad_side_sign = spi_params_.abad_side_sign;
    auto& hip_side_sign  = spi_params_.hip_side_sign;
    auto& knee_side_sign = spi_params_.knee_side_sign;
    auto& abad_offset    = spi_params_.abad_offset;
    auto& hip_offset     = spi_params_.hip_offset;
    auto& knee_offset    = spi_params_.knee_offset;

    uint32_t calc_checksum = CheckSum( ( uint32_t* )&spine_data, spi_data_size_ / 2 - 1 );  // Mini:32 cyberdog:24
    if ( calc_checksum != ( uint32_t )spine_data.checksum )
        printf( "[SpiHandler] Spi error, bad checksum, get: 0x%x expected: 0x%x\n", calc_checksum, spine_data.checksum );
    else {
        iteration_++;
        for ( int i = 0; i < 2; i++ ) {
            int spine_i = ( leg_0 == 2 ? 1 - i : i );  // for the none cross wire design
            // for startup [-pi, pi] feature
            float q_hip = spine_data.q_hip[ spine_i ];
            if ( iteration_ < 2000 ) {
                FindHipOffset( spi_params_.hip_range, q_hip, i + leg_0 );
            }
            if ( apply_hip_range_offset_[ i + leg_0 ] ) {
                q_hip += spi_params_.hip_range_offset[ i + leg_0 ];
            }

            float tmp           = 0.;
            int   wrong_reset   = 100;
            bool  is_data_wrong = false;
            tmp                 = ( spine_data.q_abad[ spine_i ] - abad_offset[ i + leg_0 ] - joints_params_.abad_calibrate[ i + leg_0 ] ) * abad_side_sign[ i + leg_0 ];
            // set offset according to ( max_motor_speed * predicted_spie_overtime * safty_gain ) is : 25 * 0.005 * 1.5
            if ( fabs( tmp - data.q_abad[ i + leg_0 ] ) < 0.1875 || iteration_ < 3000 || wrong_iteration_ == wrong_reset ) {
                data.q_abad[ i + leg_0 ] = tmp;
            }
            else {
                is_data_wrong = true;
            }
            tmp = ( q_hip - hip_offset[ i + leg_0 ] - joints_params_.hip_calibrate[ i + leg_0 ] ) * hip_side_sign[ i + leg_0 ];
            if ( fabs( tmp - data.q_hip[ i + leg_0 ] ) < 0.1875 || iteration_ < 3000 || wrong_iteration_ == wrong_reset ) {
                data.q_hip[ i + leg_0 ] = tmp;
            }
            else {
                is_data_wrong = true;
            }
            tmp = ( spine_data.q_knee[ spine_i ] - knee_offset[ i + leg_0 ] - joints_params_.knee_calibrate[ i + leg_0 ] ) * knee_side_sign[ i + leg_0 ];
            if ( fabs( tmp - data.q_knee[ i + leg_0 ] ) < 0.1875 || iteration_ < 3000 || wrong_iteration_ == wrong_reset ) {
                data.q_knee[ i + leg_0 ] = tmp;
            }
            else {
                is_data_wrong = true;
            }
            if ( is_data_wrong ) {
                if ( wrong_iteration_ % 10 == 0 ) {
                    std::cout << "[SPIHander] Get wrong spi data!!!" << std::endl;
                }
                wrong_iteration_++;
                if ( wrong_iteration_ > wrong_reset )
                    wrong_iteration_ = 0;
            }

            data.qd_abad[ i + leg_0 ] = spine_data.qd_abad[ spine_i ] * abad_side_sign[ i + leg_0 ];
            data.qd_hip[ i + leg_0 ]  = spine_data.qd_hip[ spine_i ] * hip_side_sign[ i + leg_0 ];
            data.qd_knee[ i + leg_0 ] = spine_data.qd_knee[ spine_i ] * knee_side_sign[ i + leg_0 ];

            for ( int motor = 0; motor < 3; ++motor ) {
                data.flags[ motor + 3 * i + 3 * leg_0 ] = spine_data.flags[ motor + 3 * spine_i ];
            }

            data.tau_abad[ i + leg_0 ] = spine_data.tau_abad[ spine_i ] * abad_side_sign[ i + leg_0 ];
            data.tau_hip[ i + leg_0 ]  = spine_data.tau_hip[ spine_i ] * hip_side_sign[ i + leg_0 ];
            data.tau_knee[ i + leg_0 ] = spine_data.tau_knee[ spine_i ] * knee_side_sign[ i + leg_0 ];

            data.tmp_abad[ i + leg_0 ] = spine_data.tmp_abad[ spine_i ] / 1.0;
            data.tmp_hip[ i + leg_0 ]  = spine_data.tmp_hip[ spine_i ] / 1.0;
            data.tmp_knee[ i + leg_0 ] = spine_data.tmp_knee[ spine_i ] / 1.0;
        }
    }
}

/**
 * @brief Clear up buffer memory of spi device
 *
 */
void SpiHandler::FlushSpiBuffer() {
    // transmit and receive buffers
    uint16_t      tx_buf[ K_WORDS_PER_MESSAGE ];
    uint16_t      rx_buf[ K_WORDS_PER_MESSAGE ];
    unsigned char spi_bits_per_word = 8;

    // flush the can for 20 times to discard buff pos
    for ( int k = 0; k < 20; k++ ) {
        for ( int spi_board = 0; spi_board < 2; spi_board++ ) {
            // zero rx/tx buffer
            memset( rx_buf, 0, K_WORDS_PER_MESSAGE * sizeof( uint16_t ) );
            memset( tx_buf, 0, K_WORDS_PER_MESSAGE * sizeof( uint16_t ) );
            // spi message struct
            struct spi_ioc_transfer spi_message[ 1 ];

            // zero message struct.
            memset( spi_message, 0, 1 * sizeof( struct spi_ioc_transfer ) );

            // set up message struct
            for ( int i = 0; i < 1; i++ ) {
                spi_message[ i ].bits_per_word = spi_bits_per_word;
                spi_message[ i ].cs_change     = 1;
                spi_message[ i ].delay_usecs   = 0;
                spi_message[ i ].len           = 2 * 66;
                spi_message[ i ].rx_buf        = ( uint64_t )rx_buf;
                spi_message[ i ].tx_buf        = ( uint64_t )tx_buf;
            }

            errno = 0;
            // do spi communication
            int rv = ioctl( spi_board == 0 ? spi_fd1_ : spi_fd2_, SPI_IOC_MESSAGE( 1 ), &spi_message );
            ( void )rv;
            if ( errno != 0 ) {
                printf( "[SpiHandler] Flush_spi_buffer, spi ioctl ret(%d)error(%d) %s\n", rv, errno, strerror( errno ) );
            }
        }
    }
}

/**
 * @brief Publish spi command and data by lcm
 *
 * @param spi_lcm lcm url
 */
void SpiHandler::PublishSpi( lcm::LCM& spi_lcm ) {
    spi_lcm.publish( "spi_data", &spi_data_ );
    spi_lcm.publish( "spi_command", &spi_command_ );
}

/**
 * @brief Set spi command zero
 *
 */
void SpiHandler::SetCommandZero() {
    memset( &spi_command_, 0, sizeof( spi_command_t ) );
}

/**
 * @brief Reset some logic variables
 *
 */
void SpiHandler::ResetSpi() {
    iteration_ = 0;
    for ( int i = 0; i < 4; i++ ) {
        apply_hip_range_offset_[ i ] = false;
    }
}

/**
 * @brief Set spi command zero and send flag to motor controller
 *
 */
void SpiHandler::SetMotorZeroFlag() {
    SetCommandZero();
    for ( int i = 0; i < 4; i++ ) {
        spi_command_.flags[ i ] = ZERO_FLAG;
    }
}

/**
 * @brief Reset flag of spi command to motor controller
 *
 */
void SpiHandler::ResetMotorZeroFlag() {
    for ( int i = 0; i < 4; i++ ) {
        spi_command_.flags[ i ] = 0;
    }
}

/**
 * @brief Transfer data from spi to spine
 *
 * @param data spi data
 * @param spine_data spine data
 * @param leg_0 start leg id for two spines
 */
void SpiHandler::SpiDataToSpineData( const spi_data_t* data, SpineDate* spine_data, int leg_0 ) {
    auto& abad_side_sign = spi_params_.abad_side_sign;
    auto& hip_side_sign  = spi_params_.hip_side_sign;
    auto& knee_side_sign = spi_params_.knee_side_sign;
    auto& abad_offset    = spi_params_.abad_offset;
    auto& hip_offset     = spi_params_.hip_offset;
    auto& knee_offset    = spi_params_.knee_offset;
    for ( int i = 0; i < 2; i++ ) {
        spine_data->q_abad[ i + leg_0 ]   = ( data->q_abad[ i + leg_0 ] * abad_side_sign[ i + leg_0 ] ) + abad_offset[ i + leg_0 ];
        spine_data->q_hip[ i + leg_0 ]    = ( data->q_hip[ i + leg_0 ] * hip_side_sign[ i + leg_0 ] ) + hip_offset[ i + leg_0 ];
        spine_data->q_knee[ i + leg_0 ]   = ( data->q_knee[ i + leg_0 ] * knee_side_sign[ i + leg_0 ] ) + knee_offset[ i + leg_0 ];
        spine_data->qd_abad[ i + leg_0 ]  = data->qd_abad[ i + leg_0 ] * abad_side_sign[ i + leg_0 ];
        spine_data->qd_hip[ i + leg_0 ]   = data->qd_hip[ i + leg_0 ] * hip_side_sign[ i + leg_0 ];
        spine_data->qd_knee[ i + leg_0 ]  = data->qd_knee[ i + leg_0 ] * knee_side_sign[ i + leg_0 ];
        spine_data->tau_abad[ i + leg_0 ] = data->tau_abad[ i + leg_0 ] * abad_side_sign[ i + leg_0 ];
        spine_data->tau_hip[ i + leg_0 ]  = data->tau_hip[ i + leg_0 ] * hip_side_sign[ i + leg_0 ];
        spine_data->tau_knee[ i + leg_0 ] = data->tau_knee[ i + leg_0 ] * knee_side_sign[ i + leg_0 ];
    }
}

/**
 * @brief Transfer command from spine to spi
 *
 * @param spine_cmd spine command
 * @param cmd spi command
 * @param leg_0 start leg id for two spines
 */
void SpiHandler::SpineCmdToSpiCmd( const SpineCommand* spine_cmd, spi_command_t* cmd, int leg_0 ) {
    auto& abad_side_sign = spi_params_.abad_side_sign;
    auto& hip_side_sign  = spi_params_.hip_side_sign;
    auto& knee_side_sign = spi_params_.knee_side_sign;
    auto& abad_offset    = spi_params_.abad_offset;
    auto& hip_offset     = spi_params_.hip_offset;
    auto& knee_offset    = spi_params_.knee_offset;
    for ( int i = 0; i < 2; i++ ) {
        cmd->q_des_abad[ i + leg_0 ]  = ( spine_cmd->q_des_abad[ i ] - abad_offset[ i + leg_0 ] ) * abad_side_sign[ i + leg_0 ];
        cmd->q_des_hip[ i + leg_0 ]   = ( spine_cmd->q_des_hip[ i ] - hip_offset[ i + leg_0 ] ) * hip_side_sign[ i + leg_0 ];
        cmd->q_des_knee[ i + leg_0 ]  = ( spine_cmd->q_des_knee[ i ] - knee_offset[ i + leg_0 ] ) * knee_side_sign[ i + leg_0 ];
        cmd->qd_des_abad[ i + leg_0 ] = spine_cmd->qd_des_abad[ i ] * abad_side_sign[ i + leg_0 ];
        cmd->qd_des_hip[ i + leg_0 ]  = spine_cmd->qd_des_hip[ i ] * hip_side_sign[ i + leg_0 ];
        cmd->qd_des_knee[ i + leg_0 ] = spine_cmd->qd_des_knee[ i ] * knee_side_sign[ i + leg_0 ];
        cmd->tau_abad_ff[ i + leg_0 ] = spine_cmd->tau_abad_ff[ i ] * abad_side_sign[ i + leg_0 ];
        cmd->tau_hip_ff[ i + leg_0 ]  = spine_cmd->tau_hip_ff[ i ] * hip_side_sign[ i + leg_0 ];
        cmd->tau_knee_ff[ i + leg_0 ] = spine_cmd->tau_knee_ff[ i ] * knee_side_sign[ i + leg_0 ];
    }
}

/**
 * @brief Communicate with spi device
 *
 */
void SpiHandler::DriverRunTest( uint16_t idSum ) {

    unsigned char spi_bits_per_word = 8;
    // transmit and receive buffers
    uint16_t tx_buf[ K_WORDS_PER_MESSAGE ];
    uint16_t rx_buf[ K_WORDS_PER_MESSAGE ];
    bool     communicateFlag[ 2 ] = { 1 };
    communicateFlag[ 0 ]          = ( idSum == 1 || idSum == 3 ) ? 1 : 0;
    communicateFlag[ 1 ]          = ( idSum == 2 || idSum == 3 ) ? 1 : 0;
    for ( int spi_board = 0; spi_board < 2; spi_board++ ) {
        if ( communicateFlag[ spi_board ] ) {
            // calculate checksum
            spine_command_[ spi_board ].checksum = CheckSum( ( uint32_t* )&spine_command_[ spi_board ], 32 );

            // pointers to command/data spine array
            uint16_t* cmd_d  = ( uint16_t* )&spine_command_[ spi_board ];
            uint16_t* data_d = ( uint16_t* )&spine_data_[ spi_board ];

            // zero rx buffer
            memset( rx_buf, 0, K_WORDS_PER_MESSAGE * sizeof( uint16_t ) );

            // copy into tx buffer flipping bytes
            for ( int i = 0; i < K_WORDS_PER_MESSAGE; i++ )
                tx_buf[ i ] = cmd_d[ i ];

            // each word is two bytes long
            size_t word_len = 2;  // 16 bit word

            // spi message struct
            struct spi_ioc_transfer spi_message[ 1 ];

            // zero message struct.
            memset( spi_message, 0, 1 * sizeof( struct spi_ioc_transfer ) );

            // set up message struct
            for ( int i = 0; i < 1; i++ ) {
                spi_message[ i ].bits_per_word = spi_bits_per_word;
                spi_message[ i ].cs_change     = 1;
                spi_message[ i ].delay_usecs   = 0;
                spi_message[ i ].len           = word_len * K_WORDS_PER_MESSAGE;
                spi_message[ i ].rx_buf        = ( uint64_t )rx_buf;
                spi_message[ i ].tx_buf        = ( uint64_t )tx_buf;
            }

            // do spi communication
            int rv = ioctl( spi_board == 0 ? spi_fd1_ : spi_fd2_, SPI_IOC_MESSAGE( 1 ), &spi_message );
            ( void )rv;

            for ( int i = 0; i < K_WORDS_PER_MESSAGE; i++ )
                data_d[ i ] = rx_buf[ i ];

            uint32_t calc_checksum = CheckSum( ( uint32_t* )&spine_data_[ spi_board ], 24 );
            ( void )calc_checksum;
            if ( calc_checksum != ( uint32_t )spine_data_[ spi_board ].checksum ) {
                printf( "[SpiHandler] Spi error, bad checksum, get: 0x%x expected: 0x%x\n", calc_checksum, spine_data_[ spi_board ].checksum );
            }
        }
    }
}

/**
 * @brief Input spine command
 *
 * @param cmd spine command
 * @param spine_board spine id
 */
void SpiHandler::SetSpineCommand( const SpineCommand& cmd, int spine_board ) {
    assert( spine_board == 0 || spine_board == 1 );
    memcpy( &spine_command_[ spine_board ], &cmd, sizeof( SpineCommand ) );
}

/**
 * @brief Output spine data
 *
 * @param data spine data
 * @param spine_board spine id
 */
void SpiHandler::GetSpineData( SpineDate& data, int spine_board ) {
    assert( spine_board == 0 || spine_board == 1 );
    memcpy( &data, &spine_data_[ spine_board ], sizeof( SpineDate ) );
}

/**
 * @brief Compute checksum of spi message
 *
 * @param data spi data or command
 * @param len data length(in 32-bit words)
 * @return uint32_t
 */
uint32_t SpiHandler::CheckSum( uint32_t* data, size_t len ) {
    uint32_t t = 0;
    for ( size_t i = 0; i < len; i++ )
        t = t ^ data[ i ];
    return t;
}