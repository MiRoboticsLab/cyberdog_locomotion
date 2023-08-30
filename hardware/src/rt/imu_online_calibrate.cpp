#include <cstring>
#include <memory>
#include <mutex>
#include <numeric>
#include <thread>
#include <unistd.h>

#include "control_flags.hpp"
#include "control_parameters/control_parameter_interface.hpp"
#include "header/lcm_type/motion_control_response_lcmt.hpp"
#include "rt/imu_online_calibrate.h"
#include "utilities/toolkit.hpp"

MessageHandler::MessageHandler()
    : handle_msg_lcm_( GetLcmUrl( 255 ) ), handle_msg_lcm_interface_( GetLcmUrlWithPort( 7668, 255 ) ), handle_msg_lcm_execution_( GetLcmUrlWithPort( 7670, 1 ) ), ring_buf_imu_( 10000 ) {
    std::cout << "MessageHandler() was invoked!" << std::endl;

    first_calibrate_down_ = false;
    in_run_bias_.setZero();
    curr_gyro_bias_.setZero();
    handle_msg_lcm_interface_.subscribe( "interface_response", &MessageHandler::HandleControlParameter, this );
    handle_msg_lcm_execution_.subscribe( "exec_response", &MessageHandler::HandleMotionControlResponse, this );

    imu_sub_ = handle_msg_lcm_.subscribe( "myIMU", &MessageHandler::HandleImuData, this );

    parameter_request_lcmt_.requestNumber = 1;
    handle_msg_thread_                    = std::thread( &MessageHandler::HandleMessageThread, this );
    handle_msg_thread_interface_          = std::thread( &MessageHandler::HandleMessageThreadInterface, this );
    handle_msg_thread_execution_          = std::thread( &MessageHandler::HandleMessageThreadExecution, this );
}

MessageHandler::~MessageHandler() {
    std::cout << "~MessageHandler() was invoked!" << std::endl;
}

void MessageHandler::HandleControlParameter( const lcm::ReceiveBuffer* received_buffer, const std::string& chan, const ::control_parameter_respones_lcmt* msg ) {
    ( void )received_buffer;
    ( void )chan;
    ( void )msg;

    std::cout << "[IMU_ONLINE_CALIBRATE]Receive imu data response,very good!"
              << "  And the value of parameter_request_lcmt_.requestNumber is: " << parameter_request_lcmt_.requestNumber << std::endl;

    parameter_request_lcmt_.requestNumber++;
}

void MessageHandler::HandleMotionControlResponse( const lcm::ReceiveBuffer* received_buffer, const std::string& chan, const motion_control_response_lcmt* msg ) {
    ( void )received_buffer;
    ( void )chan;

    control_mode_ = msg->pattern;
}

void MessageHandler::HandleImuData( const lcm::ReceiveBuffer* received_buffer, const std::string& chan, const microstrain_lcmt* msg ) {
    ( void )received_buffer;
    ( void )chan;
    Eigen::Vector3d               acc( msg->acc[ 0 ], msg->acc[ 1 ], msg->acc[ 2 ] );
    Eigen::Vector3d               omega( msg->omega[ 0 ], msg->omega[ 1 ], msg->omega[ 2 ] );
    Eigen::Matrix< double, 2, 3 > imu_data;
    imu_data << acc.transpose(), omega.transpose();

    {
        std::lock_guard< std::mutex > guard( ring_buf_mtx_ );
        ring_buf_imu_.PushBack( imu_data );
    }
}

bool MessageHandler::Run( const std::string& name ) {
    ( void )name;
    int last_mode = control_mode_;

    while ( true ) {
        if ( last_mode != ( int )control_mode_ ) {
            SwitchCalibMode( last_mode, ( int )control_mode_ );
        }

        if ( IsInCalibMode( control_mode_ ) ) {
            if ( ( !first_calibrate_down_ && ring_buf_imu_.GetCurrentSize() > kFirstCalibrateSampleTime_ ) || ( first_calibrate_down_ && ring_buf_imu_.IsFull() ) ) {
                // detect movement
                if ( IsDataNoMove() ) {
                    // we have static for a long time, calibrate gyro bias
                    in_run_bias_ += curr_gyro_bias_;

                    std::string name                    = "inrun_w_bias";
                    parameter_request_lcmt_.requestKind = ( int8_t )ControlParameterRequestKind::kSET_IMU_CALIBRATE_PARAM_BY_NAME;
                    strcpy( ( char* )parameter_request_lcmt_.name, name.c_str() );
                    memcpy( parameter_request_lcmt_.value, in_run_bias_.data(), sizeof( in_run_bias_ ) );
                    parameter_request_lcmt_.parameterKind = ( int8_t )ControlParameterValueKind::kVEC_X_DOUBLE;

                    std::cout << "[" << __FUNCTION__ << "] send control paramter set request of inrun_w_bias: " << in_run_bias_.transpose() << std::endl;
                    if ( !first_calibrate_down_ ) {
                        std::cout << "[" << __FUNCTION__ << "] finish the first calib, slow calib rate" << std::endl;
                        first_calibrate_down_ = true;
                    }

                    handle_msg_lcm_interface_.publish( "interface_request", &parameter_request_lcmt_ );
                }
            }
        }
        else {
            // sleep, sleep my sweety heart
            sleep( 1 );
        }
        sleep( 0.1 );
    }

    return true;
}

bool MessageHandler::HandleMessageThread() {
    while ( 1 ) {
        handle_msg_lcm_.handleTimeout( 1000 );
        usleep( 500 );
    }
    return true;
}

bool MessageHandler::HandleMessageThreadInterface() {
    while ( 1 ) {
        handle_msg_lcm_interface_.handleTimeout( 1000 );
        usleep( 500 );
    }
    return true;
}

bool MessageHandler::HandleMessageThreadExecution() {
    while ( 1 ) {
        handle_msg_lcm_execution_.handleTimeout( 1000 );
        usleep( 500 );
    }
    return true;
}

void MessageHandler::SwitchCalibMode( int& last_mode, int current_mode ) {
    bool last_calib_mode    = IsInCalibMode( last_mode );
    bool current_calib_mode = IsInCalibMode( current_mode );

    last_mode = current_mode;

    if ( !last_calib_mode && current_calib_mode ) {  // non-calib mode -> calib mode
        std::cout << "[MessageHandler::SwitchCalibMode] switch from non-calib to calib mode" << std::endl;
        if ( imu_sub_ == nullptr ) {
            imu_sub_ = handle_msg_lcm_.subscribe( IMU_CHANNEL_NAME, &MessageHandler::HandleImuData, this );
            std::cout << "[MessageHandler::SwitchCalibMode] subscribe IMU!" << std::endl;
        }
        first_calibrate_down_ = false;
    }
    else if ( last_calib_mode && !current_calib_mode ) {  // calib mode -> non-calib mode
        std::cout << "[MessageHandler::SwitchCalibMode] switch from calib to non-calib mode" << std::endl;
        if ( imu_sub_ != nullptr ) {
            handle_msg_lcm_.unsubscribe( imu_sub_ );
            imu_sub_ = nullptr;
        }
        {
            std::lock_guard< std::mutex > guard( ring_buf_mtx_ );
            ring_buf_imu_.Clear();
        }
    }
}

/**
 * @brief process all IMU data in the ring buffer and detect is
 * there any movement, if no movement detected return true, else false.
 *
 */
bool MessageHandler::IsDataNoMove() {
    imu_matrix mean, stddev;
    mean.setZero();
    stddev.setZero();
    {
        std::lock_guard< std::mutex > guard( ring_buf_mtx_ );
        int                           current_size = ring_buf_imu_.GetCurrentSize();
        mean                                       = std::accumulate( ring_buf_imu_.Begin(), ring_buf_imu_.End(), mean ) / current_size;
        stddev                                     = ( std::inner_product(
                       ring_buf_imu_.Begin(), ring_buf_imu_.End(), ring_buf_imu_.Begin(), stddev, []( imu_matrix const& x, imu_matrix const& y ) { return x + y; },
                       [ mean ]( imu_matrix const& x, imu_matrix const& y ) { return ( x - mean ).cwiseProduct( ( y - mean ) ); } )
                   / current_size )
                     .cwiseSqrt();
        // Clear the ring buffer
        ring_buf_imu_.Clear();
    }

    // check static
    Eigen::Vector3d acc_mean    = mean.block< 1, 3 >( 0, 0 ).transpose();
    Eigen::Vector3d gyro_mean   = mean.block< 1, 3 >( 1, 0 ).transpose();
    Eigen::Vector3d acc_stddev  = stddev.block< 1, 3 >( 0, 0 ).transpose();
    Eigen::Vector3d gyro_stddev = stddev.block< 1, 3 >( 1, 0 ).transpose();

    // check stddev for gyro and acc
    bool acc_stddev_small  = ( acc_stddev.array() < kStaticAccStddevLimit_ ).all();
    bool gyro_stddev_small = ( gyro_stddev.array() < kStaticGyroStddevLimit_ ).all();

    // check gyro bias
    bool gyro_bias_small  = ( gyro_mean.array() < kStaticGyroBiasLimit_ ).all();
    bool inrun_bias_small = ( ( in_run_bias_ + gyro_mean ).array() < kStaticAccGravityLimit_ ).all();

    // compare acc with gravity
    bool acc_gravity_close = ( ( acc_mean ).norm() - GRAVITY ) < kStaticAccGravityLimit_;

    // profile the static detection logic
    std::cout << "[MessageHandler::IsDataNoMove] acc_mean: " << acc_mean.transpose() << ", gyro_mean: " << gyro_mean.transpose() << ", acc_stddev: " << acc_stddev.transpose()
              << ", gyro_stddev: " << gyro_stddev.transpose() << "\n"
              << "acc_stddev_small: " << acc_stddev_small << ", gyro_stddev_small: " << gyro_stddev_small << ", gyro_bias_small: " << gyro_bias_small << ", inrun_bias_small: " << inrun_bias_small
              << ", acc_gravity_close: " << acc_gravity_close << std::endl;

    curr_gyro_bias_ = gyro_mean;

    return ( acc_stddev_small && gyro_stddev_small && gyro_bias_small && acc_gravity_close );
}
