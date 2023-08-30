#ifndef IMU_ONLINE_CALIBRATE_H_
#define IMU_ONLINE_CALIBRATE_H_

//#undef __ARM_NEON__
//#undef __ARM_NEON

#include <assert.h>
#include <iostream>
#include <mutex>
#include <stdlib.h>
#include <thread>

#include <Eigen/Dense>
#include <lcm/lcm-cpp.hpp>

#include "control_flags.hpp"
#include "header/lcm_type/control_parameter_request_lcmt.hpp"
#include "header/lcm_type/control_parameter_respones_lcmt.hpp"
#include "header/lcm_type/microstrain_lcmt.hpp"
#include "header/lcm_type/motion_control_response_lcmt.hpp"
#include "utilities/ring_queue.hpp"

#define IMU_CHANNEL_NAME "myIMU"
#define GRAVITY 9.81

// #define IMU_DEBUG_PRINTF 1

typedef Eigen::Matrix< double, 2, 3 > imu_matrix;

class MessageHandler {
public:
    MessageHandler();

    virtual ~MessageHandler();

    void HandleControlParameter( const lcm::ReceiveBuffer* received_buffer, const std::string& chan, const control_parameter_respones_lcmt* msg );

    void HandleMotionControlResponse( const lcm::ReceiveBuffer* received_buffer, const std::string& chan, const motion_control_response_lcmt* msg );

    bool Run( const std::string& name = "" );

    bool HandleMessageThread();
    bool HandleMessageThreadInterface();
    bool HandleMessageThreadExecution();

private:
    void HandleImuData( const lcm::ReceiveBuffer* received_buffer, const std::string& chan, const microstrain_lcmt* msg );

    void SwitchCalibMode( int& last_mode, int current_mode );

    bool IsInCalibMode( int mode ) {
        return mode == LCM_GAIT_PASSIVE || mode == LCM_GAIT_KNEEL;
    }

    bool IsDataNoMove();

    const int    kFirstCalibrateSampleTime_ = 3000;
    const double kStaticAccStddevLimit_     = 0.03;
    const double kStaticAccGravityLimit_    = 0.1;
    const double kStaticGyroStddevLimit_    = 0.0007;
    const double kStaticGyroBiasLimit_      = 0.015;

    lcm::LCM           handle_msg_lcm_;
    lcm::LCM           handle_msg_lcm_interface_;
    lcm::LCM           handle_msg_lcm_execution_;
    std::thread        handle_msg_thread_;
    std::thread        handle_msg_thread_interface_;
    std::thread        handle_msg_thread_execution_;
    lcm::Subscription* imu_sub_ = nullptr;
    int8_t             control_mode_;
    bool               first_calibrate_down_;
    Eigen::Vector3d    in_run_bias_;
    Eigen::Vector3d    curr_gyro_bias_;

    RingQueue< imu_matrix, Eigen::aligned_allocator< imu_matrix > > ring_buf_imu_;
    std::mutex                                                      ring_buf_mtx_;

    control_parameter_request_lcmt parameter_request_lcmt_;
};

#endif  // IMU_ONLINE_CALIBRATE_H_
