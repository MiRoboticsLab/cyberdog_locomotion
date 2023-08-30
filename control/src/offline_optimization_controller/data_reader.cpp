#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "Configuration.h"
#include "control_flags.hpp"
#include "offline_optimization_controller/data_reader.hpp"

/**
 * @brief Construct a new Data Reader:: Data Reader object
 *
 * @param type robot type, cyberdog or cyberdog2
 * @param fsm_state_name
 * @param gait_id
 */
DataReader::DataReader( const RobotType& type, FsmStateName fsm_state_name, const int gait_id ) : robot_type_( type ) {
    if ( robot_type_ == RobotType::CYBERDOG || robot_type_ == RobotType::CYBERDOG2 ) {
        if ( fsm_state_name == FsmStateName::kJump3d ) {
            data_cols_ = 103;
            switch ( gait_id ) {
            case JumpId::kJumpPosYaw90:
                LoadOptimalizedTrajectory( THIS_COM "control/offline_trajectory/jump3d/SRBM_four_leg_jump_yaw_+90deg.dat" );
                break;
            case JumpId::kJumpPosX60:
                LoadOptimalizedTrajectory( THIS_COM "control/offline_trajectory/jump3d/Five-link_four_leg_jump_x_+60cm.dat" );
                break;
            case JumpId::kJumpNegYaw90:
                LoadOptimalizedTrajectory( THIS_COM "control/offline_trajectory/jump3d/SRBM_four_leg_jump_yaw_-90deg.dat" );
                break;
            case JumpId::kJumpPosZ30:
                LoadOptimalizedTrajectory( THIS_COM "control/offline_trajectory/jump3d/SRBM_four_leg_jump_z_+30cm.dat" );
                break;
            case JumpId::kJumpDownStair:
                LoadOptimalizedTrajectory( THIS_COM "control/offline_trajectory/jump3d/SRBM_jump_down_stair_x_+50cm.dat" );
                break;
            case JumpId::kJumpPosY20:
                LoadOptimalizedTrajectory( THIS_COM "control/offline_trajectory/jump3d/SRBM_four_leg_jump_y_+20cm.dat" );
                break;
            case JumpId::kJumpPosX30:
                LoadOptimalizedTrajectory( THIS_COM "control/offline_trajectory/jump3d/Five-link_four_leg_jump_x_+30cm.dat" );
                break;
            case JumpId::kJumpNegY20:
                LoadOptimalizedTrajectory( THIS_COM "control/offline_trajectory/jump3d/SRBM_four_leg_jump_y_-20cm.dat" );
                break;
            default:
                LoadOptimalizedTrajectory( THIS_COM "control/offline_trajectory/jump3d/SRBM_four_leg_jump_yaw_+90deg.dat" );
                break;
            }
            printf( "[DataReader] Setup jump for cyberdog\n" );
        }
    }

    printf( "[DataReader] Constructed.\n" );
}

/**
 * @brief Destroy the Data Reader:: Data Reader object
 *
 */
DataReader::~DataReader() {
    UnLoadOptimalizedTrajectory();
}

/**
 * @brief Load the input files
 *
 * @param filename file name and path
 */
void DataReader::LoadOptimalizedTrajectory( const char* filename ) {
    printf( "[DataReader] Loading offline trajectory %s...\n", filename );
    FILE* f = fopen( filename, "rb" );
    if ( !f ) {
        printf( "[DataReader] Error loading offline trajectory!\n" );
        return;
    }
    fseek( f, 0, SEEK_END );
    uint64_t file_size = ftell( f );
    fseek( f, 0, SEEK_SET );

    printf( "[DataReader] Allocating %ld bytes for offline trajectory\n", file_size );

    data_location_ = ( float* )malloc( file_size + 1 );

    if ( !data_location_ ) {
        printf( "[DataReader] malloc failed!\n" );
        return;
    }

    uint64_t read_success = fread( data_location_, file_size, 1, f );
    if ( !read_success ) {
        printf( "[DataReader] Error, fread failed!\n" );
    }

    if ( file_size % sizeof( float ) ) {
        printf( "[DataReader] Error, file size isn't divisible by size of "
                "float!\n" );
    }

    fclose( f );

    data_loaded_     = true;
    data_time_steps_ = file_size / ( sizeof( float ) * data_cols_ );
    printf( "[DataReader] Successed loading offline trajectory for %d time steps\n", data_time_steps_ );
}

/**
 * @brief Get initial offline trajectory
 *
 * @return float* return the result
 */
float* DataReader::GetInitialConfiguration() {
    if ( !data_loaded_ ) {
        printf( "[DataReader] Error, failed loading offline trajectory!\n" );
        return nullptr;
    }

    return data_location_;
}

/**
 * @brief Get the offline trajectory at one point
 *
 * @param time_step time step on planning horizon
 * @return float* return the result
 */
float* DataReader::GetTrajectoryAtOnePoint( int time_step ) {
    if ( !data_loaded_ ) {
        printf( "[DataReader] Error: GetTrajectoryAtOnePoint called without a plan!\n" );
        return nullptr;
    }

    // if ( time_step < 0 || time_step >= data_time_steps_ ) {
    //     time_step = data_time_steps_ - 1;
    // }

    if ( time_step < 0 ) {
        return data_location_;
    }
    if ( time_step >= data_time_steps_ ) {
        time_step = data_time_steps_ - 1;
    }

    return data_location_ + data_cols_ * time_step;
}

/**
 * @brief Unload offline trajectory, free the pointer and set default value for operation variables
 *
 */
void DataReader::UnLoadOptimalizedTrajectory() {
    free( data_location_ );
    data_time_steps_ = -1;
    data_loaded_     = false;
    printf( "[DataReader] Unloaded offline trajectory\n" );
}
