#include <fstream>
#include <iostream>

#include "offline_optimization_controller/offline_data_ctrl.hpp"

template < typename T > OfflineDataCtrl< T >::OfflineDataCtrl( DataReader* data_reader, float dt ) : DataReaderCtrl< T >( data_reader, dt ) {}

template < typename T > OfflineDataCtrl< T >::~OfflineDataCtrl() {}

template < typename T > void OfflineDataCtrl< T >::OneStep( float current_time, bool b_preparation, LegControllerCommand< T >* command ) {
    DataCtrl::state_machine_time_ = current_time - DataCtrl::ctrl_start_time_;

    DataCtrl::be_preparation_ = b_preparation;

    for ( int leg = 0; leg < 4; ++leg ) {
        for ( int jidx = 0; jidx < 3; ++jidx ) {
            command[ leg ].tau_feed_forward[ jidx ] = DataCtrl::desired_joint_torque_[ 3 * leg + jidx ];
            command[ leg ].q_des[ jidx ]            = DataCtrl::desired_joint_pos_[ 3 * leg + jidx ] + 0 * current_time;
            command[ leg ].qd_des[ jidx ]           = DataCtrl::desired_joint_vel_[ 3 * leg + jidx ];
            command[ leg ].kp_joint( jidx, jidx )   = DataCtrl::kp_joint_[ jidx ];
            command[ leg ].kd_joint( jidx, jidx )   = DataCtrl::kd_joint_[ jidx ];
        }
    }
}

/**
 * @brief get current iteration
 *
 * @return int
 */
template < typename T > int OfflineDataCtrl< T >::GetCurrentIteration() {
    return DataCtrl::current_iteration_;
}

/**
 * @brief get data end flag
 *
 * @return int
 */
template < typename T > int OfflineDataCtrl< T >::GetDataEndFlag() {
    return data_end_;
}

/**
 * @brief get joint position desire
 *
 * @param leg_num which leg is selected
 * @return Vec3< T >
 */
template < typename T > Vec3< T > OfflineDataCtrl< T >::GetJointPosDesire( int leg_num ) {
    return joint_pos_des_[ leg_num ];
}

/**
 * @brief get joint velocity desire
 *
 * @param leg_num which leg is selected
 * @return Vec3< T >
 */
template < typename T > Vec3< T > OfflineDataCtrl< T >::GetJointVelDesire( int leg_num ) {
    return joint_vel_des_[ leg_num ];
}

/**
 * @brief get joint feedforward torque
 *
 * @param leg_num which leg is selected
 * @return Vec3< T >
 */
template < typename T > Vec3< T > OfflineDataCtrl< T >::GetJointTorqueFeedforward( int leg_num ) {
    return joint_torque_feedforward_[ leg_num ];
}

/**
 * @brief get foot reaction force
 *
 * @param leg_num which leg is selected
 * @return Vec3< T >
 */
template < typename T > Vec3< T > OfflineDataCtrl< T >::GetReactionForceDesire( int leg_num ) {
    return reaction_force_des_[ leg_num ];
}

/**
 * @brief set body initial value
 *
 * @param body_pos_init body position initial value
 * @param body_rpy_init body posture initial value
 */
template < typename T > void OfflineDataCtrl< T >::SetBodyInit( Vec3< T > body_pos_init, Vec3< T > body_rpy_init ) {
    static RotMat< T > rot_matrix;

    body_pos_init_ = body_pos_init;
    body_rpy_init_ = body_rpy_init;
    rot_matrix << cos( body_rpy_init_( 2 ) ), -sin( body_rpy_init_( 2 ) ), 0.0, sin( body_rpy_init_( 2 ) ), cos( body_rpy_init_( 2 ) ), 0.0, 0.0, 0.0, 1.0;
    body_rot_matrix_init_ = rot_matrix.transpose();
}

/**
 * @brief Read offline data every time step
 *
 * @param current_time current time for offline data
 * @param b_preparation whether in preparation mode
 * @param offline_opt_data wbc data struct
 * @param pre_mode_duration duration for robot goes to initial state
 */
template < typename T > void OfflineDataCtrl< T >::OfflineDataCtrlRun( float current_time, bool b_preparation, OfflineOptCtrlData< T >* offline_opt_data, int pre_mode_duration ) {
    DataCtrl::state_machine_time_ = current_time - DataCtrl::ctrl_start_time_;

    DataCtrl::be_preparation_ = b_preparation;
    pre_mode_duration_        = pre_mode_duration;
    UpdateRobotRef();

    offline_opt_data->body_pos_des = body_pos_des_;
    offline_opt_data->body_vel_des = body_vel_des_;
    offline_opt_data->body_acc_des = body_acc_des_;

    offline_opt_data->body_rpy_des = body_rpy_des_;
    offline_opt_data->body_omg_des = body_omg_des_;

    for ( size_t i( 0 ); i < 4; ++i ) {
        offline_opt_data->foot_pos_des[ i ]       = foot_pos_des_[ i ];
        offline_opt_data->foot_vel_des[ i ]       = foot_vel_des_[ i ];
        offline_opt_data->foot_acc_des[ i ]       = foot_acc_des_[ i ];
        offline_opt_data->reaction_force_des[ i ] = reaction_force_des_[ i ];
    }
    offline_opt_data->contact_state = contact_state_;

    // no vel after data_end_
    if ( data_end_ ) {
        offline_opt_data->body_vel_des.setZero();
        offline_opt_data->body_acc_des.setZero();
        offline_opt_data->body_omg_des.setZero();
        for ( size_t i( 0 ); i < 4; ++i ) {
            offline_opt_data->foot_vel_des[ i ].setZero();
            offline_opt_data->foot_acc_des[ i ].setZero();
        }
    }
}

template < typename T > void OfflineDataCtrl< T >::UpdateRobotRef() {

    if ( DataCtrl::pre_mode_iteration_ < pre_mode_duration_ ) {
        if ( DataCtrl::pre_mode_iteration_ == 0 ) {
            printf( "[OfflineDataCtrl] data_time_steps_: %d \n", DataCtrl::data_reader_->GetDataTimeSteps() );
        }
        DataCtrl::pre_mode_iteration_ += DataCtrl::unit_time_step_;
        DataCtrl::current_iteration_ = 0;
    }

    float* data_current_step_ptr = DataCtrl::data_reader_->GetTrajectoryAtOnePoint( DataCtrl::current_iteration_ );

    body_pos_des_ << data_current_step_ptr[ 0 ], data_current_step_ptr[ 1 ], data_current_step_ptr[ 2 ];
    body_vel_des_ << data_current_step_ptr[ 3 ], data_current_step_ptr[ 4 ], data_current_step_ptr[ 5 ];
    body_acc_des_ << data_current_step_ptr[ 6 ], data_current_step_ptr[ 7 ], data_current_step_ptr[ 8 ];
    // robot frame to world frame
    body_pos_des_ = body_rot_matrix_init_.transpose() * body_pos_des_;
    body_vel_des_ = body_rot_matrix_init_.transpose() * body_vel_des_;
    body_acc_des_ = body_rot_matrix_init_.transpose() * body_acc_des_;
    // init pos
    body_pos_des_[ 0 ] = body_pos_des_[ 0 ] + body_pos_init_[ 0 ];
    body_pos_des_[ 1 ] = body_pos_des_[ 1 ] + body_pos_init_[ 1 ];

    body_rpy_des_ << data_current_step_ptr[ 9 ], data_current_step_ptr[ 10 ], data_current_step_ptr[ 11 ] + body_rpy_init_[ 2 ];
    body_omg_des_ << data_current_step_ptr[ 12 ], data_current_step_ptr[ 13 ], data_current_step_ptr[ 14 ];

    for ( int i = 0; i < 4; i++ ) {
        // WBC data
        foot_pos_des_[ i ] << data_current_step_ptr[ 3 * i + 15 ], data_current_step_ptr[ 3 * i + 16 ], data_current_step_ptr[ 3 * i + 17 ];
        foot_vel_des_[ i ] << data_current_step_ptr[ 3 * i + 27 ], data_current_step_ptr[ 3 * i + 28 ], data_current_step_ptr[ 3 * i + 29 ];
        foot_acc_des_[ i ] << data_current_step_ptr[ 3 * i + 39 ], data_current_step_ptr[ 3 * i + 40 ], data_current_step_ptr[ 3 * i + 41 ];
        reaction_force_des_[ i ] << data_current_step_ptr[ 3 * i + 51 ], data_current_step_ptr[ 3 * i + 52 ], data_current_step_ptr[ 3 * i + 53 ];
        foot_pos_des_[ i ]       = body_rot_matrix_init_.transpose() * foot_pos_des_[ i ];
        foot_vel_des_[ i ]       = body_rot_matrix_init_.transpose() * foot_vel_des_[ i ];
        foot_acc_des_[ i ]       = body_rot_matrix_init_.transpose() * foot_acc_des_[ i ];
        reaction_force_des_[ i ] = -body_rot_matrix_init_.transpose() * reaction_force_des_[ i ];
        foot_pos_des_[ i ][ 0 ]  = foot_pos_des_[ i ][ 0 ] + body_pos_init_[ 0 ];
        foot_pos_des_[ i ][ 1 ]  = foot_pos_des_[ i ][ 1 ] + body_pos_init_[ 1 ];
        contact_state_[ i ]      = data_current_step_ptr[ i + 63 ];

        // joint data
        joint_pos_des_[ i ] << data_current_step_ptr[ 3 * i + 67 ], data_current_step_ptr[ 3 * i + 68 ], data_current_step_ptr[ 3 * i + 69 ];
        joint_vel_des_[ i ] << data_current_step_ptr[ 3 * i + 79 ], data_current_step_ptr[ 3 * i + 80 ], data_current_step_ptr[ 3 * i + 81 ];
        joint_torque_feedforward_[ i ] << data_current_step_ptr[ 3 * i + 91 ], data_current_step_ptr[ 3 * i + 92 ], data_current_step_ptr[ 3 * i + 93 ];
    }

    // Go to initial pos and rpy
    if ( DataCtrl::current_iteration_ == 0 && pre_mode_duration_ > 1 ) {

        body_pos_des_ =
            body_pos_init_ + ( body_pos_des_ - body_pos_init_ ) * ( DataCtrl::pre_mode_iteration_ > 0.8 * pre_mode_duration_ ? 1 : DataCtrl::pre_mode_iteration_ / ( 0.8 * pre_mode_duration_ ) );
        body_vel_des_.setZero();
        body_acc_des_.setZero();
        body_rpy_des_ =
            body_rpy_init_ + ( body_rpy_des_ - body_rpy_init_ ) * ( DataCtrl::pre_mode_iteration_ > 0.8 * pre_mode_duration_ ? 1 : DataCtrl::pre_mode_iteration_ / ( 0.8 * pre_mode_duration_ ) );
        body_omg_des_.setZero();

        for ( size_t i( 0 ); i < 4; ++i ) {
            foot_pos_des_[ i ].setZero();
            foot_vel_des_[ i ].setZero();
            foot_acc_des_[ i ].setZero();
            reaction_force_des_[ i ].setZero();
            // reaction_force_des_[ i ][ 2 ]   = 16.0 * 9.81 * 0.25;
            contact_state_[ i ] = 0.5;
        }
    }

    // Time based
    DataCtrl::current_iteration_ += DataCtrl::unit_time_step_;
    // Update rate 0.5kHz
    if ( DataCtrl::current_iteration_ < DataCtrl::data_reader_->GetDataTimeSteps() - 1 ) {
        data_end_ = false;
    }
    else {
        data_end_ = true;
    }
}

template class OfflineDataCtrl< double >;
template class OfflineDataCtrl< float >;
