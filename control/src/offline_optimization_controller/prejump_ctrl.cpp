#include "offline_optimization_controller/prejump_ctrl.hpp"

template < typename T > PreJumpCtrl< T >::PreJumpCtrl( DataReader* data_reader, float dt ) : DataReaderCtrl< T >( data_reader, dt ) {}

template < typename T > PreJumpCtrl< T >::~PreJumpCtrl() {}

template < typename T > void PreJumpCtrl< T >::OneStep( float current_time, bool b_preparation, LegControllerCommand< T >* command ) {
    DataCtrl::state_machine_time_ = current_time - DataCtrl::ctrl_start_time_;

    DataCtrl::be_preparation_ = b_preparation;
    UpdateJointCommand();

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

template < typename T > void PreJumpCtrl< T >::UpdateJointCommand() {

    int pre_mode_duration( 500 );
    int tuck_iteration( 560 );

    float tau_mult_rear  = 2.4;
    float tau_mult_front = 1.0;

    DataCtrl::desired_joint_pos_.setZero();
    DataCtrl::desired_joint_vel_.setZero();
    DataCtrl::desired_joint_torque_.setZero();

    DataCtrl::kp_joint_ = { 80.0, 80.0, 80.0 };
    DataCtrl::kd_joint_ = { 1.0, 1.0, 1.0 };

    // PRE JUMP PREPATATION - CROUCH (FOLLOWS PREMODE DURATION TIMESTEPS)
    if ( ( DataCtrl::pre_mode_iteration_ < pre_mode_duration ) || DataCtrl::be_preparation_ ) {
        // move to the initial configuration to prepare for FrontJumping
        if ( DataCtrl::pre_mode_iteration_ == 0 ) {
            printf( "[PreJumpCtrl] data_time_steps_: %d \n", DataCtrl::data_reader_->GetDataTimeSteps() );
        }
        // printf("pre_mode_iteration_: %d \n", pre_mode_iteration_);

        DataCtrl::pre_mode_iteration_ += DataCtrl::unit_time_step_;
        DataCtrl::current_iteration_ = 0;
        tau_mult_rear                = 0;
        tau_mult_front               = 0;
        DataCtrl::kp_joint_          = { 80.0, 80.0, 80.0 };
        DataCtrl::kd_joint_          = { 1., 1., 1. };
    }
    else {
        if ( this->data_reader_->robot_type_ == RobotType::CYBERDOG2 ) {
            tau_mult_rear  = 1.5;
            tau_mult_front = 1.2;
        }
        else {
            tau_mult_rear  = 1.8;
            tau_mult_front = 1.2;
        }
    }

    // OBTAIN TIMSTEP DATA FROM THE DATA FILE
    if ( DataCtrl::current_iteration_ > DataCtrl::data_reader_->GetDataTimeSteps() - 1 ) {
        DataCtrl::current_iteration_ = DataCtrl::data_reader_->GetDataTimeSteps() - 1;
    }

    // OBTAIN DATA FROM THE JUMP_DATA FILE GENERATED IN MATLAB
    float* data_current_step_ptr = DataCtrl::data_reader_->GetTrajectoryAtOnePoint( DataCtrl::current_iteration_ );
    //  printf("%.2f\t,%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",data_current_step_ptr[0],data_current_step_ptr[1],data_current_step_ptr[2],data_current_step_ptr[3],
    //          data_current_step_ptr[4],data_current_step_ptr[5],data_current_step_ptr[6] );
    float* tau = data_current_step_ptr + kJointTorqueOffset;

    // INITIALIZE JOINT PARAMETERS AND TORQUES
    Vec3< float > q_des_front;
    Vec3< float > q_des_rear;
    Vec3< float > qd_des_front;
    Vec3< float > qd_des_rear;
    Vec3< float > tau_front;
    Vec3< float > tau_rear;

    // SETTING THE JOINT POSITIONS AND VELOCITIES AND FEEDING FORWARD THE JOINT TORQUES obtained from the data file
    q_des_front << 0.0, data_current_step_ptr[ 3 ], data_current_step_ptr[ 4 ];
    q_des_rear << 0.0, data_current_step_ptr[ 5 ], data_current_step_ptr[ 6 ];
    qd_des_front << 0.0, data_current_step_ptr[ 10 ], data_current_step_ptr[ 11 ];
    qd_des_rear << 0.0, data_current_step_ptr[ 12 ], data_current_step_ptr[ 13 ];
    tau_front << 0.0, tau_mult_front * tau[ 0 ] / 2.0, tau_mult_front * tau[ 1 ] / 2.0;
    tau_rear << 0.0, tau_mult_rear * tau[ 2 ] / 2.0, tau_mult_rear * tau[ 3 ] / 2.0;

    // Limitation set so that the arms do not swing too far back and hit the legs
    if ( q_des_front[ 1 ] < -M_PI / 1.5 ) {
        q_des_front[ 1 ]  = -M_PI / 1.5;
        qd_des_front[ 1 ] = 0.;
        tau_front[ 1 ]    = 0.;
    }

    float s( 0. );
    if ( ( DataCtrl::pre_mode_iteration_ < pre_mode_duration ) || DataCtrl::be_preparation_ ) {
        float t = DataCtrl::pre_mode_iteration_ * 1.0;
        if ( DataCtrl::pre_mode_iteration_ == DataCtrl::unit_time_step_ )
            q_init_ = q_cur_;
        for ( int i = 0; i < 4; ++i ) {
            for ( int j = 0; j < 3; ++j ) {
                if ( i < 2 )
                    DataCtrl::desired_joint_pos_[ 3 * i + j ] = q_init_( i * 3 + j ) + ( q_des_front( j ) - q_init_( i * 3 + j ) ) * ( t > pre_mode_duration ? 1 : t / pre_mode_duration );
                else
                    DataCtrl::desired_joint_pos_[ 3 * i + j ] = q_init_( i * 3 + j ) + ( q_des_rear( j ) - q_init_( i * 3 + j ) ) * ( t > pre_mode_duration ? 1 : t / pre_mode_duration );
            }
        }
    }
    else if ( DataCtrl::current_iteration_ >= tuck_iteration ) {
        // q_des_front << 0.0, -2.0795, 2.1057;
        // q_des_rear << 0.0, -1.57, 1.57;
        qd_des_front << 0.0, 0.0, 0.0;
        qd_des_rear << 0.0, 0.0, 0.0;
        tau_rear << 0.0, 0.0, 0.0;
        tau_front << 0.0, 0.0, 0.0;
        DataCtrl::kp_joint_ = { 40.0, 40.0, 40.0 };
        DataCtrl::kd_joint_ = { 1., 1., 1. };
    }

    for ( int i = 0; i < 12; i += 3 ) {
        DataCtrl::desired_joint_pos_[ i ]    = 0.0;
        DataCtrl::desired_joint_vel_[ i ]    = 0.0;
        DataCtrl::desired_joint_torque_[ i ] = 0.0;
    }
    DataCtrl::desired_joint_pos_[ 0 ] = s * ( -0.2 );
    DataCtrl::desired_joint_pos_[ 3 ] = s * ( 0.2 );
    DataCtrl::desired_joint_pos_[ 6 ] = s * ( -0.2 );
    DataCtrl::desired_joint_pos_[ 9 ] = s * ( 0.2 );

    if ( DataCtrl::current_iteration_ >= tuck_iteration ) {
        DataCtrl::desired_joint_pos_[ 0 ] = q_des_front[ 0 ];
        DataCtrl::desired_joint_pos_[ 3 ] = -q_des_front[ 0 ];
        DataCtrl::desired_joint_pos_[ 6 ] = q_des_rear[ 0 ];
        DataCtrl::desired_joint_pos_[ 9 ] = -q_des_rear[ 0 ];
        ;
    }
    if ( DataCtrl::current_iteration_ > 0 ) {
        // Front Hip
        for ( int i = 1; i < 6; i += 3 ) {
            DataCtrl::desired_joint_pos_[ i ]    = q_des_front[ 1 ];
            DataCtrl::desired_joint_vel_[ i ]    = qd_des_front[ 1 ];
            DataCtrl::desired_joint_torque_[ i ] = tau_front[ 1 ];
        }

        // Front Knee
        for ( int i = 2; i < 6; i += 3 ) {
            DataCtrl::desired_joint_pos_[ i ]    = q_des_front[ 2 ];
            DataCtrl::desired_joint_vel_[ i ]    = qd_des_front[ 2 ];
            DataCtrl::desired_joint_torque_[ i ] = tau_front[ 2 ];
        }

        // Hind Hip
        for ( int i = 7; i < 12; i += 3 ) {
            DataCtrl::desired_joint_pos_[ i ]    = q_des_rear[ 1 ];
            DataCtrl::desired_joint_vel_[ i ]    = qd_des_rear[ 1 ];
            DataCtrl::desired_joint_torque_[ i ] = tau_rear[ 1 ];
        }

        // Hind Knee
        for ( int i = 8; i < 12; i += 3 ) {
            DataCtrl::desired_joint_pos_[ i ]    = q_des_rear[ 2 ];
            DataCtrl::desired_joint_vel_[ i ]    = qd_des_rear[ 2 ];
            DataCtrl::desired_joint_torque_[ i ] = tau_rear[ 2 ];
        }
    }

    // Update rate 0.5kHz
    DataCtrl::current_iteration_ += DataCtrl::unit_time_step_;
}

template class PreJumpCtrl< double >;
template class PreJumpCtrl< float >;
