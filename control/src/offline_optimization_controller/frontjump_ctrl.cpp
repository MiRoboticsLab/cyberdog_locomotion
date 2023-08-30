#include "offline_optimization_controller/frontjump_ctrl.hpp"
#include "utilities/utilities_print.hpp"

template < typename T > FrontJumpCtrl< T >::FrontJumpCtrl( DataReader* data_reader, float dt ) : DataReaderCtrl< T >( data_reader, dt ) {}

template < typename T > FrontJumpCtrl< T >::~FrontJumpCtrl() {}

template < typename T > void FrontJumpCtrl< T >::OneStep( float current_time, bool b_preparation, LegControllerCommand< T >* command ) {
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

template < typename T > void FrontJumpCtrl< T >::UpdateJointCommand() {

    int pre_mode_duration( 1100 );
    int rear_leg_iteration( 820 );   // Stage1 + Stage2 + 20iters
    int tuck_iteration( 1146 );      // Stage1 + Stage2 + Stage3 + Stage4 + 6iters
    int ramp_end_iteration( 1286 );  // Stage1 + Stage2 + Stage3 + Stage4 +6iters + designed landing pose
    int leg_recovery_end_iteration( 1386 );
    if ( this->data_reader_->robot_type_ == RobotType::CYBERDOG2 ) {
        rear_leg_iteration         = 851;
        tuck_iteration             = 850;
        ramp_end_iteration         = 980;
        leg_recovery_end_iteration = 1160;
    }

    float tau_mult_rear  = 2.4;
    float tau_mult_front = 1.0;

    DataCtrl::desired_joint_pos_.setZero();
    DataCtrl::desired_joint_vel_.setZero();
    DataCtrl::desired_joint_torque_.setZero();

    DataCtrl::kp_joint_ = { 80.0, 130.0, 80.0 };
    DataCtrl::kd_joint_ = { 1.0, 1.0, 1.0 };

    // PRE JUMP PREPATATION - CROUCH (FOLLOWS PREMODE DURATION TIMESTEPS)
    if ( ( DataCtrl::pre_mode_iteration_ < pre_mode_duration ) || DataCtrl::be_preparation_ ) {
        // move to the initial configuration to prepare for FrontJumping
        if ( DataCtrl::pre_mode_iteration_ == 0 ) {
            printf( "[FrontJumpCtrl] data_time_steps_: %d \n", DataCtrl::data_reader_->GetDataTimeSteps() );
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
        // tau_mult = 1.;
        // if (DataCtrl::current_iteration_ > 300)
        //     tau_mult_front = 0.2;
    }

    // OBTAIN TIMSTEP DATA FROM THE DATA FILE
    if ( DataCtrl::current_iteration_ > DataCtrl::data_reader_->GetDataTimeSteps() - 1 ) {  //执行完停留在最后时刻
        DataCtrl::current_iteration_ = DataCtrl::data_reader_->GetDataTimeSteps() - 1;
    }

    // OBTAIN DATA FROM THE JUMP_DATA FILE GENERATED IN MATLAB
    float* data_current_step_ptr = DataCtrl::data_reader_->GetTrajectoryAtOnePoint( DataCtrl::current_iteration_ );  //当前
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
    // PrettyPrint( tau_front, std::cout, "tau front" );
    // PrettyPrint( tau_rear, std::cout, "tau rear" );
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
    else if ( DataCtrl::current_iteration_ >= rear_leg_iteration && DataCtrl::current_iteration_ < tuck_iteration ) {
        q_des_rear << 0.0, -2.4, 2.3;
        qd_des_rear << 0.0, 0.0, 0.0;
        tau_rear << 0.0, 0.0, 0.0;
        // tau_front *= 1.25;
    }
    else if ( DataCtrl::current_iteration_ >= tuck_iteration ) {  // ramp to landing configuration
        // printf( "tuck_iteration\n" );
        qd_des_front << 0.0, 0.0, 0.0;
        qd_des_rear << 0.0, 0.0, 0.0;
        tau_front << 0.0, 0.0, 0.0;
        tau_rear << 0.0, 0.0, 0.0;
        DataCtrl::kp_joint_ = { 20.0, 30.0, 30.0 };
        DataCtrl::kd_joint_ = { 2.0, 2.0, 2.0 };
        // start to count at the begining of tuck iteration
        static int aftertuck_iteration = tuck_iteration;
        if ( DataCtrl::current_iteration_ >= tuck_iteration && DataCtrl::current_iteration_ < tuck_iteration + DataCtrl::unit_time_step_ )
            aftertuck_iteration = tuck_iteration;
        aftertuck_iteration += DataCtrl::unit_time_step_;
        s = ( float )( aftertuck_iteration - tuck_iteration ) / ( leg_recovery_end_iteration - tuck_iteration );

        if ( s > 1 ) {
            s = 1;
        }
        Vec3< float > q_des_front_0;
        Vec3< float > q_des_rear_0;
        Vec3< float > q_des_front_m;
        Vec3< float > q_des_rear_m;
        Vec3< float > q_des_front_push;
        Vec3< float > q_des_front_f;
        Vec3< float > q_des_rear_f;

        data_current_step_ptr = DataCtrl::data_reader_->GetTrajectoryAtOnePoint( tuck_iteration );
        q_des_front_0 << 0.0, data_current_step_ptr[ 3 ], data_current_step_ptr[ 4 ];
        q_des_rear_0 << 0.0, data_current_step_ptr[ 5 ], data_current_step_ptr[ 6 ];
        if ( this->data_reader_->robot_type_ == RobotType::CYBERDOG2 ) {
            q_des_front_m << 0.0, -1.4800, 2.1897;  // 0.0, -1.6457, 2.3566;//0.0, -1.5599, 2.2732;//0.0, -1.48, 2.1897;
            q_des_rear_m << 0.0, -2.53, 2.44;
        }
        else {
            q_des_front_m << 0.0, -2.6, 2.5;
            q_des_rear_m << 0.0, -2.6, 2.5;
        }

        // SET THE DESIRED JOINT STATES FOR THE TUCK ITERATION CONTROLLER (LANDING CONTROLLER)
        // data_current_step_ptr = DataCtrl::data_reader_->GetTrajectoryAtOnePoint(0);

        if ( this->data_reader_->robot_type_ == RobotType::CYBERDOG2 ) {
            q_des_front_f << 0.0, -1.7395, 2.4405;
            q_des_rear_f << 0.0, -1.7395, 2.4405;
            q_des_front_push << 0.0, -1.1502, 1.3120;  // 0.0, -1.3773, 1.2473;//0.0, -1.2167, 1.4509;
        }
        else {
            q_des_front_f << -0.4, -1.35, 2.2;
            q_des_rear_f << -0.4, -1.35, 2.2;
            q_des_front_push << -0.4, -1.35, 2.2;
        }

        // ramp for linear interpolation for the tuck iteration
        float stage1 = ( float )( ramp_end_iteration - tuck_iteration ) / ( leg_recovery_end_iteration - tuck_iteration );

        if ( s < stage1 ) {
            DataCtrl::kp_joint_      = { 20.0, 60.0, 60.0 };
            DataCtrl::kd_joint_      = { 2.0, 2.0, 2.0 };
            DataCtrl::kp_joint_[ 1 ] = 60 - s / stage1 * 30;
            DataCtrl::kp_joint_[ 2 ] = 60 - s / stage1 * 30;
            q_des_front              = q_des_front_m;  // (stage1 - s) / stage1 * q_des_front_0 + s / stage1 * q_des_front_m;
            q_des_rear               = q_des_rear_m;   // (stage1 - s) / stage1  * q_des_rear_0 + s / stage1 * q_des_rear_m;
        }
        else if ( s < 0.95 ) {
            DataCtrl::kp_joint_      = { 20.0, 30.0, 30.0 };
            DataCtrl::kd_joint_      = { 2.0, 2.0, 2.0 };
            DataCtrl::kp_joint_[ 1 ] = 30 + ( s - stage1 ) / ( 0.95 - stage1 ) * 60;
            DataCtrl::kp_joint_[ 2 ] = 30 + ( s - stage1 ) / ( 0.95 - stage1 ) * 60;
            q_des_front              = q_des_front_push;
            q_des_rear               = q_des_rear_m;
        }
        else {
            q_des_front = q_des_front_f;  // (s - stage1) /(1 -  stage1)* q_des_front_0 + (1 - s) / (1 - stage1) * q_des_front_m;
            q_des_rear  = q_des_rear_f;   // (s - stage1) /(1 -  stage1)* q_des_rear_0 + (1 - s) / (1 - stage1) * q_des_rear_m;
        }
    }

    // Abduction
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

template class FrontJumpCtrl< double >;
template class FrontJumpCtrl< float >;
