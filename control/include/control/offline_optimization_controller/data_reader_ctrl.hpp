#ifndef DATA_READER_CTRL_HPP_
#define DATA_READER_CTRL_HPP_

#include "controllers/leg_controller.hpp"
#include "dynamics/floating_base_model.hpp"
#include "data_reader.hpp"

#define DataCtrl DataReaderCtrl< T >
#define DatasCtrl DataReadersCtrl< T >

/**
 * @brief Parent class of data-readers controller that contains data struct and operation function
 * can read one file
 *
 */
template < typename T > class DataReaderCtrl {
public:
    DataReaderCtrl( DataReader* data_reader, float dt ) {
        data_reader_    = data_reader;
        dt_             = dt;
        unit_time_step_ = ceil( dt_ * 1000 );

        printf( "dt: %f, step:%d\n", dt_, unit_time_step_ );
        kp_cartesian_.resize( 12 );
        kd_cartesian_.resize( 12 );
        desired_joint_pos_.resize( 12 );
        desired_joint_vel_.resize( 12 );
        desired_joint_torque_.resize( 12 );
        kp_joint_.resize( 3 );
        kd_joint_.resize( 3 );
    }

    ~DataReaderCtrl(){};

    virtual void OneStep( float current_time, bool b_preparation, LegControllerCommand< T >* command ) = 0;

    void FirstVisit( float current_time ) {
        ctrl_start_time_    = current_time;
        current_iteration_  = 0;
        pre_mode_iteration_ = 0;
    }

    void LastVisit() {}

    bool EndOfPhase( LegControllerData< T >* data ) {
        if ( state_machine_time_ > ( state_machine_end_time_ - 2. * dt_ ) ) {
            return true;
        }
        for ( int leg( 0 ); leg < 4; ++leg ) {
            if ( state_machine_time_ > 2.7 && data[ leg ].q[ 1 ] > max_knee_joint_pos_ && data[ leg ].qd[ 1 ] > max_knee_joint_vel_ ) {
                printf( "Contact detected at leg [%d] => Switch to the landing phase !!! \n", leg );
                printf( "state_machine_time: %lf \n", state_machine_time_ );
                printf( "Q-Knee: %lf \n", data[ leg ].q[ 1 ] );
                printf( "Qdot-Knee: %lf \n", data[ leg ].qd[ 1 ] );
                return true;
            }
        }
        return false;
    }

    void SetParameter() {
        for ( int i = 0; i < 12; i++ ) {
            kp_cartesian_[ i ] = 1000;
            kd_cartesian_[ i ] = 5.;
        }
        kp_joint_ = { 10.0, 10.0, 10.0 };
        kd_joint_ = { 1.0, 1.0, 1.0 };
    }

protected:
    DataReader* data_reader_;

    DVec< T > kp_cartesian_, kd_cartesian_;
    DVec< T > desired_joint_pos_;
    DVec< T > desired_joint_vel_;
    DVec< T > desired_joint_torque_;

    T dt_;

    std::vector< T > kp_joint_, kd_joint_;

    bool be_preparation_ = false;

    T state_machine_end_time_ = 5.5;

    T ctrl_start_time_;
    T max_knee_joint_pos_ = 2.0;
    T max_knee_joint_vel_ = 2.0;

    T state_machine_time_;

    int unit_time_step_ = 1;
    int current_iteration_, pre_mode_iteration_;
};

/**
 * @brief Parent class of data-readers controller that contains data struct and operation function
 * can read multiple files
 *
 */
template < typename T > class DataReadersCtrl {
public:
    DataReadersCtrl( std::vector< DataReader* > data_readers, float dt ) {
        data_readers_   = data_readers;
        dt_             = dt;
        unit_time_step_ = ceil( dt_ * 1000 );

        printf( "dt: %f, step:%d\n", dt_, unit_time_step_ );
        kp_cartesian_.resize( 12 );
        kd_cartesian_.resize( 12 );
        desired_joint_pos_.resize( 12 );
        desired_joint_vel_.resize( 12 );
        desired_joint_torque_.resize( 12 );
        kp_joint_.resize( 3 );
        kd_joint_.resize( 3 );
    }

    ~DataReadersCtrl(){};

    virtual void OneStep( float current_time, bool b_preparation, LegControllerCommand< T >* command ) = 0;

    void FirstVisit( float current_time ) {
        ctrl_start_time_    = current_time;
        current_iteration_  = 0;
        pre_mode_iteration_ = 0;
    }

    void LastVisit() {}

    bool EndOfPhase( LegControllerData< T >* data ) {
        if ( state_machine_time_ > ( state_machine_end_time_ - 2. * dt_ ) ) {
            return true;
        }
        for ( int leg( 0 ); leg < 4; ++leg ) {
            if ( state_machine_time_ > 2.7 && data[ leg ].q[ 1 ] > max_knee_joint_pos_ && data[ leg ].qd[ 1 ] > max_knee_joint_vel_ ) {
                printf( "Contact detected at leg [%d] => Switch to the landing phase !!! \n", leg );
                printf( "state_machine_time: %lf \n", state_machine_time_ );
                printf( "Q-Knee: %lf \n", data[ leg ].q[ 1 ] );
                printf( "Qdot-Knee: %lf \n", data[ leg ].qd[ 1 ] );
                return true;
            }
        }
        return false;
    }

    void SetParameter() {
        for ( int i = 0; i < 12; i++ ) {
            kp_cartesian_[ i ] = 1000;
            kd_cartesian_[ i ] = 5.;
        }
        kp_joint_ = { 10.0, 10.0, 10.0 };
        kd_joint_ = { 1.0, 1.0, 1.0 };
    }

protected:
    std::vector< DataReader* > data_readers_;

    DVec< T > kp_cartesian_, kd_cartesian_;
    DVec< T > desired_joint_pos_;
    DVec< T > desired_joint_vel_;
    DVec< T > desired_joint_torque_;

    T dt_;

    std::vector< T > kp_joint_, kd_joint_;

    bool be_preparation_ = false;

    bool _b_set_height_target;
    T    state_machine_end_time_ = 5.5;
    int  _dim_contact;

    T ctrl_start_time_;
    T max_knee_joint_pos_ = 2.0;
    T max_knee_joint_vel_ = 2.0;

    T state_machine_time_;

    int unit_time_step_ = 1;
    int current_iteration_, pre_mode_iteration_;
};
#endif  // DATA_READER_CTRL_HPP_
