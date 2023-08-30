#include "estimator/orientation_estimator.hpp"

/**
 * @brief Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) by copying from cheater state data
 *
 */
template < typename T > void CheaterOrientationEstimator< T >::Run() {
    this->state_estimator_data_.result->orientation                    = this->state_estimator_data_.cheater_state->orientation.template cast< T >();
    this->state_estimator_data_.result->world2body_rotation_matrix     = ori::QuaternionToRotationMatrix( this->state_estimator_data_.result->orientation );
    this->state_estimator_data_.result->angular_velocity_in_body_frame = this->state_estimator_data_.cheater_state->angular_velocity_in_body_frame.template cast< T >();
    this->state_estimator_data_.result->angular_velocity_in_world_frame =
        this->state_estimator_data_.result->world2body_rotation_matrix.transpose() * this->state_estimator_data_.result->angular_velocity_in_body_frame;
    this->state_estimator_data_.result->rpy                        = ori::QuatToRPY( this->state_estimator_data_.result->orientation );
    this->state_estimator_data_.result->acceleration_in_body_frame = this->state_estimator_data_.cheater_state->acceleration.template cast< T >();
    this->state_estimator_data_.result->acceleration_in_world_frame =
        this->state_estimator_data_.result->world2body_rotation_matrix.transpose() * this->state_estimator_data_.result->acceleration_in_body_frame;
}

template < typename T > VectorNavOrientationEstimator< T >::VectorNavOrientationEstimator() {}

/**
 * @brief Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 *
 */
template < typename T > void VectorNavOrientationEstimator< T >::Run() {
    this->state_estimator_data_.result->orientation[ 0 ] = this->state_estimator_data_.vector_nav_data->quat[ 3 ];
    this->state_estimator_data_.result->orientation[ 1 ] = this->state_estimator_data_.vector_nav_data->quat[ 0 ];
    this->state_estimator_data_.result->orientation[ 2 ] = this->state_estimator_data_.vector_nav_data->quat[ 1 ];
    this->state_estimator_data_.result->orientation[ 3 ] = this->state_estimator_data_.vector_nav_data->quat[ 2 ];

    if ( first_visit_ ) {
        Vec3< T > rpy_ini = ori::QuatToRPY( this->state_estimator_data_.result->orientation );
        rpy_ini[ 0 ]      = 0;
        rpy_ini[ 1 ]      = 0;
        ori_ini_inv_      = RpyToQuat( -rpy_ini );
        ori_cali_         = Quat< T >::Zero();
        first_visit_      = false;
    }

    // for testing imu mounting offset
    // Vec3< T > rpy_offset(-0.035, -0.035, 0); // 0.035, 0.07
    // Quat< T > _ori_offset_inv      = RpyToQuat( -rpy_offset );
    // this->state_estimator_data_.result->orientation = ori::QuatProduct( this->state_estimator_data_.result->orientation, _ori_offset_inv );

    this->state_estimator_data_.result->orientation = ori::QuatProduct( ori_ini_inv_, this->state_estimator_data_.result->orientation );

    CaliOriFromVel();
    this->state_estimator_data_.result->orientation = ori::QuatProduct( this->state_estimator_data_.result->orientation, ori_cali_ );

    this->state_estimator_data_.result->rpy = ori::QuatToRPY( this->state_estimator_data_.result->orientation );

    this->state_estimator_data_.result->world2body_rotation_matrix = ori::QuaternionToRotationMatrix( this->state_estimator_data_.result->orientation );

    this->state_estimator_data_.result->angular_velocity_in_body_frame = this->state_estimator_data_.vector_nav_data->gyro.template cast< T >();

    this->state_estimator_data_.result->angular_velocity_in_world_frame =
        this->state_estimator_data_.result->world2body_rotation_matrix.transpose() * this->state_estimator_data_.result->angular_velocity_in_body_frame;

    // consider imu mounting position
    // Vec3< T > a = this->state_estimator_data_.vector_nav_data->accelerometer.template cast< T >();
    // Vec3< T > w = this->state_estimator_data_.result->angular_velocity_in_body_frame;
    // static Vec3< T > w_last = this->state_estimator_data_.result->angular_velocity_in_body_frame;
    // Vec3< T > w_dot = (w - w_last)/this->state_estimator_data_.parameters->controller_dt;;
    // w_last = w;
    // Vec3< T > r( -0.05, 0.02, 0 );
    // this->state_estimator_data_.result->acceleration_in_body_frame = a + w_dot.cross( r ) + w.cross( w.cross( r ) );

    this->state_estimator_data_.result->acceleration_in_body_frame = this->state_estimator_data_.vector_nav_data->accelerometer.template cast< T >();  // 0 0 9.81

    // for testing imu mounting offset
    // this->state_estimator_data_.result->acceleration_in_world_frame = ori::QuaternionToRotationMatrix(_ori_offset_inv)*this->state_estimator_data_.result->world2body_rotation_matrix.transpose() *
    // this->state_estimator_data_.result->acceleration_in_body_frame;

    this->state_estimator_data_.result->acceleration_in_world_frame =
        this->state_estimator_data_.result->world2body_rotation_matrix.transpose() * this->state_estimator_data_.result->acceleration_in_body_frame;

    static int times_show( 0 );
    times_show++;
    //   if(times_show%10==1)
    //   {
    //      printf("rpy:%.2f\t%.2f\t%.2f\n",this->state_estimator_data_.result->rpy(0),this->state_estimator_data_.result->rpy(1),
    //               this->state_estimator_data_.result->rpy(2));
    //       printf("body rpy rate:%.2f\t%.2f\t%.2f\n",this->state_estimator_data_.result->angular_velocity_in_body_frame(0),this->state_estimator_data_.result->angular_velocity_in_body_frame(1),
    //              this->state_estimator_data_.result->angular_velocity_in_body_frame(2));
    //       printf("body acc:%.2f\t%.2f\t%.2f\n",this->state_estimator_data_.result->acceleration_in_body_frame(0),this->state_estimator_data_.result->acceleration_in_body_frame(1),
    //              this->state_estimator_data_.result->acceleration_in_body_frame(2));
    //       printf("-------------------------------------------------\n");
    //   }
}

/**
 * @brief Add orientation offset according to velocity.
 *
 * IMU will be less accurate if the impact gets more drastic from faster running.
 */
template < typename T > void VectorNavOrientationEstimator< T >::CaliOriFromVel() {
    static float     vel_filter      = 0.99;
    static Vec3< T > body_vel_last   = Vec3< T >::Zero();
    static Vec3< T > body_vel_filter = Vec3< T >::Zero();
    static Vec3< T > rpy_cali        = Vec3< T >::Zero();
    static Vec3< T > ori_cali_gain   = Vec3< T >::Zero();
    static Vec3< T > ori_cali_offset = Vec3< T >::Zero();

    body_vel_last   = this->state_estimator_data_.result->velocity_in_body_frame;
    body_vel_filter = vel_filter * body_vel_last + ( 1.0 - vel_filter ) * this->state_estimator_data_.result->velocity_in_body_frame;
    ori_cali_gain   = *this->state_estimator_data_.ori_cali_gain;
    ori_cali_offset = *this->state_estimator_data_.ori_cali_offset;
    rpy_cali( 0 )   = ori_cali_offset( 0 );
    if ( body_vel_filter( 0 ) >= 0.0 ) {
        rpy_cali( 1 ) = ori_cali_gain( 1 ) * body_vel_filter( 0 ) + ori_cali_offset( 1 );  // pitch vx
    }
    else {
        rpy_cali( 1 ) = ori_cali_gain( 2 ) * body_vel_filter( 0 ) + ori_cali_offset( 1 );
    }

    ori_cali_ = RpyToQuat( rpy_cali );

    // // // DEBUG
    // std::cout << "body_vel_filter: " << body_vel_filter.transpose() << std::endl;
    // std::cout << "rpy_cali: " << rpy_cali.transpose() << std::endl;
    // std::cout << "ori_cali_gain: " << ori_cali_gain.transpose() << std::endl;
}

template class CheaterOrientationEstimator< float >;
template class CheaterOrientationEstimator< double >;

template class VectorNavOrientationEstimator< float >;
template class VectorNavOrientationEstimator< double >;
