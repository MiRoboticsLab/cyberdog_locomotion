#include "wbc_ctrl/task_set/body_posture_task.hpp"
#include "dynamics/floating_base_model.hpp"
#include "dynamics/quadruped.hpp"
#include "math/orientation_tools.hpp"
#include "utilities/utilities_print.hpp"

/**
 * @brief Construct a new Body Posture Task<  T >:: Body Posture Task object
 *
 * @param robot floating dynamics model
 */
template < typename T > BodyPostureTask< T >::BodyPostureTask( const FloatingBaseModel< T >* robot ) : Task< T >( 6 ), robot_dynamics_( robot ) {
    TK::Jt_ = DMat< T >::Zero( TK::dim_task_, cyberdog2::kDimConfig );
    TK::Jt_.block( 0, 0, 6, 6 ).setIdentity();
    TK::JtDotQdot_ = DVec< T >::Zero( TK::dim_task_ );

    kp = DVec< T >::Constant( TK::dim_task_, 50. );
    kd = DVec< T >::Constant( TK::dim_task_, 1. );
}

/**
 * @brief Destroy the Body Posture Task<  T >:: Body Posture Task object
 *
 */
template < typename T > BodyPostureTask< T >::~BodyPostureTask() {}

/**
 * @brief Get desired accelation by offset between desired and real state
 *
 * @param pos_des desired quaternion and position
 * @param vel_des desired angular and linear velocity
 * @param acc_des desired angular and linear elocity
 * @return true
 * @return false
 */
template < typename T > bool BodyPostureTask< T >::UpdateCommand( const void* pos_des, const DVec< T >& vel_des, const DVec< T >& acc_des ) {
    DVec< T >* pos_cmd = ( DVec< T >* )pos_des;

    // Orientation (w, x, y, z)
    Quat< T > ori_cmd;
    for ( size_t i( 0 ); i < 4; ++i )
        ori_cmd[ i ] = ( *pos_cmd )[ i ];
    Quat< T > link_ori = ( robot_dynamics_->state_.body_orientation );
    Quat< T > link_ori_inv;
    link_ori_inv[ 0 ] = link_ori[ 0 ];
    link_ori_inv[ 1 ] = -link_ori[ 1 ];
    link_ori_inv[ 2 ] = -link_ori[ 2 ];
    link_ori_inv[ 3 ] = -link_ori[ 3 ];

    Quat< T > ori_err = ori::QuatProduct( ori_cmd, link_ori_inv );
    if ( ori_err[ 0 ] < 0. ) {
        ori_err *= ( -1. );
    }
    Vec3< T > ori_err_so3;
    ori::QuaternionToso3( ori_err, ori_err_so3 );

    // Velocity
    SVec< T > curr_vel = robot_dynamics_->state_.body_velocity;
    Mat3< T > Rot      = ori::QuaternionToRotationMatrix( link_ori );
    curr_vel.tail( 3 ) = Rot.transpose() * curr_vel.tail( 3 );

    // Rx, Ry, Rz
    for ( int i( 0 ); i < 3; ++i ) {
        TK::pos_err_[ i ] = ori_err_so3[ i ];
        TK::vel_des_[ i ] = vel_des[ i ];
        TK::acc_des_[ i ] = acc_des[ i ];

        TK::op_cmd_[ i ] = kp[ i ] * ori_err_so3[ i ] + kd[ i ] * ( TK::vel_des_[ i ] - curr_vel[ i ] ) + TK::acc_des_[ i ];
    }

    // Position
    Vec3< T > link_pos = robot_dynamics_->state_.body_position;

    // X, Y, Z
    for ( int i( 3 ); i < 6; ++i ) {
        TK::pos_err_[ i ] = ( *pos_cmd )[ i + 1 ] - link_pos[ i - 3 ];
        TK::vel_des_[ i ] = vel_des[ i ];
        TK::acc_des_[ i ] = acc_des[ i ];

        TK::op_cmd_[ i ] = kp[ i ] * TK::pos_err_[ i ] + kd[ i ] * ( TK::vel_des_[ i ] - curr_vel[ i ] ) + TK::acc_des_[ i ];
    }

    // printf("[Body Posture Task]\n");
    // PrettyPrint( acc_des, std::cout, "acc_des" );
    // PrettyPrint( TK::pos_err_, std::cout, "pos_err" );
    // PrettyPrint( *pos_cmd, std::cout, "pos cmd" );
    // PrettyPrint( TK::op_cmd_, std::cout, "op cmd" );
    // PrettyPrint( TK::Jt_, std::cout, "Jt" );

    return true;
}

/**
 * @brief Get task jacobian matrix, is Jt_
 *
 * @return true
 * @return false
 */
template < typename T > bool BodyPostureTask< T >::UpdateTaskJacobian() {
    Quat< T > quat = robot_dynamics_->state_.body_orientation;
    Mat3< T > Rot  = ori::QuaternionToRotationMatrix( quat );

    TK::Jt_.block( 0, 0, 3, 3 ) = Rot.transpose();
    TK::Jt_.block( 3, 3, 3, 3 ) = Rot.transpose();
    return true;
}

/**
 * @brief Get task jacobian*qdot, is JtDotQdot_
 *
 * @return true
 * @return false
 */
template < typename T > bool BodyPostureTask< T >::UpdateTaskJDotQdot() {
    return true;
}

template class BodyPostureTask< double >;
template class BodyPostureTask< float >;
