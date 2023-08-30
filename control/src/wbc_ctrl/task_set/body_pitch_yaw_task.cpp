#include "wbc_ctrl/task_set/body_pitch_yaw_task.hpp"
#include "dynamics/floating_base_model.hpp"
#include "dynamics/quadruped.hpp"
#include "math/orientation_tools.hpp"
#include "utilities/utilities_print.hpp"

/**
 * @brief Construct a new Body Ry Rz Task<  T >:: Body Ry Rz Task object
 *
 * @param robot floating dynamics model
 */
template < typename T > BodyRyRzTask< T >::BodyRyRzTask( const FloatingBaseModel< T >* robot ) : Task< T >( 2 ), robot_dynamics_( robot ) {
    TK::Jt_ = DMat< T >::Zero( TK::dim_task_, cyberdog2::kDimConfig );
    TK::Jt_.block( 0, 1, 2, 2 ).setIdentity();
    TK::JtDotQdot_ = DVec< T >::Zero( TK::dim_task_ );

    kp = DVec< T >::Constant( TK::dim_task_, 50. );
    kd = DVec< T >::Constant( TK::dim_task_, 3. );
}

/**
 * @brief Destroy the Body Ry Rz Task<  T >:: Body Ry Rz Task object
 *
 */
template < typename T > BodyRyRzTask< T >::~BodyRyRzTask() {}

/**
 * @brief Get desired accelation by offset between desired and real state
 *
 * @param pos_des desired quaternion of the body
 * @param vel_des desired angular velocity, only contains pitch and yaw
 * @param acc_des desired angular accelation, only contains pitch and yaw
 * @return true
 * @return false
 */
template < typename T > bool BodyRyRzTask< T >::UpdateCommand( const void* pos_des, const DVec< T >& vel_des, const DVec< T >& acc_des ) {
    Quat< T >* ori_cmd  = ( Quat< T >* )pos_des;
    Quat< T >  link_ori = ( robot_dynamics_->state_.body_orientation );

    Quat< T > link_ori_inv;
    link_ori_inv[ 0 ] = link_ori[ 0 ];
    link_ori_inv[ 1 ] = -link_ori[ 1 ];
    link_ori_inv[ 2 ] = -link_ori[ 2 ];
    link_ori_inv[ 3 ] = -link_ori[ 3 ];

    // implicit error definition
    Quat< T > ori_err = ori::QuatProduct( link_ori_inv, *ori_cmd );
    if ( ori_err[ 0 ] < 0. ) {
        ori_err *= ( -1. );
    }
    Vec3< T > ori_err_so3;
    ori::QuaternionToso3( ori_err, ori_err_so3 );
    SVec< T > curr_vel = robot_dynamics_->state_.body_velocity;

    // Ry, Rz
    for ( int i( 0 ); i < 2; ++i ) {
        TK::pos_err_[ i ] = ori_err_so3[ i + 1 ];
        TK::vel_des_[ i ] = vel_des[ i ];
        TK::acc_des_[ i ] = acc_des[ i ];

        TK::op_cmd_[ i ] = kp[ i ] * ori_err_so3[ i + 1 ] + kd[ i ] * ( TK::vel_des_[ i ] - curr_vel[ i + 1 ] ) + TK::acc_des_[ i ];
    }
    // printf("[Body Ori Pitch Yaw Task]\n");
    // PrettyPrint( TK::pos_err_, std::cout, "pos_err_" );
    // PrettyPrint( *ori_cmd, std::cout, "des_ori" );
    // PrettyPrint( link_ori, std::cout, "curr_ori" );
    // PrettyPrint( ori_err, std::cout, "quat_err" );

    // PrettyPrint( link_ori_inv, std::cout, "ori_inv" );
    // PrettyPrint( ori_err, std::cout, "ori_err" );
    // PrettyPrint( *ori_cmd, std::cout, "cmd" );
    // PrettyPrint( acc_des, std::cout, "acc_des" );
    // PrettyPrint( TK::Jt_, std::cout, "Jt" );

    return true;
}

/**
 * @brief Get task jacobian matrix, is Jt_
 *
 * @return true
 * @return false
 */
template < typename T > bool BodyRyRzTask< T >::UpdateTaskJacobian() {
    return true;
}

/**
 * @brief Get task jacobian*qdot, is JtDotQdot_
 *
 * @return true
 * @return false
 */
template < typename T > bool BodyRyRzTask< T >::UpdateTaskJDotQdot() {
    return true;
}

template class BodyRyRzTask< double >;
template class BodyRyRzTask< float >;
