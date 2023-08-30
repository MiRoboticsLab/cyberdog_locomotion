#include "wbc_ctrl/task_set/body_orientation_task.hpp"
#include "dynamics/floating_base_model.hpp"
#include "dynamics/quadruped.hpp"
#include "math/orientation_tools.hpp"
#include "utilities/utilities_print.hpp"

/**
 * @brief Construct a new Body Ori Task<  T >:: Body Ori Task object
 *
 * @param robot floating dynamics model
 */
template < typename T > BodyOriTask< T >::BodyOriTask( const FloatingBaseModel< T >* robot ) : Task< T >( 3 ), robot_dynamics_( robot ) {
    TK::Jt_ = DMat< T >::Zero( TK::dim_task_, cyberdog2::kDimConfig );
    TK::Jt_.block( 0, 0, 3, 3 ).setIdentity();
    TK::JtDotQdot_ = DVec< T >::Zero( TK::dim_task_ );

    kp = DVec< T >::Constant( TK::dim_task_, 100. );
    kd = DVec< T >::Constant( TK::dim_task_, 10. );
}

/**
 * @brief Destroy the Body Ori Task<  T >:: Body Ori Task object
 *
 */
template < typename T > BodyOriTask< T >::~BodyOriTask() {}

/**
 * @brief Get desired accelation by offset between desired and real state
 *
 * @param pos_des desired quaternion of the body
 * @param vel_des desired angular velocity
 * @param acc_des desired angular accelation
 * @return true
 * @return false
 */
template < typename T > bool BodyOriTask< T >::UpdateCommand( const void* pos_des, const DVec< T >& vel_des, const DVec< T >& acc_des ) {
    Quat< T >* ori_cmd  = ( Quat< T >* )pos_des;
    Quat< T >  link_ori = ( robot_dynamics_->state_.body_orientation );

    Quat< T > link_ori_inv;
    link_ori_inv[ 0 ] = link_ori[ 0 ];
    link_ori_inv[ 1 ] = -link_ori[ 1 ];
    link_ori_inv[ 2 ] = -link_ori[ 2 ];
    link_ori_inv[ 3 ] = -link_ori[ 3 ];

    // Explicit because operational space is in global frame
    Quat< T > ori_err = ori::QuatProduct( *ori_cmd, link_ori_inv );
    if ( ori_err[ 0 ] < 0. ) {
        ori_err *= ( -1. );
    }
    Vec3< T > ori_err_so3;
    ori::QuaternionToso3( ori_err, ori_err_so3 );
    SVec< T > curr_vel = robot_dynamics_->state_.body_velocity;

    // Configuration space: Local
    // Operational Space: Global
    Mat3< T > Rot     = ori::QuaternionToRotationMatrix( link_ori );
    Vec3< T > vel_err = Rot.transpose() * ( TK::vel_des_ - curr_vel.head( 3 ) );
    for ( int m = 0; m < 3; m++ ) {
        if ( fabs( vel_err[ m ] ) > 1000 ) {
            std::cout << "[BodyOriTask] TK::vel_des_  " << TK::vel_des_[ m ] << "\toverflow, edit to  ";
            vel_err[ m ] = 0;
            std::cout << vel_err[ m ] << std::endl;
        }
    }
    // Rx, Ry, Rz
    for ( int i( 0 ); i < 3; ++i ) {
        TK::pos_err_[ i ] = ori_err_so3[ i ];
        TK::vel_des_[ i ] = vel_des[ i ];
        TK::acc_des_[ i ] = acc_des[ i ];

        TK::op_cmd_[ i ] = kp[ i ] * ori_err_so3[ i ] + kd[ i ] * vel_err[ i ] + TK::acc_des_[ i ];
        if ( fabs( TK::op_cmd_[ i ] ) > 100 ) {
            std::cout << "-------------------big problem------------------------------" << std::endl;
            std::cout << "BodyOriTask acc error" << TK::op_cmd_[ i ] << std::endl;
            std::cout << "ori_err_so3[i]" << ori_err_so3[ i ] << std::endl;
            std::cout << "vel_err[i]" << vel_err[ i ] << std::endl;
            std::cout << "TK::acc_des_[i]" << TK::acc_des_[ i ] << std::endl;
        }
    }
    // printf("[Body Ori Task]\n");
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
template < typename T > bool BodyOriTask< T >::UpdateTaskJacobian() {
    Quat< T > quat              = robot_dynamics_->state_.body_orientation;
    Mat3< T > Rot               = ori::QuaternionToRotationMatrix( quat );
    TK::Jt_.block( 0, 0, 3, 3 ) = Rot.transpose();
    // PrettyPrint( Rot, std::cout, "Rot mat" );
    return true;
}

/**
 * @brief Get task jacobian*qdot, is JtDotQdot_
 *
 * @return true
 * @return false
 */
template < typename T > bool BodyOriTask< T >::UpdateTaskJDotQdot() {
    return true;
}

template class BodyOriTask< double >;
template class BodyOriTask< float >;
