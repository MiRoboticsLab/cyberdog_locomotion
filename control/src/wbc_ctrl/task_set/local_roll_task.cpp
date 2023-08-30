#include "wbc_ctrl/task_set/local_roll_task.hpp"
#include "dynamics/floating_base_model.hpp"
#include "dynamics/quadruped.hpp"
#include "math/orientation_tools.hpp"
#include "utilities/utilities_print.hpp"

/**
 * @brief Construct a new Local Roll Task<  T >:: Local Roll Task object
 *
 * @param robot floating dynamics model
 */
template < typename T > LocalRollTask< T >::LocalRollTask( const FloatingBaseModel< T >* robot ) : Task< T >( 1 ), robot_dynamics_( robot ) {
    TK::Jt_         = DMat< T >::Zero( TK::dim_task_, cyberdog2::kDimConfig );
    TK::Jt_( 0, 0 ) = 1.;
    TK::JtDotQdot_  = DVec< T >::Zero( TK::dim_task_ );

    kp = DVec< T >::Constant( TK::dim_task_, 350. );
    kd = DVec< T >::Constant( TK::dim_task_, 13. );
}

/**
 * @brief Destroy the Local Roll Task<  T >:: Local Roll Task object
 *
 */
template < typename T > LocalRollTask< T >::~LocalRollTask() {}

/**
 * @brief Get desired accelation by offset between desired and real state
 *
 * @param pos_des desired quaternion of the body
 * @param vel_des desired angular velocity
 * @param acc_des desired angular accelation
 * @return true
 * @return false
 */
template < typename T > bool LocalRollTask< T >::UpdateCommand( const void* pos_des, const DVec< T >& vel_des, const DVec< T >& acc_des ) {
    T*        roll_cmd = ( T* )pos_des;
    Vec3< T > rpy      = ori::QuatToRPY( robot_dynamics_->state_.body_orientation );

    // Rx
    TK::pos_err_[ 0 ] = ( *roll_cmd - rpy[ 0 ] );
    TK::vel_des_[ 0 ] = vel_des[ 0 ];
    TK::acc_des_[ 0 ] = acc_des[ 0 ];

    TK::op_cmd_[ 0 ] = kp[ 0 ] * ( *roll_cmd - rpy[ 0 ] ) + kd[ 0 ] * ( TK::vel_des_[ 0 ] - robot_dynamics_->state_.body_velocity[ 0 ] ) + TK::acc_des_[ 0 ];

    // printf("[Body Ori Roll Task]\n");
    // PrettyPrint( TK::pos_err_, std::cout, "pos_err_" );
    // PrettyPrint( rpy, std::cout, "rpy" );
    // PrettyPrint( robot_dynamics_->state_.body_velocity, std::cout, "body velocity" );
    // PrettyPrint( TK::op_cmd_, std::cout, "op cmd" );
    // PrettyPrint( acc_des, std::cout, "acc_des" );
    // printf( "\n" );
    // PrettyPrint( TK::Jt_, std::cout, "Jt" );

    return true;
}

/**
 * @brief Get task jacobian matrix, is Jt_
 *
 * @return true
 * @return false
 */
template < typename T > bool LocalRollTask< T >::UpdateTaskJacobian() {
    return true;
}

/**
 * @brief Get task jacobian*qdot, is JtDotQdot_
 *
 * @return true
 * @return false
 */
template < typename T > bool LocalRollTask< T >::UpdateTaskJDotQdot() {
    return true;
}

template class LocalRollTask< double >;
template class LocalRollTask< float >;
