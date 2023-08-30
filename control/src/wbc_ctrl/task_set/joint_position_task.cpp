#include "wbc_ctrl/task_set/joint_position_task.hpp"
#include "dynamics/floating_base_model.hpp"
#include "dynamics/quadruped.hpp"
#include "math/orientation_tools.hpp"
#include "utilities/utilities_print.hpp"

/**
 * @brief Construct a new JPosTask< T >::JPosTask object
 *
 * @param robot floating dynamics model
 */
template < typename T > JPosTask< T >::JPosTask( const FloatingBaseModel< T >* robot ) : Task< T >( cyberdog2::kNumActJoint ), robot_sys_( robot ) {
    TK::Jt_ = DMat< T >::Zero( cyberdog2::kNumActJoint, cyberdog2::kDimConfig );
    ( TK::Jt_.block( 0, 6, cyberdog2::kNumActJoint, cyberdog2::kNumActJoint ) ).setIdentity();
    TK::JtDotQdot_ = DVec< T >::Zero( cyberdog2::kNumActJoint );

    kp = DVec< T >::Constant( cyberdog2::kNumActJoint, 50. );
    kd = DVec< T >::Constant( cyberdog2::kNumActJoint, 5. );
}

/**
 * @brief Destroy the JPosTask< T >::JPosTask object
 *
 */
template < typename T > JPosTask< T >::~JPosTask() {}

/**
 * @brief Get desired accelation by offset between desired and real state
 *
 * @param pos_des desired joint position
 * @param vel_des desired joint velocity
 * @param acc_des desired joint accelation
 * @return true
 * @return false
 */
template < typename T > bool JPosTask< T >::UpdateCommand( const void* pos_des, const DVec< T >& vel_des, const DVec< T >& acc_des ) {
    DVec< T >* pos_cmd = ( DVec< T >* )pos_des;

    for ( size_t i( 0 ); i < cyberdog2::kNumActJoint; ++i ) {
        TK::pos_err_[ i ] = ( *pos_cmd )[ i ] - robot_sys_->state_.q[ i ];
        TK::vel_des_[ i ] = vel_des[ i ];
        TK::acc_des_[ i ] = acc_des[ i ];

        TK::op_cmd_[ i ] = kp[ i ] * TK::pos_err_[ i ] + kd[ i ] * ( vel_des[ i ] - robot_sys_->state_.qd[ i ] ) + acc_des[ i ];
    }
    // PrettyPrint( acc_des, std::cout, "acc_des" );
    // PrettyPrint( op_cmd_, std::cout, "op cmd" );
    // PrettyPrint( *pos_cmd, std::cout, "pos cmd" );

    return true;
}

/**
 * @brief Get task jacobian matrix, is Jt_
 *
 * @return true
 * @return false
 */
template < typename T > bool JPosTask< T >::UpdateTaskJacobian() {
    return true;
}

/**
 * @brief Get task jacobian*qdot, is JtDotQdot_
 *
 * @return true
 * @return false
 */
template < typename T > bool JPosTask< T >::UpdateTaskJDotQdot() {
    return true;
}

template class JPosTask< double >;
template class JPosTask< float >;
