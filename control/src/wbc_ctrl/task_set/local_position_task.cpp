#include "wbc_ctrl/task_set/local_position_task.hpp"
#include "dynamics/floating_base_model.hpp"
#include "dynamics/quadruped.hpp"
#include "math/orientation_tools.hpp"
#include "utilities/utilities_print.hpp"

/**
 * @brief Construct a new Local Pos Task<  T >:: Local Pos Task object
 *
 * @param robot floating dynamics model
 * @param link_index link index of the model
 * @param local_frame_idx represents the link does not link to the base
 */
template < typename T >
LocalPosTask< T >::LocalPosTask( const FloatingBaseModel< T >* robot, int link_index, int local_frame_idx )
    : Task< T >( 3 ), robot_dynamics_( robot ), _link_idx( link_index ), _local_frame_idx( local_frame_idx ) {
    TK::Jt_        = DMat< T >::Zero( TK::dim_task_, cyberdog2::kDimConfig );
    TK::JtDotQdot_ = DVec< T >::Zero( TK::dim_task_ );

    kp = DVec< T >::Constant( TK::dim_task_, 120. );
    kd = DVec< T >::Constant( TK::dim_task_, 5. );
}

/**
 * @brief Destroy the Local Pos Task<  T >:: Local Pos Task object
 *
 */
template < typename T > LocalPosTask< T >::~LocalPosTask() {}

/**
 * @brief Get desired accelation by offset between desired and real state
 *
 * @param pos_des desired position of the link on local frame
 * @param vel_des desired velocity on local frame
 * @param acc_des desired accelation on local frame
 * @return true
 * @return false
 */
template < typename T > bool LocalPosTask< T >::UpdateCommand( const void* pos_des, const DVec< T >& vel_des, const DVec< T >& acc_des ) {
    Vec3< T >* pos_cmd = ( Vec3< T >* )pos_des;
    Vec3< T >  link_pos, local_pos, local_vel;

    link_pos  = robot_dynamics_->pGC_[ _link_idx ];
    local_pos = robot_dynamics_->pGC_[ _local_frame_idx ];
    local_vel = robot_dynamics_->vGC_[ _local_frame_idx ];

    // X, Y, Z
    for ( size_t i( 0 ); i < TK::dim_task_; ++i ) {
        TK::pos_err_[ i ] = ( *pos_cmd )[ i ] - ( link_pos[ i ] - local_pos[ i ] );
        TK::vel_des_[ i ] = vel_des[ i ];
        TK::acc_des_[ i ] = acc_des[ i ];

        TK::op_cmd_[ i ] = kp[ i ] * TK::pos_err_[ i ] + kd[ i ] * ( TK::vel_des_[ i ] - ( robot_dynamics_->vGC_[ _link_idx ][ i ] - local_vel[ i ] ) ) + TK::acc_des_[ i ];
    }

    // printf("[Link Pos Task]\n");
    // PrettyPrint( acc_des, std::cout, "acc_des" );
    // PrettyPrint( TK::pos_err_, std::cout, "pos_err_" );
    // PrettyPrint( *pos_cmd, std::cout, "pos cmd" );
    // PrettyPrint( link_pos, std::cout, "link pos" );
    // PrettyPrint( local_pos, std::cout, "local pos" );
    // PrettyPrint( TK::op_cmd_, std::cout, "op cmd" );
    // TEST
    // TK::op_cmd_.setZero();
    // PrettyPrint( TK::Jt_, std::cout, "Jt" );

    return true;
}

/**
 * @brief Get task jacobian matrix, is Jt_
 *
 * @return true
 * @return false
 */
template < typename T > bool LocalPosTask< T >::UpdateTaskJacobian() {
    TK::Jt_ = robot_dynamics_->Jc_[ _link_idx ] - robot_dynamics_->Jc_[ _local_frame_idx ];
    // TEST
    // TK::Jt_.block(0,0, 3,3) = DMat<T>::Zero(3,3);
    return true;
}

/**
 * @brief Get task jacobian*qdot, is JtDotQdot_
 *
 * @return true
 * @return false
 */
template < typename T > bool LocalPosTask< T >::UpdateTaskJDotQdot() {
    TK::JtDotQdot_ = robot_dynamics_->Jcdqd_[ _link_idx ] - robot_dynamics_->Jcdqd_[ _local_frame_idx ];
    return true;
}

template class LocalPosTask< double >;
template class LocalPosTask< float >;
